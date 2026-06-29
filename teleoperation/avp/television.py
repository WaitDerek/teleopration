from __future__ import annotations

import asyncio
from collections.abc import Mapping
import time
from multiprocessing import Array, Event, Process, Queue, Value, shared_memory
from typing import Optional
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

import numpy as np

from teleoperation.preprocessing.transforms import is_valid_transform
from teleoperation.types import StreamState


class OpenTeleVision:
    """Small Vuer-based AVP event server adapted from the TeleVision third-party code."""

    def __init__(
        self,
        image_shape: tuple[int, int] = (450, 1000),
        shm_name: Optional[str] = None,
        stream_mode: str = "image",
        cert_file: str = "./cert.pem",
        key_file: str = "./key.pem",
        ngrok: bool = False,
        public_host: Optional[str] = None,
        port: int = 8012,
        debug: bool = False,
        show_hands: bool = False,
        show_images: bool = True,
        image_opacity: float = 1.0,
        client_url: Optional[str] = None,
    ) -> None:
        self.image_shape = image_shape
        self.stream_mode = stream_mode
        self.cert_file = cert_file
        self.key_file = key_file
        self.ngrok = ngrok
        self.public_host = public_host
        self.port = port
        self.debug = debug
        self.show_hands = show_hands
        self.show_images = show_images
        self.image_opacity = max(0.0, min(float(image_opacity), 1.0))
        self.client_url = client_url.rstrip("/") if client_url else None
        self.launch_id = str(int(time.time()))
        self._owned_shm: Optional[shared_memory.SharedMemory] = None

        self.left_hand_shared = Array("d", 16, lock=True)
        self.right_hand_shared = Array("d", 16, lock=True)
        self.left_landmarks_shared = Array("d", 75, lock=True)
        self.right_landmarks_shared = Array("d", 75, lock=True)
        self.head_matrix_shared = Array("d", 16, lock=True)
        self.aspect_shared = Value("d", 1.0, lock=True)
        self.right_pinch_shared = Value("b", False, lock=True)
        self._last_event_time = Value("d", 0.0, lock=True)
        self._http_requests = Value("i", 0, lock=True)
        self._active_websockets = Value("i", 0, lock=True)
        self._websocket_connects = Value("i", 0, lock=True)
        self._camera_events = Value("i", 0, lock=True)
        self._hand_events = Value("i", 0, lock=True)
        self._valid_right_hand_events = Value("i", 0, lock=True)
        self._skipped_camera_events = Value("i", 0, lock=True)
        self._skipped_hand_events = Value("i", 0, lock=True)
        self._state = StreamState.DISCONNECTED

        self.shm_name = shm_name or "teleoperation_avp_image_stream"
        self._ensure_image_shm()
        self._process: Optional[Process] = None

    def _ensure_image_shm(self) -> None:
        if self.stream_mode != "image" or not self.show_images:
            return
        height, width = self.image_shape
        shape = (height, width * 2, 3)
        size = int(np.prod(shape) * np.dtype(np.uint8).itemsize)
        try:
            self._owned_shm = shared_memory.SharedMemory(name=self.shm_name, create=True, size=size)
        except FileExistsError:
            self._owned_shm = shared_memory.SharedMemory(name=self.shm_name)
        array = np.ndarray(shape, dtype=np.uint8, buffer=self._owned_shm.buf)
        array[:] = 0

    def start(self) -> None:
        if self._process and self._process.is_alive():
            return
        self._process = Process(target=self._run_server, daemon=True)
        self._process.start()
        self._state = StreamState.WAITING

    def browser_url(self) -> Optional[str]:
        if self.ngrok or not self.public_host:
            return None
        default_client_url = "https://vuer.ai"
        return self._url_with_query(
            self.client_url or default_client_url,
            {
                "ws": f"wss://{self.public_host}:{self.port}",
                "grid": "False",
                "teleop_run": self.launch_id,
            },
        )

    def stable_browser_url(self) -> Optional[str]:
        if self.ngrok or not self.public_host:
            return None
        return f"https://{self.public_host}:{self.port}/go"

    @staticmethod
    def _url_with_query(base_url: str, extra_query: dict[str, str]) -> str:
        parts = urlsplit(base_url)
        query = dict(parse_qsl(parts.query, keep_blank_values=True))
        query.update(extra_query)
        return urlunsplit((parts.scheme, parts.netloc, parts.path, urlencode(query), parts.fragment))

    def stop(self) -> None:
        if self._process and self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2.0)
        self._process = None
        if self._owned_shm is not None:
            self._owned_shm.close()
            try:
                self._owned_shm.unlink()
            except FileNotFoundError:
                pass
            self._owned_shm = None
        self._state = StreamState.DISCONNECTED

    def state(self, stale_after_sec: float = 0.5) -> StreamState:
        if self._state == StreamState.DISCONNECTED:
            return self._state
        last_event = self._last_event_time.value
        if last_event <= 0:
            return StreamState.WAITING
        if time.monotonic() - last_event > stale_after_sec:
            return StreamState.STALE
        return StreamState.STREAMING

    @property
    def right_hand(self) -> np.ndarray:
        return np.array(self.right_hand_shared[:], dtype=float).reshape(4, 4, order="F")

    @property
    def head_matrix(self) -> np.ndarray:
        return np.array(self.head_matrix_shared[:], dtype=float).reshape(4, 4, order="F")

    @property
    def right_pinch(self) -> bool:
        return bool(self.right_pinch_shared.value)

    def latest_right_hand_matrix(self) -> Optional[np.ndarray]:
        matrix = self.right_hand
        return matrix if is_valid_transform(matrix) else None

    def latest_head_matrix(self) -> Optional[np.ndarray]:
        matrix = self.head_matrix
        return matrix if is_valid_transform(matrix) else None

    @staticmethod
    def _increment(counter, amount: int = 1) -> None:
        with counter.get_lock():
            counter.value += amount

    def diagnostics(self) -> dict[str, int]:
        return {
            "http_requests": int(self._http_requests.value),
            "ws_active": int(self._active_websockets.value),
            "ws_connects": int(self._websocket_connects.value),
            "camera_events": int(self._camera_events.value),
            "hand_events": int(self._hand_events.value),
            "valid_right_hand_events": int(self._valid_right_hand_events.value),
            "skipped_camera_events": int(self._skipped_camera_events.value),
            "skipped_hand_events": int(self._skipped_hand_events.value),
        }

    def _run_server(self) -> None:
        try:
            from aiohttp import web
            from vuer import Vuer
            from vuer.base import handle_file_request
            from vuer.schemas import Hands, ImageBackground
        except ImportError as exc:
            raise RuntimeError("Install AVP dependencies with: python -m pip install -e '.[avp]'") from exc

        browser_url = self.browser_url()

        async def websocket_handler(request, handler, **ws_kwargs):
            ws = web.WebSocketResponse(**ws_kwargs)
            await ws.prepare(request)
            self._increment(self._active_websockets)
            self._increment(self._websocket_connects)
            try:
                await handler(request, ws)
            except (ConnectionResetError, asyncio.CancelledError):
                if self.debug:
                    print("WebSocket disconnected", flush=True)
            except Exception as exc:
                if self.debug:
                    print(f"WebSocket handler error: {exc}", flush=True)
                raise
            finally:
                with self._active_websockets.get_lock():
                    self._active_websockets.value = max(0, self._active_websockets.value - 1)
                await ws.close()
                if self.debug:
                    print("WebSocket connection closed", flush=True)
            return ws

        class TeleoperationVuer(Vuer):
            async def socket_index(vuer_self, request):
                self._increment(self._http_requests)
                upgrade = request.headers.get("Upgrade", "")
                is_websocket = upgrade.lower().strip() == "websocket"
                if self.debug:
                    peer = request.transport.get_extra_info("peername") if request.transport else None
                    print(
                        "Vuer request: "
                        f"peer={peer} path={request.rel_url} upgrade={upgrade!r} "
                        f"user_agent={request.headers.get('User-Agent', '')[:120]}",
                        flush=True,
                    )
                if request.path == "/go" and browser_url and not is_websocket:
                    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="0;url={browser_url}">
  <title>Open AVP Teleoperation</title>
</head>
<body style="font-family: -apple-system, BlinkMacSystemFont, sans-serif; font-size: 24px;">
  <p>Opening Vision Pro teleoperation...</p>
  <p><a href="{browser_url}">Open manually</a></p>
  <script>location.replace({browser_url!r});</script>
</body>
</html>"""
                    return web.Response(
                        text=html,
                        content_type="text/html",
                        headers={
                            "Cache-Control": "no-store, max-age=0",
                            "Pragma": "no-cache",
                        },
                    )
                if browser_url and "ws" not in request.query and not is_websocket:
                    if self.debug:
                        print(f"Redirecting Vision Pro browser to: {browser_url}", flush=True)
                    raise web.HTTPFound(
                        browser_url,
                        headers={
                            "Cache-Control": "no-store, max-age=0",
                            "Pragma": "no-cache",
                        },
                    )
                if is_websocket:
                    return await websocket_handler(
                        request,
                        vuer_self.downlink,
                        max_msg_size=vuer_self.WEBSOCKET_MAX_SIZE,
                    )
                return await handle_file_request(request, vuer_self.client_root, filename="index.html")

        if self.ngrok:
            app = TeleoperationVuer(host="0.0.0.0", port=self.port, queries={"grid": False}, queue_len=3)
        else:
            queries = {"grid": False}
            if self.public_host:
                queries["ws"] = f"wss://{self.public_host}:{self.port}"
            app = TeleoperationVuer(
                host="0.0.0.0",
                port=self.port,
                cert=self.cert_file,
                key=self.key_file,
                domain=self.client_url or "https://vuer.ai",
                queries=queries,
                queue_len=3,
            )

        @app.add_handler("HAND_MOVE")
        async def on_hand_move(event, session, fps=60):
            try:
                value = event.value
                if not isinstance(value, Mapping):
                    self._increment(self._skipped_hand_events)
                    if self.debug:
                        print(f"Skipping HAND_MOVE payload type={type(value).__name__}", flush=True)
                    return
                self._increment(self._hand_events)
                right_hand = value.get("rightHand")
                if isinstance(right_hand, list):
                    right_hand_values = right_hand[-16:]
                    self.right_hand_shared[:] = right_hand_values
                    right_hand_matrix = np.asarray(right_hand_values, dtype=float).reshape(4, 4, order="F")
                    if is_valid_transform(right_hand_matrix):
                        self._increment(self._valid_right_hand_events)
                right_state = value.get("rightState", {})
                if not isinstance(right_state, Mapping):
                    right_state = {}
                self.right_pinch_shared.value = bool(right_state.get("pinching", False))
                self._last_event_time.value = time.monotonic()
                if self.debug:
                    print(
                        "HAND_MOVE received: "
                        f"rightHand={isinstance(right_hand, list)} "
                        f"pinching={self.right_pinch_shared.value}",
                        flush=True,
                    )
            except Exception as exc:
                if self.debug:
                    print(f"Error handling HAND_MOVE: {exc}", flush=True)

        @app.add_handler("CAMERA_MOVE")
        async def on_cam_move(event, session, fps=60):
            try:
                value = event.value
                if not isinstance(value, Mapping):
                    self._increment(self._skipped_camera_events)
                    if self.debug:
                        print(f"Skipping CAMERA_MOVE payload type={type(value).__name__}", flush=True)
                    return
                self._increment(self._camera_events)
                camera = value.get("camera", {})
                if not isinstance(camera, Mapping):
                    camera = {}
                if isinstance(camera.get("matrix"), list):
                    self.head_matrix_shared[:] = camera["matrix"]
                if "aspect" in camera:
                    self.aspect_shared.value = float(camera["aspect"])
                if self.debug:
                    print(
                        "CAMERA_MOVE received: "
                        f"matrix={isinstance(camera.get('matrix'), list)} aspect={camera.get('aspect')}",
                        flush=True,
                    )
            except Exception:
                pass

        @app.spawn(start=False)
        async def main_image(session, fps=60):
            try:
                session.upsert @ Hands(
                    fps=fps,
                    stream=True,
                    key="hands",
                    showLeft=self.show_hands,
                    showRight=self.show_hands,
                    computeGestures=True,
                )
                if not self.show_images:
                    heartbeat_image = np.zeros((2, 2, 4), dtype=np.uint8)
                    while True:
                        session.upsert(
                            [
                                ImageBackground(
                                    heartbeat_image,
                                    format="png",
                                    key="left-heartbeat",
                                    transparent=True,
                                    opacity=0.0,
                                    interpolate=True,
                                    aspect=1.66667,
                                    height=8,
                                    position=[0, -1, 3],
                                    layers=1,
                                ),
                                ImageBackground(
                                    heartbeat_image,
                                    format="png",
                                    key="right-heartbeat",
                                    transparent=True,
                                    opacity=0.0,
                                    interpolate=True,
                                    aspect=1.66667,
                                    height=8,
                                    position=[0, -1, 3],
                                    layers=2,
                                ),
                            ],
                            to="bgChildren",
                        )
                        await asyncio.sleep(0.03)

                existing_shm = shared_memory.SharedMemory(name=self.shm_name)
                height, width = self.image_shape
                image_array = np.ndarray((height, width * 2, 3), dtype=np.uint8, buffer=existing_shm.buf)
                try:
                    while True:
                        session.upsert(
                            [
                                ImageBackground(
                                    image_array[::2, :width],
                                    format="jpeg",
                                    quality=80,
                                    key="left-image",
                                    interpolate=True,
                                    aspect=1.66667,
                                    height=8,
                                    position=[0, -1, 3],
                                    layers=1,
                                    transparent=self.image_opacity < 1.0,
                                    opacity=self.image_opacity,
                                ),
                                ImageBackground(
                                    image_array[::2, width:],
                                    format="jpeg",
                                    quality=80,
                                    key="right-image",
                                    interpolate=True,
                                    aspect=1.66667,
                                    height=8,
                                    position=[0, -1, 3],
                                    layers=2,
                                    transparent=self.image_opacity < 1.0,
                                    opacity=self.image_opacity,
                                ),
                            ],
                            to="bgChildren",
                        )
                        await asyncio.sleep(0.03)
                finally:
                    existing_shm.close()
            except AssertionError as exc:
                if "Websocket session is missing" not in str(exc):
                    raise
                if self.debug:
                    print("Vision Pro websocket session ended; stopping stream task.", flush=True)

        app._route("/go", app.socket_index, method="GET")
        app._route("/go/", app.socket_index, method="GET")
        app.run()
