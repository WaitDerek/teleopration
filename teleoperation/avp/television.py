from __future__ import annotations

import asyncio
import time
from multiprocessing import Array, Event, Process, Queue, Value, shared_memory
from typing import Optional

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
    ) -> None:
        self.image_shape = image_shape
        self.stream_mode = stream_mode
        self.cert_file = cert_file
        self.key_file = key_file
        self.ngrok = ngrok
        self._owned_shm: Optional[shared_memory.SharedMemory] = None

        self.left_hand_shared = Array("d", 16, lock=True)
        self.right_hand_shared = Array("d", 16, lock=True)
        self.left_landmarks_shared = Array("d", 75, lock=True)
        self.right_landmarks_shared = Array("d", 75, lock=True)
        self.head_matrix_shared = Array("d", 16, lock=True)
        self.aspect_shared = Value("d", 1.0, lock=True)
        self.right_pinch_shared = Value("b", False, lock=True)
        self._last_event_time = Value("d", 0.0, lock=True)
        self._state = StreamState.DISCONNECTED

        self.shm_name = shm_name or "teleoperation_avp_image_stream"
        self._ensure_image_shm()
        self._process: Optional[Process] = None

    def _ensure_image_shm(self) -> None:
        if self.stream_mode != "image":
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

    def _run_server(self) -> None:
        try:
            from vuer import Vuer
            from vuer.schemas import Hands, ImageBackground
        except ImportError as exc:
            raise RuntimeError("Install AVP dependencies with: python -m pip install -e '.[avp]'") from exc

        if self.ngrok:
            app = Vuer(host="0.0.0.0", queries={"grid": False}, queue_len=3)
        else:
            app = Vuer(host="0.0.0.0", cert=self.cert_file, key=self.key_file, queries={"grid": False}, queue_len=3)

        @app.add_handler("HAND_MOVE")
        async def on_hand_move(event, session, fps=60):
            try:
                if isinstance(event.value.get("rightHand"), list):
                    self.right_hand_shared[:] = event.value["rightHand"][-16:]
                right_state = event.value.get("rightState", {})
                self.right_pinch_shared.value = bool(right_state.get("pinching", False))
                self._last_event_time.value = time.monotonic()
            except Exception as exc:
                print(f"Error handling HAND_MOVE: {exc}")

        @app.add_handler("CAMERA_MOVE")
        async def on_cam_move(event, session, fps=60):
            try:
                camera = event.value.get("camera", {})
                if isinstance(camera.get("matrix"), list):
                    self.head_matrix_shared[:] = camera["matrix"]
                if "aspect" in camera:
                    self.aspect_shared.value = float(camera["aspect"])
            except Exception:
                pass

        @app.spawn(start=False)
        async def main_image(session, fps=60):
            session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False, computeGestures=True)
            existing_shm = shared_memory.SharedMemory(name=self.shm_name)
            height, width = self.image_shape
            image_array = np.ndarray((height, width * 2, 3), dtype=np.uint8, buffer=existing_shm.buf)
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
                        ),
                    ],
                    to="bgChildren",
                )
                await asyncio.sleep(0.03)

        app.run()
