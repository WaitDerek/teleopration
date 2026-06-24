from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Optional

import numpy as np

from teleoperation.avp import OpenTeleVision
from teleoperation.cli.network import guess_lan_ip
from teleoperation.cli.url_display import show_open_url
from teleoperation.session import TeleopSession
from teleoperation.types import StreamState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record live Vision Pro teleoperation data to a replayable NPZ file.")
    parser.add_argument("--output", default=None, help="Output .npz path. Defaults to recordings/avp_<timestamp>.npz.")
    parser.add_argument("--duration", type=float, default=10.0, help="Recording duration after first valid pose sample.")
    parser.add_argument("--rate", type=float, default=30.0, help="Sampling rate in Hz.")
    parser.add_argument("--cert", default="./cert.pem", help="TLS certificate for Vuer.")
    parser.add_argument("--key", default="./key.pem", help="TLS key for Vuer.")
    parser.add_argument("--ngrok", action="store_true", help="Run Vuer without local TLS cert/key.")
    parser.add_argument("--public-host", default=None, help="LAN IP or hostname opened from Vision Pro.")
    parser.add_argument("--port", type=int, default=8012, help="Vuer HTTPS/websocket port.")
    parser.add_argument("--client-url", default=None, help="Vuer web client URL.")
    parser.add_argument("--debug-avp", action="store_true", help="Print Vuer HTTP/websocket and AVP event diagnostics.")
    parser.add_argument("--hide-hands", action="store_true", help="Hide Vuer hand overlays during recording.")
    parser.add_argument(
        "--hide-images",
        action="store_true",
        help="Disable image heartbeat. This can make HAND_MOVE less reliable in some Vuer clients.",
    )
    parser.add_argument(
        "--image-opacity",
        type=float,
        default=0.05,
        help="Opacity for the image heartbeat/background.",
    )
    parser.add_argument("--stale-after", type=float, default=0.5, help="Treat AVP events as stale after this many seconds.")
    parser.add_argument(
        "--calibration-delay-samples",
        type=int,
        default=30,
        help="Ignore this many valid right-hand samples before setting the zero pose.",
    )
    parser.add_argument(
        "--position-scale",
        type=float,
        default=0.1,
        help="Scale AVP translation deltas before saving replay pose data.",
    )
    parser.add_argument(
        "--orientation-scale",
        type=float,
        default=1.0,
        help="Scale AVP relative rotation angle before saving replay pose data. Use 0 to lock orientation.",
    )
    parser.add_argument(
        "--max-position-norm",
        type=float,
        default=0.10,
        help="Drop scaled pose deltas above this many meters. Use <=0 to disable.",
    )
    parser.add_argument(
        "--max-angular-norm",
        type=float,
        default=0.0,
        help="Drop scaled rotation deltas above this many radians. Use <=0 to disable.",
    )
    parser.add_argument("--print-every", type=float, default=1.0, help="Status print interval. Use 0 to disable.")
    return parser.parse_args()


def _default_output_path() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("recordings") / f"avp_{stamp}.npz"


def _matrix_or_nan(matrix: Optional[np.ndarray]) -> np.ndarray:
    if matrix is None:
        return np.full((4, 4), np.nan, dtype=float)
    return np.asarray(matrix, dtype=float)


def main() -> None:
    args = parse_args()
    if args.duration <= 0:
        raise ValueError("--duration must be positive")
    if args.rate <= 0:
        raise ValueError("--rate must be positive")
    if args.calibration_delay_samples < 0:
        raise ValueError("--calibration-delay-samples must be non-negative")
    if args.position_scale <= 0:
        raise ValueError("--position-scale must be positive")
    if args.orientation_scale < 0:
        raise ValueError("--orientation-scale must be non-negative")

    max_position_norm = None if args.max_position_norm <= 0 else args.max_position_norm
    max_angular_norm = None if args.max_angular_norm <= 0 else args.max_angular_norm
    output_path = Path(args.output) if args.output else _default_output_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    public_host = None if args.ngrok else (args.public_host or guess_lan_ip())
    source = OpenTeleVision(
        cert_file=args.cert,
        key_file=args.key,
        ngrok=args.ngrok,
        public_host=public_host,
        port=args.port,
        debug=args.debug_avp,
        show_hands=not args.hide_hands,
        show_images=not args.hide_images,
        image_opacity=args.image_opacity,
        client_url=args.client_url,
    )
    session = TeleopSession(
        stale_after_sec=args.stale_after,
        position_scale=args.position_scale,
        max_position_norm=max_position_norm,
        orientation_scale=args.orientation_scale,
        max_angular_norm=max_angular_norm,
        calibration_delay_samples=args.calibration_delay_samples,
    )

    period = 1.0 / args.rate
    launch_time = time.monotonic()
    record_start: Optional[float] = None
    last_print = launch_time
    last_gripper: Optional[float] = None
    gripper_changes = 0

    time_sec: list[float] = []
    wall_time_sec: list[float] = []
    right_hand_matrix: list[np.ndarray] = []
    head_matrix: list[np.ndarray] = []
    pose_position: list[np.ndarray] = []
    pose_orientation_wxyz: list[np.ndarray] = []
    right_pinch: list[bool] = []
    gripper_position: list[float] = []

    source.start()
    print("Waiting for Vision Pro browser input...")
    show_open_url(source.stable_browser_url(), label="Vision Pro URL")

    try:
        while True:
            now = time.monotonic()
            state = source.state(args.stale_after)
            pose = session.target_from_source(source)
            gripper = session.gripper_from_source(source)

            if gripper is not None:
                if last_gripper is not None and abs(gripper.position - last_gripper) > 1e-6:
                    gripper_changes += 1
                last_gripper = gripper.position

            if state == StreamState.STREAMING and pose is not None:
                if record_start is None:
                    record_start = now
                    print("First streaming pose sample received; starting recording.")
                elapsed = now - record_start
                if elapsed >= args.duration:
                    break

                raw_right_hand = source.latest_right_hand_matrix()
                raw_head = source.latest_head_matrix()
                time_sec.append(elapsed)
                wall_time_sec.append(time.time())
                right_hand_matrix.append(_matrix_or_nan(raw_right_hand))
                head_matrix.append(_matrix_or_nan(raw_head))
                pose_position.append(pose.position.copy())
                pose_orientation_wxyz.append(pose.orientation_wxyz.copy())
                right_pinch.append(source.right_pinch)
                gripper_position.append(float("nan") if gripper is None else gripper.position)

            if args.print_every > 0 and now - last_print >= args.print_every:
                elapsed_text = "waiting" if record_start is None else f"{now - record_start:5.2f}s"
                pose_text = "none" if pose is None else pose.position.round(4).tolist()
                diag = source.diagnostics()
                print(
                    f"record={elapsed_text} state={state.value} samples={len(time_sec)} "
                    f"pose_delta={pose_text} gripper_changes={gripper_changes} "
                    f"http={diag['http_requests']} ws={diag['ws_active']}/{diag['ws_connects']} "
                    f"cam={diag['camera_events']} hand={diag['hand_events']} "
                    f"valid_hand={diag['valid_right_hand_events']} skip_hand={diag['skipped_hand_events']}"
                )
                last_print = now

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        source.stop()

    if not time_sec:
        diag = source.diagnostics()
        print(
            "diagnostics: "
            f"http={diag['http_requests']} ws={diag['ws_active']}/{diag['ws_connects']} cam={diag['camera_events']} "
            f"hand={diag['hand_events']} valid_hand={diag['valid_right_hand_events']} "
            f"skip_cam={diag['skipped_camera_events']} skip_hand={diag['skipped_hand_events']}"
        )
        raise SystemExit("FAIL: no valid pose samples recorded")

    metadata = {
        "created_time_sec": time.time(),
        "duration_sec": args.duration,
        "rate_hz": args.rate,
        "position_scale": args.position_scale,
        "orientation_scale": args.orientation_scale,
        "max_position_norm": max_position_norm,
        "max_angular_norm": max_angular_norm,
        "calibration_delay_samples": args.calibration_delay_samples,
        "stale_after_sec": args.stale_after,
        "public_host": public_host,
        "port": args.port,
        "columns": {
            "pose_position": "scaled relative translation delta in current teleoperation frame, meters",
            "pose_orientation_wxyz": "scaled relative orientation delta quaternion, wxyz",
            "right_hand_matrix": "raw latest right hand 4x4 matrix from AVP/Vuer",
            "head_matrix": "raw latest camera/head 4x4 matrix from AVP/Vuer when valid, otherwise NaN",
            "gripper_position": "normalized closure, 0 open and 1 closed",
        },
    }
    np.savez_compressed(
        output_path,
        time_sec=np.asarray(time_sec, dtype=float),
        wall_time_sec=np.asarray(wall_time_sec, dtype=float),
        right_hand_matrix=np.asarray(right_hand_matrix, dtype=float),
        head_matrix=np.asarray(head_matrix, dtype=float),
        pose_position=np.asarray(pose_position, dtype=float),
        pose_orientation_wxyz=np.asarray(pose_orientation_wxyz, dtype=float),
        right_pinch=np.asarray(right_pinch, dtype=bool),
        gripper_position=np.asarray(gripper_position, dtype=float),
        metadata_json=np.asarray(json.dumps(metadata, sort_keys=True), dtype=np.str_),
    )
    print(
        f"saved={output_path} samples={len(time_sec)} duration={time_sec[-1]:.2f}s "
        f"gripper_changes={gripper_changes}"
    )


if __name__ == "__main__":
    main()
