from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np

from teleoperation.avp import OpenTeleVision
from teleoperation.cli.config import config_int, config_value, local_client_url
from teleoperation.cli.url_display import show_open_url
from teleoperation.preprocessing.transforms import relative_avp_hand_pose
from teleoperation.types import StreamState


AXES = [
    ("x", np.array([1.0, 0.0, 0.0], dtype=float)),
    ("y", np.array([0.0, 1.0, 0.0], dtype=float)),
    ("z", np.array([0.0, 0.0, 1.0], dtype=float)),
]

DEFAULT_FRAME_CALIBRATION_FILE = "recordings/avp_forward_xyz_calibration.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate AVP movement axes to UR base axes.")
    parser.add_argument("--cert", default="./cert.pem", help="TLS certificate for Vuer.")
    parser.add_argument("--key", default="./key.pem", help="TLS key for Vuer.")
    parser.add_argument(
        "--public-host",
        default=config_value("PUBLIC_HOST"),
        help="LAN IP or hostname opened from Vision Pro. Defaults to config/teleop.env PUBLIC_HOST.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=config_int("PORT", default=8012),
        help="Vuer HTTPS/websocket port.",
    )
    parser.add_argument(
        "--client-url",
        default=None,
        help="Vuer web client URL. Use https://192.168.x.x:8012 for local client.",
    )
    parser.add_argument("--debug-avp", action="store_true", help="Print Vuer diagnostics.")
    parser.add_argument("--hide-hands", action="store_true", help="Hide Vuer hand overlays on Vision Pro.")
    parser.add_argument("--image-opacity", type=float, default=0.0, help="Image heartbeat opacity.")
    parser.add_argument("--sample-rate", type=float, default=30.0, help="Hand sampling rate in Hz.")
    parser.add_argument("--capture-sec", type=float, default=3.0, help="Seconds to capture each axis movement.")
    parser.add_argument(
        "--enter-to-stop",
        action="store_true",
        help="Press Enter once to start each axis capture, then press Enter again to stop it.",
    )
    parser.add_argument("--stale-after", type=float, default=0.5, help="AVP stream stale timeout in seconds.")
    parser.add_argument(
        "--align-wrist-to-base",
        action="store_true",
        help="Calibrate on raw wrist-relative deltas instead of the default AVP-to-UR mapping.",
    )
    parser.add_argument(
        "--output",
        default=config_value("FRAME_CALIBRATION_FILE", default=DEFAULT_FRAME_CALIBRATION_FILE),
        help="Output calibration JSON path.",
    )
    return parser.parse_args()


def wait_for_hand(source: OpenTeleVision, stale_after_sec: float) -> np.ndarray:
    while True:
        if source.state(stale_after_sec) == StreamState.STREAMING:
            matrix = source.latest_right_hand_matrix()
            if matrix is not None:
                return matrix.copy()
        time.sleep(0.02)


def capture_axis(source: OpenTeleVision, args: argparse.Namespace, axis_name: str) -> np.ndarray:
    input(f"\nHold your right hand at the start point for UR base +{axis_name.upper()}, then press Enter.")
    start = wait_for_hand(source, args.stale_after)
    if args.enter_to_stop:
        print(f"Move only along UR base +{axis_name.upper()}, then press Enter again to stop capture.")
    else:
        print(f"Move only along UR base +{axis_name.upper()} for {args.capture_sec:.1f}s, starting now.")

    period = 1.0 / args.sample_rate
    end = start
    if args.enter_to_stop:
        stop_event = threading.Event()

        def wait_for_stop() -> None:
            input()
            stop_event.set()

        thread = threading.Thread(target=wait_for_stop, daemon=True)
        thread.start()
        while not stop_event.is_set():
            end = wait_for_hand(source, args.stale_after)
            time.sleep(period)
    else:
        deadline = time.monotonic() + args.capture_sec
        while time.monotonic() < deadline:
            end = wait_for_hand(source, args.stale_after)
            time.sleep(period)

    transform = relative_avp_hand_pose(
        start,
        end,
        align_wrist_to_base=args.align_wrist_to_base,
    )
    vector = transform[:3, 3]
    norm = float(np.linalg.norm(vector))
    if norm < 1e-6:
        raise RuntimeError(f"Captured +{axis_name.upper()} movement is too small")
    direction = vector / norm
    print(f"Captured +{axis_name.upper()} measured direction: {direction.tolist()} length={norm:.4f}")
    return direction


def solve_rotation(measured_axes: np.ndarray, desired_axes: np.ndarray) -> np.ndarray:
    covariance = desired_axes @ measured_axes.T
    u, _, vt = np.linalg.svd(covariance)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def main() -> None:
    args = parse_args()
    if args.sample_rate <= 0:
        raise ValueError("--sample-rate must be positive")
    if not args.enter_to_stop and args.capture_sec <= 0:
        raise ValueError("--capture-sec must be positive")

    public_host = args.public_host
    if not public_host:
        raise ValueError("PUBLIC_HOST is required. Set it in config/teleop.env or pass --public-host.")
    client_url = args.client_url or local_client_url(public_host, args.port)
    source = OpenTeleVision(
        cert_file=args.cert,
        key_file=args.key,
        public_host=public_host,
        port=args.port,
        debug=args.debug_avp,
        show_hands=not args.hide_hands,
        image_opacity=args.image_opacity,
        client_url=client_url,
    )

    measured = []
    desired = []
    source.start()
    show_open_url(source.stable_browser_url(), label="Vision Pro URL")
    try:
        print("Open the URL on Vision Pro and enter the Vuer session before starting calibration.")
        for axis_name, axis_vector in AXES:
            measured.append(capture_axis(source, args, axis_name))
            desired.append(axis_vector)

        measured_axes = np.stack(measured, axis=1)
        desired_axes = np.stack(desired, axis=1)
        rotation = solve_rotation(measured_axes, desired_axes)
        residual = rotation @ measured_axes - desired_axes
        rms_error = float(np.sqrt(np.mean(residual * residual)))

        output = {
            "rotation_matrix": rotation.tolist(),
            "measured_axes": measured_axes.tolist(),
            "desired_axes": desired_axes.tolist(),
            "rms_error": rms_error,
            "align_wrist_to_base": bool(args.align_wrist_to_base),
        }
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(output, indent=2) + "\n")
        print(f"\nSaved calibration to: {output_path}")
        print(f"RMS axis error: {rms_error:.6f}")
        print("Use it with: --frame-calibration-file " + str(output_path))
    finally:
        source.stop()


if __name__ == "__main__":
    main()
