from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from teleoperation.avp import OpenTeleVision
from teleoperation.cli.config import config_float_tuple, config_int, config_value, local_client_url
from teleoperation.cli.url_display import show_open_url
from teleoperation.preprocessing.transforms import apply_frame_rotation, relative_avp_hand_pose
from teleoperation.types import StreamState

DEFAULT_FRAME_CALIBRATION_FILE = "recordings/avp_forward_xyz_calibration.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print calibrated AVP right-hand delta from the start pose.")
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
    parser.add_argument(
        "--hide-images",
        action="store_true",
        help="Disable shared-memory image background.",
    )
    parser.add_argument("--image-opacity", type=float, default=0.0, help="Image background opacity.")
    parser.add_argument("--rate", type=float, default=10.0, help="Print rate in Hz.")
    parser.add_argument("--stale-after", type=float, default=0.5, help="AVP stream stale timeout in seconds.")
    parser.add_argument(
        "--zero-samples",
        type=int,
        default=30,
        help="Number of valid hand samples to ignore before setting the zero pose.",
    )
    parser.add_argument(
        "--position-scale",
        type=float,
        default=0.08,
        help="Scale calibrated translation deltas the same way as publish_avp_pose.",
    )
    parser.add_argument(
        "--output-axis-sign",
        default=",".join(str(value) for value in config_float_tuple("OUTPUT_AXIS_SIGN", default=(-1.0, 1.0, 1.0))),
        help="Comma-separated signs applied to calibrated output XYZ before scaling. Default from config/teleop.env.",
    )
    parser.add_argument(
        "--frame-calibration-file",
        default=config_value("FRAME_CALIBRATION_FILE", default=DEFAULT_FRAME_CALIBRATION_FILE),
        help="JSON file containing a 3x3 rotation_matrix from calibrate_avp_frame.",
    )
    parser.add_argument(
        "--align-wrist-to-base",
        action="store_true",
        help="Use raw wrist-relative deltas instead of the default AVP-to-UR mapping.",
    )
    return parser.parse_args()


def load_calibration(path: str) -> tuple[np.ndarray, bool]:
    calibration = json.loads(Path(path).read_text())
    rotation_value = calibration.get("rotation_matrix", calibration)
    rotation = np.asarray(rotation_value, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("--frame-calibration-file must contain a 3x3 rotation_matrix")
    return rotation, bool(calibration.get("align_wrist_to_base", False))


def latest_hand_or_none(source: OpenTeleVision, stale_after_sec: float) -> np.ndarray | None:
    if source.state(stale_after_sec) != StreamState.STREAMING:
        return None
    hand = source.latest_right_hand_matrix()
    return None if hand is None else hand.copy()


def main() -> None:
    args = parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be positive")
    if args.zero_samples < 0:
        raise ValueError("--zero-samples must be non-negative")
    if args.position_scale <= 0:
        raise ValueError("--position-scale must be positive")
    output_axis_sign = np.asarray([float(part.strip()) for part in args.output_axis_sign.split(",")], dtype=float)
    if output_axis_sign.shape != (3,):
        raise ValueError("--output-axis-sign must contain exactly 3 comma-separated numbers")
    if np.linalg.det(np.diag(output_axis_sign)) <= 0:
        raise ValueError("--output-axis-sign must preserve handedness, for example -1,-1,1")

    frame_rotation, calibration_align_wrist = load_calibration(args.frame_calibration_file)
    align_wrist_to_base = args.align_wrist_to_base or calibration_align_wrist

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
        show_images=not args.hide_images,
        image_opacity=args.image_opacity,
        client_url=client_url,
    )
    period = 1.0 / args.rate

    source.start()
    show_open_url(source.stable_browser_url(), label="Vision Pro URL")
    try:
        print("Open the URL on Vision Pro and enter the Vuer session.")
        print("Waiting for valid right-hand samples before setting zero pose...")
        start_hand = None
        valid_samples = 0
        while start_hand is None:
            hand = latest_hand_or_none(source, args.stale_after)
            if hand is None:
                valid_samples = 0
                time.sleep(0.02)
                continue
            valid_samples += 1
            if valid_samples > args.zero_samples:
                start_hand = hand
            time.sleep(0.02)

        print("Zero pose set. Move your hand to inspect calibrated delta.")
        print("raw_delta_m is before position_scale; scaled_delta_m matches publish_avp_pose translation scaling.")
        while True:
            hand = latest_hand_or_none(source, args.stale_after)
            if hand is None:
                print("stream stale: waiting for right hand...")
                time.sleep(period)
                continue
            transform = relative_avp_hand_pose(
                start_hand,
                hand,
                align_wrist_to_base=align_wrist_to_base,
                frame_rotation=frame_rotation,
            )
            raw_delta = transform[:3, 3]
            signed_transform = apply_frame_rotation(transform, np.diag(output_axis_sign))
            signed_delta = signed_transform[:3, 3]
            scaled_delta = signed_delta * args.position_scale
            print(
                "raw_delta_m="
                f"x={raw_delta[0]:+.4f} y={raw_delta[1]:+.4f} z={raw_delta[2]:+.4f} "
                "signed_delta_m="
                f"x={signed_delta[0]:+.4f} y={signed_delta[1]:+.4f} z={signed_delta[2]:+.4f} "
                "scaled_delta_m="
                f"x={scaled_delta[0]:+.4f} y={scaled_delta[1]:+.4f} z={scaled_delta[2]:+.4f}",
                flush=True,
            )
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping AVP calibrated delta check.")
    finally:
        source.stop()


if __name__ == "__main__":
    main()
