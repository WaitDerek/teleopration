from __future__ import annotations

import argparse
import time
from multiprocessing import shared_memory

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream a USB camera into the shared-memory image buffer used by Vision Pro."
    )
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index.")
    parser.add_argument(
        "--shm-name",
        default="teleoperation_avp_image_stream",
        help="Shared memory name created by teleop-publish-avp-pose.",
    )
    parser.add_argument("--capture-width", type=int, default=1280, help="Requested USB camera width.")
    parser.add_argument("--capture-height", type=int, default=720, help="Requested USB camera height.")
    parser.add_argument("--capture-fps", type=float, default=30.0, help="Requested USB camera FPS.")
    parser.add_argument("--image-width", type=int, default=1000, help="Per-eye output width in pixels.")
    parser.add_argument("--image-height", type=int, default=450, help="Per-eye output height in pixels.")
    parser.add_argument(
        "--side-by-side",
        action="store_true",
        help="Interpret the USB frame as stereo side-by-side instead of duplicating one image.",
    )
    parser.add_argument(
        "--no-mirror",
        action="store_true",
        help="Disable horizontal mirroring for monocular USB cameras.",
    )
    parser.add_argument(
        "--wait-for-shm",
        action="store_true",
        help="Wait until the teleoperation image shared memory exists.",
    )
    parser.add_argument(
        "--backend-v4l2",
        action="store_true",
        help="Use cv2.CAP_V4L2 when opening the camera on Linux.",
    )
    return parser.parse_args()


def _open_camera(index: int, use_v4l2: bool):
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is required. Install image dependencies first.") from exc

    backend = cv2.CAP_V4L2 if use_v4l2 else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened() and use_v4l2:
        cap = cv2.VideoCapture(index)
    return cv2, cap


def _connect_shm(name: str, wait: bool):
    while True:
        try:
            return shared_memory.SharedMemory(name=name)
        except FileNotFoundError:
            if not wait:
                raise
            print(f"Waiting for shared memory: {name}", flush=True)
            time.sleep(0.5)


def main() -> None:
    args = parse_args()
    if args.capture_width <= 0 or args.capture_height <= 0:
        raise ValueError("capture dimensions must be positive")
    if args.image_width <= 0 or args.image_height <= 0:
        raise ValueError("image dimensions must be positive")
    if args.capture_fps <= 0:
        raise ValueError("capture FPS must be positive")

    cv2, cap = _open_camera(args.camera_index, args.backend_v4l2)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open USB camera index {args.camera_index}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.capture_width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.capture_height))
    cap.set(cv2.CAP_PROP_FPS, float(args.capture_fps))

    shm = _connect_shm(args.shm_name, wait=args.wait_for_shm)
    frame_shape = (args.image_height, args.image_width * 2, 3)
    shm_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)

    print(
        f"Streaming USB camera {args.camera_index} to shared memory {args.shm_name} "
        f"as {args.image_width}x{args.image_height} per eye",
        flush=True,
    )

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.03)
                continue

            if args.side_by_side:
                half_width = frame.shape[1] // 2
                left_frame = frame[:, :half_width]
                right_frame = frame[:, half_width:]
            else:
                if not args.no_mirror:
                    frame = cv2.flip(frame, 1)
                left_frame = frame
                right_frame = frame

            left_frame = cv2.resize(left_frame, (args.image_width, args.image_height), interpolation=cv2.INTER_AREA)
            right_frame = cv2.resize(
                right_frame, (args.image_width, args.image_height), interpolation=cv2.INTER_AREA
            )

            # Vuer expects RGB image arrays.
            shm_array[:, : args.image_width] = cv2.cvtColor(left_frame, cv2.COLOR_BGR2RGB)
            shm_array[:, args.image_width :] = cv2.cvtColor(right_frame, cv2.COLOR_BGR2RGB)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        shm.close()


if __name__ == "__main__":
    main()
