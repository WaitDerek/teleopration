from __future__ import annotations

import argparse
import time
from typing import Optional

import numpy as np

from teleoperation.avp import OpenTeleVision
from teleoperation.cli.network import guess_lan_ip
from teleoperation.cli.url_display import show_open_url
from teleoperation.ros2.pose_publisher import _require_ros2, make_pose_publisher_node
from teleoperation.session import TeleopSession
from teleoperation.types import StreamState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check live Vision Pro teleoperation pose and pinch output.")
    parser.add_argument("--cert", default="./cert.pem", help="TLS certificate for Vuer.")
    parser.add_argument("--key", default="./key.pem", help="TLS key for Vuer.")
    parser.add_argument("--ngrok", action="store_true", help="Run Vuer without local TLS cert/key.")
    parser.add_argument("--public-host", default=None, help="LAN IP or hostname opened from Vision Pro.")
    parser.add_argument("--port", type=int, default=8012, help="Vuer HTTPS/websocket port.")
    parser.add_argument(
        "--client-url",
        default=None,
        help="Vuer web client URL. Use https://vuer.ai to load the official client while connecting to local WSS.",
    )
    parser.add_argument("--debug-avp", action="store_true", help="Print Vuer HTTP/websocket and AVP event diagnostics.")
    parser.add_argument("--hide-hands", action="store_true", help="Hide Vuer hand overlays during the check.")
    parser.add_argument(
        "--hide-images",
        action="store_true",
        help="Disable shared-memory stereo image heartbeat. This can make HAND_MOVE less reliable in some Vuer clients.",
    )
    parser.add_argument(
        "--image-opacity",
        type=float,
        default=0.05,
        help="Opacity for the image heartbeat/background. Low values reduce the dark overlay.",
    )
    parser.add_argument("--duration", type=float, default=15.0, help="Check duration in seconds. Use <=0 to run forever.")
    parser.add_argument(
        "--count-from-start",
        action="store_true",
        help="Start --duration at process launch instead of after the first streaming pose sample.",
    )
    parser.add_argument("--rate", type=float, default=30.0, help="Sampling rate in Hz.")
    parser.add_argument("--stale-after", type=float, default=0.5, help="Treat AVP events as stale after this many seconds.")
    parser.add_argument(
        "--calibration-delay-samples",
        type=int,
        default=0,
        help="Ignore this many valid right-hand samples before setting the zero pose.",
    )
    parser.add_argument(
        "--position-scale",
        type=float,
        default=1.0,
        help="Scale AVP translation deltas before checking or publishing.",
    )
    parser.add_argument(
        "--orientation-scale",
        type=float,
        default=1.0,
        help="Scale AVP relative rotation angle before checking or publishing. Use 0 to lock orientation.",
    )
    parser.add_argument(
        "--max-position-norm",
        type=float,
        default=None,
        help="Drop pose deltas whose scaled translation norm exceeds this many meters.",
    )
    parser.add_argument(
        "--max-angular-norm",
        type=float,
        default=None,
        help="Drop pose deltas whose scaled relative rotation exceeds this many radians.",
    )
    parser.add_argument("--min-samples", type=int, default=10, help="Minimum valid pose samples required to pass.")
    parser.add_argument("--require-motion", type=float, default=0.0, help="Require max translation norm above this value.")
    parser.add_argument("--require-gripper-change", action="store_true", help="Require at least one pinch state transition.")
    parser.add_argument("--print-every", type=float, default=1.0, help="Status print interval in seconds.")
    parser.add_argument("--publish", action="store_true", help="Also publish checked output to ROS2.")
    parser.add_argument("--topic", default="Target_Pose", help="ROS2 PoseStamped topic when --publish is set.")
    parser.add_argument("--frame-id", default="base_link", help="PoseStamped frame_id when --publish is set.")
    parser.add_argument(
        "--gripper-topic",
        default="Gripper_Command",
        help="ROS2 std_msgs/Float32 gripper topic when --publish is set.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be positive")
    if args.calibration_delay_samples < 0:
        raise ValueError("--calibration-delay-samples must be non-negative")
    if args.position_scale <= 0:
        raise ValueError("--position-scale must be positive")
    if args.orientation_scale < 0:
        raise ValueError("--orientation-scale must be non-negative")
    if args.max_position_norm is not None and args.max_position_norm <= 0:
        raise ValueError("--max-position-norm must be positive")
    if args.max_angular_norm is not None and args.max_angular_norm <= 0:
        raise ValueError("--max-angular-norm must be positive")

    rclpy = None
    node = None
    if args.publish:
        rclpy, _, _ = _require_ros2()
        rclpy.init()
        node_cls = make_pose_publisher_node(
            topic_name=args.topic,
            frame_id=args.frame_id,
            gripper_topic_name=args.gripper_topic,
        )
        node = node_cls()

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
        max_position_norm=args.max_position_norm,
        orientation_scale=args.orientation_scale,
        max_angular_norm=args.max_angular_norm,
        calibration_delay_samples=args.calibration_delay_samples,
    )
    period = 1.0 / args.rate
    launch_time = time.monotonic()
    check_start: Optional[float] = launch_time if args.count_from_start else None
    last_print = launch_time
    valid_samples = 0
    stale_samples = 0
    max_position_norm = 0.0
    last_gripper: Optional[float] = None
    gripper_changes = 0
    last_pose = None
    last_state = StreamState.WAITING

    source.start()
    print("Waiting for Vision Pro browser input...")
    show_open_url(source.stable_browser_url(), label="Vision Pro URL")
    try:
        while True:
            now = time.monotonic()
            wall_elapsed = now - launch_time
            check_elapsed = 0.0 if check_start is None else now - check_start
            if args.duration > 0 and check_start is not None and check_elapsed >= args.duration:
                break

            state = source.state(args.stale_after)
            last_state = state
            pose = session.target_from_source(source)
            gripper = session.gripper_from_source(source)

            if state == StreamState.STALE:
                stale_samples += 1
            if pose is not None and state == StreamState.STREAMING:
                if check_start is None:
                    check_start = now
                    check_elapsed = 0.0
                    print("First streaming pose sample received; starting timed check.")
                valid_samples += 1
                last_pose = pose
                max_position_norm = max(max_position_norm, float(np.linalg.norm(pose.position)))
                if node is not None:
                    node.publish_target(pose)
            if gripper is not None:
                if last_gripper is not None and abs(gripper.position - last_gripper) > 1e-6:
                    gripper_changes += 1
                last_gripper = gripper.position
                if node is not None:
                    node.publish_gripper(gripper)

            if args.print_every > 0 and now - last_print >= args.print_every:
                pose_text = "none" if last_pose is None else last_pose.position.round(4).tolist()
                gripper_text = "none" if last_gripper is None else f"{last_gripper:.2f}"
                timer_name = "wait" if check_start is None else "t"
                elapsed = wall_elapsed if check_start is None else check_elapsed
                diag = source.diagnostics()
                print(
                    f"{timer_name}={elapsed:6.2f}s state={state.value} valid={valid_samples} "
                    f"pose_delta={pose_text} gripper={gripper_text} gripper_changes={gripper_changes} "
                    f"http={diag['http_requests']} ws={diag['ws_active']}/{diag['ws_connects']} "
                    f"cam={diag['camera_events']} hand={diag['hand_events']} "
                    f"valid_hand={diag['valid_right_hand_events']} skip_hand={diag['skipped_hand_events']}"
                )
                last_print = now
            if rclpy is not None and node is not None:
                rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        source.stop()
        if node is not None:
            node.destroy_node()
        if rclpy is not None:
            rclpy.shutdown()

    print(
        "summary: "
        f"state={last_state.value} valid_samples={valid_samples} stale_samples={stale_samples} "
        f"max_position_norm={max_position_norm:.4f} gripper_changes={gripper_changes}"
    )
    diag = source.diagnostics()
    print(
        "diagnostics: "
        f"http={diag['http_requests']} ws={diag['ws_active']}/{diag['ws_connects']} cam={diag['camera_events']} "
        f"hand={diag['hand_events']} valid_hand={diag['valid_right_hand_events']} "
        f"skip_cam={diag['skipped_camera_events']} skip_hand={diag['skipped_hand_events']}"
    )
    failed = False
    if valid_samples < args.min_samples:
        print(f"FAIL: valid_samples {valid_samples} < min_samples {args.min_samples}")
        failed = True
    if args.require_motion > 0 and max_position_norm < args.require_motion:
        print(f"FAIL: max_position_norm {max_position_norm:.4f} < require_motion {args.require_motion:.4f}")
        failed = True
    if args.require_gripper_change and gripper_changes < 1:
        print("FAIL: no pinch/gripper state transition observed")
        failed = True
    if failed:
        raise SystemExit(1)
    print("PASS: Vision Pro output is streaming valid pose samples")


if __name__ == "__main__":
    main()
