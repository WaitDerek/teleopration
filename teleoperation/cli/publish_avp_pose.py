from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from teleoperation.avp import OpenTeleVision
from teleoperation.cli.network import guess_lan_ip
from teleoperation.cli.url_display import show_open_url
from teleoperation.ros2.pose_publisher import _require_ros2, make_pose_publisher_node
from teleoperation.session import TeleopSession
from teleoperation.types import StreamState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish AVP right-hand teleoperation target poses to ROS2.")
    parser.add_argument("--topic", default="Target_Pose", help="ROS2 PoseStamped topic to publish.")
    parser.add_argument("--frame-id", default="base_link", help="PoseStamped frame_id.")
    parser.add_argument(
        "--gripper-topic",
        default="Gripper_Command",
        help="ROS2 std_msgs/Float32 topic for normalized gripper closure, 0.0 open and 1.0 closed.",
    )
    parser.add_argument("--no-gripper", action="store_true", help="Do not publish AVP pinch gripper commands.")
    parser.add_argument("--gripper-open", type=float, default=0.0, help="Published gripper value when not pinching.")
    parser.add_argument("--gripper-closed", type=float, default=1.0, help="Published gripper value when pinching.")
    parser.add_argument("--gripper-effort", type=float, default=0.5, help="Normalized effort stored in GripperCommand.")
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
    parser.add_argument("--show-hands", action="store_true", help="Show Vuer hand overlays on Vision Pro.")
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
    parser.add_argument("--rate", type=float, default=50.0, help="Publish loop rate in Hz.")
    parser.add_argument("--stale-after", type=float, default=0.5, help="Stop publishing after this many seconds without AVP events.")
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
        help="Scale AVP translation deltas before publishing.",
    )
    parser.add_argument(
        "--orientation-scale",
        type=float,
        default=0.25,
        help="Scale AVP relative rotation angle before publishing. Use 0 to lock orientation.",
    )
    parser.add_argument(
        "--max-position-norm",
        type=float,
        default=0.10,
        help="Drop pose deltas whose scaled translation norm exceeds this many meters.",
    )
    parser.add_argument(
        "--max-angular-norm",
        type=float,
        default=None,
        help="Drop pose deltas whose scaled relative rotation exceeds this many radians.",
    )
    parser.add_argument(
        "--align-wrist-to-base",
        action="store_true",
        help="Treat the initial right-wrist XYZ axes as aligned with the UR base XYZ axes.",
    )
    parser.add_argument(
        "--frame-roll-deg",
        type=float,
        default=0.0,
        help="Rotate published deltas around UR base +X by this many degrees.",
    )
    parser.add_argument(
        "--frame-pitch-deg",
        type=float,
        default=0.0,
        help="Rotate published deltas around UR base +Y by this many degrees.",
    )
    parser.add_argument(
        "--frame-yaw-deg",
        type=float,
        default=0.0,
        help="Rotate published deltas around UR base +Z by this many degrees.",
    )
    parser.add_argument(
        "--frame-calibration-file",
        default=None,
        help="JSON file containing a 3x3 rotation_matrix from calibrate_avp_frame.",
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
    frame_rotation_matrix = None
    if args.frame_calibration_file:
        calibration_path = Path(args.frame_calibration_file)
        calibration = json.loads(calibration_path.read_text())
        rotation_value = calibration.get("rotation_matrix", calibration)
        frame_rotation_matrix = np.asarray(rotation_value, dtype=float)
        if frame_rotation_matrix.shape != (3, 3):
            raise ValueError("--frame-calibration-file must contain a 3x3 rotation_matrix")

    rclpy, _, _ = _require_ros2()
    rclpy.init()
    gripper_topic = None if args.no_gripper else args.gripper_topic
    node_cls = make_pose_publisher_node(topic_name=args.topic, frame_id=args.frame_id, gripper_topic_name=gripper_topic)
    node = node_cls()
    public_host = None if args.ngrok else (args.public_host or guess_lan_ip())
    source = OpenTeleVision(
        cert_file=args.cert,
        key_file=args.key,
        ngrok=args.ngrok,
        public_host=public_host,
        port=args.port,
        debug=args.debug_avp,
        show_hands=args.show_hands,
        show_images=not args.hide_images,
        image_opacity=args.image_opacity,
        client_url=args.client_url,
    )
    session = TeleopSession(
        stale_after_sec=args.stale_after,
        gripper_open_position=args.gripper_open,
        gripper_closed_position=args.gripper_closed,
        gripper_effort=args.gripper_effort,
        position_scale=args.position_scale,
        max_position_norm=args.max_position_norm,
        orientation_scale=args.orientation_scale,
        max_angular_norm=args.max_angular_norm,
        calibration_delay_samples=args.calibration_delay_samples,
        align_wrist_to_base=args.align_wrist_to_base,
        frame_roll_deg=args.frame_roll_deg,
        frame_pitch_deg=args.frame_pitch_deg,
        frame_yaw_deg=args.frame_yaw_deg,
        frame_rotation_matrix=frame_rotation_matrix,
    )
    period = 1.0 / args.rate

    source.start()
    show_open_url(source.stable_browser_url(), label="Vision Pro URL")
    if gripper_topic is None:
        node.get_logger().info(f"Publishing AVP teleoperation target poses on {args.topic}")
    else:
        node.get_logger().info(
            f"Publishing AVP target poses on {args.topic} and gripper closure on {gripper_topic}"
        )
    try:
        while rclpy.ok():
            state = source.state(args.stale_after)
            pose = session.target_from_source(source)
            if pose is not None and state == StreamState.STREAMING:
                node.publish_target(pose)
            gripper_command = session.gripper_from_source(source)
            if gripper_command is not None:
                node.publish_gripper(gripper_command)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping AVP pose publisher.")
    finally:
        source.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
