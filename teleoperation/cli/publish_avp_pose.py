from __future__ import annotations

import argparse
import time

from teleoperation.avp import OpenTeleVision
from teleoperation.ros2.pose_publisher import _require_ros2, make_pose_publisher_node
from teleoperation.session import TeleopSession
from teleoperation.types import StreamState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish AVP right-hand teleoperation target poses to ROS2.")
    parser.add_argument("--topic", default="Target_Pose", help="ROS2 PoseStamped topic to publish.")
    parser.add_argument("--frame-id", default="base_link", help="PoseStamped frame_id.")
    parser.add_argument("--cert", default="./cert.pem", help="TLS certificate for Vuer.")
    parser.add_argument("--key", default="./key.pem", help="TLS key for Vuer.")
    parser.add_argument("--ngrok", action="store_true", help="Run Vuer without local TLS cert/key.")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish loop rate in Hz.")
    parser.add_argument("--stale-after", type=float, default=0.5, help="Stop publishing after this many seconds without AVP events.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy, _, _ = _require_ros2()
    rclpy.init()
    node_cls = make_pose_publisher_node(topic_name=args.topic, frame_id=args.frame_id)
    node = node_cls()
    source = OpenTeleVision(cert_file=args.cert, key_file=args.key, ngrok=args.ngrok)
    session = TeleopSession(stale_after_sec=args.stale_after)
    period = 1.0 / args.rate

    source.start()
    node.get_logger().info(f"Publishing AVP teleoperation target poses on {args.topic}")
    try:
        while rclpy.ok():
            pose = session.target_from_source(source)
            state = source.state(args.stale_after)
            if pose is not None and state == StreamState.STREAMING:
                node.publish_target(pose)
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
