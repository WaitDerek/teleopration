from __future__ import annotations

import argparse
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np

from teleop_ur10e_rtde.driver import RtdeServoConfig, UR10eRtdeDriver


@dataclass
class LatestTarget:
    position: Optional[np.ndarray] = None
    orientation_wxyz: Optional[np.ndarray] = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Control a UR10e from teleoperation PoseStamped targets.")
    parser.add_argument("--robot-ip", required=True, help="UR10e controller IP address.")
    parser.add_argument("--topic", default="Target_Pose", help="ROS2 geometry_msgs/PoseStamped target topic.")
    parser.add_argument("--speed", type=float, default=0.05, help="RTDE servoL speed.")
    parser.add_argument("--acceleration", type=float, default=0.5, help="RTDE servoL acceleration.")
    parser.add_argument("--dt", type=float, default=0.02, help="RTDE servo period.")
    parser.add_argument("--lookahead-time", type=float, default=0.15, help="RTDE servoL lookahead time.")
    parser.add_argument("--gain", type=float, default=1000.0, help="RTDE servoL gain.")
    parser.add_argument("--max-position-delta", type=float, default=0.35, help="Safety limit for relative translation norm.")
    return parser.parse_args()


def _require_ros2():
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
    except ImportError as exc:
        raise RuntimeError("ROS2 Python packages are required. Source your ROS2 workspace before running this backend.") from exc
    return rclpy, PoseStamped


def main() -> None:
    args = parse_args()
    rclpy, PoseStamped = _require_ros2()
    from rclpy.node import Node

    target = LatestTarget()
    lock = threading.Lock()

    class UR10eTeleopNode(Node):
        def __init__(self) -> None:
            super().__init__("teleop_ur10e_rtde")
            self.create_subscription(PoseStamped, args.topic, self._on_pose, 10)

        def _on_pose(self, msg) -> None:
            position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
            orientation = np.array(
                [
                    msg.pose.orientation.w,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                ],
                dtype=float,
            )
            if np.linalg.norm(position) > args.max_position_delta:
                self.get_logger().warn(
                    f"Ignoring target outside max position delta: {np.linalg.norm(position):.3f} m"
                )
                return
            with lock:
                target.position = position
                target.orientation_wxyz = orientation

    servo = RtdeServoConfig(
        speed=args.speed,
        acceleration=args.acceleration,
        dt=args.dt,
        lookahead_time=args.lookahead_time,
        gain=args.gain,
    )
    driver = UR10eRtdeDriver(args.robot_ip, servo)
    driver.connect()
    driver.enable()

    rclpy.init()
    node = UR10eTeleopNode()
    node.get_logger().info(f"UR10e RTDE backend subscribed to {args.topic}")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=args.dt)
            with lock:
                position = None if target.position is None else target.position.copy()
                orientation = None if target.orientation_wxyz is None else target.orientation_wxyz.copy()
            if position is not None and orientation is not None:
                driver.send_relative_pose(position, orientation)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping UR10e RTDE backend.")
    finally:
        driver.emergency_stop()
        driver.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
