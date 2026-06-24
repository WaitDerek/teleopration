from __future__ import annotations

import argparse
import time
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np

from teleop_ur10e_rtde.driver import RobotiqGripperConfig, RtdeServoConfig, UR10eRtdeDriver
from teleop_ur10e_rtde.math_utils import normalize_quat_wxyz, quat_angle_rad_wxyz


@dataclass
class LatestTarget:
    position: Optional[np.ndarray] = None
    orientation_wxyz: Optional[np.ndarray] = None
    pose_time: float = 0.0
    gripper_position: Optional[float] = None
    gripper_time: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Control a UR10e from teleoperation PoseStamped targets.")
    parser.add_argument("--robot-ip", required=True, help="UR10e controller IP address.")
    parser.add_argument("--topic", default="Target_Pose", help="ROS2 geometry_msgs/PoseStamped target topic.")
    parser.add_argument(
        "--gripper-topic",
        default="Gripper_Command",
        help="ROS2 std_msgs/Float32 gripper closure topic, 0.0 open and 1.0 closed.",
    )
    parser.add_argument("--speed", type=float, default=0.05, help="RTDE servoL speed.")
    parser.add_argument("--acceleration", type=float, default=0.5, help="RTDE servoL acceleration.")
    parser.add_argument("--dt", type=float, default=0.02, help="RTDE servo period.")
    parser.add_argument("--lookahead-time", type=float, default=0.15, help="RTDE servoL lookahead time.")
    parser.add_argument("--gain", type=float, default=1000.0, help="RTDE servoL gain.")
    parser.add_argument("--max-position-delta", type=float, default=0.35, help="Safety limit for relative translation norm.")
    parser.add_argument("--max-angular-delta", type=float, default=1.2, help="Safety limit for relative rotation in radians.")
    parser.add_argument(
        "--max-target-step",
        type=float,
        default=0.015,
        help="Reject targets that jump this many meters from the last accepted target. Use <=0 to disable.",
    )
    parser.add_argument(
        "--max-target-speed",
        type=float,
        default=0.20,
        help="Reject targets whose relative translation changes faster than this many m/s. Use <=0 to disable.",
    )
    parser.add_argument(
        "--max-angular-step",
        type=float,
        default=0.10,
        help="Reject targets that jump this many radians from the last accepted orientation. Use <=0 to disable.",
    )
    parser.add_argument(
        "--max-angular-speed",
        type=float,
        default=1.0,
        help="Reject targets whose relative orientation changes faster than this many rad/s. Use <=0 to disable.",
    )
    parser.add_argument("--stale-after", type=float, default=0.25, help="Stop servoL after this many seconds without target poses.")
    parser.add_argument("--skip-robot-safety-check", action="store_true", help="Do not call RTDE isPoseWithinSafetyLimits.")
    parser.add_argument("--enable-gripper", action="store_true", help="Connect to and command a Robotiq 2F85 gripper.")
    parser.add_argument("--gripper-host", default=None, help="Robotiq socket host. Defaults to --robot-ip.")
    parser.add_argument("--gripper-port", type=int, default=63352, help="Robotiq URCap socket port.")
    parser.add_argument("--gripper-speed", type=float, default=0.5, help="Normalized Robotiq gripper speed.")
    parser.add_argument("--gripper-force", type=float, default=0.5, help="Normalized Robotiq gripper force.")
    parser.add_argument("--gripper-deadband", type=float, default=0.05, help="Minimum closure change before sending.")
    parser.add_argument("--gripper-min-interval", type=float, default=0.1, help="Minimum seconds between gripper sends.")
    parser.add_argument("--no-gripper-activate", action="store_true", help="Do not activate the gripper on connect.")
    return parser.parse_args()


def quat_distance_rad_wxyz(a: np.ndarray, b: np.ndarray) -> float:
    qa = normalize_quat_wxyz(a)
    qb = normalize_quat_wxyz(b)
    return float(2.0 * np.arccos(np.clip(abs(float(np.dot(qa, qb))), -1.0, 1.0)))


def _require_ros2():
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Float32
    except ImportError as exc:
        raise RuntimeError("ROS2 Python packages are required. Source your ROS2 workspace before running this backend.") from exc
    return rclpy, PoseStamped, Float32


def main() -> None:
    args = parse_args()
    if args.speed <= 0:
        raise ValueError("--speed must be positive")
    if args.acceleration <= 0:
        raise ValueError("--acceleration must be positive")
    if args.dt <= 0:
        raise ValueError("--dt must be positive")
    if args.max_position_delta <= 0:
        raise ValueError("--max-position-delta must be positive")
    if args.max_angular_delta <= 0:
        raise ValueError("--max-angular-delta must be positive")
    if args.stale_after <= 0:
        raise ValueError("--stale-after must be positive")
    rclpy, PoseStamped, Float32 = _require_ros2()
    from rclpy.node import Node

    target = LatestTarget()
    lock = threading.Lock()

    class UR10eTeleopNode(Node):
        def __init__(self) -> None:
            super().__init__("teleop_ur10e_rtde")
            self.create_subscription(PoseStamped, args.topic, self._on_pose, 10)
            if args.enable_gripper:
                self.create_subscription(Float32, args.gripper_topic, self._on_gripper, 10)

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
            position_norm = np.linalg.norm(position)
            if position_norm > args.max_position_delta:
                self.get_logger().warn(
                    f"Ignoring target outside max position delta: {position_norm:.3f} m"
                )
                return
            angular_delta = quat_angle_rad_wxyz(orientation)
            if angular_delta > args.max_angular_delta:
                self.get_logger().warn(
                    f"Ignoring target outside max angular delta: {angular_delta:.3f} rad"
                )
                return
            with lock:
                target.position = position
                target.orientation_wxyz = orientation
                target.pose_time = time.monotonic()

        def _on_gripper(self, msg) -> None:
            gripper_position = float(np.clip(msg.data, 0.0, 1.0))
            with lock:
                target.gripper_position = gripper_position
                target.gripper_time = time.monotonic()

    servo = RtdeServoConfig(
        speed=args.speed,
        acceleration=args.acceleration,
        dt=args.dt,
        lookahead_time=args.lookahead_time,
        gain=args.gain,
    )
    gripper = RobotiqGripperConfig(
        enabled=args.enable_gripper,
        host=args.gripper_host,
        port=args.gripper_port,
        speed=args.gripper_speed,
        force=args.gripper_force,
        activate_on_connect=not args.no_gripper_activate,
    )
    driver = UR10eRtdeDriver(
        args.robot_ip,
        servo,
        gripper=gripper,
        check_safety_limits=not args.skip_robot_safety_check,
    )
    node = None
    rclpy_started = False
    servo_stopped = True
    last_gripper_position: Optional[float] = None
    last_gripper_send_time = 0.0
    last_accepted_position: Optional[np.ndarray] = None
    last_accepted_orientation: Optional[np.ndarray] = None
    last_accepted_time = 0.0
    last_protective_warning_time = 0.0

    def smooth_target_check(position: np.ndarray, orientation: np.ndarray, now: float) -> tuple[bool, str]:
        nonlocal last_accepted_position, last_accepted_orientation, last_accepted_time
        if last_accepted_position is None or last_accepted_orientation is None:
            return True, "first accepted target"
        elapsed = now - last_accepted_time
        if elapsed > args.stale_after:
            last_accepted_position = None
            last_accepted_orientation = None
            return True, "fresh target after stale stop"
        dt = max(elapsed, 1e-6)
        position_step = float(np.linalg.norm(position - last_accepted_position))
        if args.max_target_step > 0 and position_step > args.max_target_step:
            return False, f"position step {position_step:.4f} m > {args.max_target_step:.4f} m"
        position_speed = position_step / dt
        if args.max_target_speed > 0 and position_speed > args.max_target_speed:
            return False, f"position speed {position_speed:.3f} m/s > {args.max_target_speed:.3f} m/s"
        angular_step = quat_distance_rad_wxyz(orientation, last_accepted_orientation)
        if args.max_angular_step > 0 and angular_step > args.max_angular_step:
            return False, f"angular step {angular_step:.3f} rad > {args.max_angular_step:.3f} rad"
        angular_speed = angular_step / dt
        if args.max_angular_speed > 0 and angular_speed > args.max_angular_speed:
            return False, f"angular speed {angular_speed:.3f} rad/s > {args.max_angular_speed:.3f} rad/s"
        return True, "smooth target"

    try:
        driver.connect()
        driver.enable()

        rclpy.init()
        rclpy_started = True
        node = UR10eTeleopNode()
        if args.enable_gripper:
            node.get_logger().info(f"UR10e RTDE backend subscribed to {args.topic} and {args.gripper_topic}")
        else:
            node.get_logger().info(f"UR10e RTDE backend subscribed to {args.topic}")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=args.dt)
            with lock:
                position = None if target.position is None else target.position.copy()
                orientation = None if target.orientation_wxyz is None else target.orientation_wxyz.copy()
                pose_time = target.pose_time
                gripper_position = target.gripper_position
            now = time.monotonic()
            if position is not None and orientation is not None and now - pose_time <= args.stale_after:
                is_smooth, reason = smooth_target_check(position, orientation, now)
                if is_smooth:
                    if driver.send_relative_pose(position, orientation):
                        last_accepted_position = position
                        last_accepted_orientation = orientation
                        last_accepted_time = now
                        servo_stopped = False
                    else:
                        driver.stop_servo()
                        servo_stopped = True
                        if node is not None and now - last_protective_warning_time >= 0.5:
                            node.get_logger().warn("Protective stop: robot safety check rejected target")
                            last_protective_warning_time = now
                else:
                    driver.stop_servo()
                    servo_stopped = True
                    if node is not None and now - last_protective_warning_time >= 0.5:
                        node.get_logger().warn(f"Protective stop: rejecting target jump ({reason})")
                        last_protective_warning_time = now
            elif not servo_stopped:
                driver.stop_servo()
                servo_stopped = True
                last_accepted_position = None
                last_accepted_orientation = None

            if args.enable_gripper and gripper_position is not None:
                should_send = last_gripper_position is None
                if last_gripper_position is not None:
                    should_send = abs(gripper_position - last_gripper_position) >= args.gripper_deadband
                should_send = should_send and now - last_gripper_send_time >= args.gripper_min_interval
                if should_send:
                    driver.send_gripper(gripper_position)
                    last_gripper_position = gripper_position
                    last_gripper_send_time = now
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Stopping UR10e RTDE backend.")
    finally:
        driver.emergency_stop()
        driver.disconnect()
        if node is not None:
            node.destroy_node()
        if rclpy_started and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
