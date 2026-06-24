from __future__ import annotations

import argparse
import math
import time
from typing import Optional

import numpy as np

from teleoperation.ros2.pose_publisher import _require_ros2, make_pose_publisher_node
from teleoperation.types import GripperCommand, Pose


def axis_angle_quat_wxyz(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm <= 0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    axis = axis / norm
    half = angle_rad / 2.0
    return np.array([math.cos(half), *(axis * math.sin(half))], dtype=float)


def make_pose(
    elapsed_sec: float,
    pattern: str,
    amplitude_m: float,
    angular_amplitude_rad: float,
    period_sec: float,
) -> Pose:
    phase = 2.0 * math.pi * elapsed_sec / period_sec if period_sec > 0 else 0.0
    position = np.zeros(3, dtype=float)
    if pattern == "hold":
        pass
    elif pattern == "line-x":
        position[0] = amplitude_m * math.sin(phase)
    elif pattern == "line-z":
        position[2] = amplitude_m * math.sin(phase)
    elif pattern == "circle-xy":
        position[0] = amplitude_m * math.sin(phase)
        position[1] = amplitude_m * (math.cos(phase) - 1.0)
    else:
        raise ValueError(f"Unknown pattern: {pattern}")

    orientation = axis_angle_quat_wxyz(np.array([0.0, 0.0, 1.0]), angular_amplitude_rad * math.sin(phase))
    return Pose(position=position, orientation_wxyz=orientation, timestamp_sec=time.time())


def gripper_position(
    elapsed_sec: float,
    toggle: bool,
    toggle_period_sec: float,
    open_position: float,
    closed_position: float,
) -> float:
    if not toggle:
        return open_position
    step = int(elapsed_sec / max(toggle_period_sec, 1e-6))
    return closed_position if step % 2 else open_position


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish deterministic ROS2 messages that mimic Vision Pro teleoperation output."
    )
    parser.add_argument("--topic", default="Target_Pose", help="ROS2 PoseStamped topic to publish.")
    parser.add_argument("--frame-id", default="base_link", help="PoseStamped frame_id.")
    parser.add_argument(
        "--gripper-topic",
        default="Gripper_Command",
        help="ROS2 std_msgs/Float32 topic for normalized gripper closure, 0.0 open and 1.0 closed.",
    )
    parser.add_argument("--no-gripper", action="store_true", help="Do not publish simulated gripper commands.")
    parser.add_argument(
        "--pattern",
        choices=("hold", "line-x", "line-z", "circle-xy"),
        default="line-x",
        help="Relative delta-pose motion pattern.",
    )
    parser.add_argument("--amplitude", type=float, default=0.02, help="Translation amplitude in meters.")
    parser.add_argument("--angular-amplitude", type=float, default=0.0, help="Yaw rotation amplitude in radians.")
    parser.add_argument("--period", type=float, default=6.0, help="Motion period in seconds.")
    parser.add_argument("--duration", type=float, default=20.0, help="Run duration in seconds. Use <=0 to run forever.")
    parser.add_argument("--rate", type=float, default=50.0, help="Publish rate in Hz.")
    parser.add_argument("--toggle-gripper", action="store_true", help="Alternate gripper open/closed commands.")
    parser.add_argument("--gripper-period", type=float, default=3.0, help="Seconds between gripper state toggles.")
    parser.add_argument("--gripper-open", type=float, default=0.0, help="Open gripper closure value.")
    parser.add_argument("--gripper-closed", type=float, default=1.0, help="Closed gripper closure value.")
    parser.add_argument("--print-every", type=float, default=1.0, help="Status print interval in seconds. Use 0 to disable.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be positive")
    if args.amplitude < 0:
        raise ValueError("--amplitude must be non-negative")
    if args.angular_amplitude < 0:
        raise ValueError("--angular-amplitude must be non-negative")

    rclpy, _, _ = _require_ros2()
    rclpy.init()
    gripper_topic: Optional[str] = None if args.no_gripper else args.gripper_topic
    node_cls = make_pose_publisher_node(
        topic_name=args.topic,
        frame_id=args.frame_id,
        gripper_topic_name=gripper_topic,
    )
    node = node_cls()
    period = 1.0 / args.rate
    start = time.monotonic()
    last_print = start
    count = 0

    node.get_logger().info(
        f"Publishing simulated AVP output on {args.topic}"
        + ("" if gripper_topic is None else f" and {gripper_topic}")
    )
    try:
        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - start
            if args.duration > 0 and elapsed >= args.duration:
                break

            pose = make_pose(
                elapsed_sec=elapsed,
                pattern=args.pattern,
                amplitude_m=args.amplitude,
                angular_amplitude_rad=args.angular_amplitude,
                period_sec=args.period,
            )
            node.publish_target(pose)
            gripper = None
            if gripper_topic is not None:
                gripper = GripperCommand(
                    position=gripper_position(
                        elapsed,
                        args.toggle_gripper,
                        args.gripper_period,
                        args.gripper_open,
                        args.gripper_closed,
                    )
                )
                node.publish_gripper(gripper)

            if args.print_every > 0 and now - last_print >= args.print_every:
                gripper_text = "disabled" if gripper is None else f"{gripper.position:.2f}"
                print(
                    f"t={elapsed:6.2f}s pose_delta={pose.position.round(4).tolist()} "
                    f"quat_wxyz={pose.orientation_wxyz.round(4).tolist()} gripper={gripper_text}"
                )
                last_print = now
            rclpy.spin_once(node, timeout_sec=0.0)
            count += 1
            time.sleep(period)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping simulated AVP output publisher.")
    finally:
        print(f"published_samples={count}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
