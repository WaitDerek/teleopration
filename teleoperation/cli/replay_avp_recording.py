from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from teleoperation.ros2.pose_publisher import _require_ros2, make_pose_publisher_node
from teleoperation.types import GripperCommand, Pose


PositionMapper = Callable[[np.ndarray], np.ndarray]


def parse_position_map(spec: str) -> PositionMapper:
    tokens = [token.strip() for token in spec.split(",")]
    if len(tokens) != 3:
        raise ValueError("--position-map must contain exactly three comma-separated axes")
    axes = {"x": 0, "y": 1, "z": 2}
    parsed: list[tuple[int, float]] = []
    for token in tokens:
        sign = -1.0 if token.startswith("-") else 1.0
        axis_name = token[1:] if token.startswith("-") else token
        if axis_name not in axes:
            raise ValueError(f"Invalid axis in --position-map: {token!r}")
        parsed.append((axes[axis_name], sign))

    def mapper(position: np.ndarray) -> np.ndarray:
        position = np.asarray(position, dtype=float)
        return np.array([sign * position[index] for index, sign in parsed], dtype=float)

    return mapper


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay recorded AVP teleoperation NPZ data to ROS2 topics.")
    parser.add_argument("recording", help="Path to .npz file produced by teleop-record-avp-data.")
    parser.add_argument("--topic", default="Target_Pose", help="ROS2 PoseStamped topic to publish.")
    parser.add_argument("--frame-id", default="base_link", help="PoseStamped frame_id.")
    parser.add_argument(
        "--gripper-topic",
        default="Gripper_Command",
        help="ROS2 std_msgs/Float32 gripper topic.",
    )
    parser.add_argument("--no-gripper", action="store_true", help="Do not replay gripper commands.")
    parser.add_argument(
        "--position-map",
        default="x,y,z",
        help="Axis remap for recorded position, e.g. x,y,z or x,-z,y.",
    )
    parser.add_argument("--position-scale", type=float, default=1.0, help="Additional replay-time position scale.")
    parser.add_argument(
        "--max-position-norm",
        type=float,
        default=0.10,
        help="Drop replay poses above this many meters after mapping/scaling. Use <=0 to disable.",
    )
    parser.add_argument(
        "--keep-orientation",
        action="store_true",
        help="Replay recorded orientation deltas. By default orientation is identity for safer first robot tests.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=0.0,
        help="Fixed replay publish rate in Hz. Use <=0 to preserve recorded timing.",
    )
    parser.add_argument("--loop", action="store_true", help="Replay the recording repeatedly until interrupted.")
    parser.add_argument("--print-every", type=float, default=1.0, help="Status print interval. Use 0 to disable.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.position_scale <= 0:
        raise ValueError("--position-scale must be positive")
    if args.rate < 0:
        raise ValueError("--rate must be non-negative")

    recording_path = Path(args.recording)
    data = np.load(recording_path, allow_pickle=False)
    time_sec = np.asarray(data["time_sec"], dtype=float)
    positions = np.asarray(data["pose_position"], dtype=float)
    orientations = np.asarray(data["pose_orientation_wxyz"], dtype=float)
    gripper_positions = np.asarray(data["gripper_position"], dtype=float)
    if len(time_sec) == 0:
        raise ValueError(f"{recording_path} contains no samples")
    if positions.shape[0] != time_sec.shape[0] or orientations.shape[0] != time_sec.shape[0]:
        raise ValueError(f"{recording_path} has inconsistent pose array lengths")

    position_mapper = parse_position_map(args.position_map)
    max_position_norm: Optional[float] = None if args.max_position_norm <= 0 else args.max_position_norm
    rclpy, _, _ = _require_ros2()
    rclpy.init()
    gripper_topic = None if args.no_gripper else args.gripper_topic
    node_cls = make_pose_publisher_node(
        topic_name=args.topic,
        frame_id=args.frame_id,
        gripper_topic_name=gripper_topic,
    )
    node = node_cls()
    identity_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    last_print = time.monotonic()
    published = 0
    loops = 0

    node.get_logger().info(
        f"Replaying {recording_path} to {args.topic}"
        + ("" if gripper_topic is None else f" and {gripper_topic}")
    )
    try:
        while rclpy.ok():
            loops += 1
            replay_start = time.monotonic()
            for index, recorded_time in enumerate(time_sec):
                if args.rate > 0:
                    target_time = replay_start + index / args.rate
                else:
                    target_time = replay_start + float(recorded_time)
                while rclpy.ok():
                    now = time.monotonic()
                    if now >= target_time:
                        break
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(min(0.002, target_time - now))
                if not rclpy.ok():
                    break

                position = position_mapper(positions[index]) * args.position_scale
                if max_position_norm is not None and np.linalg.norm(position) > max_position_norm:
                    continue
                orientation = orientations[index] if args.keep_orientation else identity_quat
                pose = Pose(position=position, orientation_wxyz=orientation, timestamp_sec=time.time())
                node.publish_target(pose)

                if gripper_topic is not None and index < gripper_positions.shape[0]:
                    gripper_position = gripper_positions[index]
                    if np.isfinite(gripper_position):
                        node.publish_gripper(GripperCommand(float(gripper_position)))

                now = time.monotonic()
                if args.print_every > 0 and now - last_print >= args.print_every:
                    print(
                        f"loop={loops} sample={index + 1}/{len(time_sec)} "
                        f"pose_delta={pose.position.round(4).tolist()}"
                    )
                    last_print = now
                rclpy.spin_once(node, timeout_sec=0.0)
                published += 1
            if not args.loop:
                break
    except KeyboardInterrupt:
        node.get_logger().info("Stopping AVP recording replay.")
    finally:
        print(f"published_samples={published}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
