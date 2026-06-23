from __future__ import annotations

from teleoperation.types import Pose


def _require_ros2():
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Header
    except ImportError as exc:
        raise RuntimeError("ROS2 Python packages are required. Source your ROS2 workspace before running this command.") from exc
    return rclpy, PoseStamped, Header


def make_pose_publisher_node(topic_name: str = "Target_Pose", frame_id: str = "base_link"):
    rclpy, PoseStamped, Header = _require_ros2()
    from rclpy.node import Node

    class TargetPosePublisher(Node):
        def __init__(self) -> None:
            super().__init__("teleoperation_target_pose_publisher")
            self.publisher_ = self.create_publisher(PoseStamped, topic_name, 10)

        def publish_target(self, pose: Pose) -> None:
            msg = PoseStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.pose.position.x = float(pose.position[0])
            msg.pose.position.y = float(pose.position[1])
            msg.pose.position.z = float(pose.position[2])
            msg.pose.orientation.w = float(pose.orientation_wxyz[0])
            msg.pose.orientation.x = float(pose.orientation_wxyz[1])
            msg.pose.orientation.y = float(pose.orientation_wxyz[2])
            msg.pose.orientation.z = float(pose.orientation_wxyz[3])
            self.publisher_.publish(msg)

    return TargetPosePublisher
