from __future__ import annotations

import threading
from typing import Any

from .types import RobotPose


def extract_named_pose(message: Any, rigid_body_name: str) -> RobotPose:
    timestamp_s = _stamp_to_seconds(message.header.stamp)
    for named_pose in message.poses:
        if named_pose.name == rigid_body_name:
            pose = named_pose.pose
            return RobotPose(
                timestamp_s=timestamp_s,
                position_m=[pose.position.x, pose.position.y, pose.position.z],
                quaternion_xyzw=[
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                name=rigid_body_name,
            )
    raise KeyError(f"rigid body {rigid_body_name!r} not found in NamedPoseArray")


def extract_tf_pose(message: Any, child_frame_id: str, name: str | None = None) -> RobotPose:
    for transform_stamped in message.transforms:
        if transform_stamped.child_frame_id == child_frame_id:
            transform = transform_stamped.transform
            return RobotPose(
                timestamp_s=_stamp_to_seconds(transform_stamped.header.stamp),
                position_m=[
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ],
                quaternion_xyzw=[
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ],
                name=name or child_frame_id,
            )
    raise KeyError(f"child frame {child_frame_id!r} not found in TFMessage")


def _stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class HopeRosNamedPoseReceiver:
    def __init__(
        self,
        rigid_body_name: str = "P1",
        topic: str = "/motion_capture_tracking/poses",
        node_name: str = "hope_optitrack_robot_receiver",
    ) -> None:
        try:
            import rclpy
            from motion_capture_tracking_interfaces.msg import NamedPoseArray
        except ImportError as exc:
            raise RuntimeError("ROS receiver requires rclpy and motion_capture_tracking_interfaces") from exc

        self._rclpy = rclpy
        self._rigid_body_name = rigid_body_name
        self._latest: RobotPose | None = None
        self._lock = threading.Lock()
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(node_name)
        self._subscription = self._node.create_subscription(
            NamedPoseArray,
            topic,
            self._on_named_pose_array,
            10,
        )

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def latest_robot_pose(self) -> RobotPose | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._node.destroy_node()

    def _on_named_pose_array(self, message: Any) -> None:
        try:
            robot = extract_named_pose(message, self._rigid_body_name)
        except KeyError:
            return
        with self._lock:
            self._latest = robot
