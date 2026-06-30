from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from teleoperation.preprocessing.transforms import (
    apply_frame_rotation,
    matrix_to_pose,
    quat_angle_rad_wxyz,
    relative_avp_hand_pose,
    rotation_matrix_from_euler_deg,
    scale_quat_angle_wxyz,
)
from teleoperation.types import GripperCommand, Pose, StreamState


@dataclass
class TeleopSession:
    stale_after_sec: float = 0.5
    gripper_open_position: float = 0.0
    gripper_closed_position: float = 1.0
    gripper_effort: float = 0.5
    position_scale: float = 1.0
    max_position_norm: Optional[float] = None
    orientation_scale: float = 1.0
    max_angular_norm: Optional[float] = None
    calibration_delay_samples: int = 0
    align_wrist_to_base: bool = False
    frame_roll_deg: float = 0.0
    frame_pitch_deg: float = 0.0
    frame_yaw_deg: float = 0.0
    frame_rotation_matrix: Optional[np.ndarray] = None
    output_axis_sign: np.ndarray = field(default_factory=lambda: np.ones(3, dtype=float))
    _previous_right_hand: Optional[np.ndarray] = None
    _valid_hand_samples: int = 0

    def reset_calibration(self) -> None:
        self._previous_right_hand = None
        self._valid_hand_samples = 0

    def target_from_source(self, source) -> Optional[Pose]:
        if source.state(self.stale_after_sec) not in (StreamState.WAITING, StreamState.STREAMING):
            self._previous_right_hand = None
            return None
        right_hand = source.latest_right_hand_matrix()
        if right_hand is None:
            return None
        if self._previous_right_hand is None:
            self._valid_hand_samples += 1
            if self._valid_hand_samples <= self.calibration_delay_samples:
                return None
            self._previous_right_hand = right_hand.copy()
            return None
        frame_rotation = self.frame_rotation_matrix
        if frame_rotation is None and (
            self.frame_roll_deg != 0.0 or self.frame_pitch_deg != 0.0 or self.frame_yaw_deg != 0.0
        ):
            frame_rotation = rotation_matrix_from_euler_deg(
                roll_deg=self.frame_roll_deg,
                pitch_deg=self.frame_pitch_deg,
                yaw_deg=self.frame_yaw_deg,
            )
        target_matrix = relative_avp_hand_pose(
            self._previous_right_hand,
            right_hand,
            align_wrist_to_base=self.align_wrist_to_base,
            frame_rotation=frame_rotation,
        )
        output_axis_sign = np.asarray(self.output_axis_sign, dtype=float)
        if not np.allclose(output_axis_sign, np.ones(3, dtype=float)):
            output_rotation = np.diag(output_axis_sign)
            if np.linalg.det(output_rotation) <= 0:
                raise ValueError("output_axis_sign must preserve handedness when orientation is enabled")
            target_matrix = apply_frame_rotation(target_matrix, output_rotation)
        self._previous_right_hand = right_hand.copy()
        pose = matrix_to_pose(target_matrix, timestamp_sec=time.time())
        orientation = pose.orientation_wxyz
        if self.orientation_scale != 1.0:
            orientation = scale_quat_angle_wxyz(orientation, self.orientation_scale)
        if self.position_scale != 1.0 or self.orientation_scale != 1.0:
            pose = Pose(
                position=pose.position * self.position_scale,
                orientation_wxyz=orientation,
                timestamp_sec=pose.timestamp_sec,
            )
        if self.max_position_norm is not None and np.linalg.norm(pose.position) > self.max_position_norm:
            return None
        if self.max_angular_norm is not None and quat_angle_rad_wxyz(pose.orientation_wxyz) > self.max_angular_norm:
            return None
        return pose

    def gripper_from_source(self, source) -> Optional[GripperCommand]:
        if source.state(self.stale_after_sec) != StreamState.STREAMING:
            return None
        position = self.gripper_closed_position if source.right_pinch else self.gripper_open_position
        return GripperCommand(position=position, max_effort=self.gripper_effort)
