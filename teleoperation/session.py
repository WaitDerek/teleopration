from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from teleoperation.preprocessing.transforms import matrix_to_pose, relative_avp_hand_pose
from teleoperation.types import GripperCommand, Pose, StreamState


@dataclass
class TeleopSession:
    stale_after_sec: float = 0.5
    gripper_open_position: float = 0.0
    gripper_closed_position: float = 1.0
    gripper_effort: float = 0.5
    position_scale: float = 1.0
    max_position_norm: Optional[float] = None
    calibration_delay_samples: int = 0
    _initial_right_hand: Optional[np.ndarray] = None
    _valid_hand_samples: int = 0

    def reset_calibration(self) -> None:
        self._initial_right_hand = None
        self._valid_hand_samples = 0

    def target_from_source(self, source) -> Optional[Pose]:
        if source.state(self.stale_after_sec) not in (StreamState.WAITING, StreamState.STREAMING):
            return None
        right_hand = source.latest_right_hand_matrix()
        if right_hand is None:
            return None
        if self._initial_right_hand is None:
            self._valid_hand_samples += 1
            if self._valid_hand_samples <= self.calibration_delay_samples:
                return None
            self._initial_right_hand = right_hand.copy()
        target_matrix = relative_avp_hand_pose(self._initial_right_hand, right_hand)
        pose = matrix_to_pose(target_matrix, timestamp_sec=time.time())
        if self.position_scale != 1.0:
            pose = Pose(
                position=pose.position * self.position_scale,
                orientation_wxyz=pose.orientation_wxyz,
                timestamp_sec=pose.timestamp_sec,
            )
        if self.max_position_norm is not None and np.linalg.norm(pose.position) > self.max_position_norm:
            return None
        return pose

    def gripper_from_source(self, source) -> Optional[GripperCommand]:
        if source.state(self.stale_after_sec) != StreamState.STREAMING:
            return None
        position = self.gripper_closed_position if source.right_pinch else self.gripper_open_position
        return GripperCommand(position=position, max_effort=self.gripper_effort)
