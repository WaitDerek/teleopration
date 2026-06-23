from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from teleoperation.preprocessing.transforms import matrix_to_pose, relative_avp_hand_pose
from teleoperation.types import Pose, StreamState


@dataclass
class TeleopSession:
    stale_after_sec: float = 0.5
    _initial_right_hand: Optional[np.ndarray] = None

    def reset_calibration(self) -> None:
        self._initial_right_hand = None

    def target_from_source(self, source) -> Optional[Pose]:
        if source.state(self.stale_after_sec) not in (StreamState.WAITING, StreamState.STREAMING):
            return None
        right_hand = source.latest_right_hand_matrix()
        if right_hand is None:
            return None
        if self._initial_right_hand is None:
            self._initial_right_hand = right_hand.copy()
        target_matrix = relative_avp_hand_pose(self._initial_right_hand, right_hand)
        return matrix_to_pose(target_matrix, timestamp_sec=time.time())
