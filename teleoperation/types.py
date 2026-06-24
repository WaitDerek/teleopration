from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    orientation_wxyz: np.ndarray
    timestamp_sec: float = 0.0

    def __post_init__(self) -> None:
        position = np.asarray(self.position, dtype=float)
        orientation = np.asarray(self.orientation_wxyz, dtype=float)
        if position.shape != (3,):
            raise ValueError(f"position must have shape (3,), got {position.shape}")
        if orientation.shape != (4,):
            raise ValueError(f"orientation_wxyz must have shape (4,), got {orientation.shape}")
        norm = np.linalg.norm(orientation)
        if norm <= 0:
            raise ValueError("orientation_wxyz must be non-zero")
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "orientation_wxyz", orientation / norm)


class StreamState(Enum):
    DISCONNECTED = "disconnected"
    WAITING = "waiting"
    STREAMING = "streaming"
    STALE = "stale"
    ERROR = "error"


@dataclass(frozen=True)
class MotionOptions:
    command_period_sec: float = 0.02
    max_linear_speed_mps: float = 0.25
    max_angular_speed_radps: float = 0.5


@dataclass(frozen=True)
class GripperCommand:
    """Normalized gripper command.

    position is a closed fraction: 0.0 is fully open, 1.0 is fully closed.
    max_effort is normalized to the selected gripper backend.
    """

    position: float
    max_effort: float = 0.5

    def __post_init__(self) -> None:
        position = float(self.position)
        max_effort = float(self.max_effort)
        if not np.isfinite(position):
            raise ValueError("position must be finite")
        if not np.isfinite(max_effort):
            raise ValueError("max_effort must be finite")
        object.__setattr__(self, "position", min(max(position, 0.0), 1.0))
        object.__setattr__(self, "max_effort", min(max(max_effort, 0.0), 1.0))


class RobotDriver(Protocol):
    def connect(self) -> None:
        ...

    def disconnect(self) -> None:
        ...

    def enable(self) -> None:
        ...

    def send_cartesian_pose(self, target: Pose, options: MotionOptions) -> None:
        ...

    def send_gripper(self, command: GripperCommand) -> None:
        ...

    def emergency_stop(self) -> None:
        ...
