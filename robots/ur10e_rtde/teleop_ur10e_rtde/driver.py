from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from teleop_ur10e_rtde.math_utils import quat_multiply_wxyz, quat_wxyz_to_rotvec, rotvec_to_quat_wxyz


@dataclass(frozen=True)
class RtdeServoConfig:
    speed: float = 0.05
    acceleration: float = 0.5
    dt: float = 0.02
    lookahead_time: float = 0.15
    gain: float = 1000.0


class UR10eRtdeDriver:
    def __init__(self, robot_ip: str, servo: RtdeServoConfig = RtdeServoConfig()) -> None:
        self.robot_ip = robot_ip
        self.servo = servo
        self._rtde_control = None
        self._rtde_receive = None
        self._initial_tcp_pose: Optional[np.ndarray] = None
        self._enabled = False

    def connect(self) -> None:
        try:
            import rtde_control
            import rtde_receive
        except ImportError as exc:
            raise RuntimeError(
                "UR RTDE Python modules are required. Install/source ur_rtde before running this backend."
            ) from exc

        self._rtde_receive = rtde_receive.RTDEReceiveInterface(self.robot_ip)
        self._rtde_control = rtde_control.RTDEControlInterface(self.robot_ip)
        self._initial_tcp_pose = np.asarray(self._rtde_receive.getActualTCPPose(), dtype=float)
        if self._initial_tcp_pose.shape != (6,):
            raise RuntimeError(f"Unexpected TCP pose shape from RTDE: {self._initial_tcp_pose.shape}")

    def disconnect(self) -> None:
        if self._rtde_control is not None:
            try:
                self._rtde_control.servoStop()
            except Exception:
                pass
            self._rtde_control.disconnect()
        if self._rtde_receive is not None:
            self._rtde_receive.disconnect()
        self._rtde_control = None
        self._rtde_receive = None
        self._enabled = False

    def enable(self) -> None:
        if self._rtde_control is None or self._rtde_receive is None:
            raise RuntimeError("Driver is not connected")
        self._enabled = True

    def emergency_stop(self) -> None:
        self._enabled = False
        if self._rtde_control is not None:
            self._rtde_control.servoStop()

    def send_relative_pose(self, position_delta: np.ndarray, orientation_delta_wxyz: np.ndarray) -> None:
        if not self._enabled:
            return
        if self._rtde_control is None or self._initial_tcp_pose is None:
            raise RuntimeError("Driver is not connected")

        delta_pos = np.asarray(position_delta, dtype=float)
        if delta_pos.shape != (3,):
            raise ValueError(f"position_delta must have shape (3,), got {delta_pos.shape}")

        initial_pos = self._initial_tcp_pose[:3]
        initial_quat = rotvec_to_quat_wxyz(self._initial_tcp_pose[3:])
        target_pos = initial_pos + delta_pos
        target_quat = quat_multiply_wxyz(orientation_delta_wxyz, initial_quat)
        target_rotvec = quat_wxyz_to_rotvec(target_quat)
        target = [
            float(target_pos[0]),
            float(target_pos[1]),
            float(target_pos[2]),
            float(target_rotvec[0]),
            float(target_rotvec[1]),
            float(target_rotvec[2]),
        ]
        self._rtde_control.servoL(
            target,
            self.servo.speed,
            self.servo.acceleration,
            self.servo.dt,
            self.servo.lookahead_time,
            self.servo.gain,
        )
