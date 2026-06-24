from __future__ import annotations

import socket
import time
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


@dataclass(frozen=True)
class RobotiqGripperConfig:
    enabled: bool = False
    host: Optional[str] = None
    port: int = 63352
    speed: float = 0.5
    force: float = 0.5
    socket_timeout_sec: float = 2.0
    activation_timeout_sec: float = 5.0
    activate_on_connect: bool = True


class Robotiq2F85SocketGripper:
    """Small Robotiq URCap socket client.

    The Robotiq gripper URCap exposes a string-command socket on port 63352.
    Commands use device units: POS=0 is open and POS=255 is closed.
    """

    def __init__(self, host: str, port: int = 63352, timeout_sec: float = 2.0) -> None:
        self.host = host
        self.port = port
        self.timeout_sec = timeout_sec
        self._socket: Optional[socket.socket] = None

    def connect(self) -> None:
        self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout_sec)
        self._socket.settimeout(self.timeout_sec)

    def disconnect(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None

    def activate(self, timeout_sec: float = 5.0) -> None:
        if self._get_var("STA") == 3:
            return
        self._set_vars({"ACT": 0, "ATR": 0})
        reset_deadline = time.monotonic() + timeout_sec
        while time.monotonic() < reset_deadline:
            if self._get_var("ACT") == 0 and self._get_var("STA") == 0:
                break
            self._set_vars({"ACT": 0, "ATR": 0})
            time.sleep(0.1)
        else:
            raise RuntimeError("Timed out waiting for Robotiq gripper reset")
        self._set_var("ACT", 1)
        time.sleep(1.0)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                if self._get_var("ACT") == 1 and self._get_var("STA") == 3:
                    return
            except RuntimeError:
                pass
            time.sleep(0.1)
        raise RuntimeError("Timed out waiting for Robotiq gripper activation")

    def move(self, closed_fraction: float, speed: float = 0.5, force: float = 0.5) -> None:
        position_device = self._to_device_units(closed_fraction)
        speed_device = self._to_device_units(speed)
        force_device = self._to_device_units(force)
        self._set_vars({"POS": position_device, "SPE": speed_device, "FOR": force_device, "GTO": 1})

    def _to_device_units(self, value: float) -> int:
        if not np.isfinite(value):
            raise ValueError("gripper command values must be finite")
        return int(round(min(max(float(value), 0.0), 1.0) * 255.0))

    def _set_var(self, name: str, value: int) -> None:
        self._set_vars({name: value})

    def _set_vars(self, values: dict[str, int]) -> None:
        body = " ".join(f"{name} {int(value)}" for name, value in values.items())
        response = self._request(f"SET {body}\n")
        if response.lower() != "ack":
            raise RuntimeError(f"Robotiq gripper rejected SET {body}: {response!r}")

    def _get_var(self, name: str) -> int:
        response = self._request(f"GET {name}\n")
        parts = response.split()
        if len(parts) != 2 or parts[0] != name:
            raise RuntimeError(f"Unexpected Robotiq gripper response: {response!r}")
        return int(parts[1])

    def _request(self, command: str) -> str:
        if self._socket is None:
            raise RuntimeError("Robotiq gripper is not connected")
        self._socket.sendall(command.encode("ascii"))
        data = self._socket.recv(1024)
        if not data:
            raise RuntimeError("Robotiq gripper socket closed")
        return data.decode("ascii", errors="replace").strip()


class UR10eRtdeDriver:
    def __init__(
        self,
        robot_ip: str,
        servo: RtdeServoConfig = RtdeServoConfig(),
        gripper: RobotiqGripperConfig = RobotiqGripperConfig(),
        check_safety_limits: bool = True,
    ) -> None:
        self.robot_ip = robot_ip
        self.servo = servo
        self.gripper = gripper
        self.check_safety_limits = check_safety_limits
        self._rtde_control = None
        self._rtde_receive = None
        self._gripper: Optional[Robotiq2F85SocketGripper] = None
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

        try:
            self._rtde_receive = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self._rtde_control = rtde_control.RTDEControlInterface(self.robot_ip)
            self._initial_tcp_pose = np.asarray(self._rtde_receive.getActualTCPPose(), dtype=float)
            if self._initial_tcp_pose.shape != (6,):
                raise RuntimeError(f"Unexpected TCP pose shape from RTDE: {self._initial_tcp_pose.shape}")
            if self.gripper.enabled:
                gripper_host = self.gripper.host or self.robot_ip
                self._gripper = Robotiq2F85SocketGripper(
                    gripper_host,
                    port=self.gripper.port,
                    timeout_sec=self.gripper.socket_timeout_sec,
                )
                self._gripper.connect()
                if self.gripper.activate_on_connect:
                    self._gripper.activate(timeout_sec=self.gripper.activation_timeout_sec)
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        if self._rtde_control is not None:
            try:
                self._rtde_control.servoStop()
            except Exception:
                pass
            stop_script = getattr(self._rtde_control, "stopScript", None)
            if stop_script is not None:
                try:
                    stop_script()
                except Exception:
                    pass
            self._rtde_control.disconnect()
        if self._rtde_receive is not None:
            self._rtde_receive.disconnect()
        if self._gripper is not None:
            self._gripper.disconnect()
        self._rtde_control = None
        self._rtde_receive = None
        self._gripper = None
        self._enabled = False

    def enable(self) -> None:
        if self._rtde_control is None or self._rtde_receive is None:
            raise RuntimeError("Driver is not connected")
        self._enabled = True

    def emergency_stop(self) -> None:
        self._enabled = False
        if self._rtde_control is not None:
            self._rtde_control.servoStop()

    def stop_servo(self) -> None:
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
        if self.check_safety_limits and not self._is_target_within_safety_limits(target):
            return
        self._rtde_control.servoL(
            target,
            self.servo.speed,
            self.servo.acceleration,
            self.servo.dt,
            self.servo.lookahead_time,
            self.servo.gain,
        )

    def send_gripper(self, closed_fraction: float) -> None:
        if not self.gripper.enabled or self._gripper is None:
            return
        self._gripper.move(closed_fraction, speed=self.gripper.speed, force=self.gripper.force)

    def _is_target_within_safety_limits(self, target: list[float]) -> bool:
        if self._rtde_control is None:
            raise RuntimeError("Driver is not connected")
        checker = getattr(self._rtde_control, "isPoseWithinSafetyLimits", None)
        if checker is None:
            return True
        return bool(checker(target))
