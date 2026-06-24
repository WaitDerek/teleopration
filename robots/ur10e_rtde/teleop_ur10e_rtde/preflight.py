from __future__ import annotations

import argparse
import socket
import sys
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class CheckResult:
    name: str
    ok: bool
    detail: str
    warning: bool = False


@dataclass(frozen=True)
class DashboardState:
    banner: str
    robotmode: str
    safetymode: str
    program_state: str
    running: Optional[bool]
    remote: Optional[bool]


@dataclass(frozen=True)
class GripperState:
    values: dict[str, int]

    @property
    def activated(self) -> bool:
        return self.values.get("ACT") == 1 and self.values.get("STA") == 3

    @property
    def fault_ok(self) -> bool:
        return self.values.get("FLT", -1) == 0

    @property
    def ready(self) -> bool:
        return self.activated and self.fault_ok

    def detail(self) -> str:
        order = ("ACT", "STA", "GTO", "POS", "PRE", "OBJ", "FLT")
        return " ".join(f"{name}={self.values.get(name, 'missing')}" for name in order)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check whether the UR10e is ready for RTDE teleoperation. "
            "No target pose or gripper motion is sent."
        )
    )
    parser.add_argument("--robot-ip", default="192.168.56.2", help="UR controller IP address.")
    parser.add_argument("--dashboard-port", type=int, default=29999, help="UR Dashboard server port.")
    parser.add_argument("--gripper-port", type=int, default=63352, help="Robotiq URCap socket port.")
    parser.add_argument("--timeout", type=float, default=2.0, help="Socket/connect timeout in seconds.")
    parser.add_argument("--allow-running", action="store_true", help="Do not fail when a program is already running.")
    parser.add_argument("--allow-local", action="store_true", help="Do not fail when the robot is not in Remote Control.")
    parser.add_argument("--skip-rtde", action="store_true", help="Skip RTDEReceive readback check.")
    parser.add_argument(
        "--skip-control-check",
        action="store_true",
        help="Skip RTDEControl empty-connect check. By default this opens RTDEControl and immediately stops its script.",
    )
    parser.add_argument("--skip-gripper", action="store_true", help="Skip Robotiq gripper activation/fault check.")
    parser.add_argument("--require-gripper", action="store_true", help="Return non-zero unless gripper is also ready.")
    parser.add_argument("--check-gripper", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def dashboard_query(host: str, port: int, timeout: float, commands: list[str]) -> tuple[str, dict[str, str]]:
    responses: dict[str, str] = {}
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        banner = sock.recv(4096).decode("utf-8", errors="replace").strip()
        for command in commands:
            sock.sendall((command + "\n").encode("ascii"))
            responses[command] = sock.recv(4096).decode("utf-8", errors="replace").strip()
            time.sleep(0.03)
    return banner, responses


def value_after_colon(response: str) -> str:
    if ":" not in response:
        return response.strip()
    return response.split(":", 1)[1].strip()


def bool_response(response: str) -> Optional[bool]:
    text = value_after_colon(response).lower()
    if text == "true":
        return True
    if text == "false":
        return False
    return None


def dashboard_state(host: str, port: int, timeout: float) -> DashboardState:
    commands = ["robotmode", "safetymode", "programState", "running", "is in remote control"]
    banner, responses = dashboard_query(host, port, timeout, commands)
    return DashboardState(
        banner=banner,
        robotmode=value_after_colon(responses["robotmode"]),
        safetymode=value_after_colon(responses["safetymode"]),
        program_state=responses["programState"],
        running=bool_response(responses["running"]),
        remote=bool_response(responses["is in remote control"]),
    )


def rtde_receive_readback(host: str) -> tuple[bool, str]:
    try:
        import rtde_receive

        receiver = rtde_receive.RTDEReceiveInterface(host)
        tcp_pose = np.asarray(receiver.getActualTCPPose(), dtype=float)
        joints = np.asarray(receiver.getActualQ(), dtype=float)
        receiver.disconnect()
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"

    if tcp_pose.shape != (6,):
        return False, f"unexpected TCP pose shape {tcp_pose.shape}"
    if joints.shape != (6,):
        return False, f"unexpected joint vector shape {joints.shape}"
    tcp_text = np.array2string(tcp_pose, precision=4, separator=", ")
    joint_text = np.array2string(joints, precision=4, separator=", ")
    return True, f"tcp={tcp_text} q={joint_text}"


def rtde_control_empty_connect(host: str) -> tuple[bool, str]:
    try:
        import rtde_control

        control = rtde_control.RTDEControlInterface(host)
        try:
            control.servoStop()
        finally:
            stop_script = getattr(control, "stopScript", None)
            if stop_script is not None:
                stop_script()
            control.disconnect()
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"
    return True, "RTDEControl connected and temporary script stopped"


def parse_gripper_value(response: str, name: str) -> int:
    parts = response.strip().split()
    if len(parts) != 2 or parts[0] != name:
        raise RuntimeError(f"unexpected response for {name}: {response!r}")
    return int(parts[1], 10)


def read_gripper_state(host: str, port: int, timeout: float) -> tuple[bool, str, Optional[GripperState]]:
    names = ["ACT", "STA", "GTO", "POS", "PRE", "OBJ", "FLT"]
    values: dict[str, int] = {}
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            for name in names:
                sock.sendall((f"GET {name}\n").encode("ascii"))
                response = sock.recv(1024).decode("ascii", errors="replace").strip()
                values[name] = parse_gripper_value(response, name)
                time.sleep(0.03)
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}", None

    state = GripperState(values)
    if not state.activated:
        return False, f"not activated ({state.detail()})", state
    if not state.fault_ok:
        return False, f"fault present ({state.detail()})", state
    return True, state.detail(), state


def print_result(result: CheckResult) -> None:
    if result.ok:
        prefix = "WARN" if result.warning else "OK"
    else:
        prefix = "FAIL"
    print(f"[{prefix}] {result.name}: {result.detail}")


def bool_text(value: bool) -> str:
    return "true" if value else "false"


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        raise ValueError("--timeout must be positive")

    results: list[CheckResult] = []
    arm_ready = False
    gripper_ready = False

    try:
        dashboard = dashboard_state(args.robot_ip, args.dashboard_port, args.timeout)
    except Exception as exc:
        results.append(
            CheckResult(
                "dashboard",
                False,
                f"cannot connect to {args.robot_ip}:{args.dashboard_port} ({type(exc).__name__}: {exc})",
            )
        )
        for result in results:
            print_result(result)
        print("arm_teleop_ready=false")
        print("gripper_ready=false")
        print("teleop_without_gripper_ready=false")
        print("teleop_with_gripper_ready=false")
        return 1

    results.append(CheckResult("dashboard", True, dashboard.banner))
    robotmode_ok = dashboard.robotmode == "RUNNING"
    safetymode_ok = dashboard.safetymode == "NORMAL"
    running_ok = dashboard.running is False or args.allow_running
    remote_ok = dashboard.remote is True or args.allow_local
    dashboard_ok = robotmode_ok and safetymode_ok and running_ok and remote_ok

    results.append(CheckResult("robotmode", robotmode_ok, dashboard.robotmode))
    results.append(CheckResult("safetymode", safetymode_ok, dashboard.safetymode))
    results.append(CheckResult("programState", True, dashboard.program_state, warning="STOPPED" not in dashboard.program_state))
    results.append(
        CheckResult(
            "program running",
            running_ok,
            str(dashboard.running).lower() if dashboard.running is not None else "unknown",
            warning=bool(dashboard.running),
        )
    )
    results.append(
        CheckResult(
            "remote control",
            remote_ok,
            str(dashboard.remote).lower() if dashboard.remote is not None else "unknown",
            warning=dashboard.remote is not True,
        )
    )

    rtde_receive_ok = True
    if args.skip_rtde:
        results.append(CheckResult("rtde_receive", True, "skipped", warning=True))
    else:
        rtde_receive_ok, detail = rtde_receive_readback(args.robot_ip)
        results.append(CheckResult("rtde_receive", rtde_receive_ok, detail))

    control_ok = True
    if args.skip_control_check:
        results.append(CheckResult("rtde_control", True, "skipped", warning=True))
    elif not dashboard_ok:
        control_ok = False
        results.append(CheckResult("rtde_control", False, "skipped because dashboard prerequisites failed"))
    else:
        control_ok, detail = rtde_control_empty_connect(args.robot_ip)
        results.append(CheckResult("rtde_control", control_ok, detail))

    if args.skip_gripper:
        results.append(CheckResult("gripper", True, "skipped", warning=True))
    else:
        gripper_ready, detail, _ = read_gripper_state(args.robot_ip, args.gripper_port, args.timeout)
        results.append(CheckResult("gripper", gripper_ready, detail))

    arm_ready = dashboard_ok and rtde_receive_ok and control_ok

    for result in results:
        print_result(result)

    print(f"arm_teleop_ready={bool_text(arm_ready)}")
    print(f"gripper_ready={bool_text(gripper_ready)}")
    print(f"teleop_without_gripper_ready={bool_text(arm_ready)}")
    print(f"teleop_with_gripper_ready={bool_text(arm_ready and gripper_ready)}")

    if args.require_gripper:
        return 0 if arm_ready and gripper_ready else 1
    return 0 if arm_ready else 1


if __name__ == "__main__":
    sys.exit(main())
