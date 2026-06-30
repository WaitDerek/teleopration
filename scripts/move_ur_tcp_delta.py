#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move UR TCP by one base-frame Cartesian delta using RTDE directly.")
    parser.add_argument("--robot-ip", default="192.168.56.2", help="UR controller IP address.")
    parser.add_argument("--x", type=float, default=0.0, help="TCP delta along UR base X, meters.")
    parser.add_argument("--y", type=float, default=0.0, help="TCP delta along UR base Y, meters.")
    parser.add_argument("--z", type=float, default=0.0, help="TCP delta along UR base Z, meters.")
    parser.add_argument("--speed", type=float, default=0.02, help="moveL tool speed, m/s.")
    parser.add_argument("--max-speed", type=float, default=0.05, help="Reject --speed above this many m/s.")
    parser.add_argument("--acceleration", type=float, default=0.1, help="moveL tool acceleration, m/s^2.")
    parser.add_argument(
        "--max-acceleration",
        type=float,
        default=0.5,
        help="Reject --acceleration above this many m/s^2.",
    )
    parser.add_argument("--max-delta", type=float, default=0.10, help="Reject delta norm above this many meters.")
    parser.add_argument(
        "--max-tcp-x",
        type=float,
        default=-0.5,
        help="Reject target TCP poses whose base-frame X is greater than this many meters. Default -0.5 means x <= -500 mm.",
    )
    parser.add_argument("--settle-sec", type=float, default=0.5, help="Delay before final TCP readback.")
    parser.add_argument("--yes", action="store_true", help="Execute without interactive confirmation.")
    parser.add_argument(
        "--skip-safety-check",
        action="store_true",
        help="Skip RTDE isPoseWithinSafetyLimits before moveL.",
    )
    return parser.parse_args()


def print_pose(label: str, pose: np.ndarray) -> None:
    print(
        f"{label}: "
        f"x={pose[0]:+.4f} y={pose[1]:+.4f} z={pose[2]:+.4f} "
        f"rx={pose[3]:+.4f} ry={pose[4]:+.4f} rz={pose[5]:+.4f}"
    )


def main() -> int:
    args = parse_args()
    if args.speed <= 0:
        raise ValueError("--speed must be positive")
    if args.max_speed <= 0:
        raise ValueError("--max-speed must be positive")
    if args.speed > args.max_speed:
        raise ValueError(f"--speed {args.speed:.3f} exceeds --max-speed {args.max_speed:.3f}")
    if args.acceleration <= 0:
        raise ValueError("--acceleration must be positive")
    if args.max_acceleration <= 0:
        raise ValueError("--max-acceleration must be positive")
    if args.acceleration > args.max_acceleration:
        raise ValueError(
            f"--acceleration {args.acceleration:.3f} exceeds --max-acceleration {args.max_acceleration:.3f}"
        )
    if args.max_delta <= 0:
        raise ValueError("--max-delta must be positive")
    if args.settle_sec < 0:
        raise ValueError("--settle-sec must be non-negative")

    delta = np.array([args.x, args.y, args.z], dtype=float)
    delta_norm = float(np.linalg.norm(delta))
    if delta_norm <= 0:
        raise ValueError("at least one of --x/--y/--z must be non-zero")
    if delta_norm > args.max_delta + 1e-12:
        raise ValueError(f"requested delta {delta_norm:.4f} m exceeds --max-delta {args.max_delta:.4f} m")

    try:
        import rtde_control
        import rtde_receive
    except ImportError as exc:
        raise RuntimeError("rtde_control and rtde_receive are required in the active environment") from exc

    receiver = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    control = rtde_control.RTDEControlInterface(args.robot_ip)
    try:
        before = np.asarray(receiver.getActualTCPPose(), dtype=float)
        if before.shape != (6,):
            raise RuntimeError(f"unexpected TCP pose shape: {before.shape}")
        target = before.copy()
        target[:3] += delta

        print_pose("before_tcp", before)
        print(
            f"requested_delta: x={delta[0]:+.4f} y={delta[1]:+.4f} z={delta[2]:+.4f} norm={delta_norm:.4f} m"
        )
        print(
            f"motion_limits: speed={args.speed:.3f} <= {args.max_speed:.3f} m/s, "
            f"acceleration={args.acceleration:.3f} <= {args.max_acceleration:.3f} m/s^2"
        )
        print_pose("target_tcp", target)
        if target[0] > args.max_tcp_x:
            print(
                f"max_tcp_x_guard=false; target x={target[0]:+.4f} m is greater than {args.max_tcp_x:+.4f} m",
                file=sys.stderr,
            )
            return 2
        print(f"max_tcp_x_guard=true x<={args.max_tcp_x:+.4f} m")

        if not args.skip_safety_check:
            checker = getattr(control, "isPoseWithinSafetyLimits", None)
            if checker is not None and not bool(checker(target.tolist())):
                print("safety_check=false; target rejected before motion", file=sys.stderr)
                return 2
            print("safety_check=true")

        if not args.yes:
            reply = input("Execute this moveL? Type YES to continue: ")
            if reply != "YES":
                print("aborted")
                return 1

        ok = control.moveL(target.tolist(), args.speed, args.acceleration)
        if ok is False:
            print("moveL returned false", file=sys.stderr)
            return 3
        time.sleep(args.settle_sec)
        after = np.asarray(receiver.getActualTCPPose(), dtype=float)
        print_pose("after_tcp", after)
        print_pose("actual_delta", after - before)
        return 0
    finally:
        try:
            control.stopScript()
        except Exception:
            pass
        control.disconnect()
        receiver.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
