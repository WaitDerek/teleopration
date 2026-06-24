from __future__ import annotations

import argparse
import sys

from teleop_ur10e_rtde.driver import Robotiq2F85SocketGripper


def normalized(value: float, name: str) -> float:
    value = float(value)
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be in [0.0, 1.0]")
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control a Robotiq 2F85 through the URCap socket on the UR controller."
    )
    parser.add_argument("--robot-ip", default="192.168.56.2", help="UR controller IP address.")
    parser.add_argument("--port", type=int, default=63352, help="Robotiq URCap socket port.")
    parser.add_argument("--timeout", type=float, default=2.0, help="Socket timeout in seconds.")

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("status", help="Read ACT/STA/GTO/POS/PRE/OBJ/FLT without moving.")

    activate = subparsers.add_parser("activate", help="Activate the gripper through the URCap socket.")
    activate.add_argument("--activation-timeout", type=float, default=5.0, help="Activation timeout in seconds.")

    open_parser = subparsers.add_parser("open", help="Open the gripper.")
    open_parser.add_argument("--speed", type=float, default=0.5, help="Normalized speed.")
    open_parser.add_argument("--force", type=float, default=0.5, help="Normalized force.")
    open_parser.add_argument("--activate-first", action="store_true", help="Activate before moving if needed.")

    close = subparsers.add_parser("close", help="Close the gripper.")
    close.add_argument("--speed", type=float, default=0.5, help="Normalized speed.")
    close.add_argument("--force", type=float, default=0.5, help="Normalized force.")
    close.add_argument("--activate-first", action="store_true", help="Activate before moving if needed.")

    move = subparsers.add_parser("move", help="Move to a normalized closure position.")
    move.add_argument("--position", type=float, required=True, help="Normalized closure: 0.0 open, 1.0 closed.")
    move.add_argument("--speed", type=float, default=0.5, help="Normalized speed.")
    move.add_argument("--force", type=float, default=0.5, help="Normalized force.")
    move.add_argument("--activate-first", action="store_true", help="Activate before moving if needed.")
    return parser.parse_args()


def print_status(gripper: Robotiq2F85SocketGripper) -> None:
    status = gripper.status()
    print(" ".join(f"{name}={status[name]}" for name in ("ACT", "STA", "GTO", "POS", "PRE", "OBJ", "FLT")))
    print(f"activated={'true' if status['ACT'] == 1 and status['STA'] == 3 else 'false'}")
    print(f"fault_ok={'true' if status['FLT'] == 0 else 'false'}")


def maybe_activate(gripper: Robotiq2F85SocketGripper, activate_first: bool) -> None:
    status = gripper.status()
    if status["ACT"] == 1 and status["STA"] == 3:
        return
    if not activate_first:
        raise RuntimeError("gripper is not activated; run `activate` or pass `--activate-first`")
    gripper.activate()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        raise ValueError("--timeout must be positive")

    gripper = Robotiq2F85SocketGripper(args.robot_ip, port=args.port, timeout_sec=args.timeout)
    gripper.connect()
    try:
        if args.command == "status":
            print_status(gripper)
        elif args.command == "activate":
            gripper.activate(timeout_sec=args.activation_timeout)
            print_status(gripper)
        elif args.command == "open":
            maybe_activate(gripper, args.activate_first)
            gripper.move(0.0, speed=normalized(args.speed, "speed"), force=normalized(args.force, "force"))
            print_status(gripper)
        elif args.command == "close":
            maybe_activate(gripper, args.activate_first)
            gripper.move(1.0, speed=normalized(args.speed, "speed"), force=normalized(args.force, "force"))
            print_status(gripper)
        elif args.command == "move":
            maybe_activate(gripper, args.activate_first)
            gripper.move(
                normalized(args.position, "position"),
                speed=normalized(args.speed, "speed"),
                force=normalized(args.force, "force"),
            )
            print_status(gripper)
        else:
            raise ValueError(f"unknown command: {args.command}")
    finally:
        gripper.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
