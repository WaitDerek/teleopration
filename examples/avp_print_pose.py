#!/usr/bin/env python3
import argparse
import time

from teleoperation.avp import OpenTeleVision
from teleoperation.session import TeleopSession


def main() -> None:
    parser = argparse.ArgumentParser(description="Print AVP right-hand target poses without ROS2.")
    parser.add_argument("--cert", default="./cert.pem")
    parser.add_argument("--key", default="./key.pem")
    parser.add_argument("--ngrok", action="store_true")
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args()

    source = OpenTeleVision(cert_file=args.cert, key_file=args.key, ngrok=args.ngrok)
    session = TeleopSession()
    source.start()
    try:
        while True:
            pose = session.target_from_source(source)
            if pose is not None:
                print(f"position={pose.position} orientation_wxyz={pose.orientation_wxyz}")
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        source.stop()


if __name__ == "__main__":
    main()
