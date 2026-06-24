from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export recorded AVP NPZ data to a PlotJuggler-friendly CSV file.")
    parser.add_argument("recording", help="Path to .npz file produced by teleop-record-avp-data.")
    parser.add_argument("--output", default=None, help="Output .csv path. Defaults to the recording path with .csv suffix.")
    parser.add_argument(
        "--include-raw-matrices",
        action="store_true",
        help="Also export flattened raw right-hand and head matrices.",
    )
    return parser.parse_args()


def _matrix_columns(prefix: str) -> list[str]:
    return [f"{prefix}.m{row}{col}" for row in range(4) for col in range(4)]


def _flatten_matrix(matrix: np.ndarray) -> list[float]:
    return [float(matrix[row, col]) for row in range(4) for col in range(4)]


def main() -> None:
    args = parse_args()
    recording_path = Path(args.recording)
    output_path = Path(args.output) if args.output else recording_path.with_suffix(".csv")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    data = np.load(recording_path, allow_pickle=False)
    time_sec = np.asarray(data["time_sec"], dtype=float)
    positions = np.asarray(data["pose_position"], dtype=float)
    orientations = np.asarray(data["pose_orientation_wxyz"], dtype=float)
    gripper = np.asarray(data["gripper_position"], dtype=float)
    pinch = np.asarray(data["right_pinch"], dtype=bool)
    right_hand = np.asarray(data["right_hand_matrix"], dtype=float)
    head = np.asarray(data["head_matrix"], dtype=float)

    if len(time_sec) == 0:
        raise ValueError(f"{recording_path} contains no samples")

    headers = [
        "time",
        "pose.x",
        "pose.y",
        "pose.z",
        "pose.norm",
        "quat.w",
        "quat.x",
        "quat.y",
        "quat.z",
        "gripper",
        "pinch",
    ]
    if args.include_raw_matrices:
        headers.extend(_matrix_columns("right_hand"))
        headers.extend(_matrix_columns("head"))

    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(headers)
        for index, timestamp in enumerate(time_sec):
            position = positions[index]
            orientation = orientations[index]
            row = [
                float(timestamp),
                float(position[0]),
                float(position[1]),
                float(position[2]),
                float(np.linalg.norm(position)),
                float(orientation[0]),
                float(orientation[1]),
                float(orientation[2]),
                float(orientation[3]),
                float(gripper[index]) if np.isfinite(gripper[index]) else "",
                int(pinch[index]),
            ]
            if args.include_raw_matrices:
                row.extend(_flatten_matrix(right_hand[index]))
                row.extend(_flatten_matrix(head[index]))
            writer.writerow(row)

    print(f"saved={output_path} samples={len(time_sec)}")


if __name__ == "__main__":
    main()
