from __future__ import annotations

import numpy as np


def normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=float)
    if q.shape != (4,):
        raise ValueError(f"quaternion must have shape (4,), got {q.shape}")
    norm = np.linalg.norm(q)
    if norm <= 0:
        raise ValueError("quaternion must be non-zero")
    return q / norm


def quat_multiply_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = normalize_quat_wxyz(a)
    bw, bx, by, bz = normalize_quat_wxyz(b)
    return normalize_quat_wxyz(
        np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ],
            dtype=float,
        )
    )


def rotvec_to_quat_wxyz(rotvec: np.ndarray) -> np.ndarray:
    rv = np.asarray(rotvec, dtype=float)
    angle = np.linalg.norm(rv)
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rv / angle
    half = angle / 2.0
    return normalize_quat_wxyz(np.concatenate(([np.cos(half)], axis * np.sin(half))))


def quat_wxyz_to_rotvec(quat: np.ndarray) -> np.ndarray:
    q = normalize_quat_wxyz(quat)
    if q[0] < 0:
        q = -q
    vector_norm = np.linalg.norm(q[1:])
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(vector_norm, q[0])
    return q[1:] / vector_norm * angle


def quat_angle_rad_wxyz(quat: np.ndarray) -> float:
    q = normalize_quat_wxyz(quat)
    return float(2.0 * np.arccos(np.clip(abs(q[0]), -1.0, 1.0)))
