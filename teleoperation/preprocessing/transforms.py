from __future__ import annotations

import numpy as np

from teleoperation.types import Pose

T_BASE_VFRAME_TRANS = np.array(
    [
        [1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ],
    dtype=float,
)

T_BASE_VFRAME_ROT = np.array(
    [
        [0, 0, -1, 0],
        [-1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ],
    dtype=float,
)


def is_valid_transform(matrix: np.ndarray) -> bool:
    mat = np.asarray(matrix, dtype=float)
    if mat.shape != (4, 4):
        return False
    if not np.all(np.isfinite(mat)):
        return False
    return abs(np.linalg.det(mat[:3, :3])) > 1e-9


def fast_mat_inv(matrix: np.ndarray) -> np.ndarray:
    mat = np.asarray(matrix, dtype=float)
    if mat.shape != (4, 4):
        raise ValueError(f"matrix must have shape (4, 4), got {mat.shape}")
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret


def rotation_matrix_to_quat_wxyz(rotation: np.ndarray) -> np.ndarray:
    rot = np.asarray(rotation, dtype=float)
    if rot.shape != (3, 3):
        raise ValueError(f"rotation must have shape (3, 3), got {rot.shape}")

    trace = np.trace(rot)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s

    quat = np.array([w, x, y, z], dtype=float)
    return quat / np.linalg.norm(quat)


def normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=float)
    if q.shape != (4,):
        raise ValueError(f"quaternion must have shape (4,), got {q.shape}")
    norm = np.linalg.norm(q)
    if norm <= 0:
        raise ValueError("quaternion must be non-zero")
    return q / norm


def quat_angle_rad_wxyz(quat: np.ndarray) -> float:
    q = normalize_quat_wxyz(quat)
    return float(2.0 * np.arccos(np.clip(abs(q[0]), -1.0, 1.0)))


def scale_quat_angle_wxyz(quat: np.ndarray, scale: float) -> np.ndarray:
    if scale < 0:
        raise ValueError("scale must be non-negative")
    q = normalize_quat_wxyz(quat)
    if q[0] < 0:
        q = -q
    vector_norm = np.linalg.norm(q[1:])
    if scale == 0 or vector_norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    angle = 2.0 * np.arctan2(vector_norm, q[0])
    axis = q[1:] / vector_norm
    scaled_half_angle = angle * scale / 2.0
    return normalize_quat_wxyz(
        np.concatenate(([np.cos(scaled_half_angle)], axis * np.sin(scaled_half_angle)))
    )


def matrix_to_pose(matrix: np.ndarray, timestamp_sec: float = 0.0) -> Pose:
    mat = np.asarray(matrix, dtype=float)
    if not is_valid_transform(mat):
        raise ValueError("matrix is not a valid homogeneous transform")
    return Pose(
        position=mat[:3, 3].copy(),
        orientation_wxyz=rotation_matrix_to_quat_wxyz(mat[:3, :3]),
        timestamp_sec=timestamp_sec,
    )


def relative_avp_hand_pose(initial_hand_matrix: np.ndarray, current_hand_matrix: np.ndarray) -> np.ndarray:
    if not is_valid_transform(initial_hand_matrix):
        raise ValueError("initial_hand_matrix is invalid")
    if not is_valid_transform(current_hand_matrix):
        raise ValueError("current_hand_matrix is invalid")

    t_vframe0_vframet = fast_mat_inv(initial_hand_matrix) @ current_hand_matrix
    t_arm0_armt_trans = T_BASE_VFRAME_TRANS @ t_vframe0_vframet @ fast_mat_inv(T_BASE_VFRAME_TRANS)
    t_arm0_armt_rot = T_BASE_VFRAME_ROT @ t_vframe0_vframet @ fast_mat_inv(T_BASE_VFRAME_ROT)

    target = np.eye(4)
    target[:3, :3] = t_arm0_armt_rot[:3, :3]
    target[:3, 3] = t_arm0_armt_trans[:3, 3]
    return target
