import math
from typing import Iterable

import numpy as np


def _rx(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]])


def _ry(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]])


def _rz(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_matrix(rpy: Iterable[float], convention: str = "intrinsic_xyz") -> np.ndarray:
    roll, pitch, yaw = [float(value) for value in rpy]
    if convention == "intrinsic_xyz":
        return _rx(roll) @ _ry(pitch) @ _rz(yaw)
    if convention == "extrinsic_xyz":
        return _rz(yaw) @ _ry(pitch) @ _rx(roll)
    raise ValueError(f"unsupported RPY convention: {convention}")


def make_transform(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    transform[:3, 3] = np.asarray(list(translation), dtype=float).reshape(3)
    return transform


def pose_to_transform(
    xyz: Iterable[float],
    rpy: Iterable[float],
    convention: str = "intrinsic_xyz",
) -> np.ndarray:
    return make_transform(rpy_to_matrix(rpy, convention), xyz)


def matrix_to_rpy(
    rotation: np.ndarray, convention: str = "intrinsic_xyz"
) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    if convention != "intrinsic_xyz":
        raise ValueError(f"unsupported RPY convention: {convention}")
    pitch = math.asin(float(np.clip(matrix[0, 2], -1.0, 1.0)))
    cosine_pitch = math.cos(pitch)
    if abs(cosine_pitch) > 1.0e-8:
        roll = math.atan2(-matrix[1, 2], matrix[2, 2])
        yaw = math.atan2(-matrix[0, 1], matrix[0, 0])
    else:
        roll = math.atan2(matrix[2, 1], matrix[1, 1])
        yaw = 0.0
    return np.asarray([roll, pitch, yaw], dtype=float)


def transform_to_pose(
    transform: np.ndarray, convention: str = "intrinsic_xyz"
) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float).reshape(4, 4)
    return np.r_[matrix[:3, 3], matrix_to_rpy(matrix[:3, :3], convention)]


def rotation_distance_rad(first: np.ndarray, second: np.ndarray) -> float:
    relative = np.asarray(first)[:3, :3].T @ np.asarray(second)[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def average_rotation(rotations: Iterable[np.ndarray]) -> np.ndarray:
    matrix = np.sum([np.asarray(rotation)[:3, :3] for rotation in rotations], axis=0)
    left, _, right = np.linalg.svd(matrix)
    mean = left @ right
    if np.linalg.det(mean) < 0.0:
        left[:, -1] *= -1.0
        mean = left @ right
    return mean
