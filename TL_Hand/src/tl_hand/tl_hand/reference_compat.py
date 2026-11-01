import math
from typing import Iterable

import numpy as np

from .geometry import pose_to_transform, rotation_distance_rad


def wrap_angle_rad(value: float) -> float:
    return float((float(value) + math.pi) % (2.0 * math.pi) - math.pi)


def nearest_equivalent_joint_angles(
    target_deg: Iterable[float],
    current_deg: Iterable[float],
    minimum_deg: Iterable[float],
    maximum_deg: Iterable[float],
) -> list[float]:
    target = np.asarray(list(target_deg), dtype=float)
    current = np.asarray(list(current_deg), dtype=float)
    minimum = np.asarray(list(minimum_deg), dtype=float)
    maximum = np.asarray(list(maximum_deg), dtype=float)
    if not (target.shape == current.shape == minimum.shape == maximum.shape):
        raise ValueError("joint arrays must have equal shapes")

    normalized = []
    for raw, actual, lower, upper in zip(target, current, minimum, maximum):
        candidates = [
            float(raw + 360.0 * turns)
            for turns in range(-3, 4)
            if lower - 1e-9 <= raw + 360.0 * turns <= upper + 1e-9
        ]
        normalized.append(
            min(candidates, key=lambda value: abs(value - actual))
            if candidates
            else float(raw)
        )
    return normalized


def camera_point_to_base(
    tcp_pose_mm_rad: Iterable[float],
    tool_camera: np.ndarray,
    camera_point_m: Iterable[float],
    convention: str = "intrinsic_xyz",
) -> np.ndarray:
    pose = np.asarray(list(tcp_pose_mm_rad), dtype=float).reshape(6)
    pose[3:] = [wrap_angle_rad(value) for value in pose[3:]]
    point = np.asarray(list(camera_point_m), dtype=float).reshape(3)
    base_tool = pose_to_transform(pose[:3] / 1000.0, pose[3:], convention)
    base_point = base_tool @ np.asarray(tool_camera, dtype=float).reshape(4, 4) @ np.r_[point, 1.0]
    return base_point[:3] * 1000.0


def build_hover_pose(
    base_point_mm: Iterable[float],
    current_tcp_mm_rad: Iterable[float],
    hover_mm: float,
) -> list[float]:
    point = np.asarray(list(base_point_mm), dtype=float).reshape(3)
    tcp = np.asarray(list(current_tcp_mm_rad), dtype=float).reshape(6)
    return [
        float(point[0]),
        float(point[1]),
        float(point[2] + hover_mm),
        wrap_angle_rad(tcp[3]),
        wrap_angle_rad(tcp[4]),
        wrap_angle_rad(tcp[5]),
    ]


def tool_pose_for_virtual_tcp(
    desired_virtual_tcp_base_mm: Iterable[float],
    current_tool_pose_mm_rad: Iterable[float],
    virtual_tcp_offset_tool_mm: Iterable[float],
    convention: str = "intrinsic_xyz",
    target_tool_rpy_rad: Iterable[float] | None = None,
) -> list[float]:
    desired = np.asarray(list(desired_virtual_tcp_base_mm), dtype=float).reshape(3)
    current = np.asarray(list(current_tool_pose_mm_rad), dtype=float).reshape(6)
    current[3:] = [wrap_angle_rad(value) for value in current[3:]]
    if target_tool_rpy_rad is None:
        target_rpy = current[3:]
    else:
        target_rpy = np.asarray(list(target_tool_rpy_rad), dtype=float).reshape(3)
        target_rpy = np.asarray([wrap_angle_rad(value) for value in target_rpy])
    offset = np.asarray(list(virtual_tcp_offset_tool_mm), dtype=float).reshape(3)
    base_tool = pose_to_transform([0.0, 0.0, 0.0], target_rpy, convention)
    rotated_offset_mm = base_tool[:3, :3] @ offset
    tool_position_mm = desired - rotated_offset_mm
    return [
        float(tool_position_mm[0]),
        float(tool_position_mm[1]),
        float(tool_position_mm[2]),
        wrap_angle_rad(target_rpy[0]),
        wrap_angle_rad(target_rpy[1]),
        wrap_angle_rad(target_rpy[2]),
    ]


def xy_probe_pose(
    target_pose_mm_rad: Iterable[float],
    current_tcp_mm_rad: Iterable[float],
    shift_mm: float,
    z_lift_mm: float,
) -> tuple[list[float], np.ndarray]:
    target = np.asarray(list(target_pose_mm_rad), dtype=float).reshape(6)
    current = np.asarray(list(current_tcp_mm_rad), dtype=float).reshape(6)
    direction = current[:2] - target[:2]
    distance = float(np.linalg.norm(direction))
    if distance <= 1e-9 or shift_mm <= 0.0:
        shift = np.zeros(2, dtype=float)
    else:
        shift = direction / distance * min(float(shift_mm), distance)
    pose = target.copy()
    pose[:2] += shift
    pose[2] += float(z_lift_mm)
    return pose.tolist(), shift


def linear_joint_trajectory(
    start: Iterable[float], target: Iterable[float], point_count: int
) -> list[list[float]]:
    if point_count <= 0:
        raise ValueError("point_count must be positive")
    start_array = np.asarray(list(start), dtype=float)
    target_array = np.asarray(list(target), dtype=float)
    if start_array.shape != target_array.shape:
        raise ValueError("start and target must have equal lengths")
    return [
        (start_array + (target_array - start_array) * (index / point_count)).tolist()
        for index in range(1, point_count + 1)
    ]


def minimum_jerk_joint_trajectory(
    start: Iterable[float], target: Iterable[float], point_count: int
) -> list[list[float]]:
    if point_count <= 0:
        raise ValueError("point_count must be positive")
    start_array = np.asarray(list(start), dtype=float)
    target_array = np.asarray(list(target), dtype=float)
    if start_array.shape != target_array.shape:
        raise ValueError("start and target must have equal lengths")
    trajectory = []
    for index in range(1, point_count + 1):
        phase = index / point_count
        blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
        trajectory.append(
            (start_array + (target_array - start_array) * blend).tolist()
        )
    return trajectory


def servoj_trajectory_point_count(
    maximum_joint_delta_deg: float,
    rate_hz: float,
    speed_deg_s: float,
    minimum_duration_sec: float,
) -> tuple[int, float]:
    if maximum_joint_delta_deg < 0.0:
        raise ValueError("maximum_joint_delta_deg cannot be negative")
    if rate_hz <= 0.0 or speed_deg_s <= 0.0 or minimum_duration_sec <= 0.0:
        raise ValueError("rate, speed, and minimum duration must be positive")
    # The derivative of the quintic minimum-jerk blend peaks at 1.875.
    duration_sec = max(
        float(minimum_duration_sec),
        1.875 * float(maximum_joint_delta_deg) / float(speed_deg_s),
    )
    return max(2, int(math.ceil(duration_sec * float(rate_hz)))), duration_sec


def approach_point_above_reachable_grasp(
    reachable_grasp_mm: Iterable[float],
    approach_clearance_mm: float,
    grasp_clearance_mm: float,
) -> np.ndarray:
    grasp = np.asarray(list(reachable_grasp_mm), dtype=float).reshape(3)
    vertical_separation = max(
        0.0, float(approach_clearance_mm) - float(grasp_clearance_mm)
    )
    approach = grasp.copy()
    approach[2] += vertical_separation
    return approach


def vendor_movej_target(target_joint, target_size: int = 14) -> list[float]:
    target = [float(value) for value in target_joint]
    if len(target) < 6:
        raise ValueError("MoveJ target must contain at least six joints")
    if target_size < len(target):
        raise ValueError("MoveJ target is longer than the vendor target array")
    return target + [0.0] * (target_size - len(target))


def joint_error_degrees(current_joint, target_joint) -> np.ndarray:
    current = np.asarray(list(current_joint)[:6], dtype=float)
    target = np.asarray(list(target_joint)[:6], dtype=float)
    if current.size != 6 or target.size != 6:
        raise ValueError("joint error requires six current and target joints")
    return target - current


def format_joint_errors(error_degrees) -> str:
    errors = np.asarray(list(error_degrees), dtype=float).reshape(6)
    return "[" + ", ".join(
        f"J{index + 1}={value:+.3f}" for index, value in enumerate(errors)
    ) + "]"


def transform_delta(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    first_array = np.asarray(first, dtype=float).reshape(4, 4)
    second_array = np.asarray(second, dtype=float).reshape(4, 4)
    translation_mm = float(
        np.linalg.norm(first_array[:3, 3] - second_array[:3, 3]) * 1000.0
    )
    rotation_deg = float(math.degrees(rotation_distance_rad(first_array, second_array)))
    return translation_mm, rotation_deg


def summarize_validation_records(records) -> dict:
    if not records:
        raise ValueError("at least one validation record is required")
    corrections = np.asarray(
        [record["correction_base_mm"] for record in records], dtype=float
    )
    positions = np.asarray(
        [record["estimated_object_base_mm"] for record in records], dtype=float
    )
    mean_correction = corrections.mean(axis=0)
    xy_residuals = corrections[:, :2] - mean_correction[:2]
    residual_rms = float(np.sqrt(np.mean(np.sum(xy_residuals**2, axis=1))))
    residual_max = float(np.max(np.linalg.norm(xy_residuals, axis=1)))
    z_residuals = corrections[:, 2] - mean_correction[2]
    z_rms = float(np.sqrt(np.mean(z_residuals**2)))
    z_max = float(np.max(np.abs(z_residuals)))
    summary = {
        "samples": len(records),
        "mean_correction_base_mm": mean_correction.tolist(),
        "fixed_bias_xy_mm": float(np.linalg.norm(mean_correction[:2])),
        "fixed_bias_z_mm": float(mean_correction[2]),
        "after_fixed_bias_xy_rms_mm": residual_rms,
        "after_fixed_bias_xy_max_mm": residual_max,
        "after_fixed_bias_z_rms_mm": z_rms,
        "after_fixed_bias_z_max_mm": z_max,
        "workspace_span_xy_mm": (
            positions[:, :2].max(axis=0) - positions[:, :2].min(axis=0)
        ).tolist(),
    }
    judgment = "need at least 6 spatially distributed samples"
    if len(records) >= 6:
        centered = positions[:, :2] - positions[:, :2].mean(axis=0)
        design = np.column_stack(
            [np.ones(len(records)), centered[:, 0] / 100.0, centered[:, 1] / 100.0]
        )
        coefficients, _, rank, _ = np.linalg.lstsq(
            design, corrections[:, :2], rcond=None
        )
        if rank == 3:
            fitted = design @ coefficients
            affine_residuals = corrections[:, :2] - fitted
            affine_rms = float(
                np.sqrt(np.mean(np.sum(affine_residuals**2, axis=1)))
            )
            summary["spatial_model"] = {
                "correction_xy_mm_per_100mm_x": coefficients[1].tolist(),
                "correction_xy_mm_per_100mm_y": coefficients[2].tolist(),
                "residual_xy_rms_mm": affine_rms,
            }
            if residual_rms <= 2.0 and residual_max <= 4.0:
                judgment = "predominantly fixed offset"
            elif affine_rms <= residual_rms * 0.7:
                judgment = "position-dependent systematic error"
            else:
                judgment = "scattered measurement/calibration error"
    summary["judgment"] = judgment
    return summary
