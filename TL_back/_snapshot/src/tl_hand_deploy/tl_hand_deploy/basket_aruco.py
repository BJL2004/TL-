from collections import defaultdict
import math
from typing import Iterable

import cv2
import numpy as np

from .geometry import average_rotation, make_transform, pose_to_transform


ARUCO_DICTIONARY_NAME = "DICT_4X4_50"
REQUIRED_BASKET_POSES = ("search_pose", "basket_approach", "basket_release")


def basket_teach_status(data: dict, marker_ids: Iterable[int] = range(6)) -> dict:
    records = data.get("markers", {}) if isinstance(data, dict) else {}
    status = {}
    for marker_id in marker_ids:
        record = records.get(str(marker_id), records.get(marker_id, {})) or {}
        fields = {name: name in record for name in REQUIRED_BASKET_POSES}
        search_pose = record.get("search_pose", {}) or {}
        fields["dynamic_reference"] = (
            "T_base_marker" in search_pose and "marker_length_m" in search_pose
        )
        status[int(marker_id)] = {
            **fields,
            "complete": all(fields.values()),
        }
    return status


def require_aruco_module():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "OpenCV ArUco module is unavailable; install python3-opencv with contrib "
            "support before using basket markers"
        )
    return cv2.aruco


def create_detector():
    aruco = require_aruco_module()
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, "DetectorParameters"):
        parameters = aruco.DetectorParameters()
    else:
        parameters = aruco.DetectorParameters_create()
    if hasattr(aruco, "ArucoDetector"):
        return aruco.ArucoDetector(dictionary, parameters), dictionary, parameters
    return None, dictionary, parameters


def camera_parameters(intrinsics) -> tuple[np.ndarray, np.ndarray]:
    matrix = np.asarray(
        [
            [intrinsics.fx, 0.0, intrinsics.ppx],
            [0.0, intrinsics.fy, intrinsics.ppy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.asarray(intrinsics.coeffs, dtype=np.float64).reshape(-1, 1)
    return matrix, distortion


def detect_marker_transforms(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    marker_length_m: float,
    allowed_ids: Iterable[int] = range(6),
):
    detector, dictionary, parameters = create_detector()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if detector is not None:
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=parameters
        )
    allowed = {int(value) for value in allowed_ids}
    transforms = {}
    selected_corners = []
    selected_ids = []
    half_length = float(marker_length_m) * 0.5
    object_points = np.asarray(
        [
            [-half_length, half_length, 0.0],
            [half_length, half_length, 0.0],
            [half_length, -half_length, 0.0],
            [-half_length, -half_length, 0.0],
        ],
        dtype=np.float64,
    )
    if ids is not None:
        for marker_corners, marker_id_value in zip(corners, ids.reshape(-1)):
            marker_id = int(marker_id_value)
            if marker_id not in allowed:
                continue
            image_points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue
            rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3))
            translation = np.asarray(tvec, dtype=float).reshape(3)
            transforms[marker_id] = make_transform(rotation, translation)
            selected_corners.append(marker_corners)
            selected_ids.append([marker_id])
    return transforms, selected_corners, np.asarray(selected_ids, dtype=np.int32), rejected


def draw_marker_overlay(
    image: np.ndarray,
    corners,
    ids: np.ndarray,
    transforms: dict[int, np.ndarray],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    axis_length_m: float,
) -> np.ndarray:
    preview = image.copy()
    if len(corners) == 0:
        return preview
    cv2.aruco.drawDetectedMarkers(preview, corners, ids)
    for marker_corners, marker_id_value in zip(corners, ids.reshape(-1)):
        marker_id = int(marker_id_value)
        if marker_id not in transforms:
            continue
        transform = transforms[marker_id]
        rvec, _ = cv2.Rodrigues(transform[:3, :3])
        cv2.drawFrameAxes(
            preview,
            camera_matrix,
            distortion,
            rvec,
            transform[:3, 3],
            float(axis_length_m),
        )
        center = np.mean(np.asarray(marker_corners).reshape(-1, 2), axis=0)
        text_position = (int(center[0]) - 38, int(center[1]) - 12)
        cv2.putText(
            preview,
            f"N{marker_id + 1}/ID{marker_id}",
            text_position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 255),
            2,
        )
    return preview


def base_marker_transform(
    tcp_pose_mm_rad: Iterable[float],
    tool_camera: np.ndarray,
    camera_marker: np.ndarray,
) -> np.ndarray:
    tcp = np.asarray(list(tcp_pose_mm_rad), dtype=float).reshape(6)
    base_tool = pose_to_transform(tcp[:3] / 1000.0, tcp[3:], "intrinsic_xyz")
    return base_tool @ np.asarray(tool_camera, dtype=float).reshape(4, 4) @ camera_marker


def marker_transforms_with_depth_translation(
    camera_markers: dict[int, np.ndarray],
    corners,
    ids: np.ndarray,
    point_lookup,
) -> tuple[dict[int, np.ndarray], dict[int, float]]:
    transforms = {}
    depths = {}
    for marker_corners, marker_id_value in zip(corners, ids.reshape(-1)):
        marker_id = int(marker_id_value)
        if marker_id not in camera_markers:
            continue
        center = np.mean(
            np.asarray(marker_corners, dtype=float).reshape(4, 2), axis=0
        )
        try:
            camera_point = np.asarray(point_lookup(center), dtype=float).reshape(3)
        except (RuntimeError, TypeError, ValueError):
            continue
        if not np.all(np.isfinite(camera_point)) or camera_point[2] <= 0.0:
            continue
        transform = np.asarray(camera_markers[marker_id], dtype=float).reshape(4, 4).copy()
        transform[:3, 3] = camera_point
        transforms[marker_id] = transform
        depths[marker_id] = float(camera_point[2])
    return transforms, depths


def average_transforms(transforms: Iterable[np.ndarray]) -> np.ndarray:
    values = [np.asarray(value, dtype=float).reshape(4, 4) for value in transforms]
    if not values:
        raise ValueError("at least one transform is required")
    translations = np.asarray([value[:3, 3] for value in values], dtype=float)
    translation = np.median(translations, axis=0)
    rotation = average_rotation([value[:3, :3] for value in values])
    return make_transform(rotation, translation)


def group_and_average_marker_samples(
    samples: Iterable[dict[int, np.ndarray]], minimum_samples: int
) -> dict[int, np.ndarray]:
    grouped = defaultdict(list)
    for sample in samples:
        for marker_id, transform in sample.items():
            grouped[int(marker_id)].append(transform)
    return {
        marker_id: average_transforms(values)
        for marker_id, values in grouped.items()
        if len(values) >= int(minimum_samples)
    }


def select_stable_marker_nearest_center(
    center_samples: Iterable[dict[int, Iterable[float]]],
    minimum_samples: int,
    image_width: int,
    image_height: int,
) -> tuple[int, dict[int, dict[str, float]]]:
    grouped = defaultdict(list)
    for sample in center_samples:
        for marker_id, center in sample.items():
            grouped[int(marker_id)].append(
                np.asarray(list(center), dtype=float).reshape(2)
            )
    image_center = np.asarray(
        [float(image_width) * 0.5, float(image_height) * 0.5], dtype=float
    )
    metrics = {}
    for marker_id, centers in grouped.items():
        if len(centers) < int(minimum_samples):
            continue
        median_center = np.median(np.asarray(centers, dtype=float), axis=0)
        metrics[marker_id] = {
            "samples": float(len(centers)),
            "center_x": float(median_center[0]),
            "center_y": float(median_center[1]),
            "center_distance_px": float(np.linalg.norm(median_center - image_center)),
        }
    if not metrics:
        raise RuntimeError(
            "no taught basket marker was stable for enough frames in the current view"
        )
    selected_id = min(
        metrics,
        key=lambda marker_id: (
            metrics[marker_id]["center_distance_px"],
            -metrics[marker_id]["samples"],
            marker_id,
        ),
    )
    return selected_id, metrics


def fuse_taught_target(
    base_markers: dict[int, np.ndarray],
    marker_records: dict,
    pose_name: str,
    maximum_spread_mm: float,
    maximum_rotation_spread_deg: float = 15.0,
) -> tuple[np.ndarray, list[int], float]:
    candidates = []
    used_ids = []
    for marker_id, base_marker in base_markers.items():
        record = marker_records.get(str(marker_id), marker_records.get(marker_id))
        if not record or pose_name not in record:
            continue
        marker_tool = np.asarray(
            record[pose_name]["T_marker_tool0"], dtype=float
        ).reshape(4, 4)
        candidates.append(base_marker @ marker_tool)
        used_ids.append(int(marker_id))
    if not candidates:
        raise RuntimeError(
            f"none of the visible markers has a taught {pose_name} transform"
        )
    positions = np.asarray([candidate[:3, 3] for candidate in candidates], dtype=float)
    median = np.median(positions, axis=0)
    distances_mm = np.linalg.norm(positions - median, axis=1) * 1000.0
    keep = distances_mm <= float(maximum_spread_mm)
    if not np.any(keep):
        raise RuntimeError("all marker target estimates were rejected as outliers")
    kept = [candidate for candidate, accepted in zip(candidates, keep) if accepted]
    kept_ids = [marker_id for marker_id, accepted in zip(used_ids, keep) if accepted]
    fused = average_transforms(kept)
    spread_mm = 0.0
    if len(kept) > 1:
        fused_position = fused[:3, 3]
        spread_mm = float(
            np.max(
                [
                    np.linalg.norm(candidate[:3, 3] - fused_position) * 1000.0
                    for candidate in kept
                ]
            )
        )
    if spread_mm > float(maximum_spread_mm):
        raise RuntimeError(
            f"marker estimates disagree by {spread_mm:.1f} mm, above "
            f"{maximum_spread_mm:.1f} mm"
        )
    rotation_spread_deg = 0.0
    if len(kept) > 1:
        rotation_spread_deg = max(
            rotation_error_deg(fused, candidate) for candidate in kept
        )
    if rotation_spread_deg > float(maximum_rotation_spread_deg):
        raise RuntimeError(
            f"marker target orientations disagree by {rotation_spread_deg:.1f} deg, "
            f"above {maximum_rotation_spread_deg:.1f} deg"
        )
    return fused, kept_ids, spread_mm


def translated_taught_target(
    base_marker_now: np.ndarray,
    marker_record: dict,
    pose_name: str,
) -> tuple[np.ndarray, np.ndarray]:
    search_pose = marker_record.get("search_pose", {}) or {}
    if "T_base_marker" not in search_pose:
        raise RuntimeError(
            "basket teach record has no dynamic marker reference; re-teach this "
            "basket with the current teach_aruco_basket.sh"
        )
    pose_record = marker_record.get(pose_name, {}) or {}
    if "taught_tcp" not in pose_record:
        raise RuntimeError(f"basket teach record has no taught_tcp for {pose_name}")
    taught_tcp = np.asarray(pose_record["taught_tcp"], dtype=float).reshape(6)
    taught_target = pose_to_transform(
        taught_tcp[:3] / 1000.0, taught_tcp[3:], "intrinsic_xyz"
    )
    base_marker_teach = np.asarray(
        search_pose["T_base_marker"], dtype=float
    ).reshape(4, 4)
    shift = (
        np.asarray(base_marker_now, dtype=float).reshape(4, 4)[:3, 3]
        - base_marker_teach[:3, 3]
    )
    target = taught_target.copy()
    target[:3, 3] += shift
    return target, shift


def rotation_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    relative = first[:3, :3].T @ second[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))
