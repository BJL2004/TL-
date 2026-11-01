import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from .geometry import average_rotation, make_transform, pose_to_transform, rotation_distance_rad


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Solve and validate eye-in-hand calibration")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--min-valid-samples", type=int, default=20)
    parser.add_argument("--max-reprojection-px", type=float, default=0.8)
    parser.add_argument("--max-view-reprojection-px", type=float, default=1.0)
    parser.add_argument("--max-translation-rms-mm", type=float, default=5.0)
    parser.add_argument("--max-translation-max-mm", type=float, default=10.0)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=0.5)
    parser.add_argument("--max-rotation-max-deg", type=float, default=1.0)
    parser.add_argument("--min-motion-translation-mm", type=float, default=120.0)
    parser.add_argument("--min-motion-rotation-deg", type=float, default=40.0)
    parser.add_argument("--min-rotation-axis-span-deg", type=float, default=25.0)
    parser.add_argument("--min-rotation-principal-spread-deg", type=float, default=8.0)
    parser.add_argument("--min-rotation-axes", type=int, default=2)
    parser.add_argument("--min-image-coverage-bins", type=int, default=5)
    parser.add_argument("--min-board-area-ratio", type=float, default=0.02)
    parser.add_argument("--max-loo-translation-mm", type=float, default=5.0)
    parser.add_argument("--max-loo-rotation-deg", type=float, default=0.5)
    parser.add_argument("--max-outlier-fraction", type=float, default=0.15)
    parser.add_argument("--outlier-sigma", type=float, default=3.5)
    parser.add_argument("--outlier-translation-floor-mm", type=float, default=8.0)
    parser.add_argument("--outlier-rotation-floor-deg", type=float, default=1.0)
    parser.add_argument("--disable-outlier-rejection", action="store_true")
    parser.add_argument(
        "--refine-camera-intrinsics",
        action="store_true",
        help="refine camera intrinsics from the hand-eye images instead of fixing the RealSense factory values",
    )
    arguments = parser.parse_args(argv)
    if arguments.min_valid_samples < 10:
        parser.error("--min-valid-samples must be at least 10")
    if not 0.0 <= arguments.max_outlier_fraction < 0.5:
        parser.error("--max-outlier-fraction must be in [0, 0.5)")
    if not 1 <= arguments.min_rotation_axes <= 3:
        parser.error("--min-rotation-axes must be 1..3")
    if not 1 <= arguments.min_image_coverage_bins <= 9:
        parser.error("--min-image-coverage-bins must be 1..9")
    return arguments


def _load_dataset(dataset: Path):
    session = json.loads((dataset / "session.json").read_text(encoding="utf-8"))
    samples = [
        json.loads(line)
        for line in (dataset / "samples.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if len(samples) < 10:
        raise RuntimeError(f"at least 10 samples are required, found {len(samples)}")
    return session, samples


def _object_points(cols: int, rows: int, square_size: float):
    points = np.zeros((cols * rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    return points


def _detect(dataset: Path, session, samples):
    checkerboard = session["checkerboard"]
    cols = int(checkerboard["pattern_cols"])
    rows = int(checkerboard["pattern_rows"])
    object_template = _object_points(cols, rows, float(checkerboard["square_size_m"]))
    object_points = []
    image_points = []
    valid_samples = []
    image_size = None
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    for sample in samples:
        image = cv2.imread(str(dataset / sample["image"]))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(gray, (cols, rows))
        else:
            found, corners = cv2.findChessboardCorners(gray, (cols, rows))
        if not found:
            print(f"跳过样本 {sample['index']}：无法重新检测棋盘格")
            continue
        corners = cv2.cornerSubPix(gray, corners.astype(np.float32), (5, 5), (-1, -1), criteria)
        flat_corners = corners.reshape(-1, 2)
        hull = cv2.convexHull(flat_corners.astype(np.float32))
        area_ratio = float(cv2.contourArea(hull) / float(gray.shape[0] * gray.shape[1]))
        center = flat_corners.mean(axis=0)
        x, y, width, height = cv2.boundingRect(flat_corners.astype(np.float32))
        board_gray = gray[y : y + height, x : x + width]
        sharpness = float(cv2.Laplacian(board_gray, cv2.CV_64F).var()) if board_gray.size else 0.0
        sample = dict(sample)
        sample["checkerboard_quality"] = {
            "center_normalized": [
                float(center[0] / gray.shape[1]),
                float(center[1] / gray.shape[0]),
            ],
            "area_ratio": area_ratio,
            "sharpness_laplacian": sharpness,
        }
        object_points.append(object_template.copy())
        image_points.append(corners)
        valid_samples.append(sample)
    if len(valid_samples) < 10:
        raise RuntimeError(f"only {len(valid_samples)} samples contain a valid checkerboard")
    return object_points, image_points, valid_samples, image_size


def _camera_calibration(session, object_points, image_points, image_size, refine_intrinsics=False):
    intrinsics = session["intrinsics"]
    camera_matrix = np.array(
        [
            [intrinsics["fx"], 0.0, intrinsics["ppx"]],
            [0.0, intrinsics["fy"], intrinsics["ppy"]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    model = str(intrinsics.get("model", "")).lower()
    flags = cv2.CALIB_USE_INTRINSIC_GUESS
    if not refine_intrinsics:
        flags |= (
            cv2.CALIB_FIX_FOCAL_LENGTH
            | cv2.CALIB_FIX_PRINCIPAL_POINT
            | cv2.CALIB_FIX_K1
            | cv2.CALIB_FIX_K2
            | cv2.CALIB_FIX_K3
            | cv2.CALIB_ZERO_TANGENT_DIST
        )
    if "brown_conrady" in model and "inverse" not in model:
        distortion = np.asarray(intrinsics.get("coeffs", [0.0] * 5), dtype=np.float64)
    else:
        distortion = np.zeros(5, dtype=np.float64)
    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        camera_matrix,
        distortion,
        flags=flags,
    )
    return rms, camera_matrix, distortion, rvecs, tvecs


def _per_view_reprojection_errors(
    object_points,
    image_points,
    rvecs,
    tvecs,
    camera_matrix,
    distortion,
):
    errors = []
    for object_view, image_view, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(
            object_view,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        delta = projected.reshape(-1, 2) - image_view.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(delta**2, axis=1)))))
    return errors


def _image_coverage(samples):
    occupied = set()
    area_ratios = []
    for sample in samples:
        quality = sample.get("checkerboard_quality", {})
        center = quality.get("center_normalized")
        if center is not None:
            column = min(2, max(0, int(float(center[0]) * 3.0)))
            row = min(2, max(0, int(float(center[1]) * 3.0)))
            occupied.add(row * 3 + column)
        if "area_ratio" in quality:
            area_ratios.append(float(quality["area_ratio"]))
    return {
        "image_coverage_bins": len(occupied),
        "image_coverage_bin_indices": sorted(occupied),
        "board_area_ratio_min": float(min(area_ratios)) if area_ratios else 0.0,
        "board_area_ratio_median": float(np.median(area_ratios)) if area_ratios else 0.0,
        "board_area_ratio_max": float(max(area_ratios)) if area_ratios else 0.0,
    }


def _motion_span(transforms):
    max_translation = 0.0
    max_rotation = 0.0
    for first_index, first in enumerate(transforms):
        for second in transforms[first_index + 1 :]:
            max_translation = max(max_translation, np.linalg.norm(first[:3, 3] - second[:3, 3]))
            max_rotation = max(max_rotation, rotation_distance_rad(first, second))
    return float(max_translation * 1000.0), float(math.degrees(max_rotation))


def _motion_diversity(transforms):
    translations = np.asarray([transform[:3, 3] for transform in transforms], dtype=np.float64)
    translation_axis_span = np.ptp(translations, axis=0) * 1000.0
    reference_rotation = transforms[0][:3, :3]
    rotation_vectors = []
    for transform in transforms:
        vector, _ = cv2.Rodrigues(reference_rotation.T @ transform[:3, :3])
        rotation_vectors.append(np.degrees(vector.reshape(3)))
    rotation_axis_span = np.ptp(np.asarray(rotation_vectors), axis=0)
    centered_rotation_vectors = np.asarray(rotation_vectors) - np.mean(rotation_vectors, axis=0)
    singular_values = np.linalg.svd(centered_rotation_vectors, compute_uv=False)
    principal_spread = singular_values / math.sqrt(len(rotation_vectors))
    return {
        "motion_translation_axis_span_mm": [float(value) for value in translation_axis_span],
        "motion_rotation_axis_span_deg": [float(value) for value in rotation_axis_span],
        "motion_rotation_principal_spread_deg": [
            float(value) for value in principal_spread
        ],
    }


def _evaluate(base_gripper, camera_target, gripper_camera, sample_indices=None):
    base_targets = [
        base_gripper[index] @ gripper_camera @ camera_target[index]
        for index in range(len(base_gripper))
    ]
    translations = np.array([transform[:3, 3] for transform in base_targets])
    translation_mean = translations.mean(axis=0)
    translation_errors = np.linalg.norm(translations - translation_mean, axis=1)
    rotation_mean = average_rotation([transform[:3, :3] for transform in base_targets])
    rotation_errors = np.array(
        [rotation_distance_rad(make_transform(rotation_mean, [0, 0, 0]), transform) for transform in base_targets]
    )
    indices = list(range(len(base_targets))) if sample_indices is None else list(sample_indices)
    return {
        "translation_rms_mm": float(np.sqrt(np.mean(translation_errors**2)) * 1000.0),
        "translation_max_mm": float(np.max(translation_errors) * 1000.0),
        "rotation_rms_deg": float(math.degrees(np.sqrt(np.mean(rotation_errors**2)))),
        "rotation_max_deg": float(math.degrees(np.max(rotation_errors))),
        "per_sample_residuals": [
            {
                "sample_index": int(sample_index),
                "translation_mm": float(translation_error * 1000.0),
                "rotation_deg": float(math.degrees(rotation_error)),
            }
            for sample_index, translation_error, rotation_error in zip(
                indices,
                translation_errors,
                rotation_errors,
            )
        ],
    }


HAND_EYE_METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def _base_gripper_transforms(samples, convention):
    transforms = []
    for sample in samples:
        pose = sample["tcp_pose_mm_rad"]
        transforms.append(
            pose_to_transform(
                np.asarray(pose[:3], dtype=float) / 1000.0,
                pose[3:],
                convention,
            )
        )
    return transforms


def _solve_one(base_gripper, camera_target, method_name):
    rotation, translation = cv2.calibrateHandEye(
        [transform[:3, :3] for transform in base_gripper],
        [transform[:3, 3] for transform in base_gripper],
        [transform[:3, :3] for transform in camera_target],
        [transform[:3, 3] for transform in camera_target],
        method=HAND_EYE_METHODS[method_name],
    )
    return make_transform(rotation, translation.reshape(3))


def _solve_candidates(
    samples,
    rvecs,
    tvecs,
    conventions=("intrinsic_xyz",),
    sample_indices=None,
):
    camera_target = [make_transform(cv2.Rodrigues(rvec)[0], tvec.reshape(3)) for rvec, tvec in zip(rvecs, tvecs)]
    candidates = []
    indices = [sample["index"] for sample in samples] if sample_indices is None else sample_indices
    for convention in conventions:
        base_gripper = _base_gripper_transforms(samples, convention)
        motion_translation, motion_rotation = _motion_span(base_gripper)
        diversity = _motion_diversity(base_gripper)
        for method_name in HAND_EYE_METHODS:
            try:
                gripper_camera = _solve_one(base_gripper, camera_target, method_name)
                metrics = _evaluate(base_gripper, camera_target, gripper_camera, indices)
                metrics["motion_translation_span_mm"] = motion_translation
                metrics["motion_rotation_span_deg"] = motion_rotation
                metrics.update(diversity)
                score = metrics["translation_rms_mm"] + 5.0 * metrics["rotation_rms_deg"]
                if np.isfinite(gripper_camera).all() and math.isfinite(score):
                    candidates.append(
                        {
                            "method": method_name,
                            "rpy_convention": convention,
                            "T_tool0_camera": gripper_camera,
                            "metrics": metrics,
                            "score": score,
                        }
                    )
            except cv2.error as error:
                print(f"跳过 {convention}/{method_name}: {error}")
    if not candidates:
        raise RuntimeError("all hand-eye methods failed")
    return sorted(candidates, key=lambda item: item["score"])


def _robust_upper_limit(values, absolute_floor, sigma):
    data = np.asarray(values, dtype=np.float64)
    median = float(np.median(data))
    mad = float(np.median(np.abs(data - median)))
    robust_sigma = 1.4826 * mad
    return float(max(absolute_floor, median + sigma * robust_sigma))


def _calibrate_with_reprojection_filter(
    session,
    object_points,
    image_points,
    samples,
    image_size,
    arguments,
    rejection_budget,
):
    active_object = list(object_points)
    active_image = list(image_points)
    active_samples = list(samples)
    rejected = []
    while True:
        calibration = _camera_calibration(
            session,
            active_object,
            active_image,
            image_size,
            refine_intrinsics=arguments.refine_camera_intrinsics,
        )
        reprojection, camera_matrix, distortion, rvecs, tvecs = calibration
        view_errors = _per_view_reprojection_errors(
            active_object,
            active_image,
            rvecs,
            tvecs,
            camera_matrix,
            distortion,
        )
        limit = _robust_upper_limit(
            view_errors,
            arguments.max_view_reprojection_px,
            arguments.outlier_sigma,
        )
        worst_position = int(np.argmax(view_errors))
        worst_error = view_errors[worst_position]
        can_reject = (
            not arguments.disable_outlier_rejection
            and len(rejected) < rejection_budget
            and len(active_samples) > arguments.min_valid_samples
        )
        if not can_reject or worst_error <= limit:
            return (
                reprojection,
                camera_matrix,
                distortion,
                rvecs,
                tvecs,
                view_errors,
                active_object,
                active_image,
                active_samples,
                rejected,
            )
        sample = active_samples.pop(worst_position)
        active_object.pop(worst_position)
        active_image.pop(worst_position)
        rejected.append(
            {
                "sample_index": int(sample["index"]),
                "stage": "reprojection",
                "measured_px": float(worst_error),
                "threshold_px": float(limit),
            }
        )


def _robust_handeye_solve(samples, rvecs, tvecs, convention, arguments, rejection_budget):
    active_samples = list(samples)
    active_rvecs = list(rvecs)
    active_tvecs = list(tvecs)
    rejected = []
    while True:
        candidates = _solve_candidates(
            active_samples,
            active_rvecs,
            active_tvecs,
            conventions=(convention,),
        )
        selected = candidates[0]
        residuals = selected["metrics"]["per_sample_residuals"]
        translation_limit = _robust_upper_limit(
            [item["translation_mm"] for item in residuals],
            arguments.outlier_translation_floor_mm,
            arguments.outlier_sigma,
        )
        rotation_limit = _robust_upper_limit(
            [item["rotation_deg"] for item in residuals],
            arguments.outlier_rotation_floor_deg,
            arguments.outlier_sigma,
        )
        severities = [
            max(
                item["translation_mm"] / translation_limit,
                item["rotation_deg"] / rotation_limit,
            )
            for item in residuals
        ]
        worst_position = int(np.argmax(severities))
        can_reject = (
            not arguments.disable_outlier_rejection
            and len(rejected) < rejection_budget
            and len(active_samples) > arguments.min_valid_samples
        )
        if not can_reject or severities[worst_position] <= 1.0:
            return active_samples, active_rvecs, active_tvecs, candidates, selected, rejected
        residual = residuals[worst_position]
        active_samples.pop(worst_position)
        active_rvecs.pop(worst_position)
        active_tvecs.pop(worst_position)
        rejected.append(
            {
                "sample_index": int(residual["sample_index"]),
                "stage": "handeye_consistency",
                "translation_mm": float(residual["translation_mm"]),
                "translation_threshold_mm": float(translation_limit),
                "rotation_deg": float(residual["rotation_deg"]),
                "rotation_threshold_deg": float(rotation_limit),
            }
        )


def _leave_one_out_stability(samples, rvecs, tvecs, selected):
    convention = selected["rpy_convention"]
    method_name = selected["method"]
    full_transform = selected["T_tool0_camera"]
    base_gripper = _base_gripper_transforms(samples, convention)
    camera_target = [
        make_transform(cv2.Rodrigues(rvec)[0], tvec.reshape(3))
        for rvec, tvec in zip(rvecs, tvecs)
    ]
    translation_changes = []
    rotation_changes = []
    failed_samples = []
    for omitted in range(len(samples)):
        reduced_base = [value for index, value in enumerate(base_gripper) if index != omitted]
        reduced_camera = [value for index, value in enumerate(camera_target) if index != omitted]
        try:
            estimate = _solve_one(reduced_base, reduced_camera, method_name)
            if not np.isfinite(estimate).all():
                raise ValueError("non-finite hand-eye transform")
            translation_changes.append(
                float(np.linalg.norm(estimate[:3, 3] - full_transform[:3, 3]) * 1000.0)
            )
            rotation_changes.append(float(math.degrees(rotation_distance_rad(estimate, full_transform))))
        except (cv2.error, ValueError):
            failed_samples.append(int(samples[omitted]["index"]))
    if failed_samples or not translation_changes:
        return {
            "leave_one_out_translation_rms_mm": None,
            "leave_one_out_translation_max_mm": None,
            "leave_one_out_rotation_rms_deg": None,
            "leave_one_out_rotation_max_deg": None,
            "leave_one_out_failed_sample_indices": failed_samples,
        }
    return {
        "leave_one_out_translation_rms_mm": float(
            np.sqrt(np.mean(np.asarray(translation_changes) ** 2))
        ),
        "leave_one_out_translation_max_mm": float(max(translation_changes)),
        "leave_one_out_rotation_rms_deg": float(
            np.sqrt(np.mean(np.asarray(rotation_changes) ** 2))
        ),
        "leave_one_out_rotation_max_deg": float(max(rotation_changes)),
        "leave_one_out_failed_sample_indices": [],
    }


def _precision_main(argv=None) -> None:
    arguments = parse_args(argv)
    dataset = arguments.dataset.expanduser().resolve()
    session, loaded_samples = _load_dataset(dataset)
    production_convention = session.get("robot_pose", {}).get("rpy_convention")
    if production_convention is None:
        production_convention = "intrinsic_xyz"
        print("WARNING: legacy dataset has no rpy_convention; using intrinsic_xyz for TL /tcp_pose")
    if production_convention != "intrinsic_xyz":
        raise RuntimeError(
            "this robot runtime requires intrinsic_xyz; refusing to select a different convention"
        )

    object_points, image_points, detected_samples, image_size = _detect(
        dataset,
        session,
        loaded_samples,
    )
    quality_rejections = []
    quality_object = []
    quality_image = []
    quality_samples = []
    for object_view, image_view, sample in zip(object_points, image_points, detected_samples):
        area_ratio = float(sample.get("checkerboard_quality", {}).get("area_ratio", 0.0))
        if area_ratio < arguments.min_board_area_ratio:
            quality_rejections.append(
                {
                    "sample_index": int(sample["index"]),
                    "stage": "checkerboard_quality",
                    "board_area_ratio": area_ratio,
                    "minimum_board_area_ratio": float(arguments.min_board_area_ratio),
                }
            )
            continue
        quality_object.append(object_view)
        quality_image.append(image_view)
        quality_samples.append(sample)
    if len(quality_samples) < arguments.min_valid_samples:
        raise RuntimeError(
            f"only {len(quality_samples)} quality samples remain; "
            f"need at least {arguments.min_valid_samples}"
        )

    rejection_budget = int(math.floor(len(quality_samples) * arguments.max_outlier_fraction))
    camera_result = _calibrate_with_reprojection_filter(
        session,
        quality_object,
        quality_image,
        quality_samples,
        image_size,
        arguments,
        rejection_budget,
    )
    (
        reprojection,
        camera_matrix,
        distortion,
        rvecs,
        tvecs,
        view_errors,
        active_object,
        active_image,
        active_samples,
        reprojection_rejections,
    ) = camera_result
    remaining_budget = max(0, rejection_budget - len(reprojection_rejections))
    (
        final_samples,
        _,
        _,
        _,
        _,
        handeye_rejections,
    ) = _robust_handeye_solve(
        active_samples,
        rvecs,
        tvecs,
        production_convention,
        arguments,
        remaining_budget,
    )

    retained_indices = {int(sample["index"]) for sample in final_samples}
    final_object = [
        value
        for value, sample in zip(active_object, active_samples)
        if int(sample["index"]) in retained_indices
    ]
    final_image = [
        value
        for value, sample in zip(active_image, active_samples)
        if int(sample["index"]) in retained_indices
    ]
    (
        reprojection,
        camera_matrix,
        distortion,
        rvecs,
        tvecs,
    ) = _camera_calibration(
        session,
        final_object,
        final_image,
        image_size,
        refine_intrinsics=arguments.refine_camera_intrinsics,
    )
    view_errors = _per_view_reprojection_errors(
        final_object,
        final_image,
        rvecs,
        tvecs,
        camera_matrix,
        distortion,
    )
    candidates = _solve_candidates(
        final_samples,
        rvecs,
        tvecs,
        conventions=(production_convention,),
    )
    selected = candidates[0]
    try:
        diagnostic_candidates = _solve_candidates(
            final_samples,
            rvecs,
            tvecs,
            conventions=("extrinsic_xyz",),
        )
    except RuntimeError:
        diagnostic_candidates = []

    metrics = selected["metrics"]
    metrics.update(_image_coverage(final_samples))
    metrics.update(_leave_one_out_stability(final_samples, rvecs, tvecs, selected))
    metrics["camera_reprojection_rms_px"] = float(reprojection)
    metrics["camera_reprojection_max_view_px"] = float(max(view_errors))
    metrics["valid_samples"] = len(final_samples)
    metrics["detected_samples"] = len(detected_samples)
    rotation_component_axes = sum(
        value >= arguments.min_rotation_axis_span_deg
        for value in metrics["motion_rotation_axis_span_deg"]
    )
    rotation_principal_axes = sum(
        value >= arguments.min_rotation_principal_spread_deg
        for value in metrics["motion_rotation_principal_spread_deg"]
    )
    metrics["motion_rotation_component_axes_passing"] = int(rotation_component_axes)
    metrics["motion_rotation_principal_axes_passing"] = int(rotation_principal_axes)

    quality_checks = {
        "minimum_sample_count": len(final_samples) >= arguments.min_valid_samples,
        "camera_reprojection_rms": reprojection <= arguments.max_reprojection_px,
        "camera_reprojection_per_view": max(view_errors) <= arguments.max_view_reprojection_px,
        "translation_rms": metrics["translation_rms_mm"] <= arguments.max_translation_rms_mm,
        "translation_max": metrics["translation_max_mm"] <= arguments.max_translation_max_mm,
        "rotation_rms": metrics["rotation_rms_deg"] <= arguments.max_rotation_rms_deg,
        "rotation_max": metrics["rotation_max_deg"] <= arguments.max_rotation_max_deg,
        "translation_motion_span": (
            metrics["motion_translation_span_mm"] >= arguments.min_motion_translation_mm
        ),
        "rotation_motion_span": (
            metrics["motion_rotation_span_deg"] >= arguments.min_motion_rotation_deg
        ),
        "rotation_component_span": rotation_component_axes >= arguments.min_rotation_axes,
        "rotation_independent_axes": rotation_principal_axes >= arguments.min_rotation_axes,
        "image_coverage": metrics["image_coverage_bins"] >= arguments.min_image_coverage_bins,
        "leave_one_out_translation": (
            metrics["leave_one_out_translation_max_mm"] is not None
            and metrics["leave_one_out_translation_max_mm"] <= arguments.max_loo_translation_mm
        ),
        "leave_one_out_rotation": (
            metrics["leave_one_out_rotation_max_deg"] is not None
            and metrics["leave_one_out_rotation_max_deg"] <= arguments.max_loo_rotation_deg
        ),
    }
    accepted = all(quality_checks.values())
    rejected_samples = quality_rejections + reprojection_rejections + handeye_rejections
    result = {
        "accepted": bool(accepted),
        "acceptance_scope": (
            "internal dataset consistency only; independent 3x3 workspace validation is required"
        ),
        "transform_semantics": "p_tool0 = T_tool0_camera * p_camera",
        "parent_frame": "tool0_flange",
        "child_frame": "camera_color_optical_frame",
        "translation_unit": "m",
        "rotation": "3x3 matrix in row-major 4x4 transform",
        "camera_intrinsics_source": (
            "refined from hand-eye images"
            if arguments.refine_camera_intrinsics
            else "RealSense factory profile, fixed during hand-eye solve"
        ),
        "method": selected["method"],
        "rpy_convention": selected["rpy_convention"],
        "T_tool0_camera": selected["T_tool0_camera"].tolist(),
        "metrics": metrics,
        "quality_checks": quality_checks,
        "quality_thresholds": {
            "min_valid_samples": arguments.min_valid_samples,
            "max_reprojection_px": arguments.max_reprojection_px,
            "max_view_reprojection_px": arguments.max_view_reprojection_px,
            "max_translation_rms_mm": arguments.max_translation_rms_mm,
            "max_translation_max_mm": arguments.max_translation_max_mm,
            "max_rotation_rms_deg": arguments.max_rotation_rms_deg,
            "max_rotation_max_deg": arguments.max_rotation_max_deg,
            "min_motion_translation_mm": arguments.min_motion_translation_mm,
            "min_motion_rotation_deg": arguments.min_motion_rotation_deg,
            "min_rotation_axis_span_deg": arguments.min_rotation_axis_span_deg,
            "min_rotation_principal_spread_deg": (
                arguments.min_rotation_principal_spread_deg
            ),
            "min_rotation_axes": arguments.min_rotation_axes,
            "min_image_coverage_bins": arguments.min_image_coverage_bins,
            "min_board_area_ratio": arguments.min_board_area_ratio,
            "max_loo_translation_mm": arguments.max_loo_translation_mm,
            "max_loo_rotation_deg": arguments.max_loo_rotation_deg,
        },
        "outlier_rejection": {
            "enabled": not arguments.disable_outlier_rejection,
            "maximum_fraction": arguments.max_outlier_fraction,
            "maximum_count": rejection_budget,
            "rejected_samples": rejected_samples,
            "retained_sample_indices": [int(sample["index"]) for sample in final_samples],
        },
        "camera_matrix": camera_matrix.tolist(),
        "distortion": distortion.reshape(-1).tolist(),
    }

    output_dir = dataset / "calibration"
    output_dir.mkdir(exist_ok=True)
    output_name = "handeye_accepted.yaml" if accepted else "handeye_candidate_rejected.yaml"
    stale_name = "handeye_candidate_rejected.yaml" if accepted else "handeye_accepted.yaml"
    (output_dir / stale_name).unlink(missing_ok=True)
    with (output_dir / output_name).open("w", encoding="utf-8") as stream:
        yaml.safe_dump(result, stream, allow_unicode=True, sort_keys=False)
    report = {
        "selected": {
            key: value
            for key, value in result.items()
            if key not in {"camera_matrix", "distortion"}
        },
        "production_candidates": [
            {
                "method": candidate["method"],
                "rpy_convention": candidate["rpy_convention"],
                "score": candidate["score"],
                "metrics": candidate["metrics"],
            }
            for candidate in candidates
        ],
        "diagnostic_candidates_not_eligible_for_selection": [
            {
                "method": candidate["method"],
                "rpy_convention": candidate["rpy_convention"],
                "score": candidate["score"],
                "metrics": candidate["metrics"],
            }
            for candidate in diagnostic_candidates
        ],
    }
    (output_dir / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(yaml.safe_dump(result, allow_unicode=True, sort_keys=False))
    print(f"Result file: {output_dir / output_name}")
    if not accepted:
        failed = [name for name, passed in quality_checks.items() if not passed]
        print(f"Calibration rejected; failed checks: {', '.join(failed)}")
        print("Do not use this transform for automatic grasping.")
        raise SystemExit(2)


def main(argv=None) -> None:
    _precision_main(argv)


if __name__ == "__main__":
    main()
