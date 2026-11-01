import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .geometry import make_transform, pose_to_transform
from .handeye_calibrate import (
    _camera_calibration,
    _detect,
    _evaluate,
    _load_dataset,
    _solve_candidates,
)
from .reference_compat import transform_delta


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Compare TL_Dobot calibration with the TL_TCB Tsai/intrinsic-XYZ procedure"
    )
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--max-translation-rms-mm", type=float, default=8.0)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=2.0)
    return parser.parse_args(argv)


def _find_calibration(dataset: Path, explicit_path):
    if explicit_path is not None:
        return explicit_path.expanduser().resolve()
    accepted = dataset / "calibration" / "handeye_accepted.yaml"
    rejected = dataset / "calibration" / "handeye_candidate_rejected.yaml"
    if accepted.exists():
        return accepted
    if rejected.exists():
        return rejected
    raise RuntimeError("no hand-eye calibration YAML found")


def _camera_targets(rvecs, tvecs):
    return [
        make_transform(cv2.Rodrigues(rvec)[0], tvec.reshape(3))
        for rvec, tvec in zip(rvecs, tvecs)
    ]


def _base_gripper(samples):
    return [
        pose_to_transform(
            np.asarray(sample["tcp_pose_mm_rad"][:3], dtype=float) / 1000.0,
            sample["tcp_pose_mm_rad"][3:],
            "intrinsic_xyz",
        )
        for sample in samples
    ]


def _reference_detect(dataset, session, samples):
    checkerboard = session["checkerboard"]
    cols = int(checkerboard["pattern_cols"])
    rows = int(checkerboard["pattern_rows"])
    square_size = float(checkerboard["square_size_m"])
    object_template = np.zeros((cols * rows, 3), np.float32)
    object_template[:, :2] = (
        np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    )
    criteria = (
        cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS,
        30,
        0.001,
    )
    object_points = []
    image_points = []
    valid_samples = []
    image_size = None
    for sample in samples:
        image = cv2.imread(str(dataset / sample["image"]))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if not found:
            continue
        refined = cv2.cornerSubPix(
            gray, corners, (5, 5), (-1, -1), criteria
        )
        object_points.append(object_template.copy())
        image_points.append(refined if refined is not None else corners)
        valid_samples.append(sample)
    if len(valid_samples) < 5:
        raise RuntimeError(
            f"TL_TCB reference detector found only {len(valid_samples)} checkerboards"
        )
    return object_points, image_points, valid_samples, image_size


def _reference_solve(object_points, image_points, image_size, samples):
    reprojection, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    camera_target = _camera_targets(rvecs, tvecs)
    base_gripper = _base_gripper(samples)
    rotation, translation = cv2.calibrateHandEye(
        [transform[:3, :3] for transform in base_gripper],
        [transform[:3, 3] for transform in base_gripper],
        [transform[:3, :3] for transform in camera_target],
        [transform[:3, 3] for transform in camera_target],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    transform = make_transform(rotation, translation.reshape(3))
    metrics = _evaluate(base_gripper, camera_target, transform)
    metrics["camera_reprojection_rms_px"] = float(reprojection)
    return transform, metrics, camera_matrix, distortion


def _plain_value(value):
    if isinstance(value, dict):
        return {key: _plain_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain_value(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def _plain_metrics(metrics):
    return _plain_value(metrics)


def main(argv=None) -> None:
    arguments = parse_args(argv)
    dataset = arguments.dataset.expanduser().resolve()
    calibration_path = _find_calibration(dataset, arguments.calibration)
    calibration = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    current_transform = np.asarray(
        calibration["T_tool0_camera"], dtype=float
    ).reshape(4, 4)

    session, raw_samples = _load_dataset(dataset)
    object_points, image_points, production_samples, image_size = _detect(
        dataset, session, raw_samples
    )
    production_reprojection, _, _, production_rvecs, production_tvecs = (
        _camera_calibration(
            session,
            object_points,
            image_points,
            image_size,
            refine_intrinsics=False,
        )
    )
    production_candidates = _solve_candidates(
        production_samples, production_rvecs, production_tvecs
    )
    production_selected = production_candidates[0]
    production_metrics = dict(production_selected["metrics"])
    production_metrics["camera_reprojection_rms_px"] = float(
        production_reprojection
    )

    current_metrics = _evaluate(
        _base_gripper(production_samples),
        _camera_targets(production_rvecs, production_tvecs),
        current_transform,
    )
    current_metrics["camera_reprojection_rms_px"] = float(
        production_reprojection
    )
    reference_object, reference_image, reference_samples, reference_size = (
        _reference_detect(dataset, session, raw_samples)
    )
    reference_transform, reference_metrics, reference_camera, reference_distortion = (
        _reference_solve(
            reference_object,
            reference_image,
            reference_size,
            reference_samples,
        )
    )
    current_vs_reference = transform_delta(current_transform, reference_transform)
    current_vs_recomputed = transform_delta(
        current_transform, production_selected["T_tool0_camera"]
    )
    passes_repeatability = (
        current_metrics["translation_rms_mm"]
        <= arguments.max_translation_rms_mm
        and current_metrics["rotation_rms_deg"]
        <= arguments.max_rotation_rms_deg
    )

    report = {
        "warning": (
            "This evaluates consistency on the checkerboard dataset, not absolute "
            "tool-to-object positioning accuracy."
        ),
        "dataset": str(dataset),
        "production_valid_samples": len(production_samples),
        "tl_tcb_reference_valid_samples": len(reference_samples),
        "current_file": {
            "path": str(calibration_path),
            "accepted_in_file": bool(calibration.get("accepted", False)),
            "method": calibration.get("method"),
            "rpy_convention": calibration.get("rpy_convention"),
            "metrics_recomputed_with_factory_intrinsics": _plain_metrics(
                current_metrics
            ),
            "passes_repeatability_threshold": bool(passes_repeatability),
        },
        "tl_dobot_recomputed": {
            "method": production_selected["method"],
            "rpy_convention": production_selected["rpy_convention"],
            "T_tool0_camera": production_selected["T_tool0_camera"].tolist(),
            "metrics": _plain_metrics(production_metrics),
            "delta_from_current_translation_mm": current_vs_recomputed[0],
            "delta_from_current_rotation_deg": current_vs_recomputed[1],
        },
        "tl_tcb_reference_recomputed": {
            "method": "tsai",
            "rpy_convention": "intrinsic_xyz",
            "camera_intrinsics_source": "cv2.calibrateCamera from checkerboard images",
            "T_tool0_camera": reference_transform.tolist(),
            "metrics": _plain_metrics(reference_metrics),
            "delta_from_current_translation_mm": current_vs_reference[0],
            "delta_from_current_rotation_deg": current_vs_reference[1],
            "camera_matrix": reference_camera.tolist(),
            "distortion": reference_distortion.reshape(-1).tolist(),
        },
        "production_candidates": [
            {
                "method": candidate["method"],
                "rpy_convention": candidate["rpy_convention"],
                "score": float(candidate["score"]),
                "metrics": _plain_metrics(candidate["metrics"]),
            }
            for candidate in production_candidates
        ],
    }
    output_path = dataset / "calibration" / "handeye_accuracy_comparison.json"
    output_path.parent.mkdir(exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print("Hand-eye consistency comparison")
    print(f"dataset: {dataset}")
    print(
        f"valid samples: TL_Dobot={len(production_samples)}, "
        f"TL_TCB={len(reference_samples)}"
    )
    print(
        "current: "
        f"translation RMS={current_metrics['translation_rms_mm']:.3f} mm, "
        f"max={current_metrics['translation_max_mm']:.3f} mm, "
        f"rotation RMS={current_metrics['rotation_rms_deg']:.3f} deg"
    )
    print(
        "TL_TCB Tsai/intrinsic_xyz: "
        f"translation RMS={reference_metrics['translation_rms_mm']:.3f} mm, "
        f"max={reference_metrics['translation_max_mm']:.3f} mm, "
        f"rotation RMS={reference_metrics['rotation_rms_deg']:.3f} deg"
    )
    print(
        "current vs TL_TCB recomputed: "
        f"translation delta={current_vs_reference[0]:.3f} mm, "
        f"rotation delta={current_vs_reference[1]:.3f} deg"
    )
    status = "PASS" if passes_repeatability else "REJECT"
    print(f"repeatability threshold: {status}")
    print("Absolute positioning still requires the no-gripper overhead offset test.")
    print(f"report: {output_path}")
    if not passes_repeatability:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
