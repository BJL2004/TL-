import argparse
import json
import math
from pathlib import Path

import numpy as np


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Strictly accept or reject uncompensated 3x3 hand-eye validation records"
    )
    parser.add_argument("validation_log", type=Path)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--min-samples", type=int, default=9)
    parser.add_argument("--min-workspace-span-mm", type=float, default=100.0)
    parser.add_argument("--max-xy-rms-mm", type=float, default=5.0)
    parser.add_argument("--max-xy-error-mm", type=float, default=8.0)
    parser.add_argument("--max-z-rms-mm", type=float, default=5.0)
    parser.add_argument("--max-z-error-mm", type=float, default=8.0)
    return parser.parse_args(argv)


def load_records(path: Path, calibration=None):
    records = []
    for line in path.expanduser().read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        record_calibration = Path(record.get("calibration", "")).expanduser().resolve()
        if calibration is None or record_calibration == calibration:
            records.append(record)
    if not records:
        raise RuntimeError("no matching validation records")
    return records


def evaluate_records(records, arguments):
    calibrations = {str(record.get("calibration", "")) for record in records}
    uncompensated = all(
        np.linalg.norm(np.asarray(record.get("base_xy_bias_mm", [0.0, 0.0]), dtype=float))
        <= 0.1
        and abs(float(record.get("base_z_bias_mm", float("inf")))) <= 0.1
        for record in records
    )
    no_mock_tool = all("mock_grasp_offset_tool0_mm" not in record for record in records)
    corrections = np.asarray([record["correction_base_mm"] for record in records], dtype=float)
    positions = np.asarray(
        [record["estimated_object_base_mm"] for record in records],
        dtype=float,
    )
    xy_errors = np.linalg.norm(corrections[:, :2], axis=1)
    z_errors = np.abs(corrections[:, 2])
    workspace_span = np.ptp(positions[:, :2], axis=0)
    metrics = {
        "samples": len(records),
        "calibrations": sorted(calibrations),
        "mean_correction_base_mm": [float(value) for value in corrections.mean(axis=0)],
        "xy_rms_mm": float(math.sqrt(np.mean(xy_errors**2))),
        "xy_max_mm": float(max(xy_errors)),
        "z_rms_mm": float(math.sqrt(np.mean(z_errors**2))),
        "z_max_mm": float(max(z_errors)),
        "workspace_span_xy_mm": [float(value) for value in workspace_span],
    }
    checks = {
        "single_calibration": len(calibrations) == 1 and "" not in calibrations,
        "uncompensated_measurements": uncompensated,
        "no_mock_tool_offset": no_mock_tool,
        "minimum_sample_count": len(records) >= arguments.min_samples,
        "workspace_x_span": workspace_span[0] >= arguments.min_workspace_span_mm,
        "workspace_y_span": workspace_span[1] >= arguments.min_workspace_span_mm,
        "xy_rms": metrics["xy_rms_mm"] <= arguments.max_xy_rms_mm,
        "xy_max": metrics["xy_max_mm"] <= arguments.max_xy_error_mm,
        "z_rms": metrics["z_rms_mm"] <= arguments.max_z_rms_mm,
        "z_max": metrics["z_max_mm"] <= arguments.max_z_error_mm,
    }
    return {
        "accepted": all(checks.values()),
        "acceptance_scope": "uncompensated 3x3 workspace pointer validation",
        "metrics": metrics,
        "quality_checks": checks,
        "thresholds": {
            "min_samples": arguments.min_samples,
            "min_workspace_span_mm": arguments.min_workspace_span_mm,
            "max_xy_rms_mm": arguments.max_xy_rms_mm,
            "max_xy_error_mm": arguments.max_xy_error_mm,
            "max_z_rms_mm": arguments.max_z_rms_mm,
            "max_z_error_mm": arguments.max_z_error_mm,
        },
    }


def main(argv=None):
    arguments = parse_args(argv)
    log_path = arguments.validation_log.expanduser().resolve()
    calibration = (
        arguments.calibration.expanduser().resolve()
        if arguments.calibration is not None
        else None
    )
    records = load_records(log_path, calibration)
    result = evaluate_records(records, arguments)
    output = log_path.with_suffix(".acceptance.json")
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"Acceptance report: {output}")
    if not result["accepted"]:
        failed = [name for name, passed in result["quality_checks"].items() if not passed]
        print(f"Validation rejected; failed checks: {', '.join(failed)}")
        raise SystemExit(2)


if __name__ == "__main__":
    main()
