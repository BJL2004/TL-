import argparse
from pathlib import Path

import numpy as np
import yaml

from .geometry import pose_to_transform


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert flange-based hand-eye calibration to an installed TCP frame"
    )
    parser.add_argument("handeye_yaml", type=Path)
    parser.add_argument(
        "--flange-to-tool",
        nargs=6,
        type=float,
        required=True,
        metavar=("X_MM", "Y_MM", "Z_MM", "RX_RAD", "RY_RAD", "RZ_RAD"),
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--allow-rejected",
        action="store_true",
        help="diagnostic use only: convert a rejected source calibration",
    )
    parser.add_argument(
        "--rpy-convention",
        choices=["intrinsic_xyz", "extrinsic_xyz"],
        default="intrinsic_xyz",
    )
    return parser.parse_args(argv)


def main(argv=None) -> None:
    arguments = parse_args(argv)
    source = yaml.safe_load(arguments.handeye_yaml.expanduser().read_text(encoding="utf-8"))
    if not bool(source.get("accepted", False)) and not arguments.allow_rejected:
        raise RuntimeError(
            "source hand-eye calibration is rejected; conversion is blocked. "
            "Use --allow-rejected only for non-motion diagnostics."
        )
    source_convention = source.get("rpy_convention", "intrinsic_xyz")
    if source_convention != arguments.rpy_convention:
        raise RuntimeError(
            f"RPY convention mismatch: source={source_convention}, "
            f"flange-to-tool={arguments.rpy_convention}"
        )
    flange_camera = np.asarray(source["T_tool0_camera"], dtype=float).reshape(4, 4)
    pose = arguments.flange_to_tool
    flange_tool = pose_to_transform(
        np.asarray(pose[:3], dtype=float) / 1000.0,
        pose[3:],
        arguments.rpy_convention,
    )
    tool_camera = np.linalg.inv(flange_tool) @ flange_camera
    result = {
        "accepted": bool(source.get("accepted", False)),
        "transform_semantics": "p_tool = T_tool_camera * p_camera",
        "parent_frame": "configured_grasp_tcp",
        "child_frame": source.get("child_frame", "camera_color_optical_frame"),
        "translation_unit": "m",
        "flange_to_tool_mm_rad": list(pose),
        "T_tool_camera": tool_camera.tolist(),
        "source_handeye": str(arguments.handeye_yaml.expanduser()),
        "source_method": source.get("method"),
        "source_rpy_convention": source_convention,
        "source_metrics": source.get("metrics", {}),
        "source_quality_checks": source.get("quality_checks", {}),
    }
    output = arguments.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(result, allow_unicode=True, sort_keys=False))
