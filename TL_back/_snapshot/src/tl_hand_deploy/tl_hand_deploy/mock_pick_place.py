import argparse
import math
from pathlib import Path
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import yaml
from std_srvs.srv import SetBool, Trigger
from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam
from tl_ros2_interface.srv import ModbusWrite

from .basket_aruco import (
    basket_teach_status,
    base_marker_transform,
    camera_parameters,
    detect_marker_transforms,
    draw_marker_overlay,
    fuse_taught_target,
    group_and_average_marker_samples,
    marker_transforms_with_depth_translation,
    select_stable_marker_nearest_center,
    translated_taught_target,
)
from .geometry import pose_to_transform, transform_to_pose
from .overhead_test import ReferenceOverheadTest, load_calibration
from .reference_compat import (
    approach_point_above_reachable_grasp,
    camera_point_to_base,
    tool_pose_for_virtual_tcp,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Mock-hand pick-and-place into a taught basket pose"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path("~/tl_handeye_data/best/handeye_best.yaml"),
    )
    parser.add_argument(
        "--place-file",
        type=Path,
        default=Path("~/tl_mock_pick_place/place_pose.yaml"),
    )
    parser.add_argument(
        "--basket-aruco-file",
        type=Path,
        default=None,
        help="use marker-relative dynamic basket poses instead of fixed place poses",
    )
    parser.add_argument("--marker-samples", type=int, default=30)
    parser.add_argument("--marker-minimum-samples", type=int, default=8)
    parser.add_argument("--marker-fusion-max-mm", type=float, default=30.0)
    parser.add_argument(
        "--basket-motion-mode",
        choices=("translation", "rigid"),
        default="translation",
        help="translation keeps taught tool orientation; rigid follows full marker PnP pose",
    )
    parser.add_argument("--max-basket-shift-mm", type=float, default=200.0)
    parser.add_argument(
        "--basket-number",
        type=int,
        choices=range(1, 7),
        default=None,
        help="independent basket number 1..6 to localize and place into",
    )
    parser.add_argument("--base-xy-bias-mm", type=float, nargs=2, default=[-53.2, -34.2])
    parser.add_argument("--grasp-rpy-rad", type=float, nargs=3, default=[-2.043, 1.471, -1.025])
    parser.add_argument("--mock-grasp-offset-mm", type=float, nargs=3, default=[-6.3, 34.0, 42.5])
    parser.add_argument("--pick-approach-clearance-mm", type=float, default=130.0)
    parser.add_argument("--pick-grasp-clearance-mm", type=float, default=50.0)
    parser.add_argument(
        "--final-grasp-z-mm",
        type=float,
        default=None,
        help="absolute tool0 Z in BASE mm for the grasp pose, tuned with the "
        "overhead_test g-key descent; overrides --pick-grasp-clearance-mm",
    )
    parser.add_argument("--mock-grasp-clearance-mm", type=float, default=20.0)
    parser.add_argument(
        "--min-mock-clearance-mm",
        type=float,
        default=None,
        help="lower bound for the mock grasp clearance during fine adjustment; "
        "defaults to --mock-grasp-clearance-mm",
    )
    parser.add_argument(
        "--min-tool-z-mm",
        type=float,
        default=-150.0,
        help="hard floor for the target tool0 Z in BASE frame mm; the work surface "
        "sits around -120..-135, so keep this low for auto-lift to reach it",
    )
    parser.add_argument("--min-target-depth-m", type=float, default=0.15)
    parser.add_argument("--max-target-depth-m", type=float, default=0.65)
    parser.add_argument("--interp-points", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--servoj-speed-deg-s", type=float, default=20.0)
    parser.add_argument("--servoj-precision-speed-deg-s", type=float, default=8.0)
    parser.add_argument("--servoj-min-duration-sec", type=float, default=1.0)
    parser.add_argument("--movej-velocity", type=float, default=5.0)
    parser.add_argument("--movej-acceleration", type=float, default=5.0)
    parser.add_argument("--movej-deceleration", type=float, default=5.0)
    parser.add_argument("--movej-max-retries", type=int, default=2)
    parser.add_argument("--movej-retry-max-error-deg", type=float, default=10.0)
    parser.add_argument("--movej-status-grace-sec", type=float, default=2.0)
    parser.add_argument("--movej-stall-sec", type=float, default=2.0)
    parser.add_argument("--movej-progress-log-sec", type=float, default=5.0)
    parser.add_argument("--arrival-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--max-joint-delta-deg", type=float, default=220.0)
    parser.add_argument("--auto-lift-to-reachable", action="store_true")
    parser.add_argument("--max-auto-lift-mm", type=float, default=200.0)
    parser.add_argument("--fine-step-mm", type=float, default=1.0)
    parser.add_argument("--fine-z-step-mm", type=float, default=5.0)
    parser.add_argument("--fine-interp-points", type=int, default=100)
    parser.add_argument(
        "--observation-joints",
        type=float,
        nargs=6,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="move to this fixed six-axis observation pose at startup and use it for return",
    )
    parser.add_argument(
        "--home-joint-deg",
        type=float,
        nargs=6,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="home pose in degrees: start here, return here between pick and place "
        "and after the cycle, instead of the observation pose",
    )
    parser.add_argument(
        "--validation-log",
        type=Path,
        default=Path("~/tl_handeye_validation/pick_place_offsets.jsonl"),
    )
    parser.add_argument("--allow-rejected", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--use-hand",
        action="store_true",
        help="call /tl_hand/open and /tl_hand/grasp; keep disabled for fake-hand tests",
    )
    parser.add_argument(
        "--hand-grasp-positions",
        type=int,
        nargs=6,
        default=[35000, 35000, 35000, 35000, 35000, 25000],
        help="ROH-LiteS001 6-finger target positions for grasp (level 5 strong)",
    )
    parser.add_argument(
        "--hand-open-positions",
        type=int,
        nargs=6,
        default=[0, 0, 0, 0, 0, 0],
        help="ROH-LiteS001 6-finger target positions for open/release",
    )
    parser.add_argument(
        "--target-only",
        action="store_true",
        help="disable basket loading and keep only click, calculate, move, and return",
    )
    parser.add_argument(
        "--place-only",
        action="store_true",
        help="skip pick steps: localize the basket, move above it, release, and return",
    )
    parser.add_argument("--hand-settle-sec", type=float, default=1.5)
    arguments = parser.parse_args(argv)
    if arguments.pick_grasp_clearance_mm < 0.0:
        parser.error("--pick-grasp-clearance-mm cannot be negative")
    if arguments.pick_approach_clearance_mm < arguments.pick_grasp_clearance_mm:
        parser.error("--pick-approach-clearance-mm must be >= --pick-grasp-clearance-mm")
    if arguments.max_joint_delta_deg <= 0.0:
        parser.error("--max-joint-delta-deg must be positive")
    if arguments.marker_samples < arguments.marker_minimum_samples:
        parser.error("--marker-samples must be >= --marker-minimum-samples")
    if arguments.max_basket_shift_mm <= 0.0:
        parser.error("--max-basket-shift-mm must be positive")
    if arguments.interp_points != 0 and arguments.interp_points < 100:
        parser.error("--interp-points must be 0 (automatic) or at least 100")
    if arguments.rate_hz <= 0.0 or arguments.rate_hz > 200.0:
        parser.error("--rate-hz must be in (0, 200]")
    if arguments.servoj_speed_deg_s <= 0.0 or arguments.servoj_speed_deg_s > 45.0:
        parser.error("--servoj-speed-deg-s must be in (0, 45]")
    if (
        arguments.servoj_precision_speed_deg_s <= 0.0
        or arguments.servoj_precision_speed_deg_s > arguments.servoj_speed_deg_s
    ):
        parser.error(
            "--servoj-precision-speed-deg-s must be positive and no greater than "
            "--servoj-speed-deg-s"
        )
    if arguments.servoj_min_duration_sec <= 0.0:
        parser.error("--servoj-min-duration-sec must be positive")
    for name in ("movej_velocity", "movej_acceleration", "movej_deceleration"):
        value = float(getattr(arguments, name))
        if value <= 0.0 or value > 100.0:
            parser.error(f"--{name.replace('_', '-')} must be in (0, 100]")
    if arguments.movej_max_retries < 0 or arguments.movej_max_retries > 5:
        parser.error("--movej-max-retries must be in [0, 5]")
    if arguments.movej_retry_max_error_deg <= 0.0:
        parser.error("--movej-retry-max-error-deg must be positive")
    if arguments.movej_status_grace_sec < 0.0:
        parser.error("--movej-status-grace-sec cannot be negative")
    if arguments.movej_stall_sec <= 0.0:
        parser.error("--movej-stall-sec must be positive")
    if arguments.movej_progress_log_sec <= 0.0:
        parser.error("--movej-progress-log-sec must be positive")

    # ReferenceOverheadTest expects these attributes.
    arguments.hover_mm = arguments.pick_grasp_clearance_mm
    arguments.pointer_length_mm = 0.0
    arguments.min_tip_clearance_mm = 0.0
    arguments.mock_grasp_clearance_mm = arguments.pick_grasp_clearance_mm
    return arguments


class MockPickPlaceNode(ReferenceOverheadTest):
    def __init__(self, arguments, calibration_path, calibration, tool_camera):
        super().__init__(arguments, calibration_path, calibration, tool_camera)
        self.basket_aruco = (
            self._load_basket_aruco(
                arguments.basket_aruco_file, arguments.basket_number
            )
            if not arguments.target_only and arguments.basket_aruco_file is not None
            else None
        )
        self.place_poses = (
            None
            if arguments.target_only or self.basket_aruco is not None
            else self._load_place_poses(arguments.place_file)
        )
        self.dynamic_place_joints = {}
        self.dynamic_place_poses = {}
        self.detect_joint = None
        self.pick_approach_joint = None
        self.pick_grasp_joint = None
        self.pick_approach_pose = None
        self.pick_grasp_pose = None
        self.pick_object_base_mm = None
        self.hand_enable_client = self.create_client(SetBool, "/tl_hand/set_enabled")
        self.hand_open_client = self.create_client(Trigger, "/tl_hand/open")
        self.hand_grasp_client = self.create_client(Trigger, "/tl_hand/grasp")
        self.hand_stop_client = self.create_client(Trigger, "/tl_hand/stop")
        self.hand_modbus_client = self.create_client(
            ModbusWrite, "/tl_driver/modbus_write"
        )
        self.pending_basket_id = None
        self.latest_visible_basket_ids = []
        self.pick_complete = False
        self.place_done = False

    @staticmethod
    def _load_basket_aruco(path: Path, basket_number: int | None) -> dict:
        config_path = path.expanduser().resolve()
        if not config_path.exists():
            raise RuntimeError(
                f"basket ArUco file is missing: {config_path}. Run the ArUco basket "
                "teach program first."
            )
        data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        teach_status = basket_teach_status(data)
        usable = [
            marker_id
            for marker_id, marker_status in sorted(teach_status.items())
            if marker_status["complete"]
        ]
        if not usable:
            raise RuntimeError(
                f"{config_path} has no complete version-2 dynamic basket record; "
                "re-teach with B, A, P, R using the current teach script"
            )
        if basket_number is None:
            selected_id = None
            incomplete = [
                marker_id + 1
                for marker_id, marker_status in sorted(teach_status.items())
                if not marker_status["complete"]
            ]
            if incomplete:
                raise RuntimeError(
                    "live auto basket selection requires all six baskets to have "
                    "version-2 search references, approach, and release poses; "
                    f"incomplete numbers={incomplete}"
                )
        else:
            selected_id = int(basket_number) - 1
            if selected_id not in usable:
                raise RuntimeError(
                    f"basket number {basket_number} has no complete version-2 dynamic "
                    f"teaching record; available numbers are "
                    f"{[marker_id + 1 for marker_id in sorted(usable)]}"
                )
        data["_path"] = str(config_path)
        data["_usable_ids"] = usable
        data["_configured_id"] = selected_id
        data["_selected_id"] = selected_id
        print(f"dynamic basket ArUco file={config_path}")
        print(f"taught marker numbers={[marker_id + 1 for marker_id in sorted(usable)]}")
        if selected_id is None:
            print("basket selection=live vision auto (nearest stable marker to image center)")
        else:
            print(f"configured independent basket number={selected_id + 1}")
        return data

    def _basket_record(self, marker_id: int) -> dict:
        return self.basket_aruco["markers"].get(
            str(marker_id), self.basket_aruco["markers"].get(marker_id, {})
        )

    def select_basket_number(self, basket_number: int) -> None:
        if self.basket_aruco is None:
            raise RuntimeError("dynamic basket mode is not enabled")
        if self.basket_aruco["_configured_id"] is not None:
            raise RuntimeError(
                "fixed --basket-number is active; restart without TL_BASKET_NUMBER "
                "to select from the preview"
            )
        marker_id = int(basket_number) - 1
        if marker_id not in self.basket_aruco["_usable_ids"]:
            raise RuntimeError(
                f"basket number {basket_number} is not fully taught; available numbers are "
                f"{[value + 1 for value in sorted(self.basket_aruco['_usable_ids'])]}"
            )
        self.pending_basket_id = marker_id
        self.basket_aruco["_selected_id"] = None
        self.dynamic_place_joints = {}
        self.dynamic_place_poses = {}
        visibility = "visible now" if marker_id in self.latest_visible_basket_ids else "not visible now"
        print(
            f"Manual basket candidate number={basket_number} ({visibility}). "
            "Press b to localize and lock it."
        )

    def reset_live_basket_selection(self) -> None:
        if self.basket_aruco is None:
            raise RuntimeError("dynamic basket mode is not enabled")
        if self.basket_aruco["_configured_id"] is not None:
            raise RuntimeError(
                "fixed --basket-number is active; restart without TL_BASKET_NUMBER for auto mode"
            )
        self.pending_basket_id = None
        self.basket_aruco["_selected_id"] = None
        self.dynamic_place_joints = {}
        self.dynamic_place_poses = {}
        print("Basket selection reset to live vision auto. Aim at a basket and press b.")

    def _discover_visible_basket(
        self, camera_matrix: np.ndarray, distortion: np.ndarray, marker_length_m: float
    ) -> int:
        allowed_ids = self.basket_aruco["_usable_ids"]
        center_samples = []
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and len(center_samples) < int(self.arguments.marker_samples):
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.0)
            try:
                frames = self.align.process(
                    self.pipeline.wait_for_frames(timeout_ms=2000)
                )
            except RuntimeError:
                continue
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            image = np.asanyarray(color_frame.get_data())
            _, corners, ids, _ = detect_marker_transforms(
                image,
                camera_matrix,
                distortion,
                marker_length_m,
                allowed_ids,
            )
            if ids.size == 0:
                continue
            centers = {}
            for marker_corners, marker_id_value in zip(corners, ids.reshape(-1)):
                marker_id = int(marker_id_value)
                centers[marker_id] = np.mean(
                    np.asarray(marker_corners, dtype=float).reshape(4, 2), axis=0
                )
            center_samples.append(centers)
        selected_id, metrics = select_stable_marker_nearest_center(
            center_samples,
            self.arguments.marker_minimum_samples,
            self.color_intrinsics.width,
            self.color_intrinsics.height,
        )
        summary = ", ".join(
            f"N{marker_id + 1}:samples={int(values['samples'])},"
            f"center_distance={values['center_distance_px']:.1f}px"
            for marker_id, values in sorted(metrics.items())
        )
        print(f"Live basket candidates: {summary}")
        print(f"Vision selected and locked basket number={selected_id + 1}")
        return selected_id

    def _discover_basket_across_taught_views(
        self, camera_matrix: np.ndarray, distortion: np.ndarray, marker_length_m: float
    ) -> int:
        current = np.asarray(self.latest_joint[:6], dtype=float)
        candidates = []
        for marker_id in self.basket_aruco["_usable_ids"]:
            record = self._basket_record(marker_id)
            search_pose = record.get("search_pose")
            if not search_pose or "joint" not in search_pose:
                continue
            search_joint = self._pad_joint(search_pose["joint"])
            distance = float(
                np.max(np.abs(np.asarray(search_joint[:6], dtype=float) - current))
            )
            candidates.append((distance, int(marker_id), search_joint))
        if len(candidates) != 6:
            raise RuntimeError(
                f"automatic rack scan needs six taught search poses; found {len(candidates)}"
            )
        failures = []
        for _, expected_id, search_joint in sorted(candidates):
            print(
                f"Auto rack scan: moving to taught view for basket number={expected_id + 1}."
            )
            self.move_joint(search_joint)
            try:
                return self._discover_visible_basket(
                    camera_matrix, distortion, marker_length_m
                )
            except RuntimeError as error:
                failures.append(f"N{expected_id + 1}: {error}")
                print(f"No stable taught marker in this view: {error}")
        raise RuntimeError(
            "automatic rack scan visited all six taught views but found no stable basket; "
            + " | ".join(failures)
        )

    @staticmethod
    def _load_place_poses(path: Path) -> dict:
        place_path = path.expanduser().resolve()
        if not place_path.exists():
            raise RuntimeError(
                f"place file is missing: {place_path}. Teach basket_approach and "
                "basket_release before running pick-place."
            )
        data = yaml.safe_load(place_path.read_text(encoding="utf-8")) or {}
        poses = data.get("poses", {})
        missing = [name for name in ["basket_approach", "basket_release"] if name not in poses]
        if missing:
            raise RuntimeError(f"place file {place_path} is missing poses: {missing}")
        for name in ["basket_approach", "basket_release"]:
            if "joint" not in poses[name] or "tcp" not in poses[name]:
                raise RuntimeError(f"pose {name} must contain both joint and tcp")
        print(f"place file={place_path}")
        print(f"basket_approach tcp={np.round(poses['basket_approach']['tcp'], 3).tolist()}")
        print(f"basket_release  tcp={np.round(poses['basket_release']['tcp'], 3).tolist()}")
        return poses

    def _place_joint(self, name: str) -> list[float]:
        if self.basket_aruco is not None:
            if name not in self.dynamic_place_joints:
                raise RuntimeError("press b to localize the basket before running the cycle")
            return self._pad_joint(self.dynamic_place_joints[name])
        return self._pad_joint(self.place_poses[name]["joint"])

    def localize_basket(self) -> None:
        if self.basket_aruco is None:
            raise RuntimeError("dynamic basket mode is not enabled")
        camera_matrix, distortion = camera_parameters(self.color_intrinsics)
        marker_length_m = float(self.basket_aruco.get("marker_length_m", 0.06))
        configured_id = self.basket_aruco["_configured_id"]
        if configured_id is not None:
            selected_id = int(configured_id)
            print(f"Using configured basket number={selected_id + 1}")
        elif self.pending_basket_id is not None:
            selected_id = int(self.pending_basket_id)
            print(f"Using manually selected basket number={selected_id + 1}")
        else:
            print("Scanning the six taught rack views; no basket number is fixed.")
            selected_id = self._discover_basket_across_taught_views(
                camera_matrix, distortion, marker_length_m
            )
        self.basket_aruco["_selected_id"] = selected_id
        marker_record = self._basket_record(selected_id)
        search_pose = marker_record.get(
            "search_pose", self.basket_aruco.get("search_pose")
        )
        if not search_pose or "joint" not in search_pose:
            raise RuntimeError(
                f"basket number {selected_id + 1} has no search pose; teach it with B"
            )
        taught_marker_length_m = float(search_pose.get("marker_length_m", 0.0))
        if not math.isclose(
            taught_marker_length_m, marker_length_m, rel_tol=0.0, abs_tol=1.0e-6
        ):
            raise RuntimeError(
                f"basket number {selected_id + 1} was taught with marker length "
                f"{taught_marker_length_m * 1000.0:.1f} mm but runtime uses "
                f"{marker_length_m * 1000.0:.1f} mm; re-teach it instead of editing "
                "the YAML top-level marker_length_m"
            )
        search_joint = self._pad_joint(search_pose["joint"])
        print(f"Moving to the taught search pose for basket number={selected_id + 1}.")
        self.move_joint(search_joint)
        allowed_ids = [selected_id]
        samples = []
        width_samples = []
        tilt_samples = []
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and len(samples) < int(self.arguments.marker_samples):
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.0)
            try:
                frames = self.align.process(
                    self.pipeline.wait_for_frames(timeout_ms=2000)
                )
            except RuntimeError:
                continue
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame or self.latest_tcp is None:
                continue
            image = np.asanyarray(color_frame.get_data())
            camera_markers, marker_corners, marker_ids, _ = detect_marker_transforms(
                image,
                camera_matrix,
                distortion,
                marker_length_m,
                allowed_ids,
            )
            if not camera_markers:
                continue
            camera_markers, marker_depths = marker_transforms_with_depth_translation(
                camera_markers,
                marker_corners,
                marker_ids,
                lambda center: rs.rs2_deproject_pixel_to_point(
                    self.color_intrinsics,
                    [float(center[0]), float(center[1])],
                    self._median_depth(
                        depth_frame, int(round(center[0])), int(round(center[1]))
                    ),
                ),
            )
            if not camera_markers:
                continue
            for marker_corner, marker_id in zip(marker_corners, marker_ids.reshape(-1)):
                if int(marker_id) != selected_id:
                    continue
                corner_points = np.asarray(marker_corner, dtype=float).reshape(4, 2)
                pixel_width = float(
                    np.linalg.norm(corner_points[0] - corner_points[1])
                )
                z_m = marker_depths.get(int(marker_id))
                if z_m is None:
                    continue
                width_samples.append(
                    pixel_width / camera_matrix[0, 0] * (z_m * 1000.0)
                )
                marker_normal = camera_markers[int(marker_id)][:3, 2]
                marker_tvec = camera_markers[int(marker_id)][:3, 3]
                view_direction = -marker_tvec / float(np.linalg.norm(marker_tvec))
                tilt_samples.append(
                    math.degrees(
                        math.acos(
                            float(abs(np.dot(marker_normal, view_direction)))
                        )
                    )
                )
            samples.append(
                {
                    marker_id: base_marker_transform(
                        self.latest_tcp, self.tool_camera, camera_marker
                    )
                    for marker_id, camera_marker in camera_markers.items()
                }
            )
        base_markers = group_and_average_marker_samples(
            samples, self.arguments.marker_minimum_samples
        )
        if not base_markers:
            raise RuntimeError(
                "no taught basket marker was stable for enough frames; return the basket "
                "to the search area and keep every visible code flat, clean, and unobstructed"
            )
        print(
            "Stable basket marker numbers="
            f"{[marker_id + 1 for marker_id in sorted(base_markers)]}"
        )
        for marker_id, base_marker in base_markers.items():
            print(
                f"marker number={marker_id + 1} estimated base position[mm]="
                f"{np.round(base_marker[:3, 3] * 1000.0, 1).tolist()}"
            )
        if width_samples:
            measured = float(np.median(width_samples))
            nominal = marker_length_m * 1000.0
            deviation = abs(measured - nominal) / nominal * 100.0
            print(
                f"marker measured black-square width={measured:.1f} mm "
                f"(nominal {nominal:.1f} mm, deviation {deviation:.1f}%)"
            )
            if deviation > 5.0:
                print(
                    "WARNING: measured marker width deviates >5% from nominal; "
                    "measure the printed black square with a ruler, re-print if needed, "
                    "and re-teach with the corrected --marker-length-m"
                )
        if tilt_samples:
            tilt = float(np.median(tilt_samples))
            print(f"marker plane tilt from camera line of sight={tilt:.1f} deg")
            if tilt > 50.0:
                print(
                    "WARNING: marker tilt exceeds 50 deg; PnP accuracy degrades and "
                    "solutions can flip. Turn the marker to face the camera more."
                )
        target_poses = {}
        target_joints = {}
        for pose_name in ["basket_approach", "basket_release"]:
            if self.arguments.basket_motion_mode == "translation":
                target_transform, shift = translated_taught_target(
                    base_markers[selected_id], marker_record, pose_name
                )
                shift_mm = shift * 1000.0
                shift_distance_mm = float(np.linalg.norm(shift_mm))
                if shift_distance_mm > float(self.arguments.max_basket_shift_mm):
                    raise RuntimeError(
                        f"basket shift {shift_distance_mm:.1f} mm exceeds "
                        f"{self.arguments.max_basket_shift_mm:.1f} mm; localization is "
                        "probably wrong or the basket left its taught rack region"
                    )
                used_ids = [selected_id]
                spread_mm = 0.0
                print(
                    "basket translation from teach[mm]="
                    f"{np.round(shift_mm, 1).tolist()} (taught orientation preserved)"
                )
            else:
                target_transform, used_ids, spread_mm = fuse_taught_target(
                    base_markers,
                    self.basket_aruco["markers"],
                    pose_name,
                    self.arguments.marker_fusion_max_mm,
                )
            target_pose = transform_to_pose(target_transform, "intrinsic_xyz")
            target_pose[:3] *= 1000.0
            target_pose = target_pose.tolist()
            target_joint = self._solve_ik(target_pose)
            self._validate_joint(target_joint)
            target_poses[pose_name] = target_pose
            target_joints[pose_name] = target_joint
            print(
                f"dynamic {pose_name} TCP={np.round(target_pose, 3).tolist()}, "
                f"numbers={[marker_id + 1 for marker_id in used_ids]}, "
                f"fusion spread={spread_mm:.1f} mm"
            )
            print(f"dynamic {pose_name} IK={np.round(target_joint[:6], 3).tolist()}")
        self.dynamic_place_poses = target_poses
        self.dynamic_place_joints = target_joints
        print(
            f"Basket number={selected_id + 1} localization READY and locked. "
            "Do not move it before pressing g."
        )

    def _mouse_callback(self, event, x, y, flags, parameter) -> None:
        super()._mouse_callback(event, x, y, flags, parameter)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pick_approach_joint = None
            self.pick_grasp_joint = None
            self.pick_approach_pose = None
            self.pick_grasp_pose = None
            self.pick_object_base_mm = None

    def _call_hand_set_enabled(self, enabled: bool) -> None:
        if not self.hand_enable_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/tl_hand/set_enabled is unavailable; start tl_hand_guard")
        request = SetBool.Request()
        request.data = bool(enabled)
        response = self._call(self.hand_enable_client, request, 5.0)
        if not response.success:
            raise RuntimeError(f"/tl_hand/set_enabled failed: {response.message}")
        print(f"hand enabled={enabled}: {response.message}")

    def _call_hand_trigger(self, name: str, client) -> None:
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"/tl_hand/{name} is unavailable; start tl_hand_guard")
        response = self._call(client, Trigger.Request(), 5.0)
        if not response.success:
            raise RuntimeError(f"/tl_hand/{name} failed: {response.message}")
        print(f"hand {name}: {response.message}")
        deadline = time.monotonic() + max(0.0, float(self.arguments.hand_settle_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _hand_write_positions(self, label: str, positions) -> None:
        """Write 6-finger target positions to the ROH-LiteS001 via the driver's
        ModBus passthrough (port 2, RS485, slave 2)."""
        request = ModbusWrite.Request()
        request.master_id = 1
        request.addr = 1135  # POS_TARGET
        request.data = [int(value) for value in positions]
        rtu = ModbusRTUParam()
        rtu.slave_id = 2
        rtu.port = 2
        rtu.baudrate = 115200
        rtu.data_bit = 8
        rtu.stop_bit = 1
        rtu.check_bit = "None"
        master = ModbusMasterParam()
        master.type = "RTU"
        master.start_addr = True
        master.rtu = rtu
        request.master_param = master
        if not self.hand_modbus_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/tl_driver/modbus_write unavailable; start the driver")
        future = self.hand_modbus_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not (future.done() and future.result() and future.result().success):
            raise RuntimeError(f"hand {label} ModBus write failed")
        print(f"hand {label}: positions={list(request.data)}")
        deadline = time.monotonic() + max(0.0, float(self.arguments.hand_settle_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def calculate_pick(self, depth_frame) -> None:
        if self.clicked_pixel is None:
            raise RuntimeError("click the object center first")
        if self.latest_tcp is None or time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError("/tcp_pose is stale")
        pixel_x, pixel_y = self.clicked_pixel
        depth_m = self._median_depth(depth_frame, pixel_x, pixel_y)
        if not self.arguments.min_target_depth_m <= depth_m <= self.arguments.max_target_depth_m:
            raise RuntimeError(
                f"selected depth {depth_m:.3f} m is outside expected range "
                f"{self.arguments.min_target_depth_m:.3f}.."
                f"{self.arguments.max_target_depth_m:.3f} m"
            )
        camera_point = rs.rs2_deproject_pixel_to_point(
            self.color_intrinsics, [float(pixel_x), float(pixel_y)], depth_m
        )
        raw_base = np.asarray(
            camera_point_to_base(self.latest_tcp, self.tool_camera, camera_point),
            dtype=float,
        )
        base_point = raw_base.copy()
        base_point[:2] += np.asarray(self.arguments.base_xy_bias_mm, dtype=float)
        print(
            "camera[m]="
            f"{np.round(camera_point, 4).tolist()}, "
            f"raw base[mm]={np.round(raw_base, 2).tolist()}"
        )
        self._recompute_pick(base_point)
        print("Press c again to pick, or m to move only. Press r to return.")
        print("Fine adjust after picking: a/d=X, s/w=Y, j/u=Z (j down).")

    def _recompute_pick(self, base_point) -> None:
        base_point = np.asarray(base_point, dtype=float).reshape(3)
        target_rpy = self._target_rpy()

        grasp_virtual = base_point.copy()
        if self.arguments.final_grasp_z_mm is not None:
            # Absolute tool0 Z tuned with overhead_test; tool_pose_for_virtual_tcp
            # places tool0 at virtual - R@offset, so add the rotated Z back.
            base_tool = pose_to_transform([0.0, 0.0, 0.0], target_rpy, "intrinsic_xyz")
            rotated_offset = base_tool[:3, :3] @ np.asarray(
                self.arguments.mock_grasp_offset_mm, dtype=float
            )
            grasp_virtual[2] = float(self.arguments.final_grasp_z_mm) + float(
                rotated_offset[2]
            )
        else:
            grasp_virtual[2] += float(self.arguments.pick_grasp_clearance_mm)
        grasp_pose = tool_pose_for_virtual_tcp(
            grasp_virtual,
            self.latest_tcp,
            self.arguments.mock_grasp_offset_mm,
            "intrinsic_xyz",
            target_rpy,
        )

        print(
            "base XY bias[mm]="
            f"{np.round(self.arguments.base_xy_bias_mm, 2).tolist()}, "
            f"compensated base[mm]={np.round(base_point, 2).tolist()}"
        )
        print(
            "grasp virtual tool0 Z[mm]="
            f"{grasp_virtual[2]:.2f}, target rpy="
            f"{np.round(target_rpy, 3).tolist()}"
        )

        self._validate_cartesian(grasp_pose, base_point, grasp_virtual)
        self.nominal_pick_grasp_z = float(grasp_pose[2])
        grasp_pose, grasp_virtual, grasp_joint = self._solve_target_with_optional_lift(
            grasp_pose, grasp_virtual
        )
        self._validate_joint(grasp_joint)

        approach_virtual = approach_point_above_reachable_grasp(
            grasp_virtual,
            self.arguments.pick_approach_clearance_mm,
            self.arguments.pick_grasp_clearance_mm,
        )
        approach_pose = tool_pose_for_virtual_tcp(
            approach_virtual,
            self.latest_tcp,
            self.arguments.mock_grasp_offset_mm,
            "intrinsic_xyz",
            target_rpy,
        )
        self._validate_cartesian(approach_pose, base_point, approach_virtual)
        approach_pose, approach_virtual, approach_joint = self._solve_target_with_optional_lift(
            approach_pose, approach_virtual
        )
        self._validate_joint(approach_joint)

        self.pick_object_base_mm = base_point.tolist()
        self.base_point_mm = base_point.tolist()
        self.pick_approach_pose = list(approach_pose)
        self.pick_grasp_pose = list(grasp_pose)
        self.pick_approach_joint = list(approach_joint)
        self.pick_grasp_joint = list(grasp_joint)
        print(
            "pick approach virtual grasp[mm]="
            f"{np.round(approach_virtual, 2).tolist()}, "
            f"tool0={np.round(approach_pose, 3).tolist()}"
        )
        print(
            "pick grasp virtual grasp[mm]="
            f"{np.round(grasp_virtual, 2).tolist()}, "
            f"tool0={np.round(grasp_pose, 3).tolist()}"
        )
        print(f"pick approach IK={np.round(approach_joint[:6], 3).tolist()}")
        print(f"pick grasp IK={np.round(grasp_joint[:6], 3).tolist()}")

    def fine_adjust_pick(
        self, delta_x_mm: float, delta_y_mm: float, delta_z_mm: float
    ) -> None:
        if self.pick_object_base_mm is None:
            raise RuntimeError("click the target and press c first")
        adjusted = np.asarray(self.pick_object_base_mm, dtype=float)
        adjusted[0] += float(delta_x_mm)
        adjusted[1] += float(delta_y_mm)
        adjusted[2] += float(delta_z_mm)
        print(
            f"Fine adjust dX={delta_x_mm:+.1f}, dY={delta_y_mm:+.1f}, "
            f"dZ={delta_z_mm:+.1f} -> "
            f"object base[mm]={np.round(adjusted, 1).tolist()}"
        )
        self._recompute_pick(adjusted)
        self.execute_pick()
        if self.pick_grasp_pose is not None:
            self._log_grasp_z(self.pick_grasp_pose)

    def move_to_pick_target(self) -> None:
        if self.pick_approach_joint is None or self.pick_grasp_joint is None:
            raise RuntimeError("click the target and press c before pressing m")
        self._validate_joint(self.pick_approach_joint)
        print("Target move 1/2: safe approach above the clicked center")
        self.move_joint(self.pick_approach_joint)
        self._validate_joint(self.pick_grasp_joint)
        print("Target move 2/2: move the fake-hand center to the calculated target")
        self.move_joint(
            self.pick_grasp_joint,
            speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
        )
        print("Target reached. Press r to return to the fixed top-down observation pose.")

    def execute_pick(self) -> None:
        if self.pick_approach_joint is None or self.pick_grasp_joint is None:
            raise RuntimeError("click the target and press c first")
        self._validate_joint(self.pick_approach_joint)
        print("Pick 1/3: move above the clicked target")
        self.move_joint(self.pick_approach_joint)
        self._validate_joint(self.pick_grasp_joint)
        print("Pick 2/3: move to the grasp pose")
        self.move_joint(
            self.pick_grasp_joint,
            speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
        )
        # Descend toward the click-computed object height (never below it), using
        # the current joint reference so IK keeps resolving; auto-lift may have
        # stopped high because it probes from the observation pose.
        self.target_reached = True
        self.target_pose = list(self.pick_grasp_pose)
        self.auto_descent(floor_z=self.nominal_pick_grasp_z)
        self.auto_descend_to_grasp_z()
        self.pick_grasp_pose = list(self.target_pose)
        self.pick_grasp_joint = list(self.target_joint)
        if self.arguments.use_hand:
            print("Pick 3/3: ROH hand grasp (level 5)")
            self._hand_write_positions(
                "grasp", self.arguments.hand_grasp_positions
            )
        else:
            print("Pick 3/3: fake hand grasp simulated")
            time.sleep(0.5)
        print("Lift above the clicked target")
        self.move_joint(self.pick_approach_joint)
        print("Returning to the default pick position (home pose).")
        self.return_observation()
        self.pick_complete = True
        print(
            "Pick complete. Press r to move to the basket place position, "
            "or q to quit."
        )

    def run_cycle(self) -> None:
        if self.arguments.target_only:
            raise RuntimeError("basket cycle is disabled in target-only mode")
        if (
            not self.arguments.place_only
            and (self.pick_approach_joint is None or self.pick_grasp_joint is None)
        ):
            raise RuntimeError("press c to calculate a pick target first")
        if self.basket_aruco is not None and not self.dynamic_place_joints:
            scan_note = (
                "before placing" if self.arguments.place_only else "before closing the hand"
            )
            print(f"Starting autonomous six-view rack scan {scan_note}.")
            self.localize_basket()
            if self.detect_joint is not None:
                self.move_joint(self.detect_joint)
            else:
                self.return_observation()
        approach_place = self._place_joint("basket_approach")
        release_place = self._place_joint("basket_release")
        self._validate_joint(approach_place)
        self._validate_joint(release_place)
        # In the full cycle the hand starts open to prepare the pick; in
        # place-only mode the hand already holds the object, so keep it closed
        # until the release step at the basket.
        if self.arguments.use_hand and not self.arguments.place_only:
            print("Opening ROH hand before the cycle.")
            self._hand_write_positions("open", self.arguments.hand_open_positions)

        if self.arguments.place_only:
            print("PLACE-ONLY MODE: pick steps are skipped.")
            print("Step 1/4: move above basket")
            self.move_joint(approach_place)
            print("Step 2/4: move to basket release pose")
            self.move_joint(
                release_place,
                speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
            )
            if self.arguments.use_hand:
                print("Step 3/4: ROH hand open (release)")
                self._hand_write_positions(
                    "open", self.arguments.hand_open_positions
                )
            else:
                print("Step 3/4: fake hand mode, release is simulated only")
                time.sleep(0.5)
            print("Step 4/4: return above basket, then to the basket detection pose")
            self.move_joint(approach_place)
            if self.detect_joint is not None:
                self.move_joint(self.detect_joint)
            self.place_done = True
            print("Place complete. At the basket detection pose. Press r to return home.")
            if (
                self.basket_aruco is not None
                and self.basket_aruco["_configured_id"] is None
            ):
                completed_id = self.basket_aruco["_selected_id"]
                self.pending_basket_id = None
                self.basket_aruco["_selected_id"] = None
                self.dynamic_place_joints = {}
                self.dynamic_place_poses = {}
                print(
                    f"Completed basket number={completed_id + 1}. Target unlocked; "
                    "press b to select a basket again for the next cycle."
                )
            return

        print("Step 1/8: move above pick target")
        self.move_joint(self.pick_approach_joint)
        print("Step 2/8: move to pick grasp pose")
        self.move_joint(
            self.pick_grasp_joint,
            speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
        )
        if self.arguments.use_hand:
            print("Step 3/8: ROH hand grasp (level 5)")
            self._hand_write_positions(
                "grasp", self.arguments.hand_grasp_positions
            )
        else:
            print("Step 3/8: fake hand mode, grasp is simulated only")
            time.sleep(0.5)
        print("Step 4/8: lift from pick target")
        self.move_joint(self.pick_approach_joint)
        print("Step 5/8: return through the verified top-down transfer pose")
        self.move_joint(self.observation_joint)
        print("Step 6/8: move above basket")
        self.move_joint(approach_place)
        print("Step 7/8: move to basket release pose")
        self.move_joint(
            release_place,
            speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
        )
        if self.arguments.use_hand:
            print("Step 8/8: ROH hand open (release)")
            self._hand_write_positions(
                "open", self.arguments.hand_open_positions
            )
        else:
            print("Step 8/8: fake hand mode, release is simulated only")
            time.sleep(0.5)
        print("Return above basket, then observation pose")
        self.move_joint(approach_place)
        self.return_observation()
        if (
            self.basket_aruco is not None
            and self.basket_aruco["_configured_id"] is None
        ):
            completed_id = self.basket_aruco["_selected_id"]
            self.pending_basket_id = None
            self.basket_aruco["_selected_id"] = None
            self.dynamic_place_joints = {}
            self.dynamic_place_poses = {}
            print(
                f"Completed basket number={completed_id + 1}. Target unlocked; "
                "press b to select a basket again for the next cycle."
            )

    def return_observation(self) -> None:
        if self.observation_joint is None:
            raise RuntimeError("observation joint is missing")
        self.move_joint(self.observation_joint)

    def smart_return(self) -> None:
        """R key: localize the basket (if needed) and move to the fixed
        basket-detection pose; after placing, return to the home pose."""
        if not self.place_done and self.basket_aruco is not None:
            if not self.dynamic_place_joints:
                print("Localizing the basket before moving to the detection pose.")
                self.localize_basket()
            if self.detect_joint is not None:
                print("Moving to the basket detection pose.")
                self.move_joint(self.detect_joint)
            print(
                "At the basket detection pose. Press g to place, "
                "or r to return home."
            )
            return
        print("Returning to the home pose.")
        self.return_observation()
        self.pick_complete = False
        self.place_done = False

    def move_to_configured_observation(self) -> None:
        if self.arguments.home_joint_deg is not None:
            configured = self._pad_joint(self.arguments.home_joint_deg)
            description = "home"
        elif self.arguments.observation_joints is None:
            print("Using startup joint state as the observation pose.")
            return
        else:
            configured = self._pad_joint(self.arguments.observation_joints)
            description = "top-down observation"
        self._validate_joint(configured)
        current = np.asarray(self.latest_joint[:6], dtype=float)
        target = np.asarray(configured[:6], dtype=float)
        error = float(np.max(np.abs(target - current)))
        print(
            f"Configured {description} joints[deg]="
            f"{np.round(target, 3).tolist()}"
        )
        if error > float(self.arguments.arrival_tolerance_deg):
            print(f"Moving to the configured {description} pose at startup.")
            self.move_joint(configured)
        else:
            print(
                f"Already at configured {description} pose; max error={error:.3f} deg"
            )
        self.observation_joint = configured
        if self.latest_tcp is not None:
            print(
                f"{description.capitalize()} TCP after startup move="
                f"{np.round(self.latest_tcp, 3).tolist()}"
            )

    def run(self) -> None:
        self.wait_for_robot()
        if self.arguments.observation_joints is not None:
            self.detect_joint = self._pad_joint(self.arguments.observation_joints)
        self.move_to_configured_observation()
        self.start_camera()
        print(f"calibration={self.calibration_path}")
        print(
            f"base XY bias[mm]={np.round(self.arguments.base_xy_bias_mm, 2).tolist()}"
        )
        print(
            "pick clearances[mm]="
            f"approach={self.arguments.pick_approach_clearance_mm:.1f}, "
            f"grasp={self.arguments.pick_grasp_clearance_mm:.1f}"
        )
        print(
            "PRIMARY: click | c pick | a/d/s/w/j/u fine | r return | q quit"
        )
        if self.arguments.target_only:
            print("TARGET-ONLY MODE: basket localization and full cycle are disabled")
        elif self.arguments.place_only:
            print(
                "PLACE-ONLY MODE: c pick | r to basket | g place | "
                "r home | 1..6 select | 0 auto"
            )
        else:
            print("ADVANCED BASKET: b localize | g full pick-place | 1..6 select | 0 auto")
        window_name = "TL mock pick-place to basket"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window_name, self._mouse_callback)
        if self.basket_aruco is not None:
            basket_camera_matrix, basket_distortion = camera_parameters(
                self.color_intrinsics
            )
            basket_marker_length_m = float(
                self.basket_aruco.get("marker_length_m", 0.06)
            )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            try:
                frames = self.align.process(
                    self.pipeline.wait_for_frames(timeout_ms=2000)
                )
            except RuntimeError as error:
                self.get_logger().warning(f"D435 frame timeout: {error}")
                continue
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            preview = np.asanyarray(color_frame.get_data()).copy()
            if self.basket_aruco is not None:
                marker_transforms, marker_corners, marker_ids, _ = (
                    detect_marker_transforms(
                        preview,
                        basket_camera_matrix,
                        basket_distortion,
                        basket_marker_length_m,
                        self.basket_aruco["_usable_ids"],
                    )
                )
                self.latest_visible_basket_ids = [
                    int(value) for value in marker_ids.reshape(-1)
                ]
                preview = draw_marker_overlay(
                    preview,
                    marker_corners,
                    marker_ids,
                    marker_transforms,
                    basket_camera_matrix,
                    basket_distortion,
                    basket_marker_length_m * 0.5,
                )
            if self.clicked_pixel is not None:
                cv2.drawMarker(
                    preview,
                    self.clicked_pixel,
                    (0, 255, 255),
                    cv2.MARKER_CROSS,
                    24,
                    2,
                )
            basket_status = ""
            if self.basket_aruco is not None:
                selected_id = self.basket_aruco["_selected_id"]
                pending_id = self.pending_basket_id
                if selected_id is not None and self.dynamic_place_joints:
                    selection_text = f"LOCKED N{selected_id + 1} READY"
                elif pending_id is not None:
                    selection_text = f"MANUAL N{pending_id + 1}; press B"
                elif self.basket_aruco["_configured_id"] is not None:
                    selection_text = (
                        f"FIXED N{self.basket_aruco['_configured_id'] + 1}; press B"
                    )
                else:
                    selection_text = "AUTO; aim marker near center, press B"
                visible_text = ",".join(
                    f"N{marker_id + 1}"
                    for marker_id in sorted(self.latest_visible_basket_ids)
                ) or "none"
                basket_status = f"{selection_text} | visible={visible_text}"
                cv2.putText(
                    preview,
                    basket_status,
                    (8, 46),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.43,
                    (0, 255, 0),
                    1,
                )
            cv2.putText(
                preview,
                "click target | C:calculate | M:move | R:return | Q:quit",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
            )
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            try:
                if key in (ord("q"), 27):
                    break
                if key == ord("c"):
                    self.calculate_pick(depth_frame)
                    self.execute_pick()
                elif key == ord("m"):
                    self.move_to_pick_target()
                elif key == ord("a"):
                    self.fine_adjust_pick(
                        -float(self.arguments.fine_step_mm), 0.0, 0.0
                    )
                elif key == ord("d"):
                    self.fine_adjust_pick(
                        float(self.arguments.fine_step_mm), 0.0, 0.0
                    )
                elif key == ord("s"):
                    self.fine_adjust_pick(
                        0.0, -float(self.arguments.fine_step_mm), 0.0
                    )
                elif key == ord("w"):
                    self.fine_adjust_pick(
                        0.0, float(self.arguments.fine_step_mm), 0.0
                    )
                elif key == ord("j"):
                    self.fine_adjust_pick(
                        0.0, 0.0, -float(self.arguments.fine_z_step_mm)
                    )
                elif key == ord("u"):
                    self.fine_adjust_pick(
                        0.0, 0.0, float(self.arguments.fine_z_step_mm)
                    )
                elif key == ord("b"):
                    if self.arguments.target_only:
                        print("B is disabled in target-only mode. Use C, M, R, or Q.")
                    else:
                        self.localize_basket()
                        self.return_observation()
                elif ord("1") <= key <= ord("6"):
                    if not self.arguments.target_only:
                        self.select_basket_number(key - ord("0"))
                elif key == ord("0"):
                    if not self.arguments.target_only:
                        self.reset_live_basket_selection()
                elif key == ord("g"):
                    if self.arguments.target_only:
                        print("G is disabled in target-only mode. Use M for the MoveJ test.")
                    else:
                        self.run_cycle()
                elif key in (ord("r"), ord("h")):
                    self.smart_return()
                elif key == ord("o") and self.arguments.use_hand:
                    self._call_hand_set_enabled(True)
                    self._call_hand_trigger("open", self.hand_open_client)
                elif key == ord("x") and self.arguments.use_hand:
                    self._call_hand_trigger("stop", self.hand_stop_client)
            except RuntimeError as error:
                print(f"Command rejected/stopped: {error}")


def main(argv=None):
    arguments = parse_args(argv)
    calibration_path, calibration, tool_camera = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    rclpy.init()
    node = MockPickPlaceNode(arguments, calibration_path, calibration, tool_camera)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
