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
from tl_ros2_interface.srv import ModbusRead, ModbusWrite

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
from .mqtt_protocol import (
    MqttClient,
    MqttCommand,
    RET_OCCUPIED,
    RET_STARTED,
    RET_SUCCESS,
    RET_RECOG_FAILED,
    parse_target_map,
    target_id_to_basket_number,
)
from .overhead_test import ReferenceOverheadTest, load_calibration
from .reference_compat import (
    approach_point_above_reachable_grasp,
    camera_point_to_base,
    tool_pose_for_virtual_tcp,
)

# “贴反”的排：把“目标位号”镜像为该排实际贴的码号（由 --reversed-rows 配置，
# 例：A3 排实际贴的是 ID71..ID48，则位置 A3-08 → 实际码号 65 / ID64）。
# 注意：镜像套两次会变回原值，别对已转换的编号重复套用。
ROW_NAMES = ("A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4")


def apply_reversed_rows(number, reversed_rows):
    """返回该编号在“贴反排”上的实际码号；其它排原样返回。"""
    number = int(number)
    row_index = (number - 1) // 24
    if ROW_NAMES[row_index] not in reversed_rows:
        return number
    index_in_row = (number - 1) % 24
    return row_index * 24 + (24 - index_in_row)


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
    parser.add_argument(
        "--marker-sample-spread-mm",
        type=float,
        default=60.0,
        help="reject a basket when its marker position samples disagree by more "
        "than this (mm). The aggregation is median-based, so frame-to-frame "
        "jitter (typical 20-55 mm with a slightly loose camera) still yields a "
        "usable median; this gate only catches gross bimodal garbage",
    )
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
        "--max-place-reachability-shift-mm",
        type=float,
        default=50.0,
        help="when the compensated place pose has no IK solution, auto-adjust "
        "it toward the nearest reachable pose if the required XY shift is "
        "within this limit (mm); larger shifts fail loudly instead",
    )
    parser.add_argument(
        "--max-taught-joint-shift-mm",
        type=float,
        default=15.0,
        help="when the place pose has no IK solution but the measured basket "
        "shift from teaching is within this limit (mm), use the taught joint "
        "directly: it was physically verified by dragging, and the driver IK "
        "is more conservative than the real arm",
    )
    parser.add_argument(
        "--max-taught-joint-rot-deg",
        type=float,
        default=5.0,
        help="same as max-taught-joint-shift-mm, for the orientation "
        "difference between the dynamic place target and the taught pose (deg)",
    )
    parser.add_argument(
        "--basket-number",
        type=int,
        choices=range(1, 193),
        default=None,
        help="independent basket number 1..192 to localize and place into",
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
        "--reversed-rows",
        nargs="*",
        default=[],
        metavar="ROW",
        help="rows whose ArUco codes are stuck in reversed order (e.g. A3): "
        "target positions are mirrored to the actual code numbers",
    )
    parser.add_argument(
        "--lower-observation-joints-a",
        type=float,
        nargs=6,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="optional fixed A-side lower-row (A1/A4) observation pose; falls back "
        "to lower_observation_pose_a in the basket YAML (press d in the teach program)",
    )
    parser.add_argument(
        "--lower-observation-joints-b",
        type=float,
        nargs=6,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="optional fixed B-side lower-row (B1/B4) observation pose; falls back "
        "to lower_observation_pose_b in the basket YAML (press d in the teach program)",
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
        "--hand-hold-positions",
        type=int,
        nargs=6,
        default=[8000, 8000, 8000, 8000, 8000, 5000],
        help="ROH-LiteS001 6-finger target positions for level-1 protective hold",
    )
    parser.add_argument(
        "--hand-level5-ratio",
        type=float,
        default=0.85,
        help="minimum actual/target finger position ratio considered level-5 closed",
    )
    parser.add_argument("--mqtt-enabled", action="store_true")
    parser.add_argument("--mqtt-host", default="127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--mqtt-command-topic", default="tl/robot/command")
    parser.add_argument("--mqtt-feedback-topic", default="tl/robot/feedback")
    parser.add_argument("--mqtt-client-id", default="tl-tcb605-arm")
    parser.add_argument("--mqtt-username", default="")
    parser.add_argument("--mqtt-password", default="")
    parser.add_argument("--mqtt-keepalive", type=int, default=60)
    parser.add_argument(
        "--mqtt-target-map",
        default="",
        help='JSON object mapping command targetId to taught basket number, for example {"A1-01":1}',
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
    parser.add_argument(
        "--fixed-pick-base-mm",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="fixed pick target in BASE mm; MQTT seed picks here without vision",
    )
    parser.add_argument("--ros-args", nargs=argparse.REMAINDER, help=argparse.SUPPRESS)
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
    if arguments.mqtt_port <= 0 or arguments.mqtt_port > 65535:
        parser.error("--mqtt-port must be in 1..65535")
    if arguments.mqtt_keepalive <= 0:
        parser.error("--mqtt-keepalive must be positive")
    if arguments.hand_level5_ratio <= 0.0 or arguments.hand_level5_ratio > 1.0:
        parser.error("--hand-level5-ratio must be in (0, 1]")
    try:
        parse_target_map(arguments.mqtt_target_map)
    except ValueError as error:
        parser.error(str(error))

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
        self.dynamic_search_joint = None
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
        self.hand_modbus_read_client = self.create_client(
            ModbusRead, "/tl_driver/modbus_read"
        )
        self.pending_basket_id = None
        self.latest_visible_basket_ids = []
        self._basket_occupied: set[int] = set()
        self.pick_complete = False
        self.place_done = False
        self.mqtt_target_map = parse_target_map(arguments.mqtt_target_map)
        self.mqtt_pending_command: MqttCommand | None = None
        # 车端协议按阶段固定反馈命令名：取种阶段用 "seed"，放置阶段用 "seed_place"
        self._mqtt_feedback_phase = "seed"
        self.mqtt_client = (
            MqttClient(
                host=arguments.mqtt_host,
                port=arguments.mqtt_port,
                command_topic=arguments.mqtt_command_topic,
                feedback_topic=arguments.mqtt_feedback_topic,
                client_id=arguments.mqtt_client_id,
                username=arguments.mqtt_username,
                password=arguments.mqtt_password,
                keepalive=arguments.mqtt_keepalive,
            )
            if arguments.mqtt_enabled
            else None
        )
        self.mqtt_auto_pick_pending = False

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
                # MQTT seed commands address a specific basket, so partial
                # teaching is fine: only the taught baskets are selectable.
                print(
                    f"WARNING: baskets {incomplete} are not fully taught yet; "
                    "MQTT seed works only for the taught baskets"
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

    def _select_basket_number_prompt(self) -> None:
        """Prompt for a basket number 1..192 (10..192 are not on the number keys)."""
        if self.basket_aruco is None:
            print("dynamic basket mode is not enabled")
            return
        try:
            raw = input("Select basket number 1..192 (A1-01=1 ... B4-24=192): ").strip()
        except EOFError:
            return
        try:
            basket_number = int(raw)
        except ValueError:
            print(f"Invalid basket number: {raw!r}")
            return
        if not 1 <= basket_number <= 192:
            print(f"Basket number must be 1..192, got {basket_number}")
            return
        try:
            self.select_basket_number(basket_number)
        except RuntimeError as error:
            print(f"Basket selection rejected: {error}")

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


    # ---------- [VLM辅助] ----------
    def _save_vlm_frame(self, image):
        """[VLM辅助] 保存最新一帧到 ~/tl_llm_agent/frames/ 供视觉大模型复核。
        仅写文件，不改变任何算法逻辑；调用处已做异常保护。"""
        import json
        import os

        root = os.path.expanduser("~/tl_llm_agent/frames")
        os.makedirs(root, exist_ok=True)
        cv2.imwrite(os.path.join(root, "latest.jpg"), image)
        with open(os.path.join(root, "latest.json"), "w", encoding="utf-8") as handle:
            json.dump({"time": time.time()}, handle)

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
            # [VLM辅助] 保存最新帧供视觉复核（异常不影响主流程）
            try:
                self._save_vlm_frame(image)
            except Exception:
                pass
            camera_markers, corners, ids, _ = detect_marker_transforms(
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
            preview = draw_marker_overlay(
                image,
                corners,
                ids,
                camera_markers,
                camera_matrix,
                distortion,
                marker_length_m * 0.5,
            )
            cv2.putText(
                preview,
                f"basket scan: samples={len(center_samples)}/"
                f"{int(self.arguments.marker_samples)} visible={sorted(centers)}",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )
            if self.window_name:
                cv2.imshow(self.window_name, preview)
                cv2.waitKey(1)
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
            if (int(marker_id) + 1) in self._basket_occupied:
                print(f"Skipping basket number={marker_id + 1} (already occupied).")
                continue
            record = self._basket_record(marker_id)
            search_pose = record.get("search_pose")
            if not search_pose or "joint" not in search_pose:
                continue
            search_joint = self._pad_joint(search_pose["joint"])
            distance = float(
                np.max(np.abs(np.asarray(search_joint[:6], dtype=float) - current))
            )
            candidates.append((distance, int(marker_id), search_joint))
        if not candidates:
            raise RuntimeError("all baskets are occupied")
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
            "automatic rack scan visited all taught views but found no stable basket; "
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

    def _lower_observation_joint(self, side: str) -> list:
        """下排（A1/A4/B1/B4）观测位：命令行参数优先，其次用示教 d 键的 YAML 留档。"""
        cli = (
            self.arguments.lower_observation_joints_a
            if side == "A"
            else self.arguments.lower_observation_joints_b
        )
        if cli is not None:
            print(f"Lower observation pose (side {side}): command-line joints.")
            return self._pad_joint(list(cli) + [0.0])
        pose = None
        if self.basket_aruco is not None:
            pose = self.basket_aruco.get(f"lower_observation_pose_{side.lower()}")
        if pose and pose.get("joint"):
            print(
                f"Lower observation pose (side {side}): "
                f"YAML lower_observation_pose_{side.lower()}."
            )
            return self._pad_joint(pose["joint"])
        if side == "B" and self.basket_aruco is not None:
            # B 侧没单独记录时自动推导：场地 A/B 精确对称（两侧起始位实测
            # J1 相差 180.000°，其余关节相同），用 A 侧记录把 J1 旋转 180°。
            pose_a = self.basket_aruco.get("lower_observation_pose_a") or {}
            if pose_a.get("joint"):
                joints = self._pad_joint(pose_a["joint"])
                joints[0] = ((joints[0] + 180.0) + 180.0) % 360.0 - 180.0
                print(
                    "Lower observation pose (side B): derived from the side A "
                    f"record rotated 180 deg (J1 {float(pose_a['joint'][0]):.3f} "
                    f"-> {joints[0]:.3f})."
                )
                return joints
        raise RuntimeError(
            f"lower observation pose for side {side} is not recorded; start the "
            f"teach program, drag the arm to a pose that clearly sees the {side}-side "
            "lower-row (A1/A4/B1/B4) codes and press d (press v to switch side), then retry"
        )

    def localize_basket(self) -> None:
        if self.basket_aruco is None:
            raise RuntimeError("dynamic basket mode is not enabled")
        camera_matrix, distortion = camera_parameters(self.color_intrinsics)
        marker_length_m = float(self.basket_aruco.get("marker_length_m", 0.054))
        configured_id = self.basket_aruco["_configured_id"]
        if configured_id is not None:
            selected_id = int(configured_id)
            print(f"Using configured basket number={selected_id + 1}")
        elif self.pending_basket_id is not None:
            selected_id = int(self.pending_basket_id)
            print(f"Using manually selected basket number={selected_id + 1}")
        else:
            print("Scanning the taught rack views; no basket number is fixed.")
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
        # Remember the taught search pose so the seed cycle can return to the
        # detection position after placing (user requirement: pick -> observe
        # -> search pose -> detect -> place -> search pose -> observe).
        self.dynamic_search_joint = search_joint
        # 上排（A2/A3/B2/B3）在检索位识别（现状）；下排（A1/A4/B1/B4）
        # 按花盆编号分侧去“下排观测位”识别（示教程序 d 键记录，A/B 分侧）。
        row_index = selected_id // 24
        if row_index in (0, 3, 4, 7):
            side = "A" if row_index < 4 else "B"
            print(
                f"Basket number={selected_id + 1} is on a lower row "
                f"(A1/A4/B1/B4); moving to the side {side} lower observation "
                "pose to detect the code."
            )
            self.move_joint(self._lower_observation_joint(side))
        else:
            print(f"Moving to the taught search pose for basket number={selected_id + 1}.")
            self.move_joint(search_joint)
        # After a ServoJ stream the driver may pause /tcp_pose publishing and
        # the arm may still be settling; sampling while the tool pose is stale
        # OR still changing projects the wrong (or moving) tool pose through
        # the marker transform: a stale pose shifts the basket by 100+ mm, a
        # moving pose blows up the sample-spread gate. Wait for a fresh AND
        # stable pose before sampling.
        settle_deadline = time.monotonic() + 5.0
        last_tcp = None
        stable_ticks = 0
        settled = False
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.monotonic() - self.latest_tcp_at > 0.25:
                stable_ticks = 0
                continue
            tcp = np.asarray(self.latest_tcp[:6], dtype=float)
            if last_tcp is not None:
                delta = float(np.max(np.abs(tcp - last_tcp)))
                if delta < 0.5:
                    stable_ticks += 1
                    if stable_ticks >= 8:
                        settled = True
                        break
                else:
                    stable_ticks = 0
            last_tcp = tcp
        if not settled:
            raise RuntimeError(
                "/tcp_pose did not settle after reaching the detection pose; "
                "check the driver and the arm"
            )
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
            if (
                not color_frame
                or not depth_frame
                or self.latest_tcp is None
                or time.monotonic() - self.latest_tcp_at > 0.3
            ):
                continue
            image = np.asanyarray(color_frame.get_data())
            # [VLM辅助] 保存最新帧供视觉复核（异常不影响主流程）
            try:
                self._save_vlm_frame(image)
            except Exception:
                pass
            camera_markers, marker_corners, marker_ids, _ = detect_marker_transforms(
                image,
                camera_matrix,
                distortion,
                marker_length_m,
                allowed_ids,
            )
            if not camera_markers:
                continue
            # Force the PnP translation (pnp_fallback_mm=0.0) so runtime
            # localization always uses the same source as the teach step; a
            # per-frame depth/PnP mix shifts the basket by 10-30 mm and trips
            # the IK auto-adjust even when the rack has not moved.
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
                pnp_fallback_mm=0.0,
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
            preview = draw_marker_overlay(
                image,
                marker_corners,
                marker_ids,
                camera_markers,
                camera_matrix,
                distortion,
                marker_length_m * 0.5,
            )
            cv2.putText(
                preview,
                f"localizing basket N{selected_id + 1}: "
                f"samples={len(samples)}/{int(self.arguments.marker_samples)}",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )
            if self.window_name:
                cv2.imshow(self.window_name, preview)
                cv2.waitKey(1)
        base_markers = group_and_average_marker_samples(
            samples, self.arguments.marker_minimum_samples
        )
        if not base_markers:
            raise RuntimeError(
                "no taught basket marker was stable for enough frames; return the basket "
                "to the search area and keep every visible code flat, clean, and unobstructed"
            )
        # Sample-spread quality gate: after the hand-eye calibration the marker
        # samples must agree with each other. A basket whose frames disagree by
        # more than the configured spread is rejected instead of averaged into a
        # wrong place pose (this is exactly what happened when the old
        # calibration was used after the arm swap).
        spread_limit_mm = float(self.arguments.marker_sample_spread_mm)
        spread_report = []
        for marker_id, marker_transform in list(base_markers.items()):
            translations = np.asarray(
                [
                    sample[marker_id][:3, 3] * 1000.0
                    for sample in samples
                    if marker_id in sample
                ],
                dtype=float,
            )
            if len(translations) < int(self.arguments.marker_minimum_samples):
                continue
            median_translation = np.median(translations, axis=0)
            deviations = np.linalg.norm(
                translations - median_translation, axis=1
            )
            spread_mm = float(np.percentile(deviations, 90))
            spread_report.append(
                f"N{marker_id + 1}={spread_mm:.1f}mm"
            )
            if spread_mm > spread_limit_mm:
                print(
                    f"WARNING: basket number={marker_id + 1} marker samples "
                    f"disagree by {spread_mm:.1f} mm (> {spread_limit_mm:.1f} mm); "
                    "localization is unstable, rejecting this basket instead of "
                    "placing into a wrong pose. Re-check the hand-eye calibration "
                    "or steady the arm."
                )
                del base_markers[marker_id]
        if spread_report:
            print("marker sample spread[mm]=" + ", ".join(spread_report))
        if not base_markers:
            raise RuntimeError(
                "all detected basket markers failed the sample-spread quality gate; "
                "re-run the hand-eye calibration and re-teach the basket before "
                "placing (no motion was sent)"
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
            try:
                target_joint = self._solve_ik(target_pose)
                self._validate_joint(target_joint)
            except RuntimeError as ik_error:
                # The shifted place pose has no IK solution, even though the arm
                # was physically dragged to it during teaching: the driver's IK
                # workspace is more conservative than the real arm. Blindly
                # driving the taught joint is still wrong (if the basket really
                # moved, the taught joint targets the OLD basket position), but
                # when the vision says the basket is still within the
                # measurement noise band of its taught spot, the taught joint
                # IS the exact verified configuration and must be used directly
                # instead of auto-adjusting the seed away from the basket
                # center.
                pose_record = marker_record.get(pose_name, {}) or {}
                taught_joint = pose_record.get("taught_joint")
                taught_tcp = pose_record.get("taught_tcp")
                if taught_joint is None:
                    raise ik_error
                taught_joint = self._pad_joint(taught_joint)
                taught_pose = None
                if taught_tcp is not None:
                    target_array = np.asarray(target_pose, dtype=float)
                    taught_array = np.asarray(taught_tcp, dtype=float)
                    translation_error = float(
                        np.linalg.norm(target_array[:3] - taught_array[:3])
                    )
                    relative = np.linalg.inv(
                        pose_to_transform(
                            taught_array[:3] / 1000.0,
                            taught_array[3:],
                            "intrinsic_xyz",
                        )
                    ) @ pose_to_transform(
                        target_array[:3] / 1000.0,
                        target_array[3:],
                        "intrinsic_xyz",
                    )
                    rotation_error = math.degrees(
                        math.acos(
                            max(
                                -1.0,
                                min(
                                    1.0,
                                    (float(np.trace(relative[:3, :3])) - 1.0) / 2.0,
                                ),
                            )
                        )
                    )
                    if (
                        translation_error
                        <= float(self.arguments.max_taught_joint_shift_mm)
                        and rotation_error
                        <= float(self.arguments.max_taught_joint_rot_deg)
                    ):
                        target_joint = taught_joint
                        self._validate_joint(target_joint)
                        target_pose = list(taught_tcp)
                        taught_pose = target_pose
                        print(
                            f"{pose_name}: basket shift {translation_error:.1f} mm "
                            f"(rotation {rotation_error:.1f} deg) is within "
                            "tolerance; using the taught joint (physically "
                            "verified during teaching) instead of the "
                            "conservative IK"
                        )
                if taught_pose is None:
                    print(
                        f"{pose_name} IK failed ({ik_error}); "
                        "retrying with the taught-joint reference"
                    )
                    response = self._request_ik(
                        target_pose, reference_joint=taught_joint
                    )
                    if response.success:
                        target_joint = self._normalize_ik_joint(response.target_pos)
                        self._validate_joint(target_joint)
                        print(
                            f"{pose_name} IK solved with taught-joint reference: "
                            f"{np.round(target_joint[:6], 3).tolist()}"
                        )
                    else:
                        # Still unreachable: auto-adjust the place pose toward the
                        # nearest reachable probe (same attitude, XY shifted toward
                        # the arm) so small basket shifts near the workspace edge
                        # do not abort the cycle. The adjustment is bounded.
                        reachability = self._find_nearest_reachable_probe(
                            target_pose
                        )
                        if reachability is None:
                            raise RuntimeError(
                                f"{pose_name} is unreachable: no reachable probe "
                                "near the compensated place pose. Restore the "
                                "basket to its taught rack region or re-teach this "
                                "basket. No motion was sent."
                            )
                        probe_pose, shift_xy, xy_shift_mm, z_lift_mm = reachability
                        max_adjust = float(
                            self.arguments.max_place_reachability_shift_mm
                        )
                        if xy_shift_mm > max_adjust or z_lift_mm > max_adjust:
                            raise RuntimeError(
                                f"{pose_name} is unreachable: the nearest "
                                f"reachable probe needs XY shift="
                                f"{xy_shift_mm:.1f} mm / Z lift={z_lift_mm:.1f} mm, "
                                f"above the {max_adjust:.1f} mm auto-adjust limit. "
                                "Restore the basket to its taught rack region or "
                                "re-teach this basket. No motion was sent."
                            )
                        probe_response = self._request_ik(
                            probe_pose, reference_joint=taught_joint
                        )
                        if not probe_response.success:
                            raise RuntimeError(
                                f"{pose_name} is unreachable even at the nearest "
                                "reachable probe. No motion was sent."
                            )
                        target_joint = self._normalize_ik_joint(
                            probe_response.target_pos
                        )
                        self._validate_joint(target_joint)
                        target_pose = list(probe_pose)
                        print(
                            f"WARNING: {pose_name} auto-adjusted to the nearest "
                            f"reachable pose: XY shift="
                            f"{np.round(shift_xy, 1).tolist()} mm "
                            f"(distance {xy_shift_mm:.1f} mm), "
                            f"Z lift={z_lift_mm:.1f} mm. "
                            f"Adjusted TCP={np.round(target_pose, 3).tolist()}"
                        )
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

    @staticmethod
    def _hand_modbus_master():
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
        return master

    def _hand_read_positions(self, address: int) -> list[int]:
        request = ModbusRead.Request()
        request.master_id = 1
        request.addr = int(address)
        request.quantity = 6
        request.master_param = self._hand_modbus_master()
        if not self.hand_modbus_read_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/tl_driver/modbus_read unavailable; start the driver")
        future = self.hand_modbus_read_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not (future.done() and future.result() and future.result().success):
            message = getattr(future.result(), "message", "request failed") if future.done() else "timeout"
            raise RuntimeError(f"hand ModBus read failed at {address}: {message}")
        data = [int(value) for value in future.result().data]
        if len(data) != 6:
            raise RuntimeError(f"hand ModBus read at {address} returned {len(data)} values")
        return data

    def _hand_is_level5_closed(self) -> bool:
        actual = self._hand_read_positions(1145)
        target = self._hand_read_positions(1135)
        configured = [int(value) for value in self.arguments.hand_grasp_positions]
        ratio = float(self.arguments.hand_level5_ratio)
        closed = all(
            actual_value >= max(1000, int(configured_value * ratio))
            for actual_value, configured_value in zip(actual, configured)
        )
        target_closed = all(
            target_value >= max(1000, int(configured_value * ratio))
            for target_value, configured_value in zip(target, configured)
        )
        print(
            f"hand level-5 check: actual={actual}, target={target}, "
            f"ratio={ratio:.2f}, closed={closed or target_closed}"
        )
        return bool(closed or target_closed)

    def _mqtt_publish_pending(
        self,
        ret_code: int,
        message: str = None,
        final: bool = False,
        command_name: str = None,
    ) -> None:
        """Publish feedback for the current MQTT command.

        Intermediate phase feedbacks (取种完成/放置响应) keep the pending
        command so the cycle can keep reporting; the final feedback clears it.
        command_name overrides the echoed name for the two-phase protocol
        (seed phase -> "seed", placement phase -> "seed_place").
        """
        if self.mqtt_client is None or self.mqtt_pending_command is None:
            return
        command = self.mqtt_pending_command
        if final:
            self.mqtt_pending_command = None
        self.mqtt_client.publish_feedback(
            command_name or command.name,
            ret_code,
            message,
            target_id=command.target_id,
        )

    def _mqtt_fail_pending(self, error: Exception) -> None:
        if self.mqtt_pending_command is None:
            return
        print(f"MQTT command failed: {error}")
        ret_code = RET_OCCUPIED if "level-5 closed" in str(error) else RET_RECOG_FAILED
        self._mqtt_publish_pending(
            ret_code,
            message=str(error),
            final=True,
            command_name=self._mqtt_feedback_phase,
        )

    def _auto_seed_pick_place(self) -> None:
        """Full seed cycle: pick at fixed point -> localize basket -> place -> return.

        Wrapped in try/except so any IK failure, unreachable pose, or basket
        localization error reports retCode via MQTT and returns to the observation
        pose instead of leaving the arm mid-cycle.
        """
        fixed_base = np.asarray(self.arguments.fixed_pick_base_mm, dtype=float)
        print(
            "MQTT seed: fixed-point pick at "
            f"base[mm]={np.round(fixed_base, 2).tolist()}"
        )
        try:
            self._recompute_pick(fixed_base)
            # Pick, then transit through the observation pose before going to
            # the basket, so the grasped seed is not carried through a long
            # high-speed move straight from the pick site.
            self.execute_pick(return_home=True)
            # 取种完成：seed:0（不清除 pending，周期继续）
            print("Auto: seed grabbed; MQTT grab-complete feedback (seed:0)")
            self._mqtt_publish_pending(RET_SUCCESS, command_name="seed")
            self._mqtt_feedback_phase = "seed_place"
            print("Auto: localizing basket...")
            self.localize_basket()
            selected_basket = (
                int(self.basket_aruco["_selected_id"]) + 1
                if self.basket_aruco is not None
                and self.basket_aruco.get("_selected_id") is not None
                else None
            )
            # After localization the arm is at the taught search pose; go
            # straight to placing (no extra trip back to the observation pose).
            approach_place = self._place_joint("basket_approach")
            release_place = self._place_joint("basket_release")
            self._validate_joint(approach_place)
            self._validate_joint(release_place)
            # 放置响应：识别成功后、开始放置前回 seed_place:1
            print("Auto: placement phase; MQTT placement response (seed_place:1)")
            self._mqtt_publish_pending(RET_STARTED, command_name="seed_place")
            print("Auto: placing into basket...")
            self.move_joint(approach_place)
            self.move_joint(
                release_place,
                speed_deg_s=self.arguments.servoj_precision_speed_deg_s,
            )
            if self.arguments.use_hand:
                print("Auto: ROH hand open (release)")
                self._hand_write_positions("open", self.arguments.hand_open_positions)
                # After releasing the seed, return to the level-1 protective
                # hold so the hand is never left open or fully clenched idle.
                self._hand_write_positions(
                    "hold level-1", self.arguments.hand_hold_positions
                )
            else:
                print("Auto: fake hand release simulated")
                time.sleep(0.5)
            self.move_joint(approach_place)
            if getattr(self, "dynamic_search_joint", None) is not None:
                print("Returning to the taught search pose after placing.")
                self.move_joint(self.dynamic_search_joint)
            self.place_done = True
            if selected_basket is not None:
                self._basket_occupied.add(selected_basket)
                print(
                    f"Basket number={selected_basket} marked as occupied. "
                    f"Occupied: {sorted(self._basket_occupied)}"
                )
            self._mqtt_publish_pending(
                RET_SUCCESS, final=True, command_name="seed_place"
            )
            print("Seed cycle complete. Returning to observation pose...")
            self.return_observation()
            self.pick_complete = False
            self.place_done = False
        except RuntimeError as error:
            print(f"Seed cycle failed: {error}")
            ret_code = RET_OCCUPIED if "occupied" in str(error) else RET_RECOG_FAILED
            self._mqtt_publish_pending(
                ret_code,
                final=True,
                command_name=self._mqtt_feedback_phase,
            )
            print("Returning to observation pose after failure...")
            try:
                self.return_observation()
            except RuntimeError:
                pass
            if self.arguments.use_hand:
                try:
                    self._hand_write_positions(
                        "hold level-1", self.arguments.hand_hold_positions
                    )
                except RuntimeError:
                    pass
            self.pick_complete = False
            self.place_done = False
            self.mqtt_auto_pick_pending = False

    def _handle_mqtt_command(self, command: MqttCommand) -> None:
        if self.mqtt_client is None:
            return
        if command.name == "estop":
            print("MQTT ESTOP: acknowledged; the arm motion is aborted/stopped")
            self.mqtt_client.publish_feedback("estop", RET_SUCCESS)
            return
        if self.mqtt_client.estop_requested:
            print(
                f"MQTT {command.name} rejected: emergency stop is active; "
                "restart the node to clear it"
            )
            self.mqtt_client.publish_feedback(
                command.name,
                RET_RECOG_FAILED,
                "emergency stop is active; restart the node",
                target_id=command.target_id,
            )
            return
        if self.mqtt_pending_command is not None:
            print(
                f"MQTT command busy: targetId={self.mqtt_pending_command.target_id}; "
                f"rejecting targetId={command.target_id} as occupied"
            )
            self.mqtt_client.publish_feedback(
                command.name, RET_OCCUPIED, target_id=command.target_id
            )
            return
        if command.name in ("trim_pick", "trim_place"):
            print(
                f"MQTT {command.name} received for targetId={command.target_id}, "
                "but trim poses are not configured; no robot motion sent"
            )
            self.mqtt_client.publish_feedback(
                command.name,
                RET_RECOG_FAILED,
                "trim poses are not configured on the seed node",
                target_id=command.target_id,
            )
            return
        if self.pick_complete or self.place_done:
            print(
                f"MQTT seed targetId={command.target_id} rejected: current cycle "
                "is not idle"
            )
            self.mqtt_client.publish_feedback(
                command.name, RET_OCCUPIED, target_id=command.target_id
            )
            return
        if self.arguments.use_hand:
            try:
                if self._hand_is_level5_closed():
                    print(
                        "MQTT seed rejected: ROH hand is already at level-5 closed "
                        "state; open it before starting a new pick"
                    )
                    self.mqtt_client.publish_feedback(
                        command.name, RET_OCCUPIED, target_id=command.target_id
                    )
                    return
            except RuntimeError as error:
                print(f"MQTT seed hand-state check failed: {error}")
                self.mqtt_client.publish_feedback(
                    command.name,
                    RET_RECOG_FAILED,
                    str(error),
                    target_id=command.target_id,
                )
                return
        basket_number = self.mqtt_target_map.get(command.target_id)
        if basket_number is None:
            basket_number = target_id_to_basket_number(command.target_id)
            if basket_number is not None:
                effective = apply_reversed_rows(
                    basket_number, self.arguments.reversed_rows
                )
                if effective != basket_number:
                    print(
                        f"Reversed-row mapping: targetId={command.target_id} "
                        f"(number {basket_number}) -> actual code number {effective}."
                    )
                    basket_number = effective
        if basket_number is not None and self.basket_aruco is not None:
            try:
                self.select_basket_number(basket_number)
            except RuntimeError as error:
                print(f"MQTT target map rejected: {error}")
                self.mqtt_client.publish_feedback(
                    command.name,
                    RET_RECOG_FAILED,
                    str(error),
                    target_id=command.target_id,
                )
                return
        self.mqtt_pending_command = command
        # 车端协议：收到命令后先回 seed:1（响应命令，开始取种）
        self._mqtt_feedback_phase = "seed"
        self.mqtt_client.publish_feedback(
            "seed", RET_STARTED, target_id=command.target_id
        )
        if self.arguments.fixed_pick_base_mm is not None:
            self.mqtt_auto_pick_pending = True
            print(
                f"MQTT seed accepted: targetId={command.target_id}. "
                "Fixed-point pick without vision detection."
            )
        else:
            print(
                f"MQTT seed accepted: targetId={command.target_id}. "
                "Click the object center, then press C; use R and G for place-only mode."
            )

    def _process_mqtt_commands(self) -> None:
        if self.mqtt_client is None:
            return
        for command in self.mqtt_client.drain_commands():
            try:
                self._handle_mqtt_command(command)
            except RuntimeError as error:
                print(f"MQTT command rejected: {error}")
                self.mqtt_client.publish_feedback(
                    command.name,
                    RET_RECOG_FAILED,
                    str(error),
                    target_id=command.target_id,
                )

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

    def execute_pick(self, return_home: bool = True) -> None:
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
        # Descend directly to the configured final grasp Z in ONE continuous
        # move (no intermediate stop at the safe height), so the approach
        # looks smooth. The final Z is the fixed taught value.
        self.target_reached = True
        self.target_pose = list(self.pick_grasp_pose)
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
        if return_home:
            print("Transit via observation pose before going to the basket.")
            self.return_observation()
        else:
            # MQTT seed cycle: keep the seed lifted near the pick site and go
            # straight to basket localization/placement. Returning home first
            # would carry the grasped seed through a long high-speed move.
            print("Holding the lifted pick pose for the basket place step.")
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
            print(f"Starting autonomous rack scan {scan_note}.")
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
            self._mqtt_publish_pending(RET_SUCCESS, final=True)
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
        self._mqtt_publish_pending(RET_SUCCESS, final=True)
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

    def stop(self) -> None:
        if self.mqtt_pending_command is not None:
            self._mqtt_publish_pending(
                RET_RECOG_FAILED,
                final=True,
                command_name=self._mqtt_feedback_phase,
            )
        if self.mqtt_client is not None:
            self.mqtt_client.stop()
        super().stop()

    def run(self) -> None:
        self.wait_for_robot()
        if self.arguments.observation_joints is not None:
            self.detect_joint = self._pad_joint(self.arguments.observation_joints)
        self.move_to_configured_observation()
        self.start_camera()
        if self.mqtt_client is not None:
            self.mqtt_client.start()
        if self.arguments.use_hand:
            # Protective initial state: keep the hand at level-1 hold so it never
            # hangs open or in an unknown pose while idle.
            try:
                self._hand_write_positions(
                    "hold level-1", self.arguments.hand_hold_positions
                )
            except RuntimeError as error:
                print(
                    f"WARNING: initial level-1 hold failed ({error}); "
                    "initialize the hand before starting a pick"
                )
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
                "r home | 1..9 select | 0 auto | type 10..192 via prompt"
            )
        else:
            print(
                "ADVANCED BASKET: b localize | g full pick-place | "
                "1..9 select | 0 auto | type 10..192 via prompt"
            )
        self.window_name = "TL mock pick-place to basket"
        window_name = self.window_name
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window_name, self._mouse_callback)
        if self.basket_aruco is not None:
            basket_camera_matrix, basket_distortion = camera_parameters(
                self.color_intrinsics
            )
            basket_marker_length_m = float(
                self.basket_aruco.get("marker_length_m", 0.054)
            )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self._process_mqtt_commands()
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
            if self.mqtt_auto_pick_pending:
                self.mqtt_auto_pick_pending = False
                try:
                    self._auto_seed_pick_place()
                except RuntimeError as error:
                    print(f"Seed cycle failed: {error}")
                    self._mqtt_fail_pending(error)
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
                    if self.arguments.use_hand and self._hand_is_level5_closed():
                        raise RuntimeError(
                            "ROH hand is already at level-5 closed state; open it "
                            "before pressing C"
                        )
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
                elif ord("1") <= key <= ord("9"):
                    if not self.arguments.target_only:
                        self.select_basket_number(key - ord("0"))
                elif key == ord("."):
                    if not self.arguments.target_only:
                        self._select_basket_number_prompt()
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
                self._mqtt_fail_pending(error)


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
