import argparse
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import yaml

from .basket_aruco import (
    ARUCO_DICTIONARY_NAME,
    basket_teach_status,
    base_marker_transform,
    camera_parameters,
    detect_marker_transforms,
    draw_marker_overlay,
    group_and_average_marker_samples,
    marker_transforms_with_depth_translation,
)
from .geometry import pose_to_transform
from .overhead_test import ReferenceOverheadTest, load_calibration


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
        description="Teach marker-relative basket approach and release poses"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path("~/tl_handeye_data/best/handeye_best.yaml"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("~/tl_mock_pick_place/basket_aruco.yaml"),
    )
    parser.add_argument("--marker-length-m", type=float, default=0.054)
    parser.add_argument(
        "--reversed-rows",
        nargs="*",
        default=[],
        metavar="ROW",
        help="rows whose ArUco codes are stuck in reversed order (e.g. A3): "
        "typed positions are mirrored to the actual code numbers",
    )
    parser.add_argument(
        "--teach-start-joint-deg",
        type=float,
        nargs=6,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="move to this fixed six-axis teach start pose at startup",
    )
    parser.add_argument(
        "--teach-start-joint-deg-b",
        type=float,
        nargs=6,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="B-side teach start pose (mirror of the A-side start, ~J1+180); "
        "press v in the teach window to move there",
    )
    parser.add_argument("--marker-ids", type=int, nargs="+", default=list(range(192)))
    parser.add_argument("--sample-window", type=int, default=30)
    parser.add_argument("--minimum-samples", type=int, default=8)
    parser.add_argument("--interp-points", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--servoj-speed-deg-s", type=float, default=15.0)
    parser.add_argument("--servoj-precision-speed-deg-s", type=float, default=8.0)
    parser.add_argument("--servoj-min-duration-sec", type=float, default=1.0)
    parser.add_argument("--arrival-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--max-joint-delta-deg", type=float, default=220.0)
    parser.add_argument("--allow-rejected", action="store_true")
    arguments = parser.parse_args(argv)
    if arguments.marker_length_m <= 0.0:
        parser.error("--marker-length-m must be positive")
    if arguments.minimum_samples <= 0:
        parser.error("--minimum-samples must be positive")
    if arguments.sample_window < arguments.minimum_samples:
        parser.error("--sample-window must be >= --minimum-samples")
    if arguments.interp_points != 0 and arguments.interp_points < 100:
        parser.error("--interp-points must be 0 (automatic) or at least 100")
    if arguments.servoj_speed_deg_s <= 0.0 or arguments.servoj_speed_deg_s > 45.0:
        parser.error("--servoj-speed-deg-s must be in (0, 45]")
    if (
        arguments.servoj_precision_speed_deg_s <= 0.0
        or arguments.servoj_precision_speed_deg_s > arguments.servoj_speed_deg_s
    ):
        parser.error("invalid ServoJ precision speed")
    if arguments.servoj_min_duration_sec <= 0.0:
        parser.error("--servoj-min-duration-sec must be positive")
    arguments.validation_log = Path("~/tl_handeye_validation/aruco_teach.jsonl")
    arguments.execute = True
    return arguments


class ArucoBasketTeachNode(ReferenceOverheadTest):
    def __init__(self, arguments, calibration_path, calibration, tool_camera):
        super().__init__(arguments, calibration_path, calibration, tool_camera)
        self.samples = deque(maxlen=int(arguments.sample_window))
        self._sample_tcp_history = deque(maxlen=10)
        self.frozen_markers = {}
        self.frozen_search_tcp = None
        self.frozen_search_joint = None
        self.selected_marker_id = None
        self.requested_marker_id = None
        self._marker_number_buffer = ""
        self.latest_marker_centers = {}
        self.current_side = "A"
        self.output_path = arguments.output.expanduser().resolve()

    @staticmethod
    def _matrix_list(transform):
        return np.asarray(transform, dtype=float).reshape(4, 4).tolist()

    def _load_output(self) -> dict:
        if not self.output_path.exists():
            return {}
        return yaml.safe_load(self.output_path.read_text(encoding="utf-8")) or {}

    def _write_output(self, data: dict) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )

    def _print_teach_progress(self, data: dict | None = None) -> None:
        current = self._load_output() if data is None else data
        status = basket_teach_status(current, self.arguments.marker_ids)
        complete = [marker_id + 1 for marker_id, value in status.items() if value["complete"]]
        incomplete = [marker_id + 1 for marker_id, value in status.items() if not value["complete"]]
        print(
            f"192-basket teach progress: complete={complete} ({len(complete)}/192), "
            f"incomplete={incomplete}"
        )
        records = current.get("markers", {}) if isinstance(current, dict) else {}
        fork_complete = []
        for marker_id in self.arguments.marker_ids:
            record = records.get(str(marker_id), records.get(marker_id, {})) or {}
            if all(
                name in record
                for name in ("search_pose", "fork_approach", "fork_insert", "fork_lift")
            ):
                fork_complete.append(marker_id + 1)
        print(
            f"U-fork teach progress: complete={fork_complete} ({len(fork_complete)}/192), "
            f"transfer={'yes' if current.get('fork_transfer_pose') else 'no'}"
        )

    def _commit_marker_number(self) -> None:
        """Commit the typed basket number (1..192) as the requested marker."""
        text_value = self._marker_number_buffer.strip()
        self._marker_number_buffer = ""
        if not text_value:
            self.requested_marker_id = None
            return
        number = int(text_value)
        if not 1 <= number <= 192:
            raise RuntimeError(f"basket number must be 1..192, got {number}")
        effective = apply_reversed_rows(number, self.arguments.reversed_rows)
        if effective != number:
            print(
                f"Reversed-row mapping: position number {number} -> actual code "
                f"number {effective} (ID{effective - 1})."
            )
        self.requested_marker_id = effective - 1
        print(f"Requested basket number={number}; press B to select it.")

    def select_basket(self) -> None:
        if self.latest_tcp is None or self.latest_joint is None:
            raise RuntimeError("/tcp_pose or /joint_states is missing")
        if time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError(
                "/tcp_pose is stale; hold the arm still until the pose "
                "refreshes, then press B again"
            )
        averaged = group_and_average_marker_samples(
            self.samples, self.arguments.minimum_samples
        )
        candidates = {
            marker_id: transform
            for marker_id, transform in averaged.items()
            if marker_id in self.latest_marker_centers
        }
        if not candidates:
            raise RuntimeError(
                "no marker has enough stable samples; keep the arm still and the "
                "target basket marker fully visible before pressing b"
            )
        if self.requested_marker_id is not None:
            if self.requested_marker_id not in candidates:
                raise RuntimeError(
                    f"requested basket number {self.requested_marker_id + 1} "
                    "is not visible or does not have enough stable samples"
                )
            selected_marker_id = self.requested_marker_id
        else:
            image_center = np.asarray([320.0, 240.0], dtype=float)
            selected_marker_id = min(
                candidates,
                key=lambda marker_id: float(
                    np.linalg.norm(self.latest_marker_centers[marker_id] - image_center)
                ),
            )
        self.selected_marker_id = int(selected_marker_id)
        self.frozen_markers = {
            self.selected_marker_id: candidates[self.selected_marker_id]
        }
        # Drop the pre-selection window: the next B press must sample only the
        # frames right before it, not the drag/return motion history.
        self.samples.clear()
        self.frozen_search_tcp = [float(value) for value in self.latest_tcp]
        self.frozen_search_joint = self._pad_joint(self.latest_joint)
        data = self._load_output()
        data.update(
            {
                "format_version": 2,
                "updated_utc": datetime.now(timezone.utc).isoformat(),
                "dictionary": ARUCO_DICTIONARY_NAME,
                "marker_length_m": float(self.arguments.marker_length_m),
                "number_to_id": {number: number - 1 for number in range(1, 193)},
                "calibration_file": str(self.calibration_path),
                "search_pose": {
                    "tcp": self.frozen_search_tcp,
                    "joint": self.frozen_search_joint,
                },
            }
        )
        data.setdefault("markers", {})
        marker_record = data["markers"].setdefault(
            str(self.selected_marker_id),
            {
                "number": self.selected_marker_id + 1,
                "aruco_id": self.selected_marker_id,
            },
        )
        marker_record["search_pose"] = {
            "tcp": self.frozen_search_tcp,
            "joint": self.frozen_search_joint,
            "T_base_marker": self._matrix_list(
                self.frozen_markers[self.selected_marker_id]
            ),
            "marker_length_m": float(self.arguments.marker_length_m),
            "translation_source": "solvepnp",
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        self._write_output(data)
        print(
            f"Selected basket number={self.selected_marker_id + 1}, "
            f"ArUco ID={self.selected_marker_id} at the saved search pose."
        )
        print(
            "Old basket poses: A=approach, P=release. Fork poses: "
            "F=approach, I=insert, L=lift; T=global transfer."
        )
        self._print_teach_progress(data)

    def record_pose(self, pose_name: str) -> None:
        if not self.frozen_markers:
            raise RuntimeError("press b to select the visible basket marker first")
        if self.latest_tcp is None or self.latest_joint is None:
            raise RuntimeError("/tcp_pose or /joint_states is missing")
        if time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError("/tcp_pose is stale")
        tcp = np.asarray(self.latest_tcp, dtype=float)
        base_tool = pose_to_transform(tcp[:3] / 1000.0, tcp[3:], "intrinsic_xyz")
        data = self._load_output()
        records = data.setdefault("markers", {})
        for marker_id, base_marker in self.frozen_markers.items():
            marker_tool = np.linalg.inv(base_marker) @ base_tool
            record = records.setdefault(
                str(marker_id), {"number": int(marker_id) + 1, "aruco_id": int(marker_id)}
            )
            record[pose_name] = {
                "T_marker_tool0": self._matrix_list(marker_tool),
                "taught_tcp": [float(value) for value in self.latest_tcp],
                "taught_joint": self._pad_joint(self.latest_joint),
                "recorded_utc": datetime.now(timezone.utc).isoformat(),
            }
        data["updated_utc"] = datetime.now(timezone.utc).isoformat()
        self._write_output(data)
        print(
            f"Saved {pose_name} for marker numbers "
            f"{[marker_id + 1 for marker_id in sorted(self.frozen_markers)]}"
        )
        if pose_name == "basket_approach" and tcp[1] < -710.0:
            print(
                f"NOTE: approach Y={tcp[1]:.0f} mm is near the IK workspace edge; "
                "if placement auto-adjusts later, re-teach 20-40 mm shallower"
            )
        if pose_name == "basket_release":
            approach = record.get("basket_approach", {}).get("taught_tcp")
            if approach is not None:
                drop = float(approach[2]) - float(tcp[2])
                print(f"basket_release is {drop:.0f} mm below basket_approach")
        print(f"file={self.output_path}")
        self._print_teach_progress(data)

    def record_transfer_pose(self) -> None:
        if self.latest_tcp is None or self.latest_joint is None:
            raise RuntimeError("/tcp_pose or /joint_states is missing")
        if time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError("/tcp_pose is stale")
        data = self._load_output()
        data["fork_transfer_pose"] = {
            "tcp": [float(value) for value in self.latest_tcp],
            "joint": self._pad_joint(self.latest_joint),
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        data["updated_utc"] = datetime.now(timezone.utc).isoformat()
        self._write_output(data)
        print(f"Saved fork_transfer_pose; file={self.output_path}")
        print("This is a global safe transfer pose shared by all baskets.")
        self._print_teach_progress(data)

    def _record_lower_observation(self) -> None:
        """d 键：把当前手臂位姿记录为“当前侧”的下排观测位。

        与 A/P/F/I/L 一样是“拖到位再按键记录”；重复按 d 覆盖旧值，
        写入 YAML 顶层 lower_observation_pose_a / lower_observation_pose_b，
        供运行端（mock_pick_place）按花盆编号分侧取用。
        """
        if self.latest_tcp is None or self.latest_joint is None:
            raise RuntimeError("/tcp_pose or /joint_states is missing")
        if time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError(
                "/tcp_pose is stale; hold the arm still until the pose "
                "refreshes, then press d again"
            )
        side = self.current_side
        joint = self._pad_joint(self.latest_joint)
        data = self._load_output()
        data[f"lower_observation_pose_{side.lower()}"] = {
            "side": side,
            "joint": joint,
            "tcp": [float(value) for value in self.latest_tcp],
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        data["updated_utc"] = datetime.now(timezone.utc).isoformat()
        self._write_output(data)
        print(
            f"Saved lower observation pose for side {side}: "
            f"joint={[round(float(value), 3) for value in joint[:6]]}"
        )
        print("下排 A1/A4/B1/B4 检测位（重复按 d 覆盖；v 切侧；g 验证回位）。")

    def _goto_lower_observation(self) -> None:
        """g 键：去“当前侧”已记录的下排观测位（记录后验证回位用）。"""
        side = self.current_side
        data = self._load_output()
        pose = data.get(f"lower_observation_pose_{side.lower()}")
        if not pose or "joint" not in pose:
            raise RuntimeError(
                f"side {side} lower observation pose is not recorded yet; "
                "drag the arm to the pose and press d first (press v to switch side)"
            )
        joint = self._pad_joint(pose["joint"])
        self._validate_joint(joint)
        print(
            f"Moving to side {side} lower observation pose "
            f"{[round(float(value), 3) for value in joint[:6]]} deg."
        )
        try:
            self.move_joint(joint)
        finally:
            self.close_servoj()

    def return_to_selected_search_pose(self) -> None:
        if self.frozen_search_joint is None or self.selected_marker_id is None:
            raise RuntimeError("press b to select a basket before pressing r")
        print(
            f"Returning to basket number {self.selected_marker_id + 1} search pose."
        )
        try:
            self.move_joint(self.frozen_search_joint)
        finally:
            self.close_servoj()
        print(
            "Returned and ServoJ released. Hold the physical drag button, translate "
            "the arm to center the next basket marker, release it, then press B."
        )

    def run(self) -> None:
        self.wait_for_robot()
        if self.arguments.teach_start_joint_deg is not None:
            start_joint = self._pad_joint(self.arguments.teach_start_joint_deg)
            self._validate_joint(start_joint)
            print(
                "Moving to the standard teach start pose "
                f"{[round(float(value), 3) for value in start_joint[:6]]} deg."
            )
            self.move_joint(start_joint)
        self.start_camera()
        camera_matrix, distortion = camera_parameters(self.color_intrinsics)
        print("Multi-basket ArUco teach mode: only v / g / R keys send robot motion.")
        print(f"marker black-square length={self.arguments.marker_length_m * 1000.0:.1f} mm")
        print(
            "Center one marker, then B select | A approach | P release | "
            "F fork-approach | I fork-insert | L fork-lift | T transfer | "
            "v switch side | d record lower-obs | g goto lower-obs | "
            "R return | Q quit"
        )
        print("Numbers 1..192 map to ArUco IDs 0..191 (8 rows x 24 baskets: A1-01..B4-24).")
        self._print_teach_progress()
        print(
            f"Current side: {self.current_side} "
            "(press v to switch; d records the lower observation pose)"
        )
        if self.arguments.reversed_rows:
            print(
                "Reversed-row code mapping active: "
                f"{list(self.arguments.reversed_rows)} (typed positions are mirrored "
                "to the actual code numbers)."
            )
        window_name = "TL basket ArUco teach - no motion"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
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
            image = np.asanyarray(color_frame.get_data()).copy()
            camera_markers, corners, ids, _ = detect_marker_transforms(
                image,
                camera_matrix,
                distortion,
                self.arguments.marker_length_m,
                self.arguments.marker_ids,
            )
            # 深度翻译（pnp_fallback_mm=0.0 强制 PnP 与运行时同源；深度只
            # 作交叉核对，不作为定位来源）
            camera_markers, _ = marker_transforms_with_depth_translation(
                camera_markers,
                corners,
                ids,
                lambda center: rs.rs2_deproject_pixel_to_point(
                    self.color_intrinsics,
                    [float(center[0]), float(center[1])],
                    self._median_depth(
                        depth_frame, int(round(center[0])), int(round(center[1]))
                    ),
                ),
                pnp_fallback_mm=0.0,
            )
            # 画面/检测不依赖 TCP（/tcp_pose 在 ServoJ 流后可能暂停发布）；
            # TCP 只影响采样，采样前单独检查。
            tcp_fresh = (
                self.latest_tcp is not None
                and time.monotonic() - self.latest_tcp_at <= 0.3
            )
            base_markers = {}
            if tcp_fresh:
                base_markers = {
                    marker_id: base_marker_transform(
                        self.latest_tcp, self.tool_camera, camera_marker
                    )
                    for marker_id, camera_marker in camera_markers.items()
                }
            # Only accumulate marker samples while the arm is truly static:
            # frames captured during the R-return motion or hand-dragging mix
            # several camera poses into one marker median and corrupt the
            # taught record (observed 100 mm X errors).
            if tcp_fresh:
                self._sample_tcp_history.append(
                    np.asarray(self.latest_tcp[:6], dtype=float)
                )
                if len(self._sample_tcp_history) >= 10:
                    drift = float(
                        np.max(
                            np.abs(
                                self._sample_tcp_history[-1]
                                - self._sample_tcp_history[0]
                            )
                        )
                    )
                    if drift < 0.5:
                        self.samples.append(base_markers)
            self.latest_marker_centers = {
                int(marker_id): np.mean(
                    np.asarray(marker_corners, dtype=float).reshape(-1, 2), axis=0
                )
                for marker_corners, marker_id in zip(corners, ids.reshape(-1))
            }
            preview = draw_marker_overlay(
                image,
                corners,
                ids,
                camera_markers,
                camera_matrix,
                distortion,
                self.arguments.marker_length_m * 0.5,
            )
            visible_numbers = [marker_id + 1 for marker_id in sorted(camera_markers)]
            selected_number = (
                None if self.selected_marker_id is None else self.selected_marker_id + 1
            )
            cv2.putText(
                preview,
                f"visible={visible_numbers} selected={selected_number} side={self.current_side}",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                preview,
                "1..192 B:select c:clear v:side d:record-lower g:goto-lower A/P F/I/L/T R:return Q:quit",
                (8, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46,
                (0, 255, 0),
                2,
            )
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            try:
                if key in (ord("q"), 27):
                    return
                if ord("0") <= key <= ord("9"):
                    digit = key - ord("0")
                    if len(self._marker_number_buffer) < 3:
                        self._marker_number_buffer += str(digit)
                    print(
                        f"Basket number buffer: {self._marker_number_buffer or '(empty)'}; "
                        "press B to select, c to clear"
                    )
                elif key == ord("c"):
                    self._marker_number_buffer = ""
                    print("Basket number buffer cleared.")
                elif key == ord("b"):
                    self._commit_marker_number()
                    self.select_basket()
                elif key == ord("a"):
                    self.record_pose("basket_approach")
                elif key == ord("p"):
                    self.record_pose("basket_release")
                elif key == ord("f"):
                    self.record_pose("fork_approach")
                elif key == ord("i"):
                    self.record_pose("fork_insert")
                elif key == ord("l"):
                    self.record_pose("fork_lift")
                elif key == ord("t"):
                    self.record_transfer_pose()
                elif key == ord("d"):
                    self._record_lower_observation()
                elif key == ord("g"):
                    self._goto_lower_observation()
                elif key == ord("v"):
                    if self.current_side == "A":
                        if self.arguments.teach_start_joint_deg_b is None:
                            print(
                                "No B-side teach start pose configured; set "
                                "--teach-start-joint-deg-b (TL_TEACH_START_JOINT_DEG_B)"
                            )
                        else:
                            start_joint = self._pad_joint(
                                self.arguments.teach_start_joint_deg_b
                            )
                            self._validate_joint(start_joint)
                            print(
                                "Moving to the B-side teach start pose "
                                f"{[round(float(value), 3) for value in start_joint[:6]]} deg."
                            )
                            self.move_joint(start_joint)
                            self.current_side = "B"
                            print(
                                "Current side: B (d records / g goes to the "
                                "B-side lower observation pose)"
                            )
                    else:
                        if self.arguments.teach_start_joint_deg is None:
                            print(
                                "No A-side teach start pose configured "
                                "(--teach-start-joint-deg); cannot switch back to side A"
                            )
                        else:
                            start_joint = self._pad_joint(
                                self.arguments.teach_start_joint_deg
                            )
                            self._validate_joint(start_joint)
                            print(
                                "Moving back to the A-side teach start pose "
                                f"{[round(float(value), 3) for value in start_joint[:6]]} deg."
                            )
                            self.move_joint(start_joint)
                            self.current_side = "A"
                            print(
                                "Current side: A (d records / g goes to the "
                                "A-side lower observation pose)"
                            )
                elif key == ord("r"):
                    self.return_to_selected_search_pose()
            except RuntimeError as error:
                print(f"Command rejected/stopped: {error}")


def main(argv=None):
    arguments = parse_args(argv)
    calibration_path, calibration, tool_camera = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    rclpy.init(args=[])
    node = ArucoBasketTeachNode(
        arguments, calibration_path, calibration, tool_camera
    )
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
