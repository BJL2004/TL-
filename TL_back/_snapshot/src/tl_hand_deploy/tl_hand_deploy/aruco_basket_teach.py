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
    parser.add_argument("--marker-length-m", type=float, default=0.058)
    parser.add_argument(
        "--teach-start-joint-deg",
        type=float,
        nargs=6,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="move to this fixed six-axis teach start pose at startup",
    )
    parser.add_argument("--marker-ids", type=int, nargs="+", default=list(range(6)))
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
        self.frozen_markers = {}
        self.frozen_search_tcp = None
        self.frozen_search_joint = None
        self.selected_marker_id = None
        self.latest_marker_centers = {}
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
            f"Six-basket teach progress: complete={complete} ({len(complete)}/6), "
            f"incomplete={incomplete}"
        )

    def select_basket(self) -> None:
        if self.latest_tcp is None or self.latest_joint is None:
            raise RuntimeError("/tcp_pose or /joint_states is missing")
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
        self.frozen_search_tcp = [float(value) for value in self.latest_tcp]
        self.frozen_search_joint = self._pad_joint(self.latest_joint)
        data = self._load_output()
        data.update(
            {
                "format_version": 2,
                "updated_utc": datetime.now(timezone.utc).isoformat(),
                "dictionary": ARUCO_DICTIONARY_NAME,
                "marker_length_m": float(self.arguments.marker_length_m),
                "number_to_id": {number: number - 1 for number in range(1, 7)},
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
            "translation_source": "aligned_d435_depth",
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        self._write_output(data)
        print(
            f"Selected basket number={self.selected_marker_id + 1}, "
            f"ArUco ID={self.selected_marker_id} at the saved search pose."
        )
        print("Drag to the safe approach pose, press A, then drag to release and press P.")
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
        print(f"file={self.output_path}")
        self._print_teach_progress(data)

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
        print("Multi-basket ArUco teach mode: only R sends robot motion.")
        print(f"marker black-square length={self.arguments.marker_length_m * 1000.0:.1f} mm")
        print("Center one marker, then B select | A approach | P release | R return | Q quit")
        print("Numbers 1..6 map to ArUco IDs 0..5.")
        self._print_teach_progress()
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
            if not color_frame or not depth_frame or self.latest_tcp is None:
                continue
            image = np.asanyarray(color_frame.get_data()).copy()
            camera_markers, corners, ids, _ = detect_marker_transforms(
                image,
                camera_matrix,
                distortion,
                self.arguments.marker_length_m,
                self.arguments.marker_ids,
            )
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
            )
            base_markers = {
                marker_id: base_marker_transform(
                    self.latest_tcp, self.tool_camera, camera_marker
                )
                for marker_id, camera_marker in camera_markers.items()
            }
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
                f"visible={visible_numbers} selected={selected_number}",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                preview,
                "B:select  A:approach  P:release  R:return  Q:quit",
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
                if key in (ord("b"), ord("d")):
                    self.select_basket()
                elif key == ord("a"):
                    self.record_pose("basket_approach")
                elif key == ord("p"):
                    self.record_pose("basket_release")
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
