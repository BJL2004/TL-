"""Standalone ArUco-relative U-fork basket teaching node.

This module intentionally contains no hand, gripper, MQTT, YOLO, or safety-proxy
client. It reads the TL robot feedback topics, shows the D435 image, and records
fork poses. Only the R key sends a return-to-search ServoJ command.
"""

from collections import deque
from datetime import datetime, timezone
import argparse
import math
from pathlib import Path
import shutil
import tempfile
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tl_ros2_interface.msg import CartesianPose
from tl_ros2_interface.srv import OpenServoJ


ARUCO_DICTIONARY_NAME = "DICT_4X4_250"
POSE_NAMES = ("fork_approach", "fork_insert", "fork_lift")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Standalone U-fork basket teaching; no hand or gripper control"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path("~/tl_handeye_data/best/handeye_best.yaml"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("~/tl_fork_teach/fork_basket_teach.yaml"),
    )
    parser.add_argument("--marker-length-m", type=float, default=0.058)
    parser.add_argument("--marker-ids", type=int, nargs="+", default=list(range(192)))
    parser.add_argument("--sample-window", type=int, default=30)
    parser.add_argument("--minimum-samples", type=int, default=8)
    parser.add_argument("--camera-serial", default="")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--servoj-speed-deg-s", type=float, default=8.0)
    parser.add_argument("--servoj-rate-hz", type=float, default=100.0)
    parser.add_argument("--servoj-min-duration-sec", type=float, default=1.0)
    parser.add_argument("--arrival-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--max-joint-delta-deg", type=float, default=220.0)
    parser.add_argument("--allow-rejected", action="store_true")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="enable actual robot motion for the R (return) key",
    )
    parser.add_argument("--minimum-insert-mm", type=float, default=20.0)
    parser.add_argument("--minimum-lift-mm", type=float, default=30.0)
    parser.add_argument("--minimum-transfer-mm", type=float, default=30.0)
    parser.add_argument(
        "--observation-joints",
        type=float,
        nargs=6,
        default=[-83.668, 67.725, 3.956, -2.101, -5.484, -1.431],
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="fixed observation (standby) joints; the O key writes these "
        "instead of the current arm pose",
    )
    parser.add_argument(
        "--maximum-cross-axis-mm",
        type=float,
        default=60.0,
        help="allowed cross-axis drift during manual teaching (F->I vertical "
        "drift and I->L horizontal drift); hand-dragging is never perfectly "
        "straight, so this is generous (default 60 mm)",
    )
    parser.add_argument("--maximum-attitude-change-deg", type=float, default=25.0)
    parser.add_argument(
        "--max-transfer-attitude-change-deg",
        type=float,
        default=120.0,
        help="allowed attitude change between the carry waypoint and the "
        "temporary hold pose (W->T). The cycle rotates IN PLACE at safe "
        "height before translating, so a large rotation is safe; this only "
        "limits how much the taught poses may differ",
    )
    arguments = parser.parse_args(argv)
    if arguments.marker_length_m <= 0.0:
        parser.error("--marker-length-m must be positive")
    if not arguments.marker_ids:
        parser.error("--marker-ids must not be empty")
    if any(marker_id < 0 or marker_id >= 250 for marker_id in arguments.marker_ids):
        parser.error("--marker-ids must be in the DICT_4X4_250 range 0..249")
    if arguments.minimum_samples <= 0:
        parser.error("--minimum-samples must be positive")
    if arguments.sample_window < arguments.minimum_samples:
        parser.error("--sample-window must be >= --minimum-samples")
    if arguments.servoj_speed_deg_s <= 0.0 or arguments.servoj_speed_deg_s > 45.0:
        parser.error("--servoj-speed-deg-s must be in (0, 45]")
    if arguments.servoj_rate_hz <= 0.0 or arguments.servoj_rate_hz > 200.0:
        parser.error("--servoj-rate-hz must be in (0, 200]")
    if arguments.servoj_min_duration_sec <= 0.0:
        parser.error("--servoj-min-duration-sec must be positive")
    if arguments.arrival_tolerance_deg <= 0.0:
        parser.error("--arrival-tolerance-deg must be positive")
    if arguments.timeout_sec <= 0.0:
        parser.error("--timeout-sec must be positive")
    if arguments.max_joint_delta_deg <= 0.0:
        parser.error("--max-joint-delta-deg must be positive")
    if arguments.minimum_insert_mm <= 0.0:
        parser.error("--minimum-insert-mm must be positive")
    if arguments.minimum_lift_mm <= 0.0:
        parser.error("--minimum-lift-mm must be positive")
    if arguments.minimum_transfer_mm <= 0.0:
        parser.error("--minimum-transfer-mm must be positive")
    if arguments.maximum_cross_axis_mm <= 0.0:
        parser.error("--maximum-cross-axis-mm must be positive")
    if arguments.maximum_attitude_change_deg <= 0.0:
        parser.error("--maximum-attitude-change-deg must be positive")
    if arguments.maximum_attitude_change_deg > 30.0:
        parser.error(
            "--maximum-attitude-change-deg must not exceed 30 while the "
            "basket remains on the fork"
        )
    if arguments.max_transfer_attitude_change_deg <= 0.0:
        parser.error("--max-transfer-attitude-change-deg must be positive")
    return arguments


def load_calibration(path: Path, allow_rejected: bool):
    calibration_path = path.expanduser().resolve()
    if not calibration_path.exists():
        raise RuntimeError(f"calibration file is missing: {calibration_path}")
    calibration = yaml.safe_load(calibration_path.read_text(encoding="utf-8")) or {}
    accepted = bool(calibration.get("accepted", False))
    if not accepted and not allow_rejected:
        raise RuntimeError(
            "calibration is rejected; use --allow-rejected only for coarse teaching"
        )
    convention = str(calibration.get("rpy_convention", "intrinsic_xyz"))
    if convention != "intrinsic_xyz":
        raise RuntimeError(
            f"this program requires intrinsic_xyz calibration, got {convention}"
        )
    transform = np.asarray(calibration.get("T_tool0_camera"), dtype=float).reshape(4, 4)
    if not np.all(np.isfinite(transform)):
        raise RuntimeError("T_tool0_camera contains non-finite values")
    return calibration_path, calibration, transform


def calibration_intrinsics(calibration):
    matrix = calibration.get("camera_matrix")
    distortion = calibration.get("distortion")
    if matrix is None or distortion is None:
        return None, None
    camera_matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
    coefficients = np.asarray(distortion, dtype=float).reshape(-1)
    if coefficients.size < 4:
        return None, None
    return camera_matrix, coefficients.reshape(-1, 1)


def pose_to_transform(tcp):
    from scipy.spatial.transform import Rotation

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_euler(
        "XYZ", np.asarray(tcp[3:6], dtype=float), degrees=False
    ).as_matrix()
    transform[:3, 3] = np.asarray(tcp[:3], dtype=float) / 1000.0
    return transform


def transform_to_list(transform):
    return np.asarray(transform, dtype=float).reshape(4, 4).tolist()


def average_transforms(transforms):
    from scipy.spatial.transform import Rotation

    values = [np.asarray(value, dtype=float).reshape(4, 4) for value in transforms]
    result = np.eye(4, dtype=float)
    result[:3, 3] = np.mean([value[:3, 3] for value in values], axis=0)
    result[:3, :3] = Rotation.concatenate(
        [Rotation.from_matrix(value[:3, :3]) for value in values]
    ).mean().as_matrix()
    return result


def minimum_jerk(start, target, count):
    start = np.asarray(start, dtype=float)
    target = np.asarray(target, dtype=float)
    if count <= 1:
        return [target.tolist()]
    result = []
    for value in np.linspace(0.0, 1.0, count):
        blend = 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5
        result.append((start + (target - start) * blend).tolist())
    return result


def pose_motion_metrics(start_tcp, target_tcp):
    from scipy.spatial.transform import Rotation

    start = np.asarray(start_tcp, dtype=float).reshape(6)
    target = np.asarray(target_tcp, dtype=float).reshape(6)
    delta = target[:3] - start[:3]
    start_rotation = Rotation.from_euler("XYZ", start[3:6], degrees=False)
    target_rotation = Rotation.from_euler("XYZ", target[3:6], degrees=False)
    attitude_change_deg = math.degrees((start_rotation.inv() * target_rotation).magnitude())
    return {
        "delta_base_mm": [float(value) for value in delta],
        "horizontal_mm": float(np.linalg.norm(delta[:2])),
        "vertical_mm": float(delta[2]),
        "distance_mm": float(np.linalg.norm(delta)),
        "attitude_change_deg": float(attitude_change_deg),
    }


class ForkBasketTeachNode(Node):
    def __init__(self, arguments, calibration_path, calibration, tool_camera):
        super().__init__("tl_fork_basket_teach")
        self.arguments = arguments
        self.calibration_path = calibration_path
        self.calibration = calibration
        self.tool_camera = tool_camera
        self.output_path = arguments.output.expanduser().resolve()
        self.calibration_matrix, self.calibration_distortion = calibration_intrinsics(
            calibration
        )
        self.latest_joint = None
        self.latest_joint_at = 0.0
        self.latest_tcp = None
        self.latest_tcp_at = 0.0
        self._sample_tcp_history = deque(maxlen=10)
        self.samples = deque(maxlen=int(arguments.sample_window))
        self.latest_marker_centers = {}
        self.latest_base_markers = {}
        self.selected_marker_id = None
        self.requested_marker_id = None
        self._marker_number_buffer = ""
        self.frozen_base_marker = None
        self.frozen_search_joint = None
        self.frozen_search_tcp = None
        self.teach_stage = "idle"
        self.servoj_open = False
        self.pipeline = None
        self.align = None
        self.color_intrinsics = None
        self.camera_matrix = None
        self.camera_distortion = None
        self.servoj_publisher = self.create_publisher(
            Float64MultiArray, "/tl_driver/set_servoj_pos", 1
        )
        self.open_servoj_client = self.create_client(OpenServoJ, "/tl_driver/open_servoj")
        self.close_servoj_client = self.create_client(Trigger, "/tl_driver/close_servoj")
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 20)
        self.create_subscription(CartesianPose, "/tcp_pose", self._tcp_callback, 20)

    def _joint_callback(self, message):
        self.latest_joint = [float(value) for value in message.position]
        self.latest_joint_at = time.monotonic()

    def _tcp_callback(self, message):
        self.latest_tcp = [
            float(message.position.x),
            float(message.position.y),
            float(message.position.z),
            float(message.rpy.x),
            float(message.rpy.y),
            float(message.rpy.z),
        ]
        self.latest_tcp_at = time.monotonic()

    @staticmethod
    def _pad_joint(values):
        result = [float(value) for value in values[:7]]
        return (result + [0.0] * 7)[:7]

    def _wait_feedback(self, timeout_sec=8.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.latest_joint is not None
                and self.latest_tcp is not None
                and time.monotonic() - self.latest_joint_at < 0.5
                and time.monotonic() - self.latest_tcp_at < 0.5
            ):
                return
        raise RuntimeError("missing fresh /joint_states or /tcp_pose")

    def _call(self, client, request, timeout_sec):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError("ROS service call timed out")
        return future.result()

    def _start_camera(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.arguments.camera_serial:
            config.enable_device(self.arguments.camera_serial)
        config.enable_stream(
            rs.stream.depth,
            self.arguments.camera_width,
            self.arguments.camera_height,
            rs.format.z16,
            self.arguments.camera_fps,
        )
        config.enable_stream(
            rs.stream.color,
            self.arguments.camera_width,
            self.arguments.camera_height,
            rs.format.bgr8,
            self.arguments.camera_fps,
        )
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.color_intrinsics = (
            profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        )
        profile_matrix = np.array(
            [
                [self.color_intrinsics.fx, 0.0, self.color_intrinsics.ppx],
                [0.0, self.color_intrinsics.fy, self.color_intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        profile_distortion = np.asarray(self.color_intrinsics.coeffs, dtype=float)
        if self.calibration_matrix is not None:
            self.camera_matrix = self.calibration_matrix
        else:
            self.camera_matrix = profile_matrix
        self.camera_distortion = self.calibration_distortion
        if self.camera_distortion is None:
            self.camera_distortion = profile_distortion.reshape(-1, 1)
        print(
            f"D435 started: {self.arguments.camera_width}x"
            f"{self.arguments.camera_height}@{self.arguments.camera_fps}"
        )

    def _median_depth(self, depth_frame, center_x, center_y):
        values = []
        radius = 3
        for y in range(center_y - radius, center_y + radius + 1):
            for x in range(center_x - radius, center_x + radius + 1):
                if 0 <= x < self.arguments.camera_width and 0 <= y < self.arguments.camera_height:
                    value = float(depth_frame.get_distance(x, y))
                    if 0.05 < value < 3.0:
                        values.append(value)
        if not values:
            return None
        return float(np.median(values))

    def _detect_markers(self, color_image, depth_frame):
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(getattr(aruco, ARUCO_DICTIONARY_NAME))
        if hasattr(aruco, "DetectorParameters"):
            parameters = aruco.DetectorParameters()
        else:
            parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            color_image, dictionary, parameters=parameters
        )
        # 过渡期老码兼容：DICT_4X4_250 未检出的码再按 DICT_4X4_50 回退识别
        # （新旧码 ID 0..5 在 A1-01..A1-06 上编号一致；新码就位后可删）
        legacy_corners, legacy_ids, _ = aruco.detectMarkers(
            color_image,
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            parameters=parameters,
        )
        if legacy_ids is not None and len(legacy_ids) > 0:
            found_ids = (
                set(int(value) for value in ids.reshape(-1))
                if ids is not None
                else set()
            )
            for index, legacy_corner in enumerate(legacy_corners):
                legacy_id = int(legacy_ids[index][0])
                if legacy_id in found_ids:
                    continue
                if ids is None:
                    corners = legacy_corner.reshape(1, 4, 2)
                    ids = np.asarray([[legacy_id]], dtype=np.int32)
                else:
                    corners = np.concatenate(
                        [corners, legacy_corner.reshape(1, 4, 2)], axis=0
                    )
                    ids = np.concatenate(
                        [ids, np.asarray([[legacy_id]], dtype=np.int32)], axis=0
                    )
                found_ids.add(legacy_id)
        transforms = {}
        centers = {}
        if ids is None:
            return transforms, centers, corners, ids
        half = float(self.arguments.marker_length_m) * 0.5
        object_points = np.asarray(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
            dtype=np.float32,
        )
        for marker_corners, marker_id_array in zip(corners, ids.reshape(-1)):
            marker_id = int(marker_id_array)
            if marker_id not in self.arguments.marker_ids:
                continue
            image_points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
            centers[marker_id] = np.mean(image_points, axis=0)
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.camera_distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue
            rotation, _ = cv2.Rodrigues(rvec)
            # Always use the PnP translation: the D435 depth stream jumps by
            # 100+ mm under changing light, and mixing depth/PnP translations
            # corrupted the seed-flow taught records and runtime localization
            # (fixed there with forced PnP). PnP depends only on the 2D
            # corners and the known marker size.
            transform = np.eye(4, dtype=float)
            transform[:3, :3] = rotation
            transform[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
            transforms[marker_id] = transform
        return transforms, centers, corners, ids

    def _base_marker_transforms(self, camera_transforms):
        if self.latest_tcp is None:
            return {}
        base_tool = pose_to_transform(self.latest_tcp)
        return {
            marker_id: base_tool @ self.tool_camera @ camera_transform
            for marker_id, camera_transform in camera_transforms.items()
        }

    def _draw_preview(self, image, corners, ids, transforms):
        preview = image.copy()
        if ids is not None and len(ids):
            cv2.aruco.drawDetectedMarkers(preview, corners, ids)
        visible = [marker_id + 1 for marker_id in sorted(transforms)]
        selected = None if self.selected_marker_id is None else self.selected_marker_id + 1
        cv2.putText(
            preview,
            f"visible={visible} selected={selected}",
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            preview,
            "B select  F approach  I horizontal insert  L vertical lift  T temporary  W waypoint  O observation  R return  Q quit",
            (8, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            preview,
            "type the basket number (1..192) then B; this program does not control a hand",
            (8, 74),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (255, 255, 0),
            2,
        )
        return preview

    def _commit_marker_number(self):
        """Commit the typed basket number (1..192) as the requested marker."""
        text = self._marker_number_buffer.strip()
        self._marker_number_buffer = ""
        if not text:
            self.requested_marker_id = None
            return
        number = int(text)
        if not 1 <= number <= 192:
            raise RuntimeError(f"basket number must be 1..192, got {number}")
        self.requested_marker_id = number - 1
        print(f"Requested basket number={number}; press B to freeze it.")

    def _select_marker(self):
        if self.latest_tcp is None or self.latest_joint is None:
            raise RuntimeError("robot feedback is not ready")
        if time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError(
                "/tcp_pose is stale; hold the arm still until the pose "
                "refreshes, then press B again"
            )
        candidates = {}
        for marker_id in self.arguments.marker_ids:
            values = [sample[marker_id] for sample in self.samples if marker_id in sample]
            if len(values) >= self.arguments.minimum_samples and marker_id in self.latest_marker_centers:
                candidates[marker_id] = average_transforms(values)
        if self.requested_marker_id is not None:
            if self.requested_marker_id not in candidates:
                raise RuntimeError(
                    f"requested marker number {self.requested_marker_id + 1} "
                    "is not visible or does not have enough stable samples"
                )
            marker_id = self.requested_marker_id
        elif candidates:
            image_center = np.asarray([self.arguments.camera_width / 2.0, self.arguments.camera_height / 2.0])
            marker_id = min(
                candidates,
                key=lambda value: float(
                    np.linalg.norm(self.latest_marker_centers[value] - image_center)
                ),
            )
        else:
            raise RuntimeError(
                "no marker has enough stable samples; hold the marker still and keep it visible"
            )
        self.selected_marker_id = int(marker_id)
        # Drop the pre-selection window: the next B press must sample only the
        # frames right before it, not the drag/return motion history.
        self.samples.clear()
        self.teach_stage = "selected"
        self.frozen_base_marker = candidates[marker_id]
        self.frozen_search_tcp = [float(value) for value in self.latest_tcp]
        self.frozen_search_joint = self._pad_joint(self.latest_joint)
        data = self._load_output()
        data["format_version"] = 2
        data["purpose"] = "standalone_u_fork_basket_teaching"
        data["dictionary"] = ARUCO_DICTIONARY_NAME
        data["marker_length_m"] = float(self.arguments.marker_length_m)
        data["number_to_id"] = {number: number - 1 for number in range(1, 7)}
        data["calibration_file"] = str(self.calibration_path)
        data["search_pose"] = {
            "tcp": self.frozen_search_tcp,
            "joint": self.frozen_search_joint,
        }
        records = data.setdefault("markers", {})
        record = records.setdefault(
            str(marker_id), {"number": marker_id + 1, "aruco_id": marker_id}
        )
        record["number"] = marker_id + 1
        record["aruco_id"] = marker_id
        record["search_pose"] = {
            "tcp": self.frozen_search_tcp,
            "joint": self.frozen_search_joint,
            "T_base_marker": transform_to_list(self.frozen_base_marker),
            "marker_length_m": float(self.arguments.marker_length_m),
            "translation_source": "aligned_d435_depth",
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        data["updated_utc"] = datetime.now(timezone.utc).isoformat()
        self._write_output(data)
        print(
            f"Selected basket number={marker_id + 1}; search pose saved. "
            "Drag the U-fork to the next pose."
        )

    def _load_output(self):
        if not self.output_path.exists():
            return {}
        value = yaml.safe_load(self.output_path.read_text(encoding="utf-8")) or {}
        if not isinstance(value, dict):
            raise RuntimeError(f"output file is not a YAML object: {self.output_path}")
        return value

    def _write_output(self, data):
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        if self.output_path.exists():
            archive_dir = self.output_path.parent / "archive"
            archive_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            shutil.copy2(
                self.output_path,
                archive_dir / f"{self.output_path.stem}_{stamp}.yaml",
            )
        payload = yaml.safe_dump(data, allow_unicode=True, sort_keys=False)
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=self.output_path.parent,
            prefix=f".{self.output_path.name}.", suffix=".tmp", delete=False
        ) as stream:
            stream.write(payload)
            temporary_path = Path(stream.name)
        temporary_path.replace(self.output_path)

    def _record_pose(self, pose_name):
        if pose_name not in POSE_NAMES:
            raise ValueError(f"unsupported fork pose {pose_name}")
        if self.frozen_base_marker is None or self.selected_marker_id is None:
            raise RuntimeError("press B to select a basket first")
        if self.latest_tcp is None or self.latest_joint is None:
            raise RuntimeError("robot feedback is not ready")
        if time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError("/tcp_pose is stale")
        base_tool = pose_to_transform(self.latest_tcp)
        marker_tool = np.linalg.inv(self.frozen_base_marker) @ base_tool
        data = self._load_output()
        record = data.setdefault("markers", {}).setdefault(
            str(self.selected_marker_id),
            {"number": self.selected_marker_id + 1, "aruco_id": self.selected_marker_id},
        )
        previous_name = None
        motion_type = "reference"
        metrics = None
        if pose_name == "fork_approach":
            if self.teach_stage not in ("selected", "approach"):
                raise RuntimeError(
                    "restart this basket sequence with B, then record F"
                )
            record.pop("fork_insert", None)
            record.pop("fork_lift", None)
        elif pose_name == "fork_insert":
            if self.teach_stage != "approach":
                raise RuntimeError("record F immediately before I")
            previous_name = "fork_approach"
            motion_type = "horizontal_insert"
            metrics = pose_motion_metrics(
                record[previous_name]["taught_tcp"], self.latest_tcp
            )
            if metrics["horizontal_mm"] < self.arguments.minimum_insert_mm:
                raise RuntimeError(
                    f"F->I horizontal travel is only {metrics['horizontal_mm']:.1f} mm; "
                    f"need at least {self.arguments.minimum_insert_mm:.1f} mm"
                )
            if abs(metrics["vertical_mm"]) > self.arguments.maximum_cross_axis_mm:
                raise RuntimeError(
                    f"F->I changed Z by {metrics['vertical_mm']:.1f} mm; "
                    "insert the U-fork horizontally"
                )
            record.pop("fork_lift", None)
        elif pose_name == "fork_lift":
            if self.teach_stage != "insert":
                raise RuntimeError("record I immediately before L")
            previous_name = "fork_insert"
            motion_type = "vertical_lift"
            metrics = pose_motion_metrics(
                record[previous_name]["taught_tcp"], self.latest_tcp
            )
            if metrics["vertical_mm"] < self.arguments.minimum_lift_mm:
                raise RuntimeError(
                    f"I->L upward travel is only {metrics['vertical_mm']:.1f} mm; "
                    f"need at least {self.arguments.minimum_lift_mm:.1f} mm"
                )
            if metrics["horizontal_mm"] > self.arguments.maximum_cross_axis_mm:
                raise RuntimeError(
                    f"I->L drifted {metrics['horizontal_mm']:.1f} mm in XY; "
                    "lift the basket vertically"
                )
        if metrics is not None and (
            metrics["attitude_change_deg"]
            > self.arguments.maximum_attitude_change_deg
        ):
            raise RuntimeError(
                f"{previous_name}->{pose_name} changed attitude by "
                f"{metrics['attitude_change_deg']:.1f} deg; keep the fork attitude "
                f"within {self.arguments.maximum_attitude_change_deg:.1f} deg"
            )
        record[pose_name] = {
            "T_marker_tool0": transform_to_list(marker_tool),
            "taught_tcp": [float(value) for value in self.latest_tcp],
            "taught_joint": self._pad_joint(self.latest_joint),
            "motion_type": motion_type,
            "motion_from": previous_name,
            "motion_metrics": metrics,
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        self.teach_stage = {
            "fork_approach": "approach",
            "fork_insert": "insert",
            "fork_lift": "lift",
        }[pose_name]
        data["updated_utc"] = datetime.now(timezone.utc).isoformat()
        self._write_output(data)
        print(
            f"Saved {pose_name} for basket number {self.selected_marker_id + 1}: "
            f"TCP={[round(float(value), 3) for value in self.latest_tcp]}"
        )

    def _record_carry_waypoint(self):
        """Record the global high-altitude carry waypoint with the W key.

        The arm carries the basket through this pose between the lift pose
        and the temporary pose, so the transfer path is exactly what was
        physically reached during teaching.
        """
        if self.latest_tcp is None or self.latest_joint is None:
            raise RuntimeError("robot feedback is not ready")
        if time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError("/tcp_pose is stale")
        if self.selected_marker_id is None or self.teach_stage != "lift":
            raise RuntimeError("record B, F, I and L before W")
        data = self._load_output()
        record = data.get("markers", {}).get(str(self.selected_marker_id), {})
        lift_entry = record.get("fork_lift") or {}
        if "taught_tcp" not in lift_entry:
            raise RuntimeError("fork_lift is missing; record L before W")
        metrics = pose_motion_metrics(lift_entry["taught_tcp"], self.latest_tcp)
        if metrics["attitude_change_deg"] > self.arguments.maximum_attitude_change_deg:
            raise RuntimeError(
                "loaded carry waypoint changes attitude by "
                f"{metrics['attitude_change_deg']:.1f} deg; keep the basket level"
            )
        data["carry_waypoint"] = {
            "tcp": [float(value) for value in self.latest_tcp],
            "joint": self._pad_joint(self.latest_joint),
            "source_marker_number": self.selected_marker_id + 1,
            "motion_from": "fork_lift",
            "motion_metrics": metrics,
            "basket_remains_on_fork": True,
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        data["updated_utc"] = datetime.now(timezone.utc).isoformat()
        self._write_output(data)
        self.teach_stage = "waypoint"
        print(
            "Saved global carry_waypoint. The basket remains on the fork; "
            "move to the fixed temporary hold and press T."
        )

    def _record_observation(self):
        """Move to the fixed observation (hand detection) pose and record it.

        The O key first drives the arm to the --observation-joints pose,
        then writes it into the YAML, independent of the current pose.
        """
        if self.teach_stage in ("lift", "waypoint", "transfer"):
            raise RuntimeError(
                "the basket is on the fork; press R to return it before O"
            )
        joints = list(self.arguments.observation_joints) + [0.0]
        print(
            "Moving to the observation (hand detection) pose: "
            f"{list(self.arguments.observation_joints)}"
        )
        if self.arguments.execute:
            self._move_joint(self._pad_joint(joints))
        data = self._load_output()
        data["observation_pose"] = {
            "joint": joints,
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        data["updated_utc"] = datetime.now(timezone.utc).isoformat()
        self._write_output(data)
        print(
            "Saved observation_pose (fixed hand detection pose): "
            f"{list(self.arguments.observation_joints)}"
        )

    def _record_transfer(self):
        if self.latest_tcp is None or self.latest_joint is None:
            raise RuntimeError("robot feedback is not ready")
        if time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError("/tcp_pose is stale")
        data = self._load_output()
        if self.selected_marker_id is None or self.teach_stage != "waypoint":
            raise RuntimeError("record B, F, I, L and W before T")
        record = data.get("markers", {}).get(str(self.selected_marker_id), {})
        lift_entry = record.get("fork_lift")
        if not lift_entry or "taught_tcp" not in lift_entry:
            raise RuntimeError("fork_lift is missing; record L before T")
        waypoint = data.get("carry_waypoint") or {}
        if "tcp" not in waypoint or "joint" not in waypoint:
            raise RuntimeError("carry_waypoint is missing; record W before T")
        overall_metrics = pose_motion_metrics(lift_entry["taught_tcp"], self.latest_tcp)
        waypoint_metrics = pose_motion_metrics(waypoint["tcp"], self.latest_tcp)
        if overall_metrics["horizontal_mm"] < self.arguments.minimum_transfer_mm:
            raise RuntimeError(
                f"L->T horizontal travel is only {overall_metrics['horizontal_mm']:.1f} mm; "
                f"need at least {self.arguments.minimum_transfer_mm:.1f} mm"
            )
        if (
            waypoint_metrics["attitude_change_deg"]
            > self.arguments.max_transfer_attitude_change_deg
        ):
            raise RuntimeError(
                f"W->T changed attitude by {waypoint_metrics['attitude_change_deg']:.1f} deg; "
                "keep the loaded fork attitude fixed while teaching the temporary hold pose"
            )
        data["fork_transfer_pose"] = {
            "tcp": [float(value) for value in self.latest_tcp],
            "joint": self._pad_joint(self.latest_joint),
            "source_marker_number": self.selected_marker_id + 1,
            "motion_type": "fixed_temporary_hold_pose",
            "motion_from": "carry_waypoint",
            "motion_metrics": waypoint_metrics,
            "overall_motion_from_lift": overall_metrics,
            "basket_remains_on_fork": True,
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
        }
        data["motion_contract"] = {
            "sequence": [
                "fork_approach",
                "fork_insert_horizontal",
                "fork_lift_vertical",
                "carry_waypoint_loaded",
                "fork_transfer_fixed_hold",
            ],
            "return_sequence": [
                "fork_transfer_fixed_hold",
                "carry_waypoint_loaded",
                "fork_lift_vertical",
                "fork_insert_horizontal",
                "fork_approach",
                "search_pose",
            ],
            "basket_loaded_between": [
                "fork_lift_vertical",
                "fork_insert_horizontal",
            ],
            "maximum_cross_axis_mm": float(self.arguments.maximum_cross_axis_mm),
            "maximum_attitude_change_deg": float(
                self.arguments.maximum_attitude_change_deg
            ),
        }
        data["updated_utc"] = datetime.now(timezone.utc).isoformat()
        self._write_output(data)
        self.teach_stage = "transfer"
        print("Saved global fork_transfer_pose. T recorded data only; no motion was sent.")

    def _assert_servoj_exclusive(self):
        publishers = self.get_publishers_info_by_topic("/tl_driver/set_servoj_pos")
        if len(publishers) > 1:
            owners = sorted(
                f"{publisher.node_namespace}{publisher.node_name}"
                for publisher in publishers
            )
            raise RuntimeError(
                f"ServoJ topic has multiple publishers {owners}; stop other motion programs"
            )

    def _open_servoj(self):
        if self.servoj_open:
            return
        self._assert_servoj_exclusive()
        if not self.open_servoj_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/tl_driver/open_servoj is unavailable")
        request = OpenServoJ.Request()
        request.vmax = [float(self.arguments.servoj_speed_deg_s)] * 7
        request.amax = [3000.0] * 7
        request.jmax = [50000.0] * 7
        response = self._call(self.open_servoj_client, request, 5.0)
        if hasattr(response, "success") and not response.success:
            raise RuntimeError(f"OpenServoJ failed: {getattr(response, 'message', '')}")
        self.servoj_open = True

    def _close_servoj(self):
        if not self.servoj_open:
            return
        if not self.close_servoj_client.wait_for_service(timeout_sec=1.0):
            self.servoj_open = False
            return
        try:
            self._call(self.close_servoj_client, Trigger.Request(), 3.0)
        finally:
            self.servoj_open = False

    def _check_estop(self):
        """Raise when an MQTT estop command is pending so the motion aborts.

        MQTT nodes set the flag from the paho callback thread; the keyboard
        nodes never create an MQTT client, so this is a no-op for them.
        """
        client = getattr(self, "mqtt_client", None)
        if client is not None and getattr(client, "estop_requested", False):
            raise RuntimeError("ESTOP requested: motion aborted")

    def _move_joint(self, target):
        if self.latest_joint is None:
            raise RuntimeError("/joint_states is missing")
        target = self._pad_joint(target)
        start = self._pad_joint(self.latest_joint)
        maximum_delta = float(np.max(np.abs(np.asarray(target) - np.asarray(start))))
        if maximum_delta > self.arguments.max_joint_delta_deg:
            raise RuntimeError(
                f"return target changes a joint by {maximum_delta:.1f} deg, "
                f"over limit {self.arguments.max_joint_delta_deg:.1f} deg"
            )
        if not self.arguments.execute:
            print(f"DRY RUN: would return to joints={np.round(target[:6], 3).tolist()}")
            return
        self._open_servoj()
        count = max(
            100,
            int(
                math.ceil(
                    maximum_delta
                    / self.arguments.servoj_speed_deg_s
                    * self.arguments.servoj_rate_hz
                )
            ),
            int(math.ceil(self.arguments.servoj_min_duration_sec * self.arguments.servoj_rate_hz)),
        )
        trajectory = minimum_jerk(start, target, count)
        period = 1.0 / self.arguments.servoj_rate_hz
        print(f"Returning with {len(trajectory)} direct ServoJ points.")
        for waypoint in trajectory:
            self._check_estop()
            message = Float64MultiArray()
            message.data = [float(value) for value in waypoint]
            self.servoj_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        deadline = time.monotonic() + self.arguments.timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._check_estop()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint is not None:
                error = float(
                    np.max(np.abs(np.asarray(self.latest_joint[:6]) - np.asarray(target[:6])))
                )
                if error <= self.arguments.arrival_tolerance_deg:
                    print(f"Returned: max joint error={error:.3f} deg")
                    return
        raise RuntimeError("ServoJ return arrival timeout")

    def _return_to_search(self):
        if self.selected_marker_id is None:
            raise RuntimeError("press B to select a basket before R")
        if self.frozen_search_joint is None:
            data = self._load_output()
            record = data.get("markers", {}).get(str(self.selected_marker_id), {})
            search = record.get("search_pose")
            if not search or "joint" not in search:
                raise RuntimeError("no search_pose recorded for the selected basket")
            search_joint = self._pad_joint(search["joint"])
            print("Loading the search pose from the YAML record.")
        else:
            search_joint = self.frozen_search_joint
        print(f"Returning to basket number {self.selected_marker_id + 1} search pose.")
        try:
            self._move_joint(search_joint)
        finally:
            self._close_servoj()
        returned_number = self.selected_marker_id + 1
        self.selected_marker_id = None
        self.frozen_base_marker = None
        self.frozen_search_joint = None
        self.frozen_search_tcp = None
        self.teach_stage = "idle"
        self.requested_marker_id = None
        print(
            f"Returned to basket number {returned_number} search/detection pose. "
            "Ready for the next basket: translate the arm, press 1..6, then B."
        )

    def run(self):
        self._wait_feedback()
        self._start_camera()
        print("Standalone U-fork basket teach mode.")
        print("No hand, gripper, MQTT, YOLO, or safety-proxy client is created.")
        print(
            "R returns the loaded basket through the taught reverse path; "
            "O is allowed only after the basket has been returned."
        )
        print(f"Output: {self.output_path}")
        print(
            "B select | F approach | I horizontal insert | L vertical lift | "
            "W loaded carry waypoint | T fixed temporary hold | "
            "O fixed observation | R return | type 1..192 then B choose | c clear | Q quit"
        )
        print(f"marker black-square length={self.arguments.marker_length_m * 1000.0:.1f} mm")
        window_name = "TL standalone U-fork basket teach"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            try:
                frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms=2000))
            except RuntimeError as error:
                self.get_logger().warning(f"D435 frame timeout: {error}")
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
            image = np.asanyarray(color_frame.get_data()).copy()
            camera_markers, centers, corners, ids = self._detect_markers(image, depth_frame)
            base_markers = self._base_marker_transforms(camera_markers)
            self.latest_base_markers = base_markers
            self.latest_marker_centers = centers
            # Only accumulate marker samples while the arm is truly static:
            # frames from the R-return motion or hand-dragging mix several
            # camera poses into one marker median and corrupt the taught
            # record (the same bug was fixed in the seed teach tool).
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
            cv2.imshow(window_name, self._draw_preview(image, corners, ids, base_markers))
            key = cv2.waitKey(1) & 0xFF
            try:
                if key in (ord("q"), 27):
                    break
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
                    self._select_marker()
                elif key == ord("f"):
                    self._record_pose("fork_approach")
                elif key == ord("i"):
                    self._record_pose("fork_insert")
                elif key == ord("l"):
                    self._record_pose("fork_lift")
                elif key == ord("t"):
                    self._record_transfer()
                elif key == ord("w"):
                    self._record_carry_waypoint()
                elif key == ord("o"):
                    self._record_observation()
                elif key == ord("r"):
                    self._return_to_search()
            except (RuntimeError, ValueError) as error:
                print(f"Command rejected/stopped: {error}")

    def stop(self):
        try:
            self._close_servoj()
        except Exception as error:
            print(f"ServoJ close warning: {error}")
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()


def main(argv=None):
    arguments = parse_args(argv)
    calibration_path, calibration, tool_camera = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    print(
        f"calibration={calibration_path}\n"
        f"accepted={bool(calibration.get('accepted', False))}, "
        f"convention={calibration.get('rpy_convention', 'intrinsic_xyz')}"
    )
    rclpy.init(args=[])
    node = ForkBasketTeachNode(arguments, calibration_path, calibration, tool_camera)
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
