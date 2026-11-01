import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

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
from tl_ros2_interface.srv import CoordTransform, OpenServoJ

from .reference_compat import (
    build_hover_pose,
    camera_point_to_base,
    minimum_jerk_joint_trajectory,
    nearest_equivalent_joint_angles,
    servoj_trajectory_point_count,
    summarize_validation_records,
    tool_pose_for_virtual_tcp,
    wrap_angle_rad,
    xy_probe_pose,
)


WORKSPACE_MIN_MM = np.array([-650.0, -650.0, 60.0], dtype=float)
WORKSPACE_MAX_MM = np.array([650.0, 650.0, 850.0], dtype=float)
REFERENCE_TEST_MIN_Z_MM = -150.0
JOINT_MIN_DEG = np.array([-170.0, -130.0, -150.0, -180.0, -210.0, -360.0])
JOINT_MAX_DEG = np.array([170.0, 130.0, 150.0, 180.0, 210.0, 360.0])
REACHABILITY_Z_LIFTS_MM = (0.0, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0)
REACHABILITY_XY_STEP_MM = 25.0
REACHABILITY_MAX_SHIFT_MM = 350.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="TL_TCB-compatible D435 no-gripper overhead test using direct ServoJ"
    )
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--hover-mm", type=float, default=100.0)
    parser.add_argument(
        "--observe-joint-deg",
        type=float,
        nargs=6,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="fixed observation joint pose (degrees); the arm moves here at startup "
        "and returns here on r before 9-point sampling",
    )
    parser.add_argument("--pointer-length-mm", type=float, default=0.0)
    parser.add_argument("--min-tip-clearance-mm", type=float, default=30.0)
    parser.add_argument("--mock-grasp-offset-mm", type=float, nargs=3, default=None)
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
        help="hard floor for the target tool0 Z in BASE frame mm; keep low so the "
        "auto-lift can raise near-table targets (the work surface sits around "
        "base Z=-120..-135 on this setup)",
    )
    parser.add_argument(
        "--final-grasp-z-mm",
        type=float,
        default=50.0,
        help="tool0 Z (BASE frame mm) reached by the one-key auto-descend (g)",
    )
    parser.add_argument(
        "--base-xy-bias-mm",
        type=float,
        nargs=2,
        default=[0.0, 0.0],
        metavar=("X", "Y"),
        help="temporary base-frame XY compensation added to the detected target",
    )
    parser.add_argument(
        "--base-z-bias-mm",
        type=float,
        default=0.0,
        help="temporary base-frame Z compensation added to the detected target",
    )
    parser.add_argument("--grasp-rpy-rad", type=float, nargs=3, default=None)
    parser.add_argument("--grasp-rpy-deg", type=float, nargs=3, default=None)
    parser.add_argument("--min-target-depth-m", type=float, default=0.15)
    parser.add_argument("--max-target-depth-m", type=float, default=0.65)
    parser.add_argument("--interp-points", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--servoj-speed-deg-s", type=float, default=20.0)
    parser.add_argument("--servoj-precision-speed-deg-s", type=float, default=8.0)
    parser.add_argument("--servoj-min-duration-sec", type=float, default=1.0)
    parser.add_argument("--arrival-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--max-joint-delta-deg", type=float, default=120.0)
    parser.add_argument(
        "--auto-lift-to-reachable",
        action="store_true",
        help="if IK fails, raise the target Z to the first same-XY reachable pose",
    )
    parser.add_argument("--max-auto-lift-mm", type=float, default=200.0)
    parser.add_argument("--fine-step-mm", type=float, default=1.0)
    parser.add_argument("--fine-z-step-mm", type=float, default=5.0)
    parser.add_argument("--fine-interp-points", type=int, default=100)
    parser.add_argument(
        "--validation-log",
        type=Path,
        default=Path("~/tl_handeye_validation/offsets.jsonl"),
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--allow-rejected", action="store_true")
    arguments = parser.parse_args(argv)
    if arguments.grasp_rpy_rad is not None and arguments.grasp_rpy_deg is not None:
        parser.error("use only one of --grasp-rpy-rad or --grasp-rpy-deg")
    if not np.isfinite(np.asarray(arguments.base_xy_bias_mm, dtype=float)).all():
        parser.error("--base-xy-bias-mm values must be finite")
    if arguments.grasp_rpy_deg is not None:
        arguments.grasp_rpy_rad = [
            float(value) for value in np.deg2rad(arguments.grasp_rpy_deg)
        ]
    mock_mode = arguments.mock_grasp_offset_mm is not None
    if mock_mode:
        if arguments.pointer_length_mm > 0.0:
            parser.error(
                "--pointer-length-mm is for legacy hover mode; use only "
                "--mock-grasp-offset-mm in mock hand mode"
            )
        if arguments.mock_grasp_clearance_mm < 0.0:
            parser.error("--mock-grasp-clearance-mm cannot be negative")
    elif arguments.hover_mm < 100.0:
        parser.error("--hover-mm must be at least 100 for a no-gripper test")
    if arguments.pointer_length_mm < 0.0:
        parser.error("--pointer-length-mm cannot be negative")
    if arguments.min_tip_clearance_mm < 0.0:
        parser.error("--min-tip-clearance-mm cannot be negative")
    required_hover = arguments.pointer_length_mm + arguments.min_tip_clearance_mm
    if (
        not mock_mode
        and arguments.pointer_length_mm > 0.0
        and arguments.hover_mm < required_hover
    ):
        parser.error(
            f"--hover-mm must be at least {required_hover:.1f} for a "
            f"{arguments.pointer_length_mm:.1f} mm pointer and "
            f"{arguments.min_tip_clearance_mm:.1f} mm tip clearance"
        )
    if not 0.0 < arguments.min_target_depth_m < arguments.max_target_depth_m <= 2.0:
        parser.error(
            "target depth range must satisfy 0 < --min-target-depth-m < "
            "--max-target-depth-m <= 2"
        )
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
    if arguments.arrival_tolerance_deg <= 0.0:
        parser.error("--arrival-tolerance-deg must be positive")
    if arguments.timeout_sec <= 0.0:
        parser.error("--timeout-sec must be positive")
    if arguments.max_joint_delta_deg <= 0.0:
        parser.error("--max-joint-delta-deg must be positive")
    if arguments.max_auto_lift_mm < 0.0:
        parser.error("--max-auto-lift-mm cannot be negative")
    if arguments.fine_step_mm <= 0.0 or arguments.fine_step_mm > 5.0:
        parser.error("--fine-step-mm must be in (0, 5]")
    if arguments.fine_z_step_mm <= 0.0 or arguments.fine_z_step_mm > 20.0:
        parser.error("--fine-z-step-mm must be in (0, 20]")
    if arguments.fine_interp_points < 50:
        parser.error("--fine-interp-points must be at least 50")
    if arguments.min_tool_z_mm < -200.0 or arguments.min_tool_z_mm > 300.0:
        parser.error("--min-tool-z-mm must be in [-200, 300]")
    if arguments.final_grasp_z_mm < -100.0 or arguments.final_grasp_z_mm > 300.0:
        parser.error("--final-grasp-z-mm must be in [-100, 300]")
    if (
        arguments.min_mock_clearance_mm is not None
        and arguments.min_mock_clearance_mm > arguments.mock_grasp_clearance_mm + 1e-6
    ):
        parser.error("--min-mock-clearance-mm must not exceed --mock-grasp-clearance-mm")
    return arguments


def load_calibration(path: Path, allow_rejected: bool):
    calibration_path = path.expanduser().resolve()
    calibration = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    accepted = bool(calibration.get("accepted", False))
    if not accepted and not allow_rejected:
        raise RuntimeError(
            "calibration is rejected; pass --allow-rejected only for a coarse, no-contact test"
        )
    convention = str(calibration.get("rpy_convention", "intrinsic_xyz"))
    if convention != "intrinsic_xyz":
        raise RuntimeError(
            f"TL_TCB compatibility requires intrinsic_xyz, calibration uses {convention}"
        )
    transform = np.asarray(calibration["T_tool0_camera"], dtype=float).reshape(4, 4)
    return calibration_path, calibration, transform


def calibration_intrinsics_from(calibration) -> dict | None:
    """Extract calibrated camera intrinsics from a calibration YAML, if present.

    Returns None when the calibration file carries no camera_matrix/distortion
    (older files), so callers fall back to the camera-reported intrinsics.
    """
    if not isinstance(calibration, dict):
        return None
    camera_matrix = calibration.get("camera_matrix")
    distortion = calibration.get("distortion")
    if camera_matrix is None or distortion is None:
        return None
    try:
        matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
        coeffs = np.asarray(distortion, dtype=float).reshape(-1)
    except (ValueError, TypeError):
        return None
    if matrix.shape != (3, 3) or coeffs.size < 5:
        return None
    return {
        "fx": float(matrix[0, 0]),
        "fy": float(matrix[1, 1]),
        "ppx": float(matrix[0, 2]),
        "ppy": float(matrix[1, 2]),
        "coeffs": [float(value) for value in coeffs[:5]],
    }


class ReferenceOverheadTest(Node):
    def __init__(self, arguments, calibration_path, calibration, tool_camera) -> None:
        super().__init__("tl_overhead_test")
        self.arguments = arguments
        self.calibration_path = calibration_path
        self.calibration = calibration
        self.tool_camera = tool_camera
        self.calibration_intrinsics = calibration_intrinsics_from(calibration)
        self.latest_tcp = None
        self.latest_tcp_at = 0.0
        self.latest_joint = None
        self.latest_joint_at = 0.0
        self.observation_joint = None
        self.clicked_pixel = None
        self.raw_base_point_mm = None
        self.base_point_mm = None
        self.target_pose = None
        self.target_joint = None
        self.nominal_target_pose = None
        self.desired_mock_grasp_base_mm = None
        self.camera_point_m = None
        self.depth_m = None
        self.target_reached = False
        self.validation_log = arguments.validation_log.expanduser().resolve()
        self.servoj_open = False
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.color_intrinsics = None
        self.servoj_publisher = self.create_publisher(
            Float64MultiArray, "/tl_driver/set_servoj_pos", 1
        )
        self.servoj_topic = "/tl_driver/set_servoj_pos"
        self.coord_client = self.create_client(
            CoordTransform, "/tl_driver/coord_transform"
        )
        self.open_servoj_client = self.create_client(
            OpenServoJ, "/tl_driver/open_servoj"
        )
        self.close_servoj_client = self.create_client(
            Trigger, "/tl_driver/close_servoj"
        )
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 20)
        self.create_subscription(CartesianPose, "/tcp_pose", self._tcp_callback, 20)

    def _joint_callback(self, message: JointState) -> None:
        self.latest_joint = [float(value) for value in message.position]
        self.latest_joint_at = time.monotonic()

    def _tcp_callback(self, message: CartesianPose) -> None:
        self.latest_tcp = [
            float(message.position.x),
            float(message.position.y),
            float(message.position.z),
            float(message.rpy.x),
            float(message.rpy.y),
            float(message.rpy.z),
        ]
        self.latest_tcp_at = time.monotonic()

    def wait_for_robot(self, timeout_sec: float = 8.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_tcp is not None and self.latest_joint is not None:
                self.observation_joint = self._pad_joint(self.latest_joint)
                return
        raise RuntimeError("missing /tcp_pose or /joint_states")

    def start_camera(self) -> None:
        last_error = None
        camera_serial = os.getenv("TL_CAMERA_SERIAL")
        for attempt in range(1, 4):
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            try:
                if camera_serial:
                    config.enable_device(camera_serial)
                profile = self.pipeline.start(config)
                self.align = rs.align(rs.stream.color)
                color_profile = profile.get_stream(
                    rs.stream.color
                ).as_video_stream_profile()
                self.color_intrinsics = color_profile.get_intrinsics()
                if self.calibration_intrinsics is not None:
                    calibrated = self.color_intrinsics
                    calibrated.ppx = self.calibration_intrinsics["ppx"]
                    calibrated.ppy = self.calibration_intrinsics["ppy"]
                    calibrated.fx = self.calibration_intrinsics["fx"]
                    calibrated.fy = self.calibration_intrinsics["fy"]
                    calibrated.model = rs.distortion.brown_conrady
                    calibrated.coeffs = self.calibration_intrinsics["coeffs"] + [0.0] * (
                        5 - len(self.calibration_intrinsics["coeffs"])
                    )
                    self.color_intrinsics = calibrated
                    self.get_logger().info(
                        f"using calibrated intrinsics from {self.calibration_path}: "
                        f"fx={calibrated.fx:.2f} fy={calibrated.fy:.2f} "
                        f"ppx={calibrated.ppx:.2f} ppy={calibrated.ppy:.2f} "
                        f"coeffs={[round(value, 4) for value in calibrated.coeffs]}"
                    )
                ready_frames = 0
                ready_deadline = time.monotonic() + 12.0
                while ready_frames < 3 and time.monotonic() < ready_deadline:
                    try:
                        frames = self.pipeline.wait_for_frames(timeout_ms=2000)
                    except RuntimeError:
                        continue
                    if frames.get_color_frame() and frames.get_depth_frame():
                        ready_frames += 1
                if ready_frames > 0:
                    if ready_frames < 3:
                        self.get_logger().warning(
                            f"D435 started with only {ready_frames} warm-up frame(s)"
                        )
                    return
                raise RuntimeError("D435 started but delivered no valid frames in 12 seconds")
            except RuntimeError as error:
                last_error = error
                visible_devices = []
                try:
                    for device in rs.context().query_devices():
                        try:
                            name = device.get_info(rs.camera_info.name)
                            serial = device.get_info(rs.camera_info.serial_number)
                        except RuntimeError:
                            continue
                        visible_devices.append(f"{name}[{serial}]")
                except RuntimeError:
                    pass
                try:
                    self.pipeline.stop()
                except RuntimeError:
                    pass
                if attempt < 3:
                    if visible_devices:
                        self.get_logger().warning(
                            f"D435 start failed on attempt {attempt}; visible devices: "
                            f"{visible_devices}; retrying"
                        )
                    else:
                        self.get_logger().warning(
                            f"D435 start failed on attempt {attempt}; no RealSense device "
                            f"visible yet; retrying"
                        )
                    time.sleep(2.0)
        raise RuntimeError(
            "D435 failed to start after three attempts; close other camera processes, "
            "check USB3 power/cable, or set TL_CAMERA_SERIAL if multiple cameras exist"
        ) from last_error

    def _wait_for_camera_device(self, timeout_sec: float) -> str:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            devices = rs.context().query_devices()
            for device in devices:
                try:
                    name = device.get_info(rs.camera_info.name)
                    serial = device.get_info(rs.camera_info.serial_number)
                except RuntimeError:
                    continue
                if "D435" in name:
                    return serial
            time.sleep(0.5)
        raise RuntimeError(
            "D435 did not appear in librealsense within 10 seconds; "
            "check for another camera process or reconnect USB3"
        )

    def stop(self) -> None:
        try:
            self.close_servoj(quiet=True)
        except Exception as error:
            if rclpy.ok():
                self.get_logger().warning(f"CloseServoJ failed during cleanup: {error}")
        try:
            self.pipeline.stop()
        except RuntimeError:
            pass
        cv2.destroyAllWindows()

    def _call(self, client, request, timeout_sec: float):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError("ROS service call timed out")
        return future.result()

    @staticmethod
    def _pad_joint(values) -> list[float]:
        result = [float(value) for value in values[:7]]
        return (result + [0.0] * 7)[:7]

    def _target_rpy(self) -> list[float]:
        if self.arguments.grasp_rpy_rad is not None:
            return [
                wrap_angle_rad(value) for value in self.arguments.grasp_rpy_rad
            ]
        if self.latest_tcp is None:
            raise RuntimeError("/tcp_pose is missing")
        return [wrap_angle_rad(value) for value in self.latest_tcp[3:6]]

    @staticmethod
    def _median_depth(depth_frame, pixel_x: int, pixel_y: int) -> float:
        values = []
        for offset_y in range(-5, 6):
            for offset_x in range(-5, 6):
                x = min(max(pixel_x + offset_x, 0), 639)
                y = min(max(pixel_y + offset_y, 0), 479)
                distance = float(depth_frame.get_distance(x, y))
                if 0.05 < distance < 2.0:
                    values.append(distance)
        if len(values) < 12:
            raise RuntimeError("selected point has insufficient valid depth samples")
        return float(np.median(np.asarray(values, dtype=float)))

    def calculate_target(self, depth_frame) -> None:
        if self.clicked_pixel is None:
            raise RuntimeError("click the object center first")
        if self.latest_tcp is None or time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError("/tcp_pose is stale")
        pixel_x, pixel_y = self.clicked_pixel
        depth_m = self._median_depth(depth_frame, pixel_x, pixel_y)
        if not self.arguments.min_target_depth_m <= depth_m <= self.arguments.max_target_depth_m:
            raise RuntimeError(
                f"selected depth {depth_m:.3f} m is outside expected target range "
                f"{self.arguments.min_target_depth_m:.3f}.."
                f"{self.arguments.max_target_depth_m:.3f} m; click the printed target, "
                "not the background"
            )
        camera_point = rs.rs2_deproject_pixel_to_point(
            self.color_intrinsics, [float(pixel_x), float(pixel_y)], depth_m
        )
        camera_point_m = [float(value) for value in camera_point]
        raw_base_point_mm = np.asarray(
            camera_point_to_base(
                self.latest_tcp,
                self.tool_camera,
                camera_point,
                "intrinsic_xyz",
            ),
            dtype=float,
        ).copy()
        base_point_mm = raw_base_point_mm.copy()
        base_xy_bias = np.asarray(self.arguments.base_xy_bias_mm, dtype=float)
        base_point_mm[:2] += base_xy_bias
        base_point_mm[2] += float(self.arguments.base_z_bias_mm)
        desired_mock_grasp = None
        target_rpy = self._target_rpy()
        if self.arguments.mock_grasp_offset_mm is None:
            target_pose = build_hover_pose(
                base_point_mm, self.latest_tcp, self.arguments.hover_mm
            )
            target_pose[3:] = target_rpy
        else:
            desired_mock_grasp = np.asarray(base_point_mm, dtype=float).copy()
            desired_mock_grasp[2] += float(self.arguments.mock_grasp_clearance_mm)
            target_pose = tool_pose_for_virtual_tcp(
                desired_mock_grasp,
                self.latest_tcp,
                self.arguments.mock_grasp_offset_mm,
                "intrinsic_xyz",
                target_rpy,
            )
        self.target_reached = False
        self.target_pose = None
        self.target_joint = None
        self.nominal_target_pose = None
        self.desired_mock_grasp_base_mm = None
        self.camera_point_m = camera_point_m
        self.depth_m = float(depth_m)
        self.raw_base_point_mm = np.asarray(raw_base_point_mm, dtype=float).tolist()
        self.base_point_mm = base_point_mm.tolist()
        print(
            "camera[m]="
            f"{np.round(camera_point, 4).tolist()}, "
            f"base[mm]={np.round(raw_base_point_mm, 2).tolist()}"
        )
        if np.linalg.norm(base_xy_bias) > 1.0e-9:
            print(
                "base XY bias[mm]="
                f"{np.round(base_xy_bias, 2).tolist()}, "
                f"compensated base[mm]={np.round(base_point_mm, 2).tolist()}"
            )
        if desired_mock_grasp is None:
            print(f"hover TCP={np.round(target_pose, 3).tolist()}")
        else:
            print(
                "mock grasp base[mm]="
                f"{np.round(desired_mock_grasp, 2).tolist()}, "
                "offset tool0[mm]="
                f"{np.round(self.arguments.mock_grasp_offset_mm, 2).tolist()}"
            )
            print(f"tool0 target TCP={np.round(target_pose, 3).tolist()}")
        if self.arguments.grasp_rpy_rad is not None:
            print(f"grasp RPY override={np.round(target_rpy, 4).tolist()}")
        self._validate_cartesian(target_pose, base_point_mm, desired_mock_grasp)
        self.nominal_grasp_z_mm = float(target_pose[2])
        target_pose, desired_mock_grasp, target_joint = self._solve_target_with_optional_lift(
            target_pose, desired_mock_grasp
        )
        self._validate_cartesian(target_pose, base_point_mm, desired_mock_grasp)
        self._validate_joint(target_joint)
        self.target_pose = list(target_pose)
        self.target_joint = list(target_joint)
        self.nominal_target_pose = list(target_pose)
        if desired_mock_grasp is not None:
            self.desired_mock_grasp_base_mm = desired_mock_grasp.tolist()
        print(f"IK joints[deg]={np.round(target_joint[:6], 3).tolist()}")
        print("Press m to execute after checking all three lines.")

    def _validate_cartesian(
        self, target_pose, base_point_mm, desired_mock_grasp=None
    ) -> None:
        position = np.asarray(target_pose[:3], dtype=float)
        if not np.isfinite(position).all():
            raise RuntimeError("target Cartesian position is not finite")
        if np.any(position[:2] < WORKSPACE_MIN_MM[:2]) or np.any(
            position[:2] > WORKSPACE_MAX_MM[:2]
        ):
            raise RuntimeError(
                f"target XY {np.round(position[:2], 2).tolist()} is outside "
                f"{WORKSPACE_MIN_MM[:2].tolist()}..{WORKSPACE_MAX_MM[:2].tolist()} mm"
            )
        if position[2] < float(self.arguments.min_tool_z_mm) or position[2] > WORKSPACE_MAX_MM[2]:
            raise RuntimeError(
                f"target Z={position[2]:.2f} mm is outside the test range "
                f"{self.arguments.min_tool_z_mm}..{WORKSPACE_MAX_MM[2]} mm"
            )
        if desired_mock_grasp is None:
            measured_clearance = float(position[2] - float(base_point_mm[2]))
            minimum_clearance = 100.0
        else:
            virtual_position = np.asarray(desired_mock_grasp, dtype=float).reshape(3)
            if not np.isfinite(virtual_position).all():
                raise RuntimeError("mock grasp target is not finite")
            measured_clearance = float(virtual_position[2] - float(base_point_mm[2]))
            minimum_clearance = float(
                self.arguments.min_mock_clearance_mm
                if self.arguments.min_mock_clearance_mm is not None
                else self.arguments.mock_grasp_clearance_mm
            )
        if measured_clearance < minimum_clearance - 1e-6:
            print(
                f"WARNING: calculated clearance {measured_clearance:.2f} mm is below "
                f"the configured minimum {minimum_clearance:.2f} mm; proceeding, "
                "the grasp may contact the object (disable by raising the object "
                "or lowering the configured clearance)"
            )

    def _request_ik(self, target_pose):
        if not self.coord_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/tl_driver/coord_transform is unavailable")
        request = CoordTransform.Request()
        request.origin_coord = 1
        request.target_coord = 0
        request.form = 0
        request.origin_pos = [float(value) for value in target_pose] + [0.0]
        request.reference_pos = (
            self._pad_joint(self.latest_joint)
            if self.latest_joint is not None
            else [0.0] * 7
        )
        return self._call(self.coord_client, request, 5.0)

    def _normalize_ik_joint(self, raw_joint) -> list[float]:
        raw_joint = self._pad_joint(raw_joint)
        if self.latest_joint is None:
            return raw_joint
        normalized_joint = list(raw_joint)
        normalized_joint[:6] = nearest_equivalent_joint_angles(
            raw_joint[:6],
            self.latest_joint[:6],
            JOINT_MIN_DEG,
            JOINT_MAX_DEG,
        )
        if not np.allclose(raw_joint[:6], normalized_joint[:6], atol=1e-6):
            print(
                "IK angle wrap: "
                f"raw={np.round(raw_joint[:6], 3).tolist()} -> "
                f"nearest={np.round(normalized_joint[:6], 3).tolist()}"
            )
        return normalized_joint

    def _solve_target_with_optional_lift(self, target_pose, desired_mock_grasp):
        response = self._request_ik(target_pose)
        if response.success:
            return list(target_pose), desired_mock_grasp, self._normalize_ik_joint(
                response.target_pos
            )
        if not self.arguments.auto_lift_to_reachable:
            return list(target_pose), desired_mock_grasp, self._solve_ik(target_pose)

        for lift_mm in REACHABILITY_Z_LIFTS_MM[1:]:
            if lift_mm > self.arguments.max_auto_lift_mm:
                continue
            probe_pose = list(target_pose)
            probe_pose[2] += float(lift_mm)
            probe_response = self._request_ik(probe_pose)
            if not probe_response.success:
                continue
            lifted_mock_grasp = None
            if desired_mock_grasp is not None:
                lifted_mock_grasp = np.asarray(desired_mock_grasp, dtype=float).copy()
                lifted_mock_grasp[2] += float(lift_mm)
            print(
                "Auto-lifted target to reachable same-XY pose: "
                f"+{lift_mm:.1f} mm, tool0 Z={probe_pose[2]:.2f} mm. "
                "This keeps the fake hand farther from the object."
            )
            if lifted_mock_grasp is not None:
                print(
                    "auto-lifted mock grasp base[mm]="
                    f"{np.round(lifted_mock_grasp, 2).tolist()}"
                )
            print(f"auto-lifted tool0 target TCP={np.round(probe_pose, 3).tolist()}")
            return (
                probe_pose,
                lifted_mock_grasp,
                self._normalize_ik_joint(probe_response.target_pos),
            )
        return list(target_pose), desired_mock_grasp, self._solve_ik(target_pose)

    def _solve_ik(self, target_pose) -> list[float]:
        response = self._request_ik(target_pose)
        if not response.success:
            target_text = np.round(target_pose, 3).tolist()
            current_ok = False
            if self.latest_tcp is not None:
                current_response = self._request_ik(self.latest_tcp)
                current_ok = bool(current_response.success)

            suggestion = ""
            if current_ok:
                for lift_mm in REACHABILITY_Z_LIFTS_MM[1:]:
                    probe_pose = list(target_pose)
                    probe_pose[2] += lift_mm
                    probe_response = self._request_ik(probe_pose)
                    if probe_response.success:
                        suggested_hover = self.arguments.hover_mm + lift_mm
                        suggestion = (
                            f" First reachable same-XY probe: Z={probe_pose[2]:.2f} mm; "
                            f"retry with --hover-mm {suggested_hover:.0f}."
                        )
                        break
                if not suggestion and self.latest_tcp is not None:
                    reachability = self._find_nearest_reachable_probe(target_pose)
                    if reachability is not None:
                        probe_pose, shift_xy, xy_shift_mm, z_lift_mm = reachability
                        suggestion = (
                            " Nearest reachable probe found without moving: "
                            f"XY shift from requested target="
                            f"{np.round(shift_xy, 1).tolist()} mm "
                            f"(distance {xy_shift_mm:.1f} mm), "
                            f"extra Z lift={z_lift_mm:.1f} mm, "
                            f"probe TCP={np.round(probe_pose, 2).tolist()}. "
                            "Move the paper/object by this base-frame XY correction "
                            "or use a closer observation pose, then recalculate."
                        )
                raise RuntimeError(
                    f"CoordTransform has no IK for target {target_text}; "
                    "current TCP IK self-check passed, so the requested pose is "
                    f"outside this IK branch/workspace.{suggestion} No motion was sent."
                )

            raise RuntimeError(
                f"CoordTransform failed for target {target_text}, and the current TCP "
                "also fails IK self-check. Restart the TL driver and verify the robot "
                "connection/current coordinate configuration. No motion was sent."
            )
        return self._normalize_ik_joint(response.target_pos)

    def _find_nearest_reachable_probe(self, target_pose):
        if self.latest_tcp is None:
            return None
        current = np.asarray(self.latest_tcp, dtype=float)
        target = np.asarray(target_pose, dtype=float)
        xy_distance = float(np.linalg.norm(current[:2] - target[:2]))
        if xy_distance <= 1e-9:
            return None
        max_shift = min(REACHABILITY_MAX_SHIFT_MM, xy_distance)
        shift_values = np.arange(
            REACHABILITY_XY_STEP_MM,
            max_shift + REACHABILITY_XY_STEP_MM * 0.5,
            REACHABILITY_XY_STEP_MM,
        )
        for shift_mm in shift_values:
            for z_lift_mm in REACHABILITY_Z_LIFTS_MM:
                probe_pose, shift_xy = xy_probe_pose(
                    target_pose,
                    self.latest_tcp,
                    float(shift_mm),
                    float(z_lift_mm),
                )
                response = self._request_ik(probe_pose)
                if response.success:
                    actual_shift_mm = float(np.linalg.norm(shift_xy))
                    return probe_pose, shift_xy, actual_shift_mm, float(z_lift_mm)
        return None

    def _validate_joint(self, target_joint) -> None:
        if self.latest_joint is None:
            raise RuntimeError("/joint_states is missing")
        # After a ServoJ stream the driver may pause /joint_states publishing
        # briefly; wait up to 2s before declaring the topic stale.
        refresh_deadline = time.monotonic() + 2.0
        while (
            rclpy.ok()
            and time.monotonic() - self.latest_joint_at > 0.5
            and time.monotonic() < refresh_deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.02)
        if time.monotonic() - self.latest_joint_at > 0.5:
            raise RuntimeError("/joint_states is stale")
        target = np.asarray(target_joint[:6], dtype=float)
        current = np.asarray(self.latest_joint[:6], dtype=float)
        if not np.isfinite(target).all():
            raise RuntimeError("IK returned non-finite joints")
        if np.any(target < JOINT_MIN_DEG) or np.any(target > JOINT_MAX_DEG):
            raise RuntimeError(f"IK joints outside configured limits: {target.tolist()}")
        maximum_delta = float(np.max(np.abs(target - current)))
        if maximum_delta > self.arguments.max_joint_delta_deg:
            raise RuntimeError(
                f"IK joint jump {maximum_delta:.2f} deg exceeds "
                f"{self.arguments.max_joint_delta_deg:.2f} deg"
            )

    def _open_servoj(self) -> None:
        if self.servoj_open:
            return
        self._assert_servoj_channel_is_exclusive()
        if not self.open_servoj_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/tl_driver/open_servoj is unavailable")
        request = OpenServoJ.Request()
        request.vmax = [float(self.arguments.servoj_speed_deg_s)] * 7
        request.amax = [3000.0] * 7
        request.jmax = [50000.0] * 7
        response = self._call(self.open_servoj_client, request, 5.0)
        if not response.success:
            raise RuntimeError(f"OpenServoJ failed: {response.message}")
        self.servoj_open = True
        print(
            f"ServoJ opened: vmax={self.arguments.servoj_speed_deg_s:.1f}, "
            "amax=3000, jmax=50000"
        )

    def _assert_servoj_channel_is_exclusive(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.1)
        publishers = self.get_publishers_info_by_topic(self.servoj_topic)
        if len(publishers) <= 1:
            return
        owners = sorted(
            f"{publisher.node_namespace}{publisher.node_name}"
            for publisher in publishers
        )
        raise RuntimeError(
            f"ServoJ channel has {len(publishers)} publishers {owners}; "
            "stop safety_only.launch.py and all other teach/test programs before moving"
        )

    def close_servoj(self, quiet: bool = False) -> None:
        if not self.servoj_open:
            return
        if not rclpy.ok():
            self.servoj_open = False
            return
        if not self.close_servoj_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("/tl_driver/close_servoj is unavailable")
        response = self._call(self.close_servoj_client, Trigger.Request(), 3.0)
        if hasattr(response, "success") and not response.success:
            raise RuntimeError(f"CloseServoJ failed: {response.message}")
        self.servoj_open = False
        if not quiet:
            print("ServoJ closed. Manual drag control is available again.")

    def _publish_joint(self, target) -> None:
        message = Float64MultiArray()
        message.data = [float(value) for value in target]
        self.servoj_publisher.publish(message)

    def move_joint(self, target_joint, point_count=None, speed_deg_s=None) -> None:
        if not self.arguments.execute:
            print("DRY RUN: no command sent. Add --execute for physical motion.")
            return
        if self.servoj_publisher.get_subscription_count() == 0:
            raise RuntimeError(
                "/tl_driver/set_servoj_pos has no subscriber; restart the TL driver"
            )
        self._validate_joint(target_joint)
        self._open_servoj()
        start_joint = self._pad_joint(self.latest_joint)
        selected_speed = float(
            self.arguments.servoj_speed_deg_s
            if speed_deg_s is None
            else speed_deg_s
        )
        maximum_delta = float(
            np.max(
                np.abs(
                    np.asarray(target_joint[:6], dtype=float)
                    - np.asarray(start_joint[:6], dtype=float)
                )
            )
        )
        if point_count is not None:
            trajectory_points = int(point_count)
            duration_sec = trajectory_points / float(self.arguments.rate_hz)
        elif int(self.arguments.interp_points) > 0:
            trajectory_points = int(self.arguments.interp_points)
            duration_sec = trajectory_points / float(self.arguments.rate_hz)
        else:
            trajectory_points, duration_sec = servoj_trajectory_point_count(
                maximum_delta,
                self.arguments.rate_hz,
                selected_speed,
                self.arguments.servoj_min_duration_sec,
            )
        trajectory = minimum_jerk_joint_trajectory(
            start_joint, target_joint, trajectory_points
        )
        period = 1.0 / self.arguments.rate_hz
        print(
            f"Streaming {len(trajectory)} minimum-jerk ServoJ points at "
            f"{self.arguments.rate_hz:.1f} Hz, duration={duration_sec:.2f}s, "
            f"max joint delta={maximum_delta:.2f} deg, speed limit={selected_speed:.1f} deg/s..."
        )
        for waypoint in trajectory:
            started = time.monotonic()
            self._publish_joint(waypoint)
            rclpy.spin_once(self, timeout_sec=0.0)
            remaining = period - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
        deadline = time.monotonic() + self.arguments.timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_joint(target_joint)
            rclpy.spin_once(self, timeout_sec=min(period, 0.02))
            if self.latest_joint is None:
                continue
            error = float(
                np.max(
                    np.abs(
                        np.asarray(self.latest_joint[:6], dtype=float)
                        - np.asarray(target_joint[:6], dtype=float)
                    )
                )
            )
            if error <= self.arguments.arrival_tolerance_deg:
                print(f"Arrived: max joint error={error:.3f} deg")
                self.close_servoj(quiet=True)
                return
            time.sleep(max(0.0, period - 0.02))
        raise RuntimeError(
            "ServoJ arrival timeout; current="
            f"{None if self.latest_joint is None else np.round(self.latest_joint[:6], 3).tolist()}"
        )

    def fine_adjust(
        self, delta_x_mm: float, delta_y_mm: float, delta_z_mm: float = 0.0
    ) -> None:
        if not self.target_reached or self.target_pose is None:
            raise RuntimeError("move to the nominal target with m before fine adjustment")
        candidate_pose = list(self.target_pose)
        candidate_pose[0] += float(delta_x_mm)
        candidate_pose[1] += float(delta_y_mm)
        candidate_pose[2] += float(delta_z_mm)
        candidate_mock_grasp = None
        if self.desired_mock_grasp_base_mm is not None:
            candidate_mock_grasp = list(self.desired_mock_grasp_base_mm)
            candidate_mock_grasp[0] += float(delta_x_mm)
            candidate_mock_grasp[1] += float(delta_y_mm)
            candidate_mock_grasp[2] += float(delta_z_mm)
        self._validate_cartesian(
            candidate_pose, self.base_point_mm, candidate_mock_grasp
        )
        candidate_joint = self._solve_ik(candidate_pose)
        self._validate_joint(candidate_joint)
        print(
            f"Fine adjust dX={delta_x_mm:+.1f}, dY={delta_y_mm:+.1f}, "
            f"dZ={delta_z_mm:+.1f} mm -> "
            f"XYZ={np.round(candidate_pose[:3], 2).tolist()}"
        )
        self.move_joint(candidate_joint, self.arguments.fine_interp_points)
        self.target_pose = candidate_pose
        self.target_joint = candidate_joint
        self._log_grasp_z(candidate_pose)

    def _log_grasp_z(self, pose) -> None:
        """Append the tuned tool0 pose to the Z record log; the last entry is the
        real object height reached by manual adjustment."""
        record = {
            "utc": datetime.now(timezone.utc).isoformat(),
            "tool0_xyz_mm": [round(float(value), 2) for value in pose[:3]],
            "tool0_rpy_rad": [round(float(value), 4) for value in pose[3:]],
        }
        log_path = Path("~/tl_handeye_validation/grasp_z_record.jsonl").expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with open(log_path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        print(f"grasp Z recorded: Z={record['tool0_xyz_mm'][2]:.2f} mm -> {log_path}")

    def auto_descent(
        self,
        step_mm: float = 10.0,
        max_steps: int = 30,
        floor_z: float | None = None,
        stop_offset_mm: float = 50.0,
    ) -> None:
        """After arriving at a reachable pose, descend step by step (IK evaluated
        with the current joint reference) to a safe height above the object
        (floor_z + stop_offset_mm); the operator then fine-adjusts the last part.
        Never goes below floor_z so the fake hand cannot press into the object."""
        if not self.target_reached or self.target_pose is None:
            raise RuntimeError("move to the target with m before descending")
        stop_z = None
        if floor_z is not None:
            stop_z = float(floor_z) + float(stop_offset_mm)
        start_z = float(self.target_pose[2])
        pose = list(self.target_pose)
        for _ in range(max_steps):
            candidate = list(pose)
            candidate[2] -= float(step_mm)
            if stop_z is not None and candidate[2] < stop_z - 1e-6:
                candidate[2] = stop_z
            response = self._request_ik(candidate)
            if not response.success:
                break
            pose = candidate
            if stop_z is not None and candidate[2] <= stop_z + 1e-6:
                break
        descended = start_z - float(pose[2])
        if descended <= 0.01:
            print("Auto-descent: already at the safe height.")
            return
        if stop_z is not None:
            print(
                f"Auto-descent {descended:.0f} mm to safe height tool0 Z={pose[2]:.1f} mm "
                f"(object floor {floor_z:.1f} mm + {stop_offset_mm:.0f} mm clearance). "
                "Fine-adjust with j/u to touch the object."
            )
        else:
            print(f"Auto-descent {descended:.0f} mm to tool0 Z={pose[2]:.1f} mm.")
        if self.desired_mock_grasp_base_mm is not None:
            grasp = list(self.desired_mock_grasp_base_mm)
            grasp[2] -= descended
            self.desired_mock_grasp_base_mm = grasp
        self.target_pose = pose
        self.target_joint = self._solve_ik(pose)
        self._validate_joint(self.target_joint)
        self.move_joint(self.target_joint, self.arguments.fine_interp_points)

    def safe_return(self) -> None:
        """Lift the hand above the object first, then return to the observation
        pose, so the hand cannot collide with the object on the way back."""
        if self.target_reached and self.target_pose is not None:
            lift_pose = list(self.target_pose)
            lift_pose[2] += float(self.arguments.hover_mm)
            response = self._request_ik(lift_pose)
            if response.success:
                lift_joint = self._normalize_ik_joint(response.target_pos)
                self._validate_joint(lift_joint)
                print(
                    "Lifting to safe height before return: "
                    f"tool0 Z={lift_pose[2]:.1f} mm"
                )
                self.move_joint(lift_joint)
        print("Returning to recorded observation joints.")
        self.move_joint(self.observation_joint)
        self.target_reached = False

    def auto_descend_to_grasp_z(self) -> None:
        """One-key smooth descent to the configured final grasp Z (base mm)."""
        if not self.target_reached or self.target_pose is None:
            raise RuntimeError("move to the nominal target with m before descending")
        target = list(self.target_pose)
        final_z = float(self.arguments.final_grasp_z_mm)
        delta_z = final_z - float(target[2])
        if abs(delta_z) < 1e-6:
            print(f"Already at final grasp Z={final_z:.1f} mm")
            return
        target[2] = final_z
        candidate_mock_grasp = None
        if self.desired_mock_grasp_base_mm is not None:
            candidate_mock_grasp = list(self.desired_mock_grasp_base_mm)
            candidate_mock_grasp[2] += delta_z
        self._validate_cartesian(target, self.base_point_mm, candidate_mock_grasp)
        candidate_joint = self._solve_ik(target)
        self._validate_joint(candidate_joint)
        print(
            f"Auto-descend to final grasp Z={final_z:.1f} mm (dZ={delta_z:+.1f}) "
            f"-> XYZ={np.round(target[:3], 2).tolist()}"
        )
        self.move_joint(candidate_joint)
        self.target_pose = target
        self.target_joint = candidate_joint
        if candidate_mock_grasp is not None:
            self.desired_mock_grasp_base_mm = candidate_mock_grasp

    def save_validation_sample(self) -> None:
        if not self.target_reached or self.nominal_target_pose is None:
            raise RuntimeError("move and align the pointer before saving")
        if self.latest_tcp is None or time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError("/tcp_pose is stale")
        actual_tcp = np.asarray(self.latest_tcp, dtype=float)
        nominal_tcp = np.asarray(self.nominal_target_pose, dtype=float)
        if self.desired_mock_grasp_base_mm is not None:
            # Mock-hand mode: lowering Z toward the grasp height is intentional;
            # compare the actual TCP against the current (adjusted) target instead
            # of the original hover pose so the arrival check stays meaningful.
            reference_tcp = np.asarray(self.target_pose, dtype=float)
        else:
            reference_tcp = nominal_tcp
        z_drift = float(actual_tcp[2] - reference_tcp[2])
        orientation_delta = np.array(
            [
                (actual_tcp[index] - reference_tcp[index] + np.pi) % (2.0 * np.pi)
                - np.pi
                for index in range(3, 6)
            ],
            dtype=float,
        )
        orientation_error = float(np.linalg.norm(orientation_delta))
        if abs(z_drift) > 10.0:
            raise RuntimeError(
                f"TCP Z changed by {z_drift:.2f} mm; keep the validation height unchanged"
            )
        if orientation_error > 0.08:
            raise RuntimeError(
                f"TCP orientation changed by {orientation_error:.3f} rad; keep the original orientation"
            )
        correction = actual_tcp[:3] - nominal_tcp[:3]
        records = self._load_validation_records()
        record = {
            "sample": len(records) + 1,
            "timestamp_ns": time.time_ns(),
            "calibration": str(self.calibration_path),
            "pixel": list(self.clicked_pixel),
            "depth_m": self.depth_m,
            "camera_point_m": self.camera_point_m,
            "raw_estimated_object_base_mm": self.raw_base_point_mm,
            "base_xy_bias_mm": [
                float(value) for value in self.arguments.base_xy_bias_mm
            ],
            "base_z_bias_mm": float(self.arguments.base_z_bias_mm),
            "estimated_object_base_mm": np.asarray(
                self.base_point_mm, dtype=float
            ).tolist(),
            "nominal_hover_tcp_mm_rad": nominal_tcp.tolist(),
            "aligned_command_tcp_mm_rad": list(self.target_pose),
            "aligned_actual_tcp_mm_rad": actual_tcp.tolist(),
            "correction_base_mm": correction.tolist(),
            "orientation_change_rad": orientation_error,
        }
        if self.arguments.mock_grasp_offset_mm is not None:
            record["mock_grasp_offset_tool0_mm"] = [
                float(value) for value in self.arguments.mock_grasp_offset_mm
            ]
            record["desired_mock_grasp_base_mm"] = self.desired_mock_grasp_base_mm
        self.validation_log.parent.mkdir(parents=True, exist_ok=True)
        with self.validation_log.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        records.append(record)
        print(
            "Saved validation sample "
            f"{record['sample']}: correction XYZ="
            f"{np.round(correction, 2).tolist()} mm"
        )
        self._write_validation_summary(records)

    def _load_validation_records(self):
        if not self.validation_log.exists():
            return []
        records = []
        for line in self.validation_log.read_text(encoding="utf-8").splitlines():
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if record.get("calibration") == str(self.calibration_path):
                records.append(record)
        return records

    def _write_validation_summary(self, records) -> None:
        summary = summarize_validation_records(records)
        summary_path = self.validation_log.with_suffix(".summary.json")
        summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(
            f"Summary n={len(records)}: mean XY="
            f"{np.round(summary['mean_correction_base_mm'][:2], 2).tolist()} mm, "
            f"Z={summary['fixed_bias_z_mm']:.1f} mm, "
            f"after-bias XY RMS={summary['after_fixed_bias_xy_rms_mm']:.2f} mm, "
            f"Z RMS={summary['after_fixed_bias_z_rms_mm']:.2f} mm"
        )
        print(f"judgment: {summary['judgment']}")
        print(f"validation log: {self.validation_log}")
        print(f"summary: {summary_path}")

    def _mouse_callback(self, event, x, y, _flags, _parameter) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_pixel = (int(x), int(y))
            self.base_point_mm = None
            self.target_pose = None
            self.target_joint = None
            self.nominal_target_pose = None
            self.desired_mock_grasp_base_mm = None
            self.camera_point_m = None
            self.depth_m = None
            self.target_reached = False
            print(f"selected pixel=({x}, {y})")

    def run(self) -> None:
        self.wait_for_robot()
        if self.arguments.observe_joint_deg is not None:
            target = self._pad_joint(list(self.arguments.observe_joint_deg))
            print(
                "Moving to fixed observation pose "
                f"{np.round(target[:6], 3).tolist()} deg..."
            )
            self.move_joint(target)
            self.observation_joint = target
            print(f"Observation pose set to {np.round(target[:6], 3).tolist()} deg")
        self.start_camera()
        print(f"calibration={self.calibration_path}")
        print(
            f"accepted={bool(self.calibration.get('accepted', False))}, "
            f"method={self.calibration.get('method')}, "
            f"convention={self.calibration.get('rpy_convention')}"
        )
        print(f"observation joints={np.round(self.observation_joint[:6], 3).tolist()}")
        print(f"observation TCP={np.round(self.latest_tcp, 3).tolist()}")
        if self.arguments.pointer_length_mm > 0.0:
            tip_clearance = (
                self.arguments.hover_mm - self.arguments.pointer_length_mm
            )
            print(
                f"pointer={self.arguments.pointer_length_mm:.1f} mm, "
                f"nominal tip clearance={tip_clearance:.1f} mm"
            )
        if self.arguments.mock_grasp_offset_mm is not None:
            print(
                "mock grasp TCP mode: "
                f"offset tool0[mm]={np.round(self.arguments.mock_grasp_offset_mm, 2).tolist()}, "
                f"clearance={self.arguments.mock_grasp_clearance_mm:.1f} mm"
            )
        if self.arguments.grasp_rpy_rad is not None:
            print(
                "fixed grasp RPY[rad]="
                f"{np.round(self.arguments.grasp_rpy_rad, 4).tolist()}"
            )
        if np.max(np.abs(np.asarray(self.observation_joint[:6], dtype=float))) < 1.0:
            print(
                "WARNING: all observation joints are near zero. Confirm this is the "
                "intended camera pose before calculating or moving."
            )
        print(
            "click | c calculate+IK | m move | "
            "a/d X-/X+ | s/w Y-/Y+ | j/u Z-/Z+ | g descend | v save | r return | q quit"
        )
        print(f"validation log={self.validation_log}")
        window_name = "TL no-gripper direct reference test"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window_name, self._mouse_callback)
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
            if self.clicked_pixel is not None:
                cv2.drawMarker(
                    preview,
                    self.clicked_pixel,
                    (0, 255, 255),
                    cv2.MARKER_CROSS,
                    24,
                    2,
                )
            mode = "EXECUTE" if self.arguments.execute else "DRY RUN"
            cv2.putText(
                preview,
                f"{mode} DIRECT | click c m | A/D:X S/W:Y ({self.arguments.fine_step_mm:g}mm) J/U:Z ({self.arguments.fine_z_step_mm:g}mm)",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                preview,
                "V:save sample  R:return  Q:quit",
                (8, 46),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                2,
            )
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            try:
                if key == ord("c"):
                    self.calculate_target(depth_frame)
                elif key == ord("m"):
                    if self.target_joint is None:
                        print("Press c before m.")
                    else:
                        print("Moving through direct TL_TCB-compatible ServoJ.")
                        self.move_joint(self.target_joint)
                        self.target_reached = True
                        self.auto_descent(floor_z=self.nominal_grasp_z_mm)
                        self.auto_descend_to_grasp_z()
                elif key == ord("a"):
                    self.fine_adjust(-self.arguments.fine_step_mm, 0.0)
                elif key == ord("d"):
                    self.fine_adjust(self.arguments.fine_step_mm, 0.0)
                elif key == ord("s"):
                    self.fine_adjust(0.0, -self.arguments.fine_step_mm)
                elif key == ord("w"):
                    self.fine_adjust(0.0, self.arguments.fine_step_mm)
                elif key == ord("j"):
                    self.fine_adjust(0.0, 0.0, -self.arguments.fine_z_step_mm)
                elif key == ord("u"):
                    self.fine_adjust(0.0, 0.0, self.arguments.fine_z_step_mm)
                elif key == ord("g"):
                    self.auto_descend_to_grasp_z()
                elif key == ord("v"):
                    self.save_validation_sample()
                elif key in (ord("r"), ord("h")):
                    self.safe_return()
                elif key == ord("q"):
                    return
            except RuntimeError as error:
                print(f"Command rejected/stopped: {error}")


def main(argv=None) -> None:
    arguments = parse_args(argv)
    calibration_path, calibration, transform = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    rclpy.init(args=[])
    node = ReferenceOverheadTest(
        arguments, calibration_path, calibration, transform
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
