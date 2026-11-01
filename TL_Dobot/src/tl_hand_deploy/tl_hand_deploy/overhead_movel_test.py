import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tl_ros2_interface.msg import ArmStatus, CartesianPose, MoveCommand

from .geometry import pose_to_transform


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Legacy MoveL D435 point selection and no-gripper overhead motion test"
    )
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--hover-mm", type=float, default=150.0)
    parser.add_argument("--safe-lift-mm", type=float, default=80.0)
    parser.add_argument("--velocity", type=float, default=20.0)
    parser.add_argument("--acceleration", type=float, default=20.0)
    parser.add_argument("--deceleration", type=float, default=20.0)
    parser.add_argument("--segment-mm", type=float, default=50.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--allow-rejected", action="store_true")
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    arguments = parser.parse_args(argv)
    if arguments.hover_mm < 100.0:
        parser.error("--hover-mm must be at least 100 for the no-gripper coarse test")
    if arguments.safe_lift_mm < 50.0:
        parser.error("--safe-lift-mm must be at least 50")
    for name in ("velocity", "acceleration", "deceleration"):
        value = float(getattr(arguments, name))
        if value <= 0.0 or value > 20.0:
            parser.error(f"--{name} must be in (0, 20]")
    if arguments.segment_mm <= 0.0 or arguments.segment_mm > 50.0:
        parser.error("--segment-mm must be in (0, 50]")
    return arguments


def _wrapped_delta(first: float, second: float) -> float:
    return (float(first) - float(second) + math.pi) % (2.0 * math.pi) - math.pi


def _load_calibration(path: Path, allow_rejected: bool):
    calibration_path = path.expanduser().resolve()
    calibration = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    accepted = bool(calibration.get("accepted", False))
    if not accepted and not allow_rejected:
        raise RuntimeError(
            "calibration is not accepted; use an accepted YAML or explicitly pass --allow-rejected for a coarse test"
        )
    transform = np.asarray(calibration["T_tool0_camera"], dtype=float).reshape(4, 4)
    convention = calibration.get("rpy_convention", "intrinsic_xyz")
    metrics = calibration.get("metrics", {})
    return calibration_path, transform, convention, accepted, metrics


class OverheadTestNode(Node):
    WORKSPACE_MIN_MM = np.array([-650.0, -650.0, 60.0], dtype=float)
    WORKSPACE_MAX_MM = np.array([650.0, 650.0, 850.0], dtype=float)

    def __init__(self, arguments, calibration_data) -> None:
        super().__init__("tl_overhead_test")
        self.arguments = arguments
        (
            self.calibration_path,
            self.tool_camera,
            self.rpy_convention,
            self.calibration_accepted,
            self.calibration_metrics,
        ) = calibration_data
        self.latest_tcp = None
        self.latest_joint = None
        self.latest_tcp_at = 0.0
        self.latest_joint_at = 0.0
        self.latest_arm_state = None
        self.latest_arm_state_at = 0.0
        self.safe_enabled = False
        self.safe_status_at = 0.0
        self.observation_pose = None
        self.clicked_pixel = None
        self.last_frame = None
        self.motion_fault = False
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.color_intrinsics = None
        self.move_publisher = self.create_publisher(MoveCommand, "/tl_safe/moveL", 10)
        self.safety_client = self.create_client(SetBool, "/tl_safe/set_enabled")
        self.create_subscription(CartesianPose, "/tcp_pose", self._tcp_callback, 20)
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 20)
        self.create_subscription(ArmStatus, "/arm_status", self._arm_status_callback, 20)
        self.create_subscription(String, "/tl_safe/status", self._safe_status_callback, 10)

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

    def _joint_callback(self, message: JointState) -> None:
        self.latest_joint = list(message.position)
        self.latest_joint_at = time.monotonic()

    def _arm_status_callback(self, message: ArmStatus) -> None:
        self.latest_arm_state = str(message.run_state).upper()
        self.latest_arm_state_at = time.monotonic()

    def _safe_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            self.safe_enabled = bool(status.get("enabled", False))
            self.safe_status_at = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            self.safe_enabled = False

    def wait_for_robot_state(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            state_is_fresh = time.monotonic() - self.latest_arm_state_at <= 0.5
            if (
                self.latest_tcp is not None
                and self.latest_joint is not None
                and state_is_fresh
                and self.latest_arm_state == "STOP"
            ):
                self.observation_pose = list(self.latest_tcp)
                return
        raise RuntimeError(
            "robot state is not ready: require fresh /tcp_pose, /joint_states and /arm_status=STOP"
        )

    def start_camera(self) -> None:
        last_error = None
        for attempt in range(1, 4):
            try:
                self.pipeline.stop()
            except RuntimeError:
                pass
            self.pipeline = rs.pipeline()
            self.align = rs.align(rs.stream.color)
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            try:
                profile = self.pipeline.start(config)
                color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
                self.color_intrinsics = color_profile.get_intrinsics()
                for _warmup_index in range(3):
                    self.pipeline.wait_for_frames(timeout_ms=3000)
                return
            except RuntimeError as error:
                last_error = error
                try:
                    self.pipeline.stop()
                except RuntimeError:
                    pass
                if attempt < 3:
                    self.get_logger().warning(f"D435启动失败，第{attempt}次重试：{error}")
                    time.sleep(1.0)
        raise RuntimeError(
            "D435启动失败，请关闭其他相机程序并检查USB3连接、线缆和供电"
        ) from last_error

    def stop_camera(self) -> None:
        try:
            self.pipeline.stop()
        except RuntimeError:
            pass
        cv2.destroyAllWindows()

    def _capture(self):
        frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms=2000))
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        return (
            np.asanyarray(color_frame.get_data()).copy(),
            depth_frame,
        )

    def _depth_median(self, depth_frame, pixel_x: int, pixel_y: int) -> float:
        samples = []
        for offset_y in range(-4, 5):
            for offset_x in range(-4, 5):
                sample_x = min(max(pixel_x + offset_x, 0), 639)
                sample_y = min(max(pixel_y + offset_y, 0), 479)
                distance_m = float(depth_frame.get_distance(sample_x, sample_y))
                if 0.05 < distance_m < 2.0:
                    samples.append(distance_m)
        if len(samples) < 8:
            raise RuntimeError("no reliable D435 depth near selected pixel")
        return float(np.median(np.asarray(samples, dtype=float)))

    def point_in_base(self, frame, depth_frame):
        if self.clicked_pixel is None:
            raise RuntimeError("click the object center first")
        if self.latest_tcp is None or time.monotonic() - self.latest_tcp_at > 0.5:
            raise RuntimeError("/tcp_pose is stale")
        pixel_x, pixel_y = self.clicked_pixel
        depth_m = self._depth_median(depth_frame, pixel_x, pixel_y)
        camera_point = rs.rs2_deproject_pixel_to_point(
            self.color_intrinsics,
            [float(pixel_x), float(pixel_y)],
            depth_m,
        )
        base_tool = pose_to_transform(
            np.asarray(self.latest_tcp[:3], dtype=float) / 1000.0,
            self.latest_tcp[3:],
            self.rpy_convention,
        )
        base_point = base_tool @ self.tool_camera @ np.array(
            [camera_point[0], camera_point[1], camera_point[2], 1.0], dtype=float
        )
        return np.asarray(base_point[:3] * 1000.0, dtype=float), depth_m, camera_point

    def _publish_move(self, target_pose) -> None:
        command = MoveCommand()
        command.target_pos_value = [float(value) for value in target_pose] + [0.0] * 8
        command.target_pos_type = 0
        command.coord = 1
        command.velocity = float(self.arguments.velocity)
        command.velocity_sync = 0.0
        command.acc = float(self.arguments.acceleration)
        command.dec = float(self.arguments.deceleration)
        command.pl = 0
        command.time = 0
        command.tool_num = 0
        command.user_num = 0
        command.posidtype = 0
        command.configuration = 0
        command.spin = 0
        command.para_sync = False
        self.move_publisher.publish(command)

    def _require_safety_proxy(self) -> None:
        if self.move_publisher.get_subscription_count() == 0:
            raise RuntimeError("/tl_safe/moveL has no subscriber; start safety_only.launch.py")
        if time.monotonic() - self.safe_status_at > 1.5:
            raise RuntimeError("/tl_safe/status is missing or stale")
        if not self.safe_enabled:
            raise RuntimeError("safety proxy is locked; call /tl_safe/set_enabled first")

    def _set_safety_enabled(self, enabled: bool) -> bool:
        if not self.safety_client.wait_for_service(timeout_sec=2.0):
            print("Safety service /tl_safe/set_enabled is unavailable.")
            return False
        request = SetBool.Request()
        request.data = enabled
        future = self.safety_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None:
            print("Safety service call timed out.")
            return False
        response = future.result()
        print(f"Safety proxy: {response.message}")
        if response.success:
            self.safe_enabled = enabled
            self.safe_status_at = time.monotonic()
        return bool(response.success)

    def _wait_at(self, target_pose) -> None:
        command_sent_at = time.monotonic()
        deadline = time.monotonic() + self.arguments.timeout_sec
        motion_started = False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_tcp is None or self.latest_arm_state is None:
                continue
            if time.monotonic() - self.latest_arm_state_at > 1.0:
                raise RuntimeError("/arm_status became stale while waiting for motion")
            position_error = math.dist(self.latest_tcp[:3], target_pose[:3])
            orientation_error = math.sqrt(
                sum(
                    _wrapped_delta(self.latest_tcp[index], target_pose[index]) ** 2
                    for index in range(3, 6)
                )
            )
            if self.latest_arm_state == "RUNNING":
                motion_started = True
            elif self.latest_arm_state == "PAUSE":
                raise RuntimeError(
                    f"driver paused before target; current={np.round(self.latest_tcp, 3).tolist()}"
                )
            elif self.latest_arm_state == "STOP":
                if position_error <= 5.0 and orientation_error <= 0.08:
                    return
                if motion_started:
                    raise RuntimeError(
                        "driver stopped before target; "
                        f"target={np.round(target_pose, 3).tolist()}, "
                        f"current={np.round(self.latest_tcp, 3).tolist()}, "
                        f"position_error_mm={position_error:.2f}"
                    )
                if time.monotonic() - command_sent_at >= 5.0:
                    raise RuntimeError(
                        "driver did not start MoveL within 5 seconds; "
                        f"target={np.round(target_pose, 3).tolist()}, "
                        f"current={np.round(self.latest_tcp, 3).tolist()}"
                    )
        raise RuntimeError(
            "motion timeout; "
            f"state={self.latest_arm_state}, "
            f"target={np.round(target_pose, 3).tolist()}, "
            f"current={None if self.latest_tcp is None else np.round(self.latest_tcp, 3).tolist()}"
        )

    def _move_segment(self, start_pose, target_pose) -> None:
        start = np.asarray(start_pose, dtype=float)
        target = np.asarray(target_pose, dtype=float)
        distance = math.dist(start[:3], target[:3])
        segment_count = max(1, math.ceil(distance / self.arguments.segment_mm))
        for segment_index in range(1, segment_count + 1):
            ratio = segment_index / segment_count
            waypoint = start + (target - start) * ratio
            print(
                f"MoveL {segment_index}/{segment_count}: "
                f"{np.round(waypoint, 3).tolist()}"
            )
            self._publish_move(waypoint.tolist())
            self._wait_at(waypoint.tolist())

    def _overhead_pose(self, base_point_mm):
        target_pose = list(self.latest_tcp)
        target_pose[:3] = [
            float(base_point_mm[0]),
            float(base_point_mm[1]),
            float(base_point_mm[2] + self.arguments.hover_mm),
        ]
        return target_pose

    def _require_workspace(self, label: str, pose) -> None:
        position = np.asarray(pose[:3], dtype=float)
        if np.any(position < self.WORKSPACE_MIN_MM) or np.any(position > self.WORKSPACE_MAX_MM):
            raise RuntimeError(
                f"{label} XYZ={np.round(position, 2).tolist()} mm超出安全工作区"
                f" {self.WORKSPACE_MIN_MM.tolist()}..{self.WORKSPACE_MAX_MM.tolist()}"
            )

    def move_to_overhead(self, base_point_mm) -> None:
        self._require_safety_proxy()
        current = list(self.latest_tcp)
        lift_pose = list(current)
        lift_pose[2] += float(self.arguments.safe_lift_mm)
        target_pose = self._overhead_pose(base_point_mm)
        transfer_pose = list(target_pose)
        transfer_pose[2] = max(lift_pose[2], target_pose[2])
        self._require_workspace("抬升点", lift_pose)
        self._require_workspace("转运点", transfer_pose)
        self._require_workspace("悬停目标", target_pose)
        self._move_segment(current, lift_pose)
        self._move_segment(lift_pose, transfer_pose)
        self._move_segment(transfer_pose, target_pose)

    def return_observation(self) -> None:
        self._require_safety_proxy()
        current = list(self.latest_tcp)
        lift_pose = list(current)
        lift_pose[2] += float(self.arguments.safe_lift_mm)
        observation_lift = list(self.observation_pose)
        observation_lift[2] = max(lift_pose[2], self.observation_pose[2])
        self._require_workspace("返回抬升点", lift_pose)
        self._require_workspace("检测位上方", observation_lift)
        self._require_workspace("检测位", self.observation_pose)
        self._move_segment(current, lift_pose)
        self._move_segment(lift_pose, observation_lift)
        self._move_segment(observation_lift, self.observation_pose)

    def _draw_overlay(self, frame, base_point_mm=None, depth_m=None):
        preview = frame.copy()
        if self.clicked_pixel is not None:
            pixel_x, pixel_y = self.clicked_pixel
            cv2.drawMarker(preview, (pixel_x, pixel_y), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
        status = "EXECUTE" if self.arguments.execute else "DRY RUN"
        cv2.putText(
            preview,
            f"{status} | click | c calc | u unlock | m move | r home | l lock | q quit",
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            2,
        )
        if depth_m is not None:
            cv2.putText(preview, f"depth={depth_m:.3f} m", (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        if base_point_mm is not None:
            text = "base xyz=" + ",".join(f"{value:.1f}" for value in base_point_mm)
            cv2.putText(preview, text, (8, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return preview

    def run(self) -> None:
        self.wait_for_robot_state()
        self.start_camera()
        base_point_mm = None
        depth_m = None
        camera_point = None
        consecutive_frame_timeouts = 0
        cv2.namedWindow("TL overhead test", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("TL overhead test", self._mouse_callback)
        print(f"检测位TCP：{np.round(self.observation_pose, 3).tolist()}")
        print(f"标定文件：{self.calibration_path}")
        print(f"标定accepted：{self.calibration_accepted}，执行模式：{self.arguments.execute}")
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.001)
                try:
                    frame, depth_frame = self._capture()
                except RuntimeError as error:
                    consecutive_frame_timeouts += 1
                    self.get_logger().warning(
                        f"D435取帧超时（{consecutive_frame_timeouts}/3）：{error}"
                    )
                    if consecutive_frame_timeouts >= 3:
                        self.get_logger().warning("D435连续取帧失败，自动重启相机流")
                        self.start_camera()
                        consecutive_frame_timeouts = 0
                    continue
                if frame is None:
                    continue
                consecutive_frame_timeouts = 0
                self.last_frame = frame
                preview = self._draw_overlay(frame, base_point_mm, depth_m)
                cv2.imshow("TL overhead test", preview)
                key_code = cv2.waitKey(1) & 0xFF
                if key_code == 255:
                    continue
                key = chr(key_code).lower()
                if key == "c":
                    try:
                        base_point_mm, depth_m, camera_point = self.point_in_base(frame, depth_frame)
                        overhead_pose = self._overhead_pose(base_point_mm)
                        print(
                            "物体点：camera[m]="
                            f"{np.round(camera_point, 4).tolist()}，base[mm]={np.round(base_point_mm, 2).tolist()}"
                        )
                        print(
                            f"悬停目标TCP：{np.round(overhead_pose, 2).tolist()}，"
                            f"hover={self.arguments.hover_mm:.1f} mm"
                        )
                        self._require_workspace("悬停目标", overhead_pose)
                    except (RuntimeError, ValueError) as error:
                        print(f"计算失败：{error}")
                elif key == "u":
                    self._set_safety_enabled(True)
                elif key == "l":
                    self._set_safety_enabled(False)
                elif key == "m":
                    if self.motion_fault:
                        print("本次程序曾发生运动超时，禁止继续发送命令；请按 q 退出后重新启动。")
                    elif base_point_mm is None:
                        print("请先点击物体，再按 c 计算目标。")
                    elif not self.arguments.execute:
                        print(
                            f"当前是DRY RUN，不会运动；预计悬停TCP="
                            f"{np.round(self._overhead_pose(base_point_mm), 2).tolist()}"
                        )
                    elif not self.calibration_accepted and not self.arguments.allow_rejected:
                        print("标定未通过；粗测需显式使用 --allow-rejected。")
                    else:
                        print("开始移动到物体上方，不下降、不接触物体。")
                        try:
                            self.move_to_overhead(base_point_mm)
                            print("已到达物体上方。按 r 返回检测位。")
                        except RuntimeError as error:
                            self.motion_fault = True
                            print(f"运动中止：{error}")
                elif key in {"r", "h"}:
                    if not self.arguments.execute:
                        print("当前是DRY RUN，不会返回运动。")
                    else:
                        print("返回检测位。")
                        try:
                            self.return_observation()
                            print("已返回检测位。")
                        except RuntimeError as error:
                            self.motion_fault = True
                            print(f"返回中止：{error}")
                elif key == "q":
                    break
        finally:
            if self.arguments.execute:
                self._set_safety_enabled(False)
            self.stop_camera()

    def _mouse_callback(self, event, pixel_x, pixel_y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_pixel = (int(pixel_x), int(pixel_y))
            print(f"已选择像素：({pixel_x}, {pixel_y})")


def main(argv=None) -> None:
    arguments = parse_args(argv)
    calibration_data = _load_calibration(arguments.calibration, arguments.allow_rejected)
    rclpy.init(args=[])
    node = OverheadTestNode(arguments, calibration_data)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
