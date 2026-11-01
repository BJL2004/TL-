import json
import math
import os
import select
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tl_ros2_interface.msg import CartesianPose

from .geometry import pose_to_transform, rotation_distance_rad


class HandEyeCollectNode(Node):
    def __init__(self) -> None:
        super().__init__("handeye_collect_node")
        self._declare_parameters()
        self.lock = threading.Lock()
        self.latest_pose = None
        self.latest_pose_at = 0.0
        self.latest_joint = None
        self.joint_history = deque()
        self.saved_samples = []
        self.pipeline = rs.pipeline()
        self.last_frame = None
        self.last_frame_timestamp_ms = None

        output_root = Path(os.path.expanduser(str(self.get_parameter("output_root").value)))
        session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
        self.session_dir = output_root / session_name
        self.image_dir = self.session_dir / "images"
        self.image_dir.mkdir(parents=True, exist_ok=False)

        self.create_subscription(
            CartesianPose,
            str(self.get_parameter("tcp_pose_topic").value),
            self._pose_callback,
            30,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._joint_callback,
            30,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("output_root", "~/tl_handeye_data")
        self.declare_parameter("tcp_pose_topic", "/tcp_pose")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("pattern_cols", 9)
        self.declare_parameter("pattern_rows", 6)
        self.declare_parameter("square_size_m", 0.025)
        self.declare_parameter("pose_timeout_sec", 0.5)
        self.declare_parameter("settle_seconds", 2.0)
        self.declare_parameter("settle_joint_range_deg", 0.15)
        self.declare_parameter("min_translation_delta_mm", 20.0)
        self.declare_parameter("min_rotation_delta_deg", 8.0)
        self.declare_parameter("min_board_area_ratio", 0.02)
        self.declare_parameter("min_board_sharpness", 40.0)
        self.declare_parameter("show_preview", True)

    def _pose_callback(self, message: CartesianPose) -> None:
        with self.lock:
            self.latest_pose = [
                float(message.position.x),
                float(message.position.y),
                float(message.position.z),
                float(message.rpy.x),
                float(message.rpy.y),
                float(message.rpy.z),
            ]
            self.latest_pose_at = time.monotonic()

    def _joint_callback(self, message: JointState) -> None:
        now = time.monotonic()
        joints = [float(value) for value in message.position[:6]]
        with self.lock:
            self.latest_joint = joints
            self.joint_history.append((now, joints))
            while self.joint_history and now - self.joint_history[0][0] > 3.0:
                self.joint_history.popleft()

    def _start_camera(self) -> None:
        config = rs.config()
        config.enable_stream(
            rs.stream.color,
            int(self.get_parameter("color_width").value),
            int(self.get_parameter("color_height").value),
            rs.format.bgr8,
            int(self.get_parameter("fps").value),
        )
        profile = self.pipeline.start(config)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_profile.get_intrinsics()
        device = profile.get_device()
        metadata = {
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "camera_serial": device.get_info(rs.camera_info.serial_number),
            "stream": {
                "width": intrinsics.width,
                "height": intrinsics.height,
                "fps": int(self.get_parameter("fps").value),
            },
            "intrinsics": {
                "fx": intrinsics.fx,
                "fy": intrinsics.fy,
                "ppx": intrinsics.ppx,
                "ppy": intrinsics.ppy,
                "model": str(intrinsics.model),
                "coeffs": list(intrinsics.coeffs),
            },
            "checkerboard": {
                "pattern_cols": int(self.get_parameter("pattern_cols").value),
                "pattern_rows": int(self.get_parameter("pattern_rows").value),
                "square_size_m": float(self.get_parameter("square_size_m").value),
            },
            "robot_pose": {
                "translation_unit": "mm",
                "rotation_unit": "rad",
                "rpy_convention": "intrinsic_xyz",
                "source": "/tcp_pose with tool_num=0 and coord=1; user frame is not used",
            },
            "joint_state_unit": "degree (vendor driver behavior)",
            "capture_quality": {
                "settle_seconds": float(self.get_parameter("settle_seconds").value),
                "settle_joint_range_deg": float(
                    self.get_parameter("settle_joint_range_deg").value
                ),
                "min_board_area_ratio": float(
                    self.get_parameter("min_board_area_ratio").value
                ),
                "min_board_sharpness": float(
                    self.get_parameter("min_board_sharpness").value
                ),
            },
        }
        (self.session_dir / "session.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    def _stable(self) -> tuple[bool, str]:
        now = time.monotonic()
        settle_seconds = float(self.get_parameter("settle_seconds").value)
        with self.lock:
            recent = [(stamp, joints) for stamp, joints in self.joint_history if now - stamp <= settle_seconds]
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < settle_seconds * 0.8:
            return False, "not enough stable joint history"
        limit = float(self.get_parameter("settle_joint_range_deg").value)
        for index in range(6):
            values = [joints[index] for _, joints in recent if len(joints) >= 6]
            if not values or max(values) - min(values) > limit:
                return False, f"J{index + 1} is still moving"
        return True, "ok"

    def _checkerboard(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern = (
            int(self.get_parameter("pattern_cols").value),
            int(self.get_parameter("pattern_rows").value),
        )
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(gray, pattern)
        else:
            found, corners = cv2.findChessboardCorners(gray, pattern)
        if not found:
            return False, corners, None
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
        corners = cv2.cornerSubPix(
            gray,
            corners.astype(np.float32),
            (5, 5),
            (-1, -1),
            criteria,
        )
        flat_corners = corners.reshape(-1, 2)
        hull = cv2.convexHull(flat_corners.astype(np.float32))
        area_ratio = float(cv2.contourArea(hull) / float(gray.shape[0] * gray.shape[1]))
        center = flat_corners.mean(axis=0)
        x, y, width, height = cv2.boundingRect(flat_corners.astype(np.float32))
        board_gray = gray[y : y + height, x : x + width]
        sharpness = float(cv2.Laplacian(board_gray, cv2.CV_64F).var()) if board_gray.size else 0.0
        quality = {
            "center_normalized": [
                float(center[0] / gray.shape[1]),
                float(center[1] / gray.shape[0]),
            ],
            "area_ratio": area_ratio,
            "sharpness_laplacian": sharpness,
        }
        return True, corners, quality

    def _coverage_bins(self):
        occupied = set()
        for sample in self.saved_samples:
            center = sample.get("checkerboard_quality", {}).get("center_normalized")
            if center is None:
                continue
            column = min(2, max(0, int(float(center[0]) * 3.0)))
            row = min(2, max(0, int(float(center[1]) * 3.0)))
            occupied.add(row * 3 + column)
        return sorted(occupied)

    def _write_collection_summary(self):
        summary = {
            "sample_count": len(self.saved_samples),
            "image_coverage_bins": len(self._coverage_bins()),
            "image_coverage_bin_indices": self._coverage_bins(),
        }
        if self.saved_samples:
            transforms = [
                pose_to_transform(
                    sample["tcp_pose_mm_rad"][:3],
                    sample["tcp_pose_mm_rad"][3:],
                    "intrinsic_xyz",
                )
                for sample in self.saved_samples
            ]
            translations = np.asarray([transform[:3, 3] for transform in transforms])
            reference = transforms[0][:3, :3]
            rotation_vectors = [
                np.degrees(cv2.Rodrigues(reference.T @ transform[:3, :3])[0].reshape(3))
                for transform in transforms
            ]
            summary["translation_axis_span_mm"] = [
                float(value) for value in np.ptp(translations, axis=0)
            ]
            summary["rotation_axis_span_deg"] = [
                float(value) for value in np.ptp(np.asarray(rotation_vectors), axis=0)
            ]
            centered_rotations = np.asarray(rotation_vectors) - np.mean(
                rotation_vectors,
                axis=0,
            )
            summary["rotation_principal_spread_deg"] = [
                float(value)
                for value in np.linalg.svd(centered_rotations, compute_uv=False)
                / math.sqrt(len(rotation_vectors))
            ]
            summary["recommended_dataset_ready"] = bool(
                len(self.saved_samples) >= 25
                and len(self._coverage_bins()) >= 5
                and sum(value >= 25.0 for value in summary["rotation_axis_span_deg"]) >= 2
                and sum(
                    value >= 8.0
                    for value in summary["rotation_principal_spread_deg"]
                )
                >= 2
                and max(summary["translation_axis_span_mm"]) >= 120.0
            )
        else:
            summary["recommended_dataset_ready"] = False
        (self.session_dir / "collection_summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def _diverse(self, pose) -> tuple[bool, str]:
        if not self.saved_samples:
            return True, "ok"
        candidate = pose_to_transform(pose[:3], pose[3:], "intrinsic_xyz")
        min_translation = float(self.get_parameter("min_translation_delta_mm").value)
        min_rotation = float(self.get_parameter("min_rotation_delta_deg").value)
        for sample in self.saved_samples:
            previous_pose = sample["tcp_pose_mm_rad"]
            previous = pose_to_transform(previous_pose[:3], previous_pose[3:], "intrinsic_xyz")
            translation = np.linalg.norm(candidate[:3, 3] - previous[:3, 3])
            rotation = math.degrees(rotation_distance_rad(candidate, previous))
            if translation < min_translation and rotation < min_rotation:
                return False, f"too similar to sample {sample['index']}: {translation:.1f} mm, {rotation:.1f} deg"
        return True, "ok"

    def _save(self) -> None:
        if self.last_frame is None:
            print("拒绝保存：没有相机图像")
            return
        with self.lock:
            pose = None if self.latest_pose is None else list(self.latest_pose)
            joints = None if self.latest_joint is None else list(self.latest_joint)
            pose_age = time.monotonic() - self.latest_pose_at
        if pose is None or pose_age > float(self.get_parameter("pose_timeout_sec").value):
            print(f"拒绝保存：/tcp_pose 过期，age={pose_age:.3f}s")
            return
        stable, reason = self._stable()
        if not stable:
            print(f"拒绝保存：{reason}，松开 P 键后至少静止 2 秒")
            return
        found, corners, quality = self._checkerboard(self.last_frame)
        if not found:
            print("拒绝保存：未检测到完整棋盘格内角点")
            return
        min_area = float(self.get_parameter("min_board_area_ratio").value)
        if quality["area_ratio"] < min_area:
            print(
                "Rejected: checkerboard is too small in the image, "
                f"area={quality['area_ratio']:.3f} < {min_area:.3f}"
            )
            return
        min_sharpness = float(self.get_parameter("min_board_sharpness").value)
        if quality["sharpness_laplacian"] < min_sharpness:
            print(
                "Rejected: checkerboard is blurred, "
                f"sharpness={quality['sharpness_laplacian']:.1f} < {min_sharpness:.1f}"
            )
            return
        diverse, reason = self._diverse(pose)
        if not diverse:
            print(f"拒绝保存：{reason}")
            return

        index = len(self.saved_samples) + 1
        image_name = f"{index:04d}.png"
        cv2.imwrite(str(self.image_dir / image_name), self.last_frame)
        sample = {
            "index": index,
            "image": f"images/{image_name}",
            "captured_utc": datetime.now(timezone.utc).isoformat(),
            "tcp_pose_mm_rad": pose,
            "joint_position_deg": joints,
            "corner_count": int(len(corners)),
            "pose_age_sec": float(pose_age),
            "camera_timestamp_ms": self.last_frame_timestamp_ms,
            "checkerboard_quality": quality,
        }
        with (self.session_dir / "samples.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(sample, ensure_ascii=False) + "\n")
        self.saved_samples.append(sample)
        self._write_collection_summary()
        print(
            f"Quality: coverage={len(self._coverage_bins())}/9, "
            f"area={quality['area_ratio']:.3f}, "
            f"sharpness={quality['sharpness_laplacian']:.1f}"
        )
        print(f"已保存第 {index} 组：{image_name}，TCP={np.round(pose, 4).tolist()}")

    def _console_key(self):
        if not sys.stdin.isatty():
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.readline().strip().lower() if readable else None

    def run(self) -> None:
        self._start_camera()
        show_preview = bool(self.get_parameter("show_preview").value)
        print("\n手眼标定采集已启动")
        print("按住机械臂末端 P 键拖动，松开并静止至少 2 秒；按 s 保存，按 q 退出。")
        print("采集程序不会发送任何机械臂运动指令。")
        print(f"数据目录：{self.session_dir}")
        consecutive_frame_timeouts = 0
        try:
            while rclpy.ok():
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=2000)
                    consecutive_frame_timeouts = 0
                except RuntimeError as exc:
                    consecutive_frame_timeouts += 1
                    if consecutive_frame_timeouts >= 5:
                        self.get_logger().error(
                            f"D435连续{consecutive_frame_timeouts}次取帧超时，结束采集：{exc}"
                        )
                        break
                    self.get_logger().warning(
                        f"D435取帧超时（{consecutive_frame_timeouts}/5），继续等待：{exc}"
                    )
                    continue
                frame = frames.get_color_frame()
                if not frame:
                    continue
                self.last_frame = np.asanyarray(frame.get_data()).copy()
                self.last_frame_timestamp_ms = float(frame.get_timestamp())
                preview = self.last_frame.copy()
                found, corners, quality = self._checkerboard(preview)
                if found:
                    pattern = (
                        int(self.get_parameter("pattern_cols").value),
                        int(self.get_parameter("pattern_rows").value),
                    )
                    cv2.drawChessboardCorners(preview, pattern, corners, found)
                cv2.putText(
                    preview,
                    f"saved={len(self.saved_samples)} board={'OK' if found else 'NO'}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0) if found else (0, 0, 255),
                    2,
                )
                if quality is not None:
                    cv2.putText(
                        preview,
                        (
                            f"coverage={len(self._coverage_bins())}/9 "
                            f"area={quality['area_ratio']:.3f} "
                            f"sharp={quality['sharpness_laplacian']:.0f}"
                        ),
                        (10, 58),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )
                key = None
                if show_preview:
                    cv2.imshow("TL hand-eye: s=save q=quit", preview)
                    code = cv2.waitKey(1) & 0xFF
                    key = chr(code) if code not in (255, 0) else None
                else:
                    key = self._console_key()
                if key == "s":
                    self._save()
                elif key == "q":
                    break
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            self._write_collection_summary()
            print(f"采集结束，共保存 {len(self.saved_samples)} 组")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandEyeCollectNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
