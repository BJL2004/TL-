#!/usr/bin/env python3
"""
MoveJ 鼠标点击到位 — 点击 D435 画面目标，MoveJ 直驱机械臂到目标上方
==================================================================
与 overhead_test.py (ServoJ 流式) 的区别：
  - 使用 /tl_driver/moveJ 话题（单次指令，驱动内部规划加减速）
  - 不需要 open_servoj / close_servoj
  - 运动更简单，适合快速粗定位

用法:
  cd ~/Desktop/TL_Dobot
  source /opt/ros/humble/setup.bash
  source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
  export PYTHONPATH="src/tl_hand_deploy:${PYTHONPATH:-}"
  python3 src/tl_hand_deploy/tl_hand_deploy/movej_click.py \
      --calibration ~/tl_handeye_data/session_20260808_022900/calibration/handeye_candidate_rejected.yaml \
      --allow-rejected \
      --execute
"""

import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tl_ros2_interface.msg import CartesianPose, MoveCommand
from tl_ros2_interface.srv import CoordTransform

# 复用 TL_Dobot 已有的坐标变换
from tl_hand_deploy.reference_compat import (
    camera_point_to_base,
    build_hover_pose,
    wrap_angle_rad,
)

WORKSPACE_MIN_MM = np.array([-650.0, -650.0, 60.0])
WORKSPACE_MAX_MM = np.array([650.0, 650.0, 850.0])


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="MoveJ 鼠标点击到位")
    p.add_argument("--calibration", type=Path, required=True)
    p.add_argument("--hover-mm", type=float, default=150.0, help="悬停在目标上方的高度 mm")
    p.add_argument("--velocity", type=float, default=20.0, help="MoveJ 速度百分比 5-100")
    p.add_argument("--acceleration", type=float, default=20.0)
    p.add_argument("--deceleration", type=float, default=20.0)
    p.add_argument("--allow-rejected", action="store_true")
    p.add_argument("--execute", action="store_true", help="不加则仅计算不动")
    return p.parse_args(argv)


class MoveJClickNode(Node):
    def __init__(self, args, calibration):
        super().__init__("movej_click")
        self.args = args
        self.calibration = calibration

        # 手眼矩阵
        self.tool_camera = np.asarray(
            calibration["T_tool0_camera"], dtype=float
        ).reshape(4, 4)

        # 状态
        self.latest_tcp = None
        self.latest_joint = None
        self.clicked_pixel = None

        # D435
        self.pipe = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.intrinsics = None

        # ROS2
        self.movej_pub = self.create_publisher(
            MoveCommand, "/tl_driver/moveJ", 10
        )
        self.ik_client = self.create_client(
            CoordTransform, "/tl_driver/coord_transform"
        )
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(CartesianPose, "/tcp_pose", self._tcp_cb, 10)

    def _joint_cb(self, msg):
        self.latest_joint = [float(v) for v in msg.position]

    def _tcp_cb(self, msg):
        self.latest_tcp = [
            float(msg.position.x), float(msg.position.y), float(msg.position.z),
            float(msg.rpy.x), float(msg.rpy.y), float(msg.rpy.z),
        ]

    # ===== D435 =====
    def start_camera(self):
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipe.start(cfg)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        for _ in range(5):
            self.pipe.wait_for_frames(timeout_ms=3000)

    def stop_camera(self):
        try:
            self.pipe.stop()
        except:
            pass
        cv2.destroyAllWindows()

    def _capture(self):
        frames = self.align.process(self.pipe.wait_for_frames(timeout_ms=2000))
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            return None, None
        return np.asanyarray(color.get_data()), depth

    def _depth_at(self, depth_frame, px, py, window=5):
        vals = []
        for dy in range(-window, window + 1):
            for dx in range(-window, window + 1):
                x = min(max(px + dx, 0), 639)
                y = min(max(py + dy, 0), 479)
                d = float(depth_frame.get_distance(x, y))
                if 0.05 < d < 2.0:
                    vals.append(d)
        if len(vals) < 12:
            raise RuntimeError("选中点深度无效")
        return float(np.median(vals))

    # ===== 坐标计算 =====
    def calculate(self, depth_frame):
        if self.clicked_pixel is None:
            raise RuntimeError("请先点击目标")
        if self.latest_tcp is None:
            raise RuntimeError("/tcp_pose 未收到")

        px, py = self.clicked_pixel
        depth_m = self._depth_at(depth_frame, px, py)

        # D435 像素 → 相机3D
        cam_pt = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [float(px), float(py)], depth_m
        )

        # 相机3D → 基座坐标
        base_mm = camera_point_to_base(
            self.latest_tcp, self.tool_camera, cam_pt, "intrinsic_xyz"
        )

        # 目标上方悬停位姿（保持当前TCP姿态）
        hover_pose = build_hover_pose(
            base_mm, self.latest_tcp, self.args.hover_mm
        )

        # IK 解算
        if not self.ik_client.wait_for_service(timeout_sec=3):
            raise RuntimeError("CoordTransform 不可用")

        req = CoordTransform.Request()
        req.origin_coord = 1  # 笛卡尔
        req.target_coord = 0  # 关节
        req.form = 0
        req.origin_pos = [float(v) for v in hover_pose] + [0.0]
        req.reference_pos = (self.latest_joint + [0.0])[:7] if self.latest_joint else [0.0] * 7

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if not future.done() or not future.result():
            raise RuntimeError("IK 服务超时")
        if not future.result().success:
            raise RuntimeError("IK 无解（目标不可达）")

        joints = list(future.result().target_pos)
        return base_mm, cam_pt, hover_pose, joints

    # ===== MoveJ =====
    def movej_to(self, joints_deg):
        if not self.args.execute:
            print("[DRY RUN] 不会真的运动，加 --execute 启用。")
            return

        cmd = MoveCommand()
        cmd.target_pos_value = [float(v) for v in joints_deg[:7]] + [0.0] * 7
        cmd.target_pos_value = cmd.target_pos_value[:14]
        cmd.coord = 0                          # 关节空间
        cmd.velocity = float(self.args.velocity)
        cmd.acc = float(self.args.acceleration)
        cmd.dec = float(self.args.deceleration)
        cmd.target_pos_type = 0
        cmd.pl = 0
        self.movej_pub.publish(cmd)
        print(f"✅ MoveJ 已发布: {np.round(joints_deg[:6], 2).tolist()} deg")

    def movej_observation(self):
        """返回检测位关节"""
        if self._obs_joints is not None:
            self.movej_to(self._obs_joints)

    # ===== 主循环 =====
    def run(self):
        print("等待 /tcp_pose 和 /joint_states ...")
        t0 = time.monotonic()
        while (self.latest_tcp is None or self.latest_joint is None) and time.monotonic() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_tcp is None:
            print("❌ 超时")
            return

        self._obs_joints = list(self.latest_joint)
        self._obs_tcp = list(self.latest_tcp)
        print(f"检测位 TCP: {np.round(self._obs_tcp, 1).tolist()}")
        print(f"检测位关节: {np.round(self._obs_joints[:6], 1).tolist()}")

        self.start_camera()
        base_mm, hover_pose, joints = None, None, None

        cv2.namedWindow("MoveJ Click | click=选择 c=计算 m=MoveJ r=回检测位 q=退出")
        cv2.setMouseCallback(
            "MoveJ Click | click=选择 c=计算 m=MoveJ r=回检测位 q=退出",
            lambda ev, x, y, *_: setattr(self, 'clicked_pixel', (x, y)) if ev == cv2.EVENT_LBUTTONDOWN else None,
        )

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                color, depth = self._capture()
                if color is None:
                    continue

                disp = color.copy()
                if self.clicked_pixel:
                    cv2.drawMarker(disp, self.clicked_pixel, (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(disp, "click | c | m | r | q",
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.imshow("MoveJ Click | click=选择 c=计算 m=MoveJ r=回检测位 q=退出", disp)

                key = cv2.waitKey(1) & 0xFF
                if key == 255:
                    continue
                k = chr(key).lower()

                if k == 'c':
                    try:
                        base_mm, cam_pt, hover_pose, joints = self.calculate(depth)
                        print(f"相机: {np.round(cam_pt, 3).tolist()} m")
                        print(f"基座: {np.round(base_mm, 1).tolist()} mm")
                        print(f"悬停TCP: {np.round(hover_pose, 1).tolist()}")
                        print(f"MoveJ关节: {np.round(joints[:6], 2).tolist()} deg")
                    except RuntimeError as e:
                        print(f"❌ 计算失败: {e}")
                        base_mm = None

                elif k == 'm':
                    if joints is None:
                        print("请先按 c 计算")
                    else:
                        self.movej_to(joints)

                elif k == 'r':
                    if self._obs_joints:
                        self.movej_to(self._obs_joints)
                        joints = None

                elif k == 'q':
                    break
        finally:
            self.stop_camera()


def main(argv=None):
    args = parse_args(argv)

    # 加载标定
    cal_path = args.calibration.expanduser().resolve()
    cal = yaml.safe_load(cal_path.read_text(encoding="utf-8"))
    if not cal.get("accepted", False) and not args.allow_rejected:
        print("标定未通过，加 --allow-rejected 强制使用")
        return
    print(f"标定: {cal_path}  accepted={cal.get('accepted', False)}")

    rclpy.init()
    node = MoveJClickNode(args, cal)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
