#!/usr/bin/env python3
"""动态高度抓取测试：抓取高度由摄像头识别到的物体表面 Z 决定（不固定）。

点击物体 → 深度测量表面 Z → 抓取位 tool0 Z = 表面 Z + 偏移（默认 0）
M 渐进下降到该高度（IK 用当前关节参考）→ 灵巧手可选抓取 → R 安全返回。

按键:
  点击 + C  计算（打印摄像头识别的物体表面高度与抓取位）
  M        移动到抓取位并下降（TL_USE_HAND=1 时灵巧手力度5抓取）
  a/d/s/w  微调 X/Y（1mm）  j/u 微调 Z（5mm，j 向下）
  R        抬升后返回观察位
  Q        退出
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam
from tl_ros2_interface.srv import ModbusWrite

from .overhead_test import ReferenceOverheadTest, load_calibration
from .reference_compat import camera_point_to_base

REG_POS_TARGET = 1135
MASTER_ID = 1
SLAVE_ID = 2
PORT = 2
BAUDRATE = 115200


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Dynamic-height grasp test: grasp Z follows the camera-measured "
        "object surface height"
    )
    parser.add_argument("--calibration", type=Path, default=Path("~/tl_handeye_data/best/handeye_best.yaml"))
    parser.add_argument("--allow-rejected", action="store_true")
    parser.add_argument("--base-xy-bias-mm", type=float, nargs=2, default=[-53.2, -34.2])
    parser.add_argument("--grasp-rpy-rad", type=float, nargs=3, default=[-2.043, 1.471, -1.025])
    parser.add_argument(
        "--grasp-surface-offset-mm",
        type=float,
        default=15.0,
        help="protection height: tool0 Z sits this far ABOVE the camera-measured "
        "object surface (15 mm keeps the ROH hand from pressing the object)",
    )
    parser.add_argument(
        "--observe-joint-deg",
        type=float,
        nargs=6,
        default=[-1.521, -1.225, 35.678, 3.856, 31.471, -3.533],
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="observation/home pose at startup and return",
    )
    parser.add_argument("--use-hand", action="store_true")
    parser.add_argument("--hand-grasp-positions", type=int, nargs=6, default=[35000, 35000, 35000, 35000, 35000, 25000])
    parser.add_argument("--hand-open-positions", type=int, nargs=6, default=[0, 0, 0, 0, 0, 0])
    parser.add_argument("--fine-step-mm", type=float, default=1.0)
    parser.add_argument("--fine-z-step-mm", type=float, default=5.0)
    parser.add_argument("--fine-interp-points", type=int, default=100)
    parser.add_argument("--arrival-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--hover-mm", type=float, default=100.0)
    parser.add_argument("--min-tool-z-mm", type=float, default=-150.0)
    parser.add_argument("--mock-grasp-clearance-mm", type=float, default=20.0)
    parser.add_argument("--min-mock-clearance-mm", type=float, default=None)
    parser.add_argument("--min-target-depth-m", type=float, default=0.15)
    parser.add_argument("--max-target-depth-m", type=float, default=0.65)
    parser.add_argument(
        "--support-depth-m",
        type=float,
        default=None,
        help="camera depth of the empty support surface; used to verify a seed "
        "exists at the grasp position before grasping",
    )
    parser.add_argument(
        "--min-object-height-mm",
        type=float,
        default=20.0,
        help="minimum apparent object height above the support surface to accept "
        "the grasp position (empty positions are rejected)",
    )
    parser.add_argument("--max-joint-delta-deg", type=float, default=220.0)
    parser.add_argument("--auto-lift-to-reachable", action="store_true")
    parser.add_argument("--max-auto-lift-mm", type=float, default=200.0)
    parser.add_argument("--interp-points", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--servoj-speed-deg-s", type=float, default=20.0)
    parser.add_argument("--servoj-precision-speed-deg-s", type=float, default=8.0)
    parser.add_argument("--servoj-min-duration-sec", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--hand-settle-sec", type=float, default=1.5)
    arguments = parser.parse_args(argv)
    arguments.validation_log = Path("~/tl_handeye_validation/dynamic_grasp.jsonl")
    arguments.execute = True
    return arguments


class DynamicGraspTestNode(ReferenceOverheadTest):
    def __init__(self, arguments, calibration_path, calibration, tool_camera):
        super().__init__(arguments, calibration_path, calibration, tool_camera)
        self.clicked_pixel = None
        self.surface_base_mm = None
        self.grasp_pose = None
        self.grasp_joint = None
        self.hand_modbus_client = self.create_client(
            ModbusWrite, "/tl_driver/modbus_write"
        )

    def _target_rpy(self):
        return np.asarray(self.arguments.grasp_rpy_rad, dtype=float)

    def calculate_target(self, depth_frame) -> None:
        if self.clicked_pixel is None:
            raise RuntimeError("click the object surface first")
        pixel_x, pixel_y = self.clicked_pixel
        depth_m = self._median_depth(depth_frame, pixel_x, pixel_y)
        if not self.arguments.min_target_depth_m <= depth_m <= self.arguments.max_target_depth_m:
            raise RuntimeError(
                f"selected depth {depth_m:.3f} m is outside "
                f"{self.arguments.min_target_depth_m:.3f}.."
                f"{self.arguments.max_target_depth_m:.3f} m"
            )
        if self.arguments.support_depth_m is not None:
            apparent_height_mm = (
                float(self.arguments.support_depth_m) - depth_m
            ) * 1000.0
            if apparent_height_mm < float(self.arguments.min_object_height_mm):
                raise RuntimeError(
                    f"no seed at the grasp position: apparent height above the "
                    f"support surface {apparent_height_mm:.0f} mm < "
                    f"{self.arguments.min_object_height_mm:.0f} mm "
                    f"(click depth {depth_m:.3f} m vs support {self.arguments.support_depth_m:.3f} m)"
                )
            print(
                f"seed detected at the grasp position: apparent height "
                f"≈{apparent_height_mm:.0f} mm above the support surface"
            )
        print(f"click depth={depth_m:.3f} m (support-surface depth for calibration)")
        camera_point = np.asarray(
            self._deproject(pixel_x, pixel_y, depth_m), dtype=float
        )
        surface = np.asarray(
            camera_point_to_base(
                self.latest_tcp, self.tool_camera, camera_point
            ),
            dtype=float,
        )
        surface[:2] += np.asarray(self.arguments.base_xy_bias_mm, dtype=float)
        grasp = surface.copy()
        grasp[2] += float(self.arguments.grasp_surface_offset_mm)
        target = [float(value) for value in grasp[:3]] + list(self._target_rpy())
        self.surface_base_mm = surface.tolist()
        self.nominal_grasp_z = float(grasp[2])
        # Same mechanism as the basket flow: auto-lift when unreachable from the
        # observation reference, then the move step descends progressively with
        # the current joint reference.
        target, _, target_joint = self._solve_target_with_optional_lift(
            target, None
        )
        self.grasp_pose = list(target)
        self.grasp_joint = list(target_joint)
        self._validate_joint(self.grasp_joint)
        print(
            "camera-measured object surface base[mm]="
            f"{np.round(surface, 2).tolist()}"
        )
        print(
            f"dynamic grasp tool0 Z={self.nominal_grasp_z:.2f} mm "
            f"(surface {surface[2]:.2f} + offset {self.arguments.grasp_surface_offset_mm:.1f})"
        )
        print(f"grasp TCP={np.round(target, 3).tolist()}")
        print(f"grasp IK={np.round(self.grasp_joint[:6], 3).tolist()}")
        print("Press m to move and grasp, or fine-adjust first.")

    def _deproject(self, pixel_x: int, pixel_y: int, depth_m: float):
        import pyrealsense2 as rs

        return rs.rs2_deproject_pixel_to_point(
            self.color_intrinsics, [float(pixel_x), float(pixel_y)], depth_m
        )

    def move_and_grasp(self) -> None:
        if self.grasp_joint is None or self.grasp_pose is None:
            raise RuntimeError("click and press c first")
        self._validate_joint(self.grasp_joint)
        print("Moving to the dynamic grasp pose.")
        self.move_joint(self.grasp_joint)
        self.target_reached = True
        self.target_pose = list(self.grasp_pose)
        self.auto_descent(floor_z=self.nominal_grasp_z, stop_offset_mm=0.0)
        self.grasp_pose = list(self.target_pose)
        self.grasp_joint = list(self.target_joint)
        if self.arguments.use_hand:
            print("Grasping with the ROH hand (level 5).")
            self._hand_write_positions("grasp", self.arguments.hand_grasp_positions)
        else:
            print("Fake-hand mode: grasp simulated.")
            time.sleep(0.5)
        print("Grasped. Press r to lift and return, or fine-adjust with j/u.")

    def _hand_write_positions(self, label: str, positions) -> None:
        request = ModbusWrite.Request()
        request.master_id = MASTER_ID
        request.addr = REG_POS_TARGET
        request.data = [int(value) for value in positions]
        rtu = ModbusRTUParam()
        rtu.slave_id = SLAVE_ID
        rtu.port = PORT
        rtu.baudrate = BAUDRATE
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
        deadline = time.monotonic() + float(self.arguments.hand_settle_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _mouse_callback(self, event, x, y, flags, parameter) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_pixel = (int(x), int(y))
            self.surface_base_mm = None
            self.grasp_pose = None
            self.grasp_joint = None
            print(f"selected pixel={self.clicked_pixel}")

    def run(self) -> None:
        self.wait_for_robot()
        observe = self._pad_joint(self.arguments.observe_joint_deg)
        self._validate_joint(observe)
        current = np.asarray(self.latest_joint[:6], dtype=float)
        error = float(np.max(np.abs(np.asarray(observe[:6], dtype=float) - current)))
        if error > float(self.arguments.arrival_tolerance_deg):
            print("Moving to the observation/home pose at startup.")
            self.move_joint(observe)
        self.observation_joint = observe
        self.start_camera()
        print(
            "DYNAMIC HEIGHT GRASP: grasp Z follows the camera-measured object surface."
        )
        print("click | c calculate | m move+grasp | a/d/s/w XY | j/u Z | r return | q quit")
        window_name = "TL dynamic grasp test"
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
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            try:
                if key in (ord("q"), 27):
                    break
                if key == ord("c"):
                    self.calculate_target(depth_frame)
                elif key == ord("m"):
                    self.move_and_grasp()
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
                elif key in (ord("r"), ord("h")):
                    self.safe_return()
            except RuntimeError as error:
                print(f"Command rejected/stopped: {error}")


def main(argv=None):
    arguments = parse_args(argv)
    calibration_path, calibration, tool_camera = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    rclpy.init()
    node = DynamicGraspTestNode(
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
