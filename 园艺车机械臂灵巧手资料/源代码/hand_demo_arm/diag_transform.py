#!/usr/bin/env python3
"""
坐标变换诊断工具 — 排查视觉抓取偏移问题
=========================================

原理:
  手眼标定脚本 save_poses.py 用 Rz@Ry@Rx (extrinsic xyz)
  运行时代码用 R.from_euler('XYZ') (intrinsic XYZ)
  两者对同一组 RPY 产生不同旋转矩阵 → 坐标变换完全错误

用法:
  终端1: ros2 launch tl_driver tl_driver.launch.py arm_type:=tcb605
  终端2: python3 diag_transform.py

操作:
  把一个物体放在机械臂前方桌面上(已知大致位置)
  按 [d] 检测并对比两种欧拉角约定的坐标变换结果
  按 [t] 移动到 extrinsic xyz 计算的位置上方(测试)
  按 [T] 移动到 intrinsic XYZ 计算的位置上方(测试)
  按 [q] 退出

  哪种约定算出的基座坐标更接近物体真实位置，就用那种。
"""

import signal, os, sys, time, math, threading
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tl_ros2_interface.msg import CartesianPose
from tl_ros2_interface.srv import CoordTransform, OpenServoJ
from std_msgs.msg import Float64MultiArray
from ultralytics import YOLO
import pyrealsense2 as rs


# 当前手眼矩阵 (与 vision_grasp_demo.py / control_node.yaml 一致)
HEANDEYE = np.array([
    [0.0584447131, -0.9980305435, -0.0227870524, 0.0409048638],
    [0.9982592682,  0.0586086291, -0.0065925805, -0.0039856175],
    [0.0079151146, -0.0223620848,  0.9997186045, -0.0696447468],
    [0.0,           0.0,            0.0,           1.0]
])


class DiagTransform(Node):
    def __init__(self):
        super().__init__('diag_transform')

        self.latest_joint = None
        self.latest_tcp = None
        self.servoj_target = [0.0] * 7
        self.servoj_init = False
        self.servoj_moving = False
        self.servoj_traj = []
        self.servoj_idx = 0
        self.servoj_final = None
        self.running = True

        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(CartesianPose, '/tcp_pose', self.tcp_cb, 10)
        self.servoj_pub = self.create_publisher(Float64MultiArray, '/tl_driver/set_servoj_pos', 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.ik_client = self.create_client(CoordTransform, '/tl_driver/coord_transform')
        self.servoj_srv = self.create_client(OpenServoJ, '/tl_driver/open_servoj')
        self.create_timer(0.01, self.servoj_timer_cb)

        # D435
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx, self.fy = intr.fx, intr.fy
        self.cx_cam, self.cy_cam = intr.ppx, intr.ppy
        self.get_logger().info(f"D435 内参: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx_cam:.1f} cy={self.cy_cam:.1f}")

        self._frame_lock = threading.Lock()
        self._color_np = None
        self._depth_np = None
        self._cam_running = True
        threading.Thread(target=self._cam_thread, daemon=True).start()

        # YOLO
        model_path = os.path.expanduser('~/.config/Ultralytics/yolov8n.pt')
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO 就绪")

        # 激活 ServoJ
        self._activate_servoj()

        self._cam_obj = None  # (x, y, z, name) 相机坐标系 米

        print("=" * 60)
        print("  坐标变换诊断工具")
        print("=" * 60)
        print("  [d] 检测 + 对比两种欧拉角约定")
        print("  [t] 测试 extrinsic xyz (标定脚本同款)")
        print("  [T] 测试 intrinsic XYZ  (当前代码同款)")
        print("  [z] 回零")
        print("  [q] 退出")
        print("=" * 60)
        print("  把物体放在臂前方，按 d 检测，看哪种约定算出的")
        print("  基座坐标更接近物体真实位置。")
        print("=" * 60)

    def joint_cb(self, m):
        self.latest_joint = m

    def tcp_cb(self, m):
        self.latest_tcp = m

    def _cam_thread(self):
        while self._cam_running:
            try:
                frames = self.pipe.wait_for_frames(timeout_ms=50)
                aligned = self.align.process(frames)
                color = aligned.get_color_frame()
                depth = aligned.get_depth_frame()
                if color and depth:
                    with self._frame_lock:
                        self._color_np = np.asanyarray(color.get_data())
                        self._depth_np = np.asanyarray(depth.get_data())
            except:
                pass

    def _activate_servoj(self):
        self.servoj_srv.wait_for_service(timeout_sec=5)
        req = OpenServoJ.Request()
        req.vmax = [10.0]*7
        req.amax = [3000.0]*7
        req.jmax = [50000.0]*7
        f = self.servoj_srv.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=3)
        t0 = time.time()
        while self.latest_joint is None and time.time()-t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint:
            p = [v*180.0/math.pi for v in self.latest_joint.position]
            self.servoj_target = (p + [0.]*7)[:7]
            self.servoj_init = True
            self.get_logger().info("ServoJ 就绪")

    def servoj_timer_cb(self):
        if not self.running or not self.servoj_init:
            return
        if self.servoj_moving:
            if self.servoj_idx < len(self.servoj_traj):
                self.servoj_target = self.servoj_traj[self.servoj_idx]
                self.servoj_idx += 1
                if self.servoj_final and max(abs(self.servoj_target[i]-self.servoj_final[i]) for i in range(7)) < 0.5:
                    self.servoj_target = list(self.servoj_final)
                    self.servoj_moving = False
                    self.servoj_final = None
            else:
                self.servoj_moving = False
                self.servoj_final = None
        m = Float64MultiArray()
        m.data = [float(v) for v in self.servoj_target]
        self.servoj_pub.publish(m)
        if self.latest_joint:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self.latest_joint.name)
            js.position = [v*math.pi/180. for v in self.servoj_target[:len(js.name)]]
            self.js_pub.publish(js)

    def movej(self, joints):
        tgt = list(joints) + [0.]*(7-len(joints))
        tgt = tgt[:7]
        start = list(self.servoj_target)
        n = 100
        self.servoj_traj = [[start[i]+(tgt[i]-start[i])*s/n for i in range(7)] for s in range(1, n+1)]
        self.servoj_final = tgt
        self.servoj_idx = 0
        self.servoj_moving = True

    def wait_arrival(self, t=20):
        t0 = time.time()
        while self.servoj_moving and time.time()-t0 < t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def movej_cartesian(self, cart):
        req = CoordTransform.Request()
        req.origin_coord = 1
        req.target_coord = 0
        req.form = 0
        req.origin_pos = list(cart) + [0.]*(7-len(cart))
        req.origin_pos = req.origin_pos[:7]
        req.reference_pos = [0.]*7
        f = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5)
        if f.done() and f.result() and f.result().success:
            self.movej(list(f.result().target_pos))
            return True
        return False

    # ==================== 核心：两种约定的坐标变换 ====================

    def cam_to_base_both(self, cam_xyz):
        """
        同时用两种欧拉角约定计算相机→基座坐标变换，返回两种结果。
        """
        if not self.latest_tcp:
            return None, None

        tcp = self.latest_tcp
        pos_m = np.array([tcp.position.x/1000., tcp.position.y/1000., tcp.position.z/1000.])
        rpy = np.array([tcp.rpy.x, tcp.rpy.y, tcp.rpy.z])

        # --- 约定 A: extrinsic xyz (标定脚本 save_poses.py 用的) ---
        # R = Rz(rz) @ Ry(ry) @ Rx(rx) = R.from_euler('xyz', rpy)
        R_a = Rot.from_euler('xyz', rpy).as_matrix()
        T_base_tool_a = np.eye(4)
        T_base_tool_a[:3, :3] = R_a
        T_base_tool_a[:3, 3] = pos_m
        T_base_cam_a = T_base_tool_a @ HEANDEYE
        P_base_a = (T_base_cam_a @ np.array([*cam_xyz, 1.0]))[:3] * 1000  # mm

        # --- 约定 B: intrinsic XYZ (当前运行时代码用的) ---
        # R = Rx(rx) @ Ry(ry) @ Rz(rz) = R.from_euler('XYZ', rpy)
        R_b = Rot.from_euler('XYZ', rpy).as_matrix()
        T_base_tool_b = np.eye(4)
        T_base_tool_b[:3, :3] = R_b
        T_base_tool_b[:3, 3] = pos_m
        T_base_cam_b = T_base_tool_b @ HEANDEYE
        P_base_b = (T_base_cam_b @ np.array([*cam_xyz, 1.0]))[:3] * 1000  # mm

        return P_base_a, P_base_b

    def detect(self):
        """检测物体并对比两种约定"""
        with self._frame_lock:
            color_np = self._color_np
            depth_np = self._depth_np
        if color_np is None:
            print("无相机帧")
            return

        results = self.model(color_np, conf=0.5, device='cpu', verbose=False)
        for r in results:
            for box in (r.boxes or []):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cx_px, cy_px = (x1+x2)//2, (y1+y2)//2
                cx_px = max(0, min(cx_px, 639))
                cy_px = max(0, min(cy_px, 479))
                rgn = depth_np[max(0,cy_px-5):min(479,cy_px+5), max(0,cx_px-5):min(639,cx_px+5)]
                valid = rgn[rgn > 0]
                if valid.size == 0:
                    print(f"  {r.names[int(box.cls[0])]}: 无有效深度")
                    continue
                z = np.median(valid) / 1000.0
                if z <= 0:
                    continue
                x_cam = (cx_px - self.cx_cam) * z / self.fx
                y_cam = (cy_px - self.cy_cam) * z / self.fy
                name = r.names[int(box.cls[0])]
                self._cam_obj = (x_cam, y_cam, z, name)

                P_a, P_b = self.cam_to_base_both((x_cam, y_cam, z))
                if P_a is None:
                    print("无 TCP 数据")
                    return

                diff = np.linalg.norm(P_a - P_b)

                print("\n" + "=" * 60)
                print(f"  检测到: {name}")
                print(f"  相机坐标: X={x_cam:.3f} Y={y_cam:.3f} Z={z:.3f} (米)")
                print(f"  TCP 位姿: x={self.latest_tcp.position.x:.1f} y={self.latest_tcp.position.y:.1f} "
                      f"z={self.latest_tcp.position.z:.1f} (mm)")
                print(f"  TCP RPY:  r={self.latest_tcp.rpy.x:.4f} p={self.latest_tcp.rpy.y:.4f} "
                      f"y={self.latest_tcp.rpy.z:.4f} (rad)")
                print("-" * 60)
                print(f"  约定A extrinsic xyz (标定脚本同款):")
                print(f"    基座坐标: X={P_a[0]:.1f}  Y={P_a[1]:.1f}  Z={P_a[2]:.1f}  (mm)")
                print(f"  约定B intrinsic XYZ  (当前代码同款):")
                print(f"    基座坐标: X={P_b[0]:.1f}  Y={P_b[1]:.1f}  Z={P_b[2]:.1f}  (mm)")
                print("-" * 60)
                print(f"  两者偏差: {diff:.1f} mm")
                if diff > 50:
                    print(f"  ⚠️  偏差巨大! 这就是抓不到的原因。")
                print(f"  → 看哪个坐标更接近物体真实位置，就用那个约定。")
                print("=" * 60 + "\n")

                # 画检测框
                cv2.rectangle(color_np, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_np, f"{name} {z:.2f}m", (x1, y1-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Diag", color_np)

    def test_move(self, use_extrinsic):
        """移动到检测物体上方 (Z+100mm)，测试坐标变换"""
        if not self._cam_obj:
            print("无检测目标，先按 d 检测")
            return
        x_cam, y_cam, z_cam, name = self._cam_obj
        P_a, P_b = self.cam_to_base_both((x_cam, y_cam, z_cam))
        if P_a is None:
            return

        target = P_a if use_extrinsic else P_b
        label = "extrinsic xyz" if use_extrinsic else "intrinsic XYZ"
        print(f"\n>> 测试 {label}: 移动到物体上方")
        print(f"   目标: X={target[0]:.1f} Y={target[1]:.1f} Z={target[2]+100:.1f} mm")

        GRASP_R, GRASP_P, GRASP_Y = 3.13, 0.92, 0.0
        cart = [target[0], target[1], target[2]+100, GRASP_R, GRASP_P, GRASP_Y]
        if self.movej_cartesian(cart):
            self.wait_arrival(20)
            print(f"   到位! 观察夹爪和物体的偏移 → 判断 {label} 是否正确")
        else:
            print(f"   IK 失败或不可达")

    def run(self):
        while self.running:
            rclpy.spin_once(self, timeout_sec=0.05)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q') or k == ord('Q'):
                self.running = False
            elif k == ord('d') or k == ord('D'):
                self.detect()
            elif k == ord('t'):
                self.test_move(use_extrinsic=True)
            elif k == ord('T'):
                self.test_move(use_extrinsic=False)
            elif k == ord('z') or k == ord('Z'):
                self.movej([0.0]*7)
        self._cam_running = False
        self.pipe.stop()
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = DiagTransform()
    signal.signal(signal.SIGINT, lambda *_: setattr(node, 'running', False))
    node.run()


if __name__ == '__main__':
    main()
