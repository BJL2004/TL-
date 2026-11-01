#!/usr/bin/env python3
"""
ARM版 D435视觉 + 机械臂 + 灵巧手 联合抓取 Demo
=============================================
依赖: camera_node.py + 本文件
相机装在机械臂末端 (eye-in-hand)

运行:
  终端1: python3 camera_node.py
  终端2: python3 vision_grasp_demo.py

按键:
  [z]回零  [g]检测+抓取  [o/c/p/f]手控制  [q]退出
"""

import signal
import os
import sys
import time
import math
import select
import termios
import tty
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PointStamped
from tl_ros2_interface.msg import ObjectInfo, CartesianPose, ModbusMasterParam, ModbusRTUParam
from tl_ros2_interface.srv import ModbusWrite, OpenServoJ
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import pyrealsense2 as rs


class VisionGraspDemo(Node):
    def __init__(self):
        super().__init__('vision_grasp_demo')

        # ===== 参数 =====
        self.declare_parameter('zero_joint', [0.0]*7)
        self.declare_parameter('grasp_z', 160.0)  # 抓取高度 mm
        self.declare_parameter('grasp_approach_z', 250.0)  # 接近高度 mm
        self.declare_parameter('calib_offset_x', 0.0)  # 手动 X 补偿 mm
        self.declare_parameter('calib_offset_y', 0.0)  # 手动 Y 补偿 mm
        self.declare_parameter('calib_offset_z', 0.0)  # 手动 Z 补偿 mm
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('model_path', '/home/nvidia/.config/Ultralytics/yolov8n.pt')
        self.declare_parameter('handeye_matrix',
            # ARM Jetson 标定: daniilidis/intrinsic_xyz, 23样本, RMS=9.87mm/0.91°
            [-0.0069293095,-0.9999440715, 0.0079899085, 0.0939027919,
              0.9996176779,-0.0067127277, 0.0268223296,-0.0313651033,
             -0.0267671954, 0.0081727140, 0.9996082853,-0.0125169900,
              0.0,          0.0,          0.0,          1.0])
        self.declare_parameter('hand_open', [0,0,0,0,0,0])
        self.declare_parameter('hand_close', [21000,20000,23000,60000,60000,60000])
        self.declare_parameter('servoj_interp_points', 500)  # 500点=5秒,确保物理到位

        self.zero_joint = list(self.get_parameter('zero_joint').value)
        self.grasp_z = float(self.get_parameter('grasp_z').value)
        self.grasp_approach_z = float(self.get_parameter('grasp_approach_z').value)
        self.hand_open_val = list(self.get_parameter('hand_open').value)
        self.hand_close_val = list(self.get_parameter('hand_close').value)
        self.servoj_interp_points = int(self.get_parameter('servoj_interp_points').value)

        T = np.array(self.get_parameter('handeye_matrix').value).reshape(4,4)
        self.T_tool_camera = T
        self.log(f"手眼矩阵:\n{T}")

        # ===== 状态 =====
        self.running = True
        self.latest_joint = None
        self.latest_tcp = None
        self.servoj_target = [0.0]*7
        self.servoj_init = False
        self.servoj_traj = []
        self.servoj_idx = 0
        self.servoj_moving = False
        self.servoj_final = None
        self._last_target = None
        self.hand_init = False
        self.detected = False

        # ===== 服务客户端 =====
        self.modbus = self.create_client(ModbusWrite, '/tl_driver/modbus_write')
        self.servoj_srv = self.create_client(OpenServoJ, '/tl_driver/open_servoj')

        # ===== 订阅 =====
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(CartesianPose, '/tcp_pose', self.tcp_cb, 10)

        # ===== 发布 =====
        self.servoj_pub = self.create_publisher(Float64MultiArray, '/tl_driver/set_servoj_pos', 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ===== 相机 + YOLO =====
        self.model = YOLO(self.get_parameter('model_path').value)
        self.log("YOLO 就绪")

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx, self.fy = intr.fx, intr.fy
        self.cx, self.cy = intr.ppx, intr.ppy
        self.log(f"D435 内参: fx={self.fx:.1f} fy={self.fy:.1f}")

        # 独立线程持续取帧
        import threading
        self._frame_lock = threading.Lock()
        self._color_np = None
        self._depth_np = None
        self._cam_running = True
        threading.Thread(target=self._cam_thread, daemon=True).start()

        # ===== 定时器 =====
        self.create_timer(0.01, self.servoj_timer_cb)
        self.create_timer(0.1, self.detect_timer_cb)

        # ===== 键盘 =====
        self.setup_kb()

        # ===== 初始化 =====
        self.modbus.wait_for_service(timeout_sec=10)
        self._hand_init()

        self._servoj_active = False  # 延迟激活标记

        # 数据采集
        import os as _os
        self._data_dir = _os.path.expanduser('~/grasp_data')
        _os.makedirs(self._data_dir, exist_ok=True)
        import glob as _glob
        existing = _glob.glob(f'{self._data_dir}/episode_*.hdf5')
        self._episode_idx = len(existing)
        self.log(f"数据目录: {self._data_dir} (已有{self._episode_idx}条)")

        self.log("=" * 50)
        self.log("  视觉抓取 Demo")
        self.log("  [z]回零  [g]检测+抓取  [t]测试偏移")
        self.log("  [o]手开 [c]手闭 [q]退出")
        self.log("=" * 50)

    def log(self, msg):
        self.get_logger().info(str(msg))

    # ===== 回调 =====
    def joint_cb(self, msg):
        self.latest_joint = msg

    def tcp_cb(self, msg):
        self.latest_tcp = msg

    # ===== ServoJ =====
    def _open_servoj(self):
        req = OpenServoJ.Request()
        req.vmax = [10.0]*7
        req.amax = [3000.0]*7
        req.jmax = [50000.0]*7
        try:
            f = self.servoj_srv.call_async(req)
            rclpy.spin_until_future_complete(self, f, timeout_sec=3)
            ok = f.done() and f.result() and f.result().success
            self.log(f"ServoJ: {'OK' if ok else 'FAIL'}")
        except: pass

    def servoj_timer_cb(self):
        if not self.running: return
        if not self.servoj_init and self.latest_joint:
            p = [v for v in self.latest_joint.position]
            self.servoj_target = (p+[0.]*7)[:7]
            self.servoj_init = True
        if not self.servoj_init: return

        if self.servoj_moving:
            if self.servoj_idx < len(self.servoj_traj):
                self.servoj_target = self.servoj_traj[self.servoj_idx]
                self.servoj_idx += 1
                if self.servoj_final and max(abs(self.servoj_target[i]-self.servoj_final[i]) for i in range(7))<0.5:
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
            js.position = [v for v in self.servoj_target[:len(js.name)]]
            self.js_pub.publish(js)

    def _ensure_servoj(self):
        if self._servoj_active:
            return True
        self.log("激活 ServoJ...")
        self.servoj_srv.wait_for_service(timeout_sec=5.0)
        self._open_servoj()
        t0 = time.time()
        while self.latest_joint is None and time.time()-t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint is None:
            self.log("关节数据超时")
            return False
        p = [v for v in self.latest_joint.position]
        self.servoj_target = (p+[0.]*7)[:7]
        self.servoj_init = True
        self._servoj_active = True
        self.log("ServoJ 就绪")
        return True

    def movej(self, joints):
        if not self._ensure_servoj():
            return False
        if not self.servoj_init: return False
        tgt = list(joints)+[0.]*(7-len(joints)); tgt = tgt[:7]
        start = list(self.servoj_target)
        n = self.servoj_interp_points
        self.servoj_traj = [[start[i]+(tgt[i]-start[i])*s/n for i in range(7)] for s in range(1,n+1)]
        self.servoj_final = tgt
        self._last_target = list(tgt)
        self.servoj_idx = 0
        self.servoj_moving = True
        return True

    def wait_arrival(self, t=30):
        """等轨迹发完 + 等实际关节到位 (误差<1度)"""
        # 先等 servoj_moving 结束
        t0 = time.time()
        while self.servoj_moving and time.time()-t0 < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        # 用 _last_target 等实际关节到位
        if self._last_target:
            tgt = self._last_target
            t0 = time.time()
            while time.time()-t0 < t:
                if self.latest_joint:
                    actual = [v for v in self.latest_joint.position]
                    err = max(abs(actual[i]-tgt[i]) for i in range(min(6, len(tgt))))
                    if err < 1.0:
                        break
                rclpy.spin_once(self, timeout_sec=0.05)
        return True

    # ===== 灵巧手 =====
    def _hand_write(self, addr, data):
        req = ModbusWrite.Request()
        req.master_id = 1; req.addr = addr
        req.data = [int(v) for v in data]
        rtu = ModbusRTUParam()
        rtu.slave_id=2; rtu.port=2; rtu.baudrate=115200
        rtu.data_bit=8; rtu.stop_bit=1; rtu.check_bit="None"
        p = ModbusMasterParam(); p.type="RTU"; p.start_addr=True; p.rtu=rtu
        req.master_param = p
        try:
            f = self.modbus.call_async(req)
            rclpy.spin_until_future_complete(self, f, timeout_sec=3)
            return f.done() and f.result() and f.result().success
        except: return False

    def _hand_init(self):
        self._hand_write(1008, [1]); time.sleep(0.2)
        self._hand_write(1013, [1]); time.sleep(2)
        self.hand_init = True
        self.log("灵巧手就绪")

    def hand_open(self):
        self._hand_write(1135, self.hand_open_val)

    def hand_close(self):
        self._hand_write(1135, self.hand_close_val)

    # ===== 键盘 =====
    def setup_kb(self):
        try:
            self.kb_fd = sys.stdin.fileno() if sys.stdin.isatty() else os.open('/dev/tty', os.O_RDONLY|os.O_NONBLOCK)
            self.kb_old = termios.tcgetattr(self.kb_fd)
            tty.setcbreak(self.kb_fd)
        except: self.kb_fd = None

    def read_key(self):
        # 优先从终端读取（支持 headless / SSH / NoMachine）
        if self.kb_fd is not None:
            try:
                dr, _, _ = select.select([self.kb_fd], [], [], 0)
                if dr:
                    return os.read(self.kb_fd, 1).decode(errors='ignore').lower()
            except: pass
        # 回退：从 OpenCV 窗口读取（有显示器时可用）
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q') or k == ord('Q'): return 'q'
        if k == ord('z') or k == ord('Z'): return 'z'
        if k == ord('g') or k == ord('G'): return 'g'
        if k == ord('o') or k == ord('O'): return 'o'
        if k == ord('c') or k == ord('C'): return 'c'
        if k == ord('t') or k == ord('T'): return 't'
        return None

    # ===== 检测 =====
    def _cam_thread(self):
        """独立线程持续取D435帧"""
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

    def detect_timer_cb(self):
        if not self.running: return
        with self._frame_lock:
            color_np = self._color_np
            depth_np = self._depth_np
        if color_np is None: return

        display = color_np.copy()
        try:
            results = self.model(color_np, conf=0.5, device=0, verbose=False)
            for r in results:
                for box in (r.boxes or []):
                    x1,y1,x2,y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cx,cy = (x1+x2)//2, (y1+y2)//2
                    cx = max(0,min(cx,639)); cy = max(0,min(cy,479))
                    rgn = depth_np[max(0,cy-5):min(479,cy+5),max(0,cx-5):min(639,cx+5)]
                    z = np.median(rgn[rgn>0])/1000. if rgn[rgn>0].size>0 else 0
                    if z<=0: continue
                    x = (cx-self.cx)*z/self.fx
                    y = (cy-self.cy)*z/self.fy
                    self.detected = True
                    self._cam_obj = (x, y, z, r.names[int(box.cls[0])])
                    # 同步快照TCP（检测时刻位姿），避免帧间TCP跳变导致基座坐标乱跳
                    if self.latest_tcp:
                        self._cam_tcp = (
                            self.latest_tcp.position.x / 1000.,
                            self.latest_tcp.position.y / 1000.,
                            self.latest_tcp.position.z / 1000.,
                            self.latest_tcp.rpy.x,
                            self.latest_tcp.rpy.y,
                            self.latest_tcp.rpy.z,
                        )
                    cv2.rectangle(display, (x1,y1), (x2,y2), (0,255,0), 2)
                    cv2.putText(display, f"{r.names[int(box.cls[0])]} {z:.2f}m", (x1,y1-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            cv2.imshow("D435 Detection", display)
        except Exception as e:
            if not hasattr(self, '_cam_err_cnt'): self._cam_err_cnt = 0
            self._cam_err_cnt += 1
            if self._cam_err_cnt % 30 == 1:
                self.log(f"检测错误: {e}")

    # ===== 坐标变换 =====
    def cam_to_base(self, cam_xyz, tcp_data=None):
        """相机系 → 基座系 (eye-in-hand)
        tcp_data: (x_m, y_m, z_m, rx_rad, ry_rad, rz_rad) 检测时刻快照, 不传则用实时TCP
        """
        if tcp_data is not None:
            px, py, pz, rx, ry, rz = tcp_data
        elif self.latest_tcp:
            tcp = self.latest_tcp
            px = tcp.position.x / 1000.
            py = tcp.position.y / 1000.
            pz = tcp.position.z / 1000.
            rx, ry, rz = tcp.rpy.x, tcp.rpy.y, tcp.rpy.z
        else:
            return None
        pos = np.array([px, py, pz])
        rpy = np.array([rx, ry, rz])
        R_t = R.from_euler('XYZ', rpy).as_matrix()  # intrinsic XYZ, 匹配ROS2 rpy
        T_base_tool = np.eye(4)
        T_base_tool[:3,:3] = R_t
        T_base_tool[:3,3] = pos
        T_base_cam = T_base_tool @ self.T_tool_camera
        P_cam = np.array([*cam_xyz, 1.0])
        P_base = T_base_cam @ P_cam
        return P_base[:3]*1000

    # ===== 抓取 =====
    def grasp(self):
        if not hasattr(self, '_cam_obj'):
            self.log("无检测目标")
            return

        x, y, z, name = self._cam_obj
        self.log(f"目标: {name} @ [{x:.2f},{y:.2f},{z:.2f}]m")

        base_xyz = self.cam_to_base((x,y,z),
            tcp_data=getattr(self, '_cam_tcp', None))
        if base_xyz is None:
            self.log("坐标变换失败")
            return
        self.log(f"基座坐标: [{base_xyz[0]:.0f},{base_xyz[1]:.0f},{base_xyz[2]:.0f}]mm")

        # Z轴安全钳位: 目标低于工作区(60mm)则拒绝
        if base_xyz[2] < 60.0:
            self.log(f"目标Z={base_xyz[2]:.0f}mm 低于安全工作区(60mm), 跳过抓取")
            return

        # 接近 → 下降 → 抓取 → 抬起
        bx, by, bz = base_xyz[0], base_xyz[1], base_xyz[2]
        # 用当前姿态而非硬编码, 避免IK奇异
        GRASP_R = self.latest_tcp.rpy.x
        GRASP_P = self.latest_tcp.rpy.y
        GRASP_Y = self.latest_tcp.rpy.z

        approach_z = bz + 100  # 物体上方100mm接近
        grasp_z = bz + 30      # 物体上方30mm抓取
        for step_name, step_z in [("接近", approach_z), ("下降", grasp_z)]:
            self.log(f"   {step_name} Z={step_z:.0f}mm")
            if not self.movej_cartesian(p := [bx, by, step_z, GRASP_R, GRASP_P, GRASP_Y]):
                self.log(f"  {step_name} IK失败!")
                return
            self.wait_arrival(15)

        self.hand_open(); time.sleep(0.5)
        self.hand_close(); time.sleep(2)
        self.log("抓取完成")

        # 抬起
        self.movej_cartesian([bx, by, approach_z, GRASP_R, GRASP_P, GRASP_Y])
        self.wait_arrival(15)
        self.movej(self.zero_joint)
        self.wait_arrival(15)
        self.log("归位完成")

        # 保存训练数据
        self._save_episode(name)

    def movej_cartesian(self, cart):
        """笛卡尔坐标 → 关节坐标 → ServoJ"""
        from tl_ros2_interface.srv import CoordTransform
        if not hasattr(self, '_ik_client'):
            self._ik_client = self.create_client(CoordTransform, '/tl_driver/coord_transform')
            self._ik_client.wait_for_service(timeout_sec=5)

        req = CoordTransform.Request()
        req.origin_coord = 1
        req.target_coord = 0
        req.form = 0
        req.origin_pos = list(cart) + [0.]*(7-len(cart))
        req.origin_pos = req.origin_pos[:7]
        req.reference_pos = [0.]*7

        f = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5)
        if f.done() and f.result() and f.result().success:
            joints = list(f.result().target_pos)
            return self.movej(joints)
        return False

    def _save_episode(self, obj_name):
        try:
            import h5py
            with self._frame_lock:
                img = self._color_np.copy() if self._color_np is not None else None
            if img is None or self.latest_joint is None:
                return
            img_small = cv2.resize(img, (224, 224))
            joints = [v for v in self.latest_joint.position][:6]
            path = f'{self._data_dir}/episode_{self._episode_idx:04d}.hdf5'
            import h5py as _h5
            with _h5.File(path, 'w') as f:
                f.create_dataset('image', data=img_small)
                f.create_dataset('joints', data=np.array(joints, dtype=np.float32))
                f.create_dataset('object_name', data=obj_name.encode())
            self._episode_idx += 1
            self.log(f"数据已保存: {path}")
        except Exception as e:
            self.log(f"保存失败: {e}")

    # ===== 按键 =====
    def handle_key(self, k):
        if k=='z': self.movej(self.zero_joint)
        elif k=='g': self.grasp()
        elif k=='o': self.hand_open()
        elif k=='c': self.hand_close()
        elif k=='t': self._test_transform()
        elif k=='q': self.running=False

    def _test_transform(self):
        """诊断: 把TCP移到检测到的物体位置, 看偏移多少"""
        if not hasattr(self, '_cam_obj'):
            self.log("无检测目标")
            return
        x, y, z, name = self._cam_obj
        base_xyz = self.cam_to_base((x,y,z),
            tcp_data=getattr(self, '_cam_tcp', None))
        if base_xyz is None: return
        self.log(f"测试: 物体@{name} 相机({x:.2f},{y:.2f},{z:.2f})m → 基座({base_xyz[0]:.0f},{base_xyz[1]:.0f},{base_xyz[2]:.0f})mm")
        self.log(f"当前TCP: ({self.latest_tcp.position.x:.0f},{self.latest_tcp.position.y:.0f},{self.latest_tcp.position.z:.0f})mm")
        # 用当前TCP姿态，避免硬编码导致姿态异常
        GRASP_R = self.latest_tcp.rpy.x
        GRASP_P = self.latest_tcp.rpy.y
        GRASP_Y = self.latest_tcp.rpy.z
        p = [base_xyz[0], base_xyz[1], base_xyz[2]+100, GRASP_R, GRASP_P, GRASP_Y]
        self.log(f"移到目标上方...")
        self.movej_cartesian(p)
        self.wait_arrival(20)
        self.log("到位, 观察夹爪和物体的偏移")

    def run(self):
        while self.running:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
                k = self.read_key()
                if k: self.handle_key(k)
            except KeyboardInterrupt:
                self.running = False
        self.log("退出")
        self._cam_running = False
        try: termios.tcsetattr(self.kb_fd, termios.TCSADRAIN, self.kb_old)
        except: pass
        try: self.pipe.stop()
        except: pass
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = VisionGraspDemo()
    signal.signal(signal.SIGINT, lambda *_: setattr(node,'running',False))
    node.run()


if __name__ == '__main__':
    main()
