#!/usr/bin/env python3
"""手眼标定: 点动机械臂 + 采集图像+位姿 (一键脚本)"""

import os, sys, time, math, signal, select, termios, tty
import numpy as np, cv2
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tl_ros2_interface.msg import CartesianPose
from tl_ros2_interface.srv import OpenServoJ


class CalibCollectAndJog(Node):
    def __init__(self):
        super().__init__('calib_jog')
        self.data_dir = os.path.expanduser('~/calib_data')
        os.makedirs(self.data_dir, exist_ok=True)
        self.idx = len([f for f in os.listdir(self.data_dir) if f.startswith('img_')])

        # ServoJ
        self.latest_joint = None
        self.latest_tcp = None
        self.pos = [0.]*7
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(CartesianPose, '/tcp_pose', self.tcp_cb, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/tl_driver/set_servoj_pos', 10)
        self.srv = self.create_client(OpenServoJ, '/tl_driver/open_servoj')
        self.create_timer(0.01, self.timer_cb)

        # 激活ServoJ
        self.srv.wait_for_service(5)
        req = OpenServoJ.Request()
        req.vmax = [10.]*7; req.amax = [3000.]*7; req.jmax = [50000.]*7
        self.srv.call_async(req)
        t0 = time.time()
        while self.latest_joint is None and time.time()-t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint:
            self.pos = [v*180./math.pi for v in self.latest_joint.position]
            self.pos = (self.pos + [0.]*7)[:7]

        # D435
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipe.start(cfg)

        # 键盘
        try:
            fd = sys.stdin.fileno() if sys.stdin.isatty() else os.open('/dev/tty', os.O_RDONLY)
            self.old = termios.tcgetattr(fd); tty.setcbreak(fd); self.fd = fd
        except: self.fd = None

        print("=" * 50)
        print("  手眼标定采集")
        print(f"  已有 {self.idx} 组 | 存到 {self.data_dir}")
        print("  [1-6] J1-J6 正转  [Shift+1-6] 反转")
        print("  [s] 保存图像+位姿  [q] 退出")
        print("  棋盘格固定不动, 移动臂从15-20个角度拍照")
        print("=" * 50)

    def joint_cb(self, m): self.latest_joint = m
    def tcp_cb(self, m): self.latest_tcp = m

    def timer_cb(self):
        m = Float64MultiArray()
        m.data = [float(v) for v in self.pos]
        self.pub.publish(m)

    def read_key(self):
        if not self.fd: return None
        try:
            dr,_,_ = select.select([self.fd],[],[],0)
            return os.read(self.fd,1).decode(errors='ignore') if dr else None
        except: return None

    def save(self):
        frames = self.pipe.wait_for_frames()
        color = frames.get_color_frame()
        if not color or self.latest_tcp is None:
            print("无数据")
            return
        img = np.asanyarray(color.get_data())
        tcp = self.latest_tcp
        cv2.imwrite(f'{self.data_dir}/img_{self.idx:03d}.png', img)
        with open(f'{self.data_dir}/pose_{self.idx:03d}.txt', 'w') as f:
            f.write(f'{tcp.position.x},{tcp.position.y},{tcp.position.z},'
                    f'{tcp.rpy.x},{tcp.rpy.y},{tcp.rpy.z}\n')
        self.idx += 1
        print(f"[{self.idx}] 保存 x={tcp.position.x:.1f} y={tcp.position.y:.1f} z={tcp.position.z:.1f}")

    def run(self):
        inc = 5.0  # 每按一次移动5度
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            frames = self.pipe.wait_for_frames(timeout_ms=10)
            color = frames.get_color_frame()
            if color:
                img = cv2.resize(np.asanyarray(color.get_data()), (640, 360))
                cv2.putText(img, f"Saved: {self.idx}", (10,30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                cv2.imshow("Calib | 1-6 jog | s save | q quit", img)
            cv2.waitKey(1)

            k = self.read_key()
            if not k: continue
            if k == 'q': break
            if k == 's': self.save(); continue
            if k == 'z': self.pos = [0.]*7; print("回零"); continue
            # 数字键1-6 + Shift=负方向
            mapping = {'1':0,'!':0,'2':1,'@':1,'3':2,'#':2,'4':3,'$':3,'5':4,'%':4,'6':5,'^':5}
            if k in mapping:
                j = mapping[k]
                d = -inc if k in '!@#$%^' else inc
                self.pos[j] += d
                print(f"J{j+1}: {self.pos[j]:.1f}")

        cv2.destroyAllWindows(); self.pipe.stop()
        try: termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        except: pass
        self.destroy_node(); rclpy.shutdown()
        print(f"共采集 {self.idx} 组")


if __name__ == '__main__':
    rclpy.init()
    CalibCollectAndJog().run()
