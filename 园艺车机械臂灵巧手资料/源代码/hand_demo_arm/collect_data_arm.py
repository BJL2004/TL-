#!/usr/bin/env python3
"""
ARM版 手眼标定数据采集 — 适配原 compute_in_hand.py
输出格式: eye_hand_data/<timestamp>/1.jpg, 2.jpg ... + poses.txt
"""

import os, time, numpy as np, cv2, pyrealsense2 as rs, rclpy
from rclpy.node import Node
from tl_ros2_interface.msg import CartesianPose
from datetime import datetime


class CollectData(Node):
    def __init__(self):
        super().__init__('collect_data')

        # 创建输出目录
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.dir = os.path.expanduser(f'~/eye_hand_data/{ts}')
        os.makedirs(self.dir, exist_ok=True)
        self.pose_file = os.path.join(self.dir, 'poses.txt')
        open(self.pose_file, 'w').close()  # 清空
        self.count = 1

        # D435
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipe.start(cfg)

        # 切换到Base坐标系 (官方collect_data.py第180行)
        import socket as _sock
        try:
            s = _sock.socket(_sock.AF_INET, _sock.SOCK_STREAM)
            s.settimeout(3)
            s.connect(('192.168.1.13', 6001))
            s.send(b'{"command":"set_change_work_frame","frame_name":"Base"}')
            resp = s.recv(1024)
            s.close()
            print(f"坐标系已切换到 Base: {resp.decode()}")
        except Exception as e:
            print(f"切换坐标系失败: {e} (不影响采集,继续...)")

        # TCP位姿
        self.latest_tcp = None
        self.create_subscription(CartesianPose, '/tcp_pose', self.tcp_cb, 10)

        print("=" * 50)
        print(f"  手眼标定数据采集 (ARM)")
        print(f"  存储: {self.dir}")
        print("  [s] 保存  [q] 退出")
        print("  需要15-20组, 每次绕末端轴旋转>30度")
        print("=" * 50)

    def tcp_cb(self, m):
        self.latest_tcp = m

    def save(self, img):
        if self.latest_tcp is None:
            print("TCP位姿未收到")
            return
        tcp = self.latest_tcp

        # 保存图像
        img_path = os.path.join(self.dir, f'{self.count}.jpg')
        cv2.imwrite(img_path, img)

        # 保存位姿 (mm → m)
        x, y, z = tcp.position.x/1000.0, tcp.position.y/1000.0, tcp.position.z/1000.0
        with open(self.pose_file, 'a') as f:
            f.write(f'{x},{y},{z},{tcp.rpy.x},{tcp.rpy.y},{tcp.rpy.z}\n')

        self.count += 1
        print(f"[{self.count-1}] 保存 x={x:.4f} y={y:.4f} z={z:.4f}")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                frames = self.pipe.wait_for_frames()
            except RuntimeError:
                print("D435断连, 重新初始化...")
                self.pipe.stop()
                import time as _t; _t.sleep(1)
                cfg = rs.config()
                cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                self.pipe.start(cfg)
                continue
            color = frames.get_color_frame()
            if not color: continue
            img = np.asanyarray(color.get_data())
            display = cv2.resize(img, (960, 540))
            cv2.putText(display, f"Count: {self.count-1}", (10,30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.imshow("Collect | s=save q=quit", display)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('s'): self.save(img)
            elif k == ord('q'): break

        cv2.destroyAllWindows()
        self.pipe.stop()
        print(f"共采集{self.count-1}组 → {self.dir}")
        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    CollectData().run()
