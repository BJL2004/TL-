#!/usr/bin/env python3
"""标定采集 v3 — 带位姿变化质量提示"""
import os, time, numpy as np, cv2, pyrealsense2 as rs, rclpy
from rclpy.node import Node
from tl_ros2_interface.msg import CartesianPose
from datetime import datetime

class Collector(Node):
    def __init__(self):
        super().__init__('cl')
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.dir = os.path.expanduser(f'~/eye_hand_data/{ts}')
        os.makedirs(self.dir, exist_ok=True)
        self.pf = open(os.path.join(self.dir,'poses.txt'),'w')
        self.cnt = 1
        self.last_rpy = None
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipe.start(cfg)
        self.tcp = None
        self.create_subscription(CartesianPose,'/tcp_pose',self.cb,10)
        print(f"存储: {self.dir}")
        print("每保存一组会显示与上一组的姿态差异(度)")
        print("要求: 每组绕末端轴向(X/Y/Z)变化>20deg")
        print("[s]保存 [q]退出")

    def cb(self, m): self.tcp = m

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            try: frames = self.pipe.wait_for_frames()
            except RuntimeError:
                print("D435断连重试..."); self.pipe.stop(); time.sleep(1)
                cfg=rs.config(); cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
                cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30); self.pipe.start(cfg)
                continue
            color = frames.get_color_frame()
            if not color: continue
            img = np.asanyarray(color.get_data())
            cv2.imshow("Calib", cv2.resize(img,(960,540)))
            k = cv2.waitKey(1) & 0xFF
            if k == ord('s'):
                if self.tcp is None:
                    print("无TCP数据"); continue
                t = self.tcp
                # 与上一组对比旋转变化
                cur_rpy = np.array([t.rpy.x, t.rpy.y, t.rpy.z])
                if self.last_rpy is not None:
                    diff = np.abs(cur_rpy - self.last_rpy) * 180 / np.pi
                    print(f"  姿态变化(deg): rx={diff[0]:.0f} ry={diff[1]:.0f} rz={diff[2]:.0f}")
                    if max(diff) < 15:
                        print("  ⚠ 变化太小! 建议绕末端轴旋转>20度")
                self.last_rpy = cur_rpy

                cv2.imwrite(f'{self.dir}/{self.cnt}.jpg', img)
                self.pf.write(f'{t.position.x/1000},{t.position.y/1000},{t.position.z/1000},{t.rpy.x},{t.rpy.y},{t.rpy.z}\n')
                print(f"[{self.cnt}] 保存 pos=({t.position.x:.0f},{t.position.y:.0f},{t.position.z:.0f})mm")
                self.cnt += 1
            elif k == ord('q'): break
        cv2.destroyAllWindows(); self.pipe.stop(); self.pf.close()
        print(f"共{self.cnt-1}组 → {self.dir}"); self.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init(); Collector().run()
