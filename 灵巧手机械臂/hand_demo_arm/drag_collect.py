#!/usr/bin/env python3
"""
拖动示教数据采集
按 r=记录观测位  d=记录抓取位  s=保存一对  q=退出
存到 ~/drag_data/
"""

import os, time, numpy as np, cv2, h5py, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pyrealsense2 as rs
from datetime import datetime

DATA_DIR = os.path.expanduser('~/drag_data')

class DragCollect(Node):
    def __init__(self):
        super().__init__('drag')
        os.makedirs(DATA_DIR, exist_ok=True)
        existing = [f for f in os.listdir(DATA_DIR) if f.endswith('.hdf5')]
        self.idx = len(existing)

        self.latest_joint = None
        self.obs_joints = None
        self.grasp_joints = None
        self.obs_img = None

        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipe.start(cfg)

        print("=" * 50)
        print(f"  拖动示教数据采集")
        print(f"  已有 {self.idx} 条 | 存到 {DATA_DIR}")
        print("  [r] 记录观测位  [d] 记录抓取位  [s] 保存  [q] 退出")
        print("=" * 50)

    def joint_cb(self, m):
        self.latest_joint = m

    def get_joints(self):
        if self.latest_joint is None: return None
        return [v * 180.0 / np.pi for v in self.latest_joint.position][:6]

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            frames = self.pipe.wait_for_frames()
            color = frames.get_color_frame()
            img = np.asanyarray(color.get_data()) if color else None

            if img is not None:
                dsp = cv2.resize(img, (960, 540))
                status = []
                if self.obs_joints is not None:
                    j = [f"{v:.1f}" for v in self.obs_joints]
                    status.append(f"Obs: J=[{','.join(j)}]")
                if self.grasp_joints is not None:
                    j = [f"{v:.1f}" for v in self.grasp_joints]
                    status.append(f"Grasp: J=[{','.join(j)}]")
                if self.obs_joints is not None and self.grasp_joints is not None:
                    status.append("READY to save!")
                for i, s in enumerate(status):
                    cv2.putText(dsp, s, (10, 30 + i*25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("DragCollect", dsp)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('r'):
                self.obs_img = img.copy() if img is not None else None
                self.obs_joints = self.get_joints()
                print(f"[r] 观测位 : {[round(v,1) for v in self.obs_joints] if self.obs_joints else 'None'}")
            elif k == ord('d'):
                self.grasp_joints = self.get_joints()
                print(f"[d] 抓取位 : {[round(v,1) for v in self.grasp_joints] if self.grasp_joints else 'None'}")
            elif k == ord('s'):
                if self.obs_img is None or self.obs_joints is None or self.grasp_joints is None:
                    print("[s] 数据不完整: 需要先按 r 和 d")
                    continue
                img_small = cv2.resize(self.obs_img, (224, 224))
                path = os.path.join(DATA_DIR, f'ep_{self.idx:05d}.hdf5')
                with h5py.File(path, 'w') as f:
                    f.create_dataset('image', data=img_small)
                    f.create_dataset('obs_joints', data=np.array(self.obs_joints, dtype=np.float32))
                    f.create_dataset('grasp_joints', data=np.array(self.grasp_joints, dtype=np.float32))
                    f.create_dataset('timestamp', data=datetime.now().isoformat())
                self.idx += 1
                print(f"[s] 已保存 {path} (共{self.idx}条)")
                self.obs_img = None
                self.obs_joints = None
                self.grasp_joints = None
            elif k == ord('q'):
                break

        cv2.destroyAllWindows()
        self.pipe.stop()
        self.destroy_node()
        rclpy.shutdown()
        print(f"共采集 {self.idx} 条")


if __name__ == '__main__':
    rclpy.init()
    DragCollect().run()
