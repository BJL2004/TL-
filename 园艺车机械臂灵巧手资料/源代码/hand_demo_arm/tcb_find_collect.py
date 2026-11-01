#!/usr/bin/env python3
"""
TCB605 找寻目标 VLA 数据采集（无灵巧手版）
==========================================
依赖: tl_driver 已启动 (ros2 launch tl_driver tl_driver.launch.py arm_type:=tcb605)
相机: D435 眼在手上, USB 3.0

运行:
  source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
  python3 tcb_find_collect.py

按键:
  [r] 开始录制   [f] 保存 episode   [x] 废弃   [q] 退出
  录制时按示教器 P 键进入拖拽模式, 手拖机械臂接近目标
"""

import signal
import os
import sys
import time
import json
import select
import termios
import tty
import threading
import glob as _glob

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tl_ros2_interface.msg import CartesianPose, MoveCommand
import pyrealsense2 as rs


class TcbFindCollect(Node):
    def __init__(self):
        super().__init__('tcb_find_collect')

        # ===== 参数 =====
        self.declare_parameter('observe_joints', [0.0, -28.5, -91.8, 0.0, -59.5, 0.0, 0.0])
        self.declare_parameter('record_fps', 8)
        self.declare_parameter('output_dir', os.path.expanduser('~/tcb_find_data/raw'))
        self.declare_parameter('task', '接近红色方块')

        self.observe_joints = [float(v) for v in self.get_parameter('observe_joints').value]
        self.record_fps = int(self.get_parameter('record_fps').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.task = str(self.get_parameter('task').value)

        os.makedirs(self.output_dir, exist_ok=True)

        # ===== 状态 =====
        self.latest_tcp = None
        self.latest_joint = None
        self.recording = False
        self.current_episode = []
        self._current_ep_dir = None
        self.episode_count = self._count_episodes()

        # ===== 订阅 =====
        self.create_subscription(CartesianPose, '/tcp_pose', self._tcp_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        # ===== 发布 (MoveJ 回观测位) =====
        self.movej_pub = self.create_publisher(MoveCommand, '/tl_driver/moveJ', 10)

        # ===== D435 相机 =====
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipe.start(cfg)
        self._frame_lock = threading.Lock()
        self._color_np = None
        self._cam_running = True
        threading.Thread(target=self._cam_thread, daemon=True).start()

        # ===== 键盘 =====
        self._setup_kb()

        # ===== 采集定时器 =====
        self.create_timer(1.0 / self.record_fps, self._record_tick)

        # ===== 启动提示 + 回观测位 =====
        self._log(f"采集就绪，已有 {self.episode_count} 条 episode")
        self._log("按键: [r]开始录制  [f]保存  [x]废弃  [q]退出")
        self._log("提示: 录制时按示教器 P 键进入拖拽模式")
        self._go_observe()

    def _log(self, msg):
        self.get_logger().info(str(msg))

    def _count_episodes(self):
        return len(_glob.glob(f'{self.output_dir}/episode_*'))

    # ===== 回调 =====
    def _tcp_cb(self, msg):
        self.latest_tcp = msg

    def _joint_cb(self, msg):
        self.latest_joint = msg

    # ===== 相机线程 =====
    def _cam_thread(self):
        while self._cam_running:
            try:
                frames = self.pipe.wait_for_frames(timeout_ms=50)
                color = frames.get_color_frame()
                if color:
                    with self._frame_lock:
                        self._color_np = np.asanyarray(color.get_data())
            except Exception:
                pass

    # ===== 键盘 =====
    def _setup_kb(self):
        try:
            self.kb_fd = sys.stdin.fileno() if sys.stdin.isatty() else os.open(
                '/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            self.kb_old = termios.tcgetattr(self.kb_fd)
            tty.setcbreak(self.kb_fd)
        except Exception:
            self.kb_fd = None

    def _read_key(self):
        if self.kb_fd is not None:
            try:
                dr, _, _ = select.select([self.kb_fd], [], [], 0)
                if dr:
                    return os.read(self.kb_fd, 1).decode(errors='ignore').lower()
            except Exception:
                pass
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            return 'q'
        if k == ord('r'):
            return 'r'
        if k == ord('f'):
            return 'f'
        if k == ord('x'):
            return 'x'
        return None

    # ===== MoveJ 回观测位 =====
    def _go_observe(self):
        msg = MoveCommand()
        # target_pos_value 14 维: 前6=关节角(度), 后8备用
        msg.target_pos_value = [float(v) for v in self.observe_joints[:7]] + [0.0] * 7
        msg.target_pos_name = ''
        msg.target_pos_type = 0
        msg.coord = 0  # 关节坐标系
        msg.velocity = 20.0
        msg.velocity_sync = 0.0
        msg.acc = 20.0
        msg.dec = 20.0
        msg.pl = 0
        msg.time = 0
        msg.tool_num = 0
        msg.user_num = 0
        msg.posidtype = 0
        msg.configuration = 0
        msg.spin = 0
        msg.para_sync = False
        self.movej_pub.publish(msg)
        self._log(f"MoveJ 回观测位: {self.observe_joints[:6]}")

    # ===== 采集循环 =====
    def _record_tick(self):
        if not self.recording:
            return
        if self.latest_tcp is None or self.latest_joint is None:
            return
        with self._frame_lock:
            color = self._color_np.copy() if self._color_np is not None else None
        if color is None:
            return

        frame_idx = len(self.current_episode)
        ts = time.time_ns()

        # 保存图像
        img_name = f'rgb_{frame_idx:06d}.jpg'
        img_path = os.path.join(self._current_ep_dir, 'rgb', img_name)
        cv2.imwrite(img_path, color)

        # 记录元数据
        tcp = self.latest_tcp
        joint = self.latest_joint
        record = {
            'frame_index': frame_idx,
            'timestamp_ns': ts,
            'rgb': f'rgb/{img_name}',
            'tcp_pose': [
                float(tcp.position.x), float(tcp.position.y), float(tcp.position.z),
                float(tcp.rpy.x), float(tcp.rpy.y), float(tcp.rpy.z),
            ],
            'joint_angle': [float(v) for v in list(joint.position)[:6]],
            'task': self.task,
            'event': 'start' if frame_idx == 0 else '',
        }
        self.current_episode.append(record)

    # ===== 录制控制 =====
    def _start_record(self):
        if self.recording:
            self._log("已在录制中")
            return
        ep_name = f'episode_{self.episode_count + 1:06d}'
        self._current_ep_dir = os.path.join(self.output_dir, ep_name)
        os.makedirs(os.path.join(self._current_ep_dir, 'rgb'), exist_ok=True)
        self.current_episode = []
        self.recording = True
        self._log(f"开始录制 {ep_name}（现在按示教器 P 键进入拖拽）")

    def _save_episode(self):
        if not self.recording or len(self.current_episode) < 2:
            self._log("录制数据不足，无法保存")
            self.recording = False
            return
        self.recording = False
        # 写 metadata.jsonl
        meta_path = os.path.join(self._current_ep_dir, 'metadata.jsonl')
        with open(meta_path, 'w', encoding='utf-8') as f:
            for r in self.current_episode:
                f.write(json.dumps(r, ensure_ascii=False) + '\n')
        # 写 episode.json
        ep_info = {
            'task': self.task,
            'frames': len(self.current_episode),
            'success': True,
            'create_time': time.strftime('%Y-%m-%d %H:%M:%S'),
        }
        with open(os.path.join(self._current_ep_dir, 'episode.json'), 'w',
                  encoding='utf-8') as f:
            json.dump(ep_info, f, ensure_ascii=False, indent=2)
        self.episode_count += 1
        self._log(f"保存成功: {self._current_ep_dir} ({len(self.current_episode)} 帧)")

    def _discard_episode(self):
        if not self.recording:
            return
        self.recording = False
        import shutil
        shutil.rmtree(self._current_ep_dir, ignore_errors=True)
        self._log("已废弃当前 episode")

    # ===== 主循环 =====
    def run(self):
        running = True
        while running:
            rclpy.spin_once(self, timeout_sec=0.05)
            k = self._read_key()
            if k == 'r':
                self._start_record()
            elif k == 'f':
                self._save_episode()
            elif k == 'x':
                self._discard_episode()
            elif k == 'q':
                running = False
            # 显示画面
            with self._frame_lock:
                color = self._color_np
            if color is not None:
                disp = color.copy()
                status = "RECORDING" if self.recording else "IDLE"
                color_txt = (0, 0, 255) if self.recording else (0, 255, 0)
                cv2.putText(disp, f"{status} | frames={len(self.current_episode)}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_txt, 2)
                cv2.putText(disp, "[r]rec [f]save [x]discard [q]quit",
                            (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                cv2.imshow("TcbFindCollect", disp)
        self._log("退出")
        self._cam_running = False
        time.sleep(0.2)
        try:
            termios.tcsetattr(self.kb_fd, termios.TCSADRAIN, self.kb_old)
        except Exception:
            pass
        self.pipe.stop()
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TcbFindCollect()
    signal.signal(signal.SIGINT, lambda *_: None)
    node.run()


if __name__ == '__main__':
    main()
