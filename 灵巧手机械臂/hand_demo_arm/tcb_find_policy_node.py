#!/usr/bin/env python3
"""
TCB605 找寻目标 VLA 执行节点
============================
ROS2 节点，订阅 D435 图像 + /tcp_pose，POST 到推理服务，
安全过滤后发布 /tl_driver/moveL 控制机械臂接近目标。

运行:
  # 终端1: 先启动推理服务
  python3 tcb_find_infer_server.py --model-path ~/tcb_find_models/checkpoints/last

  # 终端2: dry-run (只打印动作，不动机械臂)
  source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
  python3 tcb_find_policy_node.py --task "接近红色方块" --dry-run

  # 真实执行 (低速)
  python3 tcb_find_policy_node.py --task "接近红色方块" --execute --max-step-mm 10.0 --loop-hz 1.0

按键: [q]退出  [s]急停
"""

import signal
import os
import sys
import time
import base64
import argparse
import select
import termios
import tty
import threading

import numpy as np
import cv2
import requests

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tl_ros2_interface.msg import CartesianPose, MoveCommand
import pyrealsense2 as rs


class TcbFindPolicyNode(Node):
    def __init__(self, args):
        super().__init__('tcb_find_policy_node')

        self.args = args
        self.server_url = args.server_url
        self.task = args.task
        self.dry_run = args.dry_run
        self.max_step_mm = args.max_step_mm
        self.max_step_z = args.max_step_z
        self.min_z = args.min_z
        self.loop_hz = args.loop_hz
        self.max_steps = args.max_steps

        # 状态
        self.latest_tcp = None
        self.latest_joint = None
        self.running = True
        self.step_count = 0
        self.error_count = 0

        # 订阅
        self.create_subscription(CartesianPose, '/tcp_pose', self._tcp_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        # 发布 MoveL
        self.movel_pub = self.create_publisher(MoveCommand, '/tl_driver/moveL', 10)

        # D435
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipe.start(cfg)
        self._frame_lock = threading.Lock()
        self._color_np = None
        self._cam_running = True
        threading.Thread(target=self._cam_thread, daemon=True).start()

        # 键盘
        self._setup_kb()

        # 主循环定时器
        self.create_timer(1.0 / self.loop_hz, self._loop_tick)

        self._log(f"执行节点就绪 | mode={'DRY-RUN' if self.dry_run else 'EXECUTE'} | loop={self.loop_hz}Hz")
        self._log(f"安全: max_step={self.max_step_mm}mm max_z={self.max_step_z}mm min_z={self.min_z}mm max_steps={self.max_steps}")
        self._log(f"推理服务: {self.server_url}")
        self._log(f"任务: {self.task}")
        self._log("按 [q]退出  [s]急停")

    def _log(self, msg):
        self.get_logger().info(str(msg))

    def _tcp_cb(self, msg):
        self.latest_tcp = msg

    def _joint_cb(self, msg):
        self.latest_joint = msg

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

    def _setup_kb(self):
        try:
            self.kb_fd = (sys.stdin.fileno() if sys.stdin.isatty()
                          else os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK))
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
        return None

    def _safety_filter(self, action, current_tcp):
        """安全过滤: 限幅 + Z 轴保护"""
        dx, dy, dz = action
        # 限幅
        dx = max(-self.max_step_mm, min(self.max_step_mm, dx))
        dy = max(-self.max_step_mm, min(self.max_step_mm, dy))
        dz = max(-self.max_step_z, min(self.max_step_z, dz))

        # Z 轴下限保护 (防撞桌面)
        target_z = current_tcp.position.z + dz
        if target_z < self.min_z:
            dz = max(0.0, self.min_z - current_tcp.position.z)
            self._log(f"[安全] Z限制: target_z={target_z:.1f} < min_z={self.min_z}, dz→{dz:.1f}")

        return [dx, dy, dz]

    def _movel(self, target_xyz, target_rpy):
        """发布 /tl_driver/moveL 话题 (coord=1 直角系, 绝对位姿)"""
        msg = MoveCommand()
        # target_pos_value 14维: [x,y,z,rx,ry,rz, 备用*8]
        msg.target_pos_value = [
            float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2]),
            float(target_rpy[0]), float(target_rpy[1]), float(target_rpy[2]),
        ] + [0.0] * 8
        msg.target_pos_name = ''
        msg.target_pos_type = 0
        msg.coord = 1  # 直角坐标系
        msg.velocity = 15.0  # 15% 速度
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
        self.movel_pub.publish(msg)

    def _loop_tick(self):
        if not self.running:
            return
        if self.latest_tcp is None or self.latest_joint is None:
            return

        # 步数上限检查
        if self.step_count >= self.max_steps:
            self._log(f"[停止] 达到最大步数 {self.max_steps}")
            self.running = False
            return

        # 取图
        with self._frame_lock:
            color = self._color_np.copy() if self._color_np is not None else None
        if color is None:
            return

        # 编码图像 base64 (224x224)
        _, buf = cv2.imencode('.jpg', cv2.resize(color, (224, 224)))
        image_b64 = base64.b64encode(buf).decode()

        # 构造 state (12维: tcp_xyz + tcp_rpy + joint_6)
        tcp = self.latest_tcp
        joint = self.latest_joint
        state = [
            tcp.position.x, tcp.position.y, tcp.position.z,
            tcp.rpy.x, tcp.rpy.y, tcp.rpy.z,
        ] + [float(v) for v in list(joint.position)[:6]]

        # POST 推理服务
        try:
            resp = requests.post(self.server_url, json={
                'image': image_b64,
                'state': state,
                'task': self.task,
            }, timeout=5.0)
            resp.raise_for_status()
            result = resp.json()
            action = result['action']
            infer_time = result['inference_time']
            skeleton = result.get('skeleton', False)
        except Exception as e:
            self.error_count += 1
            self._log(f"[推理失败 {self.error_count}] {e}")
            if self.error_count >= 3:
                self._log("[停止] 连续3次推理失败")
                self.running = False
            return
        self.error_count = 0

        # 安全过滤
        action_safe = self._safety_filter(action, tcp)

        self._log(f"[step {self.step_count + 1}] action=[{action_safe[0]:.1f},"
                  f"{action_safe[1]:.1f},{action_safe[2]:.1f}]mm | "
                  f"infer={infer_time:.3f}s | skeleton={skeleton}")

        # 终止条件: action 接近0 (模型认为到位了)
        if (abs(action_safe[0]) < 0.5 and abs(action_safe[1]) < 0.5
                and abs(action_safe[2]) < 0.5):
            self._log("[成功] 动作接近0，判定找寻完成")
            self.running = False
            return

        if self.dry_run:
            self._log(f"  [dry-run] 不执行, target=["
                      f"{tcp.position.x + action_safe[0]:.1f},"
                      f"{tcp.position.y + action_safe[1]:.1f},"
                      f"{tcp.position.z + action_safe[2]:.1f}]")
        else:
            # 执行 MoveL (保持当前姿态)
            target_xyz = [
                tcp.position.x + action_safe[0],
                tcp.position.y + action_safe[1],
                tcp.position.z + action_safe[2],
            ]
            target_rpy = [tcp.rpy.x, tcp.rpy.y, tcp.rpy.z]
            self._movel(target_xyz, target_rpy)
            self._log(f"  [execute] MoveL → [{target_xyz[0]:.1f},"
                      f"{target_xyz[1]:.1f},{target_xyz[2]:.1f}]")

        self.step_count += 1

        # 显示画面
        disp = color.copy()
        mode_txt = 'DRY-RUN' if self.dry_run else 'EXECUTE'
        color_txt = (0, 255, 255) if self.dry_run else (0, 0, 255)
        cv2.putText(disp, f"{mode_txt} step={self.step_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_txt, 2)
        cv2.putText(disp, f"action=[{action_safe[0]:.1f},{action_safe[1]:.1f},{action_safe[2]:.1f}]",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow('TcbFindPolicy', disp)

    def run(self):
        while self.running:
            rclpy.spin_once(self, timeout_sec=0.05)
            k = self._read_key()
            if k == 'q':
                self.running = False
            elif k == 's':
                self._log('[急停]')
                self.running = False
            cv2.waitKey(1)
        self._log('退出')
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
    parser = argparse.ArgumentParser(description='TCB605 找寻目标 VLA 执行节点')
    parser.add_argument('--server-url', default='http://127.0.0.1:18080/predict')
    parser.add_argument('--task', default='接近红色方块')
    parser.add_argument('--dry-run', action='store_true', help='只打印不执行')
    parser.add_argument('--execute', action='store_true', help='真实执行')
    parser.add_argument('--max-step-mm', type=float, default=20.0)
    parser.add_argument('--max-step-z', type=float, default=10.0)
    parser.add_argument('--min-z', type=float, default=30.0)
    parser.add_argument('--loop-hz', type=float, default=1.0)
    parser.add_argument('--max-steps', type=int, default=30)
    args = parser.parse_args()

    if not args.dry_run and not args.execute:
        print('请指定 --dry-run 或 --execute')
        return

    rclpy.init()
    node = TcbFindPolicyNode(args)
    signal.signal(signal.SIGINT, lambda *_: setattr(node, 'running', False))
    node.run()


if __name__ == '__main__':
    main()
