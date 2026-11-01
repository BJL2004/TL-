#!/usr/bin/env python3
"""
ARM版 机械臂+灵巧手 联合控制 Demo
基于 hand_demo/control_node.py 裁剪，16GB Jetson 专用

运行:
  source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
  python3 arm_hand_demo.py
"""

import signal
import os
import sys
import time
import math
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam
from tl_ros2_interface.srv import ModbusWrite, OpenServoJ
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class ArmHandDemo(Node):
    def __init__(self):
        super().__init__('arm_hand_demo')

        # ===== 参数 =====
        self.declare_parameter('zero_joint', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('servoj_interp_points', 500)
        self.declare_parameter('servoj_vmax', [10.0]*7)
        self.declare_parameter('servoj_amax', [3000.0]*7)
        self.declare_parameter('servoj_jmax', [50000.0]*7)

        self.declare_parameter('hand_open', [0, 0, 0, 0, 0, 0])
        self.declare_parameter('hand_close', [21000, 20000, 23000, 60000, 60000, 60000])
        self.declare_parameter('hand_pinch', [20000, 20000, 0, 0, 0, 30000])
        self.declare_parameter('hand_fist', [60000, 60000, 60000, 60000, 60000, 30000])

        self.zero_joint = list(self.get_parameter('zero_joint').value)
        self.servoj_interp_points = int(self.get_parameter('servoj_interp_points').value)
        self.servoj_vmax = list(self.get_parameter('servoj_vmax').value)
        self.servoj_amax = list(self.get_parameter('servoj_amax').value)
        self.servoj_jmax = list(self.get_parameter('servoj_jmax').value)
        self.hand_open_val = list(self.get_parameter('hand_open').value)
        self.hand_close_val = list(self.get_parameter('hand_close').value)
        self.hand_pinch_val = list(self.get_parameter('hand_pinch').value)
        self.hand_fist_val = list(self.get_parameter('hand_fist').value)

        # ===== 状态 =====
        self.running = True
        self.latest_joint_state = None
        self.servoj_current_target = [0.0] * 7
        self.servoj_initialized = False
        self.servoj_current_final = None
        self.servoj_trajectory = []
        self.servoj_traj_idx = 0
        self.servoj_moving = False

        # ===== 服务客户端 =====
        self.modbus_client = self.create_client(ModbusWrite, '/tl_driver/modbus_write')
        self.open_servoj_client = self.create_client(OpenServoJ, '/tl_driver/open_servoj')

        # ===== 话题 =====
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.servoj_pos_pub = self.create_publisher(Float64MultiArray, '/tl_driver/set_servoj_pos', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.servoj_timer = self.create_timer(0.01, self.servoj_timer_cb)

        # ===== 键盘 =====
        self.setup_keyboard()

        # ===== 初始化 =====
        self.modbus_client.wait_for_service(timeout_sec=10.0)
        self._hand_start_init()

        self._servoj_active = False  # 延迟激活标记

        self.log("=" * 50)
        self.log("  机械臂 + 灵巧手 联合控制 Demo")
        self.log("=" * 50)
        self.log("  [z]回零  [1-4]预设关节")
        self.log("  [o]手张开  [c]手闭合  [p]捏合  [f]拳")
        self.log("  [5]自动序列")
        self.log("  [q]退出")

    def log(self, msg):
        self.get_logger().info(str(msg))

    # ===== 键盘 =====
    def setup_keyboard(self):
        try:
            if sys.stdin.isatty():
                self.kb_fd = sys.stdin.fileno()
            else:
                self.kb_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            self.kb_old = termios.tcgetattr(self.kb_fd)
            tty.setcbreak(self.kb_fd)
        except:
            self.kb_fd = None

    def read_key(self):
        if self.kb_fd is None:
            return None
        try:
            dr, _, _ = select.select([self.kb_fd], [], [], 0)
            if dr:
                return os.read(self.kb_fd, 1).decode(errors='ignore')
        except:
            pass
        return None

    # ===== 话题回调 =====
    def joint_cb(self, msg):
        self.latest_joint_state = msg

    # ===== ServoJ =====
    def _open_servoj(self):
        req = OpenServoJ.Request()
        req.vmax = self.servoj_vmax
        req.amax = self.servoj_amax
        req.jmax = self.servoj_jmax
        try:
            future = self.open_servoj_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            ok = future.done() and future.result() and future.result().success
            self.log(f"ServoJ: {'OK' if ok else 'FAIL'}")
        except Exception as e:
            self.log(f"ServoJ 异常: {e}")

    def servoj_timer_cb(self):
        if not self.running:
            return
        if not self.servoj_initialized and self.latest_joint_state:
            pos = [v for v in self.latest_joint_state.position]
            self.servoj_current_target = (pos + [0.0]*7)[:7]
            self.servoj_initialized = True
        if not self.servoj_initialized:
            return

        if self.servoj_moving:
            if self.servoj_traj_idx < len(self.servoj_trajectory):
                self.servoj_current_target = self.servoj_trajectory[self.servoj_traj_idx]
                self.servoj_traj_idx += 1
                if self.servoj_current_final and max(abs(self.servoj_current_target[i]-self.servoj_current_final[i])for i in range(7))<0.5:
                    self.servoj_current_target = list(self.servoj_current_final)
                    self.servoj_moving = False
                    self.servoj_current_final = None
            else:
                self.servoj_moving = False
                self.servoj_current_final = None

        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.servoj_current_target]
        self.servoj_pos_pub.publish(msg)

        if self.latest_joint_state:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self.latest_joint_state.name)
            js.position = [v for v in self.servoj_current_target[:len(js.name)]]
            self.joint_state_pub.publish(js)

    def _ensure_servoj(self):
        """首次按键时激活 ServoJ"""
        if self._servoj_active:
            return True
        self.log("激活 ServoJ...")
        self.open_servoj_client.wait_for_service(timeout_sec=5.0)
        self._open_servoj()
        # 等待关节数据
        t0 = time.time()
        while self.latest_joint_state is None and time.time()-t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint_state is None:
            self.log("关节数据超时")
            return False
        pos = [v for v in self.latest_joint_state.position]
        self.servoj_current_target = (pos + [0.0]*7)[:7]
        self.servoj_initialized = True
        self._servoj_active = True
        self.log("ServoJ 就绪")
        return True

    def movej(self, joint_pos):
        if not self._ensure_servoj():
            return False
        target = list(joint_pos) + [0.0]*(7-len(joint_pos))
        target = target[:7]
        start = list(self.servoj_current_target)
        n = self.servoj_interp_points
        self.servoj_trajectory = []
        for step in range(1, n+1):
            ratio = step / n
            self.servoj_trajectory.append([start[i]+(target[i]-start[i])*ratio for i in range(7)])
        self.servoj_current_final = target
        self.servoj_traj_idx = 0
        self.servoj_moving = True
        return True

    def wait_arrival(self, timeout=30.0):
        t0 = time.time()
        while self.servoj_moving and time.time()-t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return not self.servoj_moving

    # ===== 灵巧手 ModBus =====
    def _hand_write(self, addr, data):
        req = ModbusWrite.Request()
        req.master_id = 1
        req.addr = addr
        req.data = [int(v) for v in data]

        rtu = ModbusRTUParam()
        rtu.slave_id = 2
        rtu.port = 2
        rtu.baudrate = 115200
        rtu.data_bit = 8
        rtu.stop_bit = 1
        rtu.check_bit = "None"

        param = ModbusMasterParam()
        param.type = "RTU"
        param.start_addr = True
        param.rtu = rtu
        req.master_param = param

        try:
            future = self.modbus_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            ok = future.done() and future.result() and future.result().success
            if not ok:
                self.log(f"[ERR] ModBus写 {addr} 失败")
            return ok
        except Exception as e:
            self.log(f"[ERR] ModBus写 {addr}: {e}")
            return False

    def _hand_start_init(self):
        self.log("灵巧手初始化 SELF_TEST=1...")
        self._hand_write(1008, [1])
        time.sleep(0.2)
        self.log("灵巧手初始化 START_INIT=1...")
        self._hand_write(1013, [1])
        time.sleep(2.0)
        self.log("灵巧手就绪")

    # ===== 命令 =====
    def cmd_zero(self):
        self.log("→ 回零")
        self.movej(self.zero_joint)

    def cmd_joint(self, joints, name):
        self.log(f"→ MoveJ {name}")
        self.movej(joints)

    def cmd_hand(self, vals, name):
        self.log(f"→ 灵巧手 {name}")
        self._hand_write(1135, vals)

    # ===== 按键 =====
    def handle_key(self, key):
        presets = {
            '1': ([30, -20, 0, 0, 0, 0], "pos1"),
            '2': ([-30, 20, 0, 0, 0, 0], "pos2"),
            '3': ([0, 0, 30, 0, 0, 0], "pos3"),
            '4': ([0, 0, -30, 0, 0, 0], "pos4"),
        }
        if key == 'z':
            self.cmd_zero()
        elif key in presets:
            j, n = presets[key]
            self.cmd_joint(j, n)
        elif key == 'o':
            self.cmd_hand(self.hand_open_val, "张开")
        elif key == 'c':
            self.cmd_hand(self.hand_close_val, "闭合")
        elif key == 'p':
            self.cmd_hand(self.hand_pinch_val, "捏合")
        elif key == 'f':
            self.cmd_hand(self.hand_fist_val, "握拳")
        elif key == '5':
            self._auto_seq()
        elif key == 'q':
            self.running = False

    def _auto_seq(self):
        self.log("=== 自动: 零→拳→开→零 ===")
        self.cmd_zero(); self.wait_arrival(); time.sleep(1)
        self.cmd_hand(self.hand_fist_val, "拳"); time.sleep(2)
        self.cmd_hand(self.hand_open_val, "开"); time.sleep(2)
        self.cmd_zero(); self.wait_arrival()
        self.log("=== 完成 ===")

    # ===== 主循环 =====
    def run(self):
        while self.running:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
                key = self.read_key()
                if key:
                    self.handle_key(key)
            except KeyboardInterrupt:
                self.running = False
        self.log("退出")
        try: termios.tcsetattr(self.kb_fd, termios.TCSADRAIN, self.kb_old)
        except: pass
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ArmHandDemo()
    signal.signal(signal.SIGINT, lambda s,f: setattr(node, 'running', False))
    node.run()


if __name__ == '__main__':
    main()
