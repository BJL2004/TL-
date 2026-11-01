#!/usr/bin/env python3
"""
顺序点位 + 灵巧手 联合控制程序
0 → J1=90 → 目标位(握拳) → J1=90 → 归零(张开)

运行:
  cd ~/Desktop/hand_demo_arm
  source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
  python3 seq_move.py
"""

import rclpy, time, math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tl_ros2_interface.srv import OpenServoJ, ModbusWrite
from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam


# 目标点位（J6 已损坏，程序中锁定 J6 不驱动）
POS_ZERO   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POS_J1_90  = [90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POS_TARGET = [90.391, -55.323, 36.901, -4.723, -20.477, 0.0]

# 灵巧手手势 (ROH-LiteS001 POS_TARGET 寄存器 1135-1140)
HAND_OPEN = [0, 0, 0, 0, 0, 0]
HAND_FIST = [60000, 60000, 60000, 60000, 60000, 30000]


class SeqMove(Node):
    def __init__(self):
        super().__init__('seq_move')
        self.latest_joint = None

        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.servoj_pub = self.create_publisher(Float64MultiArray, '/tl_driver/set_servoj_pos', 10)

        # 激活 ServoJ
        self.servoj_srv = self.create_client(OpenServoJ, '/tl_driver/open_servoj')
        self.servoj_srv.wait_for_service(5)
        req = OpenServoJ.Request()
        req.vmax = [10.0] * 7
        req.amax = [3000.0] * 7
        req.jmax = [50000.0] * 7
        self.servoj_srv.call_async(req)

        # ModBus 客户端（灵巧手）
        self.modbus = self.create_client(ModbusWrite, '/tl_driver/modbus_write')
        self.modbus.wait_for_service(timeout_sec=10)

        # 等待关节数据
        t0 = time.time()
        while self.latest_joint is None and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint is None:
            self.get_logger().error("无法获取关节状态，退出")
            raise RuntimeError("no joint states")

        self.current_pos = [v for v in self.latest_joint.position]
        self.current_pos = (self.current_pos + [0.0] * 7)[:7]
        self.j6_locked = self.current_pos[5]
        self.get_logger().info(f"J6 已锁定: {self.j6_locked:.2f}°")

        # 初始化灵巧手
        self._hand_init()

        # 轨迹插值
        self.traj = []
        self.traj_idx = 0
        self.moving = False
        self.create_timer(0.01, self.servoj_timer_cb)

        self.get_logger().info("SeqMove 就绪")

    def joint_cb(self, msg):
        self.latest_joint = msg

    def servoj_timer_cb(self):
        if self.moving and self.traj_idx < len(self.traj):
            self.current_pos = self.traj[self.traj_idx]
            self.traj_idx += 1
        elif self.moving:
            self.moving = False

        out = list(self.current_pos)
        out[5] = self.j6_locked
        msg = Float64MultiArray()
        msg.data = [float(v) for v in out]
        self.servoj_pub.publish(msg)

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
            f = self.modbus.call_async(req)
            rclpy.spin_until_future_complete(self, f, timeout_sec=3)
            ok = f.done() and f.result() and f.result().success
            if not ok:
                self.get_logger().error(f"ModBus写 {addr} 失败")
            return ok
        except Exception as e:
            self.get_logger().error(f"ModBus写 {addr} 异常: {e}")
            return False

    def _hand_init(self):
        self.get_logger().info("灵巧手初始化 SELF_TEST=1...")
        self._hand_write(1008, [1])
        time.sleep(0.2)
        self.get_logger().info("灵巧手初始化 START_INIT=1...")
        self._hand_write(1013, [1])
        time.sleep(2.0)
        self.get_logger().info("灵巧手就绪")

    def hand_fist(self):
        self.get_logger().info("→ 灵巧手 握拳")
        self._hand_write(1135, HAND_FIST)

    def hand_open(self):
        self.get_logger().info("→ 灵巧手 张开")
        self._hand_write(1135, HAND_OPEN)

    # ===== 机械臂 =====
    def movej(self, target, name="", interp_points=300):
        tgt = list(target) + [0.0] * (7 - len(target))
        tgt = tgt[:7]
        tgt[5] = self.j6_locked

        start = list(self.current_pos)
        n = interp_points
        self.traj = [
            [start[i] + (tgt[i] - start[i]) * s / n for i in range(7)]
            for s in range(1, n + 1)
        ]
        self.traj_idx = 0
        self.traj[-1] = list(tgt)
        self.moving = True

        self.get_logger().info(f"→ {name}: {[round(v,2) for v in tgt[:6]]}")

        while self.moving:
            rclpy.spin_once(self, timeout_sec=0.1)

        t0 = time.time()
        while self.latest_joint and time.time() - t0 < 15:
            actual = [v for v in self.latest_joint.position]
            err = max(abs(actual[i] - tgt[i]) for i in range(5))
            if err < 1.0:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        time.sleep(0.3)

        if self.latest_joint:
            j = [round(v, 3) for v in self.latest_joint.position]
            self.get_logger().info(f"    实际: {j[:6]}")

    def log_current(self, tag):
        if not self.latest_joint:
            return
        j = [round(v, 3) for v in self.latest_joint.position]
        self.get_logger().info(f"  [{tag}] 关节: {j[:6]}")


def main():
    rclpy.init()
    node = SeqMove()

    node.log_current("启动时")

    # 1. 归零（手张开）
    node.hand_open()
    node.movej(POS_ZERO, "归零")
    node.log_current("零位")

    # 2. J1 转到 90°
    node.movej(POS_J1_90, "J1=90°")
    node.log_current("J1=90")

    # 3a. J5 先到位（防止灵巧手碰撞）
    cur = [v for v in node.latest_joint.position] if node.latest_joint else [0]*7
    pos_j5_first = list(cur[:5]) + [cur[5]]  # J1~J5取当前值
    pos_j5_first[4] = POS_TARGET[4]           # 只改 J5
    node.movej(pos_j5_first, "J5先行")
    node.log_current("J5到位")

    # 3b. 其余关节到位 → 握拳
    node.movej(POS_TARGET, "目标位")
    node.hand_fist()
    node.log_current("目标位")

    # 4. 回到 J1=90°（手保持握拳）
    node.movej(POS_J1_90, "J1=90°")
    node.log_current("J1=90")

    # 5. 归零 → 张开
    node.movej(POS_ZERO, "归零")
    node.hand_open()
    node.log_current("零位")

    node.get_logger().info("======== 完成 ========")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
