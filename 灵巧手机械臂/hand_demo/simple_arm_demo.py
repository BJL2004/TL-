#!/usr/bin/env python3
"""简易机械臂运动 demo —— 直接发布 MoveJ 话题，ARM/Jetson 可用"""

import sys
import time
import rclpy
from rclpy.node import Node
from tl_ros2_interface.msg import MoveCommand


PRESET_POSITIONS = {
    "zero":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "pos1":  [30.0, -20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "pos2":  [-30.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "pos3":  [0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0.0],
    "pos4":  [0.0, 0.0, -30.0, 0.0, 0.0, 0.0, 0.0],
}


class SimpleArmDemo(Node):
    def __init__(self):
        super().__init__('simple_arm_demo')
        self.pub = self.create_publisher(MoveCommand, '/tl_driver/moveJ', 10)
        self.get_logger().info('简易机械臂 Demo 已就绪')

    def movej(self, joints, speed=20.0):
        msg = MoveCommand()
        msg.target_pos_value = [float(j) for j in joints] + [0.0] * (14 - len(joints))
        msg.target_pos_value = msg.target_pos_value[:14]
        msg.coord = 0
        msg.velocity = float(speed)
        msg.acc = float(speed)
        msg.dec = float(speed)
        self.pub.publish(msg)
        self.get_logger().info(f'MoveJ -> {joints[:6]} (speed={speed})')
        time.sleep(speed / 20.0 + 1.0)


def main():
    rclpy.init(args=sys.argv)
    node = SimpleArmDemo()

    sequence = ["pos1", "pos2", "pos3", "pos4", "zero"]
    speed = 20.0

    print("\n===== 简易机械臂运动 Demo =====")
    print(f"运动序列: {' -> '.join(sequence)}")
    print(f"速度: {speed}%")
    print("3 秒后开始...\n")
    time.sleep(3)

    for name in sequence:
        joints = PRESET_POSITIONS[name]
        print(f"\n>> 运动到 {name}: {joints[:6]}")
        node.movej(joints, speed=speed)
        rclpy.spin_once(node, timeout_sec=0.1)

    print("\n===== Demo 完成 =====")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
