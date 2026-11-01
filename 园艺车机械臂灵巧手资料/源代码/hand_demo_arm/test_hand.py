#!/usr/bin/env python3
"""
ARM 版灵巧手独立测试
=====================
直接在 Jetson 上测试灵巧手 ROH-LiteS001，不依赖视觉系统。

用法:
  cd ~/Desktop/hand_demo_arm
  source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
  python3 test_hand.py

按键:
  0/o - 张开    1/f - 握拳     2/v - 比耶
  3/m - 竖中指  4   - 赞      5   - 捏合
  6   - OK      7   - 摇滚    8   - 食指指
  i   - 初始化  r   - 复位    q   - 退出
"""

import sys
import os
import select
import termios
import tty
import time
import threading

import rclpy
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hand_controller import ROHHandController


class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def read(self):
        dr, _, _ = select.select([self.fd], [], [], 0)
        if dr:
            return os.read(self.fd, 1).decode(errors='ignore')
        return None

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


# 手势定义
GESTURES = {
    "OPEN":     {"label": "张开 ✋",     "pos": [    0,     0,     0,     0,     0,     0]},
    "FIST":     {"label": "握拳 ✊",     "pos": [60000, 60000, 60000, 60000, 60000, 30000]},
    "VICTORY":  {"label": "比耶 ✌",     "pos": [30000,     0,     0, 60000, 60000, 15000]},
    "MIDDLE":   {"label": "竖中指 🖕",   "pos": [60000, 60000,     0, 60000, 60000,     0]},
    "THUMB_UP": {"label": "赞 👍",       "pos": [    0, 60000, 60000, 60000, 60000, 60000]},
    "PINCH":    {"label": "捏合 🤏",     "pos": [20000, 20000,     0,     0,     0, 30000]},
    "OK":       {"label": "OK 👌",       "pos": [10000, 15000,     0,     0,     0, 20000]},
    "ROCK":     {"label": "摇滚 🤘",     "pos": [30000,     0, 60000, 60000,     0, 15000]},
    "POINT":    {"label": "指 ☝",       "pos": [60000,     0, 60000, 60000, 60000,     0]},
}

KEY_MAP = {
    '0': 'OPEN', 'o': 'OPEN',
    '1': 'FIST', 'f': 'FIST',
    '2': 'VICTORY', 'v': 'VICTORY',
    '3': 'MIDDLE',
    '4': 'THUMB_UP',
    '5': 'PINCH',
    '6': 'OK',
    '7': 'ROCK',
    '8': 'POINT',
}


def main():
    rclpy.init()

    node = Node("hand_test_arm")
    hand = ROHHandController(node)

    if not hand.init():
        node.get_logger().error("灵巧手初始化失败！检查连接和 ModBus 配置")
        rclpy.shutdown()
        return

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    kb = Keyboard()

    print("\n" + "=" * 45)
    print("  灵巧手 ROH-LiteS001 独立测试 (ARM版)")
    print("=" * 45)
    for key, name in sorted(KEY_MAP.items()):
        g = GESTURES[name]
        print(f"  [{key}]  {g['label']}")
    print("  [i]  初始化 (自检)")
    print("  [r]  软复位")
    print("  [q]  退出")
    print("=" * 45 + "\n")

    current = None
    try:
        while rclpy.ok():
            key = kb.read()
            if key is None:
                time.sleep(0.05)
                continue

            key = key.lower()

            if key == 'q':
                print("\n退出")
                break
            elif key == 'i':
                print("  → 初始化中...（手会咔咔响，正常）")
                hand.start_init()
                time.sleep(2)
                hand.open()
                current = None
                continue
            elif key == 'r':
                print("  → 软复位...")
                hand.reset()
                time.sleep(1)
                current = None
                continue

            if key not in KEY_MAP:
                continue

            name = KEY_MAP[key]
            g = GESTURES[name]

            if name == current:
                continue

            current = name
            print(f"  → {g['label']}  pos={g['pos']}")
            hand.set_fingers(g['pos'])

    except KeyboardInterrupt:
        pass
    finally:
        kb.restore()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
