#!/usr/bin/env python3
"""
灵巧手键盘控制 —— 握拳 / 比耶 / 竖中指

按键:
  1 或 f  ── 握拳
  2 或 v  ── 比耶 ✌
  3 或 m  ── 竖中指 🖕
  0 或 o  ── 张开

运行:
  cd ~/Desktop/hand_demo
  source ~/Desktop/tl_robot_ros2_py-x86/install/setup.bash
  python3 hand_keyboard.py
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

# 同目录导入 hand_controller
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hand_controller import ROHHandController


# ── 键盘读取 ──────────────────────────────────────────────────

class Keyboard:
    """非阻塞键盘输入"""

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


# ── 自定义手势 ──────────────────────────────────────────────

# 位置值含义: 0=完全张开, 65535=完全闭合 (实际范围因手而异)
# 手指顺序: 拇指弯曲, 食指, 中指, 无名指, 小指, 拇指旋转

GESTURES = {

    # --- 基础 ---
    "OPEN": {
        "label": "张开 ✋",
        "pos":   [    0,     0,     0,     0,     0,     0],
    },

    "FIST": {
        "label": "握拳 ✊",
        "pos":   [60000, 60000, 60000, 60000, 60000, 30000],
    },

    # --- 好玩手势 ---
    "VICTORY": {
        "label": "比耶 ✌",
        "pos":   [30000,     0,     0, 60000, 60000, 15000],
    },

    "MIDDLE": {
        "label": "竖中指 🖕",
        "pos":   [60000, 60000,     0, 60000, 60000,     0],
    },

    # --- 可扩展 ---
    "THUMB_UP": {
        "label": "赞 👍",
        "pos":   [    0, 60000, 60000, 60000, 60000, 60000],
    },

    "PINCH": {
        "label": "捏合 🤏",
        "pos":   [20000, 20000,     0,     0,     0, 30000],
    },

    "OK": {
        "label": "OK 👌",
        "pos":   [10000, 15000,     0,     0,     0, 20000],
    },

    "ROCK": {
        "label": "摇滚 🤘",
        "pos":   [30000,     0, 60000, 60000,     0, 15000],
    },

    "POINT": {
        "label": "指 ☝",
        "pos":   [60000,     0, 60000, 60000, 60000,     0],
    },
}

# 键盘映射: 按键 → 手势名
KEY_MAP = {
    '0': 'OPEN',
    'o': 'OPEN',
    '1': 'FIST',
    'f': 'FIST',
    '2': 'VICTORY',
    'v': 'VICTORY',
    '3': 'MIDDLE',
    'm': 'MIDDLE',
    '4': 'THUMB_UP',
    '5': 'PINCH',
    '6': 'OK',
    '7': 'ROCK',
    '8': 'POINT',
}


# ── 主程序 ────────────────────────────────────────────────────

def main():
    rclpy.init()

    node = Node("hand_keyboard")
    hand = ROHHandController(node)

    if not hand.init():
        node.get_logger().error("灵巧手初始化失败，退出")
        rclpy.shutdown()
        return

    # 后台 spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    kb = Keyboard()

    # 打印帮助
    print("\n" + "=" * 40)
    print("  灵巧手键盘控制")
    print("=" * 40)
    for key, name in sorted(KEY_MAP.items()):
        g = GESTURES[name]
        print(f"  [{key}]  {g['label']}")
    print("  [q]  退出")
    print("=" * 40 + "\n")

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

            if key not in KEY_MAP:
                continue

            name = KEY_MAP[key]
            g = GESTURES[name]

            if name == current:
                continue  # 防止重复发送
            current = name

            print(f"  → {g['label']}")
            hand.set_fingers(g['pos'])

    except KeyboardInterrupt:
        pass
    finally:
        kb.restore()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
