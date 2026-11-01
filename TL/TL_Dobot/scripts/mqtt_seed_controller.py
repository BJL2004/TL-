#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MQTT seed command controller (vehicle side).

Send seed commands to the robot arm and receive feedback.

Usage:
    python3 mqtt_seed_controller.py                          # interactive mode
    python3 mqtt_seed_controller.py --target A1-01           # single command
    python3 mqtt_seed_controller.py --target A1-01 A2-01 A3-01  # batch commands
    python3 mqtt_seed_controller.py --host 192.168.1.100     # remote broker

Interactive commands:
    A1-01          send seed to target A1-01
    A1-01 A2-01    send to multiple targets sequentially
    q              quit
"""

import argparse
import json
import sys
import threading
import time

import paho.mqtt.client as mqtt


COMMAND_TOPIC = "tl/robot/command"
FEEDBACK_TOPIC = "tl/robot/feedback"
RET_CODES = {0: "成功", 1: "失败(未检测到种子或执行出错)", 2: "位置被占(上一轮未完成)"}


class SeedController:
    def __init__(self, host, port=1883):
        self.host = host
        self.port = port
        self.feedback_received = threading.Event()
        self.last_feedback = None
        try:
            self.client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2, client_id="seed-controller"
            )
        except AttributeError:
            self.client = mqtt.Client(client_id="seed-controller")
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if int(getattr(reason_code, "value", reason_code)) != 0:
            print(f"Broker 连接失败: {reason_code}")
            sys.exit(1)
        client.subscribe(FEEDBACK_TOPIC, qos=1)
        print(f"已连接 Broker {self.host}:{self.port}, 监听反馈: {FEEDBACK_TOPIC}")

    def _on_disconnect(self, client, userdata, *args):
        print("Broker 已断开")

    def _on_message(self, client, userdata, message):
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            print(f"收到无效反馈: {message.payload}")
            return
        cmd = payload.get("cmd", "?")
        ret_code = payload.get("retCode", -1)
        desc = RET_CODES.get(ret_code, f"未知 retCode={ret_code}")
        print(f"\n反馈: cmd={cmd}, retCode={ret_code} ({desc})")
        self.last_feedback = payload
        self.feedback_received.set()

    def start(self):
        self.client.connect(self.host, self.port, 60)
        self.client.loop_start()
        time.sleep(0.5)

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()

    def send_seed(self, target_id, timeout=120):
        payload = json.dumps({"cmd": "seed", "targetId": target_id})
        self.feedback_received.clear()
        self.last_feedback = None
        print(f"发送: {payload} -> {COMMAND_TOPIC}")
        result = self.client.publish(COMMAND_TOPIC, payload, qos=1)
        if result.rc != 0:
            print(f"发送失败: rc={result.rc}")
            return False
        print(f"等待反馈 (超时 {timeout}s)...", end="", flush=True)
        if not self.feedback_received.wait(timeout=timeout):
            print("超时: 未收到反馈")
            return False
        return self.last_feedback is not None

    def send_batch(self, target_ids):
        for i, tid in enumerate(target_ids):
            print(f"\n=== [{i + 1}/{len(target_ids)}] targetId={tid} ===")
            self.send_seed(tid)
            if i < len(target_ids) - 1:
                print("等待 2s 后发送下一个...")
                time.sleep(2)

    def interactive(self):
        print("\n交互模式: 输入 targetId 发送 seed 命令,多个用空格分隔,q 退出")
        print(f"示例: A1-01  或  A1-01 A2-01 A3-01\n")
        while True:
            try:
                line = input(">>> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue
            if line.lower() == "q":
                break
            target_ids = line.split()
            if len(target_ids) == 1:
                self.send_seed(target_ids[0])
            else:
                self.send_batch(target_ids)


def main():
    parser = argparse.ArgumentParser(description="MQTT seed command controller")
    parser.add_argument("--host", default="127.0.0.1", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--target", nargs="+", help="target ID(s) for non-interactive mode")
    parser.add_argument("--timeout", type=int, default=120, help="feedback timeout in seconds")
    args = parser.parse_args()

    controller = SeedController(args.host, args.port)
    controller.start()

    try:
        if args.target:
            controller.send_batch(args.target)
            print("\n进入交互模式,继续输入 targetId 发送命令,q 退出")
            controller.interactive()
        else:
            controller.interactive()
    finally:
        controller.stop()


if __name__ == "__main__":
    main()