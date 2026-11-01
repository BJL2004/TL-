#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TL 模拟执行端（演示/联调用）
=============================
模拟 TL TCB605 程序在 MQTT 上的行为：订阅指令 → 打印收到什么 → 延时后回执。
不连接任何真实机械臂，只用于在 PC 上演示/联调 "LLM → MQTT → 执行 → 回执" 全链路。

用法：
  python3 mock_tl.py                                    # 本地 broker（127.0.0.1:1883）
  python3 mock_tl.py --host 116.205.245.172 --username shapandevice --password "qwer1234!!" \
      --down node/arm/99/down --up node/arm/99/up       # 用云端 broker 的【演示主题】
  python3 mock_tl.py --delay 4 --fail-rate 0.3          # 模拟慢动作 / 30%失败率

⚠️ 演示安全：务必使用演示主题（默认 node/arm/99/down|up），
   【不要】使用真实的 node/arm/01/* ——真实机器人如果在线，会真的执行！

回执语义（与真实执行端一致）：
  seed/seed_place: seed:1（已接受）→ seed:0（取种完成，非终态）
                  → seed_place:1（开始放置）→ seed_place:0 成功 / 3 失败
  trim_pick/trim_place: 先回 1（已开始动作），再回 0 成功 / 3 失败
  estop: 立即回 0
"""

import argparse
import json
import random
import sys
import threading
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("缺少 paho-mqtt：请先 pip install paho-mqtt")
    sys.exit(1)


def build_client(client_id):
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:
        return mqtt.Client(client_id=client_id)


def main():
    parser = argparse.ArgumentParser(description="TL 模拟执行端（MQTT 演示用）")
    parser.add_argument("--host", default="127.0.0.1", help="broker 地址（默认 127.0.0.1）")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--username", default="")
    parser.add_argument("--password", default="")
    parser.add_argument("--down", default="node/arm/99/down", help="指令主题（演示用 99，勿用 01）")
    parser.add_argument("--up", default="node/arm/99/up", help="回执主题")
    parser.add_argument("--client-id", default="mock-tl-demo")
    parser.add_argument("--delay", type=float, default=2.0, help="模拟执行耗时(秒)")
    parser.add_argument("--fail-rate", type=float, default=0.0, help="模拟失败概率 0~1")
    args = parser.parse_args()

    client = build_client(args.client_id)
    if args.username:
        client.username_pw_set(args.username, args.password)

    def publish_feedback(cmd, target_id, code):
        payload = {"cmd": cmd, "retCode": int(code)}
        if target_id:
            payload["targetId"] = target_id
        data = json.dumps(payload, ensure_ascii=False)
        client.publish(args.up, data, qos=1)
        print(f"📤 模拟回执: {data}")

    def handle(cmd_obj):
        name = cmd_obj.get("cmd")
        target = str(cmd_obj.get("targetId", ""))
        print(f"📩 模拟 TL 收到指令: {json.dumps(cmd_obj, ensure_ascii=False)}")
        if name == "estop":
            print("🛑 模拟端：急停锁存")
            publish_feedback("estop", "", 0)
            return
        if name in ("seed", "seed_place"):
            print(
                f"⏳ 模拟执行 {name} {target}"
                f"（取种→识码→放置→回位，约 {args.delay * 3:.1f}s）…"
            )
            # 与真实执行端一致的两段式反馈：
            # seed:1 → seed:0（取种完成，非终态）→ seed_place:1 → seed_place:0/3
            publish_feedback(name, target, 1)
            time.sleep(args.delay)
            publish_feedback("seed", target, 0)
            time.sleep(args.delay)
            publish_feedback("seed_place", target, 1)
            time.sleep(args.delay)
            code = 3 if random.random() < args.fail_rate else 0
            publish_feedback("seed_place", target, code)
            return
        if name in ("trim_pick", "trim_place"):
            print("▶ 模拟已开始动作（trim 协议先发 retCode=1）")
            publish_feedback(name, target, 1)
            time.sleep(args.delay)
            code = 3 if random.random() < args.fail_rate else 0
            publish_feedback(name, target, code)
            return
        print(f"⚠️ 未知命令: {name}")
        publish_feedback(str(name), target, 1)

    def on_connect(client_, userdata, flags, rc, properties=None):
        code = getattr(rc, "value", rc)
        if int(code) == 0:
            client_.subscribe(args.down, qos=1)
            print(f"✅ 模拟 TL 已连接 {args.host}:{args.port}")
            print(f"   订阅指令: {args.down}")
            print(f"   回执主题: {args.up}")
            print("   等待指令…（Ctrl+C 退出）")
        else:
            print(f"❌ MQTT 连接被拒: rc={code}")

    def on_message(client_, userdata, message):
        try:
            cmd_obj = json.loads(message.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            print(f"⚠️ 收到非 JSON 负载: {message.payload!r}")
            return
        threading.Thread(target=handle, args=(cmd_obj,), daemon=True).start()

    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(args.host, args.port, 60)
    except Exception as error:
        print(f"❌ 连不上 broker {args.host}:{args.port} — {error}")
        print("   → 本地演示先启动 Mosquitto；或用 --host 指定可用的 broker")
        sys.exit(1)
    client.loop_forever()


if __name__ == "__main__":
    main()
