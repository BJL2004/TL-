#!/usr/bin/env python3
# coding=utf-8
"""VLA 前置验证 (第1步) —— 上 VLA 之前必须先回答的三个问题:
  1) 反馈频率: /tcp_pose /joint_states /arm_status 各多少 Hz (采数据/闭环的命脉)
  2) 手拖记录: 拖拽模式下能否稳定记录轨迹 (示范采集的命脉) -> 存 CSV
  3) 流式下发: /tl_safe/moveL 能否 10~20Hz 连续小步下发并跟得住 (VLA 闭环的命脉)

安全说明: 所有运动仅为当前位置周围 ±3mm 的小圆; 全程监控 /arm_status, 异常立即停。
不使能任何大动作; 结束后把起点发回去。手放急停旁。

在机器人上运行(先 source workspace, 并启动你们平时 moveL 前的安全代理):
  python3 vla_stream_probe.py --test feedback
  python3 vla_stream_probe.py --test drag        # 按提示手拖10秒
  python3 vla_stream_probe.py --test stream
  python3 vla_stream_probe.py --test all
"""
import argparse
import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

try:
    from tl_ros2_interface.msg import ArmStatus, CartesianPose, MoveCommand
except ImportError:
    print("找不到 tl_ros2_interface: 请在机器人上运行, 先 "
          "`source install/setup.bash` (或你的 workspace 环境)")
    sys.exit(2)


class Probe(Node):
    def __init__(self):
        super().__init__("vla_stream_probe")
        self.tcp = None
        self.tcp_t = 0.0
        self.tcp_n = 0
        self.joint = None
        self.joint_t = 0.0
        self.joint_n = 0
        self.state = None
        self.state_t = 0.0
        self.state_n = 0
        self.safe_enabled = False
        self.safe_t = 0.0
        self.move_pub = self.create_publisher(MoveCommand, "/tl_safe/moveL", 50)
        self.safe_cli = self.create_client(SetBool, "/tl_safe/set_enabled")
        self.create_subscription(CartesianPose, "/tcp_pose", self._on_tcp, 50)
        self.create_subscription(JointState, "/joint_states", self._on_joint, 50)
        self.create_subscription(ArmStatus, "/arm_status", self._on_state, 50)
        self.create_subscription(String, "/tl_safe/status", self._on_safe, 10)

    # ---------- 回调 ----------
    def _on_tcp(self, m):
        self.tcp = [float(m.position.x), float(m.position.y),
                    float(m.position.z), float(m.rpy.x), float(m.rpy.y),
                    float(m.rpy.z)]
        self.tcp_t = time.monotonic()
        self.tcp_n += 1

    def _on_joint(self, m):
        self.joint = list(m.position)
        self.joint_t = time.monotonic()
        self.joint_n += 1

    def _on_state(self, m):
        self.state = str(m.run_state).upper()
        self.state_t = time.monotonic()
        self.state_n += 1

    def _on_safe(self, m):
        try:
            self.safe_enabled = bool(json.loads(m.data).get("enabled", False))
            self.safe_t = time.monotonic()
        except Exception:
            pass

    # ---------- 工具 ----------
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self, need_move_sub=True, timeout=6.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            fresh = (self.tcp is not None and self.joint is not None
                     and time.monotonic() - self.tcp_t < 0.5)
            sub_ok = (not need_move_sub
                      or self.move_pub.get_subscription_count() > 0)
            if fresh and sub_ok:
                return True
        return False

    def set_enabled(self, en):
        if not self.safe_cli.wait_for_service(timeout_sec=2.0):
            print("[安全] /tl_safe/set_enabled 不可用 (安全代理没启动?)")
            return False
        req = SetBool.Request()
        req.data = bool(en)
        fut = self.safe_cli.call_async(req)
        end = time.monotonic() + 3.0
        while rclpy.ok() and not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        try:
            r = fut.result()
            print(f"[安全] set_enabled({en}) -> success={r.success} "
                  f"{getattr(r, 'message', '')}")
            return bool(r.success)
        except Exception as e:
            print(f"[安全] 调用异常: {e}")
            return False

    def send_moveL(self, pose6, v=10.0, a=10.0):
        c = MoveCommand()
        c.target_pos_value = [float(x) for x in pose6] + [0.0] * 8
        c.target_pos_type = 0
        c.coord = 1
        c.velocity = float(v)
        c.velocity_sync = 0.0
        c.acc = float(a)
        c.dec = float(a)
        c.pl = 0
        c.time = 0
        c.tool_num = 0
        c.user_num = 0
        c.posidtype = 0
        c.configuration = 0
        c.spin = 0
        c.para_sync = False
        self.move_pub.publish(c)

    # ---------- 测试1: 反馈频率 ----------
    def test_feedback(self, sec=5.0):
        print(f"\n===== 测试1: 反馈频率 (统计 {sec:.0f}s) =====")
        if not self.wait_ready(need_move_sub=False):
            print("[失败] 收不到 /tcp_pose /joint_states, 检查话题名/网络")
            return
        n0 = (self.tcp_n, self.joint_n, self.state_n)
        t0 = time.monotonic()
        self.spin_for(sec)
        dt = time.monotonic() - t0
        print(f"  /tcp_pose     : {(self.tcp_n - n0[0]) / dt:6.1f} Hz")
        print(f"  /joint_states : {(self.joint_n - n0[1]) / dt:6.1f} Hz")
        print(f"  /arm_status   : {(self.state_n - n0[2]) / dt:6.1f} Hz")
        print(f"  当前位姿: {[round(x, 3) for x in self.tcp]}")
        print(f"  当前关节: {[round(x, 3) for x in self.joint]}")
        print(f"  当前状态: {self.state}  安全代理 enabled={self.safe_enabled}")
        print("  (位姿单位按你们工程惯例应为 mm+rad —— 看一眼数值是否合理)")

    # ---------- 测试2: 手拖记录 ----------
    def test_drag(self, sec=10.0, out=None):
        print(f"\n===== 测试2: 手拖记录 ({sec:.0f}s) =====")
        if not self.wait_ready(need_move_sub=False):
            print("[失败] 状态不新鲜")
            return
        print(">>> 现在切到拖拽模式, 用手拖动机械臂, 慢慢画一个 20cm 左右的"
              "方块 (xs移动), 尽量稳 <<<")
        input(">>> 摆好后回本窗口按回车开始计时: ")
        samples = []
        last_n = self.tcp_n
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.tcp_n != last_n and self.tcp is not None:
                last_n = self.tcp_n
                samples.append([round(time.monotonic() - t0, 4)] + self.tcp)
        if not samples:
            print("  没收到样本 —— 拖拽模式下 /tcp_pose 停止发布? 这本身是重要结论")
            return
        dts = [samples[i + 1][0] - samples[i][0]
               for i in range(len(samples) - 1)]
        hz = 1.0 / statistics.median(dts) if dts else 0.0
        cols = list(zip(*[s[1:] for s in samples]))
        rng = [max(c) - min(c) for c in cols]
        print(f"  记录 {len(samples)} 个位姿样本, 中位频率 {hz:.1f} Hz")
        print(f"  轨迹范围: dx={rng[0]:.1f} dy={rng[1]:.1f} dz={rng[2]:.1f} "
              f"(单位同 /tcp_pose)")
        print(f"  最大相邻帧跳变: "
              f"{max(math.dist(samples[i][1:4], samples[i+1][1:4]) for i in range(len(samples)-1)):.2f}")
        fn = out or f"drag_record_{time.strftime('%H%M%S')}.csv"
        with open(fn, "w", encoding="utf-8") as f:
            f.write("t,x,y,z,rx,ry,rz\n")
            for s in samples:
                f.write(",".join(str(v) for v in s) + "\n")
        print(f"  已存 {fn} (拖拽后可以把文件发我看单位/抖动/频率质量)")

    # ---------- 测试3: moveL 流式下发 ----------
    def test_stream(self, n_steps=60, hz=15.0, amp=3.0, v=10.0):
        print(f"\n===== 测试3: moveL 流式下发 ({n_steps}步 @{hz:.0f}Hz, "
              f"半径{amp}mm 小圆) =====")
        if not self.wait_ready():
            print("[失败] 反馈不新鲜 或 /tl_safe/moveL 没有订阅者"
                  "(安全代理没启动?)")
            return
        print(f"  起点位姿: {[round(x, 2) for x in self.tcp]}")
        if not self.set_enabled(True):
            print("  [注意] 自动解锁未成功 —— 若你们流程要求手动解锁, "
                  "解锁后重跑本测试")
        p0 = list(self.tcp)
        sent = 0
        worst_lag = 0.0
        t0 = time.monotonic()
        aborted = False
        for k in range(n_steps):
            ph = 2.0 * math.pi * k / n_steps
            tgt = list(p0)
            tgt[0] = p0[0] + amp * math.cos(ph)
            tgt[1] = p0[1] + amp * math.sin(ph)
            self.send_moveL(tgt, v, 10.0)
            sent += 1
            if self.state == "PAUSE" or time.monotonic() - self.state_t > 1.5:
                print(f"  [中止] 状态异常: {self.state}")
                aborted = True
                break
            nxt = t0 + k / hz
            while time.monotonic() < nxt:
                rclpy.spin_once(self, timeout_sec=0.005)
            if self.tcp is not None:
                lag = math.dist(self.tcp[:3], tgt[:3])
                worst_lag = max(worst_lag, lag)
            if k % 10 == 0:
                print(f"  第{k:3d}步 已发{sent:3d}  当前跟踪误差 "
                      f"{(math.dist(self.tcp[:3], tgt[:3]) if self.tcp else -1):5.2f}  "
                      f"状态 {self.state}")
        dt = time.monotonic() - t0
        print(f"  ---- 结果: 发出 {sent} 条 / {dt:.1f}s = "
              f"实际下发率 {sent / max(dt, 1e-6):.1f} Hz (目标 {hz})")
        print(f"  ---- 最大跟踪误差 {worst_lag:.2f} (同/tcp_pose单位; 越小越好)")
        print("  回起点 ...")
        self.send_moveL(p0, v, 10.0)
        self.spin_for(2.0)
        if self.tcp is not None:
            print(f"  回位残差 {math.dist(self.tcp[:3], p0[:3]):.2f}")
        if aborted:
            print("  [提示] 中途被中止, 请检查安全代理日志")
        print("  (测试完可把安全锁交回: /tl_safe/set_enabled false)")


def main():
    ap = argparse.ArgumentParser(description="VLA 前置验证: 反馈/手拖/流式")
    ap.add_argument("--test", default="all",
                    choices=["feedback", "drag", "stream", "all"])
    ap.add_argument("--stream-steps", type=int, default=60)
    ap.add_argument("--stream-hz", type=float, default=15.0)
    ap.add_argument("--stream-amp", type=float, default=3.0,
                    help="小圆半径(与位姿同单位, mm)")
    ap.add_argument("--velocity", type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    try:
        if args.test in ("feedback", "all"):
            node.test_feedback()
        if args.test in ("drag", "all"):
            node.test_drag()
        if args.test in ("stream", "all"):
            node.test_stream(args.stream_steps, args.stream_hz,
                             args.stream_amp, args.velocity)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
