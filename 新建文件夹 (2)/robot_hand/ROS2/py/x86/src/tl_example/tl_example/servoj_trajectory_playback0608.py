#!/usr/bin/env python3
"""
Servoj 轨迹回放脚本（ROS2 节点）

从 config/testv1.json 读取关节角度轨迹点，以 servoj 模式逐点发送给机械臂执行。

使用方法：
  1. 确保 tl_driver 节点已启动并连接机械臂
  2. 运行本脚本：ros2 run tl_example servoj_trajectory_playback
  3. 在终端菜单中选择要执行的轨迹组
"""

import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import tl_ros2_interface.srv as srvs
import tl_ros2_interface.msg as msgs

# ============================================================
# 全局可调参数（直接修改这里的值即可）
# ============================================================
PUBLISH_FREQUENCY = 50  # servoj 关节角度发布频率（Hz），范围 100~250

OPEN_SERVOJ_VMAX = [300.0] * 7  # servoj 最大速度
OPEN_SERVOJ_AMAX = [3000.0] * 7  # servoj 最大加速度
OPEN_SERVOJ_JMAX = [50000.0] * 7  # servoj 最大加加速度

ZERO_JOINT = [3.0, -92.5, -3.2, 6.6, -90.0, -31.7, -10.0]  # 初始关节角度


MOVE_SPEED = 60.0  # 机械臂运行速度（%），MoveJ/MoveL 共用
TARGET_POINTS = 400  # 插值后总目标点数（关键帧之间线性插值）
# ============================================================

try:
    PKG_SHARE_DIR = get_package_share_directory("tl_example")
except Exception:
    # Fallback：未安装时指向包根目录
    PKG_SHARE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
JSON_PATH = os.path.join(PKG_SHARE_DIR, "config", "testv1.json")


class ServojTrajectoryPlayback(Node):
    def __init__(self):
        super().__init__("servoj_trajectory_playback")

        # ---- 服务客户端 ----
        self.cli_connect = self.create_client(Trigger, "/tl_driver/connect_arm")
        self.cli_power_on = self.create_client(Trigger, "/tl_driver/power_on")
        self.cli_open_servoj = self.create_client(srvs.OpenServoJ, "/tl_driver/open_servoj")
        self.cli_set_speed = self.create_client(srvs.SetSpeed, "/tl_driver/set_speed")
        self.cli_close_servoj = self.create_client(Trigger, "/tl_driver/close_servoj")

        # ---- 发布器 ----
        self.servoj_pub = self.create_publisher(Float64MultiArray, "/tl_driver/set_servoj_pos", 10)
        self.movej_pub = self.create_publisher(msgs.MoveCommand, "/tl_driver/moveJ", 10)

        # ---- 关节状态订阅（用于确认 MoveJ 到位） ----
        self.latest_joint_pos_ = None  # 弧度制
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.handle_joint_state, 10
        )

        self.get_logger().info("Servoj 轨迹回放节点已启动")

    def handle_joint_state(self, msg):
        """缓存最新的关节状态（弧度）"""
        self.latest_joint_pos_ = list(msg.position)

    # ---------------------------------------------------------------
    # 工具方法
    # ---------------------------------------------------------------
    def wait_for_service(self, cli, name, timeout_sec=10.0):
        """等待单个服务可用"""
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(f"服务 {name} 不可用（超时 {timeout_sec}s）")

    def call_service(self, cli, request, timeout_sec=30.0):
        """调用服务并等待响应

        注意：coord_transform 等服务可能耗时较长，
        如有超时可增大 timeout_sec 或修改顶部的 SERVICE_TIMEOUT。
        """
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(f"服务调用超时 ({timeout_sec}s)")
        return future.result()

    # ---------------------------------------------------------------
    # 核心流程
    # ---------------------------------------------------------------

    def set_speed(self):
        """设置机械臂运行速度"""
        self.get_logger().info(f"设置运行速度 = {MOVE_SPEED}%")
        req = srvs.SetSpeed.Request()
        req.speed = float(MOVE_SPEED)
        resp = self.call_service(self.cli_set_speed, req)
        if not resp.success:
            raise RuntimeError(f"设置速度失败：{resp.message}")
        self.get_logger().info(f"运行速度已设为 {MOVE_SPEED}%")

    def move_to_zero(self, timeout_sec=15.0):
        """MoveJ 回到零位（关节坐标系），等待到位确认"""
        self.get_logger().info("MoveJ 回到零位...")
        msg = msgs.MoveCommand()
        msg.target_pos_value = list(ZERO_JOINT)
        msg.target_pos_name = ""
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

        # 等待到位：订阅 /joint_states 确认所有关节到达目标
        target_rad = [math.radians(j) for j in ZERO_JOINT]  # 角度→弧度
        TOLERANCE = math.radians(2.0)  # 允许 2 度误差
        start = time.time()
        arrived = False
        self.get_logger().info("等待 MoveJ 运动到位...")

        while time.time() - start < timeout_sec:
            pos = self.latest_joint_pos_
            if pos is not None and len(pos) >= len(target_rad):
                # 检查所有关节是否在容差内
                ok = all(abs(pos[i] - target_rad[i]) < TOLERANCE for i in range(len(target_rad)))
                if ok:
                    arrived = True
                    break
            # 短暂 spinning 以接收话题
            rclpy.spin_once(self, timeout_sec=0.05)

        if arrived:
            self.get_logger().info("MoveJ 到位 ✓")
        else:
            self.get_logger().warn(f"MoveJ 未在 {timeout_sec}s 内到位，继续执行...")
        # 额外留一点缓冲
        time.sleep(0.3)

    def open_servoj_mode(self):
        """打开关节跟踪模式"""
        self.get_logger().info("打开 servoj 模式...")
        req = srvs.OpenServoJ.Request()
        req.vmax = OPEN_SERVOJ_VMAX
        req.amax = OPEN_SERVOJ_AMAX
        req.jmax = OPEN_SERVOJ_JMAX
        resp = self.call_service(self.cli_open_servoj, req)
        if not resp.success:
            raise RuntimeError(f"打开 servoj 失败：{resp.message}")
        self.get_logger().info("servoj 模式已打开")

    def close_servoj_mode(self):
        """关闭关节跟踪模式"""
        self.get_logger().info("关闭 servoj 模式...")
        resp = self.call_service(self.cli_close_servoj, Trigger.Request())
        if not resp.success:
            raise RuntimeError(f"关闭 servoj 失败：{resp.message}")
        self.get_logger().info("servoj 模式已关闭")

    def interpolate_keyframes(self, keyframes, target_total):
        """在关键帧之间线性插值，生成 target_total 个点"""
        n_kf = len(keyframes)
        if n_kf < 2:
            return keyframes
        n_seg = n_kf - 1
        ndim = len(keyframes[0])

        # 计算每段需要插入的点数
        pts_per_seg = []
        remaining = target_total - n_kf
        for i in range(n_seg):
            cnt = remaining // (n_seg - i)
            pts_per_seg.append(cnt)
            remaining -= cnt

        result = []
        for i in range(n_seg):
            a = keyframes[i]
            b = keyframes[i + 1]
            result.append(a)
            cnt = pts_per_seg[i]
            for s in range(1, cnt + 1):
                t = s / (cnt + 1)
                pt = [(1 - t) * a[j] + t * b[j] for j in range(ndim)]
                result.append(pt)
        result.append(keyframes[-1])
        return result

    def send_joint_angles(self, joint_angles):
        """发布关节角度到 /tl_driver/set_servoj_pos"""
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joint_angles]
        self.servoj_pub.publish(msg)

    def execute_group(self, name, keyframes):
        """执行一组轨迹点

        从 JSON 读取关键帧关节角度，插值后以 servoj 模式逐点发送给机械臂执行。
        """
        # 在关键帧之间插值
        play_points = self.interpolate_keyframes(keyframes, TARGET_POINTS)
        total = len(play_points)
        n_kf = len(keyframes)
        self.get_logger().info(
            f'\n====== 开始执行组 "{name}"（{n_kf} 个关键帧 → 插值至 {total} 个点） ======'
        )

        # --------------------------------------------------
        # 阶段 1：MoveJ 归零 + 打开 servoj 模式
        # --------------------------------------------------
        self.set_speed()
        self.move_to_zero()

        self.open_servoj_mode()
        time.sleep(2.0)  # 等待 servoj 模式稳定

        # --------------------------------------------------
        # 阶段 2：精准定时回放（time.sleep + 时间补偿）
        # --------------------------------------------------
        self.get_logger().info(f"阶段 2/2：以 {PUBLISH_FREQUENCY} Hz 回放 {total} 个点...")
        interval = 1.0 / PUBLISH_FREQUENCY
        for i, joint in enumerate(play_points, 1):
            t_start = time.monotonic()
            self.send_joint_angles(joint)
            elapsed = time.monotonic() - t_start
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

        # --------------------------------------------------
        # 关闭 servoj + 归零
        # --------------------------------------------------
        self.close_servoj_mode()
        time.sleep(0.5)
        self.move_to_zero()

        self.get_logger().info(f'====== 组 "{name}" 执行完成 ======\n')


def main(args=None):
    rclpy.init(args=args)

    # 读取 JSON 数据
    if not os.path.exists(JSON_PATH):
        print(f"错误：找不到文件 {JSON_PATH}")
        sys.exit(1)

    with open(JSON_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)

    group_keys = list(data.keys())
    num_groups = len(group_keys)

    # 终端选择菜单
    print("\n" + "=" * 55)
    print("  轨迹执行脚本")
    print("=" * 55)
    for i, key in enumerate(group_keys, 1):
        print(f'  {i}. 执行 "{key}"（{len(data[key])} 个点）')
    if num_groups > 1:
        total_pts = sum(len(data[k]) for k in group_keys)
        print(f"  A. 全部执行（共 {total_pts} 个点）")
    print(f"  0. 取消退出")
    print("=" * 55)

    choice = input("请输入编号：").strip()

    if choice == "0":
        print("已取消，退出脚本。")
        rclpy.shutdown()
        return
    elif choice.lower() == "a" and num_groups > 1:
        groups = [(k, data[k]) for k in group_keys]
    else:
        try:
            idx = int(choice) - 1
            if 0 <= idx < num_groups:
                key = group_keys[idx]
                groups = [(key, data[key])]
            else:
                print("无效输入，退出脚本。")
                rclpy.shutdown()
                return
        except (ValueError, IndexError):
            print("无效输入，退出脚本。")
            rclpy.shutdown()
            return

    # 初始化节点并执行
    node = ServojTrajectoryPlayback()

    try:
        # 等待所有服务就绪
        node.get_logger().info("等待 ROS2 服务就绪...")
        for cli, name in [
            (node.cli_connect, "/tl_driver/connect_arm"),
            (node.cli_power_on, "/tl_driver/power_on"),
            (node.cli_open_servoj, "/tl_driver/open_servoj"),
            (node.cli_set_speed, "/tl_driver/set_speed"),
            (node.cli_close_servoj, "/tl_driver/close_servoj"),
        ]:
            node.wait_for_service(cli, name)
        node.get_logger().info("所有服务已就绪")

        # 执行轨迹（可能一组或两组连续）
        for gname, gpoints in groups:
            node.execute_group(gname, gpoints)

    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    except Exception as e:
        node.get_logger().error(f"脚本异常：{e}")
        import traceback

        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
