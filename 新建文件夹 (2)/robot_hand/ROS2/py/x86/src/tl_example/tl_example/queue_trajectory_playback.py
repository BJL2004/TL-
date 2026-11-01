#!/usr/bin/env python3
"""
队列轨迹回放脚本（ROS2 节点）

从 config/saved_points.json 读取位置点，拼合固定姿态后通过 coord_transform
转换为关节角度，以队列模式（queue_motion_movej）逐点发送给机械臂执行。

队列模式特点：每调用一次 queue_motion_movej，机械臂实际执行一条 MoveJ，
轨迹由控制器内部规划，运动更平滑准确。

使用方法：
  1. 确保 tl_driver 节点已启动并连接机械臂
  2. 运行本脚本：ros2 run tl_example queue_executor
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
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import tl_ros2_interface.srv as srvs
import tl_ros2_interface.msg as msgs

# ============================================================
# 全局可调参数（直接修改这里的值即可）
# ============================================================
ARM_ANGLE = 0.0  # 臂角 (7维位姿的第7个元素)

ZERO_JOINT = [-34.381, 5.992, -0.247, 4.799, -1.359, -9.841, -62.214]  # 初始关节角度

ROLL = 3.13  # 固定横滚角
PITCH = -0.027  # 固定俯仰角
YAW = -0.485  # 固定偏航角

MOVE_SPEED = 50.0  # 全局运行速度（%），影响 set_speed 和每个 MoveCommand
QUEUE_ACC = 50.0  # 队列运动加速度
QUEUE_DEC = 50.0  # 队列运动减速度
QUEUE_WAIT_TIME = 55.0  # 插入完成后等待队列运动完成的延时（秒）
# ============================================================

try:
    PKG_SHARE_DIR = get_package_share_directory("tl_example")
except Exception:
    # Fallback：未安装时指向包根目录
    PKG_SHARE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
JSON_PATH = os.path.join(PKG_SHARE_DIR, "config", "saved_points.json")


class QueueTrajectoryPlayback(Node):
    def __init__(self):
        super().__init__("queue_trajectory_playback")

        # ---- 服务客户端 ----
        self.cli_connect = self.create_client(Trigger, "/tl_driver/connect_arm")
        self.cli_power_on = self.create_client(Trigger, "/tl_driver/power_on")
        self.cli_coord_transform = self.create_client(
            srvs.CoordTransform, "/tl_driver/coord_transform"
        )
        self.cli_set_speed = self.create_client(srvs.SetSpeed, "/tl_driver/set_speed")
        self.cli_queue_set_status = self.create_client(
            srvs.QueueMotionSetStatus, "/tl_driver/queue_motion_set_status"
        )
        self.cli_queue_movej = self.create_client(
            srvs.QueueMotionMoveJ, "/tl_driver/queue_motion_movej"
        )

        # ---- 发布器 ----
        self.movej_pub = self.create_publisher(msgs.MoveCommand, "/tl_driver/moveJ", 10)

        # ---- 关节状态订阅（用于确认 MoveJ 到位） ----
        self.latest_joint_pos_ = None  # 弧度制
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.handle_joint_state, 10
        )

        self.get_logger().info("队列轨迹回放节点已启动")

    def handle_joint_state(self, msg):
        """缓存最新的关节状态（弧度）"""
        self.latest_joint_pos_ = list(msg.position)

    # ---------------------------------------------------------------
    # 工具方法
    # ---------------------------------------------------------------
    def wait_for_service(self, cli, name, timeout_sec=10.0):
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError(f"服务 {name} 不可用（超时 {timeout_sec}s）")

    def call_service(self, cli, request, timeout_sec=60.0):
        """调用服务并等待响应"""
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(f"服务调用超时 ({timeout_sec}s)")
        return future.result()

    # ---------------------------------------------------------------
    # 核心流程
    # ---------------------------------------------------------------

    def set_speed(self):
        """设置机械臂全局运行速度"""
        self.get_logger().info(f"设置运行速度 = {MOVE_SPEED}%")
        req = srvs.SetSpeed.Request()
        req.speed = float(MOVE_SPEED)
        resp = self.call_service(self.cli_set_speed, req)
        if not resp.success:
            raise RuntimeError(f"设置速度失败：{resp.message}")
        self.get_logger().info(f"运行速度已设为 {MOVE_SPEED}%")

    def power_on(self):
        """上电（关闭队列模式后会自动下电，需要重新上电）"""
        self.get_logger().info("上电...")
        resp = self.call_service(self.cli_power_on, Trigger.Request())
        if not resp.success:
            raise RuntimeError(f"上电失败：{resp.message}")
        self.get_logger().info("上电成功")

    def move_to_zero(self, timeout_sec=15.0):
        """MoveJ 回到零位（关节坐标系），等待到位确认"""
        self.get_logger().info("MoveJ 回到零位...")
        msg = msgs.MoveCommand()
        msg.target_pos_value = ZERO_JOINT + [0.0] * 7
        msg.target_pos_name = ""
        msg.target_pos_type = 0
        msg.coord = 0
        msg.velocity = MOVE_SPEED
        msg.velocity_sync = 0.0
        msg.acc = QUEUE_ACC
        msg.dec = QUEUE_DEC
        msg.pl = 0
        msg.time = 0
        msg.tool_num = 0
        msg.user_num = 0
        msg.posidtype = 0
        msg.configuration = 0
        msg.spin = 0
        msg.para_sync = False
        self.movej_pub.publish(msg)

        # 等待到位
        target_rad = [math.radians(j) for j in ZERO_JOINT]
        TOLERANCE = math.radians(2.0)
        start = time.time()
        arrived = False
        self.get_logger().info("等待 MoveJ 运动到位...")

        while time.time() - start < timeout_sec:
            pos = self.latest_joint_pos_
            if pos is not None and len(pos) >= len(target_rad):
                ok = all(abs(pos[i] - target_rad[i]) < TOLERANCE for i in range(len(target_rad)))
                if ok:
                    arrived = True
                    break
            rclpy.spin_once(self, timeout_sec=0.05)

        if arrived:
            self.get_logger().info("MoveJ 到位 ✓")
        else:
            self.get_logger().warn(f"MoveJ 未在 {timeout_sec}s 内到位，继续执行...")
        time.sleep(0.3)

    def pose_to_joint(self, x, y, z):
        """调用 coord_transform 将单个笛卡尔位姿转换为关节角度

        拼接的完整位姿: [x, y, z, ROLL, PITCH, YAW, ARM_ANGLE]

        返回: list[float] 关节角度
        """
        req = srvs.CoordTransform.Request()
        req.origin_coord = 1  # 直角坐标系
        req.target_coord = 0  # 关节坐标系
        req.form = 0
        origin_pose = [float(x), float(y), float(z), ROLL, PITCH, YAW, ARM_ANGLE]
        req.origin_pos = origin_pose
        req.reference_pos = list(ZERO_JOINT)

        self.get_logger().info(f"  请求 origin_pos={origin_pose}")
        resp = self.call_service(self.cli_coord_transform, req)
        if not resp.success:
            raise RuntimeError(f"坐标转换失败：{resp.message}")
        result = list(resp.target_pos)
        self.get_logger().info(f"  返回 target_pos={result}")
        return result

    def set_queue_status(self, enable):
        """打开/关闭队列运动模式"""
        label = "打开" if enable else "关闭"
        self.get_logger().info(f"{label}队列模式...")
        req = srvs.QueueMotionSetStatus.Request()
        req.status = enable
        resp = self.call_service(self.cli_queue_set_status, req)
        if not resp.success:
            raise RuntimeError(f"{label}队列模式失败：{resp.message}")
        self.get_logger().info(f"队列模式已{label}")

    def queue_movej(self, joint_angles, is_continue):
        """通过 queue_motion_movej 插入一条队列 MoveJ

        Args:
            joint_angles: 目标关节角度（度）
            is_continue: 是否与上一条连续（首条设为 False，后续 True）
        """
        msg = msgs.MoveCommand()
        msg.target_pos_value = list(joint_angles) + [0.0] * 7
        msg.target_pos_name = ""
        msg.target_pos_type = 0
        msg.coord = 0
        msg.velocity = MOVE_SPEED
        msg.velocity_sync = 0.0
        msg.acc = QUEUE_ACC
        msg.dec = QUEUE_DEC
        msg.pl = 2
        msg.time = 0
        msg.tool_num = 0
        msg.user_num = 0
        msg.posidtype = 0
        msg.configuration = 0
        msg.spin = 0
        msg.para_sync = False

        req = srvs.QueueMotionMoveJ.Request()
        req.is_continue = is_continue
        req.cmd = msg

        resp = self.call_service(self.cli_queue_movej, req)
        if not resp.success:
            raise RuntimeError(f"队列 MoveJ 失败：{resp.message}")
        return resp

    def execute_group(self, name, points):
        """执行一组轨迹点（队列模式）"""
        total = len(points)
        self.get_logger().info(f'\n====== 开始执行组 "{name}"（共 {total} 个点） ======')

        # 1) 设置速度 + MoveJ 回到初始位
        self.set_speed()
        self.move_to_zero()

        # 2) 打开队列模式
        self.set_queue_status(True)
        time.sleep(1.5)
        self.set_speed()

        # 3) 逐点处理：坐标转换 → 插入队列 MoveJ
        self.get_logger().info("开始插入队列运动点...")
        for i, (x, y, z) in enumerate(points, 1):
            try:
                self.get_logger().info(f"[{i:3d}/{total}] 处理点 ({x:.1f}, {y:.1f}, {z:.1f})")
                joint = self.pose_to_joint(x, y, z)

                # 第一个点 is_continue=false，后续点 true（连续运动）
                is_cont = i > 1
                self.queue_movej(joint, is_cont)
                self.get_logger().info(f"[{i:3d}/{total}] 已插入队列 ✓")

            except Exception as e:
                self.get_logger().error(f"[{i:3d}/{total}] 处理失败：{e}")
                import traceback

                self.get_logger().error(traceback.format_exc())

        # 4) 等待队列运动完成
        self.get_logger().info(f"等待队列运动完成（{QUEUE_WAIT_TIME}s）...")
        time.sleep(QUEUE_WAIT_TIME)

        # 5) 关闭队列模式（注意：关闭后机械臂会自动下电）
        self.set_queue_status(False)
        time.sleep(1)

        # 6) 重新上电 + 回到零位
        self.power_on()
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
    if len(group_keys) < 2:
        print("错误：JSON 中至少需要两组数据（昊、辰）")
        sys.exit(1)

    group_1_name = group_keys[0]
    group_2_name = group_keys[1]

    # 终端选择菜单
    print("\n" + "=" * 55)
    print("  队列轨迹执行脚本")
    print("=" * 55)
    print(f'  1. 执行 "{group_1_name}"（{len(data[group_1_name])} 个点）')
    print(f'  2. 执行 "{group_2_name}"（{len(data[group_2_name])} 个点）')
    print(f'  3. 执行 "1+2" 连续（{len(data[group_1_name])} + {len(data[group_2_name])} 个点）')
    print(f"  0. 取消退出")
    print("=" * 55)

    choice = input("请输入编号：").strip()

    if choice == "0":
        print("已取消，退出脚本。")
        rclpy.shutdown()
        return
    elif choice == "1":
        groups = [(group_1_name, data[group_1_name])]
    elif choice == "2":
        groups = [(group_2_name, data[group_2_name])]
    elif choice == "3":
        groups = [
            (group_1_name, data[group_1_name]),
            (group_2_name, data[group_2_name]),
        ]
    else:
        print("无效输入，退出脚本。")
        rclpy.shutdown()
        return

    node = QueueTrajectoryPlayback()

    try:
        # 等待所有服务就绪
        node.get_logger().info("等待 ROS2 服务就绪...")
        for cli, name in [
            (node.cli_connect, "/tl_driver/connect_arm"),
            (node.cli_power_on, "/tl_driver/power_on"),
            (node.cli_coord_transform, "/tl_driver/coord_transform"),
            (node.cli_set_speed, "/tl_driver/set_speed"),
            (node.cli_queue_set_status, "/tl_driver/queue_motion_set_status"),
            (node.cli_queue_movej, "/tl_driver/queue_motion_movej"),
        ]:
            node.wait_for_service(cli, name)
        node.get_logger().info("所有服务已就绪")

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
