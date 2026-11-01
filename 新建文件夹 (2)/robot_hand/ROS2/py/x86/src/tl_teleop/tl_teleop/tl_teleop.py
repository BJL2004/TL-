#!/usr/bin/env python3
"""天链机械臂遥操作节点（通过 tl_driver ROS2 接口控制）。

将 VR 手柄位姿映射为机械臂末端运动指令，通过 tl_driver 的话题/服务
实现实时跟随控制。连接、上下电由 tl_driver 负责。

所有可调参数通过 ROS2 参数系统加载，配合 config/*.yaml 和 launch/*.launch.py 使用：
  - 6轴机械臂：ros2 launch tl_teleop tl_teleop_6axis.launch.py
  - 7轴机械臂：ros2 launch tl_teleop tl_teleop_7axis.launch.py
  - 通用（指定臂型）：ros2 launch tl_teleop tl_teleop.launch.py arm_type:=tcb605
"""

import time
import threading
import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from tl_ros2_interface.msg import CartesianPose
from tl_ros2_interface.srv import (
    CoordTransform,
    GetPosTransform,
    OpenServoJ,
    SetCurrentMode,
    SetSpeed,
)

import xrobotoolkit_sdk as xrt

# ===================== 运行时全局状态（非配置项，保持不变）=====================
pose_data = [0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
grip_data = 0.0
vr_home_pose = None
base_arm_quat = None
vr_home_quat = None
# 使用 threading.Event 替代无保护的 bool 标志，避免竞态条件
shutdown_event = threading.Event()

data_lock = threading.Lock()


# ========== VR 读取线程 ==========
def device_read_thread():
    """循环读取 VR 手柄位姿和握力数据。"""
    global pose_data, grip_data
    try:
        xrt.init()
    except Exception as e:
        print(f"[ERROR] 遥感设备初始化失败: {e}")
        return

    while not shutdown_event.is_set():
        try:
            pose = xrt.get_right_controller_pose()
            grip = xrt.get_right_grip()
            with data_lock:
                pose_data = pose
                grip_data = grip
            time.sleep(0.01)
        except Exception:
            time.sleep(0.05)
            continue
    xrt.close()
    print("[INFO] 遥感设备已断开")


# ========== 四元数辅助函数（纯数学，本地计算）==========
def quat2rpy(q):
    """四元数 → RPY 欧拉角（w,x,y,z 顺序）。"""
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1, min(1, sinp)))
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def quat_multiply(q1, q2):
    """四元数乘法 q1 * q2（w,x,y,z 顺序）。"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return [w, x, y, z]


def quat_inverse(q):
    """四元数共轭（逆，假设 q 为单位四元数）。"""
    w, x, y, z = q
    return [w, -x, -y, -z]


# ========== ROS2 遥操作节点 ==========
class ArmTeleopNode(Node):
    """遥操作 ROS2 节点，封装所有 tl_driver 服务客户端和话题通信。

    所有可调参数通过 ROS2 参数系统加载（declare_parameter）。
    模块级全局常量已移除，改为实例属性。
    """

    def __init__(self):
        super().__init__("tl_teleop_node")

        # ========== 声明 ROS2 参数（默认值无效，必须通过 YAML 配置文件提供）==========
        self.declare_parameter("arm_axis_mode", 0)
        # 位置控制
        self.declare_parameter("pos_scale", 0.5)
        self.declare_parameter("pos_deadzone", 0.005)
        self.declare_parameter("max_pos_delta_mm", 300.0)
        # 日志
        self.declare_parameter("log_interval", 1.0)
        # 奇异点防护
        self.declare_parameter("joint_jump_threshold", 30.0)
        self.declare_parameter("singular_angle", 160.0)
        self.declare_parameter("singular_scale", 0.2)
        self.declare_parameter("joint_limits", [0.0])
        # ServoJ 初始化
        self.declare_parameter("servo_speed", 25.0)
        self.declare_parameter("servo_vmax", 80.0)
        self.declare_parameter("servo_amax", 3000.0)
        self.declare_parameter("servo_jmax", 50000.0)

        # ========== 读取并校验关键参数 ==========
        self.arm_axis_mode_ = (
            self.get_parameter("arm_axis_mode").get_parameter_value().integer_value
        )
        if self.arm_axis_mode_ not in (6, 7):
            self.get_logger().fatal(
                f"arm_axis_mode 必须为 6 或 7，当前值: {self.arm_axis_mode_}。"
                f"请通过 YAML 配置文件提供正确的 arm_axis_mode 参数。"
            )
            raise ValueError(f"arm_axis_mode 必须为 6 或 7，当前值: {self.arm_axis_mode_}")

        # 关节限位：从 YAML 扁平数组解析，必须与 arm_axis_mode 匹配
        raw_limits = self.get_parameter("joint_limits").get_parameter_value().double_array_value
        expected_len = self.arm_axis_mode_ * 2
        if not raw_limits or len(raw_limits) != expected_len:
            self.get_logger().fatal(
                f"joint_limits 与 arm_axis_mode 不匹配："
                f"arm_axis_mode={self.arm_axis_mode_}，"
                f"期望 {expected_len} 个限位值，"
                f"实际提供了 {len(raw_limits) if raw_limits else 0} 个。"
                f"请检查 YAML 配置文件中的 joint_limits 参数。"
            )
            raise ValueError(
                f"joint_limits 长度不匹配：期望 {expected_len}，"
                f"实际 {len(raw_limits) if raw_limits else 0}"
            )
        self.joint_limits_ = [
            [raw_limits[i * 2], raw_limits[i * 2 + 1]] for i in range(self.arm_axis_mode_)
        ]

        # 关节名称根据轴数自动生成
        self.joint_names = [f"joint{i+1}" for i in range(self.arm_axis_mode_)]

        # ========== 读取其余参数 ==========
        self.pos_scale_ = self.get_parameter("pos_scale").get_parameter_value().double_value
        self.pos_deadzone_ = self.get_parameter("pos_deadzone").get_parameter_value().double_value
        self.max_pos_delta_mm_ = (
            self.get_parameter("max_pos_delta_mm").get_parameter_value().double_value
        )
        self.log_interval_ = self.get_parameter("log_interval").get_parameter_value().double_value
        self.joint_jump_threshold_ = (
            self.get_parameter("joint_jump_threshold").get_parameter_value().double_value
        )
        self.singular_angle_ = (
            self.get_parameter("singular_angle").get_parameter_value().double_value
        )
        self.singular_scale_ = (
            self.get_parameter("singular_scale").get_parameter_value().double_value
        )
        self.servo_speed_ = self.get_parameter("servo_speed").get_parameter_value().double_value
        self.servo_vmax_ = self.get_parameter("servo_vmax").get_parameter_value().double_value
        self.servo_amax_ = self.get_parameter("servo_amax").get_parameter_value().double_value
        self.servo_jmax_ = self.get_parameter("servo_jmax").get_parameter_value().double_value

        # --- 服务客户端 ---
        self.set_current_mode_client = self.create_client(
            SetCurrentMode, "/tl_driver/set_current_mode"
        )
        self.set_speed_client = self.create_client(SetSpeed, "/tl_driver/set_speed")
        self.open_servoj_client = self.create_client(OpenServoJ, "/tl_driver/open_servoj")
        self.close_servoj_client = self.create_client(Trigger, "/tl_driver/close_servoj")
        self.coord_transform_client = self.create_client(
            CoordTransform, "/tl_driver/coord_transform"
        )
        self.get_rpy2quat_client = self.create_client(GetPosTransform, "/tl_driver/get_rpy2quat")

        # --- 话题发布者 ---
        self.servoj_pos_pub = self.create_publisher(
            Float64MultiArray, "/tl_driver/set_servoj_pos", 10
        )
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # --- 话题订阅者 ---
        self._tcp_lock = threading.Lock()
        self._tcp_position = None  # [x, y, z] mm
        self._tcp_rpy = None  # [r, p, y] 弧度
        self.tcp_pose_sub = self.create_subscription(
            CartesianPose, "/tcp_pose", self._tcp_pose_cb, 10
        )

        # --- 关节状态 ---
        self.last_joints = None
        self.last_log_time = 0.0
        self._target_lock = threading.Lock()
        self._latest_target_joints = [0.0] * self.arm_axis_mode_

        # --- 定时器 ---
        self.create_timer(0.05, self._publish_joints)

        self.get_logger().info(f"遥操作节点启动（{self.arm_axis_mode_}轴模式）")

        # --- 等待 tl_driver 服务就绪 ---
        self.get_logger().info("等待 tl_driver 服务就绪...")
        if not self.wait_for_services(timeout=15.0):
            self.get_logger().fatal("tl_driver 服务不可用，请先启动 tl_driver")
            raise RuntimeError("tl_driver 服务不可用，请先启动 tl_driver")

    # -------- 话题回调 --------
    def _tcp_pose_cb(self, msg: CartesianPose):
        """缓存最新 TCP 位姿。"""
        with self._tcp_lock:
            self._tcp_position = [msg.position.x, msg.position.y, msg.position.z]
            self._tcp_rpy = [msg.rpy.x, msg.rpy.y, msg.rpy.z]

    # -------- 定时器 --------
    def _publish_joints(self):
        """发布目标关节状态（用于 RViz 可视化）。"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        with self._target_lock:
            msg.position = [math.radians(p) for p in self._latest_target_joints]
        self.joint_pub.publish(msg)

    # -------- 服务检查 --------
    def wait_for_services(self, timeout: float = 10.0) -> bool:
        """等待所有必要服务就绪。"""
        services = [
            (self.set_current_mode_client, "/tl_driver/set_current_mode"),
            (self.set_speed_client, "/tl_driver/set_speed"),
            (self.open_servoj_client, "/tl_driver/open_servoj"),
            (self.close_servoj_client, "/tl_driver/close_servoj"),
            (self.coord_transform_client, "/tl_driver/coord_transform"),
            (self.get_rpy2quat_client, "/tl_driver/get_rpy2quat"),
        ]
        all_ready = True
        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f"服务 {name} 不可用")
                all_ready = False
        return all_ready

    # -------- ServoJ 初始化序列 --------
    def init_servoj(self) -> bool:
        """执行 ServoJ 初始化：设置模式 → 速度 → 开启 ServoJ。

        速度参数从 ROS2 参数 `servo_speed` 读取。
        ServoJ 动力学限制从 `servo_vmax`、`servo_amax`、`servo_jmax` 读取。
        """
        # 1) 设置运行模式
        req = SetCurrentMode.Request()
        req.mode = 2
        result = self.set_current_mode_client.call(req)
        if not result.success:
            self.get_logger().error(f"设置模式失败: {result.message}")
            return False
        self.get_logger().info("模式已设为 2（运行）")

        # 2) 设置速度
        req_s = SetSpeed.Request()
        req_s.speed = self.servo_speed_
        result = self.set_speed_client.call(req_s)
        if not result.success:
            self.get_logger().error(f"设置速度失败: {result.message}")
            return False
        self.get_logger().info(f"速度已设为 {self.servo_speed_}")

        # 3) 开启 ServoJ
        req_o = OpenServoJ.Request()
        req_o.vmax = [self.servo_vmax_] * 7
        req_o.amax = [self.servo_amax_] * 7
        req_o.jmax = [self.servo_jmax_] * 7
        result = self.open_servoj_client.call(req_o)
        if not result.success:
            self.get_logger().error(f"开启 ServoJ 失败: {result.message}")
            return False
        self.get_logger().info("ServoJ 已开启")

        return True

    def close_servoj_async_call(self):
        """异步发起关闭 ServoJ 请求，立即返回 Future。

        调用者需通过 spin_until_future_complete 等待 Future 完成，
        并检查 future.result() 来确认 ServoJ 是否已关闭。

        Returns:
            rclpy Future 对象，若请求发送失败则返回 None。
        """
        req = Trigger.Request()
        try:
            future = self.close_servoj_client.call_async(req)
            return future
        except Exception as e:
            self.get_logger().error(f"发起关闭 ServoJ 请求异常: {e}")
            return None

    def close_servoj(self, timeout_sec: float = 3.0) -> bool:
        """关闭 ServoJ 模式（便捷同步版本，带超时保护）。

        内部通过 close_servoj_async_call() 发起异步请求，
        再用临时 SingleThreadedExecutor spin 等待结果。

        注意：调用前必须确保本 node 未被其他 executor 持有
        （即已 executor.remove_node()），否则 add_node 会失败。

        Args:
            timeout_sec: 等待 ServoJ 关闭完成的超时时间（秒）。

        Returns:
            True 表示 ServoJ 已成功关闭。
            False 表示超时、请求失败或服务端返回失败。
            调用者可通过日志确认具体原因。
        """
        future = self.close_servoj_async_call()
        if future is None:
            return False

        tmp_executor = SingleThreadedExecutor()
        tmp_executor.add_node(self)
        try:
            tmp_executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        finally:
            tmp_executor.remove_node(self)

        if not future.done():
            self.get_logger().warning(f"关闭 ServoJ 超时 ({timeout_sec}s)，可能未成功关闭")
            return False

        result = future.result()
        if result is not None and result.success:
            self.get_logger().info("ServoJ 已成功关闭 ✓")
            return True
        else:
            msg = result.message if result else "无响应"
            self.get_logger().warning(f"关闭 ServoJ 返回失败: {msg}")
            return False

    # -------- RPY → 四元数（通过服务） --------
    def call_get_rpy2quat(self, rx: float, ry: float, rz: float):
        """调用 tl_driver 的 get_rpy2quat 服务。返回 [w, x, y, z] 或 None。"""
        req = GetPosTransform.Request()
        req.input = [float(rx), float(ry), float(rz)]
        try:
            result = self.get_rpy2quat_client.call(req)
            if result.success:
                return list(result.output)  # [w, x, y, z]
            self.get_logger().warning(f"rpy2quat 失败: {result.message}")
        except Exception as e:
            self.get_logger().error(f"rpy2quat 异常: {e}")
        return None

    # -------- 逆运动学（通过 coord_transform 服务） --------
    def call_inverse_kinematics(self, x, y, z, rx, ry, rz):
        """调用 tl_driver 的 coord_transform 服务做运动学逆解。

        将直角坐标系位姿（x/y/z mm，rx/ry/rz rad）转换为关节角度（度），
        返回关节角列表（长度 = arm_axis_mode），失败返回 None。
        """
        req = CoordTransform.Request()
        req.origin_coord = 1  # 直角坐标系（输入）
        req.target_coord = 0  # 关节坐标系（输出）
        req.form = 0  # 逆解模式
        # 位姿：[x(mm), y(mm), z(mm), rx(rad), ry(rad), rz(rad), arm_angle(rad)]
        req.origin_pos = [float(x), float(y), float(z), float(rx), float(ry), float(rz), 0.0]
        req.reference_pos = [0.0] * 7

        try:
            result = self.coord_transform_client.call(req)
            if result.success and len(result.target_pos) >= self.arm_axis_mode_:
                full_joints = list(result.target_pos)
                return full_joints[: self.arm_axis_mode_]
            if not result.success:
                self.get_logger().warning(f"逆运动学失败: {result.message}")
        except Exception as e:
            self.get_logger().error(f"逆运动学调用异常: {e}")
        return None

    # -------- TCP 位姿查询 --------
    def get_tcp_pose(self):
        """获取缓存的 TCP 位姿。返回 ([x, y, z], [rx, ry, rz]) 或 (None, None)。"""
        with self._tcp_lock:
            if self._tcp_position is None or self._tcp_rpy is None:
                return None, None
            return list(self._tcp_position), list(self._tcp_rpy)

    # -------- 伺服关节下发 --------
    def servoJ_send(self, joint_target):
        """安全校验后通过话题发布目标关节角度。"""
        # 硬限位裁剪
        joint_target = self.clamp_joints(joint_target)

        # 突变检查
        if self.last_joints is not None:
            for a, b in zip(joint_target, self.last_joints):
                if abs(a - b) > self.joint_jump_threshold_:
                    self.get_logger().warn(f"关节突变过大: {abs(a - b):.1f}°，已丢弃")
                    return
        self.last_joints = joint_target.copy()

        # 更新目标关节（供可视化发布使用）
        with self._target_lock:
            self._latest_target_joints = joint_target.copy()

        # 6轴模式补零至7维（底层 SDK 固定7轴参数）
        if self.arm_axis_mode_ == 6:
            send_joints = joint_target + [0.0]
        else:
            send_joints = joint_target

        # 发布到 tl_driver
        msg = Float64MultiArray()
        msg.data = [float(v) for v in send_joints]
        self.servoj_pos_pub.publish(msg)

        # 日志打印
        now = time.time()
        if now - self.last_log_time > self.log_interval_:
            self.get_logger().info(f"[JOINT] {[round(i, 3) for i in joint_target]}")
            self.last_log_time = now

    def clamp_joints(self, joints):
        """将关节角裁剪到配置的硬限位范围内。"""
        return [max(low, min(high, val)) for val, (low, high) in zip(joints, self.joint_limits_)]


# ========== 纯控制循环（100Hz）==========
def control_loop_func(node: ArmTeleopNode):
    """遥操作控制循环：读 VR 位姿 → IK → 关节下发。

    所有可调参数通过 node 实例属性读取（由 ROS2 参数系统填充）。
    """
    global vr_home_pose, base_arm_quat, vr_home_quat

    base_x = base_y = base_z = 0.0
    base_rx = base_ry = base_rz = 0.0

    while not shutdown_event.is_set():
        t0 = time.time()
        with data_lock:
            pose = pose_data.copy()
            grip = grip_data
        x, y, z, qx, qy, qz, qw = pose
        current_vr_quat = [qx, qy, qz, qw]

        # A 键回零
        try:
            if xrt.get_A_button():
                zero_joints = [0.0] * node.arm_axis_mode_
                node.servoJ_send(zero_joints)
                vr_home_pose = None
                base_arm_quat = None
                vr_home_quat = None
                node.last_joints = None
                time.sleep(1)
                continue
        except Exception:
            pass

        # 握紧：记录基准位姿
        if grip > 0.9:
            if vr_home_pose is None:
                pos, rpy = node.get_tcp_pose()
                if pos is not None and rpy is not None and len(pos) >= 3:
                    base_x = pos[0]
                    base_y = pos[1]
                    base_z = pos[2]
                    base_rx = rpy[0]
                    base_ry = rpy[1]
                    base_rz = rpy[2]
                    base_arm_quat = node.call_get_rpy2quat(base_rx, base_ry, base_rz)
                    vr_home_pose = [x, y, z]
                    vr_home_quat = current_vr_quat.copy()
        else:
            vr_home_pose = None
            base_arm_quat = None
            vr_home_quat = None

        # 遥操作控制
        if vr_home_pose is not None and base_arm_quat is not None and vr_home_quat is not None:
            dx = x - vr_home_pose[0]
            dy = y - vr_home_pose[1]
            dz = z - vr_home_pose[2]
            if abs(dx) < node.pos_deadzone_:
                dx = 0
            if abs(dy) < node.pos_deadzone_:
                dy = 0
            if abs(dz) < node.pos_deadzone_:
                dz = 0

            # 奇异区减速
            if node.last_joints is not None:
                if node.arm_axis_mode_ == 6:
                    j5, j6 = node.last_joints[4], node.last_joints[5]
                    scale = (
                        node.singular_scale_
                        if abs(j5) > node.singular_angle_ or abs(j6) > node.singular_angle_
                        else 1.0
                    )
                else:
                    j6, j7 = node.last_joints[5], node.last_joints[6]
                    scale = (
                        node.singular_scale_
                        if abs(j6) > node.singular_angle_ or abs(j7) > node.singular_angle_
                        else 1.0
                    )
            else:
                scale = 1.0

            arm_delta_X = -dz * 1000 * node.pos_scale_ * scale
            arm_delta_Y = -dx * 1000 * node.pos_scale_ * scale
            arm_delta_Z = dy * 1000 * node.pos_scale_ * scale
            arm_delta_X = max(-node.max_pos_delta_mm_, min(node.max_pos_delta_mm_, arm_delta_X))
            arm_delta_Y = max(-node.max_pos_delta_mm_, min(node.max_pos_delta_mm_, arm_delta_Y))
            arm_delta_Z = max(-node.max_pos_delta_mm_, min(node.max_pos_delta_mm_, arm_delta_Z))

            target_x = base_x + arm_delta_X
            target_y = base_y + arm_delta_Y
            target_z = base_z + arm_delta_Z

            # 姿态最短旋转
            vr_quat_wxyz = [qw, qx, qy, qz]
            vr_home_quat_wxyz = [vr_home_quat[3], vr_home_quat[0], vr_home_quat[1], vr_home_quat[2]]
            if sum(a * b for a, b in zip(vr_quat_wxyz, vr_home_quat_wxyz)) < 0:
                vr_quat_wxyz = [-v for v in vr_quat_wxyz]

            vr_inv = quat_inverse(vr_home_quat_wxyz)
            vr_delta = quat_multiply(vr_quat_wxyz, vr_inv)

            wd, xd, yd, zd = vr_delta
            vr_delta_mapped = [wd, -zd, -xd, yd]
            target_quat = quat_multiply(vr_delta_mapped, base_arm_quat)
            target_rx, target_ry, target_rz = quat2rpy(target_quat)
            target_ry = -target_ry

            ik = node.call_inverse_kinematics(
                target_x, target_y, target_z, target_rx, target_ry, target_rz
            )
            if ik:
                node.servoJ_send(ik)

        dt = time.time() - t0
        if 0.01 - dt > 0:
            time.sleep(0.01 - dt)


def main_control_loop(node: ArmTeleopNode):
    """遥操作编排：初始化 ServoJ → 控制循环 → 关闭 ServoJ。

    在子线程中运行，spin 在主线程。初始化时的服务调用通过
    spin_until_future_complete 的临时 executor 处理。
    """
    # 初始化 ServoJ（模式 → 速度 → 开启）
    node.get_logger().info("正在初始化 ServoJ...")
    if not node.init_servoj():
        node.get_logger().fatal("ServoJ 初始化失败")
        return

    node.get_logger().info(f"{node.arm_axis_mode_}轴机械臂就绪，开始遥操作（已加固奇异点）")

    # 运行控制循环（阻塞直到 shutdown_event 被设置）
    control_loop_func(node)

    # 注意：ServoJ 关闭已移至 main() 中，
    # 在 rclpy shutdown 之前由主线程统一处理，
    # 确保关闭请求能被正确发送和处理。


# ========== 主入口 ==========
def main():
    """主入口：spin 在主线程，遥操作编排在子线程。

    线程分工：
      - 主线程：手动 spin_once 循环（处理话题/定时器/服务回调）
      - 编排子线程：main_control_loop（init → control_loop）
      - VR 读取子线程：device_read_thread

    关键设计变更：
      1. 使用 SingleThreadedExecutor.spin_once() 替代 rclpy.spin()
      2. 在初始化完成之后才覆盖 SIGINT handler，阻止 rclpy 自动 shutdown
      初始化阶段（wait_for_services 等）仍由 rclpy 默认 handler 处理，
      避免阻塞式 C 调用期间信号中断导致 rclpy 内部状态损坏。
      初始化完毕后接管 SIGINT，确保退出时能在 shutdown 之前
      执行 close_servoj() 等清理工作。
    """
    rclpy.init()

    try:
        node = ArmTeleopNode()
    except KeyboardInterrupt:
        # rclpy 默认 SIGINT handler 已调用 shutdown，无需再调
        return
    except (RuntimeError, ValueError):
        rclpy.shutdown()
        return

    # 启动 VR 读取线程
    vr_thread = threading.Thread(target=device_read_thread, daemon=True)
    vr_thread.start()

    # 启动遥操作编排子线程（内部完成 init_servoj → 控制循环）
    ctrl_thread = threading.Thread(target=main_control_loop, args=(node,), daemon=True)
    ctrl_thread.start()

    # --- 初始化完毕，现在接管 SIGINT，阻止 rclpy 自动 shutdown ---
    # rclpy.init() 注册的 SIGINT handler 会自动调用 rclpy.shutdown()，
    # 这会导致后续 close_servoj 等清理无法执行。
    # 必须在所有阻塞初始化（已在 __init__ 中完成）之后才接管，
    # 否则 SIGINT 在 C 调用期间会导致 rclpy 内部状态损坏。
    shutdown_requested = False

    def _handle_sigint(sig, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, _handle_sigint)

    # 手动 spin_once 循环：替代 rclpy.spin()，
    # 通过 shutdown_requested 标志控制退出时机
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not shutdown_requested:
            executor.spin_once(timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        # 某些环境下 spin_once 仍可能抛出这些异常
        pass

    # ===== 清理阶段：rclpy 此时仍然存活 =====
    # 所有步骤均有超时保护，确保总退出时间可控，避免被 launch 系统 SIGKILL 强杀

    # Phase 1: 停止控制循环
    # 信号通知所有子线程退出：控制循环停止下发指令，VR 线程准备断开设备
    shutdown_event.set()
    node.get_logger().info("等待控制循环停止...")
    ctrl_thread.join(timeout=3.0)
    if ctrl_thread.is_alive():
        node.get_logger().warning("控制循环线程未能在 3s 内退出")
    else:
        node.get_logger().info("控制循环已退出")

    # Phase 2: 关闭 ServoJ（异步 + 超时等待，可确认是否关闭成功）
    # 此时 executor 仍持有 node，直接用它 spin 等待异步结果
    close_future = node.close_servoj_async_call()
    if close_future is not None:
        executor.spin_until_future_complete(close_future, timeout_sec=3.0)
        if close_future.done():
            result = close_future.result()
            if result is not None and result.success:
                node.get_logger().info("ServoJ 已成功关闭 ✓")
            else:
                msg = result.message if result else "无响应"
                node.get_logger().warning(f"ServoJ 关闭返回失败: {msg}")
        else:
            node.get_logger().warning("ServoJ 关闭超时（3s），可能未成功关闭")
    else:
        node.get_logger().warning("无法发起关闭 ServoJ 请求")

    # Phase 3: 解除 executor 对 node 的绑定
    executor.remove_node(node)

    # Phase 4: 清理 VR 设备
    # VR 线程可能在 xrt.get_right_controller_pose() 阻塞 C 调用中，
    # 超时后由主线程兜底调用 xrt.close() 确保 SDK 资源释放
    vr_thread.join(timeout=2.0)
    if vr_thread.is_alive():
        node.get_logger().warning("VR 读取线程未能在 2s 内退出，将在主线程中清理 SDK")
        try:
            xrt.close()
            print("[INFO] 遥感设备已在主线程中断开")
        except Exception as e:
            print(f"[WARN] 主线程清理遥感设备异常: {e}")
    else:
        print("[INFO] 遥感设备已正常断开")

    # Phase 5: 最后关闭 rclpy
    rclpy.shutdown()
    print("[INFO] 程序结束")


if __name__ == "__main__":
    main()
