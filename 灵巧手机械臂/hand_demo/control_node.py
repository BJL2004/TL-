#!/usr/bin/env python3
import signal
import os
import sys
import time
import math
import select
import termios
import tty
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from scipy.spatial.transform import Rotation as R

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from tl_ros2_interface.msg import ObjectInfo, CartesianPose
from tl_ros2_interface.srv import CoordTransform, GetPosReachable, OpenServoJ, ModbusWrite, SetToolParam, SetCurrentCoord, SetCoordNum, GetSensor6DData, MarkBaseSensor
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

np.set_printoptions(precision=8, suppress=True)


class ObjectToBaseNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('robot_ip', '192.168.1.13')
        self.declare_parameter('robot_port', '6001')
        self.declare_parameter('camera_object_topic', '/tl_vision/object_3d_pos_camera')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('object_type', 'scissors')
        self.declare_parameter('grasp_offset', 0.2)
        self.declare_parameter('grasp_depth', 0.05)
        self.declare_parameter('approach_movetype', 'MOVJ')
        # ModbusRTU 参数（用于夹爪控制）
        self.declare_parameter('modbus_master_id', 1)
        self.declare_parameter('modbus_rtu_port', 2)
        self.declare_parameter('modbus_rtu_baudrate', 115200)
        self.declare_parameter('modbus_rtu_slave_id', 1)
        self.declare_parameter('modbus_rtu_data_bit', 8)
        self.declare_parameter('modbus_rtu_stop_bit', 1)
        self.declare_parameter('modbus_rtu_check_bit', 'None')
        self.declare_parameter('modbus_gripper_addrs', [1135, 1136, 1137, 1138, 1139, 1140])
        self.declare_parameter('modbus_gripper_close_val', [60000, 60000, 60000, 60000, 60000, 60000])
        self.declare_parameter('modbus_gripper_open_val', [0, 0, 0, 0, 0, 0])
        self.declare_parameter('z_min_limit', 68.0)   # Z 轴最低位置（mm）
        self.declare_parameter('grasp_target_z', 180.0)  # 抓取目标 Z（mm）
        self.declare_parameter('descend_y_offset', 0.0)   # 下降时Y方向手动补偿（mm）
        # 工具坐标系参数
        self.declare_parameter('tool_num', 1)
        self.declare_parameter('tool_user_num', 0)
        self.declare_parameter('tool_x', 0.0)
        self.declare_parameter('tool_y', 0.0)
        self.declare_parameter('tool_z', 0.0)
        self.declare_parameter('tool_a', 0.0)
        self.declare_parameter('tool_b', 0.0)
        self.declare_parameter('tool_c', 0.0)
        self.declare_parameter('angle_scale', 1.0)
        self.declare_parameter('angle_offset', 0.0)
        self.declare_parameter('servoj_interp_points', 2000)
        self.declare_parameter('servoj_vmax', [300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0])
        self.declare_parameter('servoj_amax', [3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0])
        self.declare_parameter('servoj_jmax', [50000.0, 50000.0, 50000.0, 50000.0, 50000.0, 50000.0, 50000.0])
        self.declare_parameter('servoj_arrival_threshold', 0.5)  # 到位判定阈值（度）
        self.declare_parameter(
            'zero_joint',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.declare_parameter(
            'after_grasp_waypoints',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description='抓取后依次经过的笛卡尔点位（每6个值一组: x,y,z,roll,pitch,yaw）'
            )
        )
        self.declare_parameter('auto_loop_enabled', False)
        self.declare_parameter('gripper_open_delay', 0.5)   # 张开夹爪后等待（秒）
        self.declare_parameter('gripper_close_delay', 0.5)  # 闭合夹爪后等待（秒）
        self.declare_parameter('pre_waypoint_delay', 1.0)   # 抓取完成到经过点位等待（秒）
        self.declare_parameter('post_waypoint_delay', 1.0)  # 到达放下点后张开夹爪等待（秒）
        self.declare_parameter('grasp_approach_z', 200.0)   # 抓取过渡Z（mm），先降到这个高度再降到grasp_target_z
        self.declare_parameter(
            'handeye_matrix',
            [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0
            ]
        )

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        self.camera_object_topic = self.get_parameter('camera_object_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.target_object_type = str(self.get_parameter('object_type').value)
        self.grasp_offset = float(self.get_parameter('grasp_offset').value)
        self.grasp_depth = float(self.get_parameter('grasp_depth').value)
        self.approach_movetype = str(self.get_parameter('approach_movetype').value)

        # ModbusRTU 参数
        self.modbus_master_id = int(self.get_parameter('modbus_master_id').value)
        self.modbus_rtu_port = int(self.get_parameter('modbus_rtu_port').value)
        self.modbus_rtu_baudrate = int(self.get_parameter('modbus_rtu_baudrate').value)
        self.modbus_rtu_slave_id = int(self.get_parameter('modbus_rtu_slave_id').value)
        self.modbus_rtu_data_bit = int(self.get_parameter('modbus_rtu_data_bit').value)
        self.modbus_rtu_stop_bit = int(self.get_parameter('modbus_rtu_stop_bit').value)
        self.modbus_rtu_check_bit = str(self.get_parameter('modbus_rtu_check_bit').value)
        self.modbus_gripper_addrs = list(self.get_parameter('modbus_gripper_addrs').value)
        self.modbus_gripper_close_val = list(self.get_parameter('modbus_gripper_close_val').value)
        self.modbus_gripper_open_val = list(self.get_parameter('modbus_gripper_open_val').value)
        self.angle_scale = float(self.get_parameter('angle_scale').value)
        self.angle_offset = float(self.get_parameter('angle_offset').value)
        self.z_min_limit = float(self.get_parameter('z_min_limit').value)  # mm
        self.grasp_target_z = float(self.get_parameter('grasp_target_z').value)  # mm
        self.descend_y_offset = float(self.get_parameter('descend_y_offset').value)  # mm
        # 工具坐标系参数
        self.tool_num = int(self.get_parameter('tool_num').value)
        self.tool_user_num = int(self.get_parameter('tool_user_num').value)
        self.tool_x = float(self.get_parameter('tool_x').value)
        self.tool_y = float(self.get_parameter('tool_y').value)
        self.tool_z = float(self.get_parameter('tool_z').value)
        self.tool_a = float(self.get_parameter('tool_a').value)
        self.tool_b = float(self.get_parameter('tool_b').value)
        self.tool_c = float(self.get_parameter('tool_c').value)
        self._gripper_closed = False  # 夹爪状态跟踪
        self.zero_joint = list(self.get_parameter('zero_joint').value)
        # 抓取后依次经过的笛卡尔点位（每6个值为 [x, y, z, roll, pitch, yaw]，mm + rad）
        wp_param = self.get_parameter('after_grasp_waypoints')
        if wp_param.type_ == ParameterType.PARAMETER_NOT_SET or wp_param.value is None:
            self.after_grasp_waypoints = []
        else:
            raw_wp = list(wp_param.value)
            self.after_grasp_waypoints = [raw_wp[i:i+6] for i in range(0, len(raw_wp), 6)]
        if self.after_grasp_waypoints:
            self.safe_log_info(f'加载 {len(self.after_grasp_waypoints)} 个抓取后经过点位')
            for i, wp in enumerate(self.after_grasp_waypoints):
                self.safe_log_info(f'  点位{i+1}: xyz=[{wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}]mm  rpy=[{wp[3]:.3f}, {wp[4]:.3f}, {wp[5]:.3f}]rad')

        self.auto_loop_enabled = bool(self.get_parameter('auto_loop_enabled').value)
        self.gripper_open_delay = float(self.get_parameter('gripper_open_delay').value)
        self.gripper_close_delay = float(self.get_parameter('gripper_close_delay').value)
        self.pre_waypoint_delay = float(self.get_parameter('pre_waypoint_delay').value)
        self.post_waypoint_delay = float(self.get_parameter('post_waypoint_delay').value)
        self.grasp_approach_z = float(self.get_parameter('grasp_approach_z').value)
        self.detection_collect_time = 0.5  # 收集检测结果的等待时间（秒）
        self.auto_loop_interrupt = False

        self.running = True
        self.cleaned = False

        self.detected_objects = []          # 检测队列 [{type, base_pos, cam_x, cam_y, angle, stamp}]
        self.latest_object_type = None
        self.latest_base_point = None        # 保留兼容（仅指向最新一个）
        self.latest_object_angle = 0.0
        self.latest_stamp = None

        # TCP 位姿缓存（来自 /tcp_pose 话题）
        self.latest_tcp_pose = None

        # 关节角度缓存（来自 /joint_states 话题）
        self.latest_joint_state = None

        # ServoJ 插值运动相关（从 ROS 参数读取）
        self.servoj_current_target = [0.0] * 7   # 当前定时器实际发送的值
        self.servoj_initialized = False           # 是否已从关节状态初始化
        self.servoj_interp_points = int(self.get_parameter('servoj_interp_points').value)
        self.servoj_arrival_threshold = float(self.get_parameter('servoj_arrival_threshold').value)
        self.servoj_current_final = None          # 当前轨迹的目标终点（到位检测用）
        self.servoj_trajectory = []               # 预生成的 N 个插值点
        self.servoj_traj_idx = 0                  # 当前发送到第几个点
        self.servoj_moving = False                # 是否正在发送轨迹中
        self.servoj_vmax = list(self.get_parameter('servoj_vmax').value)    # °/s
        self.servoj_amax = list(self.get_parameter('servoj_amax').value)    # °/s²
        self.servoj_jmax = list(self.get_parameter('servoj_jmax').value)    # °/s³
        self.servoj_timer = None

        # 键盘控制相关
        self.old_terminal_settings = None
        self.keyboard_fd = None
        self.keyboard_fd_need_close = False
        self.setup_keyboard()

        self.T_tool_camera = self.load_handeye_matrix_from_param()

        self.safe_log_info('========== T_tool_camera ==========')
        self.safe_log_info(f'\n{self.T_tool_camera}')
        self.safe_log_info('===================================')

        # 创建 ROS2 服务客户端（仅保留运行时所需服务）
        self._create_service_clients()

        # 创建 ROS2 话题订阅与发布
        self._create_topic_sub_pub()

        self.sub_object_camera = self.create_subscription(
            ObjectInfo,
            self.camera_object_topic,
            self.object_camera_callback,
            10
        )

        self.pub_object_base = self.create_publisher(
            ObjectInfo,
            '/tl_vision/object_3d_pos_base',
            10
        )

        self.safe_log_info('control_node 初始化完成')
        self.safe_log_info(f'订阅: {self.camera_object_topic}')
        self.safe_log_info(f'base_frame_id: {self.base_frame_id}')
        self.safe_log_info(f'机械臂: {self.robot_ip}:{self.robot_port}')
        self.safe_log_info(f'目标类型 object_type: {self.target_object_type}')
        self.safe_log_info(f'grasp_offset: {self.grasp_offset} m')
        self.safe_log_info(f'grasp_depth: {self.grasp_depth} m')
        self.safe_log_info(f'approach_movetype: {self.approach_movetype}')
        self.safe_log_info(f'ServoJ 插值点数: {self.servoj_interp_points}')
        self.safe_log_info(f'ServoJ vmax: {self.servoj_vmax}')
        hint = 't 夹爪开/闭 | h 抓取+经过点位 | p 经过点位 | f 显示力 | m 力标零 | z 回零 | c 中断循环 | q 退出'
        if self.auto_loop_enabled:
            hint = '自动循环模式 | c 中断循环 | z 回零 | q 退出'
        self.safe_log_info(f'按键: {hint}')

        # 设置工具参数 → 激活工具编号 → 切换工具坐标系 → 设置 ServoJ 速度限制
        self._call_set_tool_param_service()
        self._call_set_coord_num_service()
        self._call_set_tool_coord_service()
        self._call_open_servoj_service()

    def _create_service_clients(self):
        """创建运行时所需的 tl_driver 服务客户端。"""
        self.coord_transform_client = self.create_client(CoordTransform, '/tl_driver/coord_transform')
        self.get_pos_reachable_client = self.create_client(GetPosReachable, '/tl_driver/get_pos_reachable')
        self.open_servoj_client = self.create_client(OpenServoJ, '/tl_driver/open_servoj')
        self.modbus_write_client = self.create_client(ModbusWrite, '/tl_driver/modbus_write')
        self.set_tool_param_client = self.create_client(SetToolParam, '/tl_driver/set_tool_param')
        self.set_coord_num_client = self.create_client(SetCoordNum, '/tl_driver/set_coord_num')
        self.set_current_coord_client = self.create_client(SetCurrentCoord, '/tl_driver/set_current_coord')
        self.get_sensor_6d_client = self.create_client(GetSensor6DData, '/tl_driver/get_sensor_6d_data')
        self.mark_base_sensor_client = self.create_client(MarkBaseSensor, '/tl_driver/mark_base_sensor')

    def _call_open_servoj_service(self):
        """调用 /tl_driver/open_servoj 服务，传入 YAML 中配置的 vmax/amax/jmax。"""
        if not self.open_servoj_client.wait_for_service(timeout_sec=5.0):
            self.safe_log_warn('/tl_driver/open_servoj 服务不可用，跳过 open_servoj 调用')
            return

        req = OpenServoJ.Request()
        req.vmax = self.servoj_vmax
        req.amax = self.servoj_amax
        req.jmax = self.servoj_jmax

        try:
            future = self.open_servoj_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done() and future.result() is not None:
                resp = future.result()
                if resp.success:
                    self.safe_log_info(
                        f'open_servoj 成功 (vmax={self.servoj_vmax[0]}°/s)'
                    )
                else:
                    self.safe_log_warn(f'open_servoj 失败: {resp.message}')
            else:
                self.safe_log_warn('open_servoj 服务调用超时或无响应')
        except Exception as e:
            self.safe_log_error(f'open_servoj 服务调用异常: {e}')

    def _call_set_tool_param_service(self):
        """从 YAML 读取工具坐标系参数并调用 /tl_driver/set_tool_param 服务。"""
        if not self.set_tool_param_client.wait_for_service(timeout_sec=5.0):
            self.safe_log_warn('/tl_driver/set_tool_param 服务不可用，跳过工具坐标系设置')
            return

        from tl_ros2_interface.msg import ToolParam as ToolParamMsg
        req = SetToolParam.Request()
        req.tool_num = self.tool_num
        req.param = ToolParamMsg()
        req.param.x = self.tool_x
        req.param.y = self.tool_y
        req.param.z = self.tool_z
        req.param.a = self.tool_a
        req.param.b = self.tool_b
        req.param.c = self.tool_c

        try:
            future = self.set_tool_param_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done() and future.result() is not None:
                resp = future.result()
                if resp.success:
                    self.safe_log_info(
                        f'工具坐标系设置成功: tool_num={self.tool_num} '
                        f'xyz=[{self.tool_x}, {self.tool_y}, {self.tool_z}]mm '
                        f'abc=[{self.tool_a}, {self.tool_b}, {self.tool_c}]rad'
                    )
                else:
                    self.safe_log_warn(f'工具坐标系设置失败: {resp.message}')
            else:
                self.safe_log_warn('工具坐标系设置服务调用超时或无响应')
        except Exception as e:
            self.safe_log_error(f'工具坐标系设置服务调用异常: {e}')

    def _get_force_data(self):
        """获取六维力传感器去皮数据 (fx, fy, fz, mx, my, mz)，失败返回 None。"""
        if not self.get_sensor_6d_client.wait_for_service(timeout_sec=1.0):
            self.safe_log_warn('get_sensor_6d_data 服务不可用')
            return None
        req = GetSensor6DData.Request()
        try:
            future = self.get_sensor_6d_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() is not None and future.result().success:
                r = future.result()
                return (r.fx_sub_base, r.fy_sub_base, r.fz_sub_base,
                        r.mx_sub_base, r.my_sub_base, r.mz_sub_base)
        except Exception as e:
            self.safe_log_error(f'获取力数据异常: {e}')
        return None

    def _mark_base_sensor(self):
        """六维力传感器标零（去皮）。"""
        if not self.mark_base_sensor_client.wait_for_service(timeout_sec=1.0):
            self.safe_log_warn('mark_base_sensor 服务不可用')
            return False
        req = MarkBaseSensor.Request()
        try:
            future = self.mark_base_sensor_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() is not None and future.result().success:
                self.safe_log_info('六维力传感器标零成功')
                return True
        except Exception as e:
            self.safe_log_error(f'标零异常: {e}')
        return False

    def _call_set_coord_num_service(self):
        """激活工具编号，使工具参数生效。"""
        if not self.set_coord_num_client.wait_for_service(timeout_sec=5.0):
            self.safe_log_warn('/tl_driver/set_coord_num 服务不可用，跳过工具激活')
            return

        req = SetCoordNum.Request()
        req.tool_num = self.tool_num
        req.user_num = self.tool_user_num

        try:
            future = self.set_coord_num_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done() and future.result() is not None:
                resp = future.result()
                if resp.success:
                    self.safe_log_info(f'工具编号 {self.tool_num} 激活成功')
                else:
                    self.safe_log_warn(f'工具编号激活失败: {resp.message}')
            else:
                self.safe_log_warn('工具编号激活服务调用超时或无响应')
        except Exception as e:
            self.safe_log_error(f'工具编号激活服务调用异常: {e}')

    def _call_set_tool_coord_service(self):
        """切换到工具坐标系（coord=2），使工具参数生效。"""
        if not self.set_current_coord_client.wait_for_service(timeout_sec=5.0):
            self.safe_log_warn('/tl_driver/set_current_coord 服务不可用，跳过坐标系切换')
            return

        req = SetCurrentCoord.Request()
        req.coord = 2  # 2 = 工具坐标系

        try:
            future = self.set_current_coord_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done() and future.result() is not None:
                resp = future.result()
                if resp.success:
                    self.safe_log_info('已切换到工具坐标系（coord=2），tool_z偏移生效')
                else:
                    self.safe_log_warn(f'切换工具坐标系失败: {resp.message}')
            else:
                self.safe_log_warn('切换工具坐标系服务调用超时或无响应')
        except Exception as e:
            self.safe_log_error(f'切换工具坐标系服务调用异常: {e}')

    def _create_topic_sub_pub(self):
        """创建话题订阅、发布器与 ServoJ 定时器。"""
        self.tcp_pose_sub = self.create_subscription(
            CartesianPose,
            '/tcp_pose',
            self.tcp_pose_callback,
            10
        )
        # 订阅关节状态，获取当前关节角度（tl_driver 已 remap 到 /nano/joint_states）
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/nano/joint_states',
            self.joint_state_callback,
            10
        )
        # ServoJ 关节角度发布器（替换原来的 moveJ 发布器）
        self.servoj_pos_pub = self.create_publisher(
            Float64MultiArray,
            '/tl_driver/set_servoj_pos',
            10
        )
        # 发布关节状态给 robot_state_publisher（用于 RViz 显示）
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        # ServoJ 定时器（10ms = 100Hz，持续发送带插值的关节角度）
        self.servoj_timer = self.create_timer(0.01, self.servoj_timer_callback)

    def tcp_pose_callback(self, msg: CartesianPose):
        """缓存来自 /tcp_pose 话题的最新末端位姿。"""
        self.latest_tcp_pose = msg

    def joint_state_callback(self, msg: JointState):
        """缓存来自 /joint_states 话题的最新关节角度。"""
        self.latest_joint_state = msg

    def servoj_timer_callback(self):
        """定时器回调（100Hz）：从预生成轨迹中取点发送到 /tl_driver/set_servoj_pos。"""
        if not self.running:
            return

        # 首次运行时，用当前关节位置初始化 current_target（弧度→度）
        if not self.servoj_initialized and self.latest_joint_state is not None:
            current_pos_rad = list(self.latest_joint_state.position)
            current_pos_deg = [v * 180.0 / math.pi for v in current_pos_rad]
            self.servoj_current_target = (current_pos_deg + [0.0] * 7)[:7]
            self.servoj_initialized = True

        if not self.servoj_initialized:
            return

        # 从预生成轨迹中取下一个点发送
        if self.servoj_moving:
            if self.servoj_traj_idx < len(self.servoj_trajectory):
                self.servoj_current_target = self.servoj_trajectory[self.servoj_traj_idx]
                self.servoj_traj_idx += 1


                # 到位检测：所有关节与目标终点差 < 阈值，提前结束
                if self.servoj_current_final is not None:
                    max_diff = max(
                        abs(self.servoj_current_target[i] - self.servoj_current_final[i])
                        for i in range(7)
                    )
                    if max_diff < self.servoj_arrival_threshold:
                        self.servoj_current_target = list(self.servoj_current_final)
                        self.servoj_moving = False
                        self.servoj_current_final = None
                        self.safe_log_info(
                            f'ServoJ 到位 (最大偏差 {max_diff:.2f}°)，提前结束'
                        )
            else:
                self.servoj_moving = False
                self.servoj_current_final = None
                self.safe_log_info(
                    f'ServoJ 轨迹发送完成: '
                    f'共 {self.servoj_traj_idx} 点, '
                    f'耗时 {self.servoj_traj_idx * 0.01:.2f}s'
                )

        # 发布 ServoJ 关节角度到 tl_driver
        msg = Float64MultiArray()
        msg.data = self.servoj_current_target
        self.servoj_pos_pub.publish(msg)

        # 发布关节状态到 /joint_states（用于 RViz 显示）
        if self.latest_joint_state is not None:
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = list(self.latest_joint_state.name)
            # servoj_current_target 是度，转回弧度
            current_rad = [v * math.pi / 180.0 for v in self.servoj_current_target]
            js_msg.position = current_rad[:len(js_msg.name)]
            self.joint_state_pub.publish(js_msg)

    def safe_log_info(self, msg):
        if rclpy.ok():
            try:
                self.get_logger().info(str(msg))
            except Exception:
                print(msg)
        else:
            print(msg)

    def safe_log_warn(self, msg):
        if rclpy.ok():
            try:
                self.get_logger().warn(str(msg))
            except Exception:
                print(f'[WARN] {msg}')
        else:
            print(f'[WARN] {msg}')

    def safe_log_error(self, msg):
        if rclpy.ok():
            try:
                self.get_logger().error(str(msg))
            except Exception:
                print(f'[ERROR] {msg}')
        else:
            print(f'[ERROR] {msg}')

    def setup_keyboard(self):
        """
        修复 ros2 launch 下 sys.stdin 可能不是 tty，导致按 c 没反应的问题。

        优先使用 sys.stdin。
        如果 sys.stdin 不是 tty，则尝试打开 /dev/tty。
        """
        try:
            if sys.stdin.isatty():
                self.keyboard_fd = sys.stdin.fileno()
                self.keyboard_fd_need_close = False
            else:
                try:
                    self.keyboard_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
                    self.keyboard_fd_need_close = True
                except Exception:
                    self.keyboard_fd = None
                    self.keyboard_fd_need_close = False

            if self.keyboard_fd is None:
                self.safe_log_warn('未检测到可用终端，键盘控制不可用')
                return

            self.old_terminal_settings = termios.tcgetattr(self.keyboard_fd)
            tty.setcbreak(self.keyboard_fd)

            self.safe_log_info('键盘控制已启用: c 控制 | z 回零点 | q 退出')

        except Exception as e:
            print(f'键盘初始化失败: {e}')
            self.keyboard_fd = None
            self.keyboard_fd_need_close = False
            self.old_terminal_settings = None

    def restore_keyboard(self):
        try:
            if self.keyboard_fd is not None and self.old_terminal_settings is not None:
                try:
                    termios.tcsetattr(
                        self.keyboard_fd,
                        termios.TCSADRAIN,
                        self.old_terminal_settings
                    )
                except Exception:
                    pass

                self.old_terminal_settings = None

            if self.keyboard_fd is not None and self.keyboard_fd_need_close:
                try:
                    os.close(self.keyboard_fd)
                except Exception:
                    pass

            self.keyboard_fd = None
            self.keyboard_fd_need_close = False

        except Exception as e:
            print(f'恢复键盘异常: {e}')

    def read_key(self):
        try:
            if self.keyboard_fd is None:
                return None

            dr, _, _ = select.select([self.keyboard_fd], [], [], 0)

            if dr:
                data = os.read(self.keyboard_fd, 1)
                if data:
                    return data.decode(errors='ignore')

        except Exception:
            return None

        return None

    def load_handeye_matrix_from_param(self):
        try:
            handeye_list = list(self.get_parameter('handeye_matrix').value)

            if len(handeye_list) != 16:
                raise ValueError(
                    f'handeye_matrix 参数长度错误，期望 16，实际 {len(handeye_list)}'
                )

            T_tool_camera = np.array(
                handeye_list,
                dtype=np.float64
            ).reshape(4, 4)

            if not np.all(np.isfinite(T_tool_camera)):
                raise ValueError(
                    f'handeye_matrix 中存在 NaN 或 Inf:\n{T_tool_camera}'
                )

            expected_last_row = np.array(
                [0.0, 0.0, 0.0, 1.0],
                dtype=np.float64
            )

            if not np.allclose(T_tool_camera[3, :], expected_last_row, atol=1e-6):
                self.safe_log_warn(
                    f'handeye_matrix 最后一行不是 [0, 0, 0, 1]: '
                    f'{T_tool_camera[3, :]}'
                )

            return T_tool_camera

        except Exception as e:
            self.safe_log_error(f'读取 handeye_matrix 失败: {e}')
            self.cleanup_and_exit()

    def get_robot_pose(self):
        """从缓存的 /tcp_pose 话题数据获取当前末端位姿。"""
        if self.latest_tcp_pose is None:
            self.safe_log_warn('TCP pose 尚未接收到数据')
            return False, None

        try:
            # CartesianPose 位置单位为 mm，转换为米
            x = self.latest_tcp_pose.position.x / 1000.0
            y = self.latest_tcp_pose.position.y / 1000.0
            z = self.latest_tcp_pose.position.z / 1000.0

            # 从 CartesianPose.rpy 获取欧拉角（弧度）
            roll = self.latest_tcp_pose.rpy.x
            pitch = self.latest_tcp_pose.rpy.y
            yaw = self.latest_tcp_pose.rpy.z

            return True, [x, y, z, roll, pitch, yaw]

        except Exception as e:
            self.safe_log_error(f'获取位姿失败: {e}')
            return False, None

    def pose_to_tool_rt(self, pose):
        if pose is None or len(pose) < 6:
            raise ValueError(f'pose 长度不足: {pose}')

        t_tool = np.array(
            pose[0:3],
            dtype=np.float64
        ).reshape(3, 1)

        A, B, C = pose[3], pose[4], pose[5]

        rotation = R.from_euler(
            'XYZ',
            [A, B, C],
            degrees=False
        )

        R_tool = rotation.as_matrix()

        return R_tool, t_tool

    def pose_to_T_base_tool(self, pose):
        R_tool, t_tool = self.pose_to_tool_rt(pose)

        T_base_tool = np.eye(4, dtype=np.float64)
        T_base_tool[0:3, 0:3] = R_tool
        T_base_tool[0:3, 3] = t_tool.reshape(3)

        return T_base_tool

    def object_camera_callback(self, msg):
        if not self.running:
            return

        try:
            object_type = msg.type

            P_camera = np.array(
                [
                    msg.pos.point.x,
                    msg.pos.point.y,
                    msg.pos.point.z,
                    1.0
                ],
                dtype=np.float64
            )

            if not np.all(np.isfinite(P_camera)):
                self.safe_log_warn(
                    f'收到非法物体坐标，type={object_type}，跳过'
                )
                return

            success, robot_pose = self.get_robot_pose()

            if not success:
                self.safe_log_warn(
                    f'机械臂位姿获取失败，type={object_type}，跳过本次转换'
                )
                return

            T_base_tool = self.pose_to_T_base_tool(robot_pose)
            T_base_camera = T_base_tool @ self.T_tool_camera
            P_base = T_base_camera @ P_camera

            if not np.all(np.isfinite(P_base)):
                self.safe_log_warn(
                    f'转换后物体坐标非法，type={object_type}，跳过'
                )
                return

            trans_msg = ObjectInfo()
            trans_msg.type = object_type
            trans_msg.pos.header.stamp = msg.pos.header.stamp
            trans_msg.pos.header.frame_id = self.base_frame_id
            trans_msg.pos.point.x = float(P_base[0])
            trans_msg.pos.point.y = float(P_base[1])
            trans_msg.pos.point.z = float(P_base[2])

            self.pub_object_base.publish(trans_msg)

            now = time.time()
            # 加入检测队列（保留相机坐标用于排序）
            self.detected_objects.append({
                'type': str(object_type),
                'base_pos': np.array([P_base[0], P_base[1], P_base[2]], dtype=np.float64),
                'cam_x': float(msg.pos.point.x),
                'cam_y': float(msg.pos.point.y),
                'angle': msg.angle,
                'stamp': now,
            })
            self.latest_object_type = str(object_type)
            self.latest_base_point = np.array(
                [P_base[0], P_base[1], P_base[2]],
                dtype=np.float64
            )
            self.latest_object_angle = msg.angle
            self.latest_stamp = now

        except Exception as e:
            self.safe_log_error(f'物体坐标转换失败: {e}')

    def get_target_joint_from_cartesian(self, target_cart):
        """通过 /tl_driver/coord_transform 服务将笛卡尔位姿转为关节角度。"""
        req = CoordTransform.Request()
        req.origin_coord = 1  # 直角坐标
        req.target_coord = 0  # 关节坐标
        req.form = 0
        req.origin_pos = list(target_cart)
        req.reference_pos = [0.0] * 7

        # 重试机制：ServoJ 运动期间 NRC 可能繁忙，最多重试 3 次
        max_retries = 3
        for attempt in range(max_retries):
            try:
                future = self.coord_transform_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                result = future.result()

                if result.success:
                    return 0, list(result.target_pos)

                if attempt < max_retries - 1:
                    self.safe_log_warn(
                        f'笛卡尔转关节失败(第{attempt+1}次): {result.message}，重试...'
                    )
                    time.sleep(0.05)  # 等 50ms 再试
                else:
                    self.safe_log_error(
                        f'笛卡尔转关节失败(已重试{max_retries}次): {result.message}'
                    )

            except Exception as e:
                if attempt < max_retries - 1:
                    self.safe_log_warn(f'笛卡尔转关节异常(第{attempt+1}次): {e}，重试...')
                    time.sleep(0.05)
                else:
                    self.safe_log_error(f'笛卡尔转关节异常(已重试{max_retries}次): {e}')

        return -1, []

    def check_pos_reachable(self, point_values, coord=0, angle_unit=0, posture=1):
        """通过 /tl_driver/get_pos_reachable 服务检测关节位姿是否可达。"""
        # 构建 14 元素位姿数组: [coord, angle_unit, posture, 0,0,0,0, j1..j7]
        pos = [0.0] * 14
        pos[0] = float(coord)
        pos[1] = float(angle_unit)
        pos[2] = float(posture)
        for i in range(len(point_values)):
            pos[7 + i] = float(point_values[i])

        self.safe_log_info(f'可达性检测 pos14: {pos}')
        self.safe_log_info(f'可达性检测 movetype: {self.approach_movetype}')

        req = GetPosReachable.Request()
        req.pos = pos
        req.move_type = self.approach_movetype

        try:
            future = self.get_pos_reachable_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()

            self.safe_log_info(f'get_pos_reachable 返回: success={result.success}, message={result.message}')

            if not result.success:
                self.safe_log_warn(f'点位不可达: {result.message}')
                return False

            self.safe_log_info('点位可达')
            return True

        except Exception as e:
            self.safe_log_error(f'可达性判断异常: {e}')
            return False

    def movej(self, joint_pos):
        """预生成 N 个插值点轨迹，定时器依次发送。"""
        try:
            joint_list = list(joint_pos)

            if len(joint_list) < 7:
                joint_list = joint_list + [0.0] * (7 - len(joint_list))

            if not self.servoj_initialized:
                self.safe_log_warn('ServoJ 尚未初始化（未收到关节数据），跳过运动')
                return False

            final_target = joint_list[:7]
            start = list(self.servoj_current_target)

            # 预生成 N 个插值点
            n = self.servoj_interp_points
            self.servoj_trajectory = []
            for step in range(1, n + 1):
                ratio = step / n
                point = [
                    start[i] + (final_target[i] - start[i]) * ratio
                    for i in range(7)
                ]
                self.servoj_trajectory.append(point)

            self.servoj_current_final = final_target  # 记录终点
            self.servoj_traj_idx = 0
            self.servoj_moving = True
            return True

        except Exception as e:
            self.safe_log_error(f'movej 异常: {e}')
            return False

    def execute_control(self):
        if self.latest_base_point is None:
            self.safe_log_warn('当前没有检测到目标物体')
            return False

        if self.latest_object_type != self.target_object_type:
            self.safe_log_warn(
                f'当前物体类型为 {self.latest_object_type}，'
                f'目标类型为 {self.target_object_type}，不执行控制'
            )
            return False

        p = self.latest_base_point.copy()

        # 第6轴 yaw = detected_angle × angle_scale + angle_offset
        angle = self.latest_object_angle if hasattr(self, 'latest_object_angle') else 0.0
        yaw = angle * self.angle_scale + self.angle_offset
        target_z = float((p[2] + self.grasp_offset) * 1000.0)
        # Z 轴最低位置保护
        if target_z < self.z_min_limit:
            target_z = self.z_min_limit
            self.safe_log_warn(
                f'execute_control: Z 轴目标 {float((p[2] + self.grasp_offset) * 1000.0):.1f}mm '
                f'低于限制 ({self.z_min_limit:.0f}mm)，已限制为 {target_z:.1f}mm'
            )
        target_cart = [
            float(p[0] * 1000.0),
            float(p[1] * 1000.0),
            target_z,
            -3.14, 0.0, yaw, 0.0
        ]

        self.safe_log_info(
            f'目标笛卡尔坐标: '
            f'[{target_cart[0]:.3f}, {target_cart[1]:.3f}, {target_cart[2]:.3f}, '
            f'{target_cart[3]:.3f}, {target_cart[4]:.3f}, {target_cart[5]:.3f}, {target_cart[6]:.3f}]'
            f'  |  yaw={math.degrees(yaw):.1f}°'
        )

        ret, target_joint = self.get_target_joint_from_cartesian(target_cart)

        if ret != 0:
            self.safe_log_error(f'笛卡尔转关节失败: ret={ret}')
            return False

        joint_list = list(target_joint)

        if len(joint_list) < 7:
            self.safe_log_error(f'转换后的关节坐标长度不足: {joint_list}')
            return False

        self.safe_log_info(f'目标关节坐标: {joint_list}')

        if not self.check_pos_reachable(target_joint):
            self.safe_log_warn('目标位置不可达')
            return False

        self.safe_log_info('目标位置可达，开始运动')
        return self.movej(target_joint)

    def _collect_and_sort_detections(self):
        """等待并收集检测结果，按左上到右下排序（先Y后X，相机坐标系）。"""
        self.detected_objects.clear()
        # 等待一段时间让 YOLO 发布所有检测结果
        self._spin_sleep(self.detection_collect_time)
        if not self.detected_objects:
            return []
        # 按相机坐标排序：从上到下(Y小→大)，从左到右(X小→大)
        sorted_objs = sorted(self.detected_objects, key=lambda o: (o['cam_y'], o['cam_x']))
        self.safe_log_info(f'检测到 {len(sorted_objs)} 个物体，按左上→右下排序:')
        for i, obj in enumerate(sorted_objs):
            p = obj['base_pos']
            self.safe_log_info(
                f'  目标{i+1}: '
                f'cam=({obj["cam_x"]:.3f}, {obj["cam_y"]:.3f}) '
                f'base=({p[0]*1000:.1f}, {p[1]*1000:.1f}, {p[2]*1000:.1f})mm'
            )
        return sorted_objs

    def _spin_sleep(self, seconds):
        """非阻塞等待：持续调用 spin_once 确保定时器正常触发。"""
        end_time = time.time() + seconds
        while time.time() < end_time and self.running:
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except Exception:
                break

    def _wait_servoj_arrival(self, check_interval=0.5):
        """等待 ServoJ 运动到位，期间保持 spin_once 运行。
        check_interval: 每次检查到位状态的间隔（秒）。
        """
        last_check = time.time()
        while self.servoj_moving and self.running:
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except Exception:
                break
            # 每 check_interval 秒打印一次状态
            now = time.time()
            if now - last_check >= check_interval:
                last_check = now
                if self.latest_joint_state is not None and self.servoj_current_final is not None:
                    current_deg = [v * 180.0 / math.pi for v in self.latest_joint_state.position]
                    diffs = [abs(current_deg[i] - self.servoj_current_final[i]) for i in range(min(7, len(current_deg)))]
                    max_diff = max(diffs) if diffs else 0.0
                    self.safe_log_info(
                        f'  运动中... 最大关节偏差 {max_diff:.2f}° '
                        f'(阈值 {self.servoj_arrival_threshold:.1f}°)'
                    )

    def move_through_waypoints(self):
        """依次经过 after_grasp_waypoints 中所有笛卡尔点位（xyz mm + rpy rad）。"""
        if not self.after_grasp_waypoints:
            self.safe_log_warn('未配置 after_grasp_waypoints，跳过')
            return False

        self.safe_log_info(f'>> 开始依次经过 {len(self.after_grasp_waypoints)} 个点位')

        for i, wp in enumerate(self.after_grasp_waypoints):
            if len(wp) < 6:
                self.safe_log_warn(f'点位{i+1}数据不足（{len(wp)}个值），跳过')
                continue

            target_cart = [float(wp[0]), float(wp[1]), float(wp[2]),
                           float(wp[3]), float(wp[4]), float(wp[5]), 0.0]

            self.safe_log_info(
                f'>> 正在前往点位{i+1}/{len(self.after_grasp_waypoints)}: '
                f'[{target_cart[0]:.1f}, {target_cart[1]:.1f}, {target_cart[2]:.1f}, '
                f'{target_cart[3]:.3f}, {target_cart[4]:.3f}, {target_cart[5]:.3f}]'
            )

            ret, target_joint = self.get_target_joint_from_cartesian(target_cart)
            if ret != 0:
                self.safe_log_error(f'点位{i+1}笛卡尔转关节失败，跳过')
                continue

            joint_list = list(target_joint)
            if len(joint_list) < 7:
                self.safe_log_error(f'点位{i+1}关节坐标长度不足，跳过')
                continue

            if not self.check_pos_reachable(target_joint):
                self.safe_log_warn(f'点位{i+1}不可达，跳过')
                continue

            if not self.movej(target_joint):
                self.safe_log_error(f'点位{i+1}运动失败，停止后续点位')
                return False

            # 等待当前 ServoJ 运动完成（0.5s 检查一次到位状态）
            self._wait_servoj_arrival(check_interval=0.5)
            self.safe_log_info(f'点位{i+1}到达完成')

        self.safe_log_info('>> 所有点位经过完成')
        return True

    def move_zero(self):
        if len(self.zero_joint) < 7:
            zero_joint = list(self.zero_joint) + [0.0] * (7 - len(self.zero_joint))
        else:
            zero_joint = list(self.zero_joint[:7])

        self.safe_log_info(f'回零点: {zero_joint}')

        if not self.check_pos_reachable(zero_joint):
            self.safe_log_warn('零点位置不可达')
            return False

        return self.movej(zero_joint)

    def _build_modbus_rtu_master_param(self):
        """构建 ModbusRTU 的 MasterParam（从 YAML 参数读取配置）。"""
        from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam, ModbusTCPParam
        rtu = ModbusRTUParam()
        rtu.slave_id = self.modbus_rtu_slave_id
        rtu.port = self.modbus_rtu_port
        rtu.baudrate = self.modbus_rtu_baudrate
        rtu.data_bit = self.modbus_rtu_data_bit
        rtu.stop_bit = self.modbus_rtu_stop_bit
        rtu.check_bit = self.modbus_rtu_check_bit

        master = ModbusMasterParam()
        master.type = 'RTU'
        master.start_addr = True
        master.rtu = rtu
        return master

    def gripper_toggle(self):
        """切换夹爪开/闭（一次性批量写入所有寄存器，与 modbus_test.py 一致）。"""
        if self._gripper_closed:
            vals = self.modbus_gripper_open_val
            action = '打开'
        else:
            vals = self.modbus_gripper_close_val
            action = '关闭'

        addrs = self.modbus_gripper_addrs
        n = min(len(addrs), len(vals))
        self.safe_log_info(f'>> 夹爪 {action}: 起始地址 {addrs[0]} 批量写入 {n} 个值 {vals[:n]}')

        req = ModbusWrite.Request()
        req.master_id = self.modbus_master_id
        req.addr = addrs[0]           # 起始地址
        req.data = vals[:n]           # 一次性写入所有值
        req.master_param = self._build_modbus_rtu_master_param()

        ok = True
        if not self.modbus_write_client.wait_for_service(timeout_sec=1.0):
            self.safe_log_warn('modbus_write 服务不可用')
            ok = False
        else:
            try:
                future = self.modbus_write_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                if not (future.done() and future.result() is not None and future.result().success):
                    self.safe_log_error(f'批量写入失败')
                    ok = False
            except Exception as e:
                self.safe_log_error(f'modbus_write 异常: {e}')
                ok = False

        if ok:
            self._gripper_closed = not self._gripper_closed
            self.safe_log_info(f'夹爪 {action} 成功')
        else:
            self.safe_log_error(f'夹爪 {action} 失败')
        return ok

    def gripper_open(self):
        """强制打开夹爪（忽略当前状态）。"""
        self._gripper_closed = True  # 让 gripper_toggle 走打开分支
        return self.gripper_toggle()

    def gripper_close(self):
        """强制关闭夹爪（忽略当前状态）。"""
        self._gripper_closed = False
        return self.gripper_toggle()

    def grasp_at_target(self):
        """下降到目标 Z 位置 → 张开夹爪 → 闭合夹爪抓取。"""
        if self.latest_tcp_pose is None:
            self.safe_log_warn('TCP pose 尚未接收到数据，无法执行抓取')
            return False

        cx = self.latest_tcp_pose.position.x
        cy = self.latest_tcp_pose.position.y
        cz = self.latest_tcp_pose.position.z

        target_z = self.grasp_target_z  # 从 YAML 读取目标 Z（mm）
        if target_z < self.z_min_limit:
            target_z = self.z_min_limit
            self.safe_log_warn(f'抓取目标Z低于限制，已限制为 {target_z:.1f}mm')

        target_cart = [
            cx, cy + self.descend_y_offset,
            target_z,
            3.13, 0.92, 0.0, 0.0
        ]

        self.safe_log_info(f'>> 抓取: 从 Z={cz:.1f}mm 下降到 Z={target_z:.1f}mm')
        ret, target_joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0:
            self.safe_log_error('抓取时笛卡尔转关节失败')
            return False

        if not self.movej(target_joint):
            self.safe_log_error('抓取时运动失败')
            return False

        # 等待 ServoJ 运动到位（夹爪到达抓取位置）
        self.safe_log_info('>> 等待机械臂到达抓取位置...')
        self._wait_servoj_arrival(check_interval=0.5)

        # 到达目标位置 → 先张开夹爪 → 再闭合抓取
        self.safe_log_info('>> 到达抓取位置，先张开夹爪')
        self.gripper_open()
        self._spin_sleep(self.gripper_open_delay)

        self.safe_log_info('>> 闭合夹爪抓取')
        self.gripper_close()
        self._spin_sleep(self.gripper_close_delay)

        # 抓取后读取力数据，估算物体质量
        force = self._get_force_data()
        if force is not None:
            # 抓取前后 Fz 差值 ≈ 物体重量（g=9.81 m/s²）
            mass = force[2] / 9.81 if abs(force[2]) > 0.1 else 0.0
            self.safe_log_info(
                f'抓取后六维力(去皮): Fz={force[2]:.1f} N  →  '
                f'物体质量 ≈ {mass:.2f} kg ({mass*1000:.0f} g)'
            )
        return True

    def descend(self):
        """从当前位置下降 grasp_depth（米），使用固定抓取姿态 [3.13, 0.92, 0.0]。"""
        if self.latest_tcp_pose is None:
            self.safe_log_warn('TCP pose 尚未接收到数据，无法下降')
            return False

        # 获取当前 TCP 位置（mm）
        cx = self.latest_tcp_pose.position.x
        cy = self.latest_tcp_pose.position.y
        cz = self.latest_tcp_pose.position.z

        # 固定抓取姿态（roll, pitch, yaw）
        GRASP_R = 3.13
        GRASP_P = 0.92
        GRASP_Y = 0.0

        target_z = cz - self.grasp_depth * 1000.0  # grasp_depth 米 → mm

        # Z 轴最低位置保护
        if target_z < self.z_min_limit:
            target_z = self.z_min_limit
            self.safe_log_warn(
                f'Z 轴下降超过限制 ({self.z_min_limit:.0f}mm)，已限制为 {target_z:.1f}mm'
            )

        target_cart = [
            cx, cy + self.descend_y_offset,
            target_z,
            GRASP_R, GRASP_P, GRASP_Y, 0.0
        ]

        self.safe_log_info(
            f'下降: Z {cz:.1f}mm → {target_cart[2]:.1f}mm '
            f'(降 {self.grasp_depth*1000:.0f}mm) Y偏移={self.descend_y_offset:+.1f}mm '
            f'姿态: [{GRASP_R}, {GRASP_P}, {GRASP_Y}]'
        )

        ret, target_joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0:
            self.safe_log_error(f'下降时笛卡尔转关节失败')
            return False

        return self.movej(target_joint)

    def handle_key(self, key):
        self.safe_log_info(f'收到按键: {repr(key)}')

        if key == 'c':
            self.auto_loop_interrupt = True
            self.servoj_moving = False  # 停止当前轨迹
            self.safe_log_info('>> 中断信号已发送，停在当前位置')

        elif key == 'g':
            self.safe_log_info('>> 按键 g 已禁用')

        elif key == 't':
            self.gripper_toggle()

        elif key == 'h':
            self._execute_grasp_sequence()

        elif key == 'p':
            self.safe_log_info('>> 手动触发：依次经过所有配置点位')
            self.move_through_waypoints()

        elif key == 'f':
            force = self._get_force_data()
            if force is not None:
                self.safe_log_info(
                    f'六维力(去皮): Fx={force[0]:.1f}  Fy={force[1]:.1f}  Fz={force[2]:.1f}  '
                    f'Mx={force[3]:.1f}  My={force[4]:.1f}  Mz={force[5]:.1f}'
                )
                # 计算物体质量（假设 Z 轴竖直，g=9.81）
                mass = force[2] / 9.81 if abs(force[2]) > 0.1 else 0.0
                self.safe_log_info(f'估算物体质量: {mass:.2f} kg ({mass*1000:.0f} g)')
            else:
                self.safe_log_error('获取六维力数据失败')

        elif key == 'm':
            self.safe_log_info('>> 六维力传感器标零（去皮）')
            self._mark_base_sensor()

        elif key == 'z':
            self.safe_log_info('按键 z: 回零点')
            self.move_zero()

        elif key == 'q':
            print('\n按键 q: 退出 control_node')
            self.running = False

    def _grasp_single_object(self, obj, obj_index, total):
        """抓取单个物体（两段下降 → 夹爪开闭）。"""
        p = obj['base_pos']
        self.safe_log_info(f'>> 抓取目标 {obj_index}/{total}: '
                           f'位置 [{p[0]*1000:.1f}, {p[1]*1000:.1f}, {p[2]*1000:.1f}]mm')

        GRASP_R = 3.13
        GRASP_P = 0.92
        GRASP_Y = 0.0
        obj_x = float(p[0] * 1000.0)
        obj_y = float(p[1] * 1000.0) + self.descend_y_offset

        # 过渡高度
        approach_z = max(self.grasp_approach_z, self.z_min_limit)
        target_cart = [obj_x, obj_y, approach_z, GRASP_R, GRASP_P, GRASP_Y, 0.0]
        ret, joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0 or not self.check_pos_reachable(joint) or not self.movej(joint):
            self.safe_log_warn(f'目标{obj_index}过渡高度不可达/运动失败，跳过')
            return False
        self._wait_servoj_arrival(0.5)
        if self.auto_loop_interrupt:
            return False

        # 抓取高度
        grasp_z = max(self.grasp_target_z, self.z_min_limit)
        target_cart = [obj_x, obj_y, grasp_z, GRASP_R, GRASP_P, GRASP_Y, 0.0]
        ret, joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0 or not self.check_pos_reachable(joint) or not self.movej(joint):
            self.safe_log_warn(f'目标{obj_index}抓取高度不可达/运动失败，跳过')
            return False
        self._wait_servoj_arrival(0.5)
        if self.auto_loop_interrupt:
            return False

        # 夹爪抓取
        self.safe_log_info(f'>> 目标{obj_index}: 张开夹爪')
        self.gripper_open()
        self._spin_sleep(self.gripper_open_delay)
        self.safe_log_info(f'>> 目标{obj_index}: 闭合夹爪')
        self.gripper_close()
        self._spin_sleep(self.gripper_close_delay)
        if self.auto_loop_interrupt:
            return False
        return True

    def _execute_grasp_sequence(self):
        """完整流程：检测→抓取→放下→回零→检测下一个... 直到无物体。"""
        self.safe_log_info('>> 循环抓取: 检测→抓取→放下→回零→检测下一个...')

        grasped_count = 0
        while not self.auto_loop_interrupt:
            # 1. 收集并排序检测结果，取第一个（最左上角）
            targets = self._collect_and_sort_detections()
            targets = [t for t in targets if t['type'] == self.target_object_type]
            if not targets:
                if grasped_count == 0:
                    self.safe_log_warn('没有检测到目标物体')
                    return False
                else:
                    self.safe_log_info(f'>> 已抓取 {grasped_count} 个，没有更多物体')
                    break

            obj = targets[0]  # 取排序后的第一个
            grasped_count += 1
            self.safe_log_info(f'>> 第{grasped_count}个目标: 开始抓取')

            if self.auto_loop_interrupt:
                break

            # 2. 抓取目标
            if not self._grasp_single_object(obj, grasped_count, '?'):
                self.safe_log_warn(f'第{grasped_count}个抓取失败')
                continue

            if self.auto_loop_interrupt:
                break

            # 3. 经过点位（携带物体到放下位置）
            self.safe_log_info(f'>> 第{grasped_count}个: 经过点位')
            self._spin_sleep(self.pre_waypoint_delay)
            self.move_through_waypoints()

            if self.auto_loop_interrupt:
                break

            # 4. 张开夹爪放下物体
            self.safe_log_info(f'>> 第{grasped_count}个: 放下')
            self.gripper_open()
            self._spin_sleep(self.post_waypoint_delay)

            # 读取力数据
            force = self._get_force_data()
            if force is not None:
                self.safe_log_info(f'第{grasped_count}个放下后六维力: Fz={force[2]:.1f}N')

            # 5. 回零，停顿让画面稳定，重新检测下一物体
            self.safe_log_info(f'>> 第{grasped_count}个完成，回零检测下一物体')
            self.move_zero()
            self._wait_servoj_arrival(0.5)
            self.safe_log_info('>> 停顿2秒等待画面稳定...')
            self._spin_sleep(2.0)

        self.safe_log_info(f'>> 共抓取 {grasped_count} 个物体')
        return grasped_count > 0

    def auto_loop(self):
        """自动循环：回零 → 检测物体 → 抓取+经过点位 → 回零 → ... 直到无物体。"""
        self.safe_log_info('>> ═══════════════ 启动自动循环模式 ═══════════════')
        self.auto_loop_interrupt = False

        cycle_count = 0
        while self.running and not self.auto_loop_interrupt:
            cycle_count += 1
            self.safe_log_info(f'>> ══ 自动循环 第{cycle_count}轮 ══')

            # 1. 回零
            self.safe_log_info('>> 回零位置')
            if not self.move_zero():
                self.safe_log_error('回零失败，停止自动循环')
                break
            self._wait_servoj_arrival(0.5)

            if self.auto_loop_interrupt:
                break

            if self.auto_loop_interrupt:
                break

            # 2. 执行抓取流程（内部会收集检测→排序→逐个抓取→经过点位→放下）
            self.safe_log_info('>> 执行抓取流程')
            if not self._execute_grasp_sequence():
                self.safe_log_info('>> 未检测到物体或抓取失败，停止自动循环')
                break

        self.safe_log_info('>> ═══════════════ 自动循环结束 ═══════════════')

    def run(self):
        # 自动循环模式：直接进入循环，不走键盘交互
        if self.auto_loop_enabled:
            self.auto_loop()
            self.cleanup()
            return

        while self.running:
            try:
                if not rclpy.ok():
                    break

                rclpy.spin_once(self, timeout_sec=0.05)

                key = self.read_key()
                if key is not None:
                    self.handle_key(key)

            except ExternalShutdownException:
                break

            except KeyboardInterrupt:
                print('\n用户中断 control_node')
                self.running = False
                break

            except Exception as e:
                if self.running and rclpy.ok():
                    self.safe_log_error(f'主循环异常: {e}')
                else:
                    print(f'主循环退出: {e}')
                break

        self.cleanup()

    def cleanup(self):
        if self.cleaned:
            return

        self.cleaned = True
        self.running = False

        print('\n正在清理 control_node 资源...')

        self.restore_keyboard()

        print('control_node 资源清理完成')

    def cleanup_and_exit(self):
        self.cleanup()
        raise SystemExit(1)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = ObjectToBaseNode()

        def signal_handler(sig, frame):
            print('\n收到中断信号，正在退出 control_node...')
            if node is not None:
                node.running = False

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        node.run()

    except KeyboardInterrupt:
        print('\n用户中断 control_node')

    except SystemExit:
        pass

    except Exception as e:
        print(f'control_node 程序异常: {e}')

    finally:
        if node is not None:
            try:
                node.cleanup()
            except Exception as e:
                print(f'清理资源异常: {e}')

            try:
                node.destroy_node()
            except Exception as e:
                print(f'销毁节点异常: {e}')

        if rclpy.ok():
            try:
                rclpy.shutdown()
                print('ROS2已关闭')
            except Exception as e:
                print(f'ROS2关闭异常: {e}')


if __name__ == '__main__':
    main()
