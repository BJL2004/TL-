#!/usr/bin/env python3
"""
ARM 版 control_node (Jetson Orin Nano 适配)
============================================
与 x86 VM 版的区别:
  1. 移除六维力传感器相关功能 (GetSensor6DData / MarkBaseSensor 在 ARM 不存在)
  2. SetToolParam 设为可选 (ARM ctypes 版可能不支持)
  3. 关节状态从 /joint_states 直接订阅 (tl_driver 直接发布，不 remap)
  4. 支持灵巧手 ROH-LiteS001 ModBus 控制
  5. 保留完整的 ServoJ + 坐标变换 + 视觉抓取流程

运行:
  cd ~/Desktop/hand_demo_arm
  source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
  python3 control_node.py --ros-args --params-file control_node.yaml
"""

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
# ── ARM 适配：不导入 GetSensor6DData / MarkBaseSensor（ARM ctypes 版不存在）──
from tl_ros2_interface.srv import CoordTransform, GetPosReachable, OpenServoJ, ModbusWrite, SetCurrentCoord, SetCoordNum
# SetToolParam 可选（ARM 可能不完全支持）
try:
    from tl_ros2_interface.srv import SetToolParam
    HAS_SET_TOOL_PARAM = True
except ImportError:
    HAS_SET_TOOL_PARAM = False

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
        # ModbusRTU 参数（用于灵巧手控制）
        self.declare_parameter('modbus_master_id', 1)
        self.declare_parameter('modbus_rtu_port', 2)
        self.declare_parameter('modbus_rtu_baudrate', 115200)
        self.declare_parameter('modbus_rtu_slave_id', 2)
        self.declare_parameter('modbus_rtu_data_bit', 8)
        self.declare_parameter('modbus_rtu_stop_bit', 1)
        self.declare_parameter('modbus_rtu_check_bit', 'None')
        self.declare_parameter('modbus_gripper_addrs', [1135, 1136, 1137, 1138, 1139, 1140])
        self.declare_parameter('modbus_gripper_close_val', [21000, 20000, 23000, 60000, 60000, 60000])
        self.declare_parameter('modbus_gripper_open_val', [0, 0, 0, 60000, 60000, 60000])
        self.declare_parameter('z_min_limit', 68.0)
        self.declare_parameter('grasp_target_z', 180.0)
        self.declare_parameter('descend_y_offset', 0.0)
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
        self.declare_parameter('servoj_interp_points', 500)
        self.declare_parameter('servoj_vmax', [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
        self.declare_parameter('servoj_amax', [3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0])
        self.declare_parameter('servoj_jmax', [50000.0, 50000.0, 50000.0, 50000.0, 50000.0, 50000.0, 50000.0])
        self.declare_parameter('servoj_arrival_threshold', 0.5)
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
        self.declare_parameter('gripper_open_delay', 0.5)
        self.declare_parameter('gripper_close_delay', 4.0)
        self.declare_parameter('pre_waypoint_delay', 1.0)
        self.declare_parameter('post_waypoint_delay', 4.0)
        self.declare_parameter('grasp_approach_z', 200.0)
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
        self.z_min_limit = float(self.get_parameter('z_min_limit').value)
        self.grasp_target_z = float(self.get_parameter('grasp_target_z').value)
        self.descend_y_offset = float(self.get_parameter('descend_y_offset').value)
        # 工具坐标系参数
        self.tool_num = int(self.get_parameter('tool_num').value)
        self.tool_user_num = int(self.get_parameter('tool_user_num').value)
        self.tool_x = float(self.get_parameter('tool_x').value)
        self.tool_y = float(self.get_parameter('tool_y').value)
        self.tool_z = float(self.get_parameter('tool_z').value)
        self.tool_a = float(self.get_parameter('tool_a').value)
        self.tool_b = float(self.get_parameter('tool_b').value)
        self.tool_c = float(self.get_parameter('tool_c').value)
        self._gripper_closed = False
        self.zero_joint = list(self.get_parameter('zero_joint').value)
        # 抓取后点位
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
        self.detection_collect_time = 0.5
        self.auto_loop_interrupt = False

        self.running = True
        self.cleaned = False

        self.detected_objects = []
        self.latest_object_type = None
        self.latest_base_point = None
        self.latest_object_angle = 0.0
        self.latest_stamp = None

        self.latest_tcp_pose = None
        self.latest_joint_state = None

        # ServoJ 插值
        self.servoj_current_target = [0.0] * 7
        self.servoj_initialized = False
        self.servoj_interp_points = int(self.get_parameter('servoj_interp_points').value)
        self.servoj_arrival_threshold = float(self.get_parameter('servoj_arrival_threshold').value)
        self.servoj_current_final = None
        self.servoj_trajectory = []
        self.servoj_traj_idx = 0
        self.servoj_moving = False
        self.servoj_vmax = list(self.get_parameter('servoj_vmax').value)
        self.servoj_amax = list(self.get_parameter('servoj_amax').value)
        self.servoj_jmax = list(self.get_parameter('servoj_jmax').value)
        self.servoj_timer = None

        # 键盘
        self.old_terminal_settings = None
        self.keyboard_fd = None
        self.keyboard_fd_need_close = False
        self.setup_keyboard()

        self.T_tool_camera = self.load_handeye_matrix_from_param()

        self.safe_log_info('========== T_tool_camera ==========')
        self.safe_log_info(f'\n{self.T_tool_camera}')
        self.safe_log_info('===================================')

        # 创建服务客户端（ARM 版不包含力传感器）
        self._create_service_clients()

        # 创建话题订阅与发布
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

        self.safe_log_info('control_node (ARM 版) 初始化完成')
        self.safe_log_info(f'订阅: {self.camera_object_topic}')
        self.safe_log_info(f'base_frame_id: {self.base_frame_id}')
        self.safe_log_info(f'机械臂: {self.robot_ip}:{self.robot_port}')
        self.safe_log_info(f'目标类型 object_type: {self.target_object_type}')
        self.safe_log_info(f'ServoJ 插值点数: {self.servoj_interp_points}')
        hint = 't 夹爪开/闭 | h 抓取+经过点位 | p 经过点位 | z 回零 | c 中断循环 | q 退出'
        if self.auto_loop_enabled:
            hint = '自动循环模式 | c 中断循环 | z 回零 | q 退出'
        self.safe_log_info(f'按键: {hint}')

        # 初始化：设置工具参数 → 激活 → 切换坐标系 → 开 ServoJ
        if HAS_SET_TOOL_PARAM:
            self._call_set_tool_param_service()
        self._call_set_coord_num_service()
        self._call_set_tool_coord_service()
        self._call_open_servoj_service()

    def _create_service_clients(self):
        """创建 ROS2 服务客户端（ARM 版：不包含力传感器服务）。"""
        self.coord_transform_client = self.create_client(CoordTransform, '/tl_driver/coord_transform')
        self.get_pos_reachable_client = self.create_client(GetPosReachable, '/tl_driver/get_pos_reachable')
        self.open_servoj_client = self.create_client(OpenServoJ, '/tl_driver/open_servoj')
        self.modbus_write_client = self.create_client(ModbusWrite, '/tl_driver/modbus_write')
        self.set_coord_num_client = self.create_client(SetCoordNum, '/tl_driver/set_coord_num')
        self.set_current_coord_client = self.create_client(SetCurrentCoord, '/tl_driver/set_current_coord')
        if HAS_SET_TOOL_PARAM:
            self.set_tool_param_client = self.create_client(SetToolParam, '/tl_driver/set_tool_param')
        # ════════════════════════════════════════════════════════════
        # ARM 版: 六维力传感器 GetSensor6DData / MarkBaseSensor 不存在
        # 不创建对应的服务客户端
        # ════════════════════════════════════════════════════════════

    def _call_open_servoj_service(self):
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
                    self.safe_log_info(f'open_servoj 成功 (vmax={self.servoj_vmax[0]}°/s)')
                else:
                    self.safe_log_warn(f'open_servoj 失败: {resp.message}')
            else:
                self.safe_log_warn('open_servoj 服务调用超时或无响应')
        except Exception as e:
            self.safe_log_error(f'open_servoj 服务调用异常: {e}')

    def _call_set_tool_param_service(self):
        """从 YAML 读取工具坐标系参数并调用 /tl_driver/set_tool_param 服务（ARM 可选）。"""
        if not HAS_SET_TOOL_PARAM:
            self.safe_log_info('ARM 版: SetToolParam 不可用，跳过工具坐标系设置')
            return
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
                    self.safe_log_info(f'工具坐标系设置成功: tool_num={self.tool_num}')
                else:
                    self.safe_log_warn(f'工具坐标系设置失败: {resp.message}')
            else:
                self.safe_log_warn('工具坐标系设置服务调用超时')
        except Exception as e:
            self.safe_log_error(f'工具坐标系设置异常: {e}')

    # ════════════════════════════════════════════════════════════
    # ARM 版: 六维力传感器函数已移除
    # ════════════════════════════════════════════════════════════

    def _call_set_coord_num_service(self):
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
        except Exception as e:
            self.safe_log_error(f'工具编号激活异常: {e}')

    def _call_set_tool_coord_service(self):
        if not self.set_current_coord_client.wait_for_service(timeout_sec=5.0):
            self.safe_log_warn('/tl_driver/set_current_coord 服务不可用，跳过坐标系切换')
            return

        req = SetCurrentCoord.Request()
        req.coord = 2

        try:
            future = self.set_current_coord_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done() and future.result() is not None:
                resp = future.result()
                if resp.success:
                    self.safe_log_info('已切换到工具坐标系（coord=2）')
                else:
                    self.safe_log_warn(f'切换工具坐标系失败: {resp.message}')
        except Exception as e:
            self.safe_log_error(f'切换工具坐标系异常: {e}')

    def _create_topic_sub_pub(self):
        self.tcp_pose_sub = self.create_subscription(
            CartesianPose,
            '/tcp_pose',
            self.tcp_pose_callback,
            10
        )
        # ════════════════════════════════════════════════════════
        # ARM 版: tl_driver 直接发布到 /joint_states，不需要 remap
        # 原 x86 版订阅 /nano/joint_states 然后 remap 到 /joint_states
        # ════════════════════════════════════════════════════════
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.servoj_pos_pub = self.create_publisher(
            Float64MultiArray,
            '/tl_driver/set_servoj_pos',
            10
        )
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        self.servoj_timer = self.create_timer(0.01, self.servoj_timer_callback)

    def tcp_pose_callback(self, msg: CartesianPose):
        self.latest_tcp_pose = msg

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def servoj_timer_callback(self):
        if not self.running:
            return

        if not self.servoj_initialized and self.latest_joint_state is not None:
            current_pos_rad = list(self.latest_joint_state.position)
            current_pos_deg = [v * 180.0 / math.pi for v in current_pos_rad]
            self.servoj_current_target = (current_pos_deg + [0.0] * 7)[:7]
            self.servoj_initialized = True

        if not self.servoj_initialized:
            return

        if self.servoj_moving:
            if self.servoj_traj_idx < len(self.servoj_trajectory):
                self.servoj_current_target = self.servoj_trajectory[self.servoj_traj_idx]
                self.servoj_traj_idx += 1

                if self.servoj_current_final is not None:
                    max_diff = max(
                        abs(self.servoj_current_target[i] - self.servoj_current_final[i])
                        for i in range(7)
                    )
                    if max_diff < self.servoj_arrival_threshold:
                        self.servoj_current_target = list(self.servoj_current_final)
                        self.servoj_moving = False
                        self.servoj_current_final = None
            else:
                self.servoj_moving = False
                self.servoj_current_final = None

        msg = Float64MultiArray()
        msg.data = self.servoj_current_target
        self.servoj_pos_pub.publish(msg)

        if self.latest_joint_state is not None:
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = list(self.latest_joint_state.name)
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
            self.safe_log_info('键盘控制已启用')

        except Exception as e:
            print(f'键盘初始化失败: {e}')
            self.keyboard_fd = None

    def restore_keyboard(self):
        try:
            if self.keyboard_fd is not None and self.old_terminal_settings is not None:
                try:
                    termios.tcsetattr(self.keyboard_fd, termios.TCSADRAIN, self.old_terminal_settings)
                except Exception:
                    pass
                self.old_terminal_settings = None
            if self.keyboard_fd is not None and self.keyboard_fd_need_close:
                try:
                    os.close(self.keyboard_fd)
                except Exception:
                    pass
            self.keyboard_fd = None
        except Exception:
            pass

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
                raise ValueError(f'handeye_matrix 参数长度错误: {len(handeye_list)}')
            T_tool_camera = np.array(handeye_list, dtype=np.float64).reshape(4, 4)
            return T_tool_camera
        except Exception as e:
            self.safe_log_error(f'读取 handeye_matrix 失败: {e}')
            self.cleanup_and_exit()

    def get_robot_pose(self):
        if self.latest_tcp_pose is None:
            self.safe_log_warn('TCP pose 尚未接收到数据')
            return False, None
        try:
            x = self.latest_tcp_pose.position.x / 1000.0
            y = self.latest_tcp_pose.position.y / 1000.0
            z = self.latest_tcp_pose.position.z / 1000.0
            roll = self.latest_tcp_pose.rpy.x
            pitch = self.latest_tcp_pose.rpy.y
            yaw = self.latest_tcp_pose.rpy.z
            return True, [x, y, z, roll, pitch, yaw]
        except Exception as e:
            self.safe_log_error(f'获取位姿失败: {e}')
            return False, None

    def pose_to_tool_rt(self, pose):
        if pose is None or len(pose) < 6:
            raise ValueError(f'pose 长度不足')
        t_tool = np.array(pose[0:3], dtype=np.float64).reshape(3, 1)
        A, B, C = pose[3], pose[4], pose[5]
        rotation = R.from_euler('XYZ', [A, B, C], degrees=False)
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
            P_camera = np.array([msg.pos.point.x, msg.pos.point.y, msg.pos.point.z, 1.0], dtype=np.float64)
            if not np.all(np.isfinite(P_camera)):
                return

            success, robot_pose = self.get_robot_pose()
            if not success:
                return

            T_base_tool = self.pose_to_T_base_tool(robot_pose)
            T_base_camera = T_base_tool @ self.T_tool_camera
            P_base = T_base_camera @ P_camera

            if not np.all(np.isfinite(P_base)):
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
            self.detected_objects.append({
                'type': str(object_type),
                'base_pos': np.array([P_base[0], P_base[1], P_base[2]], dtype=np.float64),
                'cam_x': float(msg.pos.point.x),
                'cam_y': float(msg.pos.point.y),
                'angle': msg.angle,
                'stamp': now,
            })
            self.latest_object_type = str(object_type)
            self.latest_base_point = np.array([P_base[0], P_base[1], P_base[2]], dtype=np.float64)
            self.latest_object_angle = msg.angle
            self.latest_stamp = now
        except Exception as e:
            self.safe_log_error(f'物体坐标转换失败: {e}')

    def get_target_joint_from_cartesian(self, target_cart):
        req = CoordTransform.Request()
        req.origin_coord = 1
        req.target_coord = 0
        req.form = 0
        req.origin_pos = list(target_cart)
        req.reference_pos = [0.0] * 7

        max_retries = 3
        for attempt in range(max_retries):
            try:
                future = self.coord_transform_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                result = future.result()
                if result.success:
                    return 0, list(result.target_pos)
                if attempt < max_retries - 1:
                    self.safe_log_warn(f'笛卡尔转关节失败(第{attempt+1}次): {result.message}，重试...')
                    time.sleep(0.05)
                else:
                    self.safe_log_error(f'笛卡尔转关节失败(已重试{max_retries}次)')
            except Exception as e:
                if attempt < max_retries - 1:
                    time.sleep(0.05)
                else:
                    self.safe_log_error(f'笛卡尔转关节异常: {e}')
        return -1, []

    def check_pos_reachable(self, point_values, coord=0, angle_unit=0, posture=1):
        pos = [0.0] * 14
        pos[0] = float(coord)
        pos[1] = float(angle_unit)
        pos[2] = float(posture)
        for i in range(len(point_values)):
            pos[7 + i] = float(point_values[i])

        req = GetPosReachable.Request()
        req.pos = pos
        req.move_type = self.approach_movetype

        try:
            future = self.get_pos_reachable_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if not result.success:
                self.safe_log_warn(f'点位不可达: {result.message}')
                return False
            return True
        except Exception as e:
            self.safe_log_error(f'可达性判断异常: {e}')
            return False

    def movej(self, joint_pos):
        try:
            joint_list = list(joint_pos)
            if len(joint_list) < 7:
                joint_list = joint_list + [0.0] * (7 - len(joint_list))
            if not self.servoj_initialized:
                self.safe_log_warn('ServoJ 尚未初始化，跳过运动')
                return False

            final_target = joint_list[:7]
            start = list(self.servoj_current_target)

            n = self.servoj_interp_points
            self.servoj_trajectory = []
            for step in range(1, n + 1):
                ratio = step / n
                point = [start[i] + (final_target[i] - start[i]) * ratio for i in range(7)]
                self.servoj_trajectory.append(point)

            self.servoj_current_final = final_target
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
            self.safe_log_warn(f'当前物体 {self.latest_object_type} != 目标 {self.target_object_type}')
            return False

        p = self.latest_base_point.copy()
        angle = self.latest_object_angle if hasattr(self, 'latest_object_angle') else 0.0
        yaw = angle * self.angle_scale + self.angle_offset
        target_z = float((p[2] + self.grasp_offset) * 1000.0)
        if target_z < self.z_min_limit:
            target_z = self.z_min_limit

        target_cart = [
            float(p[0] * 1000.0), float(p[1] * 1000.0), target_z,
            -3.14, 0.0, yaw, 0.0
        ]

        ret, target_joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0:
            return False

        if not self.check_pos_reachable(target_joint):
            return False

        return self.movej(target_joint)

    def _collect_and_sort_detections(self):
        self.detected_objects.clear()
        self._spin_sleep(self.detection_collect_time)
        if not self.detected_objects:
            return []
        sorted_objs = sorted(self.detected_objects, key=lambda o: (o['cam_y'], o['cam_x']))
        return sorted_objs

    def _spin_sleep(self, seconds):
        end_time = time.time() + seconds
        while time.time() < end_time and self.running:
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except Exception:
                break

    def _wait_servoj_arrival(self, check_interval=0.5):
        while self.servoj_moving and self.running:
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except Exception:
                break
            if time.time() % check_interval < 0.01:
                if self.latest_joint_state is not None and self.servoj_current_final is not None:
                    current_deg = [v * 180.0 / math.pi for v in self.latest_joint_state.position]
                    diffs = [abs(current_deg[i] - self.servoj_current_final[i]) for i in range(min(7, len(current_deg)))]
                    self.safe_log_info(f'  运动中... 最大偏差 {max(diffs):.2f}°')

    def move_through_waypoints(self):
        if not self.after_grasp_waypoints:
            self.safe_log_warn('未配置 after_grasp_waypoints，跳过')
            return False

        for i, wp in enumerate(self.after_grasp_waypoints):
            if len(wp) < 6:
                continue
            target_cart = [float(wp[0]), float(wp[1]), float(wp[2]),
                           float(wp[3]), float(wp[4]), float(wp[5]), 0.0]
            ret, target_joint = self.get_target_joint_from_cartesian(target_cart)
            if ret != 0 or not self.check_pos_reachable(target_joint) or not self.movej(target_joint):
                continue
            self._wait_servoj_arrival(0.5)
        return True

    def move_zero(self):
        if len(self.zero_joint) < 7:
            zero_joint = list(self.zero_joint) + [0.0] * (7 - len(self.zero_joint))
        else:
            zero_joint = list(self.zero_joint[:7])
        if not self.check_pos_reachable(zero_joint):
            self.safe_log_warn('零点位置不可达')
            return False
        return self.movej(zero_joint)

    def _build_modbus_rtu_master_param(self):
        from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam
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
        """切换夹爪（灵巧手）开/闭。"""
        if self._gripper_closed:
            vals = self.modbus_gripper_open_val
            action = '张开'
        else:
            vals = self.modbus_gripper_close_val
            action = '闭合'

        addrs = self.modbus_gripper_addrs
        n = min(len(addrs), len(vals))

        req = ModbusWrite.Request()
        req.master_id = self.modbus_master_id
        req.addr = addrs[0]
        req.data = vals[:n]
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
                    self.safe_log_error(f'ModBus 写入失败')
                    ok = False
            except Exception as e:
                self.safe_log_error(f'modbus_write 异常: {e}')
                ok = False

        if ok:
            self._gripper_closed = not self._gripper_closed
            self.safe_log_info(f'灵巧手 {action} 成功')
        return ok

    def gripper_open(self):
        self._gripper_closed = True
        return self.gripper_toggle()

    def gripper_close(self):
        self._gripper_closed = False
        return self.gripper_toggle()

    def grasp_at_target(self):
        """下降到目标 Z 位置 → 张开 → 闭合抓取。"""
        if self.latest_tcp_pose is None:
            self.safe_log_warn('TCP pose 未收到，无法抓取')
            return False

        cx = self.latest_tcp_pose.position.x
        cy = self.latest_tcp_pose.position.y
        cz = self.latest_tcp_pose.position.z

        target_z = max(self.grasp_target_z, self.z_min_limit)
        target_cart = [cx, cy + self.descend_y_offset, target_z, 3.13, 0.92, 0.0, 0.0]

        self.safe_log_info(f'>> 抓取: 下降到 Z={target_z:.1f}mm')
        ret, target_joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0 or not self.movej(target_joint):
            return False

        self._wait_servoj_arrival(0.5)

        self.gripper_open()
        self._spin_sleep(self.gripper_open_delay)
        self.gripper_close()
        self._spin_sleep(self.gripper_close_delay)
        return True

    def descend(self):
        if self.latest_tcp_pose is None:
            self.safe_log_warn('TCP pose 未收到，无法下降')
            return False

        cx = self.latest_tcp_pose.position.x
        cy = self.latest_tcp_pose.position.y
        cz = self.latest_tcp_pose.position.z
        GRASP_R, GRASP_P, GRASP_Y = 3.13, 0.92, 0.0

        target_z = cz - self.grasp_depth * 1000.0
        if target_z < self.z_min_limit:
            target_z = self.z_min_limit

        target_cart = [cx, cy + self.descend_y_offset, target_z, GRASP_R, GRASP_P, GRASP_Y, 0.0]

        ret, target_joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0:
            return False
        return self.movej(target_joint)

    def handle_key(self, key):
        self.safe_log_info(f'收到按键: {repr(key)}')

        if key == 'c':
            self.auto_loop_interrupt = True
            self.servoj_moving = False
            self.safe_log_info('>> 中断信号已发送')

        elif key == 't':
            self.gripper_toggle()

        elif key == 'h':
            self._execute_grasp_sequence()

        elif key == 'p':
            self.safe_log_info('>> 手动触发：依次经过所有配置点位')
            self.move_through_waypoints()

        elif key == 'z':
            self.safe_log_info('按键 z: 回零点')
            self.move_zero()

        elif key == 'q':
            print('\n按键 q: 退出 control_node')
            self.running = False

        elif key in ('f', 'm'):
            self.safe_log_warn(f'ARM 版: 按键 {key} 不可用（六维力传感器不支持）')

    def _grasp_single_object(self, obj, obj_index, total):
        p = obj['base_pos']
        GRASP_R, GRASP_P, GRASP_Y = 3.13, 0.92, 0.0
        obj_x = float(p[0] * 1000.0)
        obj_y = float(p[1] * 1000.0) + self.descend_y_offset

        # 过渡高度
        approach_z = max(self.grasp_approach_z, self.z_min_limit)
        target_cart = [obj_x, obj_y, approach_z, GRASP_R, GRASP_P, GRASP_Y, 0.0]
        ret, joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0 or not self.check_pos_reachable(joint) or not self.movej(joint):
            return False
        self._wait_servoj_arrival(0.5)
        if self.auto_loop_interrupt:
            return False

        # 抓取高度
        grasp_z = max(self.grasp_target_z, self.z_min_limit)
        target_cart = [obj_x, obj_y, grasp_z, GRASP_R, GRASP_P, GRASP_Y, 0.0]
        ret, joint = self.get_target_joint_from_cartesian(target_cart)
        if ret != 0 or not self.check_pos_reachable(joint) or not self.movej(joint):
            return False
        self._wait_servoj_arrival(0.5)
        if self.auto_loop_interrupt:
            return False

        # 夹爪抓取
        self.gripper_open()
        self._spin_sleep(self.gripper_open_delay)
        self.gripper_close()
        self._spin_sleep(self.gripper_close_delay)
        return True

    def _execute_grasp_sequence(self):
        """完整流程：检测→抓取→放下→回零→检测下一个..."""
        grasped_count = 0
        while not self.auto_loop_interrupt:
            targets = self._collect_and_sort_detections()
            targets = [t for t in targets if t['type'] == self.target_object_type]
            if not targets:
                if grasped_count == 0:
                    self.safe_log_warn('没有检测到目标物体')
                    return False
                else:
                    self.safe_log_info(f'>> 已抓取 {grasped_count} 个，没有更多物体')
                    break

            obj = targets[0]
            grasped_count += 1

            if self.auto_loop_interrupt:
                break
            if not self._grasp_single_object(obj, grasped_count, '?'):
                continue
            if self.auto_loop_interrupt:
                break

            self._spin_sleep(self.pre_waypoint_delay)
            self.move_through_waypoints()
            if self.auto_loop_interrupt:
                break

            self.gripper_open()
            self._spin_sleep(self.post_waypoint_delay)

            self.move_zero()
            self._wait_servoj_arrival(0.5)
            self.safe_log_info('>> 停顿2秒等待画面稳定...')
            self._spin_sleep(2.0)

        self.safe_log_info(f'>> 共抓取 {grasped_count} 个物体')
        return grasped_count > 0

    def auto_loop(self):
        self.safe_log_info('>> ═══════════════ 启动自动循环模式 ═══════════════')
        self.auto_loop_interrupt = False

        cycle_count = 0
        while self.running and not self.auto_loop_interrupt:
            cycle_count += 1
            self.safe_log_info(f'>> ══ 自动循环 第{cycle_count}轮 ══')

            if not self.move_zero():
                break
            self._wait_servoj_arrival(0.5)

            if self.auto_loop_interrupt:
                break

            if not self._execute_grasp_sequence():
                self.safe_log_info('>> 未检测到物体或抓取失败，停止自动循环')
                break

        self.safe_log_info('>> ═══════════════ 自动循环结束 ═══════════════')

    def run(self):
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
            print('\n收到中断信号，正在退出...')
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
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
                print('ROS2已关闭')
            except Exception:
                pass


if __name__ == '__main__':
    main()
