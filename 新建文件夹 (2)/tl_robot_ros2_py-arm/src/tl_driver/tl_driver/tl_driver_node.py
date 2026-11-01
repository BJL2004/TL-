import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
import tl_ros2_interface.msg as msgs
import tl_ros2_interface.srv as srvs
import std_msgs.msg as std_msgs
import os
import socket
import logging
import time
import math
import threading

try:
    from tl_driver.lib import tl_interface
except Exception:
    # Fallback: try to load the SWIG wrapper from the package's sibling lib/
    # directory (useful during development where _nrc_host.so and
    # tl_interface.py live in src/tl_driver/lib)
    tl_interface = None
    try:
        import sys
        import importlib.util

        lib_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "lib"))
        nrc_path = os.path.join(lib_dir, "tl_interface.py")
        if os.path.exists(nrc_path):
            # Ensure the native module _nrc_host can be imported from lib_dir
            if lib_dir not in sys.path:
                sys.path.insert(0, lib_dir)
            spec = importlib.util.spec_from_file_location("tl_interface", nrc_path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            tl_interface = mod
    except Exception:
        tl_interface = None


def _is_success(res) -> bool:
    """Normalize various tl_interface return types to boolean success.

    - bool -> returned as-is
    - None -> treat as True (many swig void functions)
    - int -> True if equal to 0 (treat 0 as Result::SUCCESS)
    - other -> truthiness
    """
    if isinstance(res, bool):
        return res
    if res is None:
        return True
    if isinstance(res, int):
        return res == 0
    return bool(res)


def _check_host_reachable(host: str, timeout: float = 1.0) -> bool:
    """Only check whether the host IP is reachable (ICMP/TCP level).

    Unlike _check_port_open (which connected to the actual port and then
    closed it, causing the robot controller to refuse the subsequent
    native connect_robot call), this function only verifies that the host
    exists on the network.  The actual port connection is left entirely
    to connect_robot().
    """
    import subprocess as _sp

    try:
        ret = _sp.run(
            ["ping", "-c", "1", "-W", str(int(timeout)), host],
            capture_output=True,
            timeout=timeout + 1,
        )
        return ret.returncode == 0
    except Exception:
        return False


class TLArmNode(Node):
    def __init__(self):
        super().__init__("tl_driver")

        # parameters
        self.declare_parameter("arm_ip", "192.168.1.13")
        self.declare_parameter("arm_port", "6001")
        self.declare_parameter("arm_port_aux", "7000")
        self.declare_parameter("arm_type", "TCB605")
        self.declare_parameter(
            "arm_joints", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        )

        # read parameters
        self.arm_ip_ = self.get_parameter("arm_ip").value
        self.arm_port_ = self.get_parameter("arm_port").value
        self.arm_port_aux_ = self.get_parameter("arm_port_aux").value
        self.arm_type_ = self.get_parameter("arm_type").value
        self.arm_joints_ = self.get_parameter("arm_joints").value

        # dof
        self.ndof_ = len(self.arm_joints_)

        self.get_logger().info(f"arm_type={self.arm_type_}, ndof={self.ndof_}")

        # callback groups for multi-threaded executor (matching C++ node)
        self.service_group_ = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.topic_group_ = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.timer_group_ = rclpy.callback_groups.ReentrantCallbackGroup()

        # publishers
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        # create tcp_pose publisher if message available
        try:
            self.tcp_pose_pub = self.create_publisher(msgs.CartesianPose, "/tcp_pose", 10)
        except Exception:
            self.get_logger().warning(
                "CartesianPose message not available; /tcp_pose publisher skipped"
            )
            self.tcp_pose_pub = None

        # create running status publisher; if ArmStatus not available, fallback to std_msgs.String
        try:
            self.running_status_pub = self.create_publisher(msgs.ArmStatus, "/arm_status", 10)
            self._running_status_fallback = False
        except Exception:
            self.get_logger().warning(
                "ArmStatus message not available; publishing /arm_status as std_msgs.String"
            )
            self.running_status_pub = self.create_publisher(std_msgs.String, "/arm_status", 10)
            self._running_status_fallback = True

        # services
        self.create_service(
            Trigger,
            "/tl_driver/connect_arm",
            self.handle_connect_service,
            callback_group=self.service_group_,
        )
        self.create_service(
            Trigger,
            "/tl_driver/disconnect_arm",
            self.handle_disconnect_service,
            callback_group=self.service_group_,
        )
        self.create_service(
            Trigger,
            "/tl_driver/power_on",
            self.handle_poweron_service,
            callback_group=self.service_group_,
        )
        self.create_service(
            Trigger,
            "/tl_driver/power_off",
            self.handle_poweroff_service,
            callback_group=self.service_group_,
        )
        self.create_service(
            Trigger,
            "/tl_driver/clear_error",
            self.handle_clear_error_service,
            callback_group=self.service_group_,
        )
        self.create_service(
            Trigger,
            "/tl_driver/get_controller_id",
            self.handle_get_controller_id_service,
            callback_group=self.service_group_,
        )

        # typed service from interface package
        try:
            self.create_service(
                srvs.SetSpeed,
                "/tl_driver/set_speed",
                self.handle_set_speed_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetSpeed,
                "/tl_driver/get_speed",
                self.handle_get_speed_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetPosTransform,
                "/tl_driver/get_quat2rpy",
                self.handle_get_quat2rpy_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetPosTransform,
                "/tl_driver/get_rpy2quat",
                self.handle_get_rpy2quat_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetPosTransform,
                "/tl_driver/get_rpy2r",
                self.handle_get_rpy2r_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetPosTransform,
                "/tl_driver/get_tr2r",
                self.handle_get_tr2r_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetPosTransform,
                "/tl_driver/get_r2tr",
                self.handle_get_r2tr_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.SetControllerIP,
                "/tl_driver/set_controller_ip",
                self.handle_set_controller_ip_service,
                callback_group=self.service_group_,
            )

        except Exception:
            self.get_logger().warning("SetSpeed service type not available; skipping")

        # jogging & drag services
        try:
            self.create_service(
                srvs.Jogging,
                "/tl_driver/start_jogging",
                self.handle_start_jogging_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.Jogging,
                "/tl_driver/stop_jogging",
                self.handle_stop_jogging_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetRobotState,
                "/tl_driver/get_robot_state",
                self.handle_get_robot_state_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                Trigger,
                "/tl_driver/get_library_version",
                self.handle_get_library_version_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetRobotJointParam,
                "/tl_driver/get_robot_joint_param",
                self.handle_get_robot_joint_param_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.SetRobotJointParam,
                "/tl_driver/set_robot_joint_param",
                self.handle_set_robot_joint_param_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                Trigger,
                "/tl_driver/get_drag_status",
                self.handle_get_drag_status_service,
                callback_group=self.service_group_,
            )

        except Exception:
            self.get_logger().warning("Jogging/Drag service types not available; skipping")

        # system_status
        try:
            self.create_service(
                srvs.GetJointTemperature,
                "/tl_driver/get_joint_temperature",
                self.handle_get_joint_temperature_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetJointVoltage,
                "/tl_driver/get_joint_voltage",
                self.handle_get_joint_voltage_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetMotorCurrent,
                "/tl_driver/get_motor_current",
                self.handle_get_motor_current_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetCurrentMotorTorque,
                "/tl_driver/get_current_motor_torque",
                self.handle_get_current_motor_torque_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetCurrentLineJointSpeed,
                "/tl_driver/get_current_line_joint_speed",
                self.handle_get_current_line_joint_speed_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetJointSoftwareVersion,
                "/tl_driver/get_joint_software_version",
                self.handle_get_joint_software_version_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                Trigger,
                "/tl_driver/get_nexmotion_lib_version",
                self.handle_get_nexmotion_lib_version_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.RestoreDefaultDHParam,
                "/tl_driver/restore_default_dh_param",
                self.handle_restore_default_dh_param_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                Trigger,
                "/tl_driver/set_default_cartesian_param",
                self.handle_set_default_cartesian_param_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.LogDownload,
                "/tl_driver/log_download",
                self.handle_log_download_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.SetDragMode,
                "/tl_driver/set_drag_mode",
                self.handle_set_drag_mode_service,
                callback_group=self.service_group_,
            )

        except Exception:
            self.get_logger().warning("system_status service types not available; skipping")

        # queue services
        try:
            self.create_service(
                srvs.QueueMotionSetStatus,
                "/tl_driver/queue_motion_set_status",
                self.handle_queue_motion_set_status_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.QueueMotionMoveJ,
                "/tl_driver/queue_motion_movej",
                self.handle_queue_motion_movej_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                Trigger,
                "/tl_driver/queue_motion_stop",
                self.handle_queue_motion_stop_service,
                callback_group=self.service_group_,
            )
        except Exception:
            self.get_logger().warning("Queue motion service types not available; skipping")

        # tool / coordinate services and IO/Modbus
        try:
            self.create_service(
                srvs.SetToolParam,
                "/tl_driver/set_tool_param",
                self.handle_set_tool_param_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.SetUserCoord,
                "/tl_driver/set_user_coord",
                self.handle_set_user_coord_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.SetAxisZeroPos,
                "/tl_driver/set_axis_zero_pos",
                self.handle_set_axis_zero_pos_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.SetCurrentCoord,
                "/tl_driver/set_current_coord",
                self.handle_set_current_coord_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetCurrentCoord,
                "/tl_driver/get_current_coord",
                self.handle_get_current_coord_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetCoordNum,
                "/tl_driver/get_coord_num",
                self.handle_get_coord_num_service,
                callback_group=self.service_group_,
            )

            # additional services mirroring C++ node
            self.create_service(
                srvs.GetAllJobFileName,
                "/tl_driver/get_all_job_filename",
                self.handle_get_all_job_filename_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.JobRun,
                "/tl_driver/job_run",
                self.handle_job_run_service,
                callback_group=self.service_group_,
            )
            # job delete
            self.create_service(
                srvs.JobRun,
                "/tl_driver/job_delete",
                self.handle_job_delete_service,
                callback_group=self.service_group_,
            )
            # job insert (matching C++: changed from topic to service)
            self.create_service(
                srvs.JobInsertMove,
                "/tl_driver/job_insert_moveJ",
                self.handle_job_insert_movej_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.JobInsertMove,
                "/tl_driver/job_insert_moveL",
                self.handle_job_insert_movel_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.JobInsertMove,
                "/tl_driver/job_insert_iMove",
                self.handle_job_insert_imove_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.JobInsertMove,
                "/tl_driver/job_insert_imove",
                self.handle_job_insert_imove_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.JobInsertMove,
                "/tl_driver/job_insert_moveC",
                self.handle_job_insert_movec_service,
                callback_group=self.service_group_,
            )
            # set/get global pos, coord transform, reachable checks
            self.create_service(
                srvs.SetGlobalPos,
                "/tl_driver/set_global_pos",
                self.handle_set_global_pos_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetGlobalPos,
                "/tl_driver/get_global_pos",
                self.handle_get_global_pos_service,
                callback_group=self.service_group_,
            )
            try:
                self.create_service(
                    srvs.CoordTransform,
                    "/tl_driver/coord_transform",
                    self.handle_coord_transform_service,
                    callback_group=self.service_group_,
                )
            except Exception:
                # coord transform may not have SWIG mapping
                pass
            self.create_service(
                srvs.GetPosReachable,
                "/tl_driver/get_pos_reachable",
                self.handle_get_pos_reachable_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetDHParam,
                "/tl_driver/get_dh_param",
                self.handle_get_dh_param_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.SetDHParam,
                "/tl_driver/set_dh_param",
                self.handle_set_dh_param_service,
                callback_group=self.service_group_,
            )

            # set current mode and servoj close
            try:
                self.create_service(
                    srvs.SetCurrentMode,
                    "/tl_driver/set_current_mode",
                    self.handle_set_current_mode_service,
                    callback_group=self.service_group_,
                )
                self.create_service(
                    srvs.GetCurrentMode,
                    "/tl_driver/get_current_mode",
                    self.handle_get_current_mode_service,
                    callback_group=self.service_group_,
                )
            except Exception:
                pass
            self.create_service(
                Trigger,
                "/tl_driver/close_servoj",
                self.handle_close_servoj_service,
                callback_group=self.service_group_,
            )

            self.create_service(
                srvs.SetDigitalOutput,
                "/tl_driver/set_digital_output",
                self.handle_set_digital_output_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.GetDigitalInputOutput,
                "/tl_driver/get_digital_input_output",
                self.handle_get_digital_input_output_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.ModbusWrite,
                "/tl_driver/modbus_write",
                self.handle_modbus_write_service,
                callback_group=self.service_group_,
            )
            self.create_service(
                srvs.ModbusRead,
                "/tl_driver/modbus_read",
                self.handle_modbus_read_service,
                callback_group=self.service_group_,
            )
            # track/trajectory
            try:
                self.create_service(
                    srvs.TrackSave,
                    "/tl_driver/track_save",
                    self.handle_track_save_service,
                    callback_group=self.service_group_,
                )
                self.create_service(
                    srvs.TrackPlayback,
                    "/tl_driver/track_playback",
                    self.handle_track_playback_service,
                    callback_group=self.service_group_,
                )
            except Exception:
                pass
            try:
                self.create_service(
                    srvs.OpenServoJ,
                    "/tl_driver/open_servoj",
                    self.handle_open_servoj_service,
                    callback_group=self.service_group_,
                )
            except Exception:
                pass
        except Exception:
            self.get_logger().warning(
                "Tool/Coord or IO/Modbus service types not available; skipping"
            )

        # subscriptions for move commands and job insert
        try:
            self.movej_sub = self.create_subscription(
                msgs.MoveCommand,
                "/tl_driver/moveJ",
                self.handle_movej_topic,
                10,
                callback_group=self.topic_group_,
            )

            self.movel_sub = self.create_subscription(
                msgs.MoveCommand,
                "/tl_driver/moveL",
                self.handle_movel_topic,
                10,
                callback_group=self.topic_group_,
            )
            # set_servoj position topic (matches C++ node)
            try:
                self.set_servoj_pos_sub = self.create_subscription(
                    std_msgs.Float64MultiArray,
                    "/tl_driver/set_servoj_pos",
                    self.handle_set_servoj_pos_topic,
                    10,
                    callback_group=self.topic_group_,
                )
            except Exception:
                self.get_logger().warning("set_servoj_pos message type not available; skipping")
        except Exception:
            self.get_logger().warning(
                "Move/Job message types not available; skipping subscriptions"
            )

        # example subscriber (placeholder)
        # self.create_subscription(...)
        # timer for periodic state publish
        self.publish_rate = 10.0
        period = 1.0 / float(self.publish_rate)
        self.create_timer(period, self.publish_arm_state, callback_group=self.timer_group_)

        self.get_logger().info("tl_driver (python) node started")

        # native interface socket fds (set by connect)
        self.fd = None
        self.is_connected_ = False
        self.is_powered_ = False

        # # 测试用后续待定
        self.msg_received = False
        self.latest_robot_state = ""

        self.msg_id = -1
        self.msg = ""

        # synchronous connect + power_on (matching C++ init() behavior)
        try:
            self._startup_connect()
        except Exception:
            pass

        # lock to prevent concurrent connect/disconnect calls
        self._conn_lock = threading.Lock()

    def on_robot_state_message(self, message):
        self.get_logger().info("=== 收到机器人状态回调 ===")
        self.latest_robot_state = str(message)
        self.msg_received = True
        self.get_logger().info(str(message))
        self.get_logger().info("robot state callback triggered")

    def handle_trigger(self, request, response):
        response.success = True
        response.message = "ok"
        return response

    def _valid_fd(self, fd_attr="fd") -> bool:
        """Return True if the given fd attribute exists and is a positive integer."""
        fd = getattr(self, fd_attr, None)
        if fd is None:
            return False
        try:
            return int(fd) > 0
        except Exception:
            return False

    # Basic implementations that call into tl_interface if available

    def connect(self) -> bool:
        # Mirror the C++ connect() behavior: connect both primary and auxiliary ports,
        # require positive socket fds, register callbacks, and mark connected.
        self.get_logger().info("connect() called")
        if tl_interface is not None:
            self.get_logger().info(f"tl_interface file = {tl_interface.__file__}")

        if self.is_connected_:
            self.get_logger().info("[Connect]: arm already connected")
            return True

        try:
            ip = self.get_parameter("arm_ip").get_parameter_value().string_value
            port = self.get_parameter("arm_port").get_parameter_value().string_value
            port_aux = self.get_parameter("arm_port_aux").get_parameter_value().string_value
            self.get_logger().info(f"ip={repr(ip)}, port={repr(port)}, port_aux={repr(port_aux)}")
        except Exception:
            ip, port, port_aux = "192.168.1.13", "6001", "7000"

        # Check if the port is reachable
        if not _check_host_reachable(ip, timeout=1.0):
            self.get_logger().warning(f"Host {ip} is not reachable (ping failed)")
            return False

        try:
            fd = None
            fd_aux = None
            self.get_logger().info("Attempting native connect_robot calls")
            if tl_interface is not None and hasattr(tl_interface, "connect_robot"):
                try:
                    fd = tl_interface.connect_robot(ip, port)
                except Exception as e:
                    self.get_logger().error(f"tl_interface.connect_robot() primary raised: {e}")
                    fd = None

                try:
                    fd_aux = tl_interface.connect_robot(ip, port_aux)
                except Exception:
                    fd_aux = None

            self.get_logger().info(f"native connect returned fd={fd}, fd_aux={fd_aux}")

            # validate fds (C++ treats <=0 as failure)
            try:
                if fd is None or int(fd) <= 0:
                    self.get_logger().error(f"[Connect]: failed to connect to {ip}:{port}")
                    self.fd = 0
                    self.fd_aux = 0
                    self.is_connected_ = False
                    return False
            except Exception:
                self.get_logger().error("[Connect]: invalid fd from connect_robot")
                self.fd = 0
                self.fd_aux = 0
                self.is_connected_ = False
                return False

            try:
                if fd_aux is None or int(fd_aux) <= 0:
                    self.get_logger().error(f"[Connect]: failed to connect to {ip}:{port_aux}")
                    self.fd = 0
                    self.fd_aux = 0
                    self.is_connected_ = False
                    return False
            except Exception:
                self.get_logger().warning("[Connect]: invalid fd_aux from connect_robot")

            # store fds and mark connected
            self.fd = fd
            self.fd_aux = fd_aux
            self.is_connected_ = True

            # 注册机器人状态回调（必须connect成功后）
            # try:
            #     tl_interface.robot_state_callback(
            #         self.fd_aux,
            #         self._robot_state_callback
            #     )

            #     self.get_logger().info(
            #         f'robot_state_callback registered, fd_aux={self.fd_aux}'
            #     )

            # except Exception as e:
            #     self.get_logger().error(
            #         f'robot_state_callback failed: {e}'
            #     )

            # register receive callbacks if available
            try:
                if tl_interface is not None and hasattr(
                    tl_interface, "set_receive_error_or_warnning_message_callback"
                ):

                    def _receive_cb(messageType, message, messageCode):
                        try:
                            self.get_logger().warning(
                                f"receive messageType={messageType}, code={messageCode}, msg={message}"
                            )
                        except Exception:
                            pass

                    try:
                        tl_interface.set_receive_error_or_warnning_message_callback(
                            self.fd, _receive_cb
                        )
                    except Exception:
                        pass

                # start recv_message on auxiliary fd to capture asynchronous messages from controller
                if (
                    tl_interface is not None
                    and hasattr(tl_interface, "recv_message")
                    and self.fd_aux is not None
                ):
                    try:

                        def _recv_msg_cb(msg_id, msg):
                            try:
                                self.get_logger().info(f"recv_message id={msg_id} msg={msg}")
                                # store or handle as needed
                            except Exception:
                                pass

                        tl_interface.recv_message(self.fd_aux, _recv_msg_cb)
                    except Exception:
                        pass
            except Exception:
                pass

            self.get_logger().info(
                f"[Connect]: successfully connected to arm at {ip}:{port},{port_aux}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"connect() failed: {e}")
            self.fd = 0
            self.fd_aux = 0
            self.is_connected_ = False
            return False

    def _startup_connect(self):
        """启动连接与上电时序，参考 C++ TL_Arm::init() 实现。

        时序：connect → 设置示教模式 → 清除错误 → 下电释放控制器 → 上电 → 延时2s
        """
        try:
            ip = self.get_parameter("arm_ip").get_parameter_value().string_value
            port = self.get_parameter("arm_port").get_parameter_value().string_value
            port_aux = self.get_parameter("arm_port_aux").get_parameter_value().string_value
        except Exception:
            ip, port, port_aux = "192.168.1.13", "6001", "7000"

        self.get_logger().info(f"Trying to connect to {ip}:{port},{port_aux}")
        ok = self.connect()
        if ok and self._valid_fd():
            self.get_logger().info(
                f"[Connect]: successfully connected to arm at {ip}:{port},{port_aux}"
            )

            # 切换到示教模式
            try:
                ret = tl_interface.set_current_mode(self.fd, 0)
                if ret != 0:
                    self.get_logger().error(f"[Init]: failed to set teach mode, result={ret}")
                else:
                    self.get_logger().info("[Init]: set teach mode success")
            except Exception as e:
                self.get_logger().warning(f"[Init]: set_current_mode exception: {e}")

            # 清除控制器错误和报警
            try:
                ret = tl_interface.clear_error(self.fd)
                if ret != 0:
                    self.get_logger().warn(f"[Init]: clear_error result={ret}")
                else:
                    self.get_logger().info("[Init]: clear_error success")
            except Exception as e:
                self.get_logger().warning(f"[Init]: clear_error exception: {e}")

            # 清错后下电，释放控制器占用状态
            try:
                if not self.power_off():
                    self.get_logger().warning("[Init]: power_off returned false, continuing...")
            except Exception as e:
                self.get_logger().warning(f"[Init]: power_off exception: {e}")

            # 上电
            power_ok = False
            try:
                power_ok = self.power_on()
            except Exception as e:
                self.get_logger().warning(f"[Init]: power_on exception: {e}")

            if not power_ok:
                self.get_logger().error("[Init]: 上电失败，跳过上电延时")
                return

            # 上电后延时，等待伺服状态稳定
            self.get_logger().info("[Init]: 上电延时2s...")
            time.sleep(2)
            self.get_logger().info("[Init]: 上电延时完成")

    def power_on(self) -> bool:
        """上电时序，对齐 C++ TL_Arm::power_on() 行为。

        状态约定：
          0 — 停止（stop）
          1 — 就绪（ready）
          2 — 报警（alarm，无法上电）
          3 — 运行中（running，已上电）

        Returns True on success (servo_state == 3), False otherwise.
        """
        if tl_interface is None or self.fd is None:
            self.get_logger().warning("[PowerOn]: tl_interface or fd missing")
            return False

        try:
            # 先查询当前伺服状态
            ret, state = tl_interface.get_servo_state(self.fd, -1)

            # 已上电，无需操作
            if state == 3:
                self.is_powered_ = True
                self.get_logger().info("[PowerOn]: already power on")
                return True

            # 报警状态 — 直接返回 false，不清错上电
            if state == 2:
                self.get_logger().error("[PowerOn]: servo alarm state (2), cannot power on")
                return False

            # 其他状态（0 或 1）→ 手动处理状态转换（NRC API 无 power_on 封装）
            if state == 0:
                tl_interface.set_servo_state(self.fd, 1)  # 停止 → 就绪

            tl_interface.set_servo_poweron(self.fd)

            # 验证最终状态
            ret, state = tl_interface.get_servo_state(self.fd, -1)

            if state == 3:
                self.is_powered_ = True
                self.get_logger().info(f"[PowerOn]: successfully power on, servo_state = {state}")
                return True
            else:
                self.get_logger().error(f"[PowerOn]: failed to power on, servo_state = {state}")
                return False

        except Exception as e:
            self.get_logger().warning(f"[PowerOn]: power_on sequence failed: {e}")
            return False

    def power_off(self) -> bool:
        """Power off sequence."""
        self.get_logger().info("power_off() called")
        if tl_interface is None or self.fd is None:
            self.get_logger().warning("power_off: tl_interface or fd missing")
            return False

        try:
            # 新版调用
            final_state = tl_interface.power_off(self.fd)
            self.is_powered_ = False
            self.get_logger().info(f"[PowerOff]: 下电完成，当前伺服状态 = {final_state}")
            return True

        except Exception as e:
            self.get_logger().warning(f"power_off failed: {e}")
            return False

    # service handlers
    def disconnect(self) -> bool:
        """Disconnect from robot controller and clear internal state."""
        self.get_logger().info("disconnect() called")
        with getattr(self, "_conn_lock", threading.Lock()):
            try:
                if tl_interface is not None:
                    try:
                        if self._valid_fd() and hasattr(tl_interface, "disconnect_robot"):
                            tl_interface.disconnect_robot(self.fd)
                    except Exception:
                        pass
                    try:
                        if (
                            self.fd_aux is not None
                            and int(self.fd_aux) > 0
                            and hasattr(tl_interface, "disconnect_robot")
                        ):
                            tl_interface.disconnect_robot(self.fd_aux)
                    except Exception:
                        pass
            except Exception as e:
                self.get_logger().warning(f"disconnect: exception during native disconnect: {e}")

            # clear state
            try:
                self.fd = None
                self.fd_aux = None
                self.is_connected_ = False
                self.is_powered_ = False
            except Exception:
                pass

        self.get_logger().info("[Disconnect]: completed")
        return True

    def handle_connect_service(self, request, response):
        ok = self.connect()
        response.success = bool(ok)
        response.message = "connected" if ok else "failed to connect"
        return response

    def handle_disconnect_service(self, request, response):
        ok = self.disconnect()
        response.success = bool(ok)
        response.message = "disconnected" if ok else "failed to disconnect"
        return response

    def handle_poweron_service(self, request, response):
        ok = self.power_on()
        response.success = bool(ok)
        response.message = "powered on" if ok else "failed to power on"
        return response

    def handle_poweroff_service(self, request, response):
        ok = self.power_off()
        response.success = bool(ok)
        response.message = "powered off" if ok else "failed to power off"
        return response

    def handle_clear_error_service(self, request, response):
        # call underlying clear_error with fd if available
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            ret_state = tl_interface.get_servo_state(self.fd)
            self.get_logger().info(f"get_servo_state ret={ret_state}")

            if ret_state == 0:
                ret = tl_interface.clear_error(self.fd)
                response.success = ret == 0
                if response.success:
                    response.message = "Clear error successfully"
                else:
                    response.message = f"Clear error failed: {ret}"
            else:
                response.success = False
                response.message = "Not an error state"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_clear_error_service failed: {e}")

        return response

    def handle_set_speed_service(self, request, response):
        # request has 'speed' field
        try:
            speed = float(request.speed)
        except Exception:
            response.success = False
            response.message = "invalid speed"
            return response

        # placeholder: pass to tl_interface if available
        ok = True
        if tl_interface is not None and self._valid_fd() and hasattr(tl_interface, "set_speed"):
            try:
                # SWIG binding expects an integer argument for speed
                _ret = tl_interface.set_speed(self.fd, int(speed))
                # native API returns 0 for SUCCESS; treat 0 as success
                if isinstance(_ret, bool):
                    ok = bool(_ret)
                elif isinstance(_ret, (int, float)):
                    ok = int(_ret) == 0
                else:
                    ok = bool(_ret)
                if not ok:
                    self.get_logger().error(f"tl_interface.set_speed() returned: {_ret}")
            except Exception as e:
                self.get_logger().error(f"tl_interface.set_speed() failed: {e}")
                ok = False

        response.success = bool(ok)
        response.message = "Set speed successfully" if ok else "Failed to set speed"
        return response

    def handle_get_speed_service(self, request, response):
        _ = request

        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            # 调用你新写的 get_speed 函数
            ret = tl_interface.get_speed(self.fd)
            self.get_logger().info(f"get_speed ret={ret}")

            # 返回值 != 0 代表失败
            if ret < 0:  # 你也可以直接写 if ret != 0:
                response.success = False
                response.message = "Failed to get speed"
                return response

            # 成功：ret 就是速度值
            response.success = True
            response.message = "Get speed successfully"
            response.speed = float(ret)  # 直接用 ret 即可

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_get_speed_service failed: {e}")

        return response

    def handle_get_quat2rpy_service(self, request, response):

        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        # 四元数必须是4维
        if len(request.input) < 4:
            response.success = False
            response.message = "Invalid quat input (need 4 elements)"
            return response

        try:
            quat_list = list(request.input)
            ret, roll, pitch, yaw = tl_interface.get_quat2rpy(self.fd, quat_list)
            self.get_logger().info(f"get_quat2rpy ret={ret}, rpy=[{roll}, {pitch}, {yaw}]")

            if ret == 0:  # 成功
                response.success = True
                response.message = "Get quat2rpy successfully"
                response.output = [roll, pitch, yaw]  # 直接组装成列表
            else:
                response.success = False
                response.message = f"Failed to get quat2rpy, ret={ret}"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_get_quat2rpy_service failed: {e}")

        return response

    def handle_get_rpy2quat_service(self, request, response):

        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        # RPY 至少需要 3 个元素
        if len(request.input) < 3:
            response.success = False
            response.message = "Invalid rpy input (need 3 elements)"
            return response

        try:
            rpy_list = list(request.input)
            ret, qx, qy, qz, qw = tl_interface.get_rpy2quat(self.fd, rpy_list)
            self.get_logger().info(f"get_rpy2quat ret={ret}, quat=[{qx}, {qy}, {qz}, {qw}]")

            if ret == 0:
                response.success = True
                response.message = "Get rpy2quat successfully"
                response.output = [qx, qy, qz, qw]  # 直接输出四元数
            else:
                response.success = False
                response.message = f"Failed to get rpy2quat, ret={ret}"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_get_rpy2quat_service failed: {e}")

        return response

    def handle_get_rpy2r_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        # 检查输入长度
        if len(request.input) < 3:
            response.success = False
            response.message = "Invalid rpy input (need 3 elements)"
            return response

        try:
            rpy_list = list(request.input)
            result = tl_interface.get_rpy2r(self.fd, rpy_list)
            ret = result[0]  # 第一个是返回值
            rot_list = list(result[1:])  # 后面 9 个是旋转矩阵
            self.get_logger().info(f"get_rpy2r ret={ret}, rot={rot_list}")

            if ret == 0:
                response.success = True
                response.message = "Get rpy2r successfully"
                response.output = rot_list  # 直接输出 9 个元素
            else:
                response.success = False
                response.message = "Failed to get rpy2r"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_get_rpy2r_service failed: {e}")

        return response

    def handle_get_tr2r_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        # 检查输入长度
        if len(request.input) < 16:
            response.success = False
            response.message = "Invalid tr input (need 16 elements)"
            return response

        try:
            tr_list = list(request.input)
            result = tl_interface.get_tr2r(self.fd, tr_list)
            ret = result[0]  # 第一个值是返回状态
            rot_list = list(result[1:])  # 后面 9 个是旋转矩阵
            self.get_logger().info(f"get_tr2r ret={ret}, rot={rot_list}")

            if ret == 0:
                response.success = True
                response.message = "Get tr2r successfully"
                response.output = rot_list
            else:
                response.success = False
                response.message = "Failed to get tr2r"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_get_tr2r_service failed: {e}")

        return response

    def handle_get_r2tr_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        # 检查输入长度
        if len(request.input) < 9:
            response.success = False
            response.message = "Invalid tr matrix input"
            return response

        try:
            r_list = list(request.input)
            result = tl_interface.get_r2tr(self.fd, r_list)
            ret = result[0]  # 第一个是状态码
            tr_list = list(result[1:])  # 后面 16 个是位姿矩阵
            self.get_logger().info(f"get_r2tr ret={ret}, tr_matrix={tr_list}")

            if ret == 0:
                response.success = True
                response.message = "Get r2tr successfully"
                response.output = tr_list
            else:
                response.success = False
                response.message = "Failed to get r2tr"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_get_r2tr_service failed: {e}")

        return response

    def handle_set_controller_ip_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            ret = tl_interface.set_controller_ip(
                self.fd, request.name, request.addr, request.gateway, request.dns
            )

            self.get_logger().info(
                f"set_controller_ip ret={ret}, "
                f"name={request.name}, "
                f"addr={request.addr}, "
                f"gateway={request.gateway}, "
                f"dns={request.dns}"
            )
            response.success = ret == 0
            if response.success:
                response.message = "Set controller IP successfully"
            else:
                response.message = "Failed to set controller IP"

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_set_controller_ip_service failed: {e}")

        return response

    def handle_get_controller_id_service(self, request, response):
        _ = request
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            controller_id = tl_interface.string(128)  # 分配128字节char*
            ret = tl_interface.get_controller_id(self.fd, id)
            id_str = controller_id.value.decode("utf-8", "ignore").strip()
            self.get_logger().info(f"get_controller_id ret={ret}, id={id_str}")
            # self.get_logger().info(f"get_controller_id ret={ret}, id={controller_id}")
            response.success = ret == 0

            if response.success:
                response.message = id_str
            else:
                response.message = "Failed to get controller ID"

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_get_controller_id_service failed: {e}")

        return response

    # Topic handlers
    def handle_movej_topic(self, msg):
        # msg: MoveCommand
        if self.fd is None or not self.is_connected_:
            self.get_logger().warn("[MoveJ]: arm is not connected")
            return

        try:
            target_pos = list(msg.target_pos_value)  # 直接获取位置数组

            ret = tl_interface.robot_movej(
                self.fd,
                coord=msg.coord,
                vel=msg.velocity,
                acc=msg.acc,
                dec=msg.dec,
                targetPos=tuple(target_pos),
            )

            # 结果打印
            if ret == 0:
                self.get_logger().info(f"[MoveJ] successfully! ret={ret}")
            else:
                self.get_logger().error(f"[MoveJ] failed! ret={ret}")

        except Exception as e:
            self.get_logger().error(f"[MoveJ] err: {str(e)}")

    def handle_movel_topic(self, msg):
        if self.fd is None or not self.is_connected_:
            self.get_logger().warn("[MoveL]: arm is not connected")
            return

        try:
            target_pos = list(msg.target_pos_value)  # 直接获取位置数组

            ret = tl_interface.robot_movel(
                self.fd,
                coord=msg.coord,
                vel=msg.velocity,
                acc=msg.acc,
                dec=msg.dec,
                targetPos=tuple(target_pos),
            )

            if ret == 0:
                self.get_logger().info(f"[MoveL] successfully! ret={ret}")
            else:
                self.get_logger().error(f"[MoveL] failed! ret={ret}")

        except Exception as e:
            self.get_logger().error(f"[MoveL] error: {e}")

    def handle_job_insert_movej_service(self, request, response):
        if not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            line = request.line
            cmd = request.cmd

            vel = cmd.velocity
            acc = cmd.acc
            dec = cmd.dec
            pl = cmd.pl
            pos_name = cmd.target_pos_name

            ret = tl_interface.job_insert_moveJ(self.fd, line, vel, acc, dec, pl, pos_name)
            response.success = ret == 0
            response.message = (
                "Job insert movej successfully"
                if response.success
                else "Failed to insert job movej"
            )
            self.get_logger().info(f"[JobInsertMoveJ]: ret={ret}")

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_job_insert_movej_service failed: {e}")

        return response

    def handle_job_insert_movel_service(self, request, response):
        if not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            line = request.line
            cmd = request.cmd

            vel = cmd.velocity
            acc = cmd.acc
            dec = cmd.dec
            pl = cmd.pl
            pos_name = cmd.target_pos_name

            ret = tl_interface.job_insert_moveL(self.fd, line, vel, acc, dec, pl, pos_name)
            response.success = ret == 0
            response.message = (
                "Job insert movel successfully"
                if response.success
                else "Failed to insert job movel"
            )
            self.get_logger().info(f"[JobInsertMoveL]: ret={ret}")

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_job_insert_movel_service failed: {e}")

        return response

    def handle_job_insert_imove_service(self, request, response):
        if not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            line = request.line
            cmd = request.cmd

            vel = cmd.velocity
            acc = cmd.acc
            dec = cmd.dec
            pl = cmd.pl
            pos_name = cmd.target_pos_name

            ret = tl_interface.job_insert_imove(self.fd, line, vel, acc, dec, pl, pos_name)
            response.success = ret == 0
            response.message = (
                "Job insert imove successfully"
                if response.success
                else "Failed to insert job imove"
            )
            self.get_logger().info(f"[JobInsertIMove]: ret={ret}")

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_job_insert_imove_service failed: {e}")

        return response

    def handle_job_insert_movec_service(self, request, response):
        if not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            line = request.line
            cmd = request.cmd

            vel = cmd.velocity
            acc = cmd.acc
            dec = cmd.dec
            pl = cmd.pl
            pos_name = cmd.target_pos_name

            ret = tl_interface.job_insert_moveC(self.fd, line, vel, acc, dec, pl, pos_name)
            response.success = ret == 0
            response.message = (
                "Job insert movec successfully"
                if response.success
                else "Failed to insert job movec"
            )
            self.get_logger().info(f"[JobInsertMoveC]: ret={ret}")

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_job_insert_movec_service failed: {e}")

        return response

    def handle_get_all_job_filename_service(self, request, response):
        import ctypes

        if self.fd is None:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            # ===================== 适配 C 语言接口 ======================
            MAX_FILES = 64  # 最大支持文件数
            FILENAME_LEN = 64  # 每个文件名长度固定 64 字节

            # 创建 C 语言二维数组：uint8_t buffer[MAX_FILES][64]
            buffer = (ctypes.c_uint8 * FILENAME_LEN * MAX_FILES)()

            # 调用 C 接口：maxFileNum 传 0 表示获取全部
            ret = tl_interface.job_get_all_jobfile_name(
                self.fd,
                robotNum=1,  # 机器号默认 1
                buffer=buffer,
                maxFileNum=0,  # 0 = 获取所有作业文件
            )

            # ==========================================================

            # 结果判断
            response.success = ret == 0
            if not response.success:
                response.message = f"Failed to get job files, ret={ret}"
                return response

            response.message = "Get all job filename successfully"

            # 解析文件名
            tmp_list = []
            job_file_msg = msgs.JobFileName()

            for i in range(MAX_FILES):
                # 取出第 i 个文件名
                name_bytes = bytes(buffer[i]).split(b"\x00")[0]  # 去掉 \0 截断
                if not name_bytes:
                    continue

                filename = name_bytes.decode("utf-8", errors="ignore")
                self.get_logger().info(f"job file {i}: {filename}")
                job_file_msg.file_name.append(filename)

            tmp_list.append(job_file_msg)
            response.robots_file = tmp_list

        except Exception as e:
            response.success = False
            response.message = f"Exception: {str(e)}"
            self.get_logger().error(f"get job files error: {str(e)}")

        return response

    def handle_job_run_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            job_name = str(request.job_name)
            ret = tl_interface.job_open(self.fd, job_name)
            ret1 = tl_interface.job_run(self.fd, job_name)
            self.get_logger().info(f"job_open ret={ret}, job_run ret1={ret1}, job_name={job_name}")
            response.success = ret1 == 0
            if response.success:
                response.message = "Job run successfully"
            else:
                response.message = "Failed to run job"
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_job_run_service failed: {e}")

        self.power_off()
        self.power_on()  # power on again to be ready for next command

        return response

    def handle_job_delete_service(self, request, response):
        try:
            job_name = str(request.job_name)
        except Exception:
            response.success = False
            response.message = "invalid request"
            return response
        ok = True
        try:
            if tl_interface is not None and hasattr(tl_interface, "job_delete"):
                try:
                    res = (
                        tl_interface.job_delete(self.fd, job_name)
                        if self._valid_fd()
                        else tl_interface.job_delete(job_name)
                    )
                    ok = _is_success(res)
                except TypeError:
                    res = tl_interface.job_delete(job_name)
                    ok = _is_success(res)
        except Exception as e:
            self.get_logger().error(f"job_delete failed: {e}")
            ok = False
        response.success = bool(ok)
        response.message = "deleted" if ok else "failed"
        return response

    def handle_set_current_mode_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        # mode 有效范围检查
        if request.mode < 0 or request.mode > 2:
            response.success = False
            response.message = "Invalid mode"
            return response

        try:
            ret = tl_interface.set_current_mode(self.fd, request.mode)

            self.get_logger().info(f"set_current_mode ret={ret}, mode={request.mode}")

            response.success = ret == 0
            response.message = (
                "Set current mode successfully"
                if response.success
                else "Failed to set current mode"
            )

        except Exception as e:
            self.get_logger().error(f"handle_set_current_mode_service failed: {e}")
            response.success = False
            response.message = str(e)

        return response

    def handle_get_current_mode_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            ret = tl_interface.get_current_mode(self.fd)
            self.get_logger().info(f"get_current_mode ret={ret}")

            # 赋值
            response.mode = int(ret)
            response.success = True
            if response.success:
                response.message = "Get current mode successfully"
            else:
                response.message = "Failed to get current mode"

        except Exception as e:
            response.success = False
            response.message = f"Exception: {str(e)}"
            response.mode = -1

        return response

    def handle_close_servoj_service(self, request, response):
        if self.fd_aux is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            ret = tl_interface.close_servoJ(self.fd_aux)
            response.success = ret == 0
            if response.success:
                response.message = "ServoJ close successfully"
            else:
                response.message = "Failed to close ServoJ"

            self.get_logger().info(f"close_servoJ ret={ret}")

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_close_servoj_service failed: {e}")

        return response

    def handle_set_servoj_pos_topic(self, msg):
        try:
            if not self.is_connected_:
                self.get_logger().warning("[SetServoJPos]: arm is not connected")
                return

            # ROS Float64MultiArray -> SWIG VectorDouble
            pos = tl_interface.VectorDouble()
            for v in msg.data:
                pos.append(float(v))

            # call sdk
            ret = tl_interface.set_servoJ_pos(self.fd_aux, pos)
            self.get_logger().info(f"[SetServoJPos]: ret={ret}")

        except Exception as e:

            self.get_logger().error(f"handle_set_servoj_pos_topic failed: {e}")

    # Global position / coord transform / reachability / dh param
    def handle_set_global_pos_service(self, request, response):
        # 检查机械臂连接状态
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        # 检查全局点名称是否合法
        def is_valid_gp(name: str) -> bool:
            try:
                if len(name) != 6:
                    return False
                if not name.startswith("GP"):
                    return False
                num = int(name[2:])
                return 1 <= num <= 9999

            except Exception:
                return False

        if not is_valid_gp(request.pos_name):
            response.success = False
            response.message = "Invalid global pos name"
            return response

        try:
            pos_name = request.pos_name
            pos_info = list(request.pos_info)

            self.get_logger().info(f"set_global_position pos_name={pos_name}, pos_info={pos_info}")

            # ===================== 调用新版函数 ======================
            ret = tl_interface.set_global_position(
                self.fd, posName=pos_name, posInfo=pos_info  # 直接传 Python 列表即可
            )

            # ==========================================================
            self.get_logger().info(f"set_global_position ret={ret}")

            response.success = ret == 0
            if response.success:
                response.message = "Set global pos successfully"
            else:
                response.message = "Failed to set global pos"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_set_global_pos_service failed: {e}")

        return response

    def handle_get_global_pos_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        def is_valid_gp(name):
            if len(name) != 6:
                return False
            if not name.startswith("GP"):
                return False
            try:
                num = int(name[2:])
                return 1 <= num <= 9999
            except Exception:
                return False

        if not is_valid_gp(request.pos_name):
            response.success = False
            response.message = "Invalid global pos name"
            return response

        try:
            # 调用新版 get_global_position 函数（直接返回 list）
            pos_list = tl_interface.get_global_position(self.fd, request.pos_name)

            # 判断是否成功（返回的列表长度正确即为成功）
            if len(pos_list) == 14:
                response.success = True
                response.message = "Get global pos successfully"
                response.pos = [float(val) for val in pos_list]
            else:
                response.success = False
                response.message = "Failed to get global pos"
                response.pos = []

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_get_global_pos_service failed: {e}")

        return response

    def handle_coord_transform_service(self, request, response):
        try:
            origin = int(request.origin_coord)
            target = int(request.target_coord)
            form = int(request.form)
            origin_pos = list(request.origin_pos)
            reference = list(request.reference_pos)
        except Exception:
            response.success = False
            response.message = "invalid request"
            response.target_pos = []
            return response
        ok = True
        target_pos = []
        try:
            # 新版函数：origin_to_target_coord
            target_pos = []
            if self._valid_fd():
                # 调用新版坐标转换函数
                target_pos = tl_interface.origin_to_target_coord(
                    self.fd,
                    originCoord=origin,  # 原坐标系
                    originPos=origin_pos,  # 原坐标位置 (长度7)
                    targetCoord=target,  # 目标坐标系
                )

            # 判断是否成功
            ok = len(target_pos) == 7
            self.get_logger().info(
                f"coord_transform origin={origin}, target={target}, "
                f"origin_pos={origin_pos}, target_pos={target_pos}, ok={ok}"
            )

        except Exception as e:
            self.get_logger().error(f"coord_transform failed: {e}")
            ok = False
            target_pos = []

        response.success = bool(ok)
        response.message = "ok" if ok else "failed"
        response.target_pos = list(target_pos)
        return response

    def handle_get_pos_reachable_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        if request.move_type not in ["MOVJ", "MOVL"]:
            response.success = False
            response.message = "Invalid move type"
            return response

        try:
            # Python list -> VectorDouble
            query_pos = tl_interface.VectorDouble()
            for v in request.pos:
                query_pos.append(float(v))
            # bool& 输出参数
            ret, reachable = tl_interface.get_pos_reachable(
                self.fd, query_pos, request.move_type, False
            )
            self.get_logger().info(f"get_pos_reachable ret={ret}, " f"reachable={reachable}")
            if ret == 0:
                response.success = bool(reachable)
                if response.success:
                    response.message = "Target pos is reachable"
                else:
                    response.message = "Target pos is not reachable"
            else:
                response.success = False
                response.message = "Fail to get pos reachable status"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_get_pos_reachable_service failed: {e}")
        return response

    def handle_get_dh_param_service(self, request, response):
        dh_param = tl_interface.CRobotDHParam()
        ret = tl_interface.get_robot_dh_param(self.fd, dh_param)

        response.success = ret == 0
        if response.success:
            response.message = "Get DH param successfully"
        else:
            response.message = f"Failed to get DH param, ret={ret}"

        param_msg = msgs.RobotDHParam()

        param_msg.l1 = float(dh_param.L1)
        param_msg.l2 = float(dh_param.L2)
        param_msg.l3 = float(dh_param.L3)
        param_msg.l4 = float(dh_param.L4)
        param_msg.l5 = float(dh_param.L5)
        param_msg.l6 = float(dh_param.L6)
        param_msg.l7 = float(dh_param.L7)
        param_msg.l8 = float(dh_param.L8)
        param_msg.l9 = float(dh_param.L9)
        param_msg.l10 = float(dh_param.L10)
        param_msg.l11 = float(dh_param.L11)
        param_msg.l12 = float(dh_param.L12)
        param_msg.l13 = float(dh_param.L13)
        param_msg.l14 = float(dh_param.L14)
        param_msg.l15 = float(dh_param.L15)
        param_msg.l16 = float(dh_param.L16)
        param_msg.l17 = float(dh_param.L17)
        param_msg.l18 = float(dh_param.L18)
        param_msg.l19 = float(dh_param.L19)
        param_msg.l20 = float(dh_param.L20)

        # 耦合系数
        param_msg.couple_coe_1_2 = float(dh_param.Couple_Coe_1_2)
        param_msg.couple_coe_2_3 = float(dh_param.Couple_Coe_2_3)
        param_msg.couple_coe_3_2 = float(dh_param.Couple_Coe_3_2)
        param_msg.couple_coe_3_4 = float(dh_param.Couple_Coe_3_4)
        param_msg.couple_coe_4_5 = float(dh_param.Couple_Coe_4_5)
        param_msg.couple_coe_4_6 = float(dh_param.Couple_Coe_4_6)
        param_msg.couple_coe_5_6 = float(dh_param.Couple_Coe_5_6)

        # 动态限制
        param_msg.dynamic_limit_max = float(dh_param.dynamicLimit_max)
        param_msg.dynamic_limit_min = float(dh_param.dynamicLimit_min)

        # 螺距/导程
        param_msg.pitch = float(dh_param.pitch)
        param_msg.sliding_lead_value = float(dh_param.sliding_lead_value)
        param_msg.uplift_lead_value = float(dh_param.uplift_lead_value)
        param_msg.spray_distance = float(dh_param.spray_distance)

        # 轴方向
        param_msg.three_axis_direction = float(dh_param.threeAxisDirection)
        param_msg.five_axis_direction = float(dh_param.fiveAxisDirection)

        # 转换比
        param_msg.two_axis_convertion_ratio = float(dh_param.twoAxisConversionRatio)
        param_msg.three_axis_convertion_ratio = float(dh_param.threeAxisConversionRatio)
        param_msg.amplification_ratio = float(dh_param.amplificationRatio)

        param_msg.convertion_ratio_x = float(dh_param.conversionratio_x)
        param_msg.convertion_ratio_y = float(dh_param.conversionratio_y)
        param_msg.convertion_ratio_z = float(dh_param.conversionratio_z)

        param_msg.convertion_ratio_j1 = float(dh_param.conversionratio_J1)
        param_msg.convertion_ratio_j2 = float(dh_param.conversionratio_J2)
        param_msg.convertion_ratio_j3 = float(dh_param.conversionratio_J3)

        # 安装方向 int 类型不用改
        param_msg.upside_down = dh_param.upsideDown

        # 汉语参数 PC / SP / TL
        param_msg.pc = float(dh_param.hanyu)
        param_msg.sp = []
        param_msg.tl = []

        try:
            param_msg.sp = [
                dh_param.hanyu.SP.__getitem__(0),
                dh_param.hanyu.SP.__getitem__(1),
                dh_param.hanyu.SP.__getitem__(2),
            ]
        except:
            param_msg.sp = [0.0, 0.0, 0.0]

        try:
            param_msg.tl = [
                dh_param.hanyu.TL.__getitem__(0),
                dh_param.hanyu.TL.__getitem__(1),
                dh_param.hanyu.TL.__getitem__(2),
            ]
        except:
            param_msg.tl = [0.0, 0.0, 0.0]

        response.param = param_msg
        return response

    def handle_set_dh_param_service(self, request, response):

        try:

            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            dh_param = tl_interface.RobotDHParam()

            # L1 ~ L20
            dh_param.L1 = request.param.l1
            dh_param.L2 = request.param.l2
            dh_param.L3 = request.param.l3
            dh_param.L4 = request.param.l4
            dh_param.L5 = request.param.l5
            dh_param.L6 = request.param.l6
            dh_param.L7 = request.param.l7
            dh_param.L8 = request.param.l8
            dh_param.L9 = request.param.l9
            dh_param.L10 = request.param.l10
            dh_param.L11 = request.param.l11
            dh_param.L12 = request.param.l12
            dh_param.L13 = request.param.l13
            dh_param.L14 = request.param.l14
            dh_param.L15 = request.param.l15
            dh_param.L16 = request.param.l16
            dh_param.L17 = request.param.l17
            dh_param.L18 = request.param.l18
            dh_param.L19 = request.param.l19
            dh_param.L20 = request.param.l20

            # 耦合系数
            dh_param.Couple_Coe_1_2 = request.param.couple_coe_1_2
            dh_param.Couple_Coe_2_3 = request.param.couple_coe_2_3
            dh_param.Couple_Coe_3_2 = request.param.couple_coe_3_2
            dh_param.Couple_Coe_3_4 = request.param.couple_coe_3_4
            dh_param.Couple_Coe_4_5 = request.param.couple_coe_4_5
            dh_param.Couple_Coe_4_6 = request.param.couple_coe_4_6
            dh_param.Couple_Coe_5_6 = request.param.couple_coe_5_6

            # 动态限制
            dh_param.dynamicLimit_max = request.param.dynamic_limit_max
            dh_param.dynamicLimit_min = request.param.dynamic_limit_min

            # 其它参数
            dh_param.pitch = request.param.pitch
            dh_param.sliding_lead_value = request.param.sliding_lead_value
            dh_param.uplift_lead_value = request.param.uplift_lead_value
            dh_param.spray_distance = request.param.spray_distance

            dh_param.threeAxisDirection = request.param.three_axis_direction
            dh_param.fiveAxisDirection = request.param.five_axis_direction

            dh_param.twoAxisConversionRatio = request.param.two_axis_convertion_ratio
            dh_param.threeAxisConversionRatio = request.param.three_axis_convertion_ratio
            dh_param.amplificationRatio = request.param.amplification_ratio

            dh_param.conversionratio_x = request.param.convertion_ratio_x
            dh_param.conversionratio_y = request.param.convertion_ratio_y
            dh_param.conversionratio_z = request.param.convertion_ratio_z

            dh_param.conversionratio_J1 = request.param.convertion_ratio_j1
            dh_param.conversionratio_J2 = request.param.convertion_ratio_j2
            dh_param.conversionratio_J3 = request.param.convertion_ratio_j3

            dh_param.upsideDown = request.param.upside_down

            # hanyu
            dh_param.hanyu.PC = request.param.pc

            # SP
            for i in range(min(3, len(request.param.sp))):
                dh_param.hanyu.SP.__setitem__(i, request.param.sp[i])

            # TL
            for i in range(min(3, len(request.param.tl))):
                dh_param.hanyu.TL.__setitem__(i, request.param.tl[i])

            # 调用SDK
            ret = tl_interface.set_robot_dh_param(self.fd, dh_param)

            response.success = ret == 0
            if response.success:
                response.message = "Set DH param successfully"
            else:
                response.message = f"Failed to set DH param, ret={ret}"

        except Exception as e:

            self.get_logger().error(f"handle_set_dh_param_service failed: {e}")

            response.success = False
            response.message = f"call failed: {e}"

        self.power_off()
        self.power_on()

        return response

    # Track / trajectory handlers
    def handle_track_save_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            ret = tl_interface.track_record_save(self.fd, request.traj_name)
            self.get_logger().info(f"track_record_save ret={ret}, traj_name={request.traj_name}")
            response.success = ret == 0
            if response.success:
                response.message = "Track save successfully"
            else:
                response.message = "Failed to save track"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_track_save_service failed: {e}")

        return response

    def handle_track_playback_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            ret = tl_interface.track_record_playback(self.fd, request.vel)
            self.get_logger().info(f"track_record_playback ret={ret}, vel={request.vel}")
            response.success = ret == 0
            if response.success:
                response.message = "Track playback successfully"
            else:
                response.message = "Failed to playback track"

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_track_playback_service failed: {e}")

        return response

    def handle_open_servoj_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            # ========= 1. move request data =========
            vmax = list(request.vmax)
            amax = list(request.amax)
            jmax = list(request.jmax)

            # ========= 2. call native interface =========
            ret = tl_interface.open_servoJ(self.fd_aux, vmax, amax, jmax)

            self.get_logger().info(f"open_servoJ ret={ret}")

            # ========= 3. response =========
            response.success = ret == 0
            response.message = (
                "ServoJ open successfully" if response.success else "Failed to open ServoJ"
            )

            return response

        except Exception as e:
            self.get_logger().error(f"handle_open_servoj_service failed: {e}")
            response.success = False
            response.message = str(e)
            return response

    # Queue control services
    def handle_queue_motion_set_status_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            # 设置连续运动状态
            ret = tl_interface.queue_motion_set_status(self.fd, request.status)

            response.success = ret == 0
            if response.success:
                response.message = "Set queue motion status successfully"
            else:
                response.message = "Failed to set queue motion status"

            self.get_logger().info(f"queue_motion_set_status ret={ret}, status={request.status}")
            # 关闭连续运动模式后设置为示教模式
            if not request.status:
                ret_mode = tl_interface.set_current_mode(self.fd, 0)
                self.get_logger().info(f"set_current_mode ret={ret_mode}")

                if ret_mode == 0:
                    self.power_off()

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_queue_motion_set_status_service failed: {e}")

        return response

    def handle_queue_motion_movej_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            cmd = request.cmd
            pos_list = list(cmd.target_pos_value)  # 目标位置

            # ===================== 调用新版全自动队列函数 ======================
            tl_interface.queue_moveJ(
                self.fd,
                coord=cmd.coord,
                vel=cmd.velocity,
                acc=cmd.acc,
                dec=cmd.dec,
                pl=cmd.pl,
                pos_list=pos_list,
                wait_time=10.0,
            )

            response.success = True
            response.message = "Queue motion moveJ execute successfully"

        except Exception as e:
            self.get_logger().error(f"handle_queue_motion_movej_service failed: {e}")
            response.success = False
            response.message = str(e)

        return response

    def handle_queue_motion_stop_service(self, request, response):
        if self.fd is None or self.fd <= 0:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            ret, status = tl_interface.queue_motion_get_status(self.fd, False)

            self.get_logger().info(f"queue_motion_get_status ret={ret}, status={status}")

            if ret != 0:
                response.success = False
                response.message = "Failed to get queue motion status"
                return response

            if not status:
                response.success = False
                response.message = "Queue motion is not enabled"
                return response

            # 停止 queue motion
            ret = tl_interface.queue_motion_stop_not_power_off(self.fd)
            self.get_logger().info(f"queue_motion_stop_not_power_off ret={ret}")
            response.success = ret == 0
            if response.success:
                response.message = "Queue motion moveJ stop successfully"
            else:
                response.message = "Failed to stop queue motion moveJ"

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_queue_motion_stop_service failed: {e}")

        return response

    def handle_start_jogging_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            # 获取机械臂自由度
            ndof = len(self.get_parameter("arm_joints").get_parameter_value().string_array_value)

            # 检查轴号是否合法
            if request.axis < 1 or request.axis > ndof:
                response.success = False
                response.message = "Invalid axis"
                return response

            self.get_logger().info(
                f"robot_start_jogging axis={request.axis}, " f"direction={request.direction}"
            )

            # 调用底层接口
            ret = tl_interface.robot_start_jogging(self.fd, request.axis, request.direction)
            self.get_logger().info(f"robot_start_jogging ret={ret}")
            response.success = ret == 0

            if response.success:
                response.message = "Start jogging successfully"
            else:
                response.message = "Failed to start jogging"

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_start_jogging_service failed: {e}")

        return response

    def handle_stop_jogging_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            # 获取机械臂自由度
            ndof = len(self.get_parameter("arm_joints").get_parameter_value().string_array_value)

            # 检查轴号是否合法
            if request.axis < 1 or request.axis > ndof:
                response.success = False
                response.message = "Invalid axis"
                return response

            self.get_logger().info(f"robot_stop_jogging axis={request.axis}")

            # 调用底层接口
            ret = tl_interface.robot_stop_jogging(self.fd, request.axis)
            self.get_logger().info(f"robot_stop_jogging ret={ret}")
            response.success = ret == 0

            if response.success:
                response.message = "Stop jogging successfully"
            else:
                response.message = "Failed to stop jogging"

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_stop_jogging_service failed: {e}")

        return response

    def handle_get_robot_state_service(self, request, response):

        if self.fd_aux is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:

            self.msg_received = False

            param = tl_interface.RobotState()

            param.channel = int(request.channel)
            param.stop = bool(request.stop)
            param.mode = int(request.mode)
            param.interval = int(request.interval)

            param.ioState = bool(request.io_state)

            param.position = int(request.position)

            param.dataildmotionpos = bool(request.detail_motion_pos)

            param.posSum = int(request.pos_sum)

            # 不要clear vector
            # 很容易触发 SWIG 崩溃

            ret = tl_interface.get_robot_state(self.fd_aux, param)

            self.get_logger().info(f"get_robot_state ret={ret}")

            if ret != 0:
                response.success = False
                response.message = f"get_robot_state failed ret={ret}"
                return response

            timeout = 5.0
            start_time = time.time()

            while not self.msg_received:

                if time.time() - start_time > timeout:
                    response.success = False
                    response.message = "Timeout waiting robot state"
                    return response

                time.sleep(0.01)

            response.success = True
            response.message = self.latest_robot_state

            return response

        except Exception as e:

            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_get_robot_state_service failed: {e}")

            return response

    def handle_get_library_version_service(self, request, response):
        if self.fd is None:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            version = tl_interface.get_library_version()

            if not version:
                response.success = False
                response.message = "Failed to get library version"
                return response

            response.success = True
            response.message = str(version)
            return response

        except Exception as e:
            self.get_logger().error(f"get_library_version failed: {e}")
            response.success = False
            response.message = str(e)
            return response

    def handle_get_robot_joint_param_service(self, request, response):
        if not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        if request.id < 1 or request.id > self.ndof_:
            response.success = False
            response.message = "Invalid id"
            return response

        try:
            param = tl_interface.CRobotJointParam()
            ret = tl_interface.get_robot_joint_param(self.fd, request.id, param)
        except Exception as e:
            response.success = False
            response.message = f"call failed: {e}"
            return response

        if ret == 0:
            response.success = True
            response.message = "Get robot joint param successfully"

            response.param.reduction_ratio = float(param.reducRatio)
            response.param.encoder_resolution = int(param.encoderResolution)  # 必须 int
            response.param.pos_sw_limit = float(param.posSWLimit)
            response.param.neg_sw_limit = float(param.negSWLimit)
            response.param.rated_rot_speed = float(param.ratedRotSpeed)
            response.param.rated_derot_speed = float(param.deRatedRotSpeed)
            response.param.max_rot_speed = float(param.maxRotSpeed)
            response.param.max_derot_speed = float(param.maxDeRotSpeed)
            response.param.rated_vel = float(param.ratedVel)
            response.param.rated_devel = float(param.deRatedVel)
            response.param.max_acc = float(param.maxAcc)
            response.param.max_dec = float(param.maxDecel)
            response.param.direction = int(param.direction)  # 必须 int
        else:
            response.success = False
            response.message = "Failed to get robot joint param"

        return response

    def handle_set_robot_joint_param_service(self, request, response):

        try:
            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            if request.id < 1 or request.id > self.ndof_:
                response.success = False
                response.message = "Invalid id"
                return response

            # 创建 RobotJointParam 结构体
            param = tl_interface.RobotJointParam()

            # 填充参数
            param.reducRatio = float(request.param.reduction_ratio)
            param.encoderResolution = int(request.param.encoder_resolution)
            param.posSWLimit = float(request.param.pos_sw_limit)
            param.negSWLimit = float(request.param.neg_sw_limit)
            param.ratedRotSpeed = float(request.param.rated_rot_speed)
            param.ratedDeRotSpeed = float(request.param.rated_derot_speed)
            param.maxRotSpeed = float(request.param.max_rot_speed)
            param.maxDeRotSpeed = float(request.param.max_derot_speed)
            param.ratedVel = float(request.param.rated_vel)
            param.deRatedVel = float(request.param.rated_devel)
            param.maxAcc = float(request.param.max_acc)
            param.maxDecel = float(request.param.max_deacc)
            param.direction = int(request.param.direction)

            # 调用底层接口
            ret = tl_interface.set_robot_joint_param(self.fd, request.id, param)
            self.get_logger().info(f"set_robot_joint_param ret={ret}")
            response.success = ret == 0
            response.message = (
                "Set robot joint param successfully"
                if ret == 0
                else "Failed to set robot joint param"
            )

        except Exception as e:
            self.get_logger().error(f"handle_set_robot_joint_param_service failed: {e}")
            response.success = False
            response.message = f"call failed: {e}"

        self.power_off()
        self.power_on()

        return response

    def handle_set_drag_mode_service(self, request, response):
        try:
            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            if request.mode < 0 or request.mode > 3:
                response.success = False
                response.message = "Invalid mode"
                return response

            ret = tl_interface.set_darg_mode(self.fd, request.mode)

            response.success = ret == 0

            if response.success:
                response.message = "Set drag mode successfully"
            else:
                response.message = "Failed to set drag mode"

        except Exception as e:
            self.get_logger().error(f"handle_set_drag_mode_service failed: {e}")

            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_get_drag_status_service(self, request, response):
        try:
            _ = request

            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            # for name in dir(tl_interface):
            #     if "Bool" in name or "bool" in name:
            #         self.get_logger().info(name)
            #     else: self.get_logger().error("no bool type??")

            flag = False
            ret, end_flag = tl_interface.get_drag_thread_is_end(self.fd, flag)

            if ret == 0:
                response.success = bool(end_flag)

                if response.success:
                    response.message = "Drag ended"
                else:
                    response.message = "Drag not ended"

            else:
                response.success = False
                response.message = "Failed to get drag status"

        except Exception as e:
            self.get_logger().error(f"handle_get_drag_status_service failed: {e}")

            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_get_joint_temperature_service(self, request, response):

        try:
            _ = request

            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            temperatures = tl_interface.VectorDouble()
            ret = tl_interface.get_joint_temperature(self.fd, temperatures)
            self.get_logger().info(f"get_joint_temperature ret={ret}")
            if ret == 0:
                response.success = True
                response.message = "Get joint temperatures successfully"
                response.temperatures = [float(x) for x in temperatures]

            else:
                response.success = False
                response.message = "Failed to get joint temperatures"

        except Exception as e:
            self.get_logger().error(f"handle_get_joint_temperature_service failed: {e}")
            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_get_joint_voltage_service(self, request, response):
        try:
            _ = request

            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            joint_voltage = tl_interface.VectorDouble()
            positioner_voltage = tl_interface.VectorDouble()
            ret = tl_interface.get_joint_voltage(self.fd, joint_voltage, positioner_voltage)
            self.get_logger().info(f"get_joint_voltage ret={ret}")

            if ret == 0:
                response.success = True
                response.message = "Get joint voltage successfully"
                response.joint_voltage = [float(x) for x in joint_voltage]
                response.positioner_voltage = [float(x) for x in positioner_voltage]
            else:
                response.success = False
                response.message = "Failed to get joint voltage"

        except Exception as e:
            self.get_logger().error(f"handle_get_joint_voltage_service failed: {e}")

            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_get_motor_current_service(self, request, response):

        try:
            _ = request

            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            motor_current = tl_interface.VectorDouble()
            ret = tl_interface.get_current_motor_current_independent(self.fd, motor_current)
            self.get_logger().info(f"get_current_motor_current_independent ret={ret}")

            if ret == 0:
                response.success = True
                response.message = "Get motor current successfully"
                response.motor_current = [float(x) for x in motor_current]

            else:
                response.success = False
                response.message = "Failed to get motor current"

        except Exception as e:
            self.get_logger().error(f"handle_get_motor_current_service failed: {e}")

            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_get_current_motor_torque_service(self, request, response):

        try:
            _ = request

            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            ret, motor_torque_list, motor_torque_sync_list = tl_interface.get_current_motor_torque(
                self.fd
            )
            self.get_logger().info(f"get_current_motor_torque ret={ret}")

            if ret == 0:
                response.success = True
                response.message = "Get motor torque successfully"
                response.motor_torque = [int(x) for x in motor_torque_list]
                response.motor_torque_sync = [int(x) for x in motor_torque_sync_list]
            else:
                response.success = False
                response.message = "Failed to get motor torque"

        except Exception as e:
            self.get_logger().error(f"handle_get_motor_torque_service failed: {e}")
            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_get_current_line_joint_speed_service(self, request, response):

        try:
            _ = request

            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            ret, line_speed, joint_speed_list, joint_speed_sync_list = (
                tl_interface.get_current_line_speed_and_joint_speed(self.fd)
            )
            self.get_logger().info(f"get_current_line_speed_and_joint_speed ret={ret}")

            if ret == 0:
                response.success = True
                response.message = "Get current line joint speed successfully"
                response.line_speed = float(line_speed)
                response.joint_speed = [float(x) for x in joint_speed_list]
                response.joint_speed_sync = [float(x) for x in joint_speed_sync_list]
            else:
                response.success = False
                response.message = "Failed to get current line joint speed"

        except Exception as e:
            self.get_logger().error(f"handle_get_current_line_joint_speed_service failed: {e}")
            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_get_joint_software_version_service(self, request, response):

        try:
            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            # 轴号检查
            if request.axis_num < 1 or request.axis_num > self.ndof_:
                response.success = False
                response.message = "Invalid axis num"
                return response
            for name in dir(tl_interface):
                if "String" in name or "string" in name:
                    self.get_logger().info(name)

            version_string = ""
            ret = tl_interface.query_joint_software_version(
                self.fd, request.axis_num, version_string
            )
            self.get_logger().info(f"query_joint_software_version ret={ret}")

            if ret == 0:
                # VectorChar -> string
                version_str = "".join([chr(c) for c in version_string]).rstrip("\x00")
                response.success = True
                response.message = version_str

            else:
                response.success = False
                response.message = "Failed to get joint software version"

        except Exception as e:
            self.get_logger().error(f"handle_get_joint_software_version_service failed: {e}")
            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_get_nexmotion_lib_version_service(self, request, response):

        try:
            _ = request

            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            # 尝试使用 Python str
            version = ""
            ret = tl_interface.get_nexmotion_lib_version(self.fd, version)

            self.get_logger().info(f"get_nexmotion_lib_version ret={ret}")

            if ret == 0:
                response.success = True
                response.message = str(version)
            else:
                response.success = False
                response.message = "Failed to get nexmotion lib version"

        except Exception as e:
            self.get_logger().error(f"handle_get_nexmotion_lib_version_service failed: {e}")

            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_restore_default_dh_param_service(self, request, response):

        try:
            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            ret = tl_interface.restore_default_param_DH(self.fd, request.robot_num)
            response.success = ret == 0
            if response.success:
                response.message = "Restore default DH param successfully"
            else:
                response.message = "Failed to restore default DH param"

        except Exception as e:
            self.get_logger().error(f"handle_restore_default_dh_param_service failed: {e}")

            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_set_default_cartesian_param_service(self, request, response):

        try:
            _ = request
            if self.is_connected_:
                response.success = False
                response.message = "Arm already connected"
                return response

            ret = tl_interface.set_default_cartesian_params(self.fd)

            response.success = ret == 0

            if response.success:
                response.message = "Set default cartesian param successfully"
            else:
                response.message = "Failed to set default cartesian param"

        except Exception as e:
            self.get_logger().error(f"handle_set_default_cartesian_param_service failed: {e}")

            response.success = False
            response.message = f"call failed: {e}"

        return response

    def handle_log_download_service(self, request, response):

        try:
            if not self.is_connected_:
                response.success = False
                response.message = "Arm is not connected"
                return response

            ret = tl_interface.log_download_by_quantity(
                self.fd, request.count, request.directory_path
            )

            response.success = ret == 0

            if response.success:
                response.message = "Log download successfully"
            else:
                response.message = "Failed to download log"

        except Exception as e:
            self.get_logger().error(f"handle_log_download_service failed: {e}")

            response.success = False
            response.message = f"call failed: {e}"

        return response

    # Tool / Coordinate handlers
    def handle_set_tool_param_service(self, request, response):
        # 检查机械臂连接状态
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            # 创建底层 ToolParam 对象
            param = tl_interface.ToolParam()

            # 位姿参数
            param.X = request.param.x
            param.Y = request.param.y
            param.Z = request.param.z

            param.A = request.param.a
            param.B = request.param.b
            param.C = request.param.c

            # 负载参数
            param.payloadMass = request.param.payload_mass
            param.payloadInertia = request.param.payload_inertia

            # 负载质心
            param.payloadMassCenter_X = request.param.payload_mass_center_x
            param.payloadMassCenter_Y = request.param.payload_mass_center_y
            param.payloadMassCenter_Z = request.param.payload_mass_center_z

            self.get_logger().info(f"set_tool_hand_param tool_num={request.tool_num}")

            # 调用底层接口
            ret = tl_interface.set_tool_hand_param(self.fd, request.tool_num, param)
            self.get_logger().info(f"set_tool_hand_param ret={ret}")
            response.success = ret == 0

            if response.success:
                response.message = "Set tool hand param successfully"
            else:
                response.message = "Failed to set tool hand param"

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_set_tool_param_service failed: {e}")

        return response

    def handle_set_user_coord_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            # 提取位姿数据
            pos_list = [
                request.pos.position.x,
                request.pos.position.y,
                request.pos.position.z,
                request.pos.rpy.x,
                request.pos.rpy.y,
                request.pos.rpy.z,
            ]

            self.get_logger().info(
                f"set_user_coordinate_data " f"user_num={request.user_num}, " f"pos={pos_list}"
            )

            # 转换为 SWIG VectorDouble
            pos = tl_interface.VectorDouble()
            for v in pos_list:
                pos.append(float(v))

            # 调用底层接口
            ret = tl_interface.set_user_coordinate_data(self.fd, request.user_num, pos)
            self.get_logger().info(f"set_user_coordinate_data ret={ret}")
            response.success = ret == 0

            if response.success:
                response.message = "Set user coordinate successfully"
            else:
                response.message = "Failed to set user coordinate"

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_set_user_coord_service failed: {e}")

        return response

    def handle_set_axis_zero_pos_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            ret = tl_interface.set_axis_zero_position(self.fd, request.axis)

            self.get_logger().info(f"set_axis_zero_position ret={ret}, axis={request.axis}")

            response.success = ret == 0
            response.message = (
                "Set axis zero position successfully"
                if response.success
                else "Failed to set axis zero position"
            )

        except Exception as e:
            self.get_logger().error(f"handle_set_axis_zero_pos_service failed: {e}")
            response.success = False
            response.message = str(e)

        return response

    def handle_set_current_coord_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            ret = tl_interface.set_current_coord(self.fd, request.coord)

            self.get_logger().info(f"set_current_coord ret={ret}, coord={request.coord}")

            response.success = ret == 0
            response.message = (
                "Set current coordinate successfully"
                if response.success
                else "Failed to set current coordinate"
            )

        except Exception as e:
            self.get_logger().error(f"handle_set_current_coord_service failed: {e}")
            response.success = False
            response.message = str(e)

        return response

    def handle_get_current_coord_service(self, request, response):
        if not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            coord = tl_interface.get_current_coord(self.fd)

            response.success = True
            response.message = "Get current coordinate successfully"
            response.coord = coord

        except Exception as e:
            # 失败
            response.success = False
            response.message = f"Failed to get current coordinate: {str(e)}"

        return response

    def handle_get_coord_num_service(self, request, response):
        # returns tool_num, user_num
        if self.fd is None:
            response.success = False
            response.message = "Arm is not connected"
            return response
        try:
            # self.get_logger().info(tl_interface.get_tool_hand_number.__doc__)
            ret, tool_num = tl_interface.get_tool_hand_number(self.fd, -1)
            ret1, user_num = tl_interface.get_user_coord_number(self.fd, -1)
            # self.get_logger().info(f"get_tool_hand_number return: ret={ret}, tool_num={tool_num}")
            # self.get_logger().info(f"get_user_coord_number return: ret={ret1}, user_num={user_num}")
            if ret == 0 and ret1 == 0:
                response.success = True
                response.message = "Get all coordinate number successfully"
                response.tool_num = tool_num
                response.user_num = user_num
            else:
                response.success = False
                response.message = "Failed to get all coordinate number"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_get_coord_num_service failed: {e}")

        return response

    # IO / Modbus handlers
    def handle_set_digital_output_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        # value 只能为 0 或 1
        if request.value < 0 or request.value > 1:
            response.success = False
            response.message = "Invalid value"
            return response

        try:
            ret = tl_interface.set_digital_output(self.fd, request.port, request.value)

            self.get_logger().info(
                f"set_digital_output ret={ret}, " f"port={request.port}, value={request.value}"
            )

            response.success = ret == 0
            response.message = (
                "Set digital output successfully"
                if response.success
                else "Failed to set digital output"
            )

        except Exception as e:
            self.get_logger().error(f"handle_set_digital_output_service failed: {e}")
            response.success = False
            response.message = str(e)

        return response

    def handle_get_digital_input_output_service(self, request, response):
        if self.fd is None:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:

            digital_input = tl_interface.VectorInt()
            digital_output = tl_interface.VectorInt()

            ret = tl_interface.get_digital_input(self.fd, digital_input)
            ret1 = tl_interface.get_digital_output(self.fd, digital_output)
            if ret == 0 and ret1 == 0:
                response.success = True
                response.message = "Get digital input output successfully"

                # ROS2 sequence<int32>
                response.input = [int(v) for v in digital_input]
                response.output = [int(v) for v in digital_output]
            else:
                response.success = False
                response.message = "Failed to get digital input and output"

        except Exception as e:
            response.success = False
            response.message = str(e)

            self.get_logger().error(f"handle_get_digital_input_output_service failed: {e}")

        return response

    def handle_modbus_write_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            master_id = request.master_id
            master_param = request.master_param
            start_addr = master_param.start_addr

            # 1. 根据类型设置 Modbus 主站参数（新版接口）
            if master_param.type == "TCP":
                ip = master_param.tcp.ip
                port = master_param.tcp.port

                # 新版 TCP 接口
                ret = tl_interface.modbus_set_master_parameter_tcp(
                    self.fd, id=master_id, ip=ip, port=port, startAddress=start_addr
                )

            elif master_param.type == "RTU":
                slave_id = master_param.rtu.slave_id
                port = master_param.rtu.port
                baudrate = master_param.rtu.baudrate
                check_bit = master_param.rtu.check_bit
                data_bit = master_param.rtu.data_bit
                stop_bit = master_param.rtu.stop_bit

                # 新版 RTU 接口
                ret = tl_interface.modbus_set_master_parameter_rtu(
                    self.fd,
                    id=master_id,
                    slaveId=slave_id,
                    rtu_port=port,
                    baudrate=baudrate,
                    checkBit=check_bit,
                    dataBit=data_bit,
                    stopBit=stop_bit,
                    startAddress=start_addr,
                )

            else:
                response.success = False
                response.message = "Invalid master type"
                return response

            self.get_logger().info(f"modbus_set_master_parameter ret={ret}")
            if ret != 0:
                response.success = False
                response.message = "Failed to set master parameter"
                return response

            # 2. 打开主站（不变）
            ret = tl_interface.modbus_open_master(self.fd, master_id)
            self.get_logger().info(f"modbus_open_master ret={ret}")

            if ret != 0:
                response.success = False
                response.message = "Failed to open master"
                return response

            # 3. 批量写保持寄存器（新版接口，直接传 list，无需 VectorInt）
            ret = tl_interface.modbus_write_multiple_holding_registers(
                self.fd, id=master_id, address=request.addr, data=list(request.data)
            )

            self.get_logger().info(f"modbus_write ret={ret}")

            # 4. 返回结果
            response.success = ret == 0
            response.message = (
                "Modbus write successfully" if response.success else "Failed to write Modbus"
            )

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_modbus_write_service failed: {e}")

        return response

    def handle_modbus_read_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response

        try:
            master_id = request.master_id
            master_param = request.master_param
            start_addr = master_param.start_addr

            # 根据类型分别设置参数
            if master_param.type == "TCP":
                ip = master_param.tcp.ip
                port = master_param.tcp.port

                # 新版 TCP 接口
                ret = tl_interface.modbus_set_master_parameter_tcp(
                    self.fd, id=master_id, ip=ip, port=port, startAddress=start_addr
                )

            elif master_param.type == "RTU":
                slave_id = master_param.rtu.slave_id
                port = master_param.rtu.port
                baudrate = master_param.rtu.baudrate
                check_bit = master_param.rtu.check_bit
                data_bit = master_param.rtu.data_bit
                stop_bit = master_param.rtu.stop_bit

                # 新版 RTU 接口
                ret = tl_interface.modbus_set_master_parameter_rtu(
                    self.fd,
                    id=master_id,
                    slaveId=slave_id,
                    rtu_port=port,
                    baudrate=baudrate,
                    checkBit=check_bit,
                    dataBit=data_bit,
                    stopBit=stop_bit,
                    startAddress=start_addr,
                )

            else:
                response.success = False
                response.message = "Invalid master type"
                return response

            self.get_logger().info(f"modbus_set_master_parameter ret={ret}")
            if ret != 0:
                response.success = False
                response.message = "Failed to set master parameter"
                return response

            # 打开主站（不变）
            ret = tl_interface.modbus_open_master(self.fd, master_id)
            self.get_logger().info(f"modbus_open_master ret={ret}")
            if ret != 0:
                response.success = False
                response.message = "Failed to open master"
                return response

            # 新版读寄存器（直接返回 list，无 VectorInt）
            data_list = tl_interface.modbus_read_holding_registers(
                self.fd, id=master_id, address=request.addr, quantity=request.quantity
            )

            self.get_logger().info(f"modbus_read_holding_registers data={data_list}")

            if data_list is not None and len(data_list) == request.quantity:
                response.success = True
                response.message = "Modbus read successfully"
                response.data = data_list
            else:
                response.success = False
                response.message = "Failed to read Modbus"

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"handle_modbus_read_service failed: {e}")

        return response

    def publish_arm_state(self):
        if not self.is_connected_ or not self._valid_fd():
            return
        # Publish arm_status
        try:
            if hasattr(self, "running_status_pub"):
                st = msgs.ArmStatus()
                st.stamp = self.get_clock().now().to_msg()
                st.run_state = "STOP"
                try:

                    if tl_interface is not None and hasattr(tl_interface, "get_robot_state"):

                        ret = tl_interface.RobotState()
                        state = tl_interface.get_robot_state(self.fd, ret)
                        # self.get_logger().info(f'robot_state={state}')
                        if state == 0:
                            st.run_state = "STOP"
                        elif state == 1:
                            st.run_state = "PAUSE"
                        elif state == 2:
                            st.run_state = "RUNNING"
                except Exception as e:
                    self.get_logger().warning(f"get_robot_state failed: {e}")

                self.running_status_pub.publish(st)
        except Exception as e:
            self.get_logger().debug(f"publish arm_status failed: {e}")

        # Publish joint_state
        try:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            joints = self.get_parameter("arm_joints").get_parameter_value().string_array_value
            js.name = joints

            pos_ok = False
            if tl_interface is not None and hasattr(tl_interface, "get_current_position"):
                try:
                    pos_list = tl_interface.get_current_position(self.fd, 0)  # 直接返回Python列表
                    if pos_list and len(pos_list) >= 6:
                        ndof = len(joints)

                        # 6轴机器人取前6个，7轴取7个
                        if ndof == 6:
                            pos = pos_list[:6]  # 直接用角度
                        else:
                            pos = pos_list[:ndof]  # 直接用角度

                        js.position = pos
                        pos_ok = True
                except Exception as e:
                    self.get_logger().error(f"get_current_position failed: {e}")
            if not pos_ok:
                js.position = [0.0] * len(joints)

            js.velocity = [0.0] * len(joints)
            js.effort = []
            self.joint_state_pub.publish(js)
        except Exception as e:
            self.get_logger().debug(f"publish joint_state failed: {e}")

        # pulish tcp_pose
        try:

            if hasattr(self, "tcp_pose_pub") and self.tcp_pose_pub is not None:

                msg = msgs.CartesianPose()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"
                pose_ok = False
                if tl_interface is not None and hasattr(tl_interface, "get_current_position"):
                    try:

                        cartesian_list = tl_interface.get_current_position(self.fd, 1)

                        if cartesian_list and len(cartesian_list) >= 6:
                            # 直接赋值，无需遍历
                            msg.position.x = cartesian_list[0]
                            msg.position.y = cartesian_list[1]
                            msg.position.z = cartesian_list[2]

                            # rpy
                            msg.rpy.x = cartesian_list[3]
                            msg.rpy.y = cartesian_list[4]
                            msg.rpy.z = cartesian_list[5]

                            # arm angle
                            ndof = len(
                                self.get_parameter("arm_joints")
                                .get_parameter_value()
                                .string_array_value
                            )
                            if ndof == 6:
                                msg.arm_angle = 0.0
                            elif ndof == 7 and len(cartesian_list) > 6:
                                msg.arm_angle = cartesian_list[6]

                            pose_ok = True

                    except Exception as e:

                        self.get_logger().error(f"get_current_position tcp failed: {e}")
            self.tcp_pose_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"publish tcp_pose failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TLArmNode()
    # Multi-threaded executor matching C++ node (max(4, hardware_concurrency))
    num_threads = max(4, os.cpu_count() or 4)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(node)
    node.get_logger().info(f"Starting MultiThreadedExecutor with {num_threads} threads")

    # flag to prevent re-entrance on double Ctrl+C
    _shutting_down = False

    try:
        executor.spin()
    except KeyboardInterrupt:
        if _shutting_down:
            pass  # ignore second Ctrl+C
        else:
            _shutting_down = True
            node.get_logger().info("[Shutdown]: Ctrl+C received, powering off...")
            try:
                node.power_off()
            except Exception:
                pass
            try:
                node.disconnect()
            except Exception:
                pass
    finally:
        try:
            executor.remove_node(node)
            node.destroy_node()
        except Exception:
            pass

        # rclpy.shutdown() may be called already by the launch framework;
        # catch RCLError (or a generic exception) and ignore it to avoid
        # "rcl_shutdown already called" crashes on process exit.
        try:
            from rclpy.exceptions import RCLError
        except Exception:
            RCLError = Exception

        try:
            rclpy.shutdown()
        except RCLError:
            pass
        except Exception:
            pass


if __name__ == "__main__":
    main()
