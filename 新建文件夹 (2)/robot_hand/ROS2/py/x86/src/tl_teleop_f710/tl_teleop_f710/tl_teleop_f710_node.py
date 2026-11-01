#!/usr/bin/env python3
"""天链机械臂 Logitech F710 手柄遥操作节点。

真机模式：节点做 IK（coord_transform）→ 发布 servoj 关节角 → tl_driver → 机械臂。
仿真模式：节点发布 servol 笛卡尔位姿 → sim_bridge(Pinocchio IK) → Gazebo。
"""

import math
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tl_ros2_interface.msg import ServolMove
from tl_ros2_interface.srv import CoordTransform, SetCurrentMode, SetSpeed, OpenServoJ


class F710TeleopNode(Node):
    def __init__(self):
        super().__init__("tl_teleop_f710_node")
        self._declare_parameters()
        self._load_parameters()
        self._latest_joy = None
        self._last_dpad_time = 0.0
        self._last_a_press = 0.0
        self._last_b_press = 0.0
        self._last_emergency_time = 0.0
        self.speed_value_ = self.speed_default_
        self.target_pose_ = None
        self._stop_mode = False
        ndof = len(self.home_joints_)
        self._last_joint_cmd = list(self.home_joints_)
        self._joint_cmd_lock = threading.Lock()
        self._ik_pending = False
        self._ik_thread = None

        self._servoj_pub = self.create_publisher(Float64MultiArray, "/tl_driver/set_servoj_pos", 10)
        self._servol_pub = self.create_publisher(ServolMove, "/tl_driver/set_servol_pos", 10)
        self.joy_sub_ = self.create_subscription(Joy, "/joy", self._joy_callback, 10)

        self._connect_client = self.create_client(Trigger, "/tl_driver/connect_arm")
        self._power_on_client = self.create_client(Trigger, "/tl_driver/power_on")
        self._set_mode_client = self.create_client(SetCurrentMode, "/tl_driver/set_current_mode")
        self._set_speed_client = self.create_client(SetSpeed, "/tl_driver/set_speed")
        self._open_servoj_client = self.create_client(OpenServoJ, "/tl_driver/open_servoj")
        self._close_servoj_client = self.create_client(Trigger, "/tl_driver/close_servoj")
        self._coord_transform_client = self.create_client(
            CoordTransform, "/tl_driver/coord_transform"
        )
        self._init_state = 0
        self._init_future = None

        # ==================== 仿真模式 FK（仅用于 target_pose_ 同步） ====================
        self._fk_ready = False
        self._fk_model = None
        self._fk_data = None
        self._fk_tip_frame = None
        if self.simulation_mode_:
            self._init_fk_pinocchio()
            p = self._compute_fk_home()
            if p is not None:
                self.target_pose_ = list(p)

        self.create_timer(1.0, self._init_servoj)
        self.control_timer_ = self.create_timer(1.0 / self.control_rate_, self._control_loop)
        self.get_logger().info(f"F710 遥操作节点已启动 ({self.control_rate_}Hz)")

    def _declare_parameters(self):
        self.declare_parameter("control_rate", 250.0)
        self.declare_parameter("simulation_mode", False)
        self.declare_parameter("speed_default", 50.0)
        self.declare_parameter("speed_min", 5.0)
        self.declare_parameter("speed_max", 100.0)
        self.declare_parameter("speed_step", 5.0)
        self.declare_parameter("servo_speed", 25.0)
        self.declare_parameter("servo_vmax", 300.0)
        self.declare_parameter("servo_amax", 3000.0)
        self.declare_parameter("servo_jmax", 50000.0)
        self.declare_parameter("pos_sensitivity", 160.0)
        self.declare_parameter("rot_sensitivity", 1.0)
        self.declare_parameter("deadzone", 0.15)
        self.declare_parameter("arm_type", "tcb605")
        self.declare_parameter("home_joints", [0.0] * 6)
        self.declare_parameter("workspace_limits", [-500, 500, -500, 500, 0, 800])
        self.declare_parameter("axis_left_x", 0)
        self.declare_parameter("axis_left_y", 1)
        self.declare_parameter("axis_right_x", 3)
        self.declare_parameter("axis_right_y", 4)
        self.declare_parameter("axis_dpad_x", 6)
        self.declare_parameter("axis_dpad_y", 7)
        self.declare_parameter("btn_a", 1)
        self.declare_parameter("btn_b", 2)
        self.declare_parameter("btn_lb", 4)
        self.declare_parameter("btn_rb", 5)
        self.declare_parameter("btn_back", 8)
        self.declare_parameter("btn_start", 9)

    def _load_parameters(self):
        p = self.get_parameter
        self.control_rate_ = p("control_rate").value
        self.simulation_mode_ = p("simulation_mode").value
        self.speed_default_ = p("speed_default").value
        self.speed_min_ = p("speed_min").value
        self.speed_max_ = p("speed_max").value
        self.speed_step_ = p("speed_step").value
        self.servo_speed_ = p("servo_speed").value
        self.servo_vmax_ = p("servo_vmax").value
        self.servo_amax_ = p("servo_amax").value
        self.servo_jmax_ = p("servo_jmax").value
        self.pos_sensitivity_ = p("pos_sensitivity").value
        self.rot_sensitivity_ = p("rot_sensitivity").value
        self.deadzone_ = p("deadzone").value
        self.home_joints_ = list(p("home_joints").value)
        self.arm_type_ = p("arm_type").value
        self.workspace_limits_ = list(p("workspace_limits").value)
        self.axis_left_x_ = int(p("axis_left_x").value)
        self.axis_left_y_ = int(p("axis_left_y").value)
        self.axis_right_x_ = int(p("axis_right_x").value)
        self.axis_right_y_ = int(p("axis_right_y").value)
        self.axis_dpad_x_ = int(p("axis_dpad_x").value)
        self.axis_dpad_y_ = int(p("axis_dpad_y").value)
        self.btn_a_ = int(p("btn_a").value)
        self.btn_b_ = int(p("btn_b").value)
        self.btn_lb_ = int(p("btn_lb").value)
        self.btn_rb_ = int(p("btn_rb").value)
        self.btn_back_ = int(p("btn_back").value)
        self.btn_start_ = int(p("btn_start").value)

    @staticmethod
    def _apply_deadzone(value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return (abs(value) - deadzone) / (1.0 - deadzone) * (1.0 if value > 0 else -1.0)

    def _apply_workspace_limits(self, pose):
        lo = self.workspace_limits_
        pose[0] = max(lo[0], min(lo[1], pose[0]))
        pose[1] = max(lo[2], min(lo[3], pose[1]))
        pose[2] = max(lo[4], min(lo[5], pose[2]))
        return pose

    def _publish_servoj(self):
        with self._joint_cmd_lock:
            joints = list(self._last_joint_cmd)
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joints]
        self._servoj_pub.publish(msg)

    def _publish_servol(self):
        if self.target_pose_ is None:
            return
        msg = ServolMove()
        msg.target_pose = [float(v) for v in self.target_pose_]
        msg.step_size = 5.0
        msg.coord = 1
        self._servol_pub.publish(msg)

    def _init_fk_pinocchio(self):
        """加载 Pinocchio 模型用于仿真模式 FK。"""
        try:
            import pinocchio

            arm_type = self.arm_type_
            urdf_paths = [
                os.path.expanduser(f"~/tl_robot_ros2_py/src/tl_description/urdf/{arm_type}.urdf"),
            ]
            try:
                from ament_index_python.packages import get_package_share_directory

                pkg = get_package_share_directory("tl_description")
                urdf_paths.insert(0, os.path.join(pkg, "urdf", f"{arm_type}.urdf"))
            except Exception:
                pass
            for path in urdf_paths:
                if os.path.exists(path):
                    self._fk_model = pinocchio.buildModelFromUrdf(path)
                    break
            if self._fk_model is None:
                self.get_logger().warning("FK 模型加载失败")
                return
            self._fk_data = self._fk_model.createData()
            ndof = self._fk_model.nq
            self._fk_tip_frame = self._fk_model.getFrameId(f"link{ndof}")
            self._fk_ready = True
            self.get_logger().info(f"FK（Pinocchio）就绪: {ndof} 轴")
        except Exception as e:
            self.get_logger().warning(f"FK 初始化失败: {e}")

    def _compute_fk_home(self):
        """仿真模式：用 Pinocchio 计算 home_joints 对应的笛卡尔位姿。"""
        if not self._fk_ready:
            return None
        import pinocchio
        import numpy as np, math

        joints_deg = list(self.home_joints_)
        q = np.array([math.radians(v) for v in joints_deg], dtype=np.float64)
        pinocchio.forwardKinematics(self._fk_model, self._fk_data, q)
        pinocchio.updateFramePlacements(self._fk_model, self._fk_data)
        placement = self._fk_data.oMf[self._fk_tip_frame]
        x = placement.translation[0] * 1000.0
        y = placement.translation[1] * 1000.0
        z = placement.translation[2] * 1000.0
        R = placement.rotation
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.asin(-R[2, 0])
        rz = math.atan2(R[1, 0], R[0, 0])
        return [x, y, z, rx, ry, rz]

    def _home_joints_to_pose(self):
        jd = list(self.home_joints_)
        ndof = len(jd)
        if not self.simulation_mode_:
            if not self._coord_transform_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("coord_transform 服务不可用")
                return None
            req = CoordTransform.Request()
            req.origin_coord = 0
            req.target_coord = 1
            req.form = 0
            req.origin_pos = list(jd) + [0.0] * (7 - ndof)
            req.reference_pos = [0.0] * 7
            try:
                ret = self._coord_transform_client.call(req)
                if ret.success and len(ret.target_pos) >= 6:
                    return list(ret.target_pos[:6])
                self.get_logger().error(f"FK 失败: {ret.message}")
            except Exception as e:
                self.get_logger().error(f"FK 异常: {e}")
            return None
        else:
            return self._compute_fk_home()

    def _async_ik(self, target_pose):
        ndof = len(self.home_joints_)
        req = CoordTransform.Request()
        req.origin_coord = 1
        req.target_coord = 0
        req.form = 0
        req.origin_pos = list(target_pose) + [0.0] * (7 - 6)
        req.reference_pos = [0.0] * 7
        try:
            ret = self._coord_transform_client.call(req)
            if ret.success and len(ret.target_pos) >= ndof:
                with self._joint_cmd_lock:
                    self._last_joint_cmd = list(ret.target_pos[:ndof])
            else:
                self.get_logger().warning(f"IK 失败: {ret.message}", throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().warning(f"IK 异常: {e}", throttle_duration_sec=2.0)
        finally:
            self._ik_pending = False

    def _init_servoj(self):
        if self.simulation_mode_:
            self._init_state = 6
            return
        s = self._init_state
        if s == 6:
            return
        if s == 0:
            for cli in [
                self._connect_client,
                self._set_mode_client,
                self._set_speed_client,
                self._open_servoj_client,
            ]:
                if not cli.wait_for_service(timeout_sec=0.1):
                    self.get_logger().info("等待 tl_driver 服务...", throttle_duration_sec=3.0)
                    return
            self.get_logger().info("ServoJ 初始化中...")
            self._init_future = self._connect_client.call_async(Trigger.Request())
            self._init_state = 1
        elif s == 1:
            if not self._init_future.done():
                return
            if not self._init_future.result().success:
                self.get_logger().error(f"连接失败: {self._init_future.result().message}")
                self._init_state = 6
                return
            self.get_logger().info("机械臂已连接")
            self._init_future = self._power_on_client.call_async(Trigger.Request())
            self._init_state = 2
        elif s == 2:
            if not self._init_future.done():
                return
            if not self._init_future.result().success:
                self.get_logger().error(f"上电失败: {self._init_future.result().message}")
                self._init_state = 6
                return
            self.get_logger().info("机械臂已上电")
            req = SetCurrentMode.Request()
            req.mode = 2
            self._init_future = self._set_mode_client.call_async(req)
            self._init_state = 3
        elif s == 3:
            if not self._init_future.done():
                return
            if not self._init_future.result().success:
                self.get_logger().error(f"设置模式失败: {self._init_future.result().message}")
                self._init_state = 6
                return
            self.get_logger().info("模式已设为远程(2)")
            req = SetSpeed.Request()
            req.speed = self.servo_speed_
            self._init_future = self._set_speed_client.call_async(req)
            self._init_state = 4
        elif s == 4:
            if not self._init_future.done():
                return
            if not self._init_future.result().success:
                self.get_logger().error(f"设置速度失败: {self._init_future.result().message}")
                self._init_state = 6
                return
            self.get_logger().info(f"速度已设为 {self.servo_speed_}")
            req = OpenServoJ.Request()
            req.vmax = [self.servo_vmax_] * 7
            req.amax = [self.servo_amax_] * 7
            req.jmax = [self.servo_jmax_] * 7
            self._init_future = self._open_servoj_client.call_async(req)
            self._init_state = 5
        elif s == 5:
            if not self._init_future.done():
                return
            if not self._init_future.result().success:
                self.get_logger().error(f"开启 ServoJ 失败: {self._init_future.result().message}")
                self._init_state = 6
                return
            self.get_logger().info("ServoJ 已开启 ✅")
            self._init_state = 6
            self._after_init()

    def _after_init(self):
        if self.simulation_mode_:
            return
        p = self._home_joints_to_pose()
        if p is not None:
            self.target_pose_ = list(p)

    def _close_servoj(self):
        if self._init_state < 5:
            return
        try:
            self._close_servoj_client.call_async(Trigger.Request())
            self.get_logger().info("ServoJ 已关闭")
        except Exception:
            pass

    def _joy_callback(self, msg):
        self._latest_joy = msg

    def _control_loop(self):
        joy = self._latest_joy
        if joy is not None:
            now = time.time()
            back = joy.buttons[self.btn_back_] if self.btn_back_ < len(joy.buttons) else 0
            start = joy.buttons[self.btn_start_] if self.btn_start_ < len(joy.buttons) else 0
            if back == 1 and start == 1 and (now - self._last_emergency_time) > 0.5:
                self._last_emergency_time = now
                self._stop_mode = not self._stop_mode
                self.get_logger().warn(f"紧急停止 {'启用' if self._stop_mode else '解除'}")
                if self._stop_mode:
                    with self._joint_cmd_lock:
                        self._last_joint_cmd = list(self.home_joints_)
                    self._publish_servoj()
                    return
        if self._stop_mode:
            self._publish_servoj()
            return
        if joy is None:
            # 真机模式：保持 250Hz servoj 流不间断
            if not self.simulation_mode_:
                self._publish_servoj()
            return
        na = max(self.axis_left_x_, self.axis_left_y_, self.axis_right_x_, self.axis_right_y_)
        nb = max(
            self.btn_a_, self.btn_b_, self.btn_lb_, self.btn_rb_, self.btn_back_, self.btn_start_
        )
        if len(joy.axes) <= na or len(joy.buttons) <= nb:
            if not self.simulation_mode_:
                self._publish_servoj()
            return
        now = time.time()
        dy = joy.axes[self.axis_dpad_y_] if self.axis_dpad_y_ < len(joy.axes) else 0.0
        hd = self.axis_dpad_y_ < len(joy.axes)
        if joy.buttons[self.btn_a_] == 1 and (now - self._last_a_press) > 0.5:
            self._last_a_press = now
            with self._joint_cmd_lock:
                self._last_joint_cmd = list(self.home_joints_)
            self.speed_value_ = self.speed_default_
            # 同步更新 target_pose_，使后续摇杆增量从正确起点计算
            new_pose = self._home_joints_to_pose()
            if new_pose is not None:
                self.target_pose_ = list(new_pose)
            self._publish_servoj()
            if self.simulation_mode_:
                self._publish_servoj()
            self.get_logger().info("回零")
        if joy.buttons[self.btn_b_] == 1 and (now - self._last_b_press) > 0.5:
            self._last_b_press = now
            self._publish_servoj()
            return
        if hd and dy != 0.0 and (now - self._last_dpad_time) > 0.3:
            self.speed_value_ = max(
                self.speed_min_,
                min(
                    self.speed_max_,
                    self.speed_value_ + (self.speed_step_ if dy > 0 else -self.speed_step_),
                ),
            )
            self._last_dpad_time = now
        lx = self._apply_deadzone(joy.axes[self.axis_left_x_], self.deadzone_)
        ly = self._apply_deadzone(joy.axes[self.axis_left_y_], self.deadzone_)
        rx = self._apply_deadzone(joy.axes[self.axis_right_x_], self.deadzone_)
        ry = self._apply_deadzone(joy.axes[self.axis_right_y_], self.deadzone_)
        if self.target_pose_ is None:
            if not self.simulation_mode_:
                self._publish_servoj()
            return
        if any(abs(v) > 0.001 for v in (lx, ly, rx, ry)):
            dt = 1.0 / self.control_rate_
            sc = self.speed_value_ / 100.0
            dx = lx * self.pos_sensitivity_ * sc * dt
            dy = ly * self.pos_sensitivity_ * sc * dt
            dz = ry * self.pos_sensitivity_ * sc * dt
            dr = rx * self.rot_sensitivity_ * sc * dt
            lb = joy.buttons[self.btn_lb_] == 1
            rb = joy.buttons[self.btn_rb_] == 1
            roll = pitch = 0.0
            if lb and not rb:
                roll = dr
                dr = 0.0
            elif rb and not lb:
                pitch = dr
                dr = 0.0
            self.target_pose_[0] += dx
            self.target_pose_[1] += dy
            self.target_pose_[2] += dz
            self.target_pose_[3] += roll
            self.target_pose_[4] += pitch
            self.target_pose_[5] += dr
            self.target_pose_ = self._apply_workspace_limits(self.target_pose_)
            if not self.simulation_mode_:
                if not self._ik_pending:
                    self._ik_pending = True
                    self._ik_thread = threading.Thread(
                        target=self._async_ik, args=(list(self.target_pose_),), daemon=True
                    )
                    self._ik_thread.start()
            else:
                self._publish_servol()
        if not self.simulation_mode_:
            self._publish_servoj()


def main(args=None):
    rclpy.init(args=args)
    node = F710TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，退出")
    finally:
        node._close_servoj()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
