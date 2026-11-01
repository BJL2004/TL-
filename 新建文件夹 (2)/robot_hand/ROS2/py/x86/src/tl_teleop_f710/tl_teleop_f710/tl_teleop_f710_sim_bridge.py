#!/usr/bin/env python3
"""天链机械臂 F710 手柄遥操作 — Gazebo 仿真桥接节点（Pinocchio 版）。

接收 /tl_driver/set_servol_pos（ServolMove）笛卡尔位姿，
使用 Pinocchio 库本地求解 IK（阻尼伪逆法），发送到 Gazebo position controller。
"""

import math
import os
import threading

import numpy as np
import pinocchio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tl_ros2_interface.msg import ServolMove


class ServolSimBridge(Node):
    def __init__(self):
        super().__init__("tl_teleop_f710_sim_bridge")
        self.declare_parameter(
            "position_controller_topic", "/tcb_group_position_controller/commands"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("ik_eps", 1e-4)
        self.declare_parameter("ik_max_iter", 200)
        self.declare_parameter("ik_dt", 0.5)
        self.declare_parameter("arm_type", "tcb605")

        self._pos_ctrl_topic = self.get_parameter("position_controller_topic").value
        self._js_topic = self.get_parameter("joint_state_topic").value
        self._ik_eps = self.get_parameter("ik_eps").value
        self._ik_max_iter = int(self.get_parameter("ik_max_iter").value)
        self._ik_dt = self.get_parameter("ik_dt").value
        self._arm_type = self.get_parameter("arm_type").value

        # ---- 初始化 Pinocchio 模型 ----
        self._model = None
        self._data = None
        self._tip_joint_id = None
        self._tip_frame_id = None
        self._init_pinocchio()

        ndof = self._model.nq if self._model is not None else 6
        self._current_joints = np.zeros(ndof)
        self._joints_lock = threading.Lock()
        self._joint_states_received = False

        # ---- 话题 ----
        self._servol_sub = self.create_subscription(
            ServolMove, "/tl_driver/set_servol_pos", self._servol_callback, 10
        )
        # 也订阅 servoj（关节角直接下发，如回零），度→弧度→转发
        self._servoj_sub = self.create_subscription(
            Float64MultiArray, "/tl_driver/set_servoj_pos", self._servoj_callback, 10
        )
        self._js_sub = self.create_subscription(
            JointState, self._js_topic, self._joint_state_callback, 10
        )
        self._pos_pub = self.create_publisher(Float64MultiArray, self._pos_ctrl_topic, 10)

        self.get_logger().info(
            f"仿真桥接节点已启动（Pinocchio IK, "
            f"max_iter={self._ik_max_iter}, eps={self._ik_eps})"
        )

    def _init_pinocchio(self):
        arm_type = self._arm_type
        urdf_paths = [
            os.path.expanduser(f"~/tl_robot_ros2_py/src/tl_description/urdf/{arm_type}.urdf"),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg = get_package_share_directory("tl_description")
            urdf_paths.insert(0, os.path.join(pkg, "urdf", f"{arm_type}.urdf"))
        except Exception:
            pass
        loaded = False
        for path in urdf_paths:
            if os.path.exists(path):
                self.get_logger().info(f"加载 URDF: {path}")
                self._model = pinocchio.buildModelFromUrdf(path)
                loaded = True
                break
        if not loaded:
            raise FileNotFoundError(f"URDF not found: {urdf_paths}")
        self._data = self._model.createData()
        ndof = self._model.nq
        tip_joint = f"joint{ndof}"
        tip_link = f"link{ndof}"
        self._tip_joint_id = self._model.getJointId(tip_joint)
        self._tip_frame_id = self._model.getFrameId(tip_link)
        self.get_logger().info(
            f"Pinocchio 模型加载完成: {self._model.name}, "
            f"nq={ndof}, tip_joint={tip_joint}, tip_frame={tip_link}"
        )

    def _solve_ik(self, x, y, z, rx, ry, rz, q_guess=None):
        if q_guess is None:
            with self._joints_lock:
                q_guess = self._current_joints.copy()
        c, s = math.cos, math.sin
        cr, sr = c(rx * 0.5), s(rx * 0.5)
        cp, sp = c(ry * 0.5), s(ry * 0.5)
        cy, sy = c(rz * 0.5), s(rz * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        R = pinocchio.Quaternion(qw, qx, qy, qz).toRotationMatrix()
        target = pinocchio.SE3(R, np.array([x, y, z]))
        q = q_guess.copy().astype(np.float64)
        success = False
        for _ in range(self._ik_max_iter):
            pinocchio.forwardKinematics(self._model, self._data, q)
            pinocchio.updateFramePlacements(self._model, self._data)
            dMi = target.actInv(self._data.oMf[self._tip_frame_id])
            err = pinocchio.log(dMi).vector
            if np.linalg.norm(err) < self._ik_eps:
                success = True
                break
            J = pinocchio.computeJointJacobian(self._model, self._data, q, self._tip_joint_id)
            v = -np.linalg.pinv(J) @ err
            q = pinocchio.integrate(self._model, q, v * self._ik_dt)
        return q if success else None

    def _joint_state_callback(self, msg):
        joint_names = [f"joint{i+1}" for i in range(self._model.nq)]
        positions = {}
        for i, name in enumerate(msg.name):
            if name in joint_names:
                positions[name] = msg.position[i]
        if len(positions) == self._model.nq:
            with self._joints_lock:
                self._current_joints = np.array([positions[name] for name in joint_names])
            self._joint_states_received = True

    def _servoj_callback(self, msg):
        """接收 servoj 关节角（度）→ 弧度 → 转发到 Gazebo。"""
        if not self._joint_states_received:
            return
        joints_deg = list(msg.data)
        if len(joints_deg) < self._model.nq:
            return
        joints_rad = [math.radians(v) for v in joints_deg[: self._model.nq]]
        with self._joints_lock:
            self._current_joints = np.array(joints_rad)
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joints_rad
        self._pos_pub.publish(cmd_msg)

    def _servol_callback(self, msg):
        if not self._joint_states_received:
            return
        target = list(msg.target_pose)
        if len(target) < 6:
            return
        x_m = target[0] / 1000.0
        y_m = target[1] / 1000.0
        z_m = target[2] / 1000.0
        q = self._solve_ik(x_m, y_m, z_m, target[3], target[4], target[5])
        if q is None:
            self.get_logger().warning(
                f"IK 失败 target=({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})",
                throttle_duration_sec=2.0,
            )
            return
        with self._joints_lock:
            self._current_joints = q.copy()
        cmd_msg = Float64MultiArray()
        cmd_msg.data = q.tolist()
        self._pos_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServolSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
