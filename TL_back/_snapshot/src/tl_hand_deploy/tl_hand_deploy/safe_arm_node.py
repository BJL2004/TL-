import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from tl_ros2_interface.msg import ArmStatus, CartesianPose, MoveCommand

from .validation import (
    ValidationResult,
    validate_cartesian_target,
    validate_joint_target,
    validate_motion_scalars,
)


class SafeArmNode(Node):
    def __init__(self) -> None:
        super().__init__("safe_arm_node")
        self._declare_parameters()
        self.enabled = bool(self.get_parameter("enabled_on_start").value)
        self.latest_joints = None
        self.latest_tcp = None
        self.latest_arm_state = None
        self.joint_received_at = 0.0
        self.tcp_received_at = 0.0
        self.arm_state_received_at = 0.0
        self.last_decision = "startup"
        self.last_servoj_target = None
        self.servoj_accepted_count = 0

        self.state_timeout = float(self.get_parameter("state_timeout_sec").value)
        self.joint_min = list(self.get_parameter("joint_min_deg").value)
        self.joint_max = list(self.get_parameter("joint_max_deg").value)
        self.workspace_min = list(self.get_parameter("workspace_min_mm").value)
        self.workspace_max = list(self.get_parameter("workspace_max_mm").value)

        self.driver_movej = self.create_publisher(
            MoveCommand, self.get_parameter("driver_movej_topic").value, 10
        )
        self.driver_movel = self.create_publisher(
            MoveCommand, self.get_parameter("driver_movel_topic").value, 10
        )
        self.driver_servoj = self.create_publisher(
            Float64MultiArray, self.get_parameter("driver_servoj_topic").value, 10
        )
        self.status_publisher = self.create_publisher(String, "/tl_safe/status", 10)
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self._joint_callback,
            20,
        )
        self.create_subscription(
            CartesianPose,
            self.get_parameter("tcp_pose_topic").value,
            self._tcp_callback,
            20,
        )
        self.create_subscription(
            ArmStatus,
            self.get_parameter("arm_status_topic").value,
            self._arm_status_callback,
            20,
        )
        self.create_subscription(
            MoveCommand,
            self.get_parameter("safe_movej_topic").value,
            self._movej_callback,
            10,
        )
        self.create_subscription(
            MoveCommand,
            self.get_parameter("safe_movel_topic").value,
            self._movel_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("safe_servoj_topic").value,
            self._servoj_callback,
            20,
        )
        self.create_service(SetBool, "/tl_safe/set_enabled", self._set_enabled)
        self.create_service(Trigger, "/tl_safe/stop", self._stop)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().warning(
            "Safety proxy starts locked. It cannot stop an active driver motion; keep the physical E-stop available."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled_on_start", False)
        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("max_joint_step_deg", 20.0)
        self.declare_parameter("max_cartesian_step_mm", 80.0)
        self.declare_parameter("max_orientation_step_rad", 0.35)
        self.declare_parameter("max_velocity", 20.0)
        self.declare_parameter("max_acceleration", 20.0)
        self.declare_parameter("max_deceleration", 20.0)
        self.declare_parameter("max_servoj_step_deg", 1.0)
        self.declare_parameter("joint_min_deg", [-170.0] * 6)
        self.declare_parameter("joint_max_deg", [170.0] * 6)
        self.declare_parameter("workspace_min_mm", [-650.0, -650.0, 60.0])
        self.declare_parameter("workspace_max_mm", [650.0, 650.0, 850.0])
        self.declare_parameter("safe_movej_topic", "/tl_safe/moveJ")
        self.declare_parameter("safe_movel_topic", "/tl_safe/moveL")
        self.declare_parameter("safe_servoj_topic", "/tl_safe/servoj_pos")
        self.declare_parameter("driver_movej_topic", "/tl_driver/moveJ")
        self.declare_parameter("driver_movel_topic", "/tl_driver/moveL")
        self.declare_parameter("driver_servoj_topic", "/tl_driver/set_servoj_pos")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("tcp_pose_topic", "/tcp_pose")
        self.declare_parameter("arm_status_topic", "/arm_status")

    def _joint_callback(self, message: JointState) -> None:
        self.latest_joints = list(message.position)
        self.joint_received_at = time.monotonic()

    def _tcp_callback(self, message: CartesianPose) -> None:
        self.latest_tcp = [
            message.position.x,
            message.position.y,
            message.position.z,
            message.rpy.x,
            message.rpy.y,
            message.rpy.z,
        ]
        self.tcp_received_at = time.monotonic()

    def _arm_status_callback(self, message: ArmStatus) -> None:
        self.latest_arm_state = str(message.run_state).upper()
        self.arm_state_received_at = time.monotonic()

    def _set_enabled(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            if (
                not self._fresh(self.joint_received_at)
                or not self._fresh(self.tcp_received_at)
                or not self._fresh(self.arm_state_received_at)
            ):
                response.success = False
                response.message = "cannot unlock: robot state topics are stale"
                return response
            if self.latest_arm_state != "STOP":
                response.success = False
                response.message = f"cannot unlock while arm state is {self.latest_arm_state}"
                return response
            self.enabled = True
            self.last_servoj_target = None
            self.servoj_accepted_count = 0
            self.last_decision = "unlocked by service"
        else:
            self.enabled = False
            self.last_servoj_target = None
            self.last_decision = "locked by service"
        response.success = True
        response.message = self.last_decision
        self.get_logger().warning(response.message)
        return response

    def _stop(self, _request: Trigger.Request, response: Trigger.Response):
        self.enabled = False
        self.last_servoj_target = None
        self.last_decision = "future commands blocked by /tl_safe/stop"
        response.success = True
        response.message = (
            "Safety proxy locked. This does not stop a command already executing in the vendor driver; "
            "use the physical emergency stop for hazardous motion."
        )
        self.get_logger().error(response.message)
        return response

    def _movej_callback(self, message: MoveCommand) -> None:
        result = self._validate_common(message)
        if result.accepted and message.coord != 0:
            result = ValidationResult(False, "MoveJ must use joint coordinate coord=0")
        if result.accepted:
            if not self._fresh(self.joint_received_at) or self.latest_joints is None:
                result = ValidationResult(False, "joint state is stale")
            else:
                result = validate_joint_target(
                    message.target_pos_value,
                    self.latest_joints,
                    self.joint_min,
                    self.joint_max,
                    float(self.get_parameter("max_joint_step_deg").value),
                )
        self._forward_or_reject(result, message, self.driver_movej, "MoveJ")

    def _movel_callback(self, message: MoveCommand) -> None:
        result = self._validate_common(message)
        if result.accepted and message.coord != 1:
            result = ValidationResult(False, "MoveL must use Cartesian coordinate coord=1")
        if result.accepted:
            if not self._fresh(self.tcp_received_at) or self.latest_tcp is None:
                result = ValidationResult(False, "TCP pose is stale")
            else:
                result = validate_cartesian_target(
                    message.target_pos_value,
                    self.latest_tcp,
                    self.workspace_min,
                    self.workspace_max,
                    float(self.get_parameter("max_cartesian_step_mm").value),
                    float(self.get_parameter("max_orientation_step_rad").value),
                )
        self._forward_or_reject(result, message, self.driver_movel, "MoveL")

    def _servoj_callback(self, message: Float64MultiArray) -> None:
        if not self.enabled:
            result = ValidationResult(False, "safety proxy is locked")
        elif not self._fresh(self.joint_received_at) or self.latest_joints is None:
            result = ValidationResult(False, "joint state is stale")
        elif not self._fresh(self.arm_state_received_at):
            result = ValidationResult(False, "/arm_status is stale")
        elif self.latest_arm_state not in {"STOP", "RUNNING"}:
            result = ValidationResult(
                False, f"arm is {self.latest_arm_state}; ServoJ is not allowed"
            )
        elif len(message.data) < 6:
            result = ValidationResult(False, "ServoJ target must contain at least six values")
        else:
            reference = (
                self.latest_joints
                if self.last_servoj_target is None
                else self.last_servoj_target
            )
            result = validate_joint_target(
                message.data,
                reference,
                self.joint_min,
                self.joint_max,
                float(self.get_parameter("max_servoj_step_deg").value),
            )
        if result.accepted:
            self.driver_servoj.publish(message)
            self.last_servoj_target = list(message.data)
            self.servoj_accepted_count += 1
            self.last_decision = (
                f"accepted ServoJ point {self.servoj_accepted_count}: "
                f"{list(message.data[:6])}"
            )
            if self.servoj_accepted_count % 100 == 0:
                self.get_logger().info(self.last_decision)
        else:
            self.last_decision = f"rejected ServoJ: {result.reason}"
            self.get_logger().error(self.last_decision)
            self._publish_status()

    def _validate_common(self, message: MoveCommand) -> ValidationResult:
        if not self.enabled:
            return ValidationResult(False, "safety proxy is locked")
        if not self._fresh(self.arm_state_received_at):
            return ValidationResult(False, "/arm_status is stale")
        if self.latest_arm_state != "STOP":
            return ValidationResult(False, f"arm is {self.latest_arm_state}; wait for STOP")
        if len(message.target_pos_value) < 6:
            return ValidationResult(False, "target_pos_value must contain at least six values")
        return validate_motion_scalars(
            message.velocity,
            message.acc,
            message.dec,
            [
                self.get_parameter("max_velocity").value,
                self.get_parameter("max_acceleration").value,
                self.get_parameter("max_deceleration").value,
            ],
        )

    def _forward_or_reject(self, result, message, publisher, label: str) -> None:
        if result.accepted:
            publisher.publish(message)
            self.last_decision = f"accepted {label}: {list(message.target_pos_value[:6])}"
            self.get_logger().info(self.last_decision)
        else:
            self.last_decision = f"rejected {label}: {result.reason}"
            self.get_logger().error(self.last_decision)
        self._publish_status()

    def _fresh(self, received_at: float) -> bool:
        return received_at > 0.0 and time.monotonic() - received_at <= self.state_timeout

    def _publish_status(self) -> None:
        now = time.monotonic()
        payload = {
            "enabled": self.enabled,
            "joint_age_sec": None if not self.joint_received_at else now - self.joint_received_at,
            "tcp_age_sec": None if not self.tcp_received_at else now - self.tcp_received_at,
            "arm_state_age_sec": (
                None if not self.arm_state_received_at else now - self.arm_state_received_at
            ),
            "arm_run_state": self.latest_arm_state,
            "last_decision": self.last_decision,
            "servoj_accepted_count": self.servoj_accepted_count,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
