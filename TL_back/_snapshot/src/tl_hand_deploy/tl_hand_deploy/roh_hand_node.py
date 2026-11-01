import json
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam
from tl_ros2_interface.srv import ModbusRead, ModbusWrite

from .roh_protocol import (
    Register,
    classify_operation,
    release_toward_open,
    validate_register_values,
)


class RohHandNode(Node):
    def __init__(self) -> None:
        super().__init__("roh_hand_node")
        self._declare_parameters()
        self.callback_group = ReentrantCallbackGroup()
        self.io_lock = threading.Lock()
        self.enabled = bool(self.get_parameter("enabled_on_start").value)
        self.limits_verified = bool(self.get_parameter("limits_verified").value)
        if self.enabled and not self.limits_verified:
            self.enabled = False
        self.operation = "idle"
        self.operation_deadline = 0.0
        self.communication_faults = 0
        self.latest = {}

        self.read_client = self.create_client(
            ModbusRead,
            "/tl_driver/modbus_read",
            callback_group=self.callback_group,
        )
        self.write_client = self.create_client(
            ModbusWrite,
            "/tl_driver/modbus_write",
            callback_group=self.callback_group,
        )
        self.status_publisher = self.create_publisher(String, "/tl_hand/status", 10)
        self.create_service(
            SetBool,
            "/tl_hand/set_enabled",
            self._set_enabled,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger, "/tl_hand/open", self._open, callback_group=self.callback_group
        )
        self.create_service(
            Trigger, "/tl_hand/grasp", self._grasp, callback_group=self.callback_group
        )
        self.create_service(
            Trigger, "/tl_hand/stop", self._stop, callback_group=self.callback_group
        )
        self.create_timer(
            float(self.get_parameter("poll_period_sec").value),
            self._poll,
            callback_group=self.callback_group,
        )
        self.get_logger().warning(
            f"ROH guard ready: enabled={self.enabled}, limits_verified={self.limits_verified}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled_on_start", False)
        self.declare_parameter("limits_verified", False)
        self.declare_parameter("apply_protection_profile", True)
        self.declare_parameter("service_timeout_sec", 2.0)
        self.declare_parameter("poll_period_sec", 0.25)
        self.declare_parameter("motion_timeout_sec", 8.0)
        self.declare_parameter("communication_fault_limit", 3)
        self.declare_parameter("master_id", 1)
        self.declare_parameter("slave_id", 2)
        self.declare_parameter("rtu_port", 2)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("data_bit", 8)
        self.declare_parameter("stop_bit", 1)
        self.declare_parameter("check_bit", "N")
        self.declare_parameter("open_position", [0] * 6)
        self.declare_parameter("grasp_position", [32000] * 5 + [22000])
        self.declare_parameter("speed", [8000] * 5 + [6000])
        self.declare_parameter("current_limit_ma", [350] * 5 + [300])
        self.declare_parameter("force_limit_mn", [2000] * 5)
        self.declare_parameter("stall_speed", [16] * 6)
        self.declare_parameter("stall_current_ma", [200] * 5 + [180])
        self.declare_parameter("stop_after_period_ms", [300] * 6)
        self.declare_parameter("retry_after_period_ms", [1000] * 6)
        self.declare_parameter("fault_release_ratio", 0.15)
        self.declare_parameter("active_contact_fingers", [0, 1, 2, 3, 4])

    def _master_parameter(self) -> ModbusMasterParam:
        rtu = ModbusRTUParam()
        rtu.slave_id = int(self.get_parameter("slave_id").value)
        rtu.port = int(self.get_parameter("rtu_port").value)
        rtu.baudrate = int(self.get_parameter("baudrate").value)
        rtu.data_bit = int(self.get_parameter("data_bit").value)
        rtu.stop_bit = int(self.get_parameter("stop_bit").value)
        rtu.check_bit = str(self.get_parameter("check_bit").value)
        parameter = ModbusMasterParam()
        parameter.type = "RTU"
        parameter.start_addr = True
        parameter.rtu = rtu
        return parameter

    def _wait_result(self, future):
        complete = threading.Event()
        future.add_done_callback(lambda _future: complete.set())
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not complete.wait(timeout):
            future.cancel()
            return None
        try:
            return future.result()
        except Exception as error:
            self.get_logger().error(f"Modbus service exception: {error}")
            return None

    def _read(self, address: int, quantity: int):
        if not self.read_client.wait_for_service(timeout_sec=0.2):
            return None
        request = ModbusRead.Request()
        request.master_id = int(self.get_parameter("master_id").value)
        request.addr = int(address)
        request.quantity = int(quantity)
        request.master_param = self._master_parameter()
        response = self._wait_result(self.read_client.call_async(request))
        if response is None or not response.success or len(response.data) != quantity:
            return None
        return [int(value) for value in response.data]

    def _write(self, address: int, values):
        if not self.write_client.wait_for_service(timeout_sec=0.2):
            return False
        request = ModbusWrite.Request()
        request.master_id = int(self.get_parameter("master_id").value)
        request.addr = int(address)
        request.data = [int(value) for value in values]
        request.master_param = self._master_parameter()
        response = self._wait_result(self.write_client.call_async(request))
        return bool(response is not None and response.success)

    def _set_enabled(self, request: SetBool.Request, response: SetBool.Response):
        if request.data and not self.limits_verified:
            response.success = False
            response.message = "set limits_verified=true only after bench verification"
            return response
        self.enabled = bool(request.data)
        if not self.enabled:
            self.operation = "idle"
        response.success = True
        response.message = f"hand command enabled={self.enabled}"
        self.get_logger().warning(response.message)
        return response

    def _open(self, _request: Trigger.Request, response: Trigger.Response):
        return self._start_operation("open", response)

    def _grasp(self, _request: Trigger.Request, response: Trigger.Response):
        return self._start_operation("grasp", response)

    def _start_operation(self, operation: str, response: Trigger.Response):
        if not self.enabled or not self.limits_verified:
            response.success = False
            response.message = "hand guard is locked or limits are not verified"
            return response
        with self.io_lock:
            try:
                if bool(self.get_parameter("apply_protection_profile").value):
                    self._apply_profile()
                parameter = "open_position" if operation == "open" else "grasp_position"
                target = validate_register_values(self.get_parameter(parameter).value, 6, parameter)
                if not self._write(Register.POSITION_TARGET, target):
                    raise RuntimeError("failed to write position target")
                self.operation = operation
                self.operation_deadline = time.monotonic() + float(
                    self.get_parameter("motion_timeout_sec").value
                )
            except (RuntimeError, ValueError) as error:
                self.enabled = False
                response.success = False
                response.message = str(error)
                return response
        response.success = True
        response.message = f"{operation} command accepted; monitor /tl_hand/status"
        self.get_logger().warning(response.message)
        return response

    def _apply_profile(self) -> None:
        profile = [
            (Register.SPEED, "speed", 6),
            (Register.CURRENT_LIMIT, "current_limit_ma", 6),
            (Register.FORCE_LIMIT, "force_limit_mn", 5),
            (Register.STALL_SPEED, "stall_speed", 6),
            (Register.STALL_CURRENT, "stall_current_ma", 6),
            (Register.STOP_AFTER_PERIOD, "stop_after_period_ms", 6),
            (Register.RETRY_AFTER_PERIOD, "retry_after_period_ms", 6),
        ]
        for register, parameter, count in profile:
            values = validate_register_values(self.get_parameter(parameter).value, count, parameter)
            if not self._write(register, values):
                raise RuntimeError(f"failed to write protection parameter {parameter}")

    def _stop(self, _request: Trigger.Request, response: Trigger.Response):
        with self.io_lock:
            positions = self._read(Register.POSITION, 6)
            if positions is not None:
                self._write(Register.POSITION_TARGET, positions)
                target = release_toward_open(
                    positions,
                    self.get_parameter("open_position").value,
                    float(self.get_parameter("fault_release_ratio").value),
                )
                self._write(Register.POSITION_TARGET, target)
        self.operation = "idle"
        self.enabled = False
        response.success = positions is not None
        response.message = "hand frozen, slightly released, and command guard locked"
        self.get_logger().error(response.message)
        return response

    def _poll(self) -> None:
        if not self.io_lock.acquire(blocking=False):
            return
        try:
            firmware = self.latest.get("firmware")
            if firmware is None:
                values = self._read(Register.FW_VERSION, 1)
                if values:
                    firmware = values[0]
            statuses = self._read(Register.STATUS, 6)
            positions = self._read(Register.POSITION, 6)
            currents = self._read(Register.CURRENT, 6)
            forces = self._read(Register.FORCE, 5)
            if any(value is None for value in [statuses, positions, currents, forces]):
                self._communication_failure()
                return
            self.communication_faults = 0
            self.latest = {
                "firmware": firmware,
                "enabled": self.enabled,
                "operation": self.operation,
                "status": statuses,
                "position": positions,
                "current_ma": currents,
                "force_mn": forces,
            }
            if self.operation in {"open", "grasp"}:
                outcome = classify_operation(
                    statuses,
                    self.operation,
                    self.get_parameter("active_contact_fingers").value,
                )
                self.latest["outcome"] = outcome
                if outcome in {"fault", "fault_unknown_status"}:
                    self._fault_release(positions, outcome)
                elif outcome in {"open_reached", "contact"}:
                    self.operation = "idle"
                    self.latest["operation"] = "idle"
                elif outcome == "no_contact":
                    self.enabled = False
                    self.operation = "idle"
                    self.latest["enabled"] = False
                    self.latest["operation"] = "idle"
                    self.get_logger().warning("grasp ended without force contact; guard locked")
                elif time.monotonic() > self.operation_deadline:
                    self._fault_release(positions, "timeout")
            self._publish_latest()
        finally:
            self.io_lock.release()

    def _fault_release(self, positions, reason: str) -> None:
        self._write(Register.POSITION_TARGET, positions)
        target = release_toward_open(
            positions,
            self.get_parameter("open_position").value,
            float(self.get_parameter("fault_release_ratio").value),
        )
        self._write(Register.POSITION_TARGET, target)
        self.operation = "idle"
        self.enabled = False
        self.latest["operation"] = "idle"
        self.latest["enabled"] = False
        self.latest["fault"] = reason
        self.get_logger().error(f"hand fault {reason}: released and locked")

    def _communication_failure(self) -> None:
        self.communication_faults += 1
        if self.communication_faults >= int(self.get_parameter("communication_fault_limit").value):
            self.enabled = False
            self.operation = "idle"
        self.latest = {
            "enabled": self.enabled,
            "operation": self.operation,
            "communication_faults": self.communication_faults,
        }
        self._publish_latest()

    def _publish_latest(self) -> None:
        message = String()
        message.data = json.dumps(self.latest, ensure_ascii=False)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RohHandNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
