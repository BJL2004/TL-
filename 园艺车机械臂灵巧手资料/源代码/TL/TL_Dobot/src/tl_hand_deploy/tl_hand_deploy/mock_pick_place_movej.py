import time

import numpy as np
import rclpy
from std_srvs.srv import Trigger
from tl_ros2_interface.msg import ArmStatus, MoveCommand
from tl_ros2_interface.srv import GetCurrentMode, SetCurrentMode

from .mock_pick_place import MockPickPlaceNode, parse_args
from .overhead_test import load_calibration
from .reference_compat import (
    format_joint_errors,
    joint_error_degrees,
    vendor_movej_target,
)


class MoveJMockPickPlaceNode(MockPickPlaceNode):
    def __init__(self, arguments, calibration_path, calibration, tool_camera):
        super().__init__(arguments, calibration_path, calibration, tool_camera)
        self.movej_topic = "/tl_driver/moveJ"
        self.movej_publisher = self.create_publisher(
            MoveCommand, self.movej_topic, 10
        )
        self.arm_run_state = None
        self.arm_status_at = 0.0
        self.create_subscription(ArmStatus, "/arm_status", self._arm_status_callback, 20)
        self.set_mode_client = self.create_client(
            SetCurrentMode, "/tl_driver/set_current_mode"
        )
        self.get_mode_client = self.create_client(
            GetCurrentMode, "/tl_driver/get_current_mode"
        )
        self.previous_controller_mode = None
        self.movej_mode_prepared = False

    def _arm_status_callback(self, message: ArmStatus) -> None:
        self.arm_run_state = str(message.run_state).strip().upper()
        self.arm_status_at = time.monotonic()

    def _wait_for_movej_subscriber(self, timeout_sec: float = 3.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.movej_publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            "/tl_driver/moveJ has no subscriber; start or restart the TL driver"
        )

    def _get_controller_mode(self) -> int:
        if not self.get_mode_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/tl_driver/get_current_mode is unavailable")
        response = self._call(
            self.get_mode_client, GetCurrentMode.Request(), timeout_sec=3.0
        )
        if not response.success:
            raise RuntimeError(f"GetCurrentMode failed: {response.message}")
        return int(response.mode)

    def _set_controller_mode(self, mode: int) -> None:
        if not self.set_mode_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/tl_driver/set_current_mode is unavailable")
        request = SetCurrentMode.Request()
        request.mode = int(mode)
        response = self._call(self.set_mode_client, request, timeout_sec=5.0)
        if not response.success:
            raise RuntimeError(
                f"SetCurrentMode({mode}) failed: {response.message}"
            )

    def _close_any_servoj_session(self) -> None:
        if not self.close_servoj_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(
                "/tl_driver/close_servoj is unavailable; cannot make MoveJ exclusive"
            )
        response = self._call(
            self.close_servoj_client,
            Trigger.Request(),
            timeout_sec=5.0,
        )
        if not response.success:
            print(
                "CloseServoJ did not report success before MoveJ: "
                f"{response.message}. Continuing only if run mode can be confirmed."
            )
        self.servoj_open = False

    def _prepare_movej_mode(self, force: bool = False) -> None:
        current_mode = self._get_controller_mode()
        if self.previous_controller_mode is None:
            self.previous_controller_mode = current_mode
        if self.movej_mode_prepared and current_mode == 2 and not force:
            return

        self._close_any_servoj_session()
        if current_mode != 2:
            print(
                f"Controller mode={current_mode}; switching to mode=2 (run) "
                "for native MoveJ."
            )
            self._set_controller_mode(2)
        confirmed_mode = self._get_controller_mode()
        if confirmed_mode != 2:
            raise RuntimeError(
                "MoveJ cannot start because controller mode confirmation failed: "
                f"expected 2 (run), got {confirmed_mode}"
            )
        self.movej_mode_prepared = True
        print("MoveJ controller handshake OK: mode=2 (run), ServoJ released.")

    def _restore_controller_mode(self) -> None:
        if self.previous_controller_mode is None or not rclpy.ok():
            return
        restore_mode = int(self.previous_controller_mode)
        try:
            current_mode = self._get_controller_mode()
            if current_mode != restore_mode:
                self._set_controller_mode(restore_mode)
                confirmed_mode = self._get_controller_mode()
                if confirmed_mode != restore_mode:
                    raise RuntimeError(
                        f"expected mode {restore_mode}, got {confirmed_mode}"
                    )
            print(f"Controller mode restored to {restore_mode}.")
        finally:
            self.movej_mode_prepared = False

    def _build_movej_message(self, target_joint) -> MoveCommand:
        message = MoveCommand()
        message.target_pos_value = vendor_movej_target(target_joint)
        message.target_pos_name = ""
        message.target_pos_type = 0
        message.coord = 0
        message.velocity = float(self.arguments.movej_velocity)
        message.velocity_sync = 0.0
        message.acc = float(self.arguments.movej_acceleration)
        message.dec = float(self.arguments.movej_deceleration)
        message.pl = 0
        message.time = 0
        message.tool_num = 0
        message.user_num = 0
        message.posidtype = 0
        message.configuration = 0
        message.spin = 0
        message.para_sync = False
        return message

    def move_joint(self, target_joint, point_count=None, speed_deg_s=None) -> None:
        del point_count
        del speed_deg_s
        if not self.arguments.execute:
            print("DRY RUN: no MoveJ command sent. Add --execute for physical motion.")
            return
        self._validate_joint(target_joint)
        self._prepare_movej_mode()
        self._wait_for_movej_subscriber()
        target = np.asarray(target_joint[:6], dtype=float)
        current = np.asarray(self.latest_joint[:6], dtype=float)
        initial_errors = joint_error_degrees(current, target)
        initial_error = float(np.max(np.abs(initial_errors)))
        if initial_error <= float(self.arguments.arrival_tolerance_deg):
            print(f"MoveJ target already reached; max joint error={initial_error:.3f} deg")
            return

        message = self._build_movej_message(target_joint)
        print(
            "Publishing controller-planned MoveJ: "
            f"velocity={message.velocity:.1f}, acc={message.acc:.1f}, "
            f"dec={message.dec:.1f}, target={np.round(target, 3).tolist()}"
        )
        started_at = time.monotonic()
        deadline = started_at + float(self.arguments.timeout_sec)
        best_error = initial_error
        settled_samples = 0
        latest_error = initial_error
        latest_errors = initial_errors
        retry_count = 0
        published_at = started_at
        last_progress_log_at = 0.0
        last_motion_progress_at = started_at
        progress_reference_error = initial_error
        saw_running_after_publish = False
        self.movej_publisher.publish(message)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_joint is None:
                continue
            latest = np.asarray(self.latest_joint[:6], dtype=float)
            latest_errors = joint_error_degrees(latest, target)
            latest_error = float(np.max(np.abs(latest_errors)))
            best_error = min(best_error, latest_error)
            now = time.monotonic()
            state = self.arm_run_state or "UNKNOWN"
            if latest_error <= progress_reference_error - 0.05:
                progress_reference_error = latest_error
                last_motion_progress_at = now
            if state == "RUNNING":
                saw_running_after_publish = True
            if now - last_progress_log_at >= float(self.arguments.movej_progress_log_sec):
                print(
                    f"MoveJ progress: state={state}, elapsed={now - started_at:.1f}s, "
                    f"max error={latest_error:.3f} deg, "
                    f"errors={format_joint_errors(latest_errors)}"
                )
                last_progress_log_at = now
            if latest_error <= float(self.arguments.arrival_tolerance_deg):
                settled_samples += 1
                if settled_samples >= 3:
                    print(
                        f"MoveJ arrived: max joint error={latest_error:.3f} deg, "
                        f"errors={format_joint_errors(latest_errors)}"
                    )
                    return
            else:
                settled_samples = 0

            status_is_fresh = now - self.arm_status_at <= 1.0
            if status_is_fresh and state == "PAUSE":
                raise RuntimeError(
                    "MoveJ controller entered PAUSE; no retry was sent. "
                    f"errors={format_joint_errors(latest_errors)}"
                )
            controller_stopped = status_is_fresh and state == "STOP"
            grace_elapsed = now - published_at >= float(
                self.arguments.movej_status_grace_sec
            )
            motion_stalled = now - last_motion_progress_at >= float(
                self.arguments.movej_stall_sec
            )
            if controller_stopped and grace_elapsed and motion_stalled:
                retry_limit = int(self.arguments.movej_max_retries)
                retry_error_limit = float(self.arguments.movej_retry_max_error_deg)
                if latest_error > retry_error_limit:
                    if not saw_running_after_publish and now - published_at < 5.0:
                        continue
                    raise RuntimeError(
                        "MoveJ controller stopped too far from target; refusing retry: "
                        f"max error={latest_error:.3f} deg, "
                        f"errors={format_joint_errors(latest_errors)}"
                    )
                if retry_count >= retry_limit:
                    raise RuntimeError(
                        "MoveJ controller stopped with residual joint error after all "
                        f"retries: max error={latest_error:.3f} deg, "
                        f"errors={format_joint_errors(latest_errors)}"
                    )
                retry_count += 1
                print(
                    f"MoveJ controller is STOP with {latest_error:.3f} deg residual; "
                    f"retrying the same target ({retry_count}/{retry_limit}). "
                    f"errors={format_joint_errors(latest_errors)}"
                )
                self.movej_publisher.publish(message)
                published_at = now
                last_motion_progress_at = now
                progress_reference_error = latest_error
                saw_running_after_publish = False

        raise RuntimeError(
            "MoveJ did not reach the target before timeout: "
            f"initial error={initial_error:.3f} deg, "
            f"best error={best_error:.3f} deg, latest error={latest_error:.3f} deg, "
            f"state={self.arm_run_state or 'UNKNOWN'}, "
            f"errors={format_joint_errors(latest_errors)}"
        )

    def run(self) -> None:
        print(
            "Motion backend=MoveJ topic; all startup, M, R and basket-cycle joint "
            "moves are planned inside the TL controller."
        )
        print(
            f"MoveJ parameters: velocity={self.arguments.movej_velocity:.1f}, "
            f"acc={self.arguments.movej_acceleration:.1f}, "
            f"dec={self.arguments.movej_deceleration:.1f}, "
            f"retries={self.arguments.movej_max_retries}, "
            f"timeout={self.arguments.timeout_sec:.1f}s"
        )
        self._prepare_movej_mode(force=True)
        super().run()

    def stop(self) -> None:
        try:
            super().stop()
        finally:
            try:
                self._restore_controller_mode()
            except Exception as error:
                if rclpy.ok():
                    self.get_logger().error(
                        f"Failed to restore controller mode: {error}"
                    )


def main(argv=None):
    arguments = parse_args(argv)
    calibration_path, calibration, tool_camera = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    rclpy.init()
    node = MoveJMockPickPlaceNode(
        arguments, calibration_path, calibration, tool_camera
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
