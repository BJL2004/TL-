"""MQTT-driven U-fork trim cycle: trim_pick (grab -> trim position) / trim_place (return).

Protocol (车 <-> 机械臂):
  down  node/arm/01/down  车->机械臂  {"cmd":"trim_pick","targetId":"A1-01"}
  up    node/arm/01/up    机械臂->车  {"cmd":"trim_pick","targetId":"A1-01","retCode":1}
  down  node/arm/01/down  车->机械臂  {"cmd":"trim_place","targetId":"A1-01"}
  up    node/arm/01/up    机械臂->车  {"cmd":"trim_place","targetId":"A1-01","retCode":0}
  retCode: 0=success, 1=acknowledged/action started, 2=position occupied,
           3=recognition/placement failed
  targetId: rack naming A1-01..A4-24 / B1-01..B4-24 (192 baskets);
            legacy plain 1..192 is also accepted.
  Each accepted command gets two feedbacks: retCode=1 immediately (action started),
  then the final retCode (0/2/3) when the action finishes.

The trim position is the taught fixed temporary hold (T) for now.
"""

import rclpy

from .fork_basket_cycle import ForkBasketCycleNode, parse_args, load_calibration
from .fork_basket_teach import apply_reversed_rows
from tl_hand.mqtt_protocol import (
    MqttClient,
    RET_SUCCESS,
    RET_OCCUPIED,
    RET_STARTED,
    RET_RECOG_FAILED,
    target_id_to_basket_number,
)


def resolve_fork_basket(target_id) -> tuple[int | None, str | None]:
    """Map the car-side targetId to a fork basket number 1..192.

    Accepts rack naming 'A1-01'..'A4-24' / 'B1-01'..'B4-24' and the legacy
    plain '1'..'192'. Returns (basket, None) on success and
    (None, error_message) otherwise.
    """
    text = str(target_id).strip()
    if text.isdigit():
        value = int(text)
        if 1 <= value <= 192:
            return value, None
        return None, f"basket number {value} out of range 1..192"
    rack_number = target_id_to_basket_number(text)
    if rack_number is None:
        return (
            None,
            f"invalid targetId={target_id!r}; "
            "expected A1-01..A4-24 or B1-01..B4-24",
        )
    return rack_number, None


class ForkTrimMqttNode(ForkBasketCycleNode):
    def __init__(self, arguments, calibration_path, calibration, tool_camera):
        super().__init__(arguments, calibration_path, calibration, tool_camera)
        self.mqtt_client = MqttClient(
            host=arguments.mqtt_host,
            port=arguments.mqtt_port,
            command_topic=arguments.mqtt_command_topic,
            feedback_topic=arguments.mqtt_feedback_topic,
            client_id=arguments.mqtt_client_id,
            username=arguments.mqtt_username,
            password=arguments.mqtt_password,
            keepalive=arguments.mqtt_keepalive,
        )

    def run(self):
        if not self.arguments.execute:
            raise RuntimeError("start with --execute; it must send robot motion")
        self._wait_feedback()
        self._start_camera()
        self.mqtt_client.start()
        print("U-fork trim MQTT node: trim_pick -> hold at trim position, "
              "trim_place -> return to the original position.")
        if self.arguments.reversed_rows:
            print(
                "Reversed-row code mapping active: "
                f"{list(self.arguments.reversed_rows)} "
                "(target positions are mirrored to the actual code numbers)."
            )
        print(f"Command topic: {self.mqtt_client.command_topic}")
        print(f"Feedback topic: {self.mqtt_client.feedback_topic}")
        # 启动时先回到观测位。叉上还挂着花篮（活动状态存在）时不自动动：
        # 自动回位会沿未示教的路径拖拽花篮，必须先 trim_place 放回。
        if self.active_transfer is not None:
            print(
                "Startup: a basket is still held (active fork state); the arm "
                "will NOT auto-move. Send trim_place to return the basket first."
            )
        else:
            try:
                data = self._load_output()
                if data:
                    print("Startup: returning to the observation (standby) pose...")
                    self._move_to_observation(data)
            except RuntimeError as error:
                print(f"WARNING: startup observation move failed: {error}")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            for command in self.mqtt_client.drain_commands():
                self._handle_command(command)

    def _handle_command(self, command):
        if command.name == "estop":
            print("MQTT ESTOP: acknowledged; the arm motion is aborted/stopped")
            self.mqtt_client.publish_feedback("estop", RET_SUCCESS)
            return
        if self.mqtt_client.estop_requested:
            print(
                f"MQTT {command.name} rejected: emergency stop is active; "
                "restart the node to clear it"
            )
            self.mqtt_client.publish_feedback(
                command.name,
                RET_RECOG_FAILED,
                "emergency stop is active; restart the node",
                target_id=command.target_id,
            )
            return
        if command.name == "trim_pick":
            self._handle_trim_pick(command)
        elif command.name == "trim_place":
            self._handle_trim_place(command)
        else:
            print(f"Unhandled MQTT command: {command.name}")

    def _handle_trim_pick(self, command):
        if self.active_transfer is not None:
            print("trim_pick rejected: a basket is already held at the trim position")
            self.mqtt_client.publish_feedback(
                "trim_pick",
                RET_OCCUPIED,
                "a basket is already held; send trim_place first",
                target_id=command.target_id,
            )
            return
        basket, error_message = resolve_fork_basket(command.target_id)
        if basket is not None:
            effective = apply_reversed_rows(basket, self.arguments.reversed_rows)
            if effective != basket:
                print(
                    f"Reversed-row mapping: targetId={command.target_id} "
                    f"(number {basket}) -> actual code number {effective}."
                )
                basket = effective
        if basket is None:
            self.mqtt_client.publish_feedback(
                "trim_pick", RET_RECOG_FAILED, error_message,
                target_id=command.target_id,
            )
            return
        self.mqtt_client.publish_feedback(
            "trim_pick", RET_STARTED, target_id=command.target_id
        )
        self.requested_marker_id = basket - 1
        print(f"trim_pick: grabbing basket number={basket} (targetId={command.target_id})")
        try:
            self._pick_to_temporary()
        except (RuntimeError, ValueError) as error:
            print(f"trim_pick failed: {error}")
            self.mqtt_client.publish_feedback(
                "trim_pick", RET_RECOG_FAILED, str(error),
                target_id=command.target_id,
            )
            if self.active_transfer is None:
                # 参考种子流程：还没抬起花篮就失败（如一直检测不到码）时，
                # 先回待机位再等下一条命令，不要停在检索位/观测位挡着。
                try:
                    data = self._load_output()
                    if data:
                        print(
                            "trim_pick failed before the lift; returning to the "
                            "observation (standby) pose..."
                        )
                        try:
                            self._move_to_observation(data)
                        finally:
                            self._close_servoj()
                except RuntimeError as move_error:
                    print(
                        "WARNING: return-to-observation after trim_pick failure "
                        f"failed: {move_error}"
                    )
            return
        print("trim_pick: basket is held at the trim position")
        self.mqtt_client.publish_feedback(
            "trim_pick", RET_SUCCESS, target_id=command.target_id
        )

    def _handle_trim_place(self, command):
        if self.active_transfer is None:
            print("trim_place rejected: no basket is held")
            self.mqtt_client.publish_feedback(
                "trim_place", RET_RECOG_FAILED,
                "no basket is held; send trim_pick first",
                target_id=command.target_id,
            )
            return
        self.mqtt_client.publish_feedback(
            "trim_place", RET_STARTED, target_id=command.target_id
        )
        print("trim_place: returning the basket to its original position")
        try:
            self._return_to_original()
        except (RuntimeError, ValueError) as error:
            print(f"trim_place failed: {error}")
            self.mqtt_client.publish_feedback(
                "trim_place", RET_RECOG_FAILED, str(error),
                target_id=command.target_id,
            )
            return
        print("trim_place: basket returned")
        self.mqtt_client.publish_feedback(
            "trim_place", RET_SUCCESS, target_id=command.target_id
        )

    def stop(self):
        if getattr(self, "mqtt_client", None) is not None:
            self.mqtt_client.stop()
        super().stop()


def main(argv=None):
    arguments = parse_args(argv)
    calibration_path, calibration, tool_camera = load_calibration(
        arguments.calibration, arguments.allow_rejected
    )
    print(
        f"calibration={calibration_path}\n"
        f"accepted={bool(calibration.get('accepted', False))}, "
        f"convention={calibration.get('rpy_convention', 'intrinsic_xyz')}"
    )
    rclpy.init(args=[])
    node = ForkTrimMqttNode(arguments, calibration_path, calibration, tool_camera)
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
