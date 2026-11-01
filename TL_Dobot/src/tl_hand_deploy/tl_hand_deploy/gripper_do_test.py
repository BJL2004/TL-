import argparse
from datetime import datetime, timezone
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
import yaml
from tl_ros2_interface.srv import SetDigitalOutput


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Identify gripper open/close wiring on two TL digital outputs"
    )
    parser.add_argument("--do-a", type=int, default=3)
    parser.add_argument("--do-b", type=int, default=4)
    parser.add_argument(
        "--initial-high",
        type=int,
        default=3,
        choices=(3, 4),
        help="启动和退出时保持为高电平的输出，另一路保持低电平",
    )
    parser.add_argument("--pulse-sec", type=float, default=0.5)
    parser.add_argument("--dead-time-sec", type=float, default=0.2)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("~/tl_gripper_io/gripper_do_map.yaml"),
    )
    parser.add_argument("--execute", action="store_true")
    arguments = parser.parse_args(argv)
    if arguments.do_a <= 0 or arguments.do_b <= 0:
        parser.error("digital output port numbers must be positive")
    if arguments.do_a == arguments.do_b:
        parser.error("--do-a and --do-b must be different ports")
    if arguments.pulse_sec < 0.1 or arguments.pulse_sec > 3.0:
        parser.error("--pulse-sec must be in 0.1..3.0 seconds")
    if arguments.dead_time_sec < 0.1 or arguments.dead_time_sec > 1.0:
        parser.error("--dead-time-sec must be in 0.1..1.0 seconds")
    return arguments


class GripperDoTester(Node):
    def __init__(self, arguments):
        super().__init__("tl_gripper_do_test")
        self.arguments = arguments
        self.ports = (int(arguments.do_a), int(arguments.do_b))
        if arguments.initial_high not in self.ports:
            raise ValueError("--initial-high must be one of the configured outputs")
        self.initial_high_port = int(arguments.initial_high)
        self.current_high_port: int | None = None
        self.output_path = arguments.output.expanduser().resolve()
        self.client = self.create_client(
            SetDigitalOutput, "/tl_driver/set_digital_output"
        )
        self.driver_ready = False
        self.observations: dict[int, str] = {}

    def wait_for_driver(self) -> None:
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "/tl_driver/set_digital_output is unavailable; start the TL driver first"
            )
        self.driver_ready = True

    def set_output(self, port: int, value: int) -> None:
        if not self.arguments.execute:
            print(f"DRY RUN: DO{port}={value}")
            return
        request = SetDigitalOutput.Request()
        request.port = int(port)
        request.value = int(value)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=12.0)
        response = future.result() if future.done() else None
        if response is None:
            raise RuntimeError(f"DO{port}={value} timed out")
        if not response.success:
            raise RuntimeError(
                f"DO{port}={value} failed: {response.message or 'driver rejected request'}"
            )
        print(f"DO{port}={value}: {response.message}")

    def set_complementary_state(self, high_port: int, force: bool = False) -> None:
        if high_port not in self.ports:
            raise RuntimeError(f"DO{high_port} is not configured for this test")
        if self.current_high_port == high_port and not force:
            print(f"Gripper state already complementary: DO{high_port}=1")
            return
        low_port = self.ports[1] if high_port == self.ports[0] else self.ports[0]
        self.set_output(low_port, 0)
        self.set_output(high_port, 1)
        self.current_high_port = high_port
        print(f"Complementary state: DO{high_port}=1, DO{low_port}=0")

    def restore_safe_state(self, best_effort: bool = False) -> None:
        try:
            self.set_complementary_state(self.initial_high_port)
        except RuntimeError as error:
            if not best_effort:
                raise
            print(f"WARNING: failed to restore complementary safe state: {error}")

    def pulse(self, port: int) -> None:
        if port not in self.ports:
            raise RuntimeError(f"DO{port} is not configured for this test")
        print(f"Switching to complementary test state: DO{port}=1")
        try:
            self.set_complementary_state(port)
            time.sleep(float(self.arguments.pulse_sec))
        finally:
            self.restore_safe_state()
        print(f"DO{port} test finished; complementary safe state restored")

    def record_observation(self, port: int) -> None:
        answer = input(
            f"Observe the gripper for DO{port}: [o]pen / [c]lose / [n]o action: "
        ).strip().lower()
        labels = {"o": "open", "c": "close", "n": "no_action"}
        if answer not in labels:
            print("Observation not recorded")
            return
        self.observations[port] = labels[answer]
        print(f"Recorded DO{port} -> {labels[answer]}")

    def save_mapping(self) -> None:
        open_ports = [
            port for port, action in self.observations.items() if action == "open"
        ]
        close_ports = [
            port for port, action in self.observations.items() if action == "close"
        ]
        if len(open_ports) != 1 or len(close_ports) != 1:
            raise RuntimeError(
                "test both channels and record exactly one open and one close action"
            )
        data = {
            "format_version": 1,
            "updated_utc": datetime.now(timezone.utc).isoformat(),
            "service": "/tl_driver/set_digital_output",
            "active_value": 1,
            "inactive_value": 0,
            "initial_high_port": self.initial_high_port,
            "pulse_sec": float(self.arguments.pulse_sec),
            "dead_time_sec": float(self.arguments.dead_time_sec),
            "open_port": int(open_ports[0]),
            "close_port": int(close_ports[0]),
            "safe_state": {
                f"DO{port}": int(port == self.initial_high_port) for port in self.ports
            },
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
        print(f"Saved gripper mapping: {self.output_path}")
        print(f"OPEN=DO{open_ports[0]}, CLOSE=DO{close_ports[0]}")

    def run(self) -> None:
        self.wait_for_driver()
        print("TL gripper DO3/DO4 identification test")
        print("This program sends NO robot motion commands.")
        print(
            f"Configured outputs: DO{self.ports[0]} and DO{self.ports[1]}, "
            f"pulse={self.arguments.pulse_sec:.2f}s"
        )
        low_port = self.ports[1] if self.initial_high_port == self.ports[0] else self.ports[0]
        print(
            f"Initial/exit state: DO{self.initial_high_port}=1, DO{low_port}=0; "
            "outputs are never intentionally left both HIGH or both LOW"
        )
        if not self.arguments.execute:
            print("DRY RUN only. Restart with --execute to energize outputs.")
        else:
            confirmation = input(
                "Remove objects from the gripper, lower air pressure, keep E-stop ready, "
                "then type TEST: "
            ).strip()
            if confirmation != "TEST":
                print("Confirmation failed; no output was energized")
                return
        self.set_complementary_state(self.initial_high_port, force=True)
        print(
            f"Commands: {self.ports[0]} pulse DO{self.ports[0]} | "
            f"{self.ports[1]} pulse DO{self.ports[1]} | 0 restore initial state | "
            "s save mapping | q quit"
        )
        while rclpy.ok():
            command = input("gripper-do> ").strip().lower()
            if command in ("q", "quit", "exit"):
                break
            if command == "0":
                self.restore_safe_state()
                continue
            if command == "s":
                try:
                    self.save_mapping()
                except RuntimeError as error:
                    print(f"Cannot save: {error}")
                continue
            if command in (str(self.ports[0]), str(self.ports[1])):
                port = int(command)
                try:
                    self.pulse(port)
                    self.record_observation(port)
                except RuntimeError as error:
                    print(f"Output test stopped: {error}")
                    self.restore_safe_state(best_effort=True)
                continue
            print("Unknown command")


def main(argv=None):
    arguments = parse_args(argv)
    rclpy.init()
    node = GripperDoTester(arguments)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.driver_ready:
            node.restore_safe_state(best_effort=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
