import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from tl_ros2_interface.msg import MoveCommand


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Publish one command through the TL safety proxy")
    parser.add_argument("kind", choices=["joint", "cartesian"])
    parser.add_argument("target", nargs=6, type=float)
    parser.add_argument("--velocity", type=float, default=20.0)
    parser.add_argument("--acceleration", type=float, default=20.0)
    parser.add_argument("--deceleration", type=float, default=20.0)
    parser.add_argument("--execute", action="store_true", help="required to publish")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    arguments = parse_args(argv)
    print(f"kind={arguments.kind} target={arguments.target}")
    if not arguments.execute:
        print("Dry run only. Re-run with --execute after checking the target and clearing the work area.")
        return

    rclpy.init(args=None)
    node = Node("tl_safe_move_cli")
    topic = "/tl_safe/moveJ" if arguments.kind == "joint" else "/tl_safe/moveL"
    publisher = node.create_publisher(MoveCommand, topic, 10)
    deadline = time.monotonic() + 3.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.get_logger().error(f"no safety proxy subscriber on {topic}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    command = MoveCommand()
    command.target_pos_value = list(arguments.target) + [0.0] * 8
    command.coord = 0 if arguments.kind == "joint" else 1
    command.velocity = arguments.velocity
    command.velocity_sync = 0.0
    command.acc = arguments.acceleration
    command.dec = arguments.deceleration
    command.target_pos_type = 0
    command.pl = 0
    command.time = 0
    command.tool_num = 0
    command.user_num = 0
    command.posidtype = 0
    command.configuration = 0
    command.spin = 0
    command.para_sync = False
    publisher.publish(command)
    rclpy.spin_once(node, timeout_sec=0.3)
    print(f"Published to {topic}; inspect /tl_safe/status for the validation decision.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
