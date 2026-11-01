import argparse
from datetime import datetime, timezone
from pathlib import Path
import time

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tl_ros2_interface.msg import CartesianPose


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Record current TL robot pose as a mock-hand basket place pose"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("~/tl_mock_pick_place/place_pose.yaml"),
    )
    parser.add_argument(
        "--name",
        choices=["basket_approach", "basket_release", "observation"],
        required=True,
        help="basket_approach is the safe pose above the basket; "
        "basket_release is the lower pose where the hand opens",
    )
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    return parser.parse_args(argv)


class PlacePoseRecorder(Node):
    def __init__(self):
        super().__init__("tl_mock_place_teach")
        self.latest_tcp = None
        self.latest_joint = None
        self.create_subscription(CartesianPose, "/tcp_pose", self._tcp_callback, 20)
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 20)

    def _tcp_callback(self, message: CartesianPose) -> None:
        self.latest_tcp = [
            float(message.position.x),
            float(message.position.y),
            float(message.position.z),
            float(message.rpy.x),
            float(message.rpy.y),
            float(message.rpy.z),
        ]

    def _joint_callback(self, message: JointState) -> None:
        values = [float(value) for value in message.position[:7]]
        self.latest_joint = (values + [0.0] * 7)[:7]

    def wait_current_pose(self, timeout_sec: float) -> tuple[list[float], list[float]]:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_tcp is not None and self.latest_joint is not None:
                return list(self.latest_tcp), list(self.latest_joint)
        raise RuntimeError("missing /tcp_pose or /joint_states; start the TL driver first")


def main(argv=None):
    arguments = parse_args(argv)
    rclpy.init()
    node = PlacePoseRecorder()
    try:
        tcp, joint = node.wait_current_pose(arguments.timeout_sec)
        output_path = arguments.output.expanduser().resolve()
        if output_path.exists():
            data = yaml.safe_load(output_path.read_text(encoding="utf-8")) or {}
        else:
            data = {}
        data.setdefault("poses", {})
        data["updated_utc"] = datetime.now(timezone.utc).isoformat()
        data["poses"][arguments.name] = {
            "tcp": tcp,
            "joint": joint,
            "recorded_utc": datetime.now(timezone.utc).isoformat(),
            "source": "/tcp_pose and /joint_states",
        }
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        print(f"saved {arguments.name}:")
        print(f"  tcp={tcp}")
        print(f"  joint={joint[:6]}")
        print(f"  file={output_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
