import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tl_ros2_interface.msg import CartesianPose


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Check TCB605, D435, and optional ROH prerequisites")
    parser.add_argument("--with-hand", action="store_true")
    parser.add_argument("--skip-camera", action="store_true")
    parser.add_argument("--timeout", type=float, default=5.0)
    return parser.parse_args(argv)


class CheckNode(Node):
    def __init__(self):
        super().__init__("tl_system_check")
        self.joints = None
        self.tcp = None
        self.create_subscription(JointState, "/joint_states", self._joint, 10)
        self.create_subscription(CartesianPose, "/tcp_pose", self._tcp, 10)

    def _joint(self, message):
        self.joints = list(message.position)

    def _tcp(self, message):
        self.tcp = [
            message.position.x,
            message.position.y,
            message.position.z,
            message.rpy.x,
            message.rpy.y,
            message.rpy.z,
        ]


def _camera_check(timeout: float):
    try:
        import pyrealsense2 as rs

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = pipeline.start(config)
        frame = pipeline.wait_for_frames(timeout_ms=int(timeout * 1000)).get_color_frame()
        serial = profile.get_device().get_info(rs.camera_info.serial_number)
        pipeline.stop()
        return bool(frame), f"D435 serial={serial}, color={'OK' if frame else 'NO FRAME'}"
    except Exception as error:
        return False, f"D435 failed: {error}"


def main(argv=None) -> None:
    arguments = parse_args(argv)
    rclpy.init(args=None)
    node = CheckNode()
    deadline = time.monotonic() + arguments.timeout
    while time.monotonic() < deadline and (node.joints is None or node.tcp is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    services = {name for name, _types in node.get_service_names_and_types()}
    required = set()
    if arguments.with_hand:
        required.update({"/tl_driver/modbus_read", "/tl_driver/modbus_write"})
    missing = sorted(required - services)
    checks = [
        (node.joints is not None and len(node.joints) >= 6, f"/joint_states: {node.joints}"),
        (node.tcp is not None and len(node.tcp) >= 6, f"/tcp_pose: {node.tcp}"),
        (not missing, f"required services missing: {missing}" if missing else "required services: OK"),
    ]
    if not arguments.skip_camera:
        checks.append(_camera_check(arguments.timeout))
    print("\n系统检查结果")
    for passed, message in checks:
        print(f"[{'OK' if passed else 'FAIL'}] {message}")
    node.destroy_node()
    rclpy.shutdown()
    if not all(passed for passed, _message in checks):
        raise SystemExit(2)
