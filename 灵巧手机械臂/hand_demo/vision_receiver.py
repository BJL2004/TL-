#!/usr/bin/env python3
"""
Ubuntu 端: TCP 接收 Windows 视觉检测结果, 发布到 ROS2 话题

用法:
  cd ~/Desktop/hand_demo
  source ~/Desktop/tl_robot_ros2_py-x86/install/setup.bash
  python3 vision_receiver.py
"""

import json
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tl_ros2_interface.msg import ObjectInfo


class VisionReceiver(Node):
    """TCP 服务器: 接收检测结果, 发布 ObjectInfo"""

    def __init__(self, port=60001):
        super().__init__("vision_receiver")

        # TCP 服务器
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.listen(1)
        self.sock.settimeout(1.0)

        # ROS2 发布器
        self.pub = self.create_publisher(
            ObjectInfo,
            "/tl_vision/object_3d_pos_camera",
            10
        )

        self.running = True
        self.conn = None
        self.frame_count = 0

    def run(self):
        self.get_logger().info(f"等待 Windows 视觉连接 (端口 {self.port})...")

        while self.running and rclpy.ok():
            try:
                self.conn, addr = self.sock.accept()
                self.get_logger().info(f"✓ Windows 已连接: {addr[0]}")
                self._recv_loop()
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"连接异常: {e}")

    def _recv_loop(self):
        """接收循环"""
        while self.running and rclpy.ok():
            try:
                # 读 4 字节长度
                raw = self.conn.recv(4)
                if not raw:
                    self.get_logger().warn("Windows 断开连接")
                    break

                length = struct.unpack("!I", raw)[0]

                # 读 JSON 数据
                data = b""
                while len(data) < length:
                    chunk = self.conn.recv(length - len(data))
                    if not chunk:
                        break
                    data += chunk

                msg = json.loads(data.decode("utf-8"))
                self._publish(msg)

            except (ConnectionResetError, BrokenPipeError):
                self.get_logger().warn("连接断开")
                break
            except Exception as e:
                self.get_logger().error(f"接收错误: {e}")

    def _publish(self, msg):
        """发布 ObjectInfo 到 ROS2"""
        objects = msg.get("objects", [])
        self.frame_count += 1

        if self.frame_count % 30 == 0:
            self.get_logger().info(f"帧 {msg.get('frame', 0)}: 检测到 {len(objects)} 个物体")

        stamp = self.get_clock().now().to_msg()

        for obj in objects:
            xyz = obj.get("xyz", [0, 0, 0])
            oi = ObjectInfo()
            oi.type = obj.get("type", "unknown")
            oi.pos = PointStamped()
            oi.pos.header.stamp = stamp
            oi.pos.header.frame_id = "camera_color_optical_frame"
            oi.pos.point.x = xyz[0]
            oi.pos.point.y = xyz[1]
            oi.pos.point.z = xyz[2]
            self.pub.publish(oi)

    def stop(self):
        self.running = False
        if self.conn:
            self.conn.close()
        self.sock.close()


def main():
    rclpy.init()
    receiver = VisionReceiver(port=60001)

    # 后台 spin + 前台 TCP recv
    spin_thread = threading.Thread(target=rclpy.spin, args=(receiver,), daemon=True)
    spin_thread.start()

    try:
        receiver.run()
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        receiver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
