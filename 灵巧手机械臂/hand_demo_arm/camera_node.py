#!/usr/bin/env python3
"""
D435 相机节点 (pyrealsense2 直驱, 不依赖 realsense2_camera)
发布: /camera/color/image_raw, /camera/depth/image_rect_raw, /camera/camera_info

运行: python3 camera_node.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from cv_bridge import CvBridge
import pyrealsense2 as rs


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()

        # 打开发布器
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        # 打开 D435
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)

        # 获取内参
        color_intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx = color_intr.fx
        self.fy = color_intr.fy
        self.cx = color_intr.ppx
        self.cy = color_intr.ppy
        self.log(f"D435 内参: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}")

        # 发布 CameraInfo
        self.color_info = self._make_camera_info(color_intr, 'camera_color_optical_frame')
        self.depth_info = self._make_camera_info(color_intr, 'camera_depth_optical_frame')

        # 定时器 30Hz
        self.create_timer(1.0/30.0, self.publish_cb)
        self.log("相机节点就绪")

    def log(self, msg):
        self.get_logger().info(str(msg))

    def _make_camera_info(self, intr, frame_id):
        ci = CameraInfo()
        ci.header.frame_id = frame_id
        ci.height = intr.height
        ci.width = intr.width
        ci.k = [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0]
        ci.p = [intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def publish_cb(self):
        try:
            frames = self.pipe.wait_for_frames(timeout_ms=30)
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                return

            stamp = self.get_clock().now().to_msg()

            # 彩色
            color_img = np.asanyarray(color_frame.get_data())
            color_msg = self.bridge.cv2_to_imgmsg(color_img, 'bgr8')
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = 'camera_color_optical_frame'
            self.color_pub.publish(color_msg)

            # 深度
            depth_img = np.asanyarray(depth_frame.get_data())
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, '16UC1')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            self.depth_pub.publish(depth_msg)

            # CameraInfo
            self.color_info.header.stamp = stamp
            self.depth_info.header.stamp = stamp
            self.color_info_pub.publish(self.color_info)
            self.depth_info_pub.publish(self.depth_info)

        except Exception as e:
            self.log(f"帧错误: {e}")

    def destroy_node(self):
        self.pipe.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
