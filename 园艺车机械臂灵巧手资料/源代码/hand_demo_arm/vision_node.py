#!/usr/bin/env python3
"""
D435 + YOLOv8 检测节点 (ARM版)
订阅: D435 彩色/深度帧
发布: /tl_vision/object_3d_pos_camera (ObjectInfo)

运行: python3 vision_node.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tl_ros2_interface.msg import ObjectInfo
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import threading


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # 参数
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('fx', 615.0)  # D435 内参
        self.declare_parameter('fy', 615.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        self.conf = self.get_parameter('conf_threshold').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        device = self.get_parameter('device').value

        # 加载模型
        model_path = self.get_parameter('model_path').value
        self.log(f"加载 {model_path} ...")
        self.model = YOLO(model_path)
        self.log("YOLO 模型就绪")

        # 帧缓存
        self.lock = threading.Lock()
        self.color_frame = None
        self.depth_frame = None
        self.bridge = CvBridge()

        # 发布器
        self.pub = self.create_publisher(ObjectInfo, '/tl_vision/object_3d_pos_camera', 10)
        self.debug_pub = self.create_publisher(Image, '/tl_vision/detection_debug', 10)

        # 从 D435 话题订阅
        self.create_subscription(Image, '/camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, 10)

        # 检测定时器 30Hz
        self.create_timer(1.0/30.0, self.detect_cb)
        self.log("视觉节点就绪")

    def log(self, msg):
        self.get_logger().info(str(msg))

    def color_cb(self, msg):
        with self.lock:
            self.color_frame = msg

    def depth_cb(self, msg):
        with self.lock:
            self.depth_frame = msg

    def detect_cb(self):
        with self.lock:
            color = self.color_frame
            depth = self.depth_frame
        if color is None or depth is None:
            return

        try:
            color_img = self.bridge.imgmsg_to_cv2(color, 'bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth, '16UC1')
        except:
            return

        # YOLO 推理
        results = self.model(color_img, conf=self.conf, device=0, verbose=False)

        stamp = color.header.stamp
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                # 取中心点深度
                cx_px = (x1 + x2) // 2
                cy_px = (y1 + y2) // 2
                cx_px = max(0, min(cx_px, depth_img.shape[1]-1))
                cy_px = max(0, min(cy_px, depth_img.shape[0]-1))

                # 深度中位数滤波
                rgn = depth_img[max(0,cy_px-5):min(depth_img.shape[0],cy_px+5),
                               max(0,cx_px-5):min(depth_img.shape[1],cx_px+5)]
                z_mm = np.median(rgn[rgn > 0]) if np.any(rgn > 0) else 0
                if z_mm == 0:
                    continue

                # 反投影到相机坐标系 (米)
                z = z_mm / 1000.0
                x = (cx_px - self.cx) * z / self.fx
                y = (cy_px - self.cy) * z / self.fy

                # 发布
                oi = ObjectInfo()
                oi.type = cls_name
                oi.pos = PointStamped()
                oi.pos.header.stamp = stamp
                oi.pos.header.frame_id = 'camera_color_optical_frame'
                oi.pos.point.x = x
                oi.pos.point.y = y
                oi.pos.point.z = z
                self.pub.publish(oi)

                # 画框
                cv2.rectangle(color_img, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(color_img, f"{cls_name} {z:.2f}m", (x1, y1-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # 发布调试图
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(color_img, 'bgr8'))


def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
