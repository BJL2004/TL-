#!/usr/bin/env python3
"""
D435i 深度相机 + YOLO 实时目标检测
- 左窗: 彩色画面 + 检测框 + FPS
- 右窗: 深度热力图
- 按 q 退出
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import time
from ultralytics import YOLO

# ── 1. 加载模型 ──
model = YOLO("/home/nvidia/Desktop/nvidia_test/yolov8n.pt")
print("YOLO 模型加载完成")

# ── 2. 启动 D435i ──
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)
print("D435i 启动完成, 按 q 退出")

# 对齐深度到彩色
align = rs.align(rs.stream.color)

try:
    while True:
        t0 = time.time()

        frames = pipe.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        # ── 3. YOLO 推理 ──
        results = model(color, verbose=False)

        # 画检测框
        annotated = results[0].plot()

        # 标注物体距离 (深度)
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            dist = depth_frame.get_distance(cx, cy)
            cls_name = model.names[int(box.cls[0])]
            text = f"{cls_name} {dist:.2f}m"
            cv2.putText(annotated, text, (x1, max(y1-10, 10)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # 深度热力图
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)

        # FPS
        fps = 1.0 / (time.time() - t0)
        cv2.putText(annotated, f"FPS:{fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        cv2.imshow("D435i + YOLO", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()
    print("退出")
