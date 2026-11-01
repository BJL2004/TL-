#!/usr/bin/env python3
"""
Windows 端: D435 采集 + YOLO 检测 + TCP 发送到 Ubuntu

运行环境: Windows Anaconda Python 3.13 (有 pyrealsense2 + ultralytics)
发送目标: Ubuntu 的 IP + 端口

用法:
  python vision_bridge.py --host 192.168.1.xxx --port 60001
"""

import argparse
import json
import socket
import struct
import time
import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="192.168.1.100", help="Ubuntu IP")
    p.add_argument("--port", type=int, default=60001, help="TCP port")
    p.add_argument("--model", default="yolov8n.pt", help="YOLO model path")
    p.add_argument("--conf", type=float, default=0.5, help="confidence threshold")
    p.add_argument("--fx", type=float, default=610.0)
    p.add_argument("--fy", type=float, default=610.0)
    p.add_argument("--cx", type=float, default=320.0)
    p.add_argument("--cy", type=float, default=240.0)
    p.add_argument("--show", action="store_true", help="show OpenCV window")
    return p.parse_args()


def pixel_to_3d(u, v, depth_m, fx, fy, cx, cy):
    if depth_m <= 0:
        return None
    return (
        (u - cx) * depth_m / fx,
        (v - cy) * depth_m / fy,
        depth_m,
    )


def main():
    args = parse_args()

    # ── 连接 Ubuntu ──
    print(f"连接 {args.host}:{args.port} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))
    print("✓ 已连接")

    # ── 加载 YOLO ──
    print(f"加载模型 {args.model} ...")
    model = YOLO(args.model)
    print("✓ YOLO 就绪")

    # ── 启动 D435 ──
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipe.start(cfg)

    # 对齐深度到彩色
    align = rs.align(rs.stream.color)

    # 获取内参
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    print(f"相机内参: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

    print("\n开始检测，按 Ctrl+C 退出\n")

    frame_id = 0
    try:
        while True:
            frames = pipe.wait_for_frames()
            aligned = align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            depth_m = depth.astype(np.float32) / 1000.0

            frame_id += 1

            # YOLO 检测
            results = model(color, conf=args.conf, verbose=False)

            detections = []
            if len(results) > 0 and results[0].boxes is not None:
                for box in results[0].boxes:
                    cls = int(box.cls[0])
                    name = model.names[cls]
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)

                    d = depth_m[v, u] if 0 <= v < 480 and 0 <= u < 640 else 0
                    xyz = pixel_to_3d(u, v, d, fx, fy, cx, cy)

                    if xyz and d > 0.05:
                        detections.append({
                            "type": name,
                            "conf": round(conf, 3),
                            "pixel": [u, v],
                            "xyz": [round(v, 4) for v in xyz],
                        })

            if detections:
                # 发送 JSON
                msg = json.dumps({"frame": frame_id, "objects": detections})
                data = msg.encode("utf-8")
                sock.sendall(struct.pack("!I", len(data)) + data)

                for d in detections:
                    print(f"  [{d['type']}] conf={d['conf']:.2f} "
                          f"xyz=({d['xyz'][0]:.3f}, {d['xyz'][1]:.3f}, {d['xyz'][2]:.3f})m")

            # 显示
            if args.show:
                for d in detections:
                    u, v = d["pixel"]
                    cv2.circle(color, (u, v), 5, (0, 255, 0), -1)
                    cv2.putText(color, f"{d['type']} {d['conf']:.2f}",
                                (u - 30, v - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.imshow("Vision Bridge", color)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        sock.close()
        cv2.destroyAllWindows()
        print("已停止")


if __name__ == "__main__":
    main()
