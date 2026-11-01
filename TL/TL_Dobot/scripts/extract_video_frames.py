#!/usr/bin/env python3
"""按帧提取视频图像（用于 YOLO 数据采集/标注）。

用法:
  python3 scripts/extract_video_frames.py --video <视频路径> --output <输出目录>
  # 每 5 帧取 1 帧（减少相似帧，适合训练）：
  python3 scripts/extract_video_frames.py --video <视频> --output <目录> --step 5
"""

import argparse
from pathlib import Path

import cv2


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Extract frames from a video")
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--step",
        type=int,
        default=1,
        help="keep every Nth frame (1 = all frames)",
    )
    parser.add_argument("--max-frames", type=int, default=0, help="0 = no limit")
    return parser.parse_args(argv)


def main(argv=None):
    arguments = parse_args(argv)
    if arguments.step < 1:
        raise SystemExit("--step must be >= 1")
    output_dir = arguments.output.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(arguments.video.expanduser().resolve()))
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {arguments.video}")
    saved = 0
    frame_index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_index % arguments.step == 0:
            if arguments.max_frames and saved >= arguments.max_frames:
                break
            name = f"frame_{saved:06d}.jpg"
            cv2.imwrite(str(output_dir / name), frame)
            saved += 1
        frame_index += 1
    cap.release()
    print(f"video={arguments.video}")
    print(f"saved={saved} frames (step={arguments.step}) -> {output_dir}")


if __name__ == "__main__":
    main()
