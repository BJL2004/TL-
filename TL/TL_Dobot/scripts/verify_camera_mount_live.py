#!/usr/bin/env python3
"""Live camera-mount rigidity check.

Hold the arm perfectly still with the chessboard fully visible, then run this
script. It grabs ~30 frames and tracks the chessboard corner pixel positions.

If the camera is rigidly fixed to the flange:
  corner drift vs median stays < ~0.5 px (sensor noise only).
If the camera bracket / adapter / cable moves (loose mount):
  corner positions drift by > 1-2 px, often with slow oscillation.

Usage:
  python3 verify_camera_mount_live.py
"""

import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

PATTERN = (10, 7)
N_FRAMES = 30


def main() -> int:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    print("Keep the arm STILL with the chessboard fully visible...")
    time.sleep(1.5)

    corner_sets = []
    timestamps = []
    for frame_index in range(N_FRAMES):
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            continue
        image = np.asanyarray(color.get_data())
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray,
            PATTERN,
            None,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK,
        )
        if not found:
            print(f"frame {frame_index + 1}: board NOT detected (keep it fully visible)")
            continue
        corner_sets.append(corners.reshape(-1, 2))
        timestamps.append(time.monotonic())
        print(f"frame {frame_index + 1}: 70 corners OK")
        time.sleep(0.1)

    pipeline.stop()

    if len(corner_sets) < 10:
        print(f"ERROR: only {len(corner_sets)} valid frames; board must stay visible")
        return 1

    stack = np.asarray(corner_sets)  # (n, 70, 2)
    median_corners = np.median(stack, axis=0)
    drifts = np.linalg.norm(stack - median_corners, axis=2)  # (n, 70)
    per_frame_mean = drifts.mean(axis=1)
    per_frame_max = drifts.max(axis=1)

    print("\n=== Corner drift vs median position (px) ===")
    print(f"frames used : {len(corner_sets)}")
    print(f"mean drift  : {per_frame_mean.mean():.3f} px (max frame mean {per_frame_mean.max():.3f})")
    print(f"max drift   : {per_frame_max.max():.3f} px across all frames/corners")
    print(f"drift spread: {per_frame_max.max() - per_frame_max.min():.3f} px")

    worst = float(per_frame_max.max())
    print("\n=== Verdict ===")
    if worst < 0.8:
        print("MOUNT IS RIGID: corner drift is sensor noise only. Proceed to collect a")
        print("full dataset for calibration. If calibration still fails, the residual")
        print("comes from the arm itself (joint play) or board quality, not the camera mount.")
    elif worst < 2.0:
        print("BORDERLINE: some drift (up to %.1f px). Tighten the camera bracket and" % worst)
        print("re-check; also verify cables do not pull on the camera during motion.")
    else:
        print(f"MOUNT IS LOOSE: corners drift up to {worst:.1f} px while the arm is still.")
        print("Tighten every camera bracket / adapter screw, route cables so they do")
        print("not pull the camera, then re-run this check and re-collect.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
