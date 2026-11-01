#!/usr/bin/env python3
"""Verify camera mounting rigidity from three stationary samples.

Usage:
  python3 verify_handeye_static.py <session_dir> [image_names...]

Collect exactly 3 samples WITHOUT moving the arm (same pose, press s three
times). This script then detects the chessboard corners in each saved PNG and
reports how much the corner pixel positions drift between samples.

- TCP drift: the arm should report identical TCP for all three samples.
- Corner drift > ~0.5 px (or non-monotonic jumps) while TCP is identical
  means the camera is moving relative to the flange (loose bracket, cable
  pull, flexible mount) -> tighten the mount and re-collect.
- Corner drift near zero with a fixed TCP is normal camera/board noise.
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np


def find_corners(image_path: Path):
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"cannot read {image_path}")
    pattern = (10, 7)  # inner corners, 15 mm squares
    found, corners = cv2.findChessboardCorners(
        image, pattern, None,
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK,
    )
    if not found:
        return None, image.shape
    return corners.reshape(-1, 2), image.shape


def main(argv=None) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    session_dir = Path(argv[1]).expanduser()
    names = argv[2:]
    if not names:
        names = [f"{value:04d}.png" for value in range(1, 26)]
    image_paths = [session_dir / name for name in names if (session_dir / name).exists()]
    if not image_paths:
        print(f"no images found under {session_dir}")
        return 2
    image_paths = image_paths[:3]

    # TCP records from the collection summary, if present.
    tcp_by_name = {}
    summary = session_dir / "collection_summary.json"
    if summary.exists():
        try:
            data = json.loads(summary.read_text(encoding="utf-8"))
            for record in data.get("samples", []):
                tcp_by_name[Path(record.get("image", "")).name] = record.get("tcp", [])
        except Exception as error:
            print(f"WARNING: could not parse summary: {error}")

    detections = []
    for image_path in image_paths:
        corners, shape = find_corners(image_path)
        detections.append((image_path.name, corners, shape))
        print(f"{image_path.name}: shape={shape}, corners={'found' if corners is not None else 'NOT FOUND'}")

    if any(corners is None for _, corners, _ in detections):
        print("ERROR: chessboard not detected in all samples; keep the full board visible")
        return 1

    names = [name for name, _, _ in detections]
    corner_sets = [corners for _, corners, _ in detections]

    print("\nTCP reported at save time:")
    for name in names:
        tcp = tcp_by_name.get(name)
        print(f"  {name}: {[round(v, 3) for v in tcp] if tcp else '<not in summary>'}")

    # Drift between sample 0 and each later sample.
    print("\nCorner pixel drift vs sample 0 (mean / max over all 70 corners):")
    reference = corner_sets[0]
    for name, corners in zip(names[1:], corner_sets[1:]):
        delta = np.linalg.norm(corners - reference, axis=1)
        print(f"  {name}: mean={delta.mean():.3f} px, max={delta.max():.3f} px")

    # Pairwise drift between consecutive samples.
    print("\nCorner pixel drift between consecutive samples (mean / max):")
    for left_name, right_name, left, right in zip(
        names[:-1], names[1:], corner_sets[:-1], corner_sets[1:]
    ):
        delta = np.linalg.norm(right - left, axis=1)
        print(f"  {left_name} -> {right_name}: mean={delta.mean():.3f} px, max={delta.max():.3f} px")

    print("\nInterpretation:")
    print("  - TCP identical + corner drift < 0.5 px : mount is rigid, good to collect")
    print("  - TCP identical + corner drift > 1 px   : camera moves vs flange -> TIGHTEN MOUNT")
    print("  - TCP differs between samples           : arm moved or joint drifts; redo the test")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
