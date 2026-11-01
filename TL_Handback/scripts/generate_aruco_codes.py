#!/usr/bin/env python3
"""Generate 192 ArUco markers for eight 24-position rack groups.

Layout:
  A1-01..A1-24 -> IDs 0..23
  A2-01..A2-24 -> IDs 24..47
  A3-01..A3-24 -> IDs 48..71
  A4-01..A4-24 -> IDs 72..95
  B1-01..B1-24 -> IDs 96..119
  B2-01..B2-24 -> IDs 120..143
  B3-01..B3-24 -> IDs 144..167
  B4-01..B4-24 -> IDs 168..191

Black square is calibrated to 58 mm at 300 DPI (marker_length_m=0.058).
Outputs:
  <out>/aruco_A1_01-06.pdf ... 32 A4 PDFs, six markers per page
  <out>/codes/ID000_N001_A1-01.png ... 192 per-code PNG files
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

MARKER_MM = 58.0
DPI = 300
BLACK_PX = round(MARKER_MM / 25.4 * DPI)  # 685 px = 58.0 mm @300 DPI
A4_WIDTH_PX = round(210 / 25.4 * DPI)
A4_HEIGHT_PX = round(297 / 25.4 * DPI)

GROUP_NAMES = ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4"]
POSITIONS_PER_GROUP = 24
MARKERS_PER_PDF = 6
TOTAL_MARKERS = len(GROUP_NAMES) * POSITIONS_PER_GROUP


def marker_name(number: int) -> str:
    group = (number - 1) // POSITIONS_PER_GROUP
    index = (number - 1) % POSITIONS_PER_GROUP + 1
    return f"{GROUP_NAMES[group]}-{index:02d}"


def make_marker(marker_id: int) -> np.ndarray:
    """Return the marker image with the black square exactly BLACK_PX px."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    probe = cv2.aruco.generateImageMarker(dictionary, marker_id, 800)
    # generateImageMarker returns a single-channel grayscale image.
    gray = np.asarray(probe, dtype=np.uint8) if probe.ndim == 2 else cv2.cvtColor(
        probe, cv2.COLOR_BGR2GRAY
    )
    dark = gray < 128
    rows = np.flatnonzero(dark.any(axis=1))
    cols = np.flatnonzero(dark.any(axis=0))
    top, bottom = int(rows[0]), int(rows[-1]) + 1
    left, right = int(cols[0]), int(cols[-1]) + 1
    square = gray[top:bottom, left:right]
    return cv2.resize(square, (BLACK_PX, BLACK_PX), interpolation=cv2.INTER_NEAREST)


def annotate(image: np.ndarray, lines: list[str], font_scale: float = 2.4) -> np.ndarray:
    """Stack annotation lines under the image (white background)."""
    line_h = 58
    total_h = image.shape[0] + line_h * len(lines) + 12
    canvas = np.full((total_h, image.shape[1], 3), 255, dtype=np.uint8)
    canvas[: image.shape[0]] = image
    y = image.shape[0] + 44
    for line in lines:
        cv2.putText(
            canvas,
            line,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        y += line_h
    return canvas


def make_a4_page(numbers: list[int]) -> Image.Image:
    """Create one portrait A4 page containing six labeled markers."""
    page = np.full((A4_HEIGHT_PX, A4_WIDTH_PX, 3), 255, dtype=np.uint8)
    margin_x = round(12 / 25.4 * DPI)
    margin_y = round(8 / 25.4 * DPI)
    gap_x = round(8 / 25.4 * DPI)
    gap_y = round(4 / 25.4 * DPI)
    cell_w = (A4_WIDTH_PX - 2 * margin_x - gap_x) // 2
    cell_h = (A4_HEIGHT_PX - 2 * margin_y - 2 * gap_y) // 3
    square_size = min(BLACK_PX, cell_w - round(18 / 25.4 * DPI))

    for index, number in enumerate(numbers):
        marker_id = number - 1
        name = marker_name(number)
        square = make_marker(marker_id)
        square = cv2.resize(square, (square_size, square_size), interpolation=cv2.INTER_NEAREST)
        square_bgr = cv2.cvtColor(square, cv2.COLOR_GRAY2BGR)
        row, col = divmod(index, 2)
        x_cell = margin_x + col * (cell_w + gap_x)
        y_cell = margin_y + row * (cell_h + gap_y)
        x0 = x_cell + (cell_w - square_size) // 2
        y0 = y_cell + round(8 / 25.4 * DPI)
        page[y0 : y0 + square_size, x0 : x0 + square_size] = square_bgr
        label_y = y0 + square_size + round(7 / 25.4 * DPI)
        cv2.putText(
            page,
            f"#{number:02d}  {name}  ArUco ID {marker_id}",
            (x_cell, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.35,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            page,
            "black square 58 mm",
            (x_cell, label_y + round(7 / 25.4 * DPI)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.95,
            (70, 70, 70),
            2,
            cv2.LINE_AA,
        )

    return Image.fromarray(cv2.cvtColor(page, cv2.COLOR_BGR2RGB))


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate 192 ArUco basket codes")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("~/Desktop/TL_Handback/aruco_codes_192").expanduser(),
        help="output directory",
    )
    args = parser.parse_args()

    out_dir = args.out
    codes_dir = out_dir / "codes"
    codes_dir.mkdir(parents=True, exist_ok=True)

    for number in range(1, TOTAL_MARKERS + 1):
        marker_id = number - 1
        name = marker_name(number)
        square = make_marker(marker_id)
        square_bgr = cv2.cvtColor(square, cv2.COLOR_GRAY2BGR)
        # Add a quiet zone around the black square.
        margin = 90
        bordered = np.full(
            (BLACK_PX + 2 * margin, BLACK_PX + 2 * margin, 3), 255, dtype=np.uint8
        )
        bordered[margin : margin + BLACK_PX, margin : margin + BLACK_PX] = square_bgr
        labeled = annotate(
            bordered,
            [f"#{number:02d}  {name}  (ArUco ID {marker_id})", f"58mm @300DPI"],
            font_scale=1.6,
        )
        per_code = Image.fromarray(cv2.cvtColor(labeled, cv2.COLOR_BGR2RGB))
        per_code.save(codes_dir / f"ID{marker_id:03d}_N{number:03d}_{name}.png")

    pdf_paths = []
    for group_index, group_name in enumerate(GROUP_NAMES):
        group_start = group_index * POSITIONS_PER_GROUP + 1
        for page_offset in range(0, POSITIONS_PER_GROUP, MARKERS_PER_PDF):
            start = group_start + page_offset
            end = start + MARKERS_PER_PDF - 1
            first_position = page_offset + 1
            last_position = first_position + MARKERS_PER_PDF - 1
            pdf_path = out_dir / (
                f"aruco_{group_name}_{first_position:02d}-{last_position:02d}.pdf"
            )
            page = make_a4_page(list(range(start, end + 1)))
            page.save(pdf_path, "PDF", resolution=DPI)
            pdf_paths.append(pdf_path)

    print(f"Generated {TOTAL_MARKERS} codes -> {codes_dir}")
    print(f"Generated {len(pdf_paths)} A4 PDFs -> {out_dir}")
    for pdf_path in pdf_paths:
        print(f"A4 PDF -> {pdf_path}")


if __name__ == "__main__":
    main()
