#!/usr/bin/env python3
"""Generate 24 ArUco markers (IDs 0..23) for the 4-row x 6-basket rack.

Layout (rows top to bottom, 6 codes per row):
  Row 1: A1-01..A1-06  -> numbers 1..6   (IDs 0..5)
  Row 2: A2-01..A2-06  -> numbers 7..12  (IDs 6..11)
  Row 3: B1-01..B1-06  -> numbers 13..18 (IDs 12..17)
  Row 4: B2-01..B2-06  -> numbers 19..24 (IDs 18..23)

Black square is calibrated to 56 mm at 300 DPI (marker_length_m=0.056).
Outputs:
  <out>/aruco_24_codes.png    overview grid 4 rows x 6 columns
  <out>/aruco_24_codes.pdf    same grid as a printable PDF page
  <out>/aruco_<row>_6_codes.pdf  one A4 PDF per rack row, 6 codes per page
  <out>/codes/ID00_number01_A1-01.png ... per-code PNG files
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

MARKER_MM = 56.0
DPI = 300
BLACK_PX = round(MARKER_MM / 25.4 * DPI)  # 661 px = 56.0 mm @300 DPI
A4_WIDTH_PX = round(210 / 25.4 * DPI)
A4_HEIGHT_PX = round(297 / 25.4 * DPI)

ROW_NAMES = ["A1", "A2", "B1", "B2"]


def marker_name(number: int) -> str:
    row = (number - 1) // 6
    index = (number - 1) % 6 + 1
    return f"{ROW_NAMES[row]}-{index:02d}"


def make_marker(marker_id: int) -> np.ndarray:
    """Return the marker image with the black square exactly BLACK_PX px."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
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


def make_a4_row_page(numbers: list[int]) -> Image.Image:
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
            "black square 56 mm",
            (x_cell, label_y + round(7 / 25.4 * DPI)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.95,
            (70, 70, 70),
            2,
            cv2.LINE_AA,
        )

    return Image.fromarray(cv2.cvtColor(page, cv2.COLOR_BGR2RGB))


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate 24 ArUco basket codes")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("~/Desktop/TL_Handback/aruco_codes").expanduser(),
        help="output directory",
    )
    args = parser.parse_args()

    out_dir = args.out
    codes_dir = out_dir / "codes"
    codes_dir.mkdir(parents=True, exist_ok=True)

    cell_h = BLACK_PX + 170  # black square + annotation strip
    cell_w = BLACK_PX + 90
    rows, cols = 4, 6
    grid = np.full((cell_h * rows, cell_w * cols, 3), 255, dtype=np.uint8)

    for number in range(1, 25):
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
            [f"#{number:02d}  {name}  (ArUco ID {marker_id})", f"56mm @300DPI"],
            font_scale=1.6,
        )
        per_code = Image.fromarray(cv2.cvtColor(labeled, cv2.COLOR_BGR2RGB))
        per_code.save(codes_dir / f"ID{marker_id:02d}_N{number:02d}_{name}.png")

        # Place the unlabeled square into the overview grid.
        row, col = (number - 1) // 6, (number - 1) % 6
        y0 = row * cell_h + 60
        x0 = col * cell_w + 45
        grid[y0 : y0 + BLACK_PX, x0 : x0 + BLACK_PX] = square_bgr
        cv2.putText(
            grid,
            f"#{number:02d} {name}",
            (x0, y0 + BLACK_PX + 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.7,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )

    overview_path = out_dir / "aruco_24_codes.png"
    overview = Image.fromarray(cv2.cvtColor(grid, cv2.COLOR_BGR2RGB))
    overview.save(overview_path)
    pdf_path = out_dir / "aruco_24_codes.pdf"
    overview.save(pdf_path, "PDF", resolution=DPI)
    row_pdfs = []
    for row_name, start in zip(ROW_NAMES, (1, 7, 13, 19)):
        row_path = out_dir / f"aruco_{row_name}_6_codes.pdf"
        page = make_a4_row_page(list(range(start, start + 6)))
        page.save(row_path, "PDF", resolution=DPI)
        row_pdfs.append(row_path)
    print(f"Generated 24 codes -> {codes_dir}")
    print(f"Overview grid -> {overview_path}")
    print(f"PDF -> {pdf_path}")
    for row_path in row_pdfs:
        print(f"A4 row PDF -> {row_path}")


if __name__ == "__main__":
    main()
