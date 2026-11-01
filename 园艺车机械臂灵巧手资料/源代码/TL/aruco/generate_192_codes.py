#!/usr/bin/env python3
"""生成 192 个 ArUco 码（DICT_4X4_250, ID 0..191），每 6 个码一个 A4 PDF。

布局（8 排 × 24 = 192 花盆）：
  A1-01..A1-24 -> 花盆 1..24   (ID 0..23)
  A2-01..A2-24 -> 花盆 25..48  (ID 24..47)
  A3-01..A3-24 -> 花盆 49..72  (ID 48..71)
  A4-01..A4-24 -> 花盆 73..96  (ID 72..95)
  B1-01..B1-24 -> 花盆 97..120 (ID 96..119)
  B2-01..B2-24 -> 花盆 121..144(ID 120..143)
  B3-01..B3-24 -> 花盆 145..168(ID 144..167)
  B4-01..B4-24 -> 花盆 169..192(ID 168..191)

黑方块边长 54 mm @300 DPI（marker_length_m=0.054）。
输出：TL/aruco/aruco_XXX-YYY.pdf（每个 6 个码，共 32 个）
"""

from pathlib import Path

import cv2
import numpy as np
from PIL import Image

MARKER_MM = 54.0
DPI = 300
BLACK_PX = round(MARKER_MM / 25.4 * DPI)  # 638 px = 54.0 mm @300 DPI
A4_WIDTH_PX = round(210 / 25.4 * DPI)
A4_HEIGHT_PX = round(297 / 25.4 * DPI)

ROW_NAMES = ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4"]
CODES_PER_ROW = 24


def marker_name(number: int) -> str:
    row = (number - 1) // CODES_PER_ROW
    index = (number - 1) % CODES_PER_ROW + 1
    return f"{ROW_NAMES[row]}-{index:02d}"


def make_marker(marker_id: int) -> np.ndarray:
    """返回黑方块边长正好 BLACK_PX 的码图（灰度）。"""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    probe = cv2.aruco.generateImageMarker(dictionary, marker_id, 800)
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


def make_a4_page(numbers: list[int]) -> Image.Image:
    """一张竖版 A4：2 列 × 3 行，6 个带标注的码。"""
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
            f"#{number:03d}  {name}  ArUco ID {marker_id}",
            (x_cell, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.15,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            page,
            "black square 54 mm",
            (x_cell, label_y + round(7 / 25.4 * DPI)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (70, 70, 70),
            2,
            cv2.LINE_AA,
        )
    return Image.fromarray(cv2.cvtColor(page, cv2.COLOR_BGR2RGB))


def main() -> None:
    out_dir = Path(__file__).resolve().parent
    pdfs = []
    for start in range(1, 193, 6):
        numbers = list(range(start, min(start + 6, 193)))
        page = make_a4_page(numbers)
        pdf_path = out_dir / f"aruco_{start:03d}-{numbers[-1]:03d}.pdf"
        page.save(pdf_path, "PDF", resolution=DPI)
        pdfs.append(pdf_path)
        print(f"生成: {pdf_path.name}  <- {', '.join(marker_name(n) for n in numbers)}")
    print(f"\n完成: {len(pdfs)} 个 PDF（每个 6 码），共 {192} 个码，目录: {out_dir}")


if __name__ == "__main__":
    main()
