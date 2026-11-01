#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


DEFAULT_OUTPUT = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "calibration"
    / "handeye_validation_3x3_cross_60mm_A4.pdf"
)


def draw_arrow(pdf: canvas.Canvas, start: tuple[float, float], end: tuple[float, float]) -> None:
    start_x, start_y = start
    end_x, end_y = end
    pdf.line(start_x, start_y, end_x, end_y)
    if abs(end_x - start_x) > abs(end_y - start_y):
        direction = 1 if end_x > start_x else -1
        pdf.line(end_x, end_y, end_x - direction * 3 * mm, end_y + 1.5 * mm)
        pdf.line(end_x, end_y, end_x - direction * 3 * mm, end_y - 1.5 * mm)
    else:
        direction = 1 if end_y > start_y else -1
        pdf.line(end_x, end_y, end_x + 1.5 * mm, end_y - direction * 3 * mm)
        pdf.line(end_x, end_y, end_x - 1.5 * mm, end_y - direction * 3 * mm)


def draw_target(pdf: canvas.Canvas, x_mm: float, y_mm: float, label: str) -> None:
    x = x_mm * mm
    y = y_mm * mm
    arm = 10 * mm

    pdf.setLineWidth(0.45 * mm)
    pdf.line(x - arm, y, x + arm, y)
    pdf.line(x, y - arm, x, y + arm)
    pdf.setLineWidth(0.25 * mm)
    pdf.circle(x, y, 3 * mm, stroke=1, fill=0)
    pdf.setFillColorRGB(0, 0, 0)
    pdf.circle(x, y, 0.8 * mm, stroke=0, fill=1)
    pdf.setFont("Helvetica-Bold", 8)
    pdf.drawCentredString(x, y - 15 * mm, label)


def generate(output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(output), pagesize=A4, pageCompression=1)
    width, height = A4
    pdf.setTitle("Hand-eye validation target - 3x3 cross 60 mm")
    pdf.setAuthor("TL_Dobot")

    pdf.setLineWidth(0.2 * mm)
    pdf.rect(10 * mm, 10 * mm, width - 20 * mm, height - 20 * mm)

    pdf.setFont("Helvetica-Bold", 15)
    pdf.drawCentredString(width / 2, 281 * mm, "HAND-EYE VALIDATION TARGET - 3 x 3")
    pdf.setFont("Helvetica-Bold", 10)
    pdf.drawCentredString(width / 2, 273 * mm, "PRINT AT 100% / ACTUAL SIZE - DO NOT FIT TO PAGE")
    pdf.setFont("Helvetica", 9)
    pdf.drawCentredString(width / 2, 266 * mm, "Target spacing: 60 mm | Coverage: 120 x 120 mm")

    x_positions = (45.0, 105.0, 165.0)
    y_positions = (220.0, 160.0, 100.0)

    pdf.saveState()
    pdf.setDash(2 * mm, 2 * mm)
    pdf.setStrokeColorRGB(0.7, 0.7, 0.7)
    pdf.setLineWidth(0.15 * mm)
    for x_mm in x_positions:
        pdf.line(x_mm * mm, y_positions[-1] * mm, x_mm * mm, y_positions[0] * mm)
    for y_mm in y_positions:
        pdf.line(x_positions[0] * mm, y_mm * mm, x_positions[-1] * mm, y_mm * mm)
    pdf.restoreState()

    labels = (
        "P1 (-60,+60)",
        "P2 (0,+60)",
        "P3 (+60,+60)",
        "P4 (-60,0)",
        "P5 (0,0)",
        "P6 (+60,0)",
        "P7 (-60,-60)",
        "P8 (0,-60)",
        "P9 (+60,-60)",
    )
    index = 0
    for y_mm in y_positions:
        for x_mm in x_positions:
            draw_target(pdf, x_mm, y_mm, labels[index])
            index += 1

    axis_x = 25 * mm
    axis_y = 52 * mm
    pdf.setLineWidth(0.45 * mm)
    draw_arrow(pdf, (axis_x, axis_y), (axis_x + 22 * mm, axis_y))
    draw_arrow(pdf, (axis_x, axis_y), (axis_x, axis_y + 22 * mm))
    pdf.setFont("Helvetica-Bold", 8)
    pdf.drawString(axis_x + 24 * mm, axis_y - 1.5 * mm, "BASE +X")
    pdf.drawString(axis_x - 5 * mm, axis_y + 25 * mm, "BASE +Y")
    pdf.setFont("Helvetica", 7.5)
    pdf.drawString(20 * mm, 43 * mm, "Align these arrows with the robot base axes.")

    bar_start_x = 55 * mm
    bar_end_x = 155 * mm
    bar_y = 30 * mm
    pdf.setLineWidth(0.5 * mm)
    pdf.line(bar_start_x, bar_y, bar_end_x, bar_y)
    pdf.line(bar_start_x, bar_y - 3 * mm, bar_start_x, bar_y + 3 * mm)
    pdf.line(bar_end_x, bar_y - 3 * mm, bar_end_x, bar_y + 3 * mm)
    pdf.setFont("Helvetica-Bold", 8)
    pdf.drawCentredString(width / 2, 34 * mm, "100 mm SCALE CHECK")
    pdf.setFont("Helvetica", 7.5)
    pdf.drawCentredString(width / 2, 22 * mm, "Measure this line after printing. Do not use the sheet unless it is 100.0 mm.")
    pdf.drawRightString(width - 12 * mm, 13 * mm, "A4 210 x 297 mm")

    pdf.showPage()
    pdf.save()


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate the A4 hand-eye validation target.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    generate(args.output.expanduser().resolve())
    print(args.output.expanduser().resolve())


if __name__ == "__main__":
    main()
