#!/usr/bin/env python3
"""Generate a synthetic example image with known ground truth line segments.

Creates:
  - resources/example_lines.png   — 300x300 grayscale image with clear line segments
  - resources/datasets/ground_truth/example_gt.csv — matching ground truth in CSV format

The synthetic image contains horizontal, vertical, and diagonal lines drawn with
anti-aliased rendering on a gray background, making it a good test case for line
detection algorithms.  The ground truth records the exact pixel coordinates used
to draw each segment so accuracy evaluation produces meaningful metrics.

Usage:
    python tools/scripts/generate_example_gt.py
"""

from __future__ import annotations

import csv
import os
from pathlib import Path

import cv2
import numpy as np


def workspace_root() -> Path:
    """Return the workspace root directory."""
    script_dir = Path(__file__).resolve().parent
    # tools/scripts/ -> workspace root
    return script_dir.parent.parent


# ---------------------------------------------------------------------------
# Ground truth line segments: (x1, y1, x2, y2)
# Designed to cover various orientations and lengths while staying within a
# 300x300 canvas with a small margin.
# ---------------------------------------------------------------------------
SEGMENTS: list[tuple[float, float, float, float]] = [
    # Horizontal lines
    (30, 40, 270, 40),
    (50, 150, 250, 150),
    (30, 260, 270, 260),
    # Vertical lines
    (40, 30, 40, 270),
    (150, 50, 150, 250),
    (260, 30, 260, 270),
    # Diagonal lines (45 degrees)
    (50, 50, 250, 250),
    (250, 50, 50, 250),
    # Angled lines (various slopes)
    (60, 100, 240, 200),
    (80, 220, 220, 100),
    # Short segments
    (120, 20, 180, 20),
    (20, 120, 20, 180),
]

IMAGE_NAME = "example_lines.png"
IMAGE_SIZE = 300
BACKGROUND_GRAY = 180
LINE_GRAY = 30
LINE_THICKNESS = 2


def generate_image(segments: list[tuple[float, float, float, float]]) -> np.ndarray:
    """Render line segments onto a grayscale canvas.

    Args:
        segments: List of (x1, y1, x2, y2) tuples.

    Returns:
        Grayscale uint8 image (IMAGE_SIZE x IMAGE_SIZE).
    """
    img = np.full((IMAGE_SIZE, IMAGE_SIZE), BACKGROUND_GRAY, dtype=np.uint8)

    for x1, y1, x2, y2 in segments:
        pt1 = (int(round(x1)), int(round(y1)))
        pt2 = (int(round(x2)), int(round(y2)))
        cv2.line(img, pt1, pt2, int(LINE_GRAY), LINE_THICKNESS, cv2.LINE_AA)

    return img


def write_ground_truth_csv(
    csv_path: Path,
    image_name: str,
    segments: list[tuple[float, float, float, float]],
) -> None:
    """Write segments to a ground truth CSV file.

    Args:
        csv_path: Output CSV path.
        image_name: Image filename stored in the first column.
        segments: List of (x1, y1, x2, y2) tuples.
    """
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["image_name", "x1", "y1", "x2", "y2"])
        for x1, y1, x2, y2 in segments:
            writer.writerow(
                [image_name, f"{x1:.1f}", f"{y1:.1f}", f"{x2:.1f}", f"{y2:.1f}"]
            )


def main() -> None:
    """Generate the example image and ground truth CSV."""
    root = workspace_root()

    img_path = root / "resources" / IMAGE_NAME
    csv_path = root / "resources" / "datasets" / "ground_truth" / "example_gt.csv"

    # Generate and save image.
    img = generate_image(SEGMENTS)
    cv2.imwrite(str(img_path), img)
    print(
        f"Created image:  {os.path.relpath(img_path, root)}  ({IMAGE_SIZE}x{IMAGE_SIZE} grayscale)"
    )

    # Write ground truth CSV.
    write_ground_truth_csv(csv_path, IMAGE_NAME, SEGMENTS)
    print(
        f"Created GT CSV: {os.path.relpath(csv_path, root)}  ({len(SEGMENTS)} segments)"
    )
    print("\nDone. You can load example_gt.csv in the Accuracy Measure panel.")


if __name__ == "__main__":
    main()
