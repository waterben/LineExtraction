#!/usr/bin/env python3
"""Generate synthetic example images with known ground truth line segments.

Creates two example datasets for accuracy evaluation in the line analyzer:

**Easy (single hexagon):**

  - ``resources/example_lines.png`` — 320x320 single-polygon step-edge image
  - ``resources/datasets/ground_truth/example_gt.csv`` — 6 ground truth segments

**Challenge (multi-shape scene):**

  - ``resources/example_challenge.png`` — 320x320 multi-polygon scene with noise
  - ``resources/datasets/ground_truth/example_challenge_gt.csv`` — 31 ground truth
    segments at varying contrast levels

Both images use the thesis approach (Chapter 6, Sub-Pixel Edge Precision): polygons
are rasterized at ultra-high resolution (32000x32000), Gaussian-blurred, and
downscaled 100x.  This produces clean single-gradient step edges between regions —
unlike drawn lines which create double gradient ridges.

The challenge image additionally features:
  - 8 non-overlapping shapes at different gray levels
  - High-contrast edges (easy), moderate-contrast, and very-low-contrast edges (hard)
  - Gaussian noise (sigma=5) for a more realistic scenario
  - Short segments (~15-20 px) mixed with long ones (~140-250 px)
  - Close parallel edges (20 px apart)

Usage::

    python tools/scripts/generate_example_gt.py

See also:
    ``evaluation/thesis/src/spe_precision.cpp`` — original C++ approach
"""

from __future__ import annotations

import csv
import os
from pathlib import Path
from typing import List, NamedTuple, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Common configuration
# ---------------------------------------------------------------------------

#: High-resolution canvas size.  Polygons are drawn here and downscaled by
#: ``SCALE`` (100x) to produce 320x320 output images.
HIRES_SIZE: int = 32000

#: Downscale factor applied after Gaussian blurring.
SCALE: float = 0.01

#: Gaussian blur kernel (applied at high resolution before downscaling).
GAUSS_KERNEL: int = 201

#: Gaussian blur sigma (applied at high resolution before downscaling).
GAUSS_SIGMA: int = 50

#: Random seed for reproducible noise generation.
NOISE_SEED: int = 42


# ---------------------------------------------------------------------------
# Shape definition
# ---------------------------------------------------------------------------


class Shape(NamedTuple):
    """A filled polygon with a given intensity level.

    :param name: Human-readable label for the shape.
    :param vertices_hires: Polygon vertices in high-resolution coordinates.
    :param gray: Fill intensity (0-255).
    """

    name: str
    vertices_hires: List[Tuple[int, int]]
    gray: int


class SceneConfig(NamedTuple):
    """Full configuration for a synthetic scene.

    :param image_name: Output image filename (placed in ``resources/``).
    :param csv_name: Output CSV filename (placed in ``resources/datasets/ground_truth/``).
    :param txt_name: Output TXT filename for Line Analyser 2D (plain ``x1,y1,x2,y2``).
    :param background: Canvas background intensity (0-255).
    :param noise_sigma: Standard deviation of additive Gaussian noise (0 = none).
    :param shapes: List of polygon shapes composing the scene.
    """

    image_name: str
    csv_name: str
    txt_name: str
    background: int
    noise_sigma: float
    shapes: List[Shape]


# ---------------------------------------------------------------------------
# Scene definitions
# ---------------------------------------------------------------------------

#: **Easy scene** — single irregular hexagon on a black background.
#:
#: Replicates the thesis Sub-Pixel Edge Precision image (spe_precision.cpp).
#: 6 edges at various orientations, very high contrast (180 vs 0).
#:
#: ::
#:
#:       V0 ──────────── V1
#:      ╱                  ╲
#:     ╱       180          V2
#:    ╱                    ╱
#:   V4 ──────────── V3
#:        V5
EASY_SCENE = SceneConfig(
    image_name="example_lines.png",
    csv_name="example_gt.csv",
    txt_name="example_gt.txt",
    background=0,
    noise_sigma=0,
    shapes=[
        Shape(
            "hexagon",
            [
                (5089, 2023),  # V0: top-left
                (29947, 2023),  # V1: top-right
                (20971, 16007),  # V2: mid-right
                (29947, 29959),  # V3: bottom-right
                (5089, 29959),  # V4: bottom-left
                (2017, 16007),  # V5: mid-left
            ],
            180,
        ),
    ],
)

#: **Challenge scene** — 8 non-overlapping shapes with varying contrast and noise.
#:
#: Layout (output coordinates, 320x320):
#:
#: ::
#:
#:   ┌─────────────────────────────────────────┐
#:   │                 [small sq]               │
#:   │   ┌──────────┐  15x15      ╱╲           │
#:   │   │          │           ╱    ╲          │
#:   │   │  large   │  gray   ╱ large ╲        │
#:   │   │  rect    │   200  ╱triangle ╲       │
#:   │   │  gray=40 │      ╱   gray=60  ╲      │
#:   │   │          │     ╱               ╲     │
#:   │   └──────────┘    ╱─────────────────╲    │
#:   │  ┌┐                                     │
#:   │  ││tall     ◇ diamond  ┌──────────────┐  │
#:   │  ││narrow    gray=95   │ parallelogram│  │
#:   │  ││rect      (low!)    │  gray=50     │  │
#:   │  ││gray=200            └──────────────┘  │
#:   │  ││         △tiny                        │
#:   │  ││         gray=115    ╱────────╲       │
#:   │  ││         (v.low!)   │ pentagon │      │
#:   │  ││                    │ gray=175 │      │
#:   │  └┘                     ╲────────╱       │
#:   └─────────────────────────────────────────┘
#:
#: Contrast levels (|gray - 128| background):
#:   - HIGH  (≥68): large_rect=88, small_sq=72, large_tri=68,
#:                   tall_narrow=72, parallelogram=78
#:   - MODERATE (47): pentagon
#:   - LOW (33): diamond
#:   - VERY LOW (13): tiny_triangle  (SNR ≈ 2.6 with noise sigma=5)
CHALLENGE_SCENE = SceneConfig(
    image_name="example_challenge.png",
    csv_name="example_challenge_gt.csv",
    txt_name="example_challenge_gt.txt",
    background=128,
    noise_sigma=5,
    shapes=[
        # --- High contrast shapes (easy edges) ---
        Shape(
            "large_rect",
            [
                (2000, 4000),
                (13000, 4000),
                (13000, 14000),
                (2000, 14000),
            ],
            40,
        ),
        Shape(
            "small_square",
            [
                (14000, 1000),
                (15500, 1000),
                (15500, 2500),
                (14000, 2500),
            ],
            200,
        ),
        Shape(
            "large_triangle",
            [
                (17000, 4000),
                (30000, 9500),
                (20000, 15000),
            ],
            60,
        ),
        Shape(
            "tall_narrow_rect",
            [
                (1500, 16500),
                (3500, 16500),
                (3500, 30500),
                (1500, 30500),
            ],
            200,
        ),
        # --- Moderate contrast shapes ---
        Shape(
            "parallelogram",
            [
                (17000, 17500),
                (28500, 17500),
                (27000, 24000),
                (15500, 24000),
            ],
            50,
        ),
        Shape(
            "pentagon",
            [
                (22000, 25000),
                (31000, 26000),
                (30000, 31000),
                (23000, 31000),
                (19500, 28000),
            ],
            175,
        ),
        # --- Low contrast shapes (hard edges) ---
        Shape(
            "diamond",
            [
                (10000, 17000),
                (13000, 20000),
                (10000, 23000),
                (7000, 20000),
            ],
            95,
        ),
        # --- Very low contrast (likely undetectable with noise) ---
        Shape(
            "tiny_triangle",
            [
                (4700, 24500),
                (6500, 25500),
                (5000, 27000),
            ],
            115,
        ),
    ],
)


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------


def workspace_root() -> Path:
    """Return the workspace root directory.

    :return: Absolute path to the repository root.
    :rtype: Path
    """
    script_dir = Path(__file__).resolve().parent
    # tools/scripts/ -> workspace root
    return script_dir.parent.parent


def _scale_vertices(
    vertices: List[Tuple[int, int]], scale: float
) -> List[Tuple[float, float]]:
    """Scale polygon vertices by a given factor.

    :param vertices: Vertices in high-resolution coordinates.
    :type vertices: list of tuple[int, int]
    :param scale: Scale factor to apply.
    :type scale: float
    :return: Scaled vertices as float coordinate pairs.
    :rtype: list of tuple[float, float]
    """
    return [(x * scale, y * scale) for x, y in vertices]


def _polygon_edges(
    vertices: List[Tuple[float, float]],
) -> List[Tuple[float, float, float, float]]:
    """Extract edges (line segments) from a closed polygon.

    Each edge connects consecutive vertices; the last vertex connects
    back to the first.

    :param vertices: Ordered polygon vertices.
    :type vertices: list of tuple[float, float]
    :return: Line segments as (x1, y1, x2, y2) tuples.
    :rtype: list of tuple[float, float, float, float]
    """
    segments: List[Tuple[float, float, float, float]] = []
    n = len(vertices)
    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]
        segments.append((x1, y1, x2, y2))
    return segments


# ---------------------------------------------------------------------------
# Image generation
# ---------------------------------------------------------------------------


def generate_scene(
    scene: SceneConfig,
) -> Tuple[np.ndarray, List[Tuple[float, float, float, float]]]:
    """Generate a synthetic step-edge image from a scene configuration.

    All shapes are rasterized at ultra-high resolution, Gaussian-blurred,
    and downscaled.  Optional Gaussian noise is added to the final image.

    :param scene: Scene configuration describing shapes and rendering params.
    :type scene: SceneConfig
    :return: Tuple of (grayscale image, list of ground truth segments).
    :rtype: tuple[numpy.ndarray, list[tuple[float, float, float, float]]]
    """
    out_size = int(HIRES_SIZE * SCALE)
    print(
        f"  Creating {HIRES_SIZE}x{HIRES_SIZE} high-res canvas "
        f"(background={scene.background})...",
        flush=True,
    )
    hires = np.full((HIRES_SIZE, HIRES_SIZE), scene.background, dtype=np.uint8)

    # Fill each polygon at full resolution.
    for shape in scene.shapes:
        pts = np.array(shape.vertices_hires, dtype=np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(hires, [pts], shape.gray)
        contrast = abs(shape.gray - scene.background)
        print(f"    {shape.name}: gray={shape.gray}, contrast={contrast}", flush=True)

    print(
        f"  Applying Gaussian blur (kernel={GAUSS_KERNEL}, sigma={GAUSS_SIGMA})...",
        flush=True,
    )
    cv2.GaussianBlur(hires, (GAUSS_KERNEL, GAUSS_KERNEL), GAUSS_SIGMA, dst=hires)

    print(f"  Downscaling by {1 / SCALE:.0f}x -> {out_size}x{out_size}...", flush=True)
    img = cv2.resize(hires, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)

    # Add Gaussian noise if configured.
    if scene.noise_sigma > 0:
        print(
            f"  Adding Gaussian noise (sigma={scene.noise_sigma}, "
            f"seed={NOISE_SEED})...",
            flush=True,
        )
        rng = np.random.default_rng(seed=NOISE_SEED)
        noise = rng.normal(0.0, scene.noise_sigma, img.shape)
        img = np.clip(img.astype(np.float64) + noise, 0, 255).astype(np.uint8)

    # Collect ground truth segments from all shapes.
    segments: List[Tuple[float, float, float, float]] = []
    for shape in scene.shapes:
        verts_scaled = _scale_vertices(shape.vertices_hires, SCALE)
        segments.extend(_polygon_edges(verts_scaled))

    return img, segments


def write_ground_truth_csv(
    csv_path: Path,
    image_name: str,
    segments: List[Tuple[float, float, float, float]],
) -> None:
    """Write polygon edge segments to a ground truth CSV file.

    Format: ``image_name,x1,y1,x2,y2`` with a header row.
    Used by the Accuracy Measure panel.

    :param csv_path: Output CSV file path.
    :type csv_path: Path
    :param image_name: Image filename stored in the first column.
    :type image_name: str
    :param segments: Line segments as (x1, y1, x2, y2) tuples.
    :type segments: list of tuple[float, float, float, float]
    """
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["image_name", "x1", "y1", "x2", "y2"])
        for x1, y1, x2, y2 in segments:
            writer.writerow(
                [image_name, f"{x1:.2f}", f"{y1:.2f}", f"{x2:.2f}", f"{y2:.2f}"]
            )


def write_ground_truth_txt(
    txt_path: Path,
    segments: List[Tuple[float, float, float, float]],
) -> None:
    """Write polygon edge segments to a plain text file.

    Format: ``x1,y1,x2,y2`` per line, no header.  This is the format
    expected by the Line Analyser 2D extension.

    :param txt_path: Output text file path.
    :type txt_path: Path
    :param segments: Line segments as (x1, y1, x2, y2) tuples.
    :type segments: list of tuple[float, float, float, float]
    """
    txt_path.parent.mkdir(parents=True, exist_ok=True)

    with open(txt_path, "w") as f:
        for x1, y1, x2, y2 in segments:
            f.write(f"{x1:.2f},{y1:.2f},{x2:.2f},{y2:.2f}\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def generate_one(scene: SceneConfig, root: Path) -> None:
    """Generate a single scene: image file and ground truth CSV.

    :param scene: Scene configuration.
    :type scene: SceneConfig
    :param root: Workspace root directory.
    :type root: Path
    """
    img_path = root / "resources" / scene.image_name
    gt_dir = root / "resources" / "datasets" / "ground_truth"
    csv_path = gt_dir / scene.csv_name
    txt_path = gt_dir / scene.txt_name

    n_shapes = len(scene.shapes)
    total_edges = sum(len(s.vertices_hires) for s in scene.shapes)
    print(f"\n{'=' * 60}")
    print(f"Generating: {scene.image_name}")
    print(f"  Shapes: {n_shapes}, total edges: {total_edges}")
    print(f"  Background: {scene.background}, noise sigma: {scene.noise_sigma}")
    print(f"{'=' * 60}")

    img, segments = generate_scene(scene)

    cv2.imwrite(str(img_path), img)
    print(
        f"  -> Image:  {os.path.relpath(img_path, root)}  "
        f"({img.shape[1]}x{img.shape[0]})"
    )

    write_ground_truth_csv(csv_path, scene.image_name, segments)
    print(f"  -> GT CSV: {os.path.relpath(csv_path, root)}  ({len(segments)} segments)")

    write_ground_truth_txt(txt_path, segments)
    print(f"  -> GT TXT: {os.path.relpath(txt_path, root)}  ({len(segments)} segments)")

    # Print segment details.
    print("\n  Ground truth segments (output coords):")
    for i, (x1, y1, x2, y2) in enumerate(segments):
        length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        print(
            f"    [{i:2d}] ({x1:6.2f},{y1:6.2f}) -> "
            f"({x2:6.2f},{y2:6.2f})  len={length:.1f}"
        )


def main() -> None:
    """Generate all example images and ground truth CSV files."""
    root = workspace_root()

    for scene in (EASY_SCENE, CHALLENGE_SCENE):
        generate_one(scene, root)

    print(f"\n{'=' * 60}")
    print("Done. Load examples via the Accuracy Measure or")
    print("Line Analyser 2D panel buttons.")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
