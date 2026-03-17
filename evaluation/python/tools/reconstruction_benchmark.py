#!/usr/bin/env python3
"""Benchmark 3D line reconstruction on stereo datasets.

Evaluates the stereo reconstruction pipeline on MDB (Middlebury) stereo
pairs, measuring:
- Detection / matching / triangulation throughput
- 3D line count and length distribution
- Reprojection error statistics
- Comparison of triangulation methods (midpoint, plane, OpenCV)

Usage::

    # Quick benchmark on default MDB scenes
    bazel run //evaluation/python:reconstruction_benchmark

    # Select specific scenes
    bazel run //evaluation/python:reconstruction_benchmark -- \\
        --scenes Adirondack Jadeplant Piano

    # Full benchmark with all methods
    bazel run //evaluation/python:reconstruction_benchmark -- \\
        --methods midpoint plane opencv --min-length 15

Output:
    Per-scene and aggregate statistics printed to stdout.  Optionally
    writes a CSV summary to ``evaluation/python/reports/recon_benchmark.csv``.
"""

from __future__ import annotations

import argparse
import csv
import io
import logging
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

# Silence OpenCV parallel backend warnings on stderr
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data class for per-scene results
# ---------------------------------------------------------------------------


@dataclass
class SceneResult:
    """Aggregated benchmark result for one scene / method combination.

    :param scene: Scene name.
    :param method: Triangulation method used.
    :param n_segs_left: Detected left segments.
    :param n_segs_right: Detected right segments.
    :param n_matches: Number of matches after filtering.
    :param n_3d: Number of reconstructed 3D lines.
    :param time_detect: Detection time (seconds).
    :param time_desc: Descriptor time (seconds).
    :param time_match: Matching time (seconds).
    :param time_triang: Triangulation time (seconds).
    :param time_total: Total pipeline time (seconds).
    :param reproj_median: Median reprojection error (px).
    :param reproj_mean: Mean reprojection error (px).
    :param reproj_max: Max reprojection error (px).
    :param length_median: Median 3D segment length.
    :param length_mean: Mean 3D segment length.
    :param depth_median: Median depth (z-coordinate of midpoints).
    """

    scene: str = ""
    method: str = ""
    n_segs_left: int = 0
    n_segs_right: int = 0
    n_matches: int = 0
    n_3d: int = 0
    time_detect: float = 0.0
    time_desc: float = 0.0
    time_match: float = 0.0
    time_triang: float = 0.0
    time_total: float = 0.0
    reproj_median: float = 0.0
    reproj_mean: float = 0.0
    reproj_max: float = 0.0
    length_median: float = 0.0
    length_mean: float = 0.0
    depth_median: float = 0.0


# ---------------------------------------------------------------------------
# Core benchmark logic
# ---------------------------------------------------------------------------


def benchmark_scene(
    img_left: np.ndarray,
    img_right: np.ndarray,
    cam_left: Any,
    cam_right: Any,
    *,
    scene_name: str = "",
    method: str = "midpoint",
    min_length: float = 25.0,
    max_reproj: float = 10.0,
) -> SceneResult:
    """Run the full reconstruction pipeline on a stereo pair.

    Delegates to :func:`lsfm.reconstruction.reconstruct_lines_stereo`
    for the actual work.

    :param img_left: Grayscale left image.
    :type img_left: numpy.ndarray
    :param img_right: Grayscale right image.
    :type img_right: numpy.ndarray
    :param cam_left: Left camera object.
    :param cam_right: Right camera object.
    :param scene_name: Label for the scene.
    :type scene_name: str
    :param method: Triangulation method (``"midpoint"``, ``"plane"``,
        ``"opencv"``).
    :type method: str
    :param min_length: Minimum 2D segment length in pixels.
    :type min_length: float
    :param max_reproj: Maximum reprojection error for filtering.
    :type max_reproj: float
    :return: Benchmark result.
    :rtype: SceneResult
    """
    from lsfm.reconstruction import StereoConfig, reconstruct_lines_stereo

    config = StereoConfig(
        min_length=min_length,
        triangulation_method=method,
        max_reproj_error=max_reproj,
        merge_enabled=False,
    )
    sr = reconstruct_lines_stereo(
        img_left, img_right, cam_left, cam_right, config=config
    )

    res = SceneResult(scene=scene_name, method=method)
    res.n_segs_left = len(sr.segments_left)
    res.n_segs_right = len(sr.segments_right)
    res.n_matches = len(sr.matches) if sr.matches is not None else 0

    timings = sr.timings
    res.time_detect = timings.get("detection", 0.0)
    res.time_desc = timings.get("descriptors", 0.0)
    res.time_match = timings.get("matching", 0.0)
    res.time_triang = timings.get("triangulation", 0.0)

    res.n_3d = sr.n_lines_3d

    if sr.reproj_errors is not None and sr.reproj_errors.size > 0:
        res.reproj_median = float(np.median(sr.reproj_errors))
        res.reproj_mean = float(np.mean(sr.reproj_errors))
        res.reproj_max = float(np.max(sr.reproj_errors))

    good_3d = sr.segments_3d
    if good_3d:
        lengths = np.array([s.length for s in good_3d])
        res.length_median = float(np.median(lengths))
        res.length_mean = float(np.mean(lengths))

        depths = []
        for s in good_3d:
            sp = np.array(s.start_point())
            ep = np.array(s.end_point())
            depths.append((sp[2] + ep[2]) / 2.0)
        res.depth_median = float(np.median(depths))

    res.time_total = res.time_detect + res.time_desc + res.time_match + res.time_triang
    return res


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def format_table(results: list[SceneResult]) -> str:
    """Format results as an aligned text table.

    :param results: Scene benchmark results.
    :type results: list[SceneResult]
    :return: Formatted table string.
    :rtype: str
    """
    buf = io.StringIO()

    header = (
        f"{'Scene':<20} {'Method':<10} {'SegL':>5} {'SegR':>5} "
        f"{'Match':>5} {'3D':>5} {'RPmed':>6} {'RPavg':>6} "
        f"{'Len':>7} {'Depth':>7} {'Time':>7}"
    )
    sep = "-" * len(header)

    buf.write(f"\n{sep}\n{header}\n{sep}\n")
    for r in results:
        buf.write(
            f"{r.scene:<20} {r.method:<10} {r.n_segs_left:>5} "
            f"{r.n_segs_right:>5} {r.n_matches:>5} {r.n_3d:>5} "
            f"{r.reproj_median:>6.2f} {r.reproj_mean:>6.2f} "
            f"{r.length_median:>7.1f} {r.depth_median:>7.1f} "
            f"{r.time_total:>7.3f}\n"
        )
    buf.write(f"{sep}\n")

    # Aggregates per method
    methods = sorted(set(r.method for r in results))
    if len(results) > 1:
        buf.write("\nAggregates:\n")
        for m in methods:
            group = [r for r in results if r.method == m]
            n3d = sum(r.n_3d for r in group)
            avg_reproj = np.mean([r.reproj_median for r in group if r.n_3d > 0])
            avg_time = np.mean([r.time_total for r in group])
            buf.write(
                f"  {m:<10}: {n3d:>5} total 3D lines, "
                f"reproj_med={avg_reproj:.2f} px, "
                f"avg_time={avg_time:.3f}s\n"
            )

    return buf.getvalue()


def write_csv(results: list[SceneResult], path: Path) -> None:
    """Write results to a CSV file.

    :param results: Scene benchmark results.
    :type results: list[SceneResult]
    :param path: Output CSV file path.
    :type path: Path
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(SceneResult.__dataclass_fields__)  # type: ignore[attr-defined]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow(asdict(r))
    logger.info("CSV written: %s", path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


_DEFAULT_SCENES = [
    "Adirondack",
    "Jadeplant",
    "Motorcycle",
    "Piano",
    "Pipes",
    "Playroom",
    "Playtable",
    "Recycle",
    "Shelves",
    "Vintage",
]


def main() -> None:
    """Entry point for the reconstruction benchmark."""
    parser = argparse.ArgumentParser(
        description="Benchmark 3D line reconstruction on MDB stereo pairs."
    )
    parser.add_argument(
        "--scenes",
        nargs="+",
        default=None,
        help="Scene names to benchmark (default: all available MDB-H scenes).",
    )
    parser.add_argument(
        "--methods",
        nargs="+",
        default=["midpoint"],
        choices=["midpoint", "plane", "opencv"],
        help="Triangulation methods to compare.",
    )
    parser.add_argument(
        "--min-length",
        type=float,
        default=25.0,
        help="Minimum 2D segment length (pixels).",
    )
    parser.add_argument(
        "--max-reproj",
        type=float,
        default=10.0,
        help="Maximum reprojection error for filtering (pixels).",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="Path for CSV output (default: evaluation/python/reports/recon_benchmark.csv).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging.",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    from PIL import Image  # noqa: E402
    from lsfm.data import TestImages

    ds = TestImages()

    # Discover available scenes
    if args.scenes:
        scenes = args.scenes
    else:
        available = []
        for s in _DEFAULT_SCENES:
            try:
                ds.get(f"MDB/MiddEval3-H/{s}/im0.png")
                available.append(s)
            except FileNotFoundError:
                pass
        scenes = available if available else _DEFAULT_SCENES[:2]

    logger.info("Benchmarking %d scenes × %d methods", len(scenes), len(args.methods))

    results: list[SceneResult] = []

    for scene in scenes:
        try:
            img_l_path = ds.get(f"MDB/MiddEval3-H/{scene}/im0.png")
            img_r_path = ds.get(f"MDB/MiddEval3-H/{scene}/im1.png")
        except FileNotFoundError:
            logger.warning("Scene %s not found, skipping.", scene)
            continue

        img_l = np.array(Image.open(img_l_path).convert("L"), dtype=np.uint8)
        img_r = np.array(Image.open(img_r_path).convert("L"), dtype=np.uint8)

        try:
            cam_l, cam_r = ds.stereo_camera_pair(scene)
        except (FileNotFoundError, KeyError) as exc:
            logger.warning("No calibration for %s: %s", scene, exc)
            continue

        for method in args.methods:
            logger.info("  %s / %s ...", scene, method)
            res = benchmark_scene(
                img_l,
                img_r,
                cam_l,
                cam_r,
                scene_name=scene,
                method=method,
                min_length=args.min_length,
                max_reproj=args.max_reproj,
            )
            results.append(res)

    # Print results
    print(format_table(results))

    # Write CSV
    csv_path = args.csv
    if csv_path is None:
        # Default location
        ws_root = Path(__file__).resolve().parents[2]
        csv_path = str(
            ws_root / "evaluation" / "python" / "reports" / "recon_benchmark.csv"
        )
    write_csv(results, Path(csv_path))
    print(f"CSV saved: {csv_path}")


if __name__ == "__main__":
    main()
