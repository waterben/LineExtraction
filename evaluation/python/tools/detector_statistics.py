#!/usr/bin/env python3
"""Compute detection statistics for LSD detectors across benchmark datasets.

Runs each detector on York Urban and/or Wireframe images using three
parameter strategies, collects per-image precision / recall / F1, and
produces aggregate summary tables plus optional CSV / JSON output.

**Parameter strategies:**

  1. **default** — detector's built-in defaults (no profile).
  2. **profile** — ``DetectorProfile(50, 50, 50, 50)`` (neutral knobs).
  3. **adaptive** — ``DetectorProfile.from_image(img)`` (image-aware).

Additionally, ``ImageAnalyzer`` statistics are computed for every image
so the user can correlate image characteristics with detection quality.

Usage::

    # Quick test: one detector, a few images
    bazel run //evaluation/python:detector_statistics -- \\
        --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \\
        --image-dir resources/datasets/YorkUrban/images \\
        --detectors LsdFGioi --max-images 10

    # Full run on both datasets
    bazel run //evaluation/python:detector_statistics -- \\
        --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \\
                       resources/datasets/ground_truth/wireframe_gt.csv \\
        --image-dir resources/datasets/YorkUrban/images \\
                    resources/datasets/Wireframe/images \\
        --output-dir results/detector_stats

Output:
    Console summary table, per-image CSV, aggregate JSON, and optionally
    image-property analysis CSV.
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import multiprocessing as mp
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

# ---------------------------------------------------------------------------
# Lazy imports for pybind11 modules
# ---------------------------------------------------------------------------

_le_algorithm: Any = None
_le_geometry: Any = None
_le_lsd: Any = None


def _import_modules() -> None:
    """Import native pybind11 modules lazily.

    Avoids import errors when the script is analysed by linters or
    run outside of the Bazel environment.
    """
    global _le_algorithm, _le_geometry, _le_lsd  # noqa: PLW0603
    if _le_algorithm is not None:
        return
    import le_algorithm  # type: ignore[import-untyped]
    import le_geometry  # type: ignore[import-untyped]
    import le_lsd  # type: ignore[import-untyped]

    _le_algorithm = le_algorithm
    _le_geometry = le_geometry
    _le_lsd = le_lsd


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("detector_statistics")


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class ImageStats:
    """Per-image result record.

    :param image_name: Filename of the image.
    :type image_name: str
    :param detector: Detector name.
    :type detector: str
    :param strategy: Parameter strategy name.
    :type strategy: str
    :param precision: Precision value.
    :type precision: float
    :param recall: Recall value.
    :type recall: float
    :param f1: F1 score.
    :type f1: float
    :param num_detected: Number of detected segments.
    :type num_detected: int
    :param num_gt: Number of ground truth segments.
    :type num_gt: int
    :param tp: True positives.
    :type tp: int
    :param fp: False positives.
    :type fp: int
    :param fn: False negatives.
    :type fn: int
    :param detect_time_ms: Detection time in milliseconds.
    :type detect_time_ms: float
    """

    image_name: str
    detector: str
    strategy: str
    precision: float
    recall: float
    f1: float
    num_detected: int
    num_gt: int
    tp: int
    fp: int
    fn: int
    detect_time_ms: float


@dataclass
class ImagePropertyRecord:
    """Per-image ImageAnalyzer property record.

    :param image_name: Filename of the image.
    :type image_name: str
    :param contrast: Normalized contrast [0, 1].
    :type contrast: float
    :param noise_level: Normalized noise level [0, 1].
    :type noise_level: float
    :param edge_density: Normalized edge density [0, 1].
    :type edge_density: float
    :param dynamic_range: Normalized dynamic range [0, 1].
    :type dynamic_range: float
    :param hint_detail: Suggested detail knob [0, 100].
    :type hint_detail: float
    :param hint_gap_tolerance: Suggested gap tolerance knob [0, 100].
    :type hint_gap_tolerance: float
    :param hint_min_length: Suggested min_length knob [0, 100].
    :type hint_min_length: float
    :param hint_precision: Suggested precision knob [0, 100].
    :type hint_precision: float
    :param hint_contrast_factor: Adaptive contrast factor [0.5, 2.0].
    :type hint_contrast_factor: float
    :param hint_noise_factor: Adaptive noise factor [0.5, 2.0].
    :type hint_noise_factor: float
    """

    image_name: str
    contrast: float
    noise_level: float
    edge_density: float
    dynamic_range: float
    hint_detail: float
    hint_gap_tolerance: float
    hint_min_length: float
    hint_precision: float
    hint_contrast_factor: float
    hint_noise_factor: float


@dataclass
class AggregateStats:
    """Aggregate statistics for a (detector, strategy) combination.

    :param detector: Detector name.
    :type detector: str
    :param strategy: Parameter strategy name.
    :type strategy: str
    :param num_images: Number of images evaluated.
    :type num_images: int
    :param mean_precision: Mean precision across images.
    :type mean_precision: float
    :param mean_recall: Mean recall across images.
    :type mean_recall: float
    :param mean_f1: Mean F1 across images.
    :type mean_f1: float
    :param std_f1: Standard deviation of F1.
    :type std_f1: float
    :param median_f1: Median F1 across images.
    :type median_f1: float
    :param mean_detected: Mean number of detected segments.
    :type mean_detected: float
    :param mean_time_ms: Mean detection time per image (ms).
    :type mean_time_ms: float
    :param total_tp: Total true positives.
    :type total_tp: int
    :param total_fp: Total false positives.
    :type total_fp: int
    :param total_fn: Total false negatives.
    :type total_fn: int
    :param micro_precision: Micro-averaged precision (TP / (TP + FP)).
    :type micro_precision: float
    :param micro_recall: Micro-averaged recall (TP / (TP + FN)).
    :type micro_recall: float
    :param micro_f1: Micro-averaged F1.
    :type micro_f1: float
    """

    detector: str
    strategy: str
    num_images: int
    mean_precision: float
    mean_recall: float
    mean_f1: float
    std_f1: float
    median_f1: float
    mean_detected: float
    mean_time_ms: float
    total_tp: int
    total_fp: int
    total_fn: int
    micro_precision: float
    micro_recall: float
    micro_f1: float


# ---------------------------------------------------------------------------
# Detector registry
# ---------------------------------------------------------------------------

# (detector_name, class_name_in_le_lsd)
DETECTOR_REGISTRY: list[tuple[str, str]] = [
    ("LsdCC", "LsdCC_f64"),
    ("LsdCP", "LsdCP_f64"),
    ("LsdBurns", "LsdBurns_f64"),
    ("LsdFBW", "LsdFBW_f64"),
    ("LsdFGioi", "LsdFGioi_f64"),
    ("LsdEDLZ", "LsdEDLZ_f64"),
    ("LsdEL", "LsdEL_f64"),
    ("LsdEP", "LsdEP_f64"),
    ("LsdHoughP", "LsdHoughP_f64"),
]

STRATEGY_NAMES: list[str] = ["default", "profile", "adaptive"]


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------


def load_images(
    image_dirs: list[Path],
    gt_image_names: set[str],
    max_images: int = 0,
) -> list[tuple[str, np.ndarray]]:
    """Load grayscale images matching ground truth entries.

    Uses Pillow for image I/O to avoid conflicts with the statically
    linked OpenCV inside the pybind11 modules.

    :param image_dirs: Directories to search for images.
    :type image_dirs: list[Path]
    :param gt_image_names: Image filenames present in the ground truth.
    :type gt_image_names: set[str]
    :param max_images: Maximum images to load (0 = all).
    :type max_images: int
    :return: List of ``(image_name, grayscale_array)`` tuples.
    :rtype: list[tuple[str, numpy.ndarray]]
    """
    from PIL import Image

    images: list[tuple[str, np.ndarray]] = []
    seen: set[str] = set()

    for image_dir in image_dirs:
        if not image_dir.is_dir():
            log.warning("Image directory not found: %s", image_dir)
            continue
        for img_path in sorted(image_dir.iterdir()):
            if img_path.suffix.lower() not in {".jpg", ".jpeg", ".png", ".bmp"}:
                continue
            name = img_path.name
            if name not in gt_image_names or name in seen:
                continue
            try:
                pil_img = Image.open(str(img_path)).convert("L")
                img = np.array(pil_img, dtype=np.uint8)
            except Exception:
                log.warning("Failed to read: %s", img_path)
                continue
            images.append((name, img))
            seen.add(name)
            if 0 < max_images <= len(images):
                break
        if 0 < max_images <= len(images):
            break

    return images


def load_ground_truth(gt_paths: list[Path]) -> list[Any]:
    """Load ground truth entries from CSV files.

    :param gt_paths: Paths to ground truth CSV files.
    :type gt_paths: list[Path]
    :return: Combined list of GroundTruthEntry objects.
    :rtype: list
    """
    all_entries: list[Any] = []
    for gt_path in gt_paths:
        if not gt_path.is_file():
            log.warning("Ground truth not found: %s", gt_path)
            continue
        entries = _le_algorithm.GroundTruthLoader.load_csv(str(gt_path))
        log.info("Loaded %d GT entries from %s", len(entries), gt_path.name)
        all_entries.extend(entries)
    return all_entries


def format_duration(seconds: float) -> str:
    """Format seconds to human-readable string.

    :param seconds: Duration in seconds.
    :type seconds: float
    :return: Formatted string like ``"2m 30s"`` or ``"45.1s"``.
    :rtype: str
    """
    if seconds < 60:
        return f"{seconds:.1f}s"
    minutes = int(seconds // 60)
    secs = int(seconds % 60)
    if minutes < 60:
        return f"{minutes}m {secs}s"
    hours = minutes // 60
    mins = minutes % 60
    return f"{hours}h {mins}m"


# ---------------------------------------------------------------------------
# Detection with different strategies
# ---------------------------------------------------------------------------


def detect_with_strategy(
    detector_name: str,
    cls_name: str,
    strategy: str,
    image: np.ndarray,
    profile_cache: dict[str, Any] | None = None,
) -> tuple[list[Any], float]:
    """Run detection on a single image using the given strategy.

    :param detector_name: Human-readable detector name (e.g. ``"LsdFGioi"``).
    :type detector_name: str
    :param cls_name: Class name in ``le_lsd`` (e.g. ``"LsdFGioi_f64"``).
    :type cls_name: str
    :param strategy: One of ``"default"``, ``"profile"``, ``"adaptive"``.
    :type strategy: str
    :param image: Grayscale input image (uint8 numpy array).
    :type image: numpy.ndarray
    :param profile_cache: Optional dict caching adaptive profiles per image.
    :type profile_cache: dict or None
    :return: Tuple of (detected_segments, detection_time_ms).
    :rtype: tuple[list, float]
    """
    detector_cls = getattr(_le_lsd, cls_name)
    det = detector_cls()

    if strategy == "profile":
        # Neutral knobs, no adaptive factors
        profile = _le_algorithm.DetectorProfile(50, 50, 50, 50)
        params = profile.to_params_by_name(detector_name)
        _apply_params(det, params)
    elif strategy == "adaptive":
        # Image-adaptive profile
        profile = _le_algorithm.DetectorProfile.from_image(image)
        params = profile.to_params_by_name(detector_name)
        _apply_params(det, params)
    # else: "default" — use detector defaults

    t0 = time.perf_counter()
    det.detect(image)
    segments = det.line_segments()
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    return segments, elapsed_ms


def _apply_params(det: Any, params: list[dict[str, Any]]) -> None:
    """Apply a parameter config list to a detector instance.

    :param det: Detector instance with ``set_int`` / ``set_float`` methods.
    :type det: object
    :param params: Parameter configuration from ``DetectorProfile.to_params``.
    :type params: list[dict[str, Any]]
    """
    for p in params:
        name = p["name"]
        value = p["value"]
        if isinstance(value, bool):
            det.set_int(name, int(value))
        elif isinstance(value, int):
            det.set_int(name, value)
        elif isinstance(value, float):
            det.set_float(name, value)


# ---------------------------------------------------------------------------
# Image analysis
# ---------------------------------------------------------------------------


def analyze_images(
    images: list[tuple[str, np.ndarray]],
) -> list[ImagePropertyRecord]:
    """Run ImageAnalyzer on all images.

    :param images: List of ``(image_name, image)`` tuples.
    :type images: list[tuple[str, numpy.ndarray]]
    :return: List of per-image property records.
    :rtype: list[ImagePropertyRecord]
    """
    records: list[ImagePropertyRecord] = []
    for name, img in images:
        props = _le_algorithm.ImageAnalyzer.analyze(img)
        hints = props.suggest_profile()
        records.append(
            ImagePropertyRecord(
                image_name=name,
                contrast=round(props.contrast, 6),
                noise_level=round(props.noise_level, 6),
                edge_density=round(props.edge_density, 6),
                dynamic_range=round(props.dynamic_range, 6),
                hint_detail=round(hints.detail, 2),
                hint_gap_tolerance=round(hints.gap_tolerance, 2),
                hint_min_length=round(hints.min_length, 2),
                hint_precision=round(hints.precision, 2),
                hint_contrast_factor=round(hints.contrast_factor, 4),
                hint_noise_factor=round(hints.noise_factor, 4),
            )
        )
    return records


# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------


def _combo_worker(
    det_name: str,
    cls_name: str,
    strategy: str,
    images: list[tuple[str, np.ndarray]],
    gt_map: dict[str, list[Any]],
    match_threshold: float,
    result_queue: mp.Queue,
) -> None:
    """Run one (detector, strategy) combo inside a subprocess.

    Results are sent back via *result_queue*. If the worker crashes
    (e.g. segfault in native code), the parent detects a non-zero
    exit code and skips the combo.

    :param det_name: Human-readable detector name.
    :type det_name: str
    :param cls_name: Class name in ``le_lsd``.
    :type cls_name: str
    :param strategy: Strategy name.
    :type strategy: str
    :param images: ``(name, image)`` pairs.
    :type images: list[tuple[str, numpy.ndarray]]
    :param gt_map: Ground truth lookup.
    :type gt_map: dict[str, list]
    :param match_threshold: Endpoint matching threshold.
    :type match_threshold: float
    :param result_queue: Queue to put results into.
    :type result_queue: multiprocessing.Queue
    """
    measure = _le_algorithm.AccuracyMeasure(threshold=match_threshold)
    stats: list[ImageStats] = []

    for img_idx, (img_name, img) in enumerate(images):
        gt_segs = gt_map.get(img_name, [])
        if not gt_segs:
            continue

        try:
            detected, dt_ms = detect_with_strategy(det_name, cls_name, strategy, img)
        except Exception as exc:
            log.warning(
                "  %s/%s on %s failed: %s",
                det_name,
                strategy,
                img_name,
                exc,
            )
            continue

        result = measure.evaluate(detected, gt_segs)

        stats.append(
            ImageStats(
                image_name=img_name,
                detector=det_name,
                strategy=strategy,
                precision=round(result.precision, 6),
                recall=round(result.recall, 6),
                f1=round(result.f1, 6),
                num_detected=len(detected),
                num_gt=len(gt_segs),
                tp=result.true_positives,
                fp=result.false_positives,
                fn=result.false_negatives,
                detect_time_ms=round(dt_ms, 2),
            )
        )

    result_queue.put(stats)


def evaluate_all(
    detectors: list[tuple[str, str]],
    strategies: list[str],
    images: list[tuple[str, np.ndarray]],
    gt_map: dict[str, list[Any]],
    match_threshold: float,
) -> list[ImageStats]:
    """Run detection and evaluate across all combinations.

    Each (detector, strategy) combination runs in a forked subprocess so
    that native crashes (e.g. segfaults) are contained without terminating
    the whole evaluation.

    :param detectors: List of ``(detector_name, cls_name)`` tuples.
    :type detectors: list[tuple[str, str]]
    :param strategies: Strategy names to test.
    :type strategies: list[str]
    :param images: List of ``(image_name, image)`` tuples.
    :type images: list[tuple[str, numpy.ndarray]]
    :param gt_map: Ground truth segments indexed by image name.
    :type gt_map: dict[str, list]
    :param match_threshold: Endpoint distance threshold in pixels.
    :type match_threshold: float
    :return: List of per-image statistics records.
    :rtype: list[ImageStats]
    """
    all_stats: list[ImageStats] = []

    total_combos = len(detectors) * len(strategies)
    combo_idx = 0

    for det_name, cls_name in detectors:
        for strategy in strategies:
            combo_idx += 1
            log.info(
                "[%d/%d] %s / %s  (%d images)",
                combo_idx,
                total_combos,
                det_name,
                strategy,
                len(images),
            )
            t_start = time.time()

            result_queue: mp.Queue = mp.Queue()
            proc = mp.Process(
                target=_combo_worker,
                args=(
                    det_name,
                    cls_name,
                    strategy,
                    images,
                    gt_map,
                    match_threshold,
                    result_queue,
                ),
            )
            proc.start()
            proc.join()

            if proc.exitcode != 0:
                sig = -proc.exitcode if proc.exitcode < 0 else proc.exitcode
                log.warning(
                    "  CRASH %s / %s (signal %d) — skipping",
                    det_name,
                    strategy,
                    sig,
                )
                continue

            try:
                combo_stats = result_queue.get_nowait()
                all_stats.extend(combo_stats)
            except Exception:
                log.warning(
                    "  No results from %s / %s — skipping",
                    det_name,
                    strategy,
                )
                continue

            elapsed = time.time() - t_start
            log.info("  → done in %s", format_duration(elapsed))

    return all_stats

    return all_stats


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------


def aggregate(all_stats: list[ImageStats]) -> list[AggregateStats]:
    """Compute aggregate statistics per (detector, strategy).

    :param all_stats: Per-image results.
    :type all_stats: list[ImageStats]
    :return: Aggregate records sorted by detector then strategy.
    :rtype: list[AggregateStats]
    """
    from collections import defaultdict

    groups: dict[tuple[str, str], list[ImageStats]] = defaultdict(list)
    for s in all_stats:
        groups[(s.detector, s.strategy)].append(s)

    aggregates: list[AggregateStats] = []
    for (det, strat), records in sorted(groups.items()):
        n = len(records)
        if n == 0:
            continue

        precisions = [r.precision for r in records]
        recalls = [r.recall for r in records]
        f1s = [r.f1 for r in records]
        times = [r.detect_time_ms for r in records]
        detected_counts = [r.num_detected for r in records]

        total_tp = sum(r.tp for r in records)
        total_fp = sum(r.fp for r in records)
        total_fn = sum(r.fn for r in records)

        micro_p = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0.0
        micro_r = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0.0
        micro_f1 = (
            2 * micro_p * micro_r / (micro_p + micro_r)
            if (micro_p + micro_r) > 0
            else 0.0
        )

        aggregates.append(
            AggregateStats(
                detector=det,
                strategy=strat,
                num_images=n,
                mean_precision=round(statistics.mean(precisions), 4),
                mean_recall=round(statistics.mean(recalls), 4),
                mean_f1=round(statistics.mean(f1s), 4),
                std_f1=round(statistics.stdev(f1s), 4) if n > 1 else 0.0,
                median_f1=round(statistics.median(f1s), 4),
                mean_detected=round(statistics.mean(detected_counts), 1),
                mean_time_ms=round(statistics.mean(times), 2),
                total_tp=total_tp,
                total_fp=total_fp,
                total_fn=total_fn,
                micro_precision=round(micro_p, 4),
                micro_recall=round(micro_r, 4),
                micro_f1=round(micro_f1, 4),
            )
        )

    return aggregates


def aggregate_image_properties(
    records: list[ImagePropertyRecord],
) -> dict[str, Any]:
    """Compute summary statistics for image properties.

    :param records: Per-image property records.
    :type records: list[ImagePropertyRecord]
    :return: Dictionary with mean / std / min / max per property.
    :rtype: dict[str, Any]
    """
    if not records:
        return {}

    fields = [
        "contrast",
        "noise_level",
        "edge_density",
        "dynamic_range",
    ]
    summary: dict[str, Any] = {"num_images": len(records)}

    for f in fields:
        values = [getattr(r, f) for r in records]
        summary[f] = {
            "mean": round(statistics.mean(values), 4),
            "std": round(statistics.stdev(values), 4) if len(values) > 1 else 0.0,
            "min": round(min(values), 4),
            "max": round(max(values), 4),
            "median": round(statistics.median(values), 4),
        }

    return summary


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------


def print_summary_table(aggregates: list[AggregateStats]) -> None:
    """Print a formatted summary table to stdout.

    :param aggregates: Aggregate statistics to display.
    :type aggregates: list[AggregateStats]
    """
    print("\n" + "=" * 105)
    print("DETECTOR STATISTICS SUMMARY")
    print("=" * 105)
    header = (
        f"{'Detector':<12} {'Strategy':<10} {'N':>4} "
        f"{'mP':>6} {'mR':>6} {'mF1':>6} {'σF1':>5} {'mdF1':>6} "
        f"{'μP':>6} {'μR':>6} {'μF1':>6} "
        f"{'det/img':>7} {'ms/img':>7}"
    )
    print(header)
    print("-" * 105)

    current_det = ""
    for a in aggregates:
        if a.detector != current_det:
            if current_det:
                print("-" * 105)
            current_det = a.detector
        print(
            f"{a.detector:<12} {a.strategy:<10} {a.num_images:>4} "
            f"{a.mean_precision:>6.3f} {a.mean_recall:>6.3f} {a.mean_f1:>6.3f} "
            f"{a.std_f1:>5.3f} {a.median_f1:>6.3f} "
            f"{a.micro_precision:>6.3f} {a.micro_recall:>6.3f} {a.micro_f1:>6.3f} "
            f"{a.mean_detected:>7.1f} {a.mean_time_ms:>7.1f}"
        )

    print("=" * 105)

    # Strategy comparison: which strategy wins per detector
    from collections import defaultdict

    by_det: dict[str, dict[str, float]] = defaultdict(dict)
    for a in aggregates:
        by_det[a.detector][a.strategy] = a.mean_f1

    print("\nBest strategy per detector (by mean F1):")
    for det in sorted(by_det):
        strats = by_det[det]
        best = max(strats, key=lambda s: strats[s])
        print(f"  {det:<12} → {best:<10} (F1={strats[best]:.4f})")


def print_image_property_summary(summary: dict[str, Any]) -> None:
    """Print image property statistics to stdout.

    :param summary: Aggregated property statistics.
    :type summary: dict[str, Any]
    """
    print("\n" + "=" * 70)
    print(f"IMAGE PROPERTY ANALYSIS  ({summary.get('num_images', 0)} images)")
    print("=" * 70)
    header = (
        f"{'Property':<16} {'Mean':>8} {'Std':>8} {'Min':>8} {'Max':>8} {'Median':>8}"
    )
    print(header)
    print("-" * 70)
    for prop in ["contrast", "noise_level", "edge_density", "dynamic_range"]:
        if prop not in summary:
            continue
        s = summary[prop]
        print(
            f"{prop:<16} {s['mean']:>8.4f} {s['std']:>8.4f} "
            f"{s['min']:>8.4f} {s['max']:>8.4f} {s['median']:>8.4f}"
        )
    print("=" * 70)


def write_csv(
    filepath: Path,
    records: list[Any],
    fieldnames: list[str],
) -> None:
    """Write dataclass records to CSV.

    :param filepath: Output CSV path.
    :type filepath: Path
    :param records: List of dataclass instances.
    :type records: list
    :param fieldnames: CSV column names (must match dataclass fields).
    :type fieldnames: list[str]
    """
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with open(filepath, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in records:
            writer.writerow(asdict(r))
    log.info("Wrote %d rows to %s", len(records), filepath)


def write_json(filepath: Path, data: dict[str, Any]) -> None:
    """Write a dictionary to a JSON file.

    :param filepath: Output JSON path.
    :type filepath: Path
    :param data: Data to serialize.
    :type data: dict[str, Any]
    """
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    log.info("Wrote %s", filepath)


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------


def run_statistics(args: argparse.Namespace) -> dict[str, Any]:
    """Run the full statistics pipeline.

    :param args: Parsed command-line arguments.
    :type args: argparse.Namespace
    :return: Complete results dictionary.
    :rtype: dict[str, Any]
    """
    _import_modules()

    # --- Load ground truth ---
    gt_paths = [Path(p) for p in args.ground_truth]
    ground_truth = load_ground_truth(gt_paths)
    if not ground_truth:
        log.error("No ground truth loaded. Aborting.")
        sys.exit(1)

    gt_image_names = {e.image_name for e in ground_truth}
    total_gt_segs = sum(len(e.segments) for e in ground_truth)
    log.info(
        "Ground truth: %d segments across %d images", total_gt_segs, len(gt_image_names)
    )

    # Build per-image GT map
    gt_map: dict[str, list[Any]] = {}
    for entry in ground_truth:
        gt_map.setdefault(entry.image_name, []).extend(entry.segments)

    # --- Load images ---
    image_dirs = [Path(d) for d in args.image_dir]
    images = load_images(image_dirs, gt_image_names, max_images=args.max_images)
    if not images:
        log.error("No images loaded. Check --image-dir paths.")
        sys.exit(1)
    log.info("Loaded %d images with ground truth", len(images))

    # --- Select detectors ---
    if args.detectors:
        det_names = set(args.detectors)
        detectors = [(n, c) for n, c in DETECTOR_REGISTRY if n in det_names]
        missing = det_names - {n for n, _ in detectors}
        if missing:
            log.error("Unknown detectors: %s", ", ".join(sorted(missing)))
            log.error(
                "Available: %s",
                ", ".join(n for n, _ in DETECTOR_REGISTRY),
            )
            sys.exit(1)
    else:
        detectors = list(DETECTOR_REGISTRY)

    # --- Select strategies ---
    if args.strategies:
        strategies = [s for s in args.strategies if s in STRATEGY_NAMES]
        invalid = set(args.strategies) - set(STRATEGY_NAMES)
        if invalid:
            log.warning("Unknown strategies ignored: %s", ", ".join(invalid))
    else:
        strategies = list(STRATEGY_NAMES)

    # --- Image analysis ---
    log.info("Analyzing image properties ...")
    img_props = analyze_images(images)
    prop_summary = aggregate_image_properties(img_props)
    print_image_property_summary(prop_summary)

    # --- Run detection ---
    total_start = time.time()
    all_stats = evaluate_all(
        detectors, strategies, images, gt_map, args.match_threshold
    )
    total_elapsed = time.time() - total_start
    log.info("Evaluation complete in %s", format_duration(total_elapsed))

    # --- Aggregate ---
    aggregates = aggregate(all_stats)
    print_summary_table(aggregates)

    # --- Build results dict ---
    results: dict[str, Any] = {
        "metadata": {
            "ground_truth_files": [str(p) for p in gt_paths],
            "num_images": len(images),
            "num_gt_segments": total_gt_segs,
            "match_threshold": args.match_threshold,
            "strategies": strategies,
            "detectors": [n for n, _ in detectors],
            "total_duration_seconds": round(total_elapsed, 1),
        },
        "image_properties": prop_summary,
        "aggregates": [asdict(a) for a in aggregates],
    }

    # --- Write output ---
    if args.output_dir:
        out = Path(args.output_dir)

        # Per-image CSV
        write_csv(
            out / "per_image_results.csv",
            all_stats,
            list(ImageStats.__dataclass_fields__.keys()),
        )

        # Image properties CSV
        write_csv(
            out / "image_properties.csv",
            img_props,
            list(ImagePropertyRecord.__dataclass_fields__.keys()),
        )

        # Aggregate CSV
        write_csv(
            out / "aggregate_results.csv",
            aggregates,
            list(AggregateStats.__dataclass_fields__.keys()),
        )

        # Full JSON
        write_json(out / "statistics.json", results)

    return results


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    """Entry point for the detector statistics script."""
    parser = argparse.ArgumentParser(
        description="Compute detection statistics across benchmark datasets.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--ground-truth",
        required=True,
        nargs="+",
        help="Path(s) to ground truth CSV file(s).",
    )
    parser.add_argument(
        "--image-dir",
        required=True,
        nargs="+",
        help="Path(s) to image directories.",
    )
    parser.add_argument(
        "--detectors",
        nargs="*",
        default=None,
        help="Detector names to evaluate (default: all). "
        f"Available: {', '.join(n for n, _ in DETECTOR_REGISTRY)}",
    )
    parser.add_argument(
        "--strategies",
        nargs="*",
        default=None,
        help="Parameter strategies to test (default: all). "
        f"Available: {', '.join(STRATEGY_NAMES)}",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=0,
        help="Maximum number of images to use (0 = all, default: 0).",
    )
    parser.add_argument(
        "--match-threshold",
        type=float,
        default=5.0,
        help="Endpoint distance threshold in pixels (default: 5.0).",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory for CSV/JSON output (default: print only).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging.",
    )

    args = parser.parse_args()
    if args.verbose:
        log.setLevel(logging.DEBUG)

    log.info("Detectors: %s", ", ".join(n for n, _ in DETECTOR_REGISTRY))
    if args.detectors:
        log.info("Selected: %s", ", ".join(args.detectors))

    run_statistics(args)


if __name__ == "__main__":
    mp.set_start_method("fork")
    main()
