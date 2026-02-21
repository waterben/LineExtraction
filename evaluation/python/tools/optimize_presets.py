#!/usr/bin/env python3
"""Optimize LSD detector parameters over image databases to determine presets.

This script runs parameter optimization for all 9 LSD detectors using
ground truth line segment annotations from the York Urban and/or Wireframe
datasets. For each detector, three presets are generated:

- **Fast** — optimized for maximum **Precision** (fewer, confident lines)
- **Balanced** — optimized for maximum **F1** (harmonic mean of P and R)
- **Accurate** — optimized for maximum **Recall** (find all lines)

The optimization uses ``RandomSearchStrategy`` with configurable sample counts
since grid search over 5–8 parameters is intractable.

Usage::

    # Quick test on one detector
    bazel run //evaluation/python:optimize_presets -- \\
        --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \\
        --image-dir resources/datasets/YorkUrban/images \\
        --detectors LsdFGioi --samples 50

    # Full optimization on all detectors
    bazel run //evaluation/python:optimize_presets -- \\
        --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \\
        --image-dir resources/datasets/YorkUrban/images \\
        --samples 300 --output resources/presets/lsd_presets.json

    # Multiple ground truth files (combined evaluation)
    bazel run //evaluation/python:optimize_presets -- \\
        --ground-truth resources/datasets/ground_truth/york_urban_gt.csv \\
                       resources/datasets/ground_truth/wireframe_gt.csv \\
        --image-dir resources/datasets/YorkUrban/images \\
                    resources/datasets/Wireframe/images \\
        --samples 300

Output:
    JSON file with optimized parameter presets per detector and profile.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Lazy imports for pybind11 modules (only available under Bazel or after build)
# ---------------------------------------------------------------------------

_le_algorithm = None
_le_geometry = None
_le_lsd = None


def _import_modules() -> None:
    """Import native pybind11 modules lazily.

    This avoids import errors when the script is analyzed by linters
    or run outside of the Bazel environment.
    """
    global _le_algorithm, _le_geometry, _le_lsd  # noqa: PLW0603
    if _le_algorithm is not None:
        return
    import le_algorithm
    import le_geometry
    import le_lsd

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
log = logging.getLogger("optimize_presets")


# ---------------------------------------------------------------------------
# Detector registry — search spaces and constructor mapping
# ---------------------------------------------------------------------------


@dataclass
class DetectorDef:
    """Definition of a detector and its optimizable parameter space.

    :param name: Human-readable detector name.
    :type name: str
    :param cls_name: Python class name in ``le_lsd`` (double-precision variant).
    :type cls_name: str
    :param params: List of ``(name, type, min, max, step)`` tuples defining
        the search space for each parameter.
    :type params: list[tuple]
    """

    name: str
    cls_name: str
    params: list[tuple[str, str, Any, Any, Any]] = field(default_factory=list)


# Each param tuple: (name, type, min, max, step)
# type: "float", "int", "bool"
DETECTOR_DEFS: list[DetectorDef] = [
    DetectorDef(
        name="LsdCC",
        cls_name="LsdCC_f64",
        params=[
            ("nms_th_low", "float", 0.002, 0.020, 0.002),
            ("nms_th_high", "float", 0.005, 0.050, 0.005),
            ("edge_min_pixels", "int", 3, 20, 2),
            ("edge_max_gap", "int", 0, 5, 1),
            ("split_error_distance", "float", 0.5, 5.0, 0.5),
        ],
    ),
    DetectorDef(
        name="LsdCP",
        cls_name="LsdCP_f64",
        params=[
            ("nms_th_low", "float", 0.002, 0.020, 0.002),
            ("nms_th_high", "float", 0.005, 0.050, 0.005),
            ("edge_min_pixels", "int", 3, 20, 2),
            ("edge_max_gap", "int", 0, 5, 1),
            ("split_error_distance", "float", 0.5, 5.0, 0.5),
            ("edge_pattern_tolerance", "int", 1, 5, 1),
        ],
    ),
    DetectorDef(
        name="LsdBurns",
        cls_name="LsdBurns_f64",
        params=[
            ("nms_th_low", "float", 0.002, 0.020, 0.002),
            ("nms_th_high", "float", 0.005, 0.050, 0.005),
            ("edge_min_pixels", "int", 3, 20, 2),
            ("edge_partitions", "int", 4, 24, 4),
        ],
    ),
    DetectorDef(
        name="LsdFBW",
        cls_name="LsdFBW_f64",
        params=[
            ("nms_th_low", "float", 0.002, 0.020, 0.002),
            ("nms_th_high", "float", 0.005, 0.050, 0.005),
            ("edge_min_pixels", "int", 0, 20, 4),
            ("angle_th", "float", 10.0, 45.0, 5.0),
        ],
    ),
    DetectorDef(
        name="LsdFGioi",
        cls_name="LsdFGioi_f64",
        params=[
            ("quant_error", "float", 0.5, 5.0, 0.5),
            ("angle_th", "float", 10.0, 45.0, 5.0),
            ("log_eps", "float", -5.0, 5.0, 1.0),
            ("density_th", "float", 0.3, 1.0, 0.1),
            ("bins", "int", 256, 2048, 256),
        ],
    ),
    DetectorDef(
        name="LsdEDLZ",
        cls_name="LsdEDLZ_f64",
        params=[
            ("grad_th", "float", 5.0, 30.0, 5.0),
            ("anchor_th", "float", 1.0, 5.0, 1.0),
            ("scan_int", "int", 1, 4, 1),
            ("min_len", "int", 5, 30, 5),
            ("fit_error", "float", 0.5, 5.0, 0.5),
            ("validate", "bool", 0, 1, 1),
        ],
    ),
    DetectorDef(
        name="LsdEL",
        cls_name="LsdEL_f64",
        params=[
            ("nms_th_low", "float", 0.002, 0.020, 0.002),
            ("nms_th_high", "float", 0.005, 0.050, 0.005),
            ("edge_min_pixels", "int", 3, 20, 2),
            ("split_error_distance", "float", 0.5, 5.0, 0.5),
        ],
    ),
    DetectorDef(
        name="LsdEP",
        cls_name="LsdEP_f64",
        params=[
            ("nms_th_low", "float", 0.002, 0.020, 0.002),
            ("nms_th_high", "float", 0.005, 0.050, 0.005),
            ("edge_min_pixels", "int", 3, 20, 2),
            ("split_error_distance", "float", 0.5, 5.0, 0.5),
        ],
    ),
    DetectorDef(
        name="LsdHoughP",
        cls_name="LsdHoughP_f64",
        params=[
            ("hough_rho", "float", 0.5, 3.0, 0.5),
            ("hough_vote_th", "int", 50, 300, 50),
            ("edge_min_len", "float", 5.0, 30.0, 5.0),
            ("edge_max_gap", "float", 1.0, 10.0, 2.0),
        ],
    ),
]

# Preset profiles: (profile_name, OptimMetric enum value)
PRESET_PROFILES: list[tuple[str, str]] = [
    ("fast", "PRECISION"),
    ("balanced", "F1"),
    ("accurate", "RECALL"),
]


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------


def build_param_ranges(
    detector_def: DetectorDef,
) -> list[Any]:
    """Build a list of ``ParamRange`` objects from a detector definition.

    :param detector_def: Detector definition with parameter specifications.
    :type detector_def: DetectorDef
    :return: List of ParamRange objects for the optimizer.
    :rtype: list[le_algorithm.ParamRange]
    """
    ranges = []
    for name, ptype, min_val, max_val, step in detector_def.params:
        if ptype == "int":
            ranges.append(
                _le_algorithm.ParamRange.make_int(
                    name, int(min_val), int(max_val), int(step)
                )
            )
        elif ptype == "bool":
            ranges.append(_le_algorithm.ParamRange.make_bool(name))
        else:
            ranges.append(
                _le_algorithm.ParamRange(
                    name, float(min_val), float(max_val), float(step)
                )
            )
    return ranges


def make_detect_fn(
    detector_cls: type,
) -> Any:
    """Create a detection function for the optimizer.

    Returns a callable matching the signature expected by ``ParamOptimizer``:
    ``(src: np.ndarray, params: list[dict]) -> list[LineSegment_f64]``

    :param detector_cls: The LSD detector class (double-precision variant).
    :type detector_cls: type
    :return: Detection function suitable for ParamOptimizer.optimize().
    :rtype: callable
    """

    def detect_fn(
        src: np.ndarray,
        params: list[dict[str, Any]],
    ) -> list[Any]:
        """Detect line segments with the given parameters.

        :param src: Grayscale input image (uint8).
        :type src: np.ndarray
        :param params: Parameter configuration as list of name/value dicts.
        :type params: list[dict[str, Any]]
        :return: Detected line segments.
        :rtype: list[LineSegment_f64]
        """
        det = detector_cls()
        for p in params:
            name = p["name"]
            value = p["value"]
            if isinstance(value, bool):
                det.set_int(name, int(value))
            elif isinstance(value, int):
                det.set_int(name, value)
            elif isinstance(value, float):
                det.set_float(name, value)
        det.detect(src)
        return det.line_segments()

    return detect_fn


def load_images(
    image_dirs: list[Path],
    gt_image_names: set[str],
    max_images: int = 0,
) -> list[tuple[str, np.ndarray]]:
    """Load images that have corresponding ground truth entries.

    :param image_dirs: Directories to search for images.
    :type image_dirs: list[Path]
    :param gt_image_names: Set of image filenames present in ground truth.
    :type gt_image_names: set[str]
    :param max_images: Maximum number of images to load (0 = all).
    :type max_images: int
    :return: List of (image_name, grayscale_image) tuples.
    :rtype: list[tuple[str, np.ndarray]]
    """
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
            img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
            if img is None:
                log.warning("Failed to read image: %s", img_path)
                continue
            images.append((name, img))
            seen.add(name)
            if 0 < max_images <= len(images):
                break
        if 0 < max_images <= len(images):
            break

    return images


def load_ground_truth(
    gt_paths: list[Path],
) -> list[Any]:
    """Load ground truth entries from CSV files.

    :param gt_paths: Paths to ground truth CSV files.
    :type gt_paths: list[Path]
    :return: Combined list of GroundTruthEntry objects.
    :rtype: list[le_algorithm.GroundTruthEntry]
    """
    all_entries: list[Any] = []
    for gt_path in gt_paths:
        if not gt_path.is_file():
            log.warning("Ground truth file not found: %s", gt_path)
            continue
        entries = _le_algorithm.GroundTruthLoader.load_csv(str(gt_path))
        log.info("Loaded %d ground truth entries from %s", len(entries), gt_path.name)
        all_entries.extend(entries)
    return all_entries


def format_duration(seconds: float) -> str:
    """Format a duration in seconds to a human-readable string.

    :param seconds: Duration in seconds.
    :type seconds: float
    :return: Formatted string like ``"2m 30s"`` or ``"1h 15m"``.
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


def params_to_dict(params: list[dict[str, Any]]) -> dict[str, Any]:
    """Convert a param config list to a simple name→value dict.

    :param params: Parameter configuration from optimizer results.
    :type params: list[dict[str, Any]]
    :return: Dictionary mapping parameter names to values.
    :rtype: dict[str, Any]
    """
    result: dict[str, Any] = {}
    for p in params:
        val = p["value"]
        # Round floats for cleaner output
        if isinstance(val, float):
            val = round(val, 6)
        result[p["name"]] = val
    return result


# ---------------------------------------------------------------------------
# Main optimization
# ---------------------------------------------------------------------------


def optimize_detector(
    detector_def: DetectorDef,
    metric: Any,
    metric_name: str,
    images: list[tuple[str, np.ndarray]],
    ground_truth: list[Any],
    num_samples: int,
    match_threshold: float,
    seed: int,
) -> dict[str, Any]:
    """Run parameter optimization for a single detector and metric.

    :param detector_def: Detector definition with search space.
    :type detector_def: DetectorDef
    :param metric: OptimMetric enum value (F1, PRECISION, RECALL).
    :type metric: le_algorithm.OptimMetric
    :param metric_name: Human-readable metric name for logging.
    :type metric_name: str
    :param images: List of (name, image) tuples.
    :type images: list[tuple[str, np.ndarray]]
    :param ground_truth: Ground truth entries.
    :type ground_truth: list[le_algorithm.GroundTruthEntry]
    :param num_samples: Number of random parameter configurations to try.
    :type num_samples: int
    :param match_threshold: Endpoint distance threshold in pixels.
    :type match_threshold: float
    :param seed: Random seed for reproducibility.
    :type seed: int
    :return: Dictionary with optimization results.
    :rtype: dict[str, Any]
    """
    detector_cls = getattr(_le_lsd, detector_def.cls_name)
    space = build_param_ranges(detector_def)
    detect_fn = make_detect_fn(detector_cls)

    optimizer = _le_algorithm.ParamOptimizer(
        metric=metric,
        match_threshold=match_threshold,
        verbose=False,
    )
    strategy = _le_algorithm.RandomSearchStrategy(num_samples=num_samples, seed=seed)

    start_time = time.time()
    last_report = [start_time]

    def progress_callback(step: int, total: int, best_score: float) -> bool:
        """Report optimization progress periodically.

        :param step: Current step number.
        :type step: int
        :param total: Total number of steps.
        :type total: int
        :param best_score: Best score found so far.
        :type best_score: float
        :return: True to continue, False to cancel.
        :rtype: bool
        """
        now = time.time()
        if now - last_report[0] >= 10.0 or step == total:
            elapsed = now - start_time
            rate = step / elapsed if elapsed > 0 else 0
            eta = (total - step) / rate if rate > 0 else 0
            log.info(
                "  [%s/%s] %d/%d configs, best %s=%.4f, ETA %s",
                detector_def.name,
                metric_name,
                step,
                total,
                metric_name,
                best_score,
                format_duration(eta),
            )
            last_report[0] = now
        return True

    result = optimizer.optimize(
        strategy, space, images, ground_truth, detect_fn, progress_callback
    )

    elapsed = time.time() - start_time
    log.info(
        "  [%s/%s] Complete: best %s=%.4f (%d configs in %s)",
        detector_def.name,
        metric_name,
        metric_name,
        result.best_score,
        result.total_configs,
        format_duration(elapsed),
    )

    return {
        "params": params_to_dict(result.best_params),
        "score": round(result.best_score, 6),
        "configs_evaluated": result.total_configs,
        "duration_seconds": round(elapsed, 1),
    }


def run_optimization(args: argparse.Namespace) -> dict[str, Any]:
    """Run the full optimization pipeline.

    :param args: Parsed command-line arguments.
    :type args: argparse.Namespace
    :return: Complete results dictionary ready for JSON serialization.
    :rtype: dict[str, Any]
    """
    _import_modules()

    # Load ground truth
    gt_paths = [Path(p) for p in args.ground_truth]
    ground_truth = load_ground_truth(gt_paths)
    if not ground_truth:
        log.error("No ground truth data loaded. Aborting.")
        sys.exit(1)

    gt_image_names = {entry.image_name for entry in ground_truth}
    log.info(
        "Ground truth: %d entries across %d images",
        sum(len(entry.segments) for entry in ground_truth),
        len(gt_image_names),
    )

    # Load images
    image_dirs = [Path(d) for d in args.image_dir]
    images = load_images(image_dirs, gt_image_names, max_images=args.max_images)
    if not images:
        log.error("No images loaded. Check --image-dir paths.")
        sys.exit(1)
    log.info("Loaded %d images with ground truth", len(images))

    # Select detectors
    if args.detectors:
        detector_names = set(args.detectors)
        defs = [d for d in DETECTOR_DEFS if d.name in detector_names]
        missing = detector_names - {d.name for d in defs}
        if missing:
            log.error("Unknown detectors: %s", ", ".join(sorted(missing)))
            log.error("Available: %s", ", ".join(d.name for d in DETECTOR_DEFS))
            sys.exit(1)
    else:
        defs = DETECTOR_DEFS

    # Map metric names to enum
    metric_map = {
        "F1": _le_algorithm.OptimMetric.F1,
        "PRECISION": _le_algorithm.OptimMetric.PRECISION,
        "RECALL": _le_algorithm.OptimMetric.RECALL,
    }

    # Run optimization
    results: dict[str, Any] = {
        "metadata": {
            "ground_truth_files": [str(p) for p in gt_paths],
            "num_images": len(images),
            "num_gt_segments": sum(len(e.segments) for e in ground_truth),
            "num_samples": args.samples,
            "match_threshold": args.match_threshold,
            "seed": args.seed,
        },
        "detectors": {},
    }

    total_runs = len(defs) * len(PRESET_PROFILES)
    current_run = 0
    total_start = time.time()

    for det_def in defs:
        log.info("=" * 60)
        log.info("Optimizing %s (%d parameters)", det_def.name, len(det_def.params))
        log.info("=" * 60)

        det_results: dict[str, Any] = {}

        for profile_name, metric_name in PRESET_PROFILES:
            current_run += 1
            log.info(
                "[%d/%d] %s — %s (target: %s)",
                current_run,
                total_runs,
                det_def.name,
                profile_name,
                metric_name,
            )

            result = optimize_detector(
                detector_def=det_def,
                metric=metric_map[metric_name],
                metric_name=metric_name,
                images=images,
                ground_truth=ground_truth,
                num_samples=args.samples,
                match_threshold=args.match_threshold,
                seed=args.seed,
            )

            det_results[profile_name] = result

        results["detectors"][det_def.name] = det_results

    total_elapsed = time.time() - total_start
    results["metadata"]["total_duration_seconds"] = round(total_elapsed, 1)
    log.info("=" * 60)
    log.info("All optimizations complete in %s", format_duration(total_elapsed))
    log.info("=" * 60)

    return results


def print_summary(results: dict[str, Any]) -> None:
    """Print a human-readable summary of optimization results.

    :param results: Results dictionary from run_optimization.
    :type results: dict[str, Any]
    """
    print("\n" + "=" * 72)
    print("PARAMETER PRESET SUMMARY")
    print("=" * 72)

    for det_name, det_results in results["detectors"].items():
        print(f"\n{det_name}:")
        for profile_name, profile_data in det_results.items():
            score = profile_data["score"]
            params = profile_data["params"]
            param_str = ", ".join(f"{k}={v}" for k, v in params.items())
            print(f"  {profile_name:>10}: score={score:.4f}  {param_str}")

    meta = results["metadata"]
    print(f"\nTotal time: {format_duration(meta['total_duration_seconds'])}")
    print(f"Images: {meta['num_images']}, GT segments: {meta['num_gt_segments']}")
    print(f"Samples per run: {meta['num_samples']}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    """Entry point for the preset optimization script."""
    parser = argparse.ArgumentParser(
        description="Optimize LSD detector parameters to generate presets.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--ground-truth",
        required=True,
        nargs="+",
        help="Path(s) to ground truth CSV file(s)",
    )
    parser.add_argument(
        "--image-dir",
        required=True,
        nargs="+",
        help="Path(s) to image directories",
    )
    parser.add_argument(
        "--detectors",
        nargs="*",
        default=None,
        help="Detector names to optimize (default: all). "
        f"Available: {', '.join(d.name for d in DETECTOR_DEFS)}",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=300,
        help="Number of random parameter samples per detector (default: 300)",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=0,
        help="Maximum number of images to use (0 = all, default: 0)",
    )
    parser.add_argument(
        "--match-threshold",
        type=float,
        default=5.0,
        help="Endpoint distance threshold in pixels (default: 5.0)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducibility (default: 42)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="lsd_presets.json",
        help="Output JSON file path (default: lsd_presets.json)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging",
    )

    args = parser.parse_args()

    if args.verbose:
        log.setLevel(logging.DEBUG)

    # Print available detectors if none specified
    log.info("Available detectors: %s", ", ".join(d.name for d in DETECTOR_DEFS))
    if args.detectors:
        log.info("Selected detectors: %s", ", ".join(args.detectors))

    # Run optimization
    results = run_optimization(args)

    # Write output
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    log.info("Results written to %s", output_path)

    # Print summary
    print_summary(results)


if __name__ == "__main__":
    main()
