#!/usr/bin/env python3
"""Parameter sweep for LBD descriptor band configuration.

Evaluates different ``numBand`` / ``widthBand`` combinations on the curated
HPatches architectural subset to find the best descriptor parameterization.

Uses :func:`evaluation.python.tools.lbd_benchmark.evaluate_pair` for each
configuration and reports accuracy statistics.

Usage::

    # Default sweep (6 configs x 30 pairs each)
    bazel run //evaluation/python:lbd_param_sweep

    # Custom parameter grid and difficulty range
    bazel run //evaluation/python:lbd_param_sweep -- \\
        --params 9,7 7,5 11,7 13,9 --idx-range 2 6

    # Quick test on easy pairs only
    bazel run //evaluation/python:lbd_param_sweep -- \\
        --idx-range 2 3

Output:
    Table of accuracy per parameter combination, sorted by mean accuracy.
"""

from __future__ import annotations

import argparse
import logging
import os
import time

import numpy as np

from evaluation.python.tools.lbd_benchmark import _import_modules, evaluate_pair

# Silence OpenCV parallel backend warnings on stderr
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("lbd_param_sweep")

# Default parameter grid
DEFAULT_PARAMS: list[tuple[int, int]] = [
    (9, 7),  # default
    (7, 5),  # used in match_test.cpp
    (7, 7),
    (9, 5),
    (5, 5),
    (11, 7),
]


def _parse_param(s: str) -> tuple[int, int]:
    """Parse a ``numBand,widthBand`` string.

    :param s: Comma-separated pair, e.g. ``"9,7"``.
    :type s: str
    :return: ``(numBand, widthBand)``.
    :rtype: tuple[int, int]
    :raises argparse.ArgumentTypeError: If format is invalid.
    """
    parts = s.split(",")
    if len(parts) != 2:
        msg = f"Expected numBand,widthBand (e.g. 9,7), got '{s}'"
        raise argparse.ArgumentTypeError(msg)
    try:
        return int(parts[0]), int(parts[1])
    except ValueError as exc:
        msg = f"Non-integer value in '{s}': {exc}"
        raise argparse.ArgumentTypeError(msg) from exc


def _build_parser() -> argparse.ArgumentParser:
    """Build the argument parser.

    :return: Configured argument parser.
    :rtype: argparse.ArgumentParser
    """
    p = argparse.ArgumentParser(
        description="Parameter sweep for LBD descriptor band configuration.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--params",
        type=_parse_param,
        nargs="*",
        default=None,
        metavar="NB,WB",
        help=(
            "Parameter combinations as numBand,widthBand pairs "
            "(default: 9,7 7,5 7,7 9,5 5,5 11,7)."
        ),
    )
    p.add_argument(
        "--idx-range",
        type=int,
        nargs=2,
        default=[2, 4],
        metavar=("MIN", "MAX"),
        help="Difficulty range (default: 2 4, for reasonable runtime).",
    )
    p.add_argument(
        "--max-dim",
        type=int,
        default=800,
        help="Downscale longest side to this (default: 800).",
    )
    p.add_argument(
        "--top-k",
        type=int,
        default=30,
        help="Number of top matches to evaluate (default: 30).",
    )
    return p


def main() -> None:
    """Run the LBD parameter sweep."""
    from lsfm.data import HPATCHES_LINE_SEQUENCES, TestImages

    _import_modules()

    args = _build_parser().parse_args()
    param_grid = args.params if args.params else DEFAULT_PARAMS
    idx_min, idx_max = args.idx_range

    images = TestImages()

    log.info(
        "LBD Parameter Sweep — %d configs, idx %d-%d, %d sequences",
        len(param_grid),
        idx_min,
        idx_max,
        len(HPATCHES_LINE_SEQUENCES),
    )

    print(
        f"{'numBand':>7} {'widBand':>7}  {'mean':>6}  "
        f"{'median':>6}  {'n':>5}  {'time':>8}"
    )
    print("-" * 50)

    sweep_results: dict[tuple[int, int], list[float]] = {}
    t_start = time.perf_counter()

    for nb, wb in param_grid:
        accs: list[float] = []
        t0 = time.perf_counter()
        for seq_name in HPATCHES_LINE_SEQUENCES:
            for idx in range(idx_min, idx_max + 1):
                r = evaluate_pair(
                    images,
                    seq_name,
                    idx,
                    max_dim=args.max_dim,
                    top_k=args.top_k,
                    num_band=nb,
                    width_band=wb,
                )
                if r is not None:
                    accs.append(r["lbd_acc"])
        elapsed = time.perf_counter() - t0
        sweep_results[(nb, wb)] = accs
        if accs:
            print(
                f"{nb:>7} {wb:>7}  {np.mean(accs):>5.1f}%  "
                f"{np.median(accs):>5.1f}%  {len(accs):>5}  "
                f"{elapsed:>6.1f}s"
            )
        else:
            print(f"{nb:>7} {wb:>7}  {'N/A':>6}  {'N/A':>6}  {0:>5}  {elapsed:>6.1f}s")

    total_time = time.perf_counter() - t_start

    # Summary
    if sweep_results:
        best_cfg = max(
            sweep_results.items(),
            key=lambda x: np.mean(x[1]) if x[1] else 0,
        )
        print(
            f"\nBest config: numBand={best_cfg[0][0]}, "
            f"widthBand={best_cfg[0][1]} "
            f"(mean={np.mean(best_cfg[1]):.1f}%)"
        )
    print(f"Total time: {total_time:.1f}s")


if __name__ == "__main__":
    main()
