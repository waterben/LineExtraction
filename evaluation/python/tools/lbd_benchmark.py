#!/usr/bin/env python3
"""Benchmark LBD descriptor matching across HPatches sequences.

Evaluates both the internal float-based LBD descriptor and OpenCV's
binary LBD (``BinaryDescriptor``) on curated HPatches viewpoint sequences.
Each pair is matched with cross-check + ratio test, and accuracy is
measured via ground-truth homography reprojection.

Usage::

    # Run on all curated architectural sequences, difficulty 2-6
    bazel run //evaluation/python:lbd_benchmark

    # Specify difficulty range and max dimension
    bazel run //evaluation/python:lbd_benchmark -- \\
        --idx-range 2 4 --max-dim 600

    # Custom numBand / widthBand
    bazel run //evaluation/python:lbd_benchmark -- \\
        --num-band 11 --width-band 7

Output:
    Per-sequence and per-difficulty accuracy tables printed to stdout.
"""

from __future__ import annotations

import argparse
import io
import logging
import os
import sys
import time
from typing import Any

import numpy as np
from PIL import Image

# Silence OpenCV parallel backend warnings on stderr
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

# ---------------------------------------------------------------------------
# Lazy imports for pybind11 modules
# ---------------------------------------------------------------------------

_le_lsd: Any = None
_le_lfd: Any = None


def _import_modules() -> None:
    """Import native pybind11 modules lazily.

    Avoids import errors when the script is analyzed by linters or
    run outside of the Bazel environment.
    """
    global _le_lsd, _le_lfd  # noqa: PLW0603
    if _le_lsd is not None:
        return
    import le_lsd  # type: ignore[import-untyped]
    import le_lfd  # type: ignore[import-untyped]

    _le_lsd = le_lsd
    _le_lfd = le_lfd


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("lbd_benchmark")


# ---------------------------------------------------------------------------
# Image helpers
# ---------------------------------------------------------------------------


def _downscale(img: np.ndarray, max_dim: int = 800) -> tuple[np.ndarray, float]:
    """Downscale image so the longest side <= *max_dim*.

    :param img: Grayscale input image.
    :type img: numpy.ndarray
    :param max_dim: Maximum dimension in pixels.
    :type max_dim: int
    :return: ``(resized_image, scale_factor)``.
    :rtype: tuple[numpy.ndarray, float]
    """
    h, w = img.shape[:2]
    if max(h, w) <= max_dim:
        return img, 1.0
    scale = max_dim / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = Image.fromarray(img).resize((new_w, new_h), Image.LANCZOS)
    return np.array(resized), scale


# ---------------------------------------------------------------------------
# Evaluation core
# ---------------------------------------------------------------------------


def _match_error(
    seg_r: Any,
    seg_t: Any,
    H: np.ndarray,
) -> float:
    """Reprojection error for a segment match.

    :param seg_r: Reference line segment.
    :param seg_t: Target line segment.
    :param H: 3x3 ground-truth homography (ref -> tgt).
    :type H: numpy.ndarray
    :return: Euclidean distance between projected ref center and tgt center.
    :rtype: float
    """
    cx, cy = seg_r.center_point()
    pt_h = np.array([cx, cy, 1.0])
    proj = H @ pt_h
    proj = proj[:2] / proj[2]
    tx, ty = seg_t.center_point()
    return float(np.linalg.norm(proj - np.array([tx, ty])))


def _classify_matches(
    match_list: list[tuple[int, Any]],
    segs_r: list[Any],
    segs_t: list[Any],
    H: np.ndarray,
    thresh: float = 15.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Classify matches as correct/incorrect via reprojection error.

    :param match_list: List of ``(ref_idx, DescriptorMatch)`` tuples.
    :param segs_r: Reference line segments.
    :param segs_t: Target line segments.
    :param H: Ground-truth homography.
    :param thresh: Pixel threshold for "correct".
    :return: ``(errors, is_correct)`` arrays.
    :rtype: tuple[numpy.ndarray, numpy.ndarray]
    """
    errors = []
    correct = []
    for ref_idx, m in match_list:
        err = _match_error(segs_r[ref_idx], segs_t[m.match_idx], H)
        errors.append(err)
        correct.append(err < thresh)
    return np.array(errors), np.array(correct)


def evaluate_pair(
    images: Any,
    seq_name: str,
    target_idx: int,
    *,
    max_dim: int = 800,
    min_len: int = 15,
    ratio_thresh: float = 0.85,
    gt_threshold: float = 15.0,
    top_k: int = 30,
    num_band: int = 9,
    width_band: int = 7,
) -> dict[str, Any] | None:
    """Evaluate both LBD variants on a single HPatches pair.

    :param images: ``TestImages`` instance.
    :param seq_name: HPatches sequence name (e.g. ``"v_london"``).
    :type seq_name: str
    :param target_idx: Target image index (2-6).
    :type target_idx: int
    :param max_dim: Downscale long side to this.
    :type max_dim: int
    :param min_len: Minimum segment length in pixels.
    :type min_len: int
    :param ratio_thresh: Lowe's ratio test threshold.
    :type ratio_thresh: float
    :param gt_threshold: Reprojection error threshold (pixels).
    :type gt_threshold: float
    :param top_k: Number of top matches to evaluate.
    :type top_k: int
    :param num_band: LBD ``numBand`` parameter.
    :type num_band: int
    :param width_band: LBD ``widthBand`` parameter.
    :type width_band: int
    :return: Dict with accuracy metrics, or ``None`` if unavailable.
    :rtype: dict or None
    """
    _import_modules()
    try:
        ref_p, tgt_p, H = images.hpatches_pair(seq_name, target_idx)
    except FileNotFoundError as exc:
        log.debug("Pair not found: %s idx=%d: %s", seq_name, target_idx, exc)
        return None

    ref = np.array(Image.open(ref_p).convert("L"))
    tgt = np.array(Image.open(tgt_p).convert("L"))
    ref, sr = _downscale(ref, max_dim)
    tgt, st = _downscale(tgt, max_dim)
    H = np.diag([st, st, 1.0]) @ H @ np.linalg.inv(np.diag([sr, sr, 1.0]))

    det_r = _le_lsd.LsdCC()
    det_r.detect(ref)
    segs_r = det_r.line_segments()
    det_t = _le_lsd.LsdCC()
    det_t.detect(tgt)
    segs_t = det_t.line_segments()

    if len(segs_r) < 5 or len(segs_t) < 5:
        return None

    # --- Our LBD (float, L2) ---
    gx_r = det_r.image_data()[0].astype(np.float32)
    gy_r = det_r.image_data()[1].astype(np.float32)
    gx_t = det_t.image_data()[0].astype(np.float32)
    gy_t = det_t.image_data()[1].astype(np.float32)

    t0 = time.perf_counter()
    d_r = _le_lfd.FdcLBD(
        gx_r, gy_r, num_band=num_band, width_band=width_band
    ).create_list(segs_r)
    d_t = _le_lfd.FdcLBD(
        gx_t, gy_t, num_band=num_band, width_band=width_band
    ).create_list(segs_t)
    t_lbd_desc = time.perf_counter() - t0

    t0 = time.perf_counter()
    fwd = _le_lfd.BruteForceLBD()
    fwd.train(d_r, d_t)
    fwd_best = fwd.best()
    fwd_knn = fwd.knn(2)
    bwd = _le_lfd.BruteForceLBD()
    bwd.train(d_t, d_r)
    bwd_best = bwd.best()
    t_lbd_match = time.perf_counter() - t0

    lbd_matches: list[tuple[int, Any]] = []
    for i, m in enumerate(fwd_best):
        if np.isnan(m.distance):
            continue
        if segs_r[i].length < min_len or segs_t[m.match_idx].length < min_len:
            continue
        j = m.match_idx
        if j >= len(bwd_best) or bwd_best[j].match_idx != i:
            continue
        nn0, nn1 = fwd_knn[2 * i], fwd_knn[2 * i + 1]
        if np.isnan(nn1.distance) or nn1.distance <= 0:
            continue
        if nn0.distance / nn1.distance >= ratio_thresh:
            continue
        lbd_matches.append((i, m))
    lbd_matches.sort(key=lambda p: p[1].distance)

    # --- OpenCV LBD (binary, Hamming) ---
    _se = sys.stderr
    sys.stderr = io.StringIO()
    try:
        t0 = time.perf_counter()
        od_r = _le_lfd.FdcOpenCVLBD(ref).create_list(segs_r)
        od_t = _le_lfd.FdcOpenCVLBD(tgt).create_list(segs_t)
        t_ocv_desc = time.perf_counter() - t0
    finally:
        sys.stderr = _se

    t0 = time.perf_counter()
    ofwd = _le_lfd.BruteForceOpenCVLBD()
    ofwd.train(od_r, od_t)
    ofwd_best = ofwd.best()
    obwd = _le_lfd.BruteForceOpenCVLBD()
    obwd.train(od_t, od_r)
    obwd_best = obwd.best()
    t_ocv_match = time.perf_counter() - t0

    ocv_matches: list[tuple[int, Any]] = []
    for i, m in enumerate(ofwd_best):
        if np.isnan(m.distance) or m.distance > 1e10:
            continue
        if segs_r[i].length < min_len or segs_t[m.match_idx].length < min_len:
            continue
        j = m.match_idx
        if j < len(obwd_best) and obwd_best[j].match_idx == i:
            ocv_matches.append((i, m))
    ocv_matches.sort(key=lambda p: p[1].distance)

    # --- Evaluate ---
    def _accuracy(match_list: list[tuple[int, Any]], k: int) -> tuple[float, int, int]:
        if not match_list:
            return 0.0, 0, 0
        errs, ok = _classify_matches(match_list, segs_r, segs_t, H, gt_threshold)
        n = min(k, len(match_list))
        return float(ok[:n].sum()) / n * 100, int(ok[:n].sum()), n

    lbd_acc, lbd_ok, lbd_n = _accuracy(lbd_matches, top_k)
    ocv_acc, ocv_ok, ocv_n = _accuracy(ocv_matches, top_k)

    return {
        "seq": seq_name,
        "idx": target_idx,
        "lbd_acc": lbd_acc,
        "lbd_ok": lbd_ok,
        "lbd_n": lbd_n,
        "lbd_matches": len(lbd_matches),
        "ocv_acc": ocv_acc,
        "ocv_ok": ocv_ok,
        "ocv_n": ocv_n,
        "ocv_matches": len(ocv_matches),
        "t_lbd_desc": t_lbd_desc,
        "t_lbd_match": t_lbd_match,
        "t_ocv_desc": t_ocv_desc,
        "t_ocv_match": t_ocv_match,
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _build_parser() -> argparse.ArgumentParser:
    """Build the argument parser.

    :return: Configured argument parser.
    :rtype: argparse.ArgumentParser
    """
    p = argparse.ArgumentParser(
        description="Benchmark LBD descriptor matching on HPatches.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--idx-range",
        type=int,
        nargs=2,
        default=[2, 6],
        metavar=("MIN", "MAX"),
        help="Homography difficulty range (default: 2 6).",
    )
    p.add_argument(
        "--max-dim",
        type=int,
        default=800,
        help="Downscale longest side to this (default: 800).",
    )
    p.add_argument(
        "--min-len",
        type=int,
        default=15,
        help="Minimum segment length in pixels (default: 15).",
    )
    p.add_argument(
        "--ratio-thresh",
        type=float,
        default=0.85,
        help="Lowe's ratio test threshold (default: 0.85).",
    )
    p.add_argument(
        "--gt-threshold",
        type=float,
        default=15.0,
        help="Reprojection error threshold in pixels (default: 15).",
    )
    p.add_argument(
        "--top-k",
        type=int,
        default=30,
        help="Number of top matches to evaluate (default: 30).",
    )
    p.add_argument(
        "--num-band",
        type=int,
        default=9,
        help="LBD numBand parameter (default: 9).",
    )
    p.add_argument(
        "--width-band",
        type=int,
        default=7,
        help="LBD widthBand parameter (default: 7).",
    )
    return p


def main() -> None:
    """Run the LBD benchmark."""
    from collections import defaultdict

    from lsfm.data import HPATCHES_LINE_SEQUENCES, TestImages

    _import_modules()

    args = _build_parser().parse_args()
    idx_min, idx_max = args.idx_range

    images = TestImages()

    log.info(
        "LBD Benchmark — numBand=%d, widthBand=%d, idx %d-%d",
        args.num_band,
        args.width_band,
        idx_min,
        idx_max,
    )

    results: list[dict[str, Any]] = []
    t_start = time.perf_counter()

    for seq_name in HPATCHES_LINE_SEQUENCES:
        row = ""
        for idx in range(idx_min, idx_max + 1):
            try:
                r = evaluate_pair(
                    images,
                    seq_name,
                    idx,
                    max_dim=args.max_dim,
                    min_len=args.min_len,
                    ratio_thresh=args.ratio_thresh,
                    gt_threshold=args.gt_threshold,
                    top_k=args.top_k,
                    num_band=args.num_band,
                    width_band=args.width_band,
                )
            except Exception:
                log.exception("ERROR on %s idx=%d", seq_name, idx)
                r = None
            if r is not None:
                results.append(r)
                row += "." if r["lbd_acc"] >= 50 else "x"
            else:
                row += "-"
        print(f"{row}  {seq_name}")

    elapsed = time.perf_counter() - t_start
    print(f"\n{len(results)} pairs evaluated in {elapsed:.1f}s.")

    if not results:
        print(
            "No results — ensure HPatches is downloaded: "
            "./tools/scripts/setup_hpatches.sh"
        )
        return

    # --- Per-sequence summary ---
    by_seq: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for r in results:
        by_seq[r["seq"]].append(r)

    print(f"\n{'Sequence':<18} {'LBD mean':>8}  {'OCV mean':>8}  {'pairs':>5}")
    print("-" * 44)
    for name in HPATCHES_LINE_SEQUENCES:
        rs = by_seq.get(name, [])
        if rs:
            lm = np.mean([r["lbd_acc"] for r in rs])
            om = np.mean([r["ocv_acc"] for r in rs])
            print(f"{name:<18} {lm:>7.1f}%  {om:>7.1f}%  {len(rs):>5}")

    # --- Overall ---
    avg_lbd = np.mean([r["lbd_acc"] for r in results])
    avg_ocv = np.mean([r["ocv_acc"] for r in results])
    med_lbd = np.median([r["lbd_acc"] for r in results])
    med_ocv = np.median([r["ocv_acc"] for r in results])

    print(f"\n{'=' * 44}")
    print(f"Overall (n={len(results)} pairs):")
    print(f"  Our LBD:    mean={avg_lbd:.1f}%  median={med_lbd:.1f}%")
    print(f"  OpenCV LBD: mean={avg_ocv:.1f}%  median={med_ocv:.1f}%")

    # --- Per difficulty level ---
    print("\nPer difficulty level:")
    for idx in range(idx_min, idx_max + 1):
        sub = [r for r in results if r["idx"] == idx]
        if sub:
            a_lbd = np.mean([r["lbd_acc"] for r in sub])
            a_ocv = np.mean([r["ocv_acc"] for r in sub])
            print(f"  idx={idx}: LBD={a_lbd:.1f}%  OCV={a_ocv:.1f}%  (n={len(sub)})")

    # --- Runtime summary ---
    t_lbd_d = np.mean([r["t_lbd_desc"] for r in results]) * 1000
    t_lbd_m = np.mean([r["t_lbd_match"] for r in results]) * 1000
    t_ocv_d = np.mean([r["t_ocv_desc"] for r in results]) * 1000
    t_ocv_m = np.mean([r["t_ocv_match"] for r in results]) * 1000

    print("\nAverage runtime per pair (ms):")
    print(f"  {'':25s} {'Our LBD':>10s}  {'OpenCV LBD':>10s}")
    print(f"  {'-' * 50}")
    print(f"  {'Descriptor computation':<25s} {t_lbd_d:>8.1f}ms  {t_ocv_d:>8.1f}ms")
    print(f"  {'Matching (fwd+bwd+knn)':<25s} {t_lbd_m:>8.1f}ms  {t_ocv_m:>8.1f}ms")
    print(f"  {'Total':<25s} {t_lbd_d + t_lbd_m:>8.1f}ms  {t_ocv_d + t_ocv_m:>8.1f}ms")


if __name__ == "__main__":
    main()
