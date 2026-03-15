"""Evaluation metrics for detection, matching, and 3D reconstruction.

Wraps existing ``AccuracyMeasure`` from ``le_algorithm`` for 2D detection
evaluation.  Uses the shared :func:`~lsfm.synthetic.oracle.assign_detected_to_gt`
for matching evaluation.  Combines distance and angular error for 3D.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import le_geometry

    from lsfm.synthetic.ground_truth import SceneGroundTruth


@dataclass
class DetectionMetrics:
    """2D line detection evaluation results.

    :param precision: Precision score.
    :type precision: float
    :param recall: Recall score.
    :type recall: float
    :param f1: F1 score.
    :type f1: float
    :param true_positives: Number of true positive matches.
    :type true_positives: int
    :param false_positives: Number of false positive detections.
    :type false_positives: int
    :param false_negatives: Number of missed ground truth segments.
    :type false_negatives: int
    """

    precision: float
    recall: float
    f1: float
    true_positives: int
    false_positives: int
    false_negatives: int


@dataclass
class MatchMetrics:
    """Cross-view matching evaluation results.

    :param precision: Match precision (correct matches / total matches).
    :type precision: float
    :param recall: Match recall (correct matches / possible matches).
    :type recall: float
    :param f1: Match F1 score.
    :type f1: float
    :param n_correct: Number of correct matches.
    :type n_correct: int
    :param n_matches: Total number of matches produced.
    :type n_matches: int
    :param n_possible: Total number of possible GT correspondences.
    :type n_possible: int
    """

    precision: float
    recall: float
    f1: float
    n_correct: int
    n_matches: int
    n_possible: int


@dataclass
class ReconstructionMetrics:
    """3D reconstruction evaluation results.

    :param distance_mean: Mean 3D distance error.
    :type distance_mean: float
    :param distance_median: Median 3D distance error.
    :type distance_median: float
    :param angle_mean: Mean angular error in degrees.
    :type angle_mean: float
    :param angle_median: Median angular error in degrees.
    :type angle_median: float
    :param length_ratio_mean: Mean length ratio (reconstructed / GT).
    :type length_ratio_mean: float
    :param n_matched: Number of matched 3D segments.
    :type n_matched: int
    :param n_reconstructed: Total number of reconstructed segments.
    :type n_reconstructed: int
    :param n_gt: Total number of GT 3D segments.
    :type n_gt: int
    """

    distance_mean: float
    distance_median: float
    angle_mean: float
    angle_median: float
    length_ratio_mean: float
    n_matched: int
    n_reconstructed: int
    n_gt: int


def _ensure_f64_segments(
    segments: list[le_geometry.LineSegment_f64],
) -> list[le_geometry.LineSegment_f64]:
    """Convert float ``LineSegment`` objects to ``LineSegment_f64`` if needed.

    ``LsdCC`` (float) returns ``LineSegment`` (float precision) while
    ``AccuracyMeasure`` (double) expects ``LineSegment_f64``.  This helper
    transparently promotes float segments to double so that the two can
    interoperate.

    :param segments: Segments that may be float or double precision.
    :type segments: list[le_geometry.LineSegment_f64]
    :return: Segments guaranteed to be ``LineSegment_f64``.
    :rtype: list[le_geometry.LineSegment_f64]
    """
    import le_geometry as _lg

    if not segments:
        return segments
    if isinstance(segments[0], _lg.LineSegment_f64):
        return segments
    return [
        _lg.LineSegment_f64.from_endpoints(*s.start_point(), *s.end_point())
        for s in segments
    ]


def evaluate_detection(
    detected: list[le_geometry.LineSegment_f64],
    gt_segments: list[le_geometry.LineSegment_f64],
    threshold: float = 5.0,
) -> DetectionMetrics:
    """Evaluate 2D line detection using ``AccuracyMeasure``.

    :param detected: Detected 2D line segments (float or double precision).
    :type detected: list[le_geometry.LineSegment_f64]
    :param gt_segments: Ground truth 2D line segments.
    :type gt_segments: list[le_geometry.LineSegment_f64]
    :param threshold: Matching threshold in pixels.
    :type threshold: float
    :return: Detection metrics.
    :rtype: DetectionMetrics
    """
    import le_algorithm

    detected = _ensure_f64_segments(detected)
    gt_segments = _ensure_f64_segments(gt_segments)
    measure = le_algorithm.AccuracyMeasure(threshold=threshold)
    result = measure.evaluate(detected, gt_segments)
    return DetectionMetrics(
        precision=result.precision,
        recall=result.recall,
        f1=result.f1,
        true_positives=result.true_positives,
        false_positives=result.false_positives,
        false_negatives=result.false_negatives,
    )


def evaluate_matching(
    matches: list[tuple[int, int]],
    gt: SceneGroundTruth,
    detected_views: list[list[le_geometry.LineSegment_f64]],
    view_i: int,
    view_j: int,
    max_distance: float = 20.0,
) -> MatchMetrics:
    """Evaluate cross-view matching quality.

    Computes assignments internally via :func:`assign_detected_to_gt`,
    then checks how many produced matches correspond to correct GT
    correspondences.

    :param matches: Produced matches as ``(det_idx_i, det_idx_j)`` pairs.
    :type matches: list[tuple[int, int]]
    :param gt: Ground truth projection data.
    :type gt: SceneGroundTruth
    :param detected_views: Detected segments per view.
    :type detected_views: list[list[le_geometry.LineSegment_f64]]
    :param view_i: First view index.
    :type view_i: int
    :param view_j: Second view index.
    :type view_j: int
    :param max_distance: Maximum distance for GT assignment.
    :type max_distance: float
    :return: Matching metrics.
    :rtype: MatchMetrics
    """
    from lsfm.synthetic.oracle import assign_detected_to_gt

    # Assign detected to GT in each view
    assign_i = assign_detected_to_gt(
        detected_views[view_i],
        gt.segments_2d(view_i),
        max_distance=max_distance,
    )
    assign_j = assign_detected_to_gt(
        detected_views[view_j],
        gt.segments_2d(view_j),
        max_distance=max_distance,
    )

    parents_i = gt.parent_indices(view_i)
    parents_j = gt.parent_indices(view_j)

    # Count correct matches
    n_correct = 0
    for det_i, det_j in matches:
        gt_i = assign_i.get(det_i)
        gt_j = assign_j.get(det_j)
        if gt_i is None or gt_j is None:
            continue
        if parents_i[gt_i] == parents_j[gt_j]:
            n_correct += 1

    n_matches = len(matches)
    # Number of possible GT correspondences
    gt_corrs = gt.correspondences(view_i, view_j)
    n_possible = len(gt_corrs)

    precision = n_correct / max(n_matches, 1)
    recall = n_correct / max(n_possible, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-12)

    return MatchMetrics(
        precision=precision,
        recall=recall,
        f1=f1,
        n_correct=n_correct,
        n_matches=n_matches,
        n_possible=n_possible,
    )


def evaluate_reconstruction(
    segments_3d: list[le_geometry.LineSegment3_f64],
    gt_lines_3d: list[le_geometry.LineSegment3_f64],
    distance_threshold: float = 2.0,
) -> ReconstructionMetrics:
    """Evaluate 3D reconstruction quality.

    For each reconstructed segment, finds the nearest GT segment using
    combined endpoint distance and angular error.

    :param segments_3d: Reconstructed 3D line segments.
    :type segments_3d: list[le_geometry.LineSegment3_f64]
    :param gt_lines_3d: Ground truth 3D line segments.
    :type gt_lines_3d: list[le_geometry.LineSegment3_f64]
    :param distance_threshold: Maximum distance for a valid match.
    :type distance_threshold: float
    :return: Reconstruction metrics.
    :rtype: ReconstructionMetrics
    """
    if not segments_3d or not gt_lines_3d:
        return ReconstructionMetrics(
            distance_mean=float("inf"),
            distance_median=float("inf"),
            angle_mean=float("inf"),
            angle_median=float("inf"),
            length_ratio_mean=0.0,
            n_matched=0,
            n_reconstructed=len(segments_3d),
            n_gt=len(gt_lines_3d),
        )

    distances: list[float] = []
    angles: list[float] = []
    length_ratios: list[float] = []

    matched_gt: set[int] = set()

    for recon_seg in segments_3d:
        recon_sp = np.array(recon_seg.start_point(), dtype=np.float64)
        recon_ep = np.array(recon_seg.end_point(), dtype=np.float64)
        recon_mid = (recon_sp + recon_ep) / 2.0
        recon_dir = recon_ep - recon_sp
        recon_len = np.linalg.norm(recon_dir)
        if recon_len > 1e-12:
            recon_dir = recon_dir / recon_len

        best_dist = float("inf")
        best_gt_idx = -1
        best_angle = float("inf")
        best_len_ratio = 0.0

        for gt_idx, gt_seg in enumerate(gt_lines_3d):
            gt_sp = np.array(gt_seg.start_point(), dtype=np.float64)
            gt_ep = np.array(gt_seg.end_point(), dtype=np.float64)
            gt_mid = (gt_sp + gt_ep) / 2.0
            gt_dir = gt_ep - gt_sp
            gt_len = np.linalg.norm(gt_dir)
            if gt_len > 1e-12:
                gt_dir = gt_dir / gt_len

            mid_dist = float(np.linalg.norm(recon_mid - gt_mid))
            cos_angle = abs(float(np.dot(recon_dir, gt_dir)))
            cos_angle = min(cos_angle, 1.0)
            angle_deg = math.degrees(math.acos(cos_angle))

            if mid_dist < best_dist:
                best_dist = mid_dist
                best_gt_idx = gt_idx
                best_angle = angle_deg
                best_len_ratio = recon_len / max(gt_len, 1e-12)

        if best_dist <= distance_threshold and best_gt_idx not in matched_gt:
            distances.append(best_dist)
            angles.append(best_angle)
            length_ratios.append(best_len_ratio)
            matched_gt.add(best_gt_idx)

    if not distances:
        return ReconstructionMetrics(
            distance_mean=float("inf"),
            distance_median=float("inf"),
            angle_mean=float("inf"),
            angle_median=float("inf"),
            length_ratio_mean=0.0,
            n_matched=0,
            n_reconstructed=len(segments_3d),
            n_gt=len(gt_lines_3d),
        )

    return ReconstructionMetrics(
        distance_mean=float(np.mean(distances)),
        distance_median=float(np.median(distances)),
        angle_mean=float(np.mean(angles)),
        angle_median=float(np.median(angles)),
        length_ratio_mean=float(np.mean(length_ratios)),
        n_matched=len(distances),
        n_reconstructed=len(segments_3d),
        n_gt=len(gt_lines_3d),
    )


def evaluation_summary(
    detection: DetectionMetrics | None = None,
    matching: MatchMetrics | None = None,
    reconstruction: ReconstructionMetrics | None = None,
) -> dict[str, float | int]:
    """Combine evaluation metrics into a flat dictionary.

    :param detection: 2D detection metrics.
    :type detection: DetectionMetrics, optional
    :param matching: Cross-view matching metrics.
    :type matching: MatchMetrics, optional
    :param reconstruction: 3D reconstruction metrics.
    :type reconstruction: ReconstructionMetrics, optional
    :return: Combined metrics dictionary.
    :rtype: dict[str, float | int]
    """
    result: dict[str, float | int] = {}

    if detection is not None:
        result["det_precision"] = detection.precision
        result["det_recall"] = detection.recall
        result["det_f1"] = detection.f1
        result["det_tp"] = detection.true_positives
        result["det_fp"] = detection.false_positives
        result["det_fn"] = detection.false_negatives

    if matching is not None:
        result["match_precision"] = matching.precision
        result["match_recall"] = matching.recall
        result["match_f1"] = matching.f1
        result["match_correct"] = matching.n_correct
        result["match_total"] = matching.n_matches
        result["match_possible"] = matching.n_possible

    if reconstruction is not None:
        result["recon_dist_mean"] = reconstruction.distance_mean
        result["recon_dist_median"] = reconstruction.distance_median
        result["recon_angle_mean"] = reconstruction.angle_mean
        result["recon_angle_median"] = reconstruction.angle_median
        result["recon_length_ratio"] = reconstruction.length_ratio_mean
        result["recon_matched"] = reconstruction.n_matched
        result["recon_total"] = reconstruction.n_reconstructed
        result["recon_gt_total"] = reconstruction.n_gt

    return result
