"""Tests for evaluation metrics."""

import pytest

import le_geometry

from lsfm.synthetic.evaluation import (
    DetectionMetrics,
    MatchMetrics,
    ReconstructionMetrics,
    evaluate_detection,
    evaluate_reconstruction,
    evaluation_summary,
)


def _seg(x1: float, y1: float, x2: float, y2: float) -> le_geometry.LineSegment_f64:
    return le_geometry.LineSegment_f64.from_endpoints(x1, y1, x2, y2)


def _seg3d(
    x1: float, y1: float, z1: float, x2: float, y2: float, z2: float
) -> le_geometry.LineSegment3_f64:
    return le_geometry.LineSegment3_f64.from_endpoints(x1, y1, z1, x2, y2, z2)


class TestEvaluateDetection:
    """Tests for 2D detection evaluation."""

    def test_perfect_detection(self) -> None:
        segs = [_seg(0, 0, 100, 0), _seg(0, 0, 0, 100)]
        metrics = evaluate_detection(segs, segs)
        assert metrics.precision == pytest.approx(1.0)
        assert metrics.recall == pytest.approx(1.0)
        assert metrics.f1 == pytest.approx(1.0)

    def test_no_detections(self) -> None:
        gt = [_seg(0, 0, 100, 0)]
        metrics = evaluate_detection([], gt)
        assert metrics.precision == pytest.approx(0.0)
        assert metrics.recall == pytest.approx(0.0)

    def test_returns_dataclass(self) -> None:
        metrics = evaluate_detection([_seg(0, 0, 10, 0)], [_seg(0, 0, 10, 0)])
        assert isinstance(metrics, DetectionMetrics)

    def test_float_segments_promoted_to_f64(self) -> None:
        """LsdCC returns float LineSegment; AccuracyMeasure needs f64."""
        f32_seg = le_geometry.LineSegment.from_endpoints(0.0, 0.0, 100.0, 0.0)
        gt = [_seg(0, 0, 100, 0)]
        metrics = evaluate_detection([f32_seg], gt)
        assert metrics.precision == pytest.approx(1.0)
        assert metrics.recall == pytest.approx(1.0)


class TestEvaluateReconstruction:
    """Tests for 3D reconstruction evaluation."""

    def test_perfect_reconstruction(self) -> None:
        gt = [_seg3d(0, 0, 0, 1, 0, 0), _seg3d(0, 0, 0, 0, 1, 0)]
        metrics = evaluate_reconstruction(gt, gt, distance_threshold=1.0)
        assert metrics.distance_mean == pytest.approx(0.0)
        assert metrics.n_matched == 2

    def test_empty_reconstruction(self) -> None:
        gt = [_seg3d(0, 0, 0, 1, 0, 0)]
        metrics = evaluate_reconstruction([], gt)
        assert metrics.n_matched == 0
        assert metrics.n_gt == 1

    def test_returns_dataclass(self) -> None:
        metrics = evaluate_reconstruction(
            [_seg3d(0, 0, 0, 1, 0, 0)],
            [_seg3d(0, 0, 0, 1, 0, 0)],
        )
        assert isinstance(metrics, ReconstructionMetrics)


class TestEvaluationSummary:
    """Tests for evaluation_summary."""

    def test_empty(self) -> None:
        result = evaluation_summary()
        assert result == {}

    def test_with_detection(self) -> None:
        det = DetectionMetrics(
            precision=0.9,
            recall=0.8,
            f1=0.85,
            true_positives=8,
            false_positives=1,
            false_negatives=2,
        )
        result = evaluation_summary(detection=det)
        assert result["det_precision"] == pytest.approx(0.9)
        assert result["det_f1"] == pytest.approx(0.85)

    def test_combined(self) -> None:
        det = DetectionMetrics(0.9, 0.8, 0.85, 8, 1, 2)
        match = MatchMetrics(0.7, 0.6, 0.65, 6, 10, 12)
        recon = ReconstructionMetrics(0.5, 0.4, 5.0, 4.0, 0.8, 5, 10, 12)
        result = evaluation_summary(detection=det, matching=match, reconstruction=recon)
        assert "det_precision" in result
        assert "match_precision" in result
        assert "recon_dist_mean" in result
