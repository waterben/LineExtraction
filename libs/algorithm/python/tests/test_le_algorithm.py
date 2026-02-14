"""Tests for the le_algorithm Python bindings."""

import numpy as np
import pytest

import le_algorithm
import le_geometry


def _make_segment(
    x1: float, y1: float, x2: float, y2: float
) -> le_geometry.LineSegment_f64:
    """Create a double-precision LineSegment from endpoint coordinates."""
    return le_geometry.LineSegment_f64.from_endpoints(x1, y1, x2, y2)


class TestLineMerge:
    """Tests for LineMerge bindings."""

    def test_default_construction(self) -> None:
        merger = le_algorithm.LineMerge()
        assert repr(merger) == "<LineMerge>"

    def test_merge_collinear(self) -> None:
        merger = le_algorithm.LineMerge(
            max_dist=20.0, angle_error=10.0, distance_error=3.0, parallel_error=15.0
        )
        segments = [
            _make_segment(0, 0, 10, 0),
            _make_segment(12, 0, 22, 0),
        ]
        result = merger.merge_lines(segments)
        assert len(result) == 1

    def test_preserve_distant(self) -> None:
        merger = le_algorithm.LineMerge(max_dist=5.0)
        segments = [
            _make_segment(0, 0, 10, 0),
            _make_segment(100, 100, 110, 100),
        ]
        result = merger.merge_lines(segments)
        assert len(result) == 2

    def test_empty_input(self) -> None:
        merger = le_algorithm.LineMerge()
        result = merger.merge_lines([])
        assert len(result) == 0


class TestLineConnect:
    """Tests for LineConnect bindings."""

    def test_default_construction(self) -> None:
        connector = le_algorithm.LineConnect()
        assert repr(connector) == "<LineConnect>"

    def test_empty_input(self) -> None:
        connector = le_algorithm.LineConnect()
        mag = np.zeros((100, 100), dtype=np.float32)
        result = connector.connect_lines([], mag)
        assert len(result) == 0


class TestAccuracyMeasure:
    """Tests for AccuracyMeasure and AccuracyResult bindings."""

    def test_perfect_match(self) -> None:
        measure = le_algorithm.AccuracyMeasure(threshold=5.0)
        segments = [
            _make_segment(0, 0, 10, 0),
            _make_segment(0, 10, 10, 10),
        ]
        result = measure.evaluate(segments, segments)
        assert result.precision == pytest.approx(1.0)
        assert result.recall == pytest.approx(1.0)
        assert result.f1 == pytest.approx(1.0)
        assert result.true_positives == 2

    def test_no_detections(self) -> None:
        measure = le_algorithm.AccuracyMeasure(threshold=5.0)
        gt = [_make_segment(0, 0, 10, 0)]
        result = measure.evaluate([], gt)
        assert result.precision == pytest.approx(0.0)
        assert result.recall == pytest.approx(0.0)

    def test_threshold_property(self) -> None:
        measure = le_algorithm.AccuracyMeasure(threshold=5.0)
        assert measure.threshold == pytest.approx(5.0)
        measure.threshold = 10.0
        assert measure.threshold == pytest.approx(10.0)

    def test_structural_ap(self) -> None:
        measure = le_algorithm.AccuracyMeasure(threshold=5.0)
        segments = [_make_segment(0, 0, 10, 0)]
        sap = measure.structural_ap(segments, segments)
        assert sap == pytest.approx(1.0)


class TestGroundTruth:
    """Tests for GroundTruthLoader and GroundTruthEntry bindings."""

    def test_make_entry(self) -> None:
        segments = [_make_segment(0, 0, 10, 10)]
        entry = le_algorithm.GroundTruthLoader.make_entry("test.png", segments)
        assert entry.image_name == "test.png"
        assert len(entry.segments) == 1

    def test_entry_repr(self) -> None:
        entry = le_algorithm.GroundTruthEntry()
        assert "GroundTruthEntry" in repr(entry)


class TestSearchStrategy:
    """Tests for search strategies."""

    def test_grid_search_single_param(self) -> None:
        strategy = le_algorithm.GridSearchStrategy()
        space = [le_algorithm.ParamRange("a", 0.0, 1.0, 0.5)]
        configs = strategy.generate(space)
        assert len(configs) == 3  # 0.0, 0.5, 1.0

    def test_grid_search_bool(self) -> None:
        strategy = le_algorithm.GridSearchStrategy()
        space = [le_algorithm.ParamRange.make_bool("flag")]
        configs = strategy.generate(space)
        assert len(configs) == 2

    def test_random_search(self) -> None:
        strategy = le_algorithm.RandomSearchStrategy(num_samples=50, seed=42)
        space = [le_algorithm.ParamRange("a", 0.0, 10.0, 1.0)]
        configs = strategy.generate(space)
        assert len(configs) == 50

    def test_random_search_reproducible(self) -> None:
        space = [le_algorithm.ParamRange("a", 0.0, 10.0)]
        s1 = le_algorithm.RandomSearchStrategy(10, 42)
        s2 = le_algorithm.RandomSearchStrategy(10, 42)
        c1 = s1.generate(space)
        c2 = s2.generate(space)
        assert len(c1) == len(c2)


class TestParamOptimizer:
    """Tests for ParamOptimizer."""

    def test_construction(self) -> None:
        optimizer = le_algorithm.ParamOptimizer()
        assert repr(optimizer) == "<ParamOptimizer>"

    def test_basic_optimization(self) -> None:
        gt_lines = [
            _make_segment(10, 10, 90, 10),
            _make_segment(10, 50, 90, 50),
        ]
        ground_truth = [le_algorithm.GroundTruthLoader.make_entry("test.png", gt_lines)]
        test_img = np.zeros((100, 100), dtype=np.uint8)
        images = [("test.png", test_img)]

        def detect_fn(
            src: np.ndarray,
            params: list[dict],
        ) -> list[le_geometry.LineSegment_f64]:
            # params is a list of {"name": ..., "value": ...} dicts
            sensitivity = float(params[0]["value"])
            detected: list[le_geometry.LineSegment_f64] = []
            if sensitivity >= 0.3:
                detected.extend(gt_lines)
            return detected

        space = [le_algorithm.ParamRange("sensitivity", 0.0, 1.0, 0.1)]
        optimizer = le_algorithm.ParamOptimizer(
            metric=le_algorithm.OptimMetric.F1, match_threshold=5.0
        )
        strategy = le_algorithm.GridSearchStrategy()
        result = optimizer.optimize(strategy, space, images, ground_truth, detect_fn)
        assert result.best_score > 0.0


class TestEnums:
    """Tests for enum bindings."""

    def test_merge_type(self) -> None:
        assert le_algorithm.MergeType.STANDARD is not None
        assert le_algorithm.MergeType.AVG is not None

    def test_optim_metric(self) -> None:
        assert le_algorithm.OptimMetric.F1 is not None
        assert le_algorithm.OptimMetric.PRECISION is not None
        assert le_algorithm.OptimMetric.RECALL is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
