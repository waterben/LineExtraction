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

    def test_detector_id(self) -> None:
        assert le_algorithm.DetectorId.LSD_CC is not None
        assert le_algorithm.DetectorId.LSD_FGIOI is not None
        assert le_algorithm.DetectorId.LSD_HOUGHP is not None


class TestImageAnalyzer:
    """Tests for ImageAnalyzer and ImageProperties bindings."""

    def test_analyze_uniform(self) -> None:
        img = np.full((100, 100), 128, dtype=np.uint8)
        props = le_algorithm.ImageAnalyzer.analyze(img)
        assert 0.0 <= props.contrast <= 1.0
        assert props.contrast < 0.05

    def test_analyze_noisy(self) -> None:
        rng = np.random.default_rng(42)
        img = rng.normal(128, 50, (200, 200)).clip(0, 255).astype(np.uint8)
        props = le_algorithm.ImageAnalyzer.analyze(img)
        assert props.noise_level > 0.1

    def test_analyze_high_contrast(self) -> None:
        img = np.zeros((100, 100), dtype=np.uint8)
        img[::2, :] = 255
        props = le_algorithm.ImageAnalyzer.analyze(img)
        assert props.contrast > 0.3

    def test_properties_repr(self) -> None:
        props = le_algorithm.ImageProperties()
        assert "ImageProperties" in repr(props)

    def test_suggest_profile(self) -> None:
        props = le_algorithm.ImageProperties()
        props.contrast = 0.5
        props.noise_level = 0.2
        props.edge_density = 0.3
        props.dynamic_range = 0.7
        hints = props.suggest_profile()
        assert 0 <= hints.detail <= 100
        assert 0 <= hints.gap_tolerance <= 100
        assert 0 <= hints.min_length <= 100
        assert 0 <= hints.precision <= 100

    def test_hints_clamp(self) -> None:
        hints = le_algorithm.ProfileHints()
        hints.detail = 150
        hints.noise_factor = 0.1
        hints.clamp()
        assert hints.detail == 100.0
        assert hints.noise_factor == 0.5


class TestDetectorProfile:
    """Tests for DetectorProfile bindings."""

    def test_default_construction(self) -> None:
        profile = le_algorithm.DetectorProfile()
        assert profile.detail == 50.0
        assert profile.gap_tolerance == 50.0

    def test_explicit_construction(self) -> None:
        profile = le_algorithm.DetectorProfile(
            detail=80, gap_tolerance=30, min_length=60, precision=90
        )
        assert profile.detail == 80.0
        assert profile.precision == 90.0

    def test_from_hints(self) -> None:
        hints = le_algorithm.ProfileHints()
        hints.detail = 70
        hints.contrast_factor = 1.3
        profile = le_algorithm.DetectorProfile.from_hints(hints)
        assert profile.detail == 70.0
        assert profile.contrast_factor == 1.3

    def test_from_image(self) -> None:
        img = np.full((200, 200), 128, dtype=np.uint8)
        img[50:150, 50:150] = 200
        profile = le_algorithm.DetectorProfile.from_image(img)
        assert 0 <= profile.detail <= 100
        assert 0 <= profile.precision <= 100

    def test_supported_detectors(self) -> None:
        names = le_algorithm.DetectorProfile.supported_detectors()
        assert len(names) == 9
        assert "LsdCC" in names
        assert "LsdHoughP" in names

    def test_to_params(self) -> None:
        profile = le_algorithm.DetectorProfile(50, 50, 50, 50)
        params = profile.to_params(le_algorithm.DetectorId.LSD_CC)
        assert len(params) > 0
        assert all("name" in p and "value" in p for p in params)

    def test_to_params_by_name(self) -> None:
        profile = le_algorithm.DetectorProfile(50, 50, 50, 50)
        params = profile.to_params_by_name("LsdCC")
        assert len(params) > 0

    def test_all_detectors_produce_params(self) -> None:
        profile = le_algorithm.DetectorProfile(50, 50, 50, 50)
        for det_id in le_algorithm.DetectorId.__members__.values():
            params = profile.to_params(det_id)
            assert len(params) > 0, f"No params for {det_id}"

    def test_name_conversion(self) -> None:
        assert (
            le_algorithm.detector_id_to_name(le_algorithm.DetectorId.LSD_CC) == "LsdCC"
        )
        assert (
            le_algorithm.detector_name_to_id("LsdCC") == le_algorithm.DetectorId.LSD_CC
        )
        assert (
            le_algorithm.detector_name_to_id("lsdcc") == le_algorithm.DetectorId.LSD_CC
        )

    def test_property_setters(self) -> None:
        profile = le_algorithm.DetectorProfile()
        profile.detail = 80
        profile.noise_factor = 1.5
        assert profile.detail == 80.0
        assert profile.noise_factor == 1.5

    def test_repr(self) -> None:
        profile = le_algorithm.DetectorProfile(70, 30, 50, 80)
        assert "DetectorProfile" in repr(profile)

    def test_detail_affects_thresholds(self) -> None:
        lo = le_algorithm.DetectorProfile(detail=10)
        hi = le_algorithm.DetectorProfile(detail=90)
        lo_p = {
            p["name"]: p["value"] for p in lo.to_params(le_algorithm.DetectorId.LSD_CC)
        }
        hi_p = {
            p["name"]: p["value"] for p in hi.to_params(le_algorithm.DetectorId.LSD_CC)
        }
        # More detail → lower NMS thresholds
        assert lo_p["nms_th_low"] > hi_p["nms_th_low"]


class TestPrecisionOptimize:
    """Tests for PrecisionOptimize bindings."""

    def test_default_construction(self) -> None:
        opt = le_algorithm.PrecisionOptimize()
        assert repr(opt) == "<PrecisionOptimize>"

    def test_construction_from_dict(self) -> None:
        opt = le_algorithm.PrecisionOptimize({"search_range_d": 2.0, "use_fast": True})
        params = opt.get_params()
        assert params["search_range_d"] == pytest.approx(2.0)
        assert params["use_fast"] is True

    def test_get_params(self) -> None:
        opt = le_algorithm.PrecisionOptimize()
        params = opt.get_params()
        assert "search_range_d" in params
        assert "search_range_r" in params
        assert "interpolation" in params
        assert "search_strategy" in params
        assert "stop_strategy" in params
        assert "stop_delta" in params
        assert "max_iterations" in params
        assert "derivative_precision" in params
        assert "mean_param" in params
        assert "use_sampled" in params
        assert "use_fast" in params
        assert len(params) == 11

    def test_set_params(self) -> None:
        opt = le_algorithm.PrecisionOptimize()
        opt.set_params({"search_range_d": 3.5, "max_iterations": 100})
        params = opt.get_params()
        assert params["search_range_d"] == pytest.approx(3.5)
        assert params["max_iterations"] == 100

    def test_param_descriptions(self) -> None:
        opt = le_algorithm.PrecisionOptimize()
        descs = opt.param_descriptions()
        assert "search_range_d" in descs
        assert len(descs["search_range_d"]) > 0

    def test_optimize_line(self) -> None:
        # Create a horizontal bright stripe and a slightly offset line
        mag = np.zeros((100, 100), dtype=np.float32)
        mag[48:52, 10:90] = 255.0  # Bright horizontal band at y=50
        line = _make_segment(10, 51, 90, 51)  # 1 pixel off

        opt = le_algorithm.PrecisionOptimize()
        error, refined = opt.optimize_line(mag, line)
        assert isinstance(error, float)
        assert isinstance(refined, le_geometry.LineSegment_f64)

    def test_optimize_all(self) -> None:
        mag = np.zeros((100, 100), dtype=np.float32)
        mag[48:52, 10:90] = 255.0
        lines = [
            _make_segment(10, 51, 90, 51),
            _make_segment(10, 49, 90, 49),
        ]

        opt = le_algorithm.PrecisionOptimize()
        errors, refined = opt.optimize_all(mag, lines)
        assert len(errors) == 2
        assert len(refined) == 2
        assert all(isinstance(e, float) for e in errors)
        assert all(isinstance(s, le_geometry.LineSegment_f64) for s in refined)

    def test_optimize_empty(self) -> None:
        mag = np.zeros((50, 50), dtype=np.float32)
        opt = le_algorithm.PrecisionOptimize()
        errors, refined = opt.optimize_all(mag, [])
        assert len(errors) == 0
        assert len(refined) == 0

    def test_enums(self) -> None:
        assert le_algorithm.PrecisionSearchStrategy.BFGS is not None
        assert le_algorithm.PrecisionSearchStrategy.LBFGS is not None
        assert le_algorithm.PrecisionSearchStrategy.CG is not None
        assert le_algorithm.PrecisionStopStrategy.DELTA is not None
        assert le_algorithm.PrecisionStopStrategy.GRAD_NORM is not None
        assert le_algorithm.InterpolationMode.NEAREST is not None
        assert le_algorithm.InterpolationMode.NEAREST_ROUND is not None
        assert le_algorithm.InterpolationMode.BILINEAR is not None
        assert le_algorithm.InterpolationMode.BICUBIC is not None

    def test_enum_params(self) -> None:
        opt = le_algorithm.PrecisionOptimize({"search_strategy": 1, "interpolation": 3})
        params = opt.get_params()
        assert params["search_strategy"] == 1  # LBFGS
        assert params["interpolation"] == 3  # BICUBIC


# ===========================================================================
# PresetStore tests
# ===========================================================================

# Minimal JSON for testing PresetStore
_TEST_PRESETS_JSON = """{
  "metadata": {"num_images": 10},
  "detectors": {
    "LsdCC": {
      "fast": {
        "params": {"nms_th_low": 0.01, "nms_th_high": 0.03, "edge_min_pixels": 5},
        "score": 0.85
      },
      "balanced": {
        "params": {"nms_th_low": 0.008, "nms_th_high": 0.025, "edge_min_pixels": 8},
        "score": 0.90
      },
      "accurate": {
        "params": {"nms_th_low": 0.004, "edge_min_pixels": 3},
        "score": 0.78
      }
    },
    "LsdFGioi": {
      "fast": {
        "params": {"quant_error": 2.0, "bins": 1024},
        "score": 0.82
      }
    }
  }
}"""


class TestPresetStore:
    """Tests for PresetStore bindings."""

    def test_default_construction(self) -> None:
        store = le_algorithm.PresetStore()
        assert store.empty()
        assert store.num_detectors() == 0

    def test_from_string(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        assert not store.empty()
        assert store.num_detectors() == 2

    def test_invalid_json_raises(self) -> None:
        with pytest.raises(RuntimeError):
            le_algorithm.PresetStore.from_string("{bad json")

    def test_missing_detectors_raises(self) -> None:
        with pytest.raises(RuntimeError):
            le_algorithm.PresetStore.from_string('{"metadata": {}}')

    def test_nonexistent_file_raises(self) -> None:
        with pytest.raises(RuntimeError):
            le_algorithm.PresetStore("/nonexistent/preset_file.json")

    def test_detector_names(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        names = store.detector_names()
        assert "LsdCC" in names
        assert "LsdFGioi" in names

    def test_preset_names(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        names = store.preset_names()
        assert "fast" in names
        assert "balanced" in names
        assert "accurate" in names

    def test_get_by_detector_id(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        params = store.get(le_algorithm.DetectorId.LSD_CC, "balanced")
        assert isinstance(params, list)
        assert len(params) == 3
        param_dict = {p["name"]: p["value"] for p in params}
        assert abs(param_dict["nms_th_low"] - 0.008) < 1e-6
        assert param_dict["edge_min_pixels"] == 8

    def test_get_by_name(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        params = store.get_by_name("LsdFGioi", "fast")
        assert len(params) == 2
        param_dict = {p["name"]: p["value"] for p in params}
        assert abs(param_dict["quant_error"] - 2.0) < 1e-6
        assert param_dict["bins"] == 1024

    def test_get_unknown_preset_raises(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        with pytest.raises(IndexError):
            store.get(le_algorithm.DetectorId.LSD_CC, "nonexistent")

    def test_get_unknown_detector_raises(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        with pytest.raises(IndexError):
            store.get(le_algorithm.DetectorId.LSD_BURNS, "fast")

    def test_score(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        assert abs(store.score(le_algorithm.DetectorId.LSD_CC, "fast") - 0.85) < 1e-6
        assert (
            abs(store.score(le_algorithm.DetectorId.LSD_CC, "balanced") - 0.90) < 1e-6
        )

    def test_score_by_name(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        assert abs(store.score_by_name("LsdFGioi", "fast") - 0.82) < 1e-6

    def test_has(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        assert store.has(le_algorithm.DetectorId.LSD_CC, "fast")
        assert store.has(le_algorithm.DetectorId.LSD_CC, "balanced")
        assert not store.has(le_algorithm.DetectorId.LSD_CC, "nonexistent")
        assert not store.has(le_algorithm.DetectorId.LSD_BURNS, "fast")

    def test_has_by_name(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        assert store.has_by_name("LsdFGioi", "fast")
        assert not store.has_by_name("LsdFGioi", "accurate")
        assert not store.has_by_name("NoSuch", "fast")

    def test_constants(self) -> None:
        assert le_algorithm.PresetStore.FAST == "fast"
        assert le_algorithm.PresetStore.BALANCED == "balanced"
        assert le_algorithm.PresetStore.ACCURATE == "accurate"

    def test_repr(self) -> None:
        store = le_algorithm.PresetStore.from_string(_TEST_PRESETS_JSON)
        r = repr(store)
        assert "PresetStore" in r
        assert "2 detectors" in r

    def test_empty_store_repr(self) -> None:
        store = le_algorithm.PresetStore()
        r = repr(store)
        assert "0 detectors" in r


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
