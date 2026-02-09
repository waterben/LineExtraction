"""Integration tests for the le_edge Python extension module.

Tests cover:
  - Module import and core types
  - EdgeSourceI subclasses (Sobel, Scharr, Prewitt)
  - NMS wrapper
  - Edge segment detectors (Drawing, Simple, Linking, Pattern)
  - Multi-preset support (default, _16u, _f32, _f64)
  - EdgeSegment properties and flags
"""

from __future__ import annotations

import numpy as np
import pytest

import le_edge


# ============================================================================
# Fixtures
# ============================================================================


@pytest.fixture
def square_image_u8() -> np.ndarray:
    """Create a simple 100x100 uint8 image with a white square on black."""
    img = np.zeros((100, 100), dtype=np.uint8)
    img[30:70, 30:70] = 255
    return img


@pytest.fixture
def square_image_f32() -> np.ndarray:
    """Create a simple 100x100 float32 image with a white square on black."""
    img = np.zeros((100, 100), dtype=np.float32)
    img[30:70, 30:70] = 1.0
    return img


@pytest.fixture
def square_image_f64() -> np.ndarray:
    """Create a simple 100x100 float64 image with a white square on black."""
    img = np.zeros((100, 100), dtype=np.float64)
    img[30:70, 30:70] = 1.0
    return img


# ============================================================================
# Module-level tests
# ============================================================================


class TestModuleImport:
    """Test that the module and its core types are importable."""

    def test_import_module(self) -> None:
        assert hasattr(le_edge, "EdgeSourceSobel")
        assert hasattr(le_edge, "EsdDrawing")

    def test_core_enums(self) -> None:
        assert le_edge.ES_NONE == 0
        assert le_edge.ES_REVERSE == 1
        assert le_edge.ES_CLOSED == 2

    def test_direction_options_enum(self) -> None:
        assert le_edge.ESDirectionOptions.NONE is not None
        assert le_edge.ESDirectionOptions.GXGY is not None
        assert le_edge.ESDirectionOptions.DIR is not None

    def test_quadrature_options_enum(self) -> None:
        assert le_edge.ESQuadratureOptions.MAG is not None
        assert le_edge.ESQuadratureOptions.ENERGY is not None
        assert le_edge.ESQuadratureOptions.PC is not None


# ============================================================================
# EdgeSegment tests
# ============================================================================


class TestEdgeSegment:
    """Test EdgeSegment struct bindings."""

    def test_default_construction(self) -> None:
        seg = le_edge.EdgeSegment()
        assert seg.begin == 0
        assert seg.end == 0
        assert seg.size() == 0
        assert seg.flags == le_edge.ES_NONE
        assert seg.id == 0
        assert not seg.reverse()
        assert not seg.closed()

    def test_parametrized_construction(self) -> None:
        seg = le_edge.EdgeSegment(10, 25, le_edge.ES_REVERSE, 42)
        assert seg.begin == 10
        assert seg.end == 25
        assert seg.size() == 15
        assert seg.reverse()
        assert not seg.closed()
        assert seg.id == 42

    def test_closed_flag(self) -> None:
        seg = le_edge.EdgeSegment(0, 10, le_edge.ES_REVERSE | le_edge.ES_CLOSED)
        assert seg.reverse()
        assert seg.closed()

    def test_first_last(self) -> None:
        seg = le_edge.EdgeSegment(5, 15)
        assert seg.first() == 5
        assert seg.last() == 14

        seg_rev = le_edge.EdgeSegment(5, 15, le_edge.ES_REVERSE)
        assert seg_rev.first() == 14
        assert seg_rev.last() == 5

    def test_repr(self) -> None:
        seg = le_edge.EdgeSegment(0, 10)
        r = repr(seg)
        assert "EdgeSegment" in r
        assert "size=10" in r

    def test_property_setters(self) -> None:
        seg = le_edge.EdgeSegment()
        seg.begin = 5
        seg.end = 20
        seg.flags = le_edge.ES_CLOSED
        seg.id = 99
        assert seg.begin == 5
        assert seg.end == 20
        assert seg.closed()
        assert seg.id == 99


# ============================================================================
# EdgeSource tests (default uint8 preset)
# ============================================================================


class TestEdgeSourceDefault:
    """Test EdgeSource implementations with default uint8 preset."""

    @pytest.mark.parametrize(
        "cls_name", ["EdgeSourceSobel", "EdgeSourceScharr", "EdgeSourcePrewitt"]
    )
    def test_construction(self, cls_name: str) -> None:
        cls = getattr(le_edge, cls_name)
        es = cls()
        assert es is not None

    @pytest.mark.parametrize(
        "cls_name", ["EdgeSourceSobel", "EdgeSourceScharr", "EdgeSourcePrewitt"]
    )
    def test_process(self, cls_name: str, square_image_u8: np.ndarray) -> None:
        cls = getattr(le_edge, cls_name)
        es = cls()
        es.process(square_image_u8)

        mag = es.magnitude()
        assert mag is not None
        assert mag.shape == square_image_u8.shape

        gx = es.gx()
        gy = es.gy()
        assert gx.shape == square_image_u8.shape
        assert gy.shape == square_image_u8.shape

        direction = es.direction()
        assert direction.shape == square_image_u8.shape

        dmap = es.direction_map()
        assert dmap.shape == square_image_u8.shape
        assert dmap.dtype == np.int8

        seeds = es.seeds()
        assert isinstance(seeds, list)
        assert len(seeds) > 0, "Expected seeds from square image"

    @pytest.mark.parametrize(
        "cls_name", ["EdgeSourceSobel", "EdgeSourceScharr", "EdgeSourcePrewitt"]
    )
    def test_name(self, cls_name: str) -> None:
        cls = getattr(le_edge, cls_name)
        es = cls()
        n = es.name()
        assert isinstance(n, str)
        assert len(n) > 0

    def test_magnitude_max(self, square_image_u8: np.ndarray) -> None:
        es = le_edge.EdgeSourceSobel()
        es.process(square_image_u8)
        mag_max = es.magnitude_max()
        assert mag_max > 0

    def test_magnitude_threshold(self, square_image_u8: np.ndarray) -> None:
        es = le_edge.EdgeSourceSobel()
        es.process(square_image_u8)
        th = es.magnitude_threshold(0.5)
        assert isinstance(th, float)

    def test_direction_range(self, square_image_u8: np.ndarray) -> None:
        es = le_edge.EdgeSourceSobel()
        es.process(square_image_u8)
        dr = es.direction_range()
        assert hasattr(dr, "lower")
        assert hasattr(dr, "upper")

    def test_hysteresis(self, square_image_u8: np.ndarray) -> None:
        es = le_edge.EdgeSourceSobel()
        es.process(square_image_u8)

        hyst = es.hysteresis()
        assert hyst.shape == square_image_u8.shape

        hyst_bin = es.hysteresis_binary()
        assert hyst_bin.shape == square_image_u8.shape

        edgels = es.hysteresis_edgels()
        assert isinstance(edgels, list)

    def test_value_manager(self) -> None:
        es = le_edge.EdgeSourceSobel()
        vals = es.values()
        assert isinstance(vals, dict)
        assert len(vals) > 0


# ============================================================================
# NMS tests
# ============================================================================


class TestNonMaximaSuppression:
    """Test NonMaximaSuppression wrapper."""

    def test_construction_default(self) -> None:
        nms = le_edge.NonMaximaSuppression()
        assert nms.threshold_low() == pytest.approx(0.004)
        assert nms.threshold_high() == pytest.approx(0.012)
        assert nms.border() == 1

    def test_construction_custom(self) -> None:
        nms = le_edge.NonMaximaSuppression(0.01, 0.05, 2)
        assert nms.threshold_low() == pytest.approx(0.01)
        assert nms.threshold_high() == pytest.approx(0.05)
        assert nms.border() == 2

    def test_set_threshold(self) -> None:
        nms = le_edge.NonMaximaSuppression()
        nms.set_threshold(0.02, 0.08)
        assert nms.threshold_low() == pytest.approx(0.02)
        assert nms.threshold_high() == pytest.approx(0.08)

    def test_process_with_gradient(self, square_image_u8: np.ndarray) -> None:
        """Test NMS by computing gradient first, then applying NMS."""
        import le_imgproc

        grad = le_imgproc.SobelGradient()
        grad.process(square_image_u8)

        nms = le_edge.NonMaximaSuppression()
        gx = grad.gx()
        gy = grad.gy()
        mag = grad.magnitude()

        low = np.float32(grad.magnitude_threshold(0.004))
        high = np.float32(grad.magnitude_threshold(0.012))
        nms.process(gx, gy, mag, low, high)

        dmap = nms.direction_map()
        assert dmap.shape == square_image_u8.shape

        seeds = nms.seeds()
        assert isinstance(seeds, list)

    def test_name(self) -> None:
        nms = le_edge.NonMaximaSuppression()
        assert "NMS" in nms.name()

    def test_value_manager(self) -> None:
        nms = le_edge.NonMaximaSuppression()
        vals = nms.values()
        assert "nms_th_low" in vals
        assert "nms_th_high" in vals


# ============================================================================
# Edge Segment Detector tests (default preset)
# ============================================================================


class TestEsdDetectors:
    """Test edge segment detectors (EsdDrawing, EsdSimple, EsdLinking, EsdPattern)."""

    @pytest.fixture
    def edge_source(self, square_image_u8: np.ndarray) -> le_edge.EdgeSourceSobel:
        """Pre-processed edge source for use by ESD detectors."""
        es = le_edge.EdgeSourceSobel()
        es.process(square_image_u8)
        return es

    @pytest.mark.parametrize(
        "cls_name", ["EsdDrawing", "EsdSimple", "EsdLinking", "EsdPattern"]
    )
    def test_construction(self, cls_name: str) -> None:
        cls = getattr(le_edge, cls_name)
        esd = cls()
        assert esd is not None

    @pytest.mark.parametrize(
        "cls_name", ["EsdDrawing", "EsdSimple", "EsdLinking", "EsdPattern"]
    )
    def test_detect_from_source(
        self, cls_name: str, edge_source: le_edge.EdgeSourceSobel
    ) -> None:
        cls = getattr(le_edge, cls_name)
        esd = cls()
        esd.detect(edge_source)

        segments = esd.segments()
        points = esd.points()

        assert isinstance(segments, list)
        assert isinstance(points, list)
        assert len(segments) > 0, f"Expected segments from {cls_name}"
        assert len(points) > 0, f"Expected points from {cls_name}"

        # Validate segment properties
        for seg in segments:
            assert isinstance(seg, le_edge.EdgeSegment)
            assert seg.size() > 0
            assert seg.end <= len(points)

    @pytest.mark.parametrize(
        "cls_name", ["EsdDrawing", "EsdSimple", "EsdLinking", "EsdPattern"]
    )
    def test_detect_from_mats(
        self, cls_name: str, edge_source: le_edge.EdgeSourceSobel
    ) -> None:
        cls = getattr(le_edge, cls_name)
        esd = cls()
        esd.detect(
            edge_source.direction_map(), edge_source.magnitude(), edge_source.seeds()
        )

        segments = esd.segments()
        assert isinstance(segments, list)

    @pytest.mark.parametrize(
        "cls_name", ["EsdDrawing", "EsdSimple", "EsdLinking", "EsdPattern"]
    )
    def test_name(self, cls_name: str) -> None:
        cls = getattr(le_edge, cls_name)
        esd = cls()
        n = esd.name()
        assert isinstance(n, str)
        assert len(n) > 0

    def test_esd_drawing_params(self) -> None:
        esd = le_edge.EsdDrawing(min_pixels=5, mag_mul=2.0, mag_th=3.0)
        vals = esd.values()
        assert "edge_min_pixels" in vals

    def test_esd_simple_params(self) -> None:
        esd = le_edge.EsdSimple(min_pixels=5)
        vals = esd.values()
        assert "edge_min_pixels" in vals

    def test_esd_linking_params(self) -> None:
        esd = le_edge.EsdLinking(min_pixels=5, max_gap=2, mag_mul=2.0, mag_th=3.0)
        vals = esd.values()
        assert "edge_min_pixels" in vals
        assert "edge_max_gap" in vals

    def test_esd_pattern_params(self) -> None:
        esd = le_edge.EsdPattern(
            min_pixels=5, max_gap=2, mag_mul=2.0, mag_th=3.0, pat_tol=1
        )
        vals = esd.values()
        assert "edge_min_pixels" in vals
        assert "edge_pat_tol" in vals

    def test_esd_pattern_extra_accessors(
        self, edge_source: le_edge.EdgeSourceSobel
    ) -> None:
        esd = le_edge.EsdPattern()
        esd.detect(edge_source)
        patterns = esd.patterns()
        pattern_segs = esd.pattern_segments()
        assert isinstance(patterns, list)
        assert isinstance(pattern_segs, list)


# ============================================================================
# Multi-preset tests
# ============================================================================


class TestMultiPreset:
    """Test that all type presets are available and functional."""

    @pytest.mark.parametrize(
        "suffix,dtype,cls_prefix",
        [
            ("", np.uint8, "EdgeSourceSobel"),
            ("_16u", np.uint16, "EdgeSourceSobel_16u"),
            ("_f32", np.float32, "EdgeSourceSobel_f32"),
            ("_f64", np.float64, "EdgeSourceSobel_f64"),
        ],
    )
    def test_edge_source_preset(
        self, suffix: str, dtype: type, cls_prefix: str
    ) -> None:
        cls = getattr(le_edge, cls_prefix)
        es = cls()

        img = np.zeros((50, 50), dtype=dtype)
        if dtype in (np.float32, np.float64):
            img[15:35, 15:35] = 1.0
        elif dtype == np.uint16:
            img[15:35, 15:35] = 65535
        else:
            img[15:35, 15:35] = 255

        es.process(img)
        mag = es.magnitude()
        assert mag is not None
        assert mag.shape == (50, 50)

    @pytest.mark.parametrize(
        "esd_cls,suffix",
        [
            ("EsdDrawing", ""),
            ("EsdDrawing_f64", "_f64"),
            ("EsdSimple", ""),
            ("EsdSimple_f64", "_f64"),
            ("EsdLinking", ""),
            ("EsdLinking_f64", "_f64"),
            ("EsdPattern", ""),
            ("EsdPattern_f64", "_f64"),
        ],
    )
    def test_esd_presets(self, esd_cls: str, suffix: str) -> None:
        cls = getattr(le_edge, esd_cls)
        esd = cls()
        assert esd is not None

    def test_nms_presets(self) -> None:
        for cls_name in [
            "NonMaximaSuppression",
            "NonMaximaSuppression_f64",
        ]:
            cls = getattr(le_edge, cls_name)
            nms = cls()
            assert nms is not None

    @pytest.mark.parametrize("suffix", ["", "_16u", "_f32", "_f64"])
    def test_all_edge_source_types(self, suffix: str) -> None:
        for base in ["EdgeSourceSobel", "EdgeSourceScharr", "EdgeSourcePrewitt"]:
            cls_name = base + suffix
            cls = getattr(le_edge, cls_name)
            es = cls()
            assert es is not None
            assert isinstance(es.name(), str)


# ============================================================================
# End-to-end pipeline test
# ============================================================================


class TestEndToEndPipeline:
    """Full pipeline: image -> EdgeSource -> ESD -> segments -> pixel coords."""

    def test_full_pipeline(self, square_image_u8: np.ndarray) -> None:
        # Step 1: Edge source
        es = le_edge.EdgeSourceSobel()
        es.process(square_image_u8)

        # Step 2: Edge segment detection
        esd = le_edge.EsdDrawing(min_pixels=5)
        esd.detect(es)

        segments = esd.segments()
        points = esd.points()
        assert len(segments) > 0
        assert len(points) > 0

        # Step 3: Convert linear indices to (row, col) coordinates
        rows = square_image_u8.shape[0]
        cols = square_image_u8.shape[1]
        for seg in segments:
            seg_points = points[seg.begin : seg.end]
            for idx in seg_points:
                row = idx // cols
                col = idx % cols
                assert 0 <= row < rows
                assert 0 <= col < cols

    def test_pipeline_with_all_detectors(self, square_image_u8: np.ndarray) -> None:
        es = le_edge.EdgeSourceSobel()
        es.process(square_image_u8)

        detector_classes = ["EsdDrawing", "EsdSimple", "EsdLinking", "EsdPattern"]
        for cls_name in detector_classes:
            cls = getattr(le_edge, cls_name)
            esd = cls()
            esd.detect(es)
            assert len(esd.segments()) > 0, f"{cls_name} found no segments"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
