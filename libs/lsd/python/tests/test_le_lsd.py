"""Integration tests for the le_lsd Python extension module.

Tests cover:
  - Module import and core types
  - Line and LineSegment geometry types
  - LdBase / LsdBase abstract interfaces
  - Concrete detectors (CC, CP, Burns, FBW, FGioi, EDLZ, EL, EP, HoughP)
  - Multi-preset support (float default, double _f64)
  - DataDescriptorEntry properties
  - Flag constants
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import le_lsd


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
def lines_image_u8() -> np.ndarray:
    """Create a 200x200 uint8 image with several distinct lines."""
    img = np.zeros((200, 200), dtype=np.uint8)
    # Horizontal line
    img[50, 20:180] = 255
    # Vertical line
    img[20:180, 100] = 255
    # Diagonal line
    for i in range(160):
        r = 20 + i
        c = 20 + i
        if r < 200 and c < 200:
            img[r, c] = 255
    return img


# ============================================================================
# Module-level tests
# ============================================================================


class TestModuleImport:
    """Test that the module and its core types are importable."""

    def test_import_module(self) -> None:
        assert hasattr(le_lsd, "LsdCC")
        assert hasattr(le_lsd, "LsdBurns")
        assert hasattr(le_lsd, "LsdFGioi")

    def test_core_types(self) -> None:
        assert hasattr(le_lsd, "Line")
        assert hasattr(le_lsd, "LineSegment")
        assert hasattr(le_lsd, "DataDescriptorEntry")

    def test_flag_constants(self) -> None:
        assert le_lsd.CC_FIND_NEAR_COMPLEX == 1
        assert le_lsd.CC_CORNER_RULE == 2
        assert le_lsd.CC_ADD_THICK_PIXELS == 4
        assert le_lsd.BURNS_NMS == 1
        assert le_lsd.FBW_NMS == 1
        assert le_lsd.FBW_PATAN == 2
        assert le_lsd.EL_USE_NFA == 1
        assert le_lsd.EL_USE_PRECISE_SPE == 2
        assert le_lsd.EP_USE_PRECISE_SPE == 2

    def test_f64_preset_exists(self) -> None:
        assert hasattr(le_lsd, "LsdCC_f64")
        assert hasattr(le_lsd, "Line_f64")
        assert hasattr(le_lsd, "LineSegment_f64")
        assert hasattr(le_lsd, "LsdBase_f64")


# ============================================================================
# DataDescriptorEntry tests
# ============================================================================


class TestDataDescriptorEntry:
    """Test DataDescriptorEntry struct bindings."""

    def test_default_construction(self) -> None:
        entry = le_lsd.DataDescriptorEntry()
        assert entry.name == ""
        assert entry.description == ""

    def test_parametrized_construction(self) -> None:
        entry = le_lsd.DataDescriptorEntry("gradient", "Gradient magnitude map")
        assert entry.name == "gradient"
        assert entry.description == "Gradient magnitude map"

    def test_readwrite(self) -> None:
        entry = le_lsd.DataDescriptorEntry()
        entry.name = "edge_map"
        entry.description = "Binary edge map"
        assert entry.name == "edge_map"
        assert entry.description == "Binary edge map"

    def test_repr(self) -> None:
        entry = le_lsd.DataDescriptorEntry("test", "test desc")
        r = repr(entry)
        assert "DataDescriptorEntry" in r
        assert "test" in r


# ============================================================================
# Line geometry tests
# ============================================================================


class TestLine:
    """Test Line<float> geometry bindings."""

    def test_default_construction(self) -> None:
        line = le_lsd.Line()
        assert line.normal_x == 0.0
        assert line.normal_y == 0.0
        assert line.origin_dist == 0.0
        assert line.empty()
        assert not line.valid()

    def test_parametrized_construction(self) -> None:
        line = le_lsd.Line(1.0, 0.0, 5.0)
        assert line.normal_x == pytest.approx(1.0)
        assert line.normal_y == pytest.approx(0.0)
        assert line.origin_dist == pytest.approx(5.0)
        assert line.valid()
        assert not line.empty()

    def test_unit_normal(self) -> None:
        line = le_lsd.Line(0.0, 1.0, 3.0)
        assert line.valid()
        assert line.normal_angle == pytest.approx(math.pi / 2, abs=0.01)

    def test_distance(self) -> None:
        # Line: x = 5
        line = le_lsd.Line(1.0, 0.0, 5.0)
        d = line.distance(7.0, 0.0)
        assert d == pytest.approx(2.0)

    def test_repr(self) -> None:
        line = le_lsd.Line(1.0, 0.0, 5.0)
        r = repr(line)
        assert "Line" in r
        assert "nx=" in r

    def test_f64_variant(self) -> None:
        line = le_lsd.Line_f64(1.0, 0.0, 5.0)
        assert line.normal_x == pytest.approx(1.0)
        assert line.valid()


# ============================================================================
# LineSegment geometry tests
# ============================================================================


class TestLineSegment:
    """Test LineSegment<float> geometry bindings."""

    def test_default_construction(self) -> None:
        seg = le_lsd.LineSegment()
        assert seg.length == pytest.approx(0.0)

    def test_parametrized_construction(self) -> None:
        seg = le_lsd.LineSegment(0.0, 1.0, 5.0, -10.0, 10.0)
        assert seg.normal_x == pytest.approx(0.0)
        assert seg.normal_y == pytest.approx(1.0)
        assert seg.origin_dist == pytest.approx(5.0)
        assert seg.length == pytest.approx(20.0)
        assert seg.octave == 0

    def test_start_end_points(self) -> None:
        seg = le_lsd.LineSegment(0.0, 1.0, 5.0, -10.0, 10.0)
        sp = seg.start_point()
        ep = seg.end_point()
        assert isinstance(sp, tuple)
        assert len(sp) == 2
        assert isinstance(ep, tuple)
        assert len(ep) == 2

    def test_center_point(self) -> None:
        seg = le_lsd.LineSegment(0.0, 1.0, 5.0, -10.0, 10.0)
        cp = seg.center_point()
        assert isinstance(cp, tuple)
        assert len(cp) == 2

    def test_end_points_tuple(self) -> None:
        seg = le_lsd.LineSegment(0.0, 1.0, 5.0, -10.0, 10.0)
        eps = seg.end_points()
        assert isinstance(eps, tuple)
        assert len(eps) == 4

    def test_repr(self) -> None:
        seg = le_lsd.LineSegment(0.0, 1.0, 5.0, -10.0, 10.0)
        r = repr(seg)
        assert "LineSegment" in r
        assert "len=" in r

    def test_inherits_line(self) -> None:
        seg = le_lsd.LineSegment(1.0, 0.0, 3.0, -5.0, 5.0)
        # Should have Line properties
        assert seg.normal_x == pytest.approx(1.0)
        assert seg.valid()
        d = seg.distance(5.0, 0.0)
        assert d == pytest.approx(2.0)

    def test_f64_variant(self) -> None:
        seg = le_lsd.LineSegment_f64(0.0, 1.0, 5.0, -10.0, 10.0)
        assert seg.length == pytest.approx(20.0)


# ============================================================================
# Detector construction tests
# ============================================================================


class TestDetectorConstruction:
    """Test that all detectors can be constructed with defaults."""

    @pytest.mark.parametrize(
        "cls_name",
        [
            "LsdCC",
            "LsdCP",
            "LsdBurns",
            "LsdFBW",
            "LsdFGioi",
            "LsdEDLZ",
            "LsdEL",
            "LsdEP",
            "LsdHoughP",
        ],
    )
    def test_default_construction(self, cls_name: str) -> None:
        cls = getattr(le_lsd, cls_name)
        det = cls()
        assert det is not None

    @pytest.mark.parametrize(
        "cls_name",
        [
            "LsdCC_f64",
            "LsdCP_f64",
            "LsdBurns_f64",
            "LsdFBW_f64",
            "LsdFGioi_f64",
            "LsdEDLZ_f64",
            "LsdEL_f64",
            "LsdEP_f64",
            "LsdHoughP_f64",
        ],
    )
    def test_f64_construction(self, cls_name: str) -> None:
        cls = getattr(le_lsd, cls_name)
        det = cls()
        assert det is not None


# ============================================================================
# ValueManager integration tests
# ============================================================================


class TestValueManager:
    """Test ValueManager integration for detectors."""

    def test_lsd_cc_set_value(self) -> None:
        det = le_lsd.LsdCC()
        det.set_int("edge_min_pixels", 20)
        val = det.get_int("edge_min_pixels")
        assert val == 20

    def test_lsd_fgioi_set_value(self) -> None:
        det = le_lsd.LsdFGioi()
        det.set_double("angle_th", 30.0)
        val = det.get_double("angle_th")
        assert val == pytest.approx(30.0)

    def test_lsd_edlz_set_value(self) -> None:
        det = le_lsd.LsdEDLZ()
        det.set_int("min_len", 20)
        val = det.get_int("min_len")
        assert val == 20

    def test_lsd_houghp_set_value(self) -> None:
        det = le_lsd.LsdHoughP()
        det.set_double("hough_rho", 2.0)
        val = det.get_double("hough_rho")
        assert val == pytest.approx(2.0)


# ============================================================================
# Detection tests (default float preset)
# ============================================================================


class TestDetectionDefault:
    """Test line segment detection with default float preset."""

    @pytest.mark.parametrize(
        "cls_name",
        [
            "LsdCC",
            "LsdCP",
            "LsdBurns",
            "LsdFBW",
            "LsdFGioi",
            "LsdEDLZ",
            "LsdEL",
            "LsdEP",
            "LsdHoughP",
        ],
    )
    def test_detect_returns_segments(
        self, cls_name: str, square_image_u8: np.ndarray
    ) -> None:
        cls = getattr(le_lsd, cls_name)
        det = cls()
        det.detect(square_image_u8)

        segments = det.line_segments()
        assert isinstance(segments, list)
        # Square image should produce at least some line segments
        # (some detectors may find none depending on parameters)

    @pytest.mark.parametrize(
        "cls_name",
        [
            "LsdCC",
            "LsdCP",
            "LsdBurns",
            "LsdFBW",
            "LsdFGioi",
            "LsdEDLZ",
            "LsdEL",
            "LsdEP",
            "LsdHoughP",
        ],
    )
    def test_detect_end_points(
        self, cls_name: str, square_image_u8: np.ndarray
    ) -> None:
        cls = getattr(le_lsd, cls_name)
        det = cls()
        det.detect(square_image_u8)

        eps = det.end_points()
        assert isinstance(eps, list)

    @pytest.mark.parametrize(
        "cls_name",
        [
            "LsdCC",
            "LsdCP",
            "LsdBurns",
            "LsdFBW",
            "LsdFGioi",
            "LsdEDLZ",
            "LsdEL",
            "LsdEP",
            "LsdHoughP",
        ],
    )
    def test_detect_lines(self, cls_name: str, square_image_u8: np.ndarray) -> None:
        cls = getattr(le_lsd, cls_name)
        det = cls()
        det.detect(square_image_u8)

        lines = det.lines()
        assert isinstance(lines, list)


class TestDetectionQuality:
    """Test that detectors find reasonable segments on structured images."""

    def test_lsd_cc_finds_segments(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert len(segments) > 0, "LsdCC should detect segments in a square image"

    def test_lsd_fgioi_finds_segments(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdFGioi()
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert len(segments) > 0, "LsdFGioi should detect segments in a square image"

    def test_lsd_edlz_finds_segments(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdEDLZ()
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert len(segments) > 0, "LsdEDLZ should detect segments in a square image"

    def test_segment_properties(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(square_image_u8)
        segments = det.line_segments()
        if len(segments) > 0:
            seg = segments[0]
            assert isinstance(seg, le_lsd.LineSegment)
            assert seg.length > 0
            sp = seg.start_point()
            ep = seg.end_point()
            assert isinstance(sp, tuple)
            assert isinstance(ep, tuple)

    def test_endpoint_consistency(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(square_image_u8)
        segments = det.line_segments()
        endpoints = det.end_points()
        assert len(segments) == len(endpoints)


# ============================================================================
# Image data tests
# ============================================================================


class TestImageData:
    """Test auxiliary image data access."""

    def test_lsd_cc_image_data_descriptor(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(square_image_u8)
        desc = det.image_data_descriptor()
        assert isinstance(desc, list)
        assert len(desc) > 0
        assert isinstance(desc[0], le_lsd.DataDescriptorEntry)
        assert len(desc[0].name) > 0

    def test_lsd_cc_image_data(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(square_image_u8)
        data = det.image_data()
        assert isinstance(data, list)
        assert len(data) > 0
        # Each element should be a numpy array (cv::Mat)
        assert data[0].shape[0] > 0

    def test_lsd_edlz_image_data(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdEDLZ()
        det.detect(square_image_u8)
        desc = det.image_data_descriptor()
        data = det.image_data()
        assert len(desc) == len(data)


# ============================================================================
# Double precision (f64) preset tests
# ============================================================================


class TestF64Preset:
    """Test double-precision preset detectors."""

    def test_lsd_cc_f64(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC_f64()
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)
        if len(segments) > 0:
            assert isinstance(segments[0], le_lsd.LineSegment_f64)

    def test_lsd_fgioi_f64(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdFGioi_f64()
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)

    def test_lsd_edlz_f64(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdEDLZ_f64()
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)


# ============================================================================
# Custom parameter tests
# ============================================================================


class TestCustomParams:
    """Test detectors with custom parameters."""

    def test_lsd_cc_custom(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC(
            th_low=0.01, th_high=0.03, min_pix=5, max_gap=1, err_dist=3.0
        )
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)

    def test_lsd_burns_custom(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdBurns(th_low=0.01, th_high=0.03, min_pix=3, part_num=16)
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)

    def test_lsd_fgioi_custom(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdFGioi(
            quant=3.0, ang_th=30.0, log_eps=1.0, density_th=0.5, n_bins=512
        )
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)

    def test_lsd_edlz_custom(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdEDLZ(
            gradient_threshold=15.0, anchor_threshold=3.0, min_line_len=10
        )
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)

    def test_lsd_houghp_custom(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdHoughP(
            th_low=0.01, th_high=0.03, vote_threshold=50, min_length=5.0, max_gap=5.0
        )
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)

    def test_lsd_el_custom(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdEL(th_low=0.01, th_high=0.03, min_pix=5, dist=3.0, min_len=3)
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)

    def test_lsd_ep_custom(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdEP(th_low=0.01, th_high=0.03, min_pix=5, dist=3.0, min_len=3)
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)

    def test_lsd_cc_with_flags(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC(flags=le_lsd.CC_CORNER_RULE)
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)

    def test_lsd_fbw_with_flags(self, square_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdFBW(flags=le_lsd.FBW_NMS)
        det.detect(square_image_u8)
        segments = det.line_segments()
        assert isinstance(segments, list)
