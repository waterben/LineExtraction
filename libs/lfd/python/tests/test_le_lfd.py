"""Integration tests for the le_lfd Python extension module.

Tests cover:
  - Module import and core types
  - FeatureMatch / DescriptorMatch bindings
  - FdMat / FdLBD descriptor types
  - FdcLBD descriptor creator (gradient input)
  - FdcGenericLR descriptor creator
  - BruteForceLBD / BruteForceLR matchers
  - GlobalRotationFilter
  - StereoLineFilter
  - Multi-preset support (float default, double _f64)
"""

from __future__ import annotations

import numpy as np
import pytest

import le_lfd
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


def _sobel_x(img: np.ndarray) -> np.ndarray:
    """Compute horizontal Sobel gradient using numpy convolution."""
    kernel = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=img.dtype)
    from numpy.lib.stride_tricks import sliding_window_view

    padded = np.pad(img, 1, mode="edge")
    windows = sliding_window_view(padded, (3, 3))
    return np.einsum("ijkl,kl->ij", windows, kernel).astype(img.dtype)


def _sobel_y(img: np.ndarray) -> np.ndarray:
    """Compute vertical Sobel gradient using numpy convolution."""
    kernel = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=img.dtype)
    from numpy.lib.stride_tricks import sliding_window_view

    padded = np.pad(img, 1, mode="edge")
    windows = sliding_window_view(padded, (3, 3))
    return np.einsum("ijkl,kl->ij", windows, kernel).astype(img.dtype)


def _detect_and_get_gradients(
    img: np.ndarray,
) -> tuple[list, np.ndarray, np.ndarray]:
    """Detect line segments and compute Sobel gradients.

    Returns:
        Tuple of (line_segments, gx, gy).
    """
    det = le_lsd.LsdCC()
    det.detect(img)
    segments = det.line_segments()

    # Compute gradients using numpy Sobel
    img_f = img.astype(np.float32) / 255.0
    gx = _sobel_x(img_f)
    gy = _sobel_y(img_f)
    return segments, gx, gy


# ============================================================================
# Module-level tests
# ============================================================================


class TestModuleImport:
    """Test that the module and its core types are importable."""

    def test_import_module(self) -> None:
        assert hasattr(le_lfd, "FdcLBD")
        assert hasattr(le_lfd, "BruteForceLBD")
        assert hasattr(le_lfd, "GlobalRotationFilter")

    def test_core_constants(self) -> None:
        assert le_lfd.FS_NONE == 0
        assert le_lfd.FS_MASKED != 0

    def test_core_types(self) -> None:
        assert hasattr(le_lfd, "FeatureMatch")
        assert hasattr(le_lfd, "DescriptorMatch")
        assert hasattr(le_lfd, "FdMat")
        assert hasattr(le_lfd, "FdLBD")

    def test_re_exported_geometry(self) -> None:
        assert hasattr(le_lfd, "Line")
        assert hasattr(le_lfd, "LineSegment")

    def test_f64_preset_exists(self) -> None:
        assert hasattr(le_lfd, "FdcLBD_f64")
        assert hasattr(le_lfd, "BruteForceLBD_f64")
        assert hasattr(le_lfd, "GlobalRotationFilter_f64")
        assert hasattr(le_lfd, "FeatureMatch_f64")


# ============================================================================
# FeatureMatch / DescriptorMatch tests
# ============================================================================


class TestFeatureMatch:
    """Test FeatureMatch struct bindings."""

    def test_default_construction(self) -> None:
        m = le_lfd.FeatureMatch()
        assert m.query_idx == -1
        assert m.match_idx == -1
        assert m.filter_state == le_lfd.FS_NONE

    def test_construction_with_args(self) -> None:
        m = le_lfd.FeatureMatch(query_idx=3, match_idx=7, filter_state=le_lfd.FS_MASKED)
        assert m.query_idx == 3
        assert m.match_idx == 7
        assert m.filter_state == le_lfd.FS_MASKED

    def test_repr(self) -> None:
        m = le_lfd.FeatureMatch(query_idx=1, match_idx=2)
        r = repr(m)
        assert "FeatureMatch" in r
        assert "1" in r
        assert "2" in r


class TestDescriptorMatch:
    """Test DescriptorMatch struct bindings."""

    def test_default_construction(self) -> None:
        m = le_lfd.DescriptorMatch()
        assert m.query_idx == -1
        assert m.match_idx == -1
        assert m.distance > 1e30  # max float

    def test_construction_with_distance(self) -> None:
        m = le_lfd.DescriptorMatch(query_idx=0, match_idx=1, distance=0.5)
        assert m.query_idx == 0
        assert m.match_idx == 1
        assert abs(m.distance - 0.5) < 1e-6

    def test_inherits_feature_match(self) -> None:
        m = le_lfd.DescriptorMatch(query_idx=0, match_idx=1, distance=1.0)
        assert m.filter_state == le_lfd.FS_NONE

    def test_ordering(self) -> None:
        a = le_lfd.DescriptorMatch(query_idx=0, match_idx=0, distance=1.0)
        b = le_lfd.DescriptorMatch(query_idx=0, match_idx=1, distance=2.0)
        assert a < b

    def test_repr(self) -> None:
        m = le_lfd.DescriptorMatch(query_idx=0, match_idx=1, distance=0.5)
        r = repr(m)
        assert "DescriptorMatch" in r


# ============================================================================
# FdMat / FdLBD descriptor type tests
# ============================================================================


class TestFdMat:
    """Test FdMat descriptor bindings."""

    def test_default_construction(self) -> None:
        d = le_lfd.FdMat()
        assert d.data is not None

    def test_from_numpy(self) -> None:
        arr = np.random.randn(1, 32).astype(np.float32)
        d = le_lfd.FdMat(arr)
        assert d.data.shape == (1, 32)

    def test_name(self) -> None:
        d = le_lfd.FdMat()
        assert isinstance(d.name(), str)


class TestFdLBD:
    """Test FdLBD descriptor bindings."""

    def test_default_construction(self) -> None:
        d = le_lfd.FdLBD()
        assert d.data is not None

    def test_from_numpy(self) -> None:
        arr = np.random.randn(1, 72).astype(np.float32)
        d = le_lfd.FdLBD(arr)
        assert d.data.shape == (1, 72)

    def test_name_is_lbd(self) -> None:
        d = le_lfd.FdLBD()
        assert "LBD" in d.name()

    def test_distance(self) -> None:
        a_data = np.ones((1, 72), dtype=np.float32)
        b_data = np.zeros((1, 72), dtype=np.float32)
        a = le_lfd.FdLBD(a_data)
        b = le_lfd.FdLBD(b_data)
        dist = a.distance(b)
        assert dist > 0.0

    def test_distance_self_is_zero(self) -> None:
        d_data = np.ones((1, 72), dtype=np.float32)
        d = le_lfd.FdLBD(d_data)
        assert abs(d.distance(d)) < 1e-6


# ============================================================================
# FdcLBD (LBD descriptor creator) tests
# ============================================================================


class TestFdcLBD:
    """Test LBD descriptor creator bindings."""

    def test_construction_from_gradients(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        assert len(segments) > 0
        creator = le_lfd.FdcLBD(gx, gy)
        assert creator.size() > 0

    def test_create_list(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        creator = le_lfd.FdcLBD(gx, gy)
        descs = creator.create_list(segments)
        assert len(descs) == len(segments)
        assert all(isinstance(d, le_lfd.FdLBD) for d in descs)

    def test_create_mat(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        creator = le_lfd.FdcLBD(gx, gy)
        mat = creator.create_mat(segments)
        assert isinstance(mat, np.ndarray)
        assert mat.shape[0] == len(segments)

    def test_create_single(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        creator = le_lfd.FdcLBD(gx, gy)
        desc = creator.create_single(segments[0])
        assert isinstance(desc, le_lfd.FdLBD)

    def test_custom_band_params(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        creator = le_lfd.FdcLBD(gx, gy, num_band=5, width_band=5)
        descs = creator.create_list(segments)
        assert len(descs) == len(segments)

    def test_construction_from_dict(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        data = {"gx": gx, "gy": gy}
        creator = le_lfd.FdcLBD(data)
        descs = creator.create_list(segments)
        assert len(descs) > 0

    def test_create_list_masked(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        creator = le_lfd.FdcLBD(gx, gy)
        # Mask: include only first half
        mask = [1 if i < len(segments) // 2 else 0 for i in range(len(segments))]
        descs = creator.create_list_masked(segments, mask)
        assert len(descs) > 0


# ============================================================================
# FdcGenericLR (LR descriptor creator) tests
# ============================================================================


class TestFdcGenericLR:
    """Test LR descriptor creator bindings."""

    def test_construction_from_dict(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        img_f = square_image_u8.astype(np.float32) / 255.0
        data = {"gx": gx, "gy": gy, "img": img_f}
        creator = le_lfd.FdcGenericLR(data)
        assert creator.size() > 0

    def test_create_list(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        img_f = square_image_u8.astype(np.float32) / 255.0
        data = {"gx": gx, "gy": gy, "img": img_f}
        creator = le_lfd.FdcGenericLR(data)
        descs = creator.create_list(segments)
        assert len(descs) == len(segments)

    def test_create_mat(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        img_f = square_image_u8.astype(np.float32) / 255.0
        data = {"gx": gx, "gy": gy, "img": img_f}
        creator = le_lfd.FdcGenericLR(data)
        mat = creator.create_mat(segments)
        assert isinstance(mat, np.ndarray)
        assert mat.shape[0] == len(segments)


# ============================================================================
# BruteForceLBD matcher tests
# ============================================================================


class TestBruteForceLBD:
    """Test brute force LBD matcher bindings."""

    def test_default_construction(self) -> None:
        matcher = le_lfd.BruteForceLBD()
        assert matcher is not None

    def test_train_and_best(self, lines_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(lines_image_u8)
        if len(segments) < 2:
            pytest.skip("Not enough segments for matching")

        creator = le_lfd.FdcLBD(gx, gy)
        descs = creator.create_list(segments)

        # Self-match: best match distances should be zero (or NaN for degenerate
        # descriptors). Some descriptors may be identical across different segments,
        # so we only check distance, not index equality.
        matcher = le_lfd.BruteForceLBD()
        matcher.train(descs, descs)
        best = matcher.best()
        assert len(best) == len(descs)
        for m in best:
            assert isinstance(m, le_lfd.DescriptorMatch)
            if not np.isnan(m.distance):
                assert m.distance < 1e-6

    def test_knn(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        if len(segments) < 3:
            pytest.skip("Not enough segments for kNN")

        creator = le_lfd.FdcLBD(gx, gy)
        descs = creator.create_list(segments)

        matcher = le_lfd.BruteForceLBD()
        matcher.train(descs, descs)
        knn_matches = matcher.knn(2)
        assert len(knn_matches) > 0

    def test_static_match_best(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        if len(segments) < 2:
            pytest.skip("Not enough segments")

        creator = le_lfd.FdcLBD(gx, gy)
        descs = creator.create_list(segments)

        best = le_lfd.BruteForceLBD.match_best(descs, descs)
        assert len(best) == len(descs)


# ============================================================================
# BruteForceLR matcher tests
# ============================================================================


class TestBruteForceLR:
    """Test brute force LR matcher bindings."""

    def test_default_construction(self) -> None:
        matcher = le_lfd.BruteForceLR()
        assert matcher is not None

    def test_train_and_best(self, square_image_u8: np.ndarray) -> None:
        segments, gx, gy = _detect_and_get_gradients(square_image_u8)
        if len(segments) < 2:
            pytest.skip("Not enough segments")

        img_f = square_image_u8.astype(np.float32) / 255.0
        data = {"gx": gx, "gy": gy, "img": img_f}
        creator = le_lfd.FdcGenericLR(data)
        descs = creator.create_list(segments)

        matcher = le_lfd.BruteForceLR()
        matcher.train(descs, descs)
        best = matcher.best()
        assert len(best) == len(descs)


# ============================================================================
# GlobalRotationFilter tests
# ============================================================================


class TestGlobalRotationFilter:
    """Test GlobalRotationFilter bindings."""

    def test_default_construction(self) -> None:
        f = le_lfd.GlobalRotationFilter()
        assert f is not None

    def test_train_and_filter(self, lines_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(lines_image_u8)
        segments = det.line_segments()
        if len(segments) < 2:
            pytest.skip("Not enough segments for filter test")

        grf = le_lfd.GlobalRotationFilter()
        grf.train(segments, segments)

        # At least some pairs should pass the filter
        passed = 0
        for i in range(min(len(segments), 5)):
            for j in range(min(len(segments), 5)):
                if not grf.filter(i, j):
                    passed += 1
        assert passed > 0

    def test_create_matches(self, lines_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(lines_image_u8)
        segments = det.line_segments()
        if len(segments) < 2:
            pytest.skip("Not enough segments")

        grf = le_lfd.GlobalRotationFilter()
        grf.train(segments, segments)
        matches = grf.create_matches(len(segments), len(segments))
        assert len(matches) > 0
        assert all(isinstance(m, le_lfd.FeatureMatch) for m in matches)

    def test_create_matches_with_masks(self, lines_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(lines_image_u8)
        segments = det.line_segments()
        if len(segments) < 2:
            pytest.skip("Not enough segments")

        grf = le_lfd.GlobalRotationFilter()
        grf.train(segments, segments)
        result = grf.create_matches_with_masks(len(segments), len(segments))
        assert len(result) == 3
        matches, lm, rm = result
        assert len(matches) > 0
        assert len(lm) == len(segments)
        assert len(rm) == len(segments)

    def test_filter_list(self, lines_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(lines_image_u8)
        segments = det.line_segments()
        if len(segments) < 2:
            pytest.skip("Not enough segments")

        grf = le_lfd.GlobalRotationFilter()
        grf.train(segments, segments)
        matches = grf.create_matches(len(segments), len(segments))
        filtered = grf.filter_list(matches)
        assert len(filtered) == len(matches)


# ============================================================================
# StereoLineFilter tests
# ============================================================================


class TestStereoLineFilter:
    """Test StereoLineFilter bindings."""

    def test_construction(self) -> None:
        f = le_lfd.StereoLineFilter(height=480)
        assert f is not None

    def test_construction_with_params(self) -> None:
        f = le_lfd.StereoLineFilter(
            height=480, max_dis_px=5000.0, angle_th=30.0, min_y_overlap=0.3
        )
        assert f is not None

    def test_train_and_filter(self, lines_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(lines_image_u8)
        segments = det.line_segments()
        if len(segments) < 2:
            pytest.skip("Not enough segments")

        h = lines_image_u8.shape[0]
        slf = le_lfd.StereoLineFilter(height=h)
        slf.train(segments, segments)

        # Check that filter works for some pairs
        passed = 0
        for i in range(min(len(segments), 5)):
            for j in range(min(len(segments), 5)):
                if not slf.filter(i, j):
                    passed += 1
        assert passed > 0

    def test_create_matches(self, lines_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(lines_image_u8)
        segments = det.line_segments()
        if len(segments) < 2:
            pytest.skip("Not enough segments")

        h = lines_image_u8.shape[0]
        slf = le_lfd.StereoLineFilter(height=h)
        slf.train(segments, segments)
        matches = slf.create_matches(len(segments), len(segments))
        assert isinstance(matches, list)


# ============================================================================
# Filtered matching tests (integration)
# ============================================================================


class TestFilteredMatching:
    """Test combined filter + matcher workflow."""

    def test_brute_force_with_global_rotation_filter(
        self, lines_image_u8: np.ndarray
    ) -> None:
        det = le_lsd.LsdCC()
        det.detect(lines_image_u8)
        segments = det.line_segments()
        if len(segments) < 2:
            pytest.skip("Not enough segments")

        img_f = lines_image_u8.astype(np.float32) / 255.0
        gx = _sobel_x(img_f)
        gy = _sobel_y(img_f)

        creator = le_lfd.FdcLBD(gx, gy)
        descs = creator.create_list(segments)

        grf = le_lfd.GlobalRotationFilter()
        grf.train(segments, segments)

        matcher = le_lfd.BruteForceLBD()
        matcher.train_filtered(descs, descs, grf)
        best = matcher.best()
        assert len(best) > 0

    def test_brute_force_with_stereo_filter(self, lines_image_u8: np.ndarray) -> None:
        det = le_lsd.LsdCC()
        det.detect(lines_image_u8)
        segments = det.line_segments()
        if len(segments) < 2:
            pytest.skip("Not enough segments")

        img_f = lines_image_u8.astype(np.float32) / 255.0
        gx = _sobel_x(img_f)
        gy = _sobel_y(img_f)

        creator = le_lfd.FdcLBD(gx, gy)
        descs = creator.create_list(segments)

        h = lines_image_u8.shape[0]
        slf = le_lfd.StereoLineFilter(height=h)
        slf.train(segments, segments)

        matcher = le_lfd.BruteForceLBD()
        matcher.train_filtered_stereo(descs, descs, slf)
        best = matcher.best()
        assert len(best) > 0


# ============================================================================
# Double precision (f64) preset tests
# ============================================================================


class TestF64Preset:
    """Test double-precision preset types."""

    def test_feature_match_f64(self) -> None:
        m = le_lfd.FeatureMatch_f64(query_idx=1, match_idx=2)
        assert m.query_idx == 1

    def test_descriptor_match_f64(self) -> None:
        m = le_lfd.DescriptorMatch_f64(query_idx=0, match_idx=1, distance=1.0)
        assert abs(m.distance - 1.0) < 1e-12

    def test_fdlbd_f64(self) -> None:
        arr = np.random.randn(1, 72).astype(np.float64)
        d = le_lfd.FdLBD_f64(arr)
        assert "LBD" in d.name()

    def test_fdc_lbd_f64(self) -> None:
        img = np.zeros((100, 100), dtype=np.uint8)
        img[30:70, 30:70] = 255
        img_f = img.astype(np.float64) / 255.0
        gx = _sobel_x(img_f).astype(np.float64)
        gy = _sobel_y(img_f).astype(np.float64)

        det = le_lsd.LsdCC_f64()
        det.detect(img)
        segments = det.line_segments()
        if len(segments) < 1:
            pytest.skip("No segments")

        creator = le_lfd.FdcLBD_f64(gx, gy)
        descs = creator.create_list(segments)
        assert len(descs) == len(segments)

    def test_global_rotation_filter_f64(self) -> None:
        f = le_lfd.GlobalRotationFilter_f64()
        assert f is not None

    def test_brute_force_lbd_f64(self) -> None:
        matcher = le_lfd.BruteForceLBD_f64()
        assert matcher is not None


# ============================================================================
# Entry point
# ============================================================================

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
