"""Tests for le_imgproc Python bindings.

Verifies that the pybind11 bindings for FilterI, GradientI, LaplaceI,
and concrete gradient filters work correctly from Python across all
type presets (uint8, uint16, float32, float64).
"""

import numpy as np
import pytest

import le_imgproc


# ============================================================================
# Core types (shared across all presets)
# ============================================================================


class TestRangeTypes:
    """Test Range type bindings."""

    def test_range_d_construction(self) -> None:
        r = le_imgproc.RangeD(0.0, 1.0)
        assert r.lower == pytest.approx(0.0)
        assert r.upper == pytest.approx(1.0)

    def test_range_d_size(self) -> None:
        r = le_imgproc.RangeD(-1.0, 1.0)
        assert r.size() == pytest.approx(2.0)

    def test_range_d_swap(self) -> None:
        r = le_imgproc.RangeD(0.0, 10.0)
        r.swap()
        assert r.lower == pytest.approx(10.0)
        assert r.upper == pytest.approx(0.0)

    def test_range_d_repr(self) -> None:
        r = le_imgproc.RangeD(0.0, 1.0)
        assert "RangeD" in repr(r)

    def test_range_i(self) -> None:
        r = le_imgproc.RangeI(-10, 10)
        assert r.lower == -10
        assert r.upper == 10
        assert r.size() == 20

    def test_range_f(self) -> None:
        r = le_imgproc.RangeF(0.0, 1.0)
        assert r.lower == pytest.approx(0.0)
        assert r.upper == pytest.approx(1.0)

    def test_range_s(self) -> None:
        r = le_imgproc.RangeS(-100, 100)
        assert r.lower == -100
        assert r.upper == 100
        assert r.size() == 200

    def test_range_uchar(self) -> None:
        r = le_imgproc.RangeUChar(0, 255)
        assert r.lower == 0
        assert r.upper == 255
        assert r.size() == 255

    def test_range_ushort(self) -> None:
        r = le_imgproc.RangeUShort(0, 65535)
        assert r.lower == 0
        assert r.upper == 65535
        assert r.size() == 65535


class TestValue:
    """Test Value type binding."""

    def test_value_int(self) -> None:
        v = le_imgproc.Value(42)
        assert v.get_int() == 42

    def test_value_float(self) -> None:
        v = le_imgproc.Value(3.14)
        assert v.get_float() == pytest.approx(3.14)

    def test_value_bool(self) -> None:
        v = le_imgproc.Value(True)
        assert v.get_bool() is True

    def test_value_string(self) -> None:
        v = le_imgproc.Value("hello")
        assert v.get_string() == "hello"

    def test_value_repr(self) -> None:
        v = le_imgproc.Value(42)
        assert "Value" in repr(v)


class TestFilterData:
    """Test FilterData binding."""

    def test_empty_construction(self) -> None:
        fd = le_imgproc.FilterData()
        # cvnp returns a 0x0 array for empty cv::Mat (shared memory semantics)
        assert fd.data.size == 0

    def test_construction_with_data(self) -> None:
        img = np.zeros((10, 10), dtype=np.uint8)
        fd = le_imgproc.FilterData(img, 0.0, 255.0)
        assert fd.data is not None
        assert fd.range.lower == pytest.approx(0.0)
        assert fd.range.upper == pytest.approx(255.0)

    def test_repr(self) -> None:
        img = np.zeros((10, 10), dtype=np.uint8)
        fd = le_imgproc.FilterData(img, 0.0, 255.0)
        assert "FilterData" in repr(fd)


# ============================================================================
# Default preset (uint8 / uchar) — no suffix
# ============================================================================


class TestSobelGradient:
    """Test SobelGradient (DerivativeGradient with Sobel) binding."""

    def test_construction(self) -> None:
        grad = le_imgproc.SobelGradient()
        assert grad.name() == "derivative_sobel"

    def test_construction_with_range(self) -> None:
        grad = le_imgproc.SobelGradient(0, 255)
        r = grad.intensity_range()
        assert r.lower == 0
        assert r.upper == 255

    def test_process_single_channel(self) -> None:
        grad = le_imgproc.SobelGradient()

        # Create image with vertical edge (64-wide for SIMD alignment)
        img = np.zeros((64, 64), dtype=np.uint8)
        img[:, 32:] = 255

        grad.process(img)

        mag = grad.magnitude()
        assert mag is not None
        assert mag.shape == (64, 64)
        assert mag.dtype == np.float32  # float magnitude

        direction = grad.direction()
        assert direction is not None
        assert direction.shape == (64, 64)
        assert direction.dtype == np.float32

    def test_gx_gy(self) -> None:
        grad = le_imgproc.SobelGradient()
        img = np.zeros((64, 64), dtype=np.uint8)
        img[:, 32:] = 255

        grad.process(img)

        gx = grad.gx()
        gy = grad.gy()
        assert gx is not None
        assert gy is not None
        assert gx.shape == (64, 64)
        assert gy.shape == (64, 64)

        # Vertical edge should have strong gx response
        assert np.max(np.abs(gx)) > 0

    def test_results_dict(self) -> None:
        grad = le_imgproc.SobelGradient()
        img = np.zeros((64, 64), dtype=np.uint8)
        img[:, 32:] = 255
        grad.process(img)

        results = grad.results()
        assert "gx" in results
        assert "gy" in results
        assert "mag" in results
        assert "dir" in results

        # Each result should be a FilterData
        assert isinstance(results["mag"], le_imgproc.FilterData)

    def test_magnitude_range(self) -> None:
        grad = le_imgproc.SobelGradient()
        mr = grad.magnitude_range()
        assert isinstance(mr, le_imgproc.RangeF)
        assert mr.lower == pytest.approx(0.0)
        assert mr.upper > 0

    def test_direction_range(self) -> None:
        grad = le_imgproc.SobelGradient()
        dr = grad.direction_range()
        assert isinstance(dr, le_imgproc.RangeF)

    def test_magnitude_threshold(self) -> None:
        grad = le_imgproc.SobelGradient()
        t = grad.magnitude_threshold(0.5)
        assert t > 0


class TestScharrGradient:
    """Test ScharrGradient binding."""

    def test_construction(self) -> None:
        grad = le_imgproc.ScharrGradient()
        assert grad.name() == "derivative_scharr"

    def test_process(self) -> None:
        grad = le_imgproc.ScharrGradient()
        img = np.zeros((64, 64), dtype=np.uint8)
        img[:, 32:] = 200
        grad.process(img)

        mag = grad.magnitude()
        assert mag is not None
        assert mag.shape == (64, 64)


class TestPrewittGradient:
    """Test PrewittGradient binding."""

    def test_construction(self) -> None:
        grad = le_imgproc.PrewittGradient()
        assert grad.name() == "derivative_prewitt"

    def test_process(self) -> None:
        grad = le_imgproc.PrewittGradient()
        img = np.zeros((64, 64), dtype=np.uint8)
        img[:, 32:] = 200
        grad.process(img)

        mag = grad.magnitude()
        assert mag is not None
        assert np.max(mag) > 0


class TestValueManager:
    """Test ValueManager parameter access via concrete filters."""

    def test_get_values(self) -> None:
        grad = le_imgproc.SobelGradient()
        vals = grad.values()
        assert isinstance(vals, dict)
        assert len(vals) > 0

    def test_get_set_value(self) -> None:
        grad = le_imgproc.SobelGradient()
        # Get kernel size parameter
        v = grad.get_value("grad_kernel_size")
        assert v is not None

        # Set new kernel size
        grad.set_int("grad_kernel_size", 5)
        v2 = grad.get_value("grad_kernel_size")
        assert v2.get_int() == 5


# ============================================================================
# Abstract interface existence (ensures trampolines registered per preset)
# ============================================================================


class TestAbstractInterfaces:
    """Test that abstract interfaces exist for all presets."""

    @pytest.mark.parametrize(
        "cls_name",
        ["FilterI", "FilterI_16u", "FilterI_f32", "FilterI_f64"],
    )
    def test_filter_interface_exists(self, cls_name: str) -> None:
        cls = getattr(le_imgproc, cls_name)
        assert cls is not None

    @pytest.mark.parametrize(
        "cls_name",
        ["GradientI", "GradientI_16u", "GradientI_f32", "GradientI_f64"],
    )
    def test_gradient_interface_exists(self, cls_name: str) -> None:
        cls = getattr(le_imgproc, cls_name)
        assert cls is not None

    @pytest.mark.parametrize(
        "cls_name",
        ["LaplaceI", "LaplaceI_16u", "LaplaceI_f32", "LaplaceI_f64"],
    )
    def test_laplace_interface_exists(self, cls_name: str) -> None:
        cls = getattr(le_imgproc, cls_name)
        assert cls is not None

    @pytest.mark.parametrize(
        "cls_name",
        [
            "GradientBase",
            "GradientBase_16u",
            "GradientBase_f32",
            "GradientBase_f64",
        ],
    )
    def test_gradient_base_exists(self, cls_name: str) -> None:
        cls = getattr(le_imgproc, cls_name)
        assert cls is not None


# ============================================================================
# Parametrised multi-type gradient tests
# ============================================================================


def _make_edge_image(dtype: np.dtype, high: float) -> np.ndarray:
    """Create a 64x64 image with a vertical edge at column 32."""
    img = np.zeros((64, 64), dtype=dtype)
    img[:, 32:] = high
    return img


class TestSobelGradientPresets:
    """Test SobelGradient across all type presets."""

    @pytest.mark.parametrize(
        "suffix, dtype, high",
        [
            ("", np.uint8, 255),
            ("_16u", np.uint16, 60000),
            ("_f32", np.float32, 1.0),
            ("_f64", np.float64, 1.0),
        ],
    )
    def test_construction(self, suffix: str, dtype: np.dtype, high: float) -> None:
        cls = getattr(le_imgproc, "SobelGradient" + suffix)
        grad = cls()
        assert grad.name() == "derivative_sobel"

    @pytest.mark.parametrize(
        "suffix, dtype, high",
        [
            ("", np.uint8, 255),
            ("_16u", np.uint16, 60000),
            ("_f32", np.float32, 1.0),
            ("_f64", np.float64, 1.0),
        ],
    )
    def test_process_and_results(
        self, suffix: str, dtype: np.dtype, high: float
    ) -> None:
        cls = getattr(le_imgproc, "SobelGradient" + suffix)
        grad = cls()
        img = _make_edge_image(dtype, high)

        grad.process(img)

        mag = grad.magnitude()
        assert mag is not None
        assert mag.shape == (64, 64)
        assert np.max(mag) > 0, f"No gradient response for {suffix}"

        direction = grad.direction()
        assert direction is not None
        assert direction.shape == (64, 64)

        gx = grad.gx()
        gy = grad.gy()
        assert gx.shape == (64, 64)
        assert gy.shape == (64, 64)

    @pytest.mark.parametrize(
        "suffix, dtype, high",
        [
            ("", np.uint8, 255),
            ("_16u", np.uint16, 60000),
            ("_f32", np.float32, 1.0),
            ("_f64", np.float64, 1.0),
        ],
    )
    def test_results_dict(self, suffix: str, dtype: np.dtype, high: float) -> None:
        cls = getattr(le_imgproc, "SobelGradient" + suffix)
        grad = cls()
        img = _make_edge_image(dtype, high)
        grad.process(img)

        results = grad.results()
        for key in ("gx", "gy", "mag", "dir"):
            assert key in results, f"Missing '{key}' in results for {suffix}"
            assert isinstance(results[key], le_imgproc.FilterData)

    @pytest.mark.parametrize(
        "suffix, dtype, high",
        [
            ("", np.uint8, 255),
            ("_f32", np.float32, 1.0),
        ],
    )
    def test_magnitude_range(self, suffix: str, dtype: np.dtype, high: float) -> None:
        cls = getattr(le_imgproc, "SobelGradient" + suffix)
        grad = cls()
        mr = grad.magnitude_range()
        assert mr.lower == pytest.approx(0.0)
        assert mr.upper > 0


class TestScharrGradientPresets:
    """Test ScharrGradient across all type presets."""

    @pytest.mark.parametrize(
        "suffix, dtype, high",
        [
            ("", np.uint8, 200),
            ("_16u", np.uint16, 50000),
            ("_f32", np.float32, 1.0),
            ("_f64", np.float64, 1.0),
        ],
    )
    def test_process(self, suffix: str, dtype: np.dtype, high: float) -> None:
        cls = getattr(le_imgproc, "ScharrGradient" + suffix)
        grad = cls()
        assert grad.name() == "derivative_scharr"

        img = _make_edge_image(dtype, high)
        grad.process(img)

        mag = grad.magnitude()
        assert mag is not None
        assert np.max(mag) > 0, f"No gradient response for ScharrGradient{suffix}"


class TestPrewittGradientPresets:
    """Test PrewittGradient across all type presets."""

    @pytest.mark.parametrize(
        "suffix, dtype, high",
        [
            ("", np.uint8, 200),
            ("_16u", np.uint16, 50000),
            ("_f32", np.float32, 1.0),
            ("_f64", np.float64, 1.0),
        ],
    )
    def test_process(self, suffix: str, dtype: np.dtype, high: float) -> None:
        cls = getattr(le_imgproc, "PrewittGradient" + suffix)
        grad = cls()
        assert grad.name() == "derivative_prewitt"

        img = _make_edge_image(dtype, high)
        grad.process(img)

        mag = grad.magnitude()
        assert mag is not None
        assert np.max(mag) > 0, f"No gradient response for PrewittGradient{suffix}"


# ============================================================================
# Float-preset specific tests (intensity range with floating point bounds)
# ============================================================================


class TestFloatPreset:
    """Test float32 preset construction with custom intensity range."""

    def test_sobel_f32_custom_range(self) -> None:
        grad = le_imgproc.SobelGradient_f32(0.0, 1.0)
        r = grad.intensity_range()
        assert r.lower == pytest.approx(0.0)
        assert r.upper == pytest.approx(1.0)

    def test_sobel_f64_custom_range(self) -> None:
        grad = le_imgproc.SobelGradient_f64(0.0, 1.0)
        r = grad.intensity_range()
        assert r.lower == pytest.approx(0.0)
        assert r.upper == pytest.approx(1.0)

    def test_sobel_16u_custom_range(self) -> None:
        grad = le_imgproc.SobelGradient_16u(0, 4095)
        r = grad.intensity_range()
        assert r.lower == 0
        assert r.upper == 4095

    def test_value_manager_on_f32(self) -> None:
        grad = le_imgproc.SobelGradient_f32()
        vals = grad.values()
        assert isinstance(vals, dict)
        assert len(vals) > 0

        grad.set_int("grad_kernel_size", 5)
        v = grad.get_value("grad_kernel_size")
        assert v.get_int() == 5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
