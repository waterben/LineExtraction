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


# ============================================================================
# Phase 2: Laplace filters
# ============================================================================


class TestLaplaceSimple:
    """Test LaplaceSimple binding."""

    def test_construction(self) -> None:
        lap = le_imgproc.LaplaceSimple()
        assert lap.name() != ""

    def test_process(self) -> None:
        # Default uchar/int preset: cv::filter2D does not support
        # CV_8U -> CV_32S, so use float input instead.
        lap = le_imgproc.LaplaceSimple_f32()
        img = np.random.rand(64, 64).astype(np.float32)
        lap.process(img)
        result = lap.laplace()
        assert result is not None
        assert result.shape == (64, 64)

    def test_laplace_range(self) -> None:
        lap = le_imgproc.LaplaceSimple()
        r = lap.laplace_range()
        assert r.lower <= r.upper

    def test_results(self) -> None:
        lap = le_imgproc.LaplaceSimple_f32()
        img = np.random.rand(32, 32).astype(np.float32)
        lap.process(img)
        res = lap.results()
        assert isinstance(res, dict)

    def test_f32_preset(self) -> None:
        lap = le_imgproc.LaplaceSimple_f32()
        img = np.random.rand(32, 32).astype(np.float32)
        lap.process(img)
        result = lap.laplace()
        assert result.dtype == np.float32

    def test_f64_preset(self) -> None:
        lap = le_imgproc.LaplaceSimple_f64()
        img = np.random.rand(32, 32).astype(np.float64)
        lap.process(img)
        result = lap.laplace()
        assert result.dtype == np.float64


class TestLoG:
    """Test Laplacian of Gaussian binding."""

    def test_default_construction(self) -> None:
        log = le_imgproc.LoG()
        assert log.name() != ""

    def test_parametrized_construction(self) -> None:
        log = le_imgproc.LoG(kernel_size=7, kernel_spacing=1.5)
        assert log.kernel_size == 7
        assert log.kernel_spacing == pytest.approx(1.5)

    def test_process(self) -> None:
        # Default uchar/int preset: cv::filter2D does not support
        # CV_8U -> CV_32S.  Use float preset instead.
        log = le_imgproc.LoG_f32()
        img = np.random.rand(64, 64).astype(np.float32)
        log.process(img)
        result = log.laplace()
        assert result is not None
        assert result.shape == (64, 64)

    def test_kernel(self) -> None:
        log = le_imgproc.LoG(kernel_size=5)
        k = log.kernel()
        assert k is not None
        assert k.ndim == 2

    def test_even_alias(self) -> None:
        log = le_imgproc.LoG_f32()
        img = np.random.rand(32, 32).astype(np.float32)
        log.process(img)
        even = log.even()
        lap = log.laplace()
        np.testing.assert_array_equal(even, lap)

    def test_property_setters(self) -> None:
        log = le_imgproc.LoG()
        log.kernel_size = 9
        assert log.kernel_size == 9
        log.kernel_spacing = 2.0
        assert log.kernel_spacing == pytest.approx(2.0)
        log.kernel_scale = 0.5
        assert log.kernel_scale == pytest.approx(0.5)

    def test_f32_preset(self) -> None:
        log = le_imgproc.LoG_f32()
        img = np.random.rand(32, 32).astype(np.float32)
        log.process(img)
        assert log.laplace() is not None


class TestLaplaceCV:
    """Test OpenCV-based Laplacian binding."""

    def test_default_construction(self) -> None:
        lap = le_imgproc.LaplaceCV()
        assert lap.name() != ""

    def test_parametrized_construction(self) -> None:
        lap = le_imgproc.LaplaceCV(ksize=3)
        assert lap.kernel_size == 3

    def test_process(self) -> None:
        lap = le_imgproc.LaplaceCV()
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        lap.process(img)
        result = lap.laplace()
        assert result is not None
        assert result.shape == (64, 64)

    def test_kernel_size_property(self) -> None:
        lap = le_imgproc.LaplaceCV(ksize=3)
        assert lap.kernel_size == 3
        lap.kernel_size = 7
        assert lap.kernel_size == 7

    def test_f32_preset(self) -> None:
        lap = le_imgproc.LaplaceCV_f32()
        img = np.random.rand(32, 32).astype(np.float32)
        lap.process(img)
        assert lap.laplace() is not None


# ============================================================================
# Phase 2: Extra gradients (Susan, RCMG)
# ============================================================================


class TestSusanGradient:
    """Test SUSAN gradient binding."""

    def test_default_construction(self) -> None:
        susan = le_imgproc.SusanGradient()
        assert susan.name() != ""

    def test_parametrized_construction(self) -> None:
        susan = le_imgproc.SusanGradient(brightness_th=30, small_kernel=True)
        assert susan is not None

    def test_process(self) -> None:
        susan = le_imgproc.SusanGradient()
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        susan.process(img)
        mag = susan.magnitude()
        assert mag is not None
        assert mag.shape == (64, 64)
        direction = susan.direction()
        assert direction is not None
        assert direction.shape == (64, 64)

    def test_gradient_components(self) -> None:
        susan = le_imgproc.SusanGradient()
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        susan.process(img)
        gx = susan.gx()
        gy = susan.gy()
        assert gx is not None
        assert gy is not None
        assert gx.shape == gy.shape

    def test_ranges(self) -> None:
        susan = le_imgproc.SusanGradient()
        assert susan.magnitude_range().lower <= susan.magnitude_range().upper
        assert susan.direction_range().lower <= susan.direction_range().upper
        assert susan.gradient_range().lower <= susan.gradient_range().upper

    def test_value_manager(self) -> None:
        susan = le_imgproc.SusanGradient()
        vals = susan.values()
        assert isinstance(vals, dict)


class TestRCMGradientColor:
    """Test RCMG 3-channel gradient binding."""

    def test_default_construction(self) -> None:
        rcmg = le_imgproc.RCMGradientColor()
        assert rcmg.name() != ""

    def test_process(self) -> None:
        rcmg = le_imgproc.RCMGradientColor()
        img = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        rcmg.process(img)
        mag = rcmg.magnitude()
        assert mag is not None
        assert mag.shape[:2] == (64, 64)

    def test_gradient_components(self) -> None:
        rcmg = le_imgproc.RCMGradientColor()
        img = np.random.randint(0, 256, (32, 32, 3), dtype=np.uint8)
        rcmg.process(img)
        assert rcmg.gx() is not None
        assert rcmg.gy() is not None

    def test_value_manager(self) -> None:
        rcmg = le_imgproc.RCMGradientColor()
        vals = rcmg.values()
        assert isinstance(vals, dict)


class TestRCMGradientGray:
    """Test RCMG single-channel gradient binding."""

    def test_default_construction(self) -> None:
        rcmg = le_imgproc.RCMGradient()
        assert rcmg.name() != ""

    def test_process(self) -> None:
        rcmg = le_imgproc.RCMGradient()
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        rcmg.process(img)
        mag = rcmg.magnitude()
        assert mag is not None
        assert mag.shape == (64, 64)

    def test_f32_preset(self) -> None:
        rcmg = le_imgproc.RCMGradient_f32()
        img = np.random.rand(32, 32).astype(np.float32)
        rcmg.process(img)
        assert rcmg.magnitude() is not None

    def test_f64_preset(self) -> None:
        rcmg = le_imgproc.RCMGradient_f64()
        img = np.random.rand(32, 32).astype(np.float64)
        rcmg.process(img)
        assert rcmg.magnitude() is not None


# ============================================================================
# Phase 2: Image operators
# ============================================================================


class TestImageOperatorBase:
    """Test ImageOperator base class and Python subclassing."""

    def test_python_subclass(self) -> None:
        class InvertOp(le_imgproc.ImageOperator):
            def __init__(self) -> None:
                super().__init__("invert")

            def apply(self, img: np.ndarray) -> None:
                img[:] = 255 - img

        op = InvertOp()
        assert op.name() == "invert"
        img = np.full((10, 10), 100, dtype=np.uint8)
        op.apply(img)
        assert img[0, 0] == 155


class TestNoOp:
    """Test NoOp operator."""

    def test_apply(self) -> None:
        op = le_imgproc.NoOp()
        img = np.ones((10, 10), dtype=np.uint8) * 42
        original = img.copy()
        op.apply(img)
        np.testing.assert_array_equal(img, original)


class TestResizeOperator:
    """Test ResizeOperator."""

    def test_resize(self) -> None:
        op = le_imgproc.ResizeOperator(32, 32)
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        op.apply(img)
        # Note: apply modifies in-place; we can also use apply_copy
        result = op.apply_copy(np.random.randint(0, 256, (64, 64), dtype=np.uint8))
        assert result.shape == (32, 32)


class TestBlurOperators:
    """Test blur operator variants."""

    def test_box_blur(self) -> None:
        op = le_imgproc.BlurOperator(3)
        img = np.random.randint(0, 256, (32, 32), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (32, 32)

    def test_gaussian_blur_sigma(self) -> None:
        op = le_imgproc.GaussianBlurOperator(sigma=1.5)
        img = np.random.randint(0, 256, (32, 32), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (32, 32)

    def test_gaussian_blur_ksize(self) -> None:
        op = le_imgproc.GaussianBlurOperator(kernel_size=5)
        img = np.random.randint(0, 256, (32, 32), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (32, 32)

    def test_median_blur(self) -> None:
        op = le_imgproc.MedianBlurOperator(3)
        img = np.random.randint(0, 256, (32, 32), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (32, 32)

    def test_bilateral(self) -> None:
        op = le_imgproc.BilateralOperator()
        img = np.random.randint(0, 256, (32, 32), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (32, 32)


class TestNoiseOperators:
    """Test noise generation operators."""

    def test_uniform_noise(self) -> None:
        op = le_imgproc.UniformNoiseOperator(0.0, 50.0)
        img = np.zeros((32, 32), dtype=np.uint8)
        # apply() internally re-allocates, so use apply_copy
        result = op.apply_copy(img)
        assert np.any(result > 0)

    def test_gaussian_noise(self) -> None:
        op = le_imgproc.GaussianNoiseOperator(sigma=25.0)
        img = np.full((100, 100), 128, dtype=np.uint8)
        result = op.apply_copy(img)
        assert not np.array_equal(result, img)


class TestPipelineOperator:
    """Test PipelineOperator for chaining."""

    def test_push_and_apply(self) -> None:
        pipeline = le_imgproc.PipelineOperator()
        pipeline.push(le_imgproc.BlurOperator(3))
        pipeline.push(le_imgproc.MedianBlurOperator(3))
        img = np.random.randint(0, 256, (32, 32), dtype=np.uint8)
        result = pipeline.apply_copy(img)
        assert result.shape == (32, 32)

    def test_clear(self) -> None:
        pipeline = le_imgproc.PipelineOperator()
        pipeline.push(le_imgproc.NoOp())
        pipeline.clear()
        # After clear, pipeline should be empty (apply = identity)
        img = np.ones((10, 10), dtype=np.uint8) * 42
        result = pipeline.apply_copy(img)
        np.testing.assert_array_equal(result, img)


class TestGeometricOperators:
    """Test geometric transform operators (float and double)."""

    def test_rotate(self) -> None:
        # float variants use no suffix; double uses _f64
        op = le_imgproc.RotateOperator(angle=0.1)
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (64, 64)

    def test_rotate_f64(self) -> None:
        op = le_imgproc.RotateOperator_f64(angle=0.1)
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (64, 64)

    def test_scale(self) -> None:
        op = le_imgproc.ScaleOperator(scale=1.5)
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result is not None

    def test_translate(self) -> None:
        op = le_imgproc.TranslateOperator(dx=5.0, dy=3.0)
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (64, 64)

    def test_translate_f64(self) -> None:
        op = le_imgproc.TranslateOperator_f64(dx=5.0, dy=3.0)
        img = np.random.randint(0, 256, (64, 64), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (64, 64)


class TestFastNlMeansOperator:
    """Test FastNlMeans denoising operator (requires OpenCV photo module)."""

    def test_construction_and_apply(self) -> None:
        if not hasattr(le_imgproc, "FastNlMeansOperator"):
            pytest.skip("FastNlMeansOperator not available (no photo module)")
        op = le_imgproc.FastNlMeansOperator()
        img = np.random.randint(0, 256, (32, 32), dtype=np.uint8)
        result = op.apply_copy(img)
        assert result.shape == (32, 32)
