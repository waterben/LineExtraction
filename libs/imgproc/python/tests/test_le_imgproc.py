"""Tests for le_imgproc Python bindings.

Verifies that the pybind11 bindings for FilterI, GradientI, LaplaceI,
and concrete gradient filters work correctly from Python.
"""

import numpy as np
import pytest

import le_imgproc


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

    def test_range_uchar(self) -> None:
        r = le_imgproc.RangeUChar(0, 255)
        assert r.lower == 0
        assert r.upper == 255
        assert r.size() == 255


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
        assert fd.data is None  # Empty Mat -> None

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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
