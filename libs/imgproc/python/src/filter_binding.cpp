/// @file filter_binding.cpp
/// @brief pybind11 bindings for the imgproc filter interface hierarchy.
///
/// Binds core types (Range, Value, ValueManager, FilterData) and the abstract
/// filter interfaces (FilterI, GradientI, LaplaceI) along with concrete
/// implementations (DerivativeGradient with Sobel/Scharr/Prewitt).

#include "filter_binding.hpp"

#include <cvnp/cvnp.h>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/filter.hpp>
#include <imgproc/gradient.hpp>
#include <imgproc/laplace.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <utility/range.hpp>
#include <utility/value.hpp>
#include <utility/value_manager.hpp>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Trampoline classes for abstract interfaces
// ============================================================================

// Type aliases to avoid PYBIND11_COMMA issues in PYBIND11_OVERRIDE macros
using FilterI_uchar = FilterI<uchar>;
using GradientI_uchar = GradientI<uchar, short, float, float>;
using LaplaceI_uchar = LaplaceI<uchar, int>;

/// @brief Trampoline class enabling Python subclassing of FilterI<uchar>.
class PyFilterI : public FilterI_uchar {
 public:
  using FilterI_uchar::FilterI;

  Range<uchar> intensityRange() const override { PYBIND11_OVERRIDE_PURE(Range<uchar>, FilterI_uchar, intensityRange); }

  void process(const cv::Mat& img) override { PYBIND11_OVERRIDE_PURE(void, FilterI_uchar, process, img); }

  FilterResults results() const override { PYBIND11_OVERRIDE_PURE(FilterResults, FilterI_uchar, results); }

  std::string name() const override { PYBIND11_OVERRIDE_PURE(std::string, FilterI_uchar, name); }
};

/// @brief Trampoline for GradientI<uchar, short, int, float>.
class PyGradientI : public GradientI_uchar {
 public:
  using GradientI_uchar::GradientI;

  Range<uchar> intensityRange() const override {
    PYBIND11_OVERRIDE_PURE(Range<uchar>, GradientI_uchar, intensityRange);
  }

  void process(const cv::Mat& img) override { PYBIND11_OVERRIDE_PURE(void, GradientI_uchar, process, img); }

  FilterResults results() const override { PYBIND11_OVERRIDE_PURE(FilterResults, GradientI_uchar, results); }

  std::string name() const override { PYBIND11_OVERRIDE_PURE(std::string, GradientI_uchar, name); }

  cv::Mat magnitude() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, GradientI_uchar, magnitude); }

  Range<float> magnitudeRange() const override {
    PYBIND11_OVERRIDE_PURE(Range<float>, GradientI_uchar, magnitudeRange);
  }

  cv::Mat direction() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, GradientI_uchar, direction); }

  Range<float> directionRange() const override {
    PYBIND11_OVERRIDE_PURE(Range<float>, GradientI_uchar, directionRange);
  }
};

/// @brief Trampoline for LaplaceI<uchar, int>.
class PyLaplaceI : public LaplaceI_uchar {
 public:
  using LaplaceI_uchar::LaplaceI;

  Range<uchar> intensityRange() const override { PYBIND11_OVERRIDE_PURE(Range<uchar>, LaplaceI_uchar, intensityRange); }

  void process(const cv::Mat& img) override { PYBIND11_OVERRIDE_PURE(void, LaplaceI_uchar, process, img); }

  FilterResults results() const override { PYBIND11_OVERRIDE_PURE(FilterResults, LaplaceI_uchar, results); }

  std::string name() const override { PYBIND11_OVERRIDE_PURE(std::string, LaplaceI_uchar, name); }

  cv::Mat laplace() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, LaplaceI_uchar, laplace); }

  Range<int> laplaceRange() const override { PYBIND11_OVERRIDE_PURE(Range<int>, LaplaceI_uchar, laplaceRange); }
};

// ============================================================================
// Core type bindings
// ============================================================================

void bind_core_types(py::module_& m) {
  // --- Range<double> ---
  py::class_<Range<double>>(m, "RangeD", "Value range with double bounds.")
      .def(py::init<double, double>(), py::arg("lower") = 0.0, py::arg("upper") = 0.0)
      .def_readwrite("lower", &Range<double>::lower, "Lower bound.")
      .def_readwrite("upper", &Range<double>::upper, "Upper bound.")
      .def("size", &Range<double>::size, "Absolute size of range.")
      .def("swap", &Range<double>::swap, "Swap lower and upper bounds.")
      .def("__repr__", [](const Range<double>& r) {
        return "RangeD(lower=" + std::to_string(r.lower) + ", upper=" + std::to_string(r.upper) + ")";
      });

  // --- Range<int> ---
  py::class_<Range<int>>(m, "RangeI", "Value range with integer bounds.")
      .def(py::init<int, int>(), py::arg("lower") = 0, py::arg("upper") = 0)
      .def_readwrite("lower", &Range<int>::lower)
      .def_readwrite("upper", &Range<int>::upper)
      .def("size", &Range<int>::size)
      .def("swap", &Range<int>::swap)
      .def("__repr__", [](const Range<int>& r) {
        return "RangeI(lower=" + std::to_string(r.lower) + ", upper=" + std::to_string(r.upper) + ")";
      });

  // --- Range<float> ---
  py::class_<Range<float>>(m, "RangeF", "Value range with float bounds.")
      .def(py::init<float, float>(), py::arg("lower") = 0.0f, py::arg("upper") = 0.0f)
      .def_readwrite("lower", &Range<float>::lower)
      .def_readwrite("upper", &Range<float>::upper)
      .def("size", &Range<float>::size)
      .def("swap", &Range<float>::swap)
      .def("__repr__", [](const Range<float>& r) {
        return "RangeF(lower=" + std::to_string(r.lower) + ", upper=" + std::to_string(r.upper) + ")";
      });

  // --- Range<uchar> ---
  py::class_<Range<uchar>>(m, "RangeUChar", "Value range with uchar bounds (0-255).")
      .def(py::init<uchar, uchar>(), py::arg("lower") = 0, py::arg("upper") = 0)
      .def_readwrite("lower", &Range<uchar>::lower)
      .def_readwrite("upper", &Range<uchar>::upper)
      .def("size", &Range<uchar>::size)
      .def("swap", &Range<uchar>::swap)
      .def("__repr__", [](const Range<uchar>& r) {
        return "RangeUChar(lower=" + std::to_string(static_cast<int>(r.lower)) +
               ", upper=" + std::to_string(static_cast<int>(r.upper)) + ")";
      });

  // --- Range<short> ---
  py::class_<Range<short>>(m, "RangeS", "Value range with short bounds.")
      .def(py::init<short, short>(), py::arg("lower") = 0, py::arg("upper") = 0)
      .def_readwrite("lower", &Range<short>::lower)
      .def_readwrite("upper", &Range<short>::upper)
      .def("size", &Range<short>::size)
      .def("swap", &Range<short>::swap)
      .def("__repr__", [](const Range<short>& r) {
        return "RangeS(lower=" + std::to_string(r.lower) + ", upper=" + std::to_string(r.upper) + ")";
      });

  // --- FilterData ---
  py::class_<FilterData>(m, "FilterData",
                         "Container for filter output data with associated value range.\n"
                         "Stores a numpy array along with the expected range of values.")
      .def(py::init<>())
      .def(py::init<const cv::Mat&, double, double>(), py::arg("data"), py::arg("lower") = 0.0, py::arg("upper") = 0.0)
      .def_readwrite("data", &FilterData::data, "The filter output as numpy array.")
      .def_readwrite("range", &FilterData::range, "The expected value range for data.")
      .def("__repr__", [](const FilterData& fd) {
        std::string shape =
            fd.data.empty() ? "empty" : std::to_string(fd.data.rows) + "x" + std::to_string(fd.data.cols);
        return "FilterData(shape=" + shape + ", range=[" + std::to_string(fd.range.lower) + ", " +
               std::to_string(fd.range.upper) + "])";
      });

  // --- Value ---
  // Note: bool constructor must be registered before int to avoid Python
  // bool->int implicit conversion ambiguity
  py::class_<Value>(m, "Value", "Type-safe variant supporting float, int, bool, and string.")
      .def(py::init<bool>(), py::arg("val"))
      .def(py::init<int>(), py::arg("val"))
      .def(py::init<double>(), py::arg("val") = 0.0)
      .def(py::init<const std::string&>(), py::arg("val"))
      .def("get_int", &Value::getInt, "Get value as integer.")
      .def("get_float", &Value::getFloat, "Get value as float.")
      .def("get_double", &Value::getDouble, "Get value as double.")
      .def("get_bool", &Value::getBool, "Get value as bool.")
      .def("get_string", &Value::getString, "Get value as string.")
      .def("to_string", &Value::toString, "Convert to string representation.")
      .def("__repr__", [](const Value& v) { return "Value(" + v.toString() + ")"; });

  // --- ValueManager (base, exposed for get/set) ---
  py::class_<ValueManager>(m, "ValueManager",
                           "Base class for runtime-configurable parameters.\n"
                           "Provides get/set access to named algorithm parameters.")
      .def(
          "values",
          [](const ValueManager& vm) {
            auto nvv = vm.values();
            py::dict d;
            for (const auto& nv : nvv) {
              d[py::cast(nv.name)] = py::cast(nv.value);
            }
            return d;
          },
          "Get all parameters as a dict {name: Value}.")
      .def(
          "get_value", [](const ValueManager& vm, const std::string& name) { return vm.value(name); }, py::arg("name"),
          "Get parameter value by name.")
      .def(
          "set_value", [](ValueManager& vm, const std::string& name, const Value& v) { vm.value(name, v); },
          py::arg("name"), py::arg("value"), "Set parameter value by name.")
      .def(
          "set_int", [](ValueManager& vm, const std::string& name, int v) { vm.value(name, Value(v)); },
          py::arg("name"), py::arg("value"), "Set integer parameter by name.")
      .def(
          "set_float", [](ValueManager& vm, const std::string& name, double v) { vm.value(name, Value(v)); },
          py::arg("name"), py::arg("value"), "Set float parameter by name.")
      .def(
          "set_string",
          [](ValueManager& vm, const std::string& name, const std::string& v) { vm.value(name, Value(v)); },
          py::arg("name"), py::arg("value"), "Set string parameter by name.");
}

// ============================================================================
// Filter interface bindings
// ============================================================================

void bind_filter_interface(py::module_& m) {
  py::class_<FilterI<uchar>, ValueManager, PyFilterI>(m, "FilterI",
                                                      "Abstract base interface for image filters (8-bit input).\n\n"
                                                      "Provides a common interface for all image processing filters.\n"
                                                      "Subclass this from Python to implement custom filters, or use\n"
                                                      "concrete implementations like SobelGradient.")
      .def("intensity_range", &FilterI<uchar>::intensityRange, "Get the expected input image intensity range.")
      .def("process", &FilterI<uchar>::process, py::arg("img"),
           "Process the input image through the filter.\n\n"
           "Args:\n"
           "    img: Input image as numpy array (uint8).")
      .def("results", &FilterI<uchar>::results, "Get all filter outputs as a dict of {name: FilterData}.")
      .def("name", &FilterI<uchar>::name, "Get the name identifier of this filter.");
}

void bind_gradient_interface(py::module_& m) {
  using GI = GradientI<uchar, short, float, float>;

  py::class_<GI, FilterI<uchar>, PyGradientI>(m, "GradientI",
                                              "Abstract interface for gradient computation filters.\n\n"
                                              "Provides unified access to magnitude, direction, and directional\n"
                                              "derivatives (gx, gy) computed from an input image.")
      .def("magnitude", &GI::magnitude, "Get the gradient magnitude image as numpy array.")
      .def("magnitude_range", &GI::magnitudeRange, "Get the expected magnitude value range.")
      .def("magnitude_threshold", &GI::magnitudeThreshold, py::arg("val"),
           "Convert normalized threshold [0,1] to magnitude threshold.\n\n"
           "Args:\n"
           "    val: Normalized threshold in [0, 1].")
      .def("direction", &GI::direction, "Get the gradient direction image as numpy array.")
      .def("direction_range", &GI::directionRange, "Get the direction value range.")
      .def("gx", &GI::gx, "Get X-direction gradient as numpy array.")
      .def("gy", &GI::gy, "Get Y-direction gradient as numpy array.")
      .def("gradient_range", &GI::gradientRange, "Get the gradient value range for single direction.");
}

void bind_laplace_interface(py::module_& m) {
  using LI = LaplaceI<uchar, int>;

  py::class_<LI, FilterI<uchar>, PyLaplaceI>(m, "LaplaceI",
                                             "Abstract interface for Laplacian filters.\n\n"
                                             "Provides interface for computing second-order derivatives.")
      .def("laplace", &LI::laplace, "Get the Laplacian response image as numpy array.")
      .def("laplace_range", &LI::laplaceRange, "Get the expected Laplacian value range.")
      .def("laplace_threshold", &LI::laplaceThreshold, py::arg("val"),
           "Convert normalized threshold [0,1] to Laplacian threshold.");
}

// ============================================================================
// Concrete filter bindings
// ============================================================================

void bind_derivative_gradient(py::module_& m) {
  // Common typedef aliases for DerivativeGradient variants
  // Using float for magnitude type (MT) to avoid cv::sqrt SIMD alignment issues with int
  using SobelGrad = DerivativeGradient<uchar, short, float, float, SobelDerivative, Magnitude, Direction>;
  using ScharrGrad = DerivativeGradient<uchar, short, float, float, ScharrDerivative, Magnitude, Direction>;
  using PrewittGrad = DerivativeGradient<uchar, short, float, float, PrewittDerivative, Magnitude, Direction>;

  // Intermediate base: Gradient<uchar, short, float, float>
  using GradBase = Gradient<uchar, short, float, float>;
  py::class_<GradBase, GradientI<uchar, short, float, float>>(
      m, "GradientBase", "Base gradient implementation (not directly constructible).");

  // --- SobelGradient ---
  py::class_<SobelGrad, GradBase>(m, "SobelGradient",
                                  "Gradient computation using Sobel derivative operator.\n\n"
                                  "Computes image gradients using 3x3 or larger Sobel kernels.\n"
                                  "Provides magnitude, direction, and directional derivatives (gx, gy).\n\n"
                                  "Parameters (via set_value/set_int):\n"
                                  "    'grad_kernel_size': Kernel size (1, 3, 5, 7). Default: 3.\n\n"
                                  "Example:\n"
                                  "    grad = le_imgproc.SobelGradient()\n"
                                  "    grad.process(image)\n"
                                  "    mag = grad.magnitude()\n"
                                  "    dir = grad.direction()")
      .def(py::init<uchar, uchar>(), py::arg("int_lower") = 0, py::arg("int_upper") = 255,
           "Construct with intensity range bounds.")
      .def("process", static_cast<void (SobelGrad::*)(const cv::Mat&)>(&SobelGrad::process), py::arg("img"),
           "Process input image to compute Sobel gradients.")
      .def("magnitude", &SobelGrad::magnitude, "Get gradient magnitude as numpy array.")
      .def("direction", &SobelGrad::direction, "Get gradient direction as numpy array.")
      .def("gx", &SobelGrad::gx, "Get X-direction gradient as numpy array.")
      .def("gy", &SobelGrad::gy, "Get Y-direction gradient as numpy array.")
      .def("magnitude_range", &SobelGrad::magnitudeRange)
      .def("direction_range", &SobelGrad::directionRange)
      .def("gradient_range", &SobelGrad::gradientRange)
      .def("name", &SobelGrad::name)
      .def("results", &SobelGrad::results)
      .def("intensity_range", &SobelGrad::intensityRange);

  // --- ScharrGradient ---
  py::class_<ScharrGrad, GradBase>(m, "ScharrGradient",
                                   "Gradient computation using Scharr derivative operator.\n\n"
                                   "More accurate than Sobel for small kernels (3x3 only).\n"
                                   "Provides magnitude, direction, and directional derivatives.")
      .def(py::init<uchar, uchar>(), py::arg("int_lower") = 0, py::arg("int_upper") = 255)
      .def("process", static_cast<void (ScharrGrad::*)(const cv::Mat&)>(&ScharrGrad::process), py::arg("img"))
      .def("magnitude", &ScharrGrad::magnitude)
      .def("direction", &ScharrGrad::direction)
      .def("gx", &ScharrGrad::gx)
      .def("gy", &ScharrGrad::gy)
      .def("magnitude_range", &ScharrGrad::magnitudeRange)
      .def("direction_range", &ScharrGrad::directionRange)
      .def("gradient_range", &ScharrGrad::gradientRange)
      .def("name", &ScharrGrad::name)
      .def("results", &ScharrGrad::results)
      .def("intensity_range", &ScharrGrad::intensityRange);

  // --- PrewittGradient ---
  py::class_<PrewittGrad, GradBase>(m, "PrewittGradient",
                                    "Gradient computation using Prewitt derivative operator.\n\n"
                                    "Simple 3x3 averaging derivative, less noise-sensitive than Roberts.")
      .def(py::init<uchar, uchar>(), py::arg("int_lower") = 0, py::arg("int_upper") = 255)
      .def("process", static_cast<void (PrewittGrad::*)(const cv::Mat&)>(&PrewittGrad::process), py::arg("img"))
      .def("magnitude", &PrewittGrad::magnitude)
      .def("direction", &PrewittGrad::direction)
      .def("gx", &PrewittGrad::gx)
      .def("gy", &PrewittGrad::gy)
      .def("magnitude_range", &PrewittGrad::magnitudeRange)
      .def("direction_range", &PrewittGrad::directionRange)
      .def("gradient_range", &PrewittGrad::gradientRange)
      .def("name", &PrewittGrad::name)
      .def("results", &PrewittGrad::results)
      .def("intensity_range", &PrewittGrad::intensityRange);
}

}  // namespace python
}  // namespace lsfm
