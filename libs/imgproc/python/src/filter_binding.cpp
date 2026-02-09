/// @file filter_binding.cpp
/// @brief Templated pybind11 bindings for the imgproc filter hierarchy.
///
/// Implements the templated binding functions declared in filter_binding.hpp
/// and provides explicit template instantiations for the standard presets:
///
///   | Suffix  | IT     | GT     | MT     | DT     | LT     |
///   |---------|--------|--------|--------|--------|--------|
///   | (none)  | uchar  | short  | float  | float  | int    |
///   | _16u    | ushort | float  | float  | float  | int    |
///   | _f32    | float  | float  | float  | float  | float  |
///   | _f64    | double | double | double | double | double |

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

#include <limits>
#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Templated trampoline classes for abstract interfaces
// ============================================================================

/// @brief Trampoline enabling Python subclassing of FilterI<IT>.
/// @tparam IT Input image pixel type
template <class IT>
class PyFilterI : public FilterI<IT> {
 public:
  using FilterI<IT>::FilterI;

  Range<IT> intensityRange() const override { PYBIND11_OVERRIDE_PURE(Range<IT>, FilterI<IT>, intensityRange); }

  void process(const cv::Mat& img) override { PYBIND11_OVERRIDE_PURE(void, FilterI<IT>, process, img); }

  FilterResults results() const override { PYBIND11_OVERRIDE_PURE(FilterResults, FilterI<IT>, results); }

  std::string name() const override { PYBIND11_OVERRIDE_PURE(std::string, FilterI<IT>, name); }
};

/// @brief Trampoline enabling Python subclassing of GradientI<IT,GT,MT,DT>.
template <class IT, class GT, class MT, class DT>
class PyGradientI : public GradientI<IT, GT, MT, DT> {
  using Base = GradientI<IT, GT, MT, DT>;

 public:
  using Base::Base;

  Range<IT> intensityRange() const override { PYBIND11_OVERRIDE_PURE(Range<IT>, Base, intensityRange); }

  void process(const cv::Mat& img) override { PYBIND11_OVERRIDE_PURE(void, Base, process, img); }

  FilterResults results() const override { PYBIND11_OVERRIDE_PURE(FilterResults, Base, results); }

  std::string name() const override { PYBIND11_OVERRIDE_PURE(std::string, Base, name); }

  cv::Mat magnitude() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, Base, magnitude); }

  Range<MT> magnitudeRange() const override { PYBIND11_OVERRIDE_PURE(Range<MT>, Base, magnitudeRange); }

  cv::Mat direction() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, Base, direction); }

  Range<DT> directionRange() const override { PYBIND11_OVERRIDE_PURE(Range<DT>, Base, directionRange); }
};

/// @brief Trampoline enabling Python subclassing of LaplaceI<IT,LT>.
template <class IT, class LT>
class PyLaplaceI : public LaplaceI<IT, LT> {
  using Base = LaplaceI<IT, LT>;

 public:
  using Base::Base;

  Range<IT> intensityRange() const override { PYBIND11_OVERRIDE_PURE(Range<IT>, Base, intensityRange); }

  void process(const cv::Mat& img) override { PYBIND11_OVERRIDE_PURE(void, Base, process, img); }

  FilterResults results() const override { PYBIND11_OVERRIDE_PURE(FilterResults, Base, results); }

  std::string name() const override { PYBIND11_OVERRIDE_PURE(std::string, Base, name); }

  cv::Mat laplace() const override { PYBIND11_OVERRIDE_PURE(cv::Mat, Base, laplace); }

  Range<LT> laplaceRange() const override { PYBIND11_OVERRIDE_PURE(Range<LT>, Base, laplaceRange); }
};

// ============================================================================
// Core type bindings (non-templated, called once)
// ============================================================================

/// @brief Helper to bind a single Range<T> specialization.
/// @tparam T The scalar type for the range
/// @param m Module
/// @param name Python class name (e.g. "RangeD")
/// @param doc Brief docstring
template <class T>
static void bind_range(py::module_& m, const char* name, const char* doc) {
  py::class_<Range<T>>(m, name, doc)
      .def(py::init<T, T>(), py::arg("lower") = T{0}, py::arg("upper") = T{0})
      .def_readwrite("lower", &Range<T>::lower, "Lower bound.")
      .def_readwrite("upper", &Range<T>::upper, "Upper bound.")
      .def("size", &Range<T>::size, "Absolute size of range.")
      .def("swap", &Range<T>::swap, "Swap lower and upper bounds.")
      .def("__repr__", [name](const Range<T>& r) {
        // Promote small integer types to int to avoid -Wsign-promo with std::to_string.
        if constexpr (sizeof(T) < sizeof(int)) {
          return std::string(name) + "(lower=" + std::to_string(static_cast<int>(r.lower)) +
                 ", upper=" + std::to_string(static_cast<int>(r.upper)) + ")";
        } else {
          return std::string(name) + "(lower=" + std::to_string(r.lower) + ", upper=" + std::to_string(r.upper) + ")";
        }
      });
}

void bind_core_types(py::module_& m) {
  // Range specializations for every type used across all presets.
  bind_range<double>(m, "RangeD", "Value range with double bounds.");
  bind_range<float>(m, "RangeF", "Value range with float bounds.");
  bind_range<int>(m, "RangeI", "Value range with int32 bounds.");
  bind_range<short>(m, "RangeS", "Value range with int16 bounds.");
  bind_range<uchar>(m, "RangeUChar", "Value range with uint8 bounds (0-255).");
  bind_range<ushort>(m, "RangeUShort", "Value range with uint16 bounds (0-65535).");

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
  // Note: bool constructor registered before int to avoid Python bool→int ambiguity.
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

  // --- ValueManager ---
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
          "get_value", [](const ValueManager& vm, const std::string& n) { return vm.value(n); }, py::arg("name"),
          "Get parameter value by name.")
      .def(
          "set_value", [](ValueManager& vm, const std::string& n, const Value& v) { vm.value(n, v); }, py::arg("name"),
          py::arg("value"), "Set parameter value by name.")
      .def(
          "set_int", [](ValueManager& vm, const std::string& n, int v) { vm.value(n, Value(v)); }, py::arg("name"),
          py::arg("value"), "Set integer parameter by name.")
      .def(
          "set_float", [](ValueManager& vm, const std::string& n, double v) { vm.value(n, Value(v)); }, py::arg("name"),
          py::arg("value"), "Set float parameter by name.")
      .def(
          "set_string", [](ValueManager& vm, const std::string& n, const std::string& v) { vm.value(n, Value(v)); },
          py::arg("name"), py::arg("value"), "Set string parameter by name.");
}

// ============================================================================
// Templated interface bindings
// ============================================================================

template <class IT>
void bind_filter_interface(py::module_& m, const std::string& suffix) {
  using FI = FilterI<IT>;
  const std::string cls = "FilterI" + suffix;

  py::class_<FI, ValueManager, PyFilterI<IT>>(m, cls.c_str(),
                                              ("Abstract base interface for image filters" + suffix +
                                               ".\n\n"
                                               "Subclass from Python or use concrete implementations.")
                                                  .c_str())
      .def("intensity_range", &FI::intensityRange, "Get the expected input image intensity range.")
      .def("process", &FI::process, py::arg("img"), "Process the input image through the filter.")
      .def("results", &FI::results, "Get all filter outputs as a dict of {name: FilterData}.")
      .def("name", &FI::name, "Get the name identifier of this filter.");
}

template <class IT, class GT, class MT, class DT>
void bind_gradient_interface(py::module_& m, const std::string& suffix) {
  using GI = GradientI<IT, GT, MT, DT>;
  using FI = FilterI<IT>;
  const std::string cls = "GradientI" + suffix;

  py::class_<GI, FI, PyGradientI<IT, GT, MT, DT>>(m, cls.c_str(),
                                                  ("Abstract gradient computation interface" + suffix +
                                                   ".\n\n"
                                                   "Provides magnitude, direction, and derivative access.")
                                                      .c_str())
      .def("magnitude", &GI::magnitude, "Get the gradient magnitude image as numpy array.")
      .def("magnitude_range", &GI::magnitudeRange, "Get the expected magnitude value range.")
      .def("magnitude_threshold", &GI::magnitudeThreshold, py::arg("val"),
           "Convert normalized threshold [0,1] to magnitude threshold.")
      .def("direction", &GI::direction, "Get the gradient direction image as numpy array.")
      .def("direction_range", &GI::directionRange, "Get the direction value range.")
      .def("gx", &GI::gx, "Get X-direction gradient as numpy array.")
      .def("gy", &GI::gy, "Get Y-direction gradient as numpy array.")
      .def("gradient_range", &GI::gradientRange, "Get the gradient value range for single direction.");
}

template <class IT, class LT>
void bind_laplace_interface(py::module_& m, const std::string& suffix) {
  using LI = LaplaceI<IT, LT>;
  using FI = FilterI<IT>;
  const std::string cls = "LaplaceI" + suffix;

  py::class_<LI, FI, PyLaplaceI<IT, LT>>(m, cls.c_str(), ("Abstract Laplacian filter interface" + suffix + ".").c_str())
      .def("laplace", &LI::laplace, "Get the Laplacian response image as numpy array.")
      .def("laplace_range", &LI::laplaceRange, "Get the expected Laplacian value range.")
      .def("laplace_threshold", &LI::laplaceThreshold, py::arg("val"),
           "Convert normalized threshold [0,1] to Laplacian threshold.");
}

// ============================================================================
// Templated concrete filter bindings
// ============================================================================

template <class IT, class GT, class MT, class DT>
void bind_derivative_gradient(py::module_& m, const std::string& suffix) {
  using GI = GradientI<IT, GT, MT, DT>;
  using GradBase = Gradient<IT, GT, MT, DT>;
  using SobelGrad = DerivativeGradient<IT, GT, MT, DT, SobelDerivative, Magnitude, Direction>;
  using ScharrGrad = DerivativeGradient<IT, GT, MT, DT, ScharrDerivative, Magnitude, Direction>;
  using PrewittGrad = DerivativeGradient<IT, GT, MT, DT, PrewittDerivative, Magnitude, Direction>;

  const IT lo = std::numeric_limits<IT>::lowest();
  const IT hi = std::numeric_limits<IT>::max();

  // Intermediate base: Gradient<IT, GT, MT, DT>
  py::class_<GradBase, GI>(m, ("GradientBase" + suffix).c_str(),
                           "Base gradient implementation (not directly constructible).");

  // --- SobelGradient ---
  py::class_<SobelGrad, GradBase>(m, ("SobelGradient" + suffix).c_str(),
                                  ("Gradient computation using Sobel derivative operator" + suffix +
                                   ".\n\n"
                                   "Computes image gradients using 3x3 or larger Sobel kernels.\n"
                                   "Parameters (via set_value/set_int):\n"
                                   "    'grad_kernel_size': Kernel size (1, 3, 5, 7). Default: 3.")
                                      .c_str())
      .def(py::init<IT, IT>(), py::arg("int_lower") = lo, py::arg("int_upper") = hi,
           "Construct with intensity range bounds.")
      .def("process", static_cast<void (SobelGrad::*)(const cv::Mat&)>(&SobelGrad::process), py::arg("img"),
           "Process input image to compute Sobel gradients.")
      .def("magnitude", &SobelGrad::magnitude)
      .def("direction", &SobelGrad::direction)
      .def("gx", &SobelGrad::gx)
      .def("gy", &SobelGrad::gy)
      .def("magnitude_range", &SobelGrad::magnitudeRange)
      .def("direction_range", &SobelGrad::directionRange)
      .def("gradient_range", &SobelGrad::gradientRange)
      .def("name", &SobelGrad::name)
      .def("results", &SobelGrad::results)
      .def("intensity_range", &SobelGrad::intensityRange);

  // --- ScharrGradient ---
  py::class_<ScharrGrad, GradBase>(m, ("ScharrGradient" + suffix).c_str(),
                                   ("Gradient computation using Scharr derivative operator" + suffix +
                                    ".\n\n"
                                    "More accurate than Sobel for small kernels (3x3 only).")
                                       .c_str())
      .def(py::init<IT, IT>(), py::arg("int_lower") = lo, py::arg("int_upper") = hi)
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
  py::class_<PrewittGrad, GradBase>(m, ("PrewittGradient" + suffix).c_str(),
                                    ("Gradient computation using Prewitt derivative operator" + suffix +
                                     ".\n\n"
                                     "Simple 3x3 averaging derivative.")
                                        .c_str())
      .def(py::init<IT, IT>(), py::arg("int_lower") = lo, py::arg("int_upper") = hi)
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

// ============================================================================
// Convenience preset binder
// ============================================================================

template <class IT, class GT, class MT, class DT, class LT>
void bind_filter_preset(py::module_& m, const std::string& suffix) {
  bind_filter_interface<IT>(m, suffix);
  bind_gradient_interface<IT, GT, MT, DT>(m, suffix);
  bind_laplace_interface<IT, LT>(m, suffix);
  bind_derivative_gradient<IT, GT, MT, DT>(m, suffix);
}

// ============================================================================
// Explicit template instantiations for supported presets
// ============================================================================

// Default: 8-bit unsigned (uchar) — suffix ""
template void bind_filter_interface<uchar>(py::module_&, const std::string&);
template void bind_gradient_interface<uchar, short, float, float>(py::module_&, const std::string&);
template void bind_laplace_interface<uchar, int>(py::module_&, const std::string&);
template void bind_derivative_gradient<uchar, short, float, float>(py::module_&, const std::string&);
template void bind_filter_preset<uchar, short, float, float, int>(py::module_&, const std::string&);

// 16-bit unsigned (ushort) — suffix "_16u"
template void bind_filter_interface<ushort>(py::module_&, const std::string&);
template void bind_gradient_interface<ushort, float, float, float>(py::module_&, const std::string&);
template void bind_laplace_interface<ushort, int>(py::module_&, const std::string&);
template void bind_derivative_gradient<ushort, float, float, float>(py::module_&, const std::string&);
template void bind_filter_preset<ushort, float, float, float, int>(py::module_&, const std::string&);

// Float (32-bit) — suffix "_f32"
template void bind_filter_interface<float>(py::module_&, const std::string&);
template void bind_gradient_interface<float, float, float, float>(py::module_&, const std::string&);
template void bind_laplace_interface<float, float>(py::module_&, const std::string&);
template void bind_derivative_gradient<float, float, float, float>(py::module_&, const std::string&);
template void bind_filter_preset<float, float, float, float, float>(py::module_&, const std::string&);

// Double (64-bit) — suffix "_f64"
template void bind_filter_interface<double>(py::module_&, const std::string&);
template void bind_gradient_interface<double, double, double, double>(py::module_&, const std::string&);
template void bind_laplace_interface<double, double>(py::module_&, const std::string&);
template void bind_derivative_gradient<double, double, double, double>(py::module_&, const std::string&);
template void bind_filter_preset<double, double, double, double, double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
