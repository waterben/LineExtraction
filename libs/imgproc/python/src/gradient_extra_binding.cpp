/// @file gradient_extra_binding.cpp
/// @brief pybind11 bindings for SusanGradient and RCMGradient.
///
/// Binds the two non-derivative gradient operators. These types have
/// magnitude types (MT) that differ from the standard filter presets, so
/// they are bound with ValueManager as the pybind11 base rather than
/// reusing the preset GradientI registration.
///
///   | Class                   | IT     | GT    | MT    | DT    | Suffix |
///   |-------------------------|--------|-------|-------|-------|--------|
///   | SusanGradient           | uchar  | short | short | float | (none) |
///   | RCMGradientColor        | uchar  | short | int   | float | (none) |
///   | RCMGradient             | uchar  | short | int   | float | (none) |
///   | RCMGradient_f32         | float  | float | float | float | _f32   |
///   | RCMGradient_f64         | double | double| double| double| _f64   |

#include "filter_binding.hpp"
#include <cvnp/cvnp.h>
#include <imgproc/rcmg.hpp>
#include <imgproc/susan.hpp>
#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// SusanGradient  (uchar input only, MT=short)
// ============================================================================

void bind_susan_gradient(py::module_& m) {
  using Susan = SusanGradient<short, short, float>;

  py::class_<Susan, ValueManager>(m, "SusanGradient",
                                  "SUSAN edge detector and gradient operator.\n\n"
                                  "Detects edges by examining local brightness consistency\n"
                                  "within a circular mask, producing robust edge responses.\n"
                                  "Input: CV_8UC1 (uint8 grayscale only).\n\n"
                                  "Parameters (via set_value):\n"
                                  "    'grad_brightness_th': Brightness threshold [0-256] (default: 20)\n"
                                  "    'grad_small_kernel': Use 3x3 kernel (default: false)\n"
                                  "    'grad_max_no': Max response normalization (default: 2650)")
      .def(py::init<short, bool, short, uchar, uchar>(), py::arg("brightness_th") = static_cast<short>(20),
           py::arg("small_kernel") = false, py::arg("max_no") = static_cast<short>(2650),
           py::arg("int_lower") = std::numeric_limits<uchar>::lowest(),
           py::arg("int_upper") = std::numeric_limits<uchar>::max(), "Construct SUSAN gradient operator.")
      .def("process", static_cast<void (Susan::*)(const cv::Mat&)>(&Susan::process), py::arg("img"),
           "Process grayscale image (CV_8UC1).")
      .def("magnitude", &Susan::magnitude, "Get magnitude image.")
      .def("direction", &Susan::direction, "Get direction image.")
      .def("gx", &Susan::gx, "Get X-gradient image.")
      .def("gy", &Susan::gy, "Get Y-gradient image.")
      .def("magnitude_range", &Susan::magnitudeRange, "Get magnitude value range.")
      .def("direction_range", &Susan::directionRange, "Get direction value range.")
      .def("gradient_range", &Susan::gradientRange, "Get gradient component range.")
      .def("name", &Susan::name, "Get filter name.")
      .def("results", &Susan::results, "Get all filter outputs as dict.")
      .def("intensity_range", &Susan::intensityRange, "Get input intensity range.");
}

// ============================================================================
// RCMGradient  — multi-channel (3-ch, uchar, MT=int)
// ============================================================================

void bind_rcmg_gradient_color(py::module_& m) {
  using RCMG3 = RCMGradient<uchar, 3, short, int, float>;

  py::class_<RCMG3, ValueManager>(m, "RCMGradientColor",
                                  "Robust Colour Morphological Gradient for 3-channel images.\n\n"
                                  "Computes edges using morphological gradient in color space.\n"
                                  "Input: 3-channel uint8 image (e.g. BGR).\n\n"
                                  "Parameters (via set_value):\n"
                                  "    'grad_mask_size': Window size (default: 3)\n"
                                  "    'grad_rejection': Pairs to reject (default: 0)\n"
                                  "    'grad_color_norm': Norm type (default: L2SQR)")
      .def(py::init<int, int, int, uchar, uchar>(), py::arg("mask") = 3, py::arg("s") = 0,
           py::arg("norm") = static_cast<int>(cv::NORM_L2SQR),
           py::arg("int_lower") = std::numeric_limits<uchar>::lowest(),
           py::arg("int_upper") = std::numeric_limits<uchar>::max(), "Construct RCMG gradient for 3-channel images.")
      .def("process", static_cast<void (RCMG3::*)(const cv::Mat&)>(&RCMG3::process), py::arg("img"),
           "Process 3-channel image.")
      .def("magnitude", &RCMG3::magnitude, "Get magnitude image.")
      .def("direction", &RCMG3::direction, "Get direction image.")
      .def("gx", &RCMG3::gx, "Get X-gradient image.")
      .def("gy", &RCMG3::gy, "Get Y-gradient image.")
      .def("magnitude_range", &RCMG3::magnitudeRange, "Get magnitude value range.")
      .def("direction_range", &RCMG3::directionRange, "Get direction value range.")
      .def("gradient_range", &RCMG3::gradientRange, "Get gradient component range.")
      .def("name", &RCMG3::name, "Get filter name.")
      .def("results", &RCMG3::results, "Get all filter outputs as dict.")
      .def("intensity_range", &RCMG3::intensityRange, "Get input intensity range.");
}

// ============================================================================
// RCMGradient  — single-channel (templated on IT)
// ============================================================================

template <class IT, class GT, class MT, class DT>
void bind_rcmg_gradient_gray(py::module_& m, const std::string& suffix) {
  using RCMG1 = RCMGradient<IT, 1, GT, MT, DT>;
  using GradBase = Gradient<IT, GT, MT, DT>;
  const std::string cls = "RCMGradient" + suffix;

  // For presets where GradientI<IT,GT,MT,DT> is already registered
  // (f32, f64), we can use the full hierarchy.  For the uchar preset
  // (MT=int, which differs from the default MT=float), fall back to
  // ValueManager as the pybind11 base.
  if (py::detail::get_type_info(typeid(GradBase))) {
    // GradBase already registered — just add concrete class
    py::class_<RCMG1, GradBase>(m, cls.c_str(), ("Single-channel RCMG gradient" + suffix + ".").c_str())
        .def(py::init<int, int, int, IT, IT>(), py::arg("mask") = 3, py::arg("s") = 0,
             py::arg("norm") = static_cast<int>(cv::NORM_L2SQR),
             py::arg("int_lower") = std::numeric_limits<IT>::lowest(),
             py::arg("int_upper") = std::numeric_limits<IT>::max())
        .def("process", static_cast<void (RCMG1::*)(const cv::Mat&)>(&RCMG1::process), py::arg("img"))
        .def("magnitude", &RCMG1::magnitude)
        .def("direction", &RCMG1::direction)
        .def("gx", &RCMG1::gx)
        .def("gy", &RCMG1::gy)
        .def("magnitude_range", &RCMG1::magnitudeRange)
        .def("direction_range", &RCMG1::directionRange)
        .def("gradient_range", &RCMG1::gradientRange)
        .def("name", &RCMG1::name)
        .def("results", &RCMG1::results)
        .def("intensity_range", &RCMG1::intensityRange);
  } else {
    // Fall back: ValueManager as base (uchar preset where MT=int)
    py::class_<RCMG1, ValueManager>(m, cls.c_str(),
                                    ("Single-channel RCMG gradient" + suffix +
                                     ".\n\n"
                                     "Morphological gradient for grayscale images.\n\n"
                                     "Parameters (via set_value):\n"
                                     "    'grad_mask_size': Window size (default: 3)\n"
                                     "    'grad_rejection': Pairs to reject (default: 0)\n"
                                     "    'grad_norm_type': Norm type (default: L2SQR)")
                                        .c_str())
        .def(py::init<int, int, int, IT, IT>(), py::arg("mask") = 3, py::arg("s") = 0,
             py::arg("norm") = static_cast<int>(cv::NORM_L2SQR),
             py::arg("int_lower") = std::numeric_limits<IT>::lowest(),
             py::arg("int_upper") = std::numeric_limits<IT>::max(), "Construct single-channel RCMG gradient.")
        .def("process", static_cast<void (RCMG1::*)(const cv::Mat&)>(&RCMG1::process), py::arg("img"))
        .def("magnitude", &RCMG1::magnitude)
        .def("direction", &RCMG1::direction)
        .def("gx", &RCMG1::gx)
        .def("gy", &RCMG1::gy)
        .def("magnitude_range", &RCMG1::magnitudeRange)
        .def("direction_range", &RCMG1::directionRange)
        .def("gradient_range", &RCMG1::gradientRange)
        .def("name", &RCMG1::name)
        .def("results", &RCMG1::results)
        .def("intensity_range", &RCMG1::intensityRange);
  }
}

// --- Explicit template instantiations ---
template void bind_rcmg_gradient_gray<uchar, short, int, float>(py::module_&, const std::string&);
template void bind_rcmg_gradient_gray<float, float, float, float>(py::module_&, const std::string&);
template void bind_rcmg_gradient_gray<double, double, double, double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
