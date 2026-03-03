/// @file laplace_binding.cpp
/// @brief Templated pybind11 bindings for concrete Laplacian filters.
///
/// Binds LaplaceSimple, LoG, and LaplaceCV — all implement the LaplaceI
/// interface (already bound in filter_binding.cpp).
///
///   | Suffix  | IT     | LT     |
///   |---------|--------|--------|
///   | (none)  | uchar  | int    |
///   | _16u    | ushort | int    |
///   | _f32    | float  | float  |
///   | _f64    | double | double |

#include "filter_binding.hpp"
#include <cvnp/cvnp.h>
#include <imgproc/laplace.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>
#include <string>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// LaplaceSimple<IT,LT>
// ============================================================================

template <class IT, class LT>
void bind_laplace_simple(py::module_& m, const std::string& suffix) {
  using LS = LaplaceSimple<IT, LT>;
  using LI = LaplaceI<IT, LT>;
  const IT lo = std::numeric_limits<IT>::lowest();
  const IT hi = std::numeric_limits<IT>::max();
  const std::string cls = "LaplaceSimple" + suffix;

  py::class_<LS, LI>(m, cls.c_str(),
                     ("Simple 3x3 Laplacian filter" + suffix +
                      ".\n\n"
                      "Uses a 3x3 kernel with all neighbors weighted 1\n"
                      "and center -8. Fast but sensitive to noise.")
                         .c_str())
      .def(py::init<IT, IT>(), py::arg("int_lower") = lo, py::arg("int_upper") = hi,
           "Construct with intensity range bounds.")
      .def("process", &LS::process, py::arg("img"), "Apply 3x3 Laplacian to image.")
      .def("laplace", &LS::laplace, "Get Laplacian response image.")
      .def("laplace_range", &LS::laplaceRange, "Get Laplacian value range.")
      .def("name", &LS::name)
      .def("results", &LS::results)
      .def("intensity_range", &LS::intensityRange);
}

// ============================================================================
// LoG<IT,LT>
// ============================================================================

template <class IT, class LT>
void bind_log(py::module_& m, const std::string& suffix) {
  using LogT = LoG<IT, LT>;
  using LS = LaplaceSimple<IT, LT>;
  const IT lo = std::numeric_limits<IT>::lowest();
  const IT hi = std::numeric_limits<IT>::max();
  const std::string cls = "LoG" + suffix;

  py::class_<LogT, LS>(m, cls.c_str(),
                       ("Laplacian of Gaussian filter" + suffix +
                        ".\n\n"
                        "Combines Gaussian smoothing with Laplacian for\n"
                        "noise-robust blob detection (Mexican hat filter).\n\n"
                        "Parameters (via set_value):\n"
                        "    'grad_kernel_size': Kernel size (default: 5)\n"
                        "    'grad_kernel_spacing': Sample spacing (default: 1.008)\n"
                        "    'grad_kernel_scale': Kernel scale factor (default: 1)")
                           .c_str())
      .def(py::init<int, double, double, IT, IT>(), py::arg("kernel_size") = 5, py::arg("kernel_spacing") = 1.008,
           py::arg("kernel_scale") = 1.0, py::arg("int_lower") = lo, py::arg("int_upper") = hi, "Construct LoG filter.")
      .def("process", &LogT::process, py::arg("img"), "Apply LoG filter to image.")
      .def("laplace", &LogT::laplace, "Get LoG response image.")
      .def("laplace_range", &LogT::laplaceRange, "Get LoG value range.")
      .def("kernel", &LogT::kernel, "Get the LoG convolution kernel.")
      .def("even", &LogT::even, "Get even filter response (alias for laplace).")
      .def_property("kernel_size", static_cast<int (LogT::*)() const>(&LogT::kernelSize),
                    static_cast<void (LogT::*)(int)>(&LogT::kernelSize), "Kernel size (3-99, odd).")
      .def_property("kernel_spacing", static_cast<double (LogT::*)() const>(&LogT::kernelSpacing),
                    static_cast<void (LogT::*)(double)>(&LogT::kernelSpacing), "Kernel spacing between samples (> 0).")
      .def_property("kernel_scale", static_cast<double (LogT::*)() const>(&LogT::kernelScale),
                    static_cast<void (LogT::*)(double)>(&LogT::kernelScale), "Kernel scale factor (> 0).")
      .def("name", &LogT::name)
      .def("results", &LogT::results)
      .def("intensity_range", &LogT::intensityRange);
}

// ============================================================================
// LaplaceCV<IT,LT>
// ============================================================================

template <class IT, class LT>
void bind_laplace_cv(py::module_& m, const std::string& suffix) {
  using LC = LaplaceCV<IT, LT>;
  using LI = LaplaceI<IT, LT>;
  const IT lo = std::numeric_limits<IT>::lowest();
  const IT hi = std::numeric_limits<IT>::max();
  const std::string cls = "LaplaceCV" + suffix;

  py::class_<LC, LI>(m, cls.c_str(),
                     ("OpenCV Laplacian filter wrapper" + suffix +
                      ".\n\n"
                      "Uses cv::Laplacian internally with configurable kernel size.\n\n"
                      "Parameters (via set_value):\n"
                      "    'grad_kernel_size': Kernel size (1-31, odd)")
                         .c_str())
      .def(py::init<int, IT, IT>(), py::arg("ksize") = 5, py::arg("int_lower") = lo, py::arg("int_upper") = hi,
           "Construct OpenCV Laplacian filter.")
      .def("process", &LC::process, py::arg("img"), "Apply Laplacian to image.")
      .def("laplace", &LC::laplace, "Get Laplacian response image.")
      .def("laplace_range", &LC::laplaceRange, "Get Laplacian value range.")
      .def_property("kernel_size", static_cast<int (LC::*)() const>(&LC::kernelSize),
                    static_cast<void (LC::*)(int)>(&LC::kernelSize), "Kernel size (1-31, odd).")
      .def("name", &LC::name)
      .def("results", &LC::results)
      .def("intensity_range", &LC::intensityRange);
}

// ============================================================================
// Preset convenience
// ============================================================================

template <class IT, class LT>
void bind_laplace_preset(py::module_& m, const std::string& suffix) {
  bind_laplace_simple<IT, LT>(m, suffix);
  bind_log<IT, LT>(m, suffix);
  bind_laplace_cv<IT, LT>(m, suffix);
}

// --- Explicit template instantiations ---
template void bind_laplace_simple<uchar, int>(py::module_&, const std::string&);
template void bind_laplace_simple<ushort, int>(py::module_&, const std::string&);
template void bind_laplace_simple<float, float>(py::module_&, const std::string&);
template void bind_laplace_simple<double, double>(py::module_&, const std::string&);

template void bind_log<uchar, int>(py::module_&, const std::string&);
template void bind_log<ushort, int>(py::module_&, const std::string&);
template void bind_log<float, float>(py::module_&, const std::string&);
template void bind_log<double, double>(py::module_&, const std::string&);

template void bind_laplace_cv<uchar, int>(py::module_&, const std::string&);
template void bind_laplace_cv<ushort, int>(py::module_&, const std::string&);
template void bind_laplace_cv<float, float>(py::module_&, const std::string&);
template void bind_laplace_cv<double, double>(py::module_&, const std::string&);

template void bind_laplace_preset<uchar, int>(py::module_&, const std::string&);
template void bind_laplace_preset<ushort, int>(py::module_&, const std::string&);
template void bind_laplace_preset<float, float>(py::module_&, const std::string&);
template void bind_laplace_preset<double, double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
