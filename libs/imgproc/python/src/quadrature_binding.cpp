/// @file quadrature_binding.cpp
/// @brief pybind11 bindings for quadrature filter classes.
///
/// Binds the four main quadrature filter implementations:
///   - QuadratureG2  — Steerable G2/H2 separable filter pair
///   - QuadratureLGF — Log-Gabor filter (frequency domain / FFT)
///   - QuadratureS   — Difference of Poisson (spatial domain)
///   - QuadratureSF  — Difference of Poisson (frequency domain / FFT)
///
/// Each class is bound for three presets (uchar/float, float/float,
/// double/double) with ValueManager as the pybind11 base.

#include "filter_binding.hpp"
#include <cvnp/cvnp.h>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>
#include <string>
#include <utility>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Helper: bind methods common to all quadrature filter classes
// ============================================================================

/// @brief Bind methods shared across all quadrature filter types.
///
/// Attaches process, even/odd/energy/phase/direction accessors, results,
/// intensity_range, name, and Laplacian aliases to the given pybind11
/// class object.
///
/// @tparam QClass The concrete quadrature C++ class
/// @tparam PyCls  The pybind11 class_ type
/// @param cls     The pybind11 class object to add defs to
template <class QClass, class PyCls>
static void bind_quadrature_common(PyCls& cls) {
  // --- Core filter interface ---
  cls.def("process", &QClass::process, py::arg("img"),
          "Process an image through the quadrature filter.\n\n"
          "Args:\n"
          "    img: Input grayscale image as numpy array.");

  cls.def("intensity_range", &QClass::intensityRange, "Get the expected input image intensity range.");

  cls.def("results", &QClass::results,
          "Get all filter outputs as a dict of {name: FilterData}.\n\n"
          "Keys: 'even', 'oddx', 'oddy', 'odd', 'dir', 'energy', 'phase'.");

  cls.def("name", &QClass::name, "Get the filter name identifier.");

  // --- Even (symmetric / line-detecting) ---
  cls.def("even", &QClass::even, "Get even (symmetric) filter response as numpy array.");

  cls.def("even_range", &QClass::evenRange, "Get the even response value range.");

  cls.def("even_threshold", &QClass::evenThreshold, py::arg("val"),
          "Convert normalized threshold [0,1] to even threshold value.");

  // --- Odd (antisymmetric / edge-detecting) ---
  // Disambiguate the two odd() overloads: no-arg returns magnitude Mat.
  cls.def("odd", static_cast<cv::Mat (QClass::*)() const>(&QClass::odd),
          "Get odd filter response magnitude as numpy array.");

  // Two-arg odd(ox, oy) → return as Python tuple.
  cls.def(
      "odd_xy",
      [](const QClass& q) {
        cv::Mat ox, oy;
        q.odd(ox, oy);
        return py::make_tuple(std::move(ox), std::move(oy));
      },
      "Get odd X and Y components as (ox, oy) tuple of numpy arrays.");

  cls.def("oddx", &QClass::oddx, "Get X-direction odd filter response.");

  cls.def("oddy", &QClass::oddy, "Get Y-direction odd filter response.");

  cls.def("odd_range", &QClass::oddRange, "Get odd response magnitude range.");

  cls.def("odd_grad_range", &QClass::oddGradRange, "Get odd gradient range for single direction.");

  cls.def("odd_threshold", &QClass::oddThreshold, py::arg("val"),
          "Convert normalized threshold [0,1] to odd magnitude threshold.");

  // --- Direction ---
  cls.def("direction", &QClass::direction, "Get direction image as numpy array.");

  cls.def("direction_range", &QClass::directionRange, "Get direction value range.");

  // --- Energy ---
  cls.def("energy", &QClass::energy, "Get local energy image (sqrt(even^2 + odd^2)).");

  cls.def("energy_range", &QClass::energyRange, "Get energy value range.");

  cls.def("energy_threshold", &QClass::energyThreshold, py::arg("val"),
          "Convert normalized threshold [0,1] to energy threshold.");

  // --- Phase ---
  cls.def("phase", &QClass::phase, "Get local phase image (atan2(odd, even)).");

  cls.def("phase_range", &QClass::phaseRange, "Get phase value range.");

  // --- Laplacian aliases (from LaplaceI base) ---
  cls.def("laplace", &QClass::laplace, "Get Laplacian response (alias for even()).");

  cls.def("laplace_range", &QClass::laplaceRange, "Get Laplacian response range (alias for even_range()).");

  cls.def("laplace_threshold", &QClass::laplaceThreshold, py::arg("val"),
          "Convert normalized threshold to Laplacian threshold.");

  // --- Norm type (returned as int: 0=NONE, 1=L1, 2=L2, 3=L2SQR) ---
  cls.def(
      "norm_type", [](const QClass& q) { return static_cast<int>(q.normType()); },
      "Get norm type used for odd magnitude (0=NONE, 1=L1, 2=L2, 3=L2SQR).");
}


// ============================================================================
// QuadratureG2 binding
// ============================================================================

template <class IT, class FT>
void bind_quadrature_g2(py::module_& m, const std::string& suffix) {
  using QG2 = QuadratureG2<IT, FT>;
  const std::string cls_name = "QuadratureG2" + suffix;

  auto cls =
      py::class_<QG2, ValueManager>(m, cls_name.c_str(),
                                    ("Steerable G2/H2 quadrature filter" + suffix +
                                     ".\n\n"
                                     "Uses separable second-order Gaussian (G2) and Hilbert-transform\n"
                                     "(H2) basis filters. Computes dominant orientation via steering,\n"
                                     "then derives local energy and phase from the steered responses.\n\n"
                                     "Constructor args:\n"
                                     "    kernel_size: Kernel size (odd, 3-99). Default: 9.\n"
                                     "    kernel_spacing: Sample spacing for kernel. Default: 0.782.\n"
                                     "    int_lower: Lower input intensity bound.\n"
                                     "    int_upper: Upper input intensity bound.\n\n"
                                     "ValueManager keys:\n"
                                     "    'grad_kernel_size', 'grad_kernel_spacing'")
                                        .c_str())

          .def(py::init<int, FT, IT, IT>(), py::arg("kernel_size") = 9,
               py::arg("kernel_spacing") = static_cast<FT>(0.782),
               py::arg("int_lower") = std::numeric_limits<IT>::lowest(),
               py::arg("int_upper") = std::numeric_limits<IT>::max(), "Construct a G2 quadrature filter.")

          // --- Kernel size (getter/setter disambiguated) ---
          .def("kernel_size", static_cast<int (QG2::*)() const>(&QG2::kernelSize), "Get the kernel size.")
          .def("set_kernel_size", static_cast<void (QG2::*)(int)>(&QG2::kernelSize), py::arg("ks"),
               "Set the kernel size (odd, 3-99).")

          // --- Kernel spacing (getter/setter disambiguated) ---
          .def("kernel_spacing", static_cast<FT (QG2::*)() const>(&QG2::kernelSpacing), "Get the kernel spacing.")
          .def("set_kernel_spacing", static_cast<void (QG2::*)(FT)>(&QG2::kernelSpacing), py::arg("ks"),
               "Set the kernel spacing (must be positive).")

          // --- Kernel access ---
          .def("kernel", &QG2::kernel, "Get the 2D filter kernel as numpy array.")

          // --- Steer to orientation image ---
          .def(
              "steer",
              [](const QG2& q, const cv::Mat& theta) {
                cv::Mat g2, h2;
                q.steer(theta, g2, h2);
                return py::make_tuple(std::move(g2), std::move(h2));
              },
              py::arg("theta"),
              "Steer filters to orientation image.\n\n"
              "Args:\n"
              "    theta: Orientation image (angles at each pixel).\n\n"
              "Returns:\n"
              "    Tuple (g2, h2) of even and odd responses at given orientations.")

          // --- Steer using gradient components ---
          .def(
              "steer_xy",
              [](const QG2& q, const cv::Mat& gx, const cv::Mat& gy) {
                cv::Mat g2, h2;
                q.steer(gx, gy, g2, h2);
                return py::make_tuple(std::move(g2), std::move(h2));
              },
              py::arg("gx"), py::arg("gy"),
              "Steer filters using gradient X/Y components.\n\n"
              "Args:\n"
              "    gx: X-gradient component (cos theta).\n"
              "    gy: Y-gradient component (sin theta).\n\n"
              "Returns:\n"
              "    Tuple (g2, h2) of steered even and odd responses.")

          // --- Energy and phase together ---
          .def(
              "energy_phase",
              [](const QG2& q) {
                cv::Mat en, ph;
                q.energyPhase(en, ph);
                return py::make_tuple(std::move(en), std::move(ph));
              },
              "Get energy and phase together (more efficient than calling both).\n\n"
              "Returns:\n"
              "    Tuple (energy, phase) of numpy arrays.");

  bind_quadrature_common<QG2>(cls);
}


// ============================================================================
// QuadratureLGF binding
// ============================================================================

template <class IT, class FT>
void bind_quadrature_lgf(py::module_& m, const std::string& suffix) {
  using QLGF = QuadratureLGF<IT, FT>;
  const std::string cls_name = "QuadratureLGF" + suffix;

  auto cls =
      py::class_<QLGF, ValueManager>(m, cls_name.c_str(),
                                     ("Log-Gabor quadrature filter (frequency domain)" + suffix +
                                      ".\n\n"
                                      "Applies a Log-Gabor filter in the frequency domain using FFT.\n"
                                      "The filter is characterized by its wavelength and bandwidth.\n\n"
                                      "Constructor args:\n"
                                      "    wave_length: Filter wavelength (scale). Default: 5.\n"
                                      "    sigma_onf: Bandwidth parameter (std/center freq ratio). Default: 0.55.\n"
                                      "    int_lower: Lower input intensity bound.\n"
                                      "    int_upper: Upper input intensity bound.\n\n"
                                      "ValueManager keys:\n"
                                      "    'grad_waveLength', 'grad_sigmaOnf'")
                                         .c_str())

          .def(py::init<FT, FT, IT, IT>(), py::arg("wave_length") = static_cast<FT>(5),
               py::arg("sigma_onf") = static_cast<FT>(0.55), py::arg("int_lower") = std::numeric_limits<IT>::lowest(),
               py::arg("int_upper") = std::numeric_limits<IT>::max(), "Construct a Log-Gabor quadrature filter.")

          // --- Wavelength (getter/setter) ---
          .def("wave_length", static_cast<FT (QLGF::*)() const>(&QLGF::waveLength), "Get the filter wavelength.")
          .def("set_wave_length", static_cast<void (QLGF::*)(FT)>(&QLGF::waveLength), py::arg("w"),
               "Set the filter wavelength (must be positive).")

          // --- Sigma on f (getter/setter) ---
          .def("sigma_onf", static_cast<FT (QLGF::*)() const>(&QLGF::sigmaOnf),
               "Get the bandwidth parameter (sigma / center frequency).")
          .def("set_sigma_onf", static_cast<void (QLGF::*)(FT)>(&QLGF::sigmaOnf), py::arg("s"),
               "Set the bandwidth parameter (must be positive).");

  bind_quadrature_common<QLGF>(cls);
}


// ============================================================================
// QuadratureS binding
// ============================================================================

template <class IT, class GT, class FT>
void bind_quadrature_s(py::module_& m, const std::string& suffix) {
  using QS = QuadratureS<IT, GT, FT>;
  const std::string cls_name = "QuadratureS" + suffix;

  auto cls =
      py::class_<QS, ValueManager>(m, cls_name.c_str(),
                                   ("Spatial-domain quadrature filter (Difference of Poisson)" + suffix +
                                    ".\n\n"
                                    "Uses 2D non-separable Difference of Poisson (DoP) kernels.\n"
                                    "Odd filters detect steps, even filter detects ridges/valleys.\n\n"
                                    "Constructor args:\n"
                                    "    scale: Filter scale. Default: 1.\n"
                                    "    muls: Scale multiplier / bandwidth. Default: 3.\n"
                                    "    kernel_size: Kernel size (odd, 3-99). Default: 5.\n"
                                    "    kernel_spacing: Sample spacing. Default: 1.\n"
                                    "    kernel_scale: Overall kernel scale factor. Default: 1.\n"
                                    "    int_lower: Lower input intensity bound.\n"
                                    "    int_upper: Upper input intensity bound.\n\n"
                                    "ValueManager keys:\n"
                                    "    'grad_kernel_size', 'grad_kernel_spacing', 'grad_kernel_scale',\n"
                                    "    'grad_scale', 'grad_muls'")
                                       .c_str())

          .def(py::init<FT, FT, int, FT, FT, IT, IT>(), py::arg("scale") = static_cast<FT>(1),
               py::arg("muls") = static_cast<FT>(3), py::arg("kernel_size") = 5,
               py::arg("kernel_spacing") = static_cast<FT>(1), py::arg("kernel_scale") = static_cast<FT>(1),
               py::arg("int_lower") = std::numeric_limits<IT>::lowest(),
               py::arg("int_upper") = std::numeric_limits<IT>::max(), "Construct a spatial-domain quadrature filter.")

          // --- Kernel size ---
          .def("kernel_size", static_cast<int (QS::*)() const>(&QS::kernelSize), "Get the kernel size.")
          .def("set_kernel_size", static_cast<void (QS::*)(int)>(&QS::kernelSize), py::arg("ks"),
               "Set the kernel size (odd, 3-99).")

          // --- Kernel spacing ---
          .def("kernel_spacing", static_cast<FT (QS::*)() const>(&QS::kernelSpacing), "Get the kernel spacing.")
          .def("set_kernel_spacing", static_cast<void (QS::*)(FT)>(&QS::kernelSpacing), py::arg("ks"),
               "Set the kernel spacing (must be positive).")

          // --- Kernel scale ---
          .def("kernel_scale", static_cast<FT (QS::*)() const>(&QS::kernelScale), "Get the kernel scale factor.")
          .def("set_kernel_scale", static_cast<void (QS::*)(FT)>(&QS::kernelScale), py::arg("ks"),
               "Set the kernel scale factor (must be positive).")

          // --- Scale ---
          .def("scale", static_cast<FT (QS::*)() const>(&QS::scale), "Get the filter scale.")
          .def("set_scale", static_cast<void (QS::*)(FT)>(&QS::scale), py::arg("s"),
               "Set the filter scale (must be positive).")

          // --- Muls (scale multiplier) ---
          .def("muls", static_cast<FT (QS::*)() const>(&QS::muls), "Get the scale multiplier.")
          .def("set_muls", static_cast<void (QS::*)(FT)>(&QS::muls), py::arg("m"),
               "Set the scale multiplier (must be positive).")

          // --- Kernel access ---
          .def("kernel", &QS::kernel, "Get the even filter kernel as numpy array.");

  bind_quadrature_common<QS>(cls);
}


// ============================================================================
// QuadratureSF binding
// ============================================================================

template <class IT, class FT>
void bind_quadrature_sf(py::module_& m, const std::string& suffix) {
  using QSF = QuadratureSF<IT, FT>;
  const std::string cls_name = "QuadratureSF" + suffix;

  auto cls =
      py::class_<QSF, ValueManager>(m, cls_name.c_str(),
                                    ("Frequency-domain quadrature filter (Difference of Poisson)" + suffix +
                                     ".\n\n"
                                     "Uses FFT-based Difference of Poisson filter in frequency domain.\n"
                                     "Functionally equivalent to QuadratureS but operates via FFT,\n"
                                     "which can be faster for large images.\n\n"
                                     "Constructor args:\n"
                                     "    scale: Filter scale. Default: 1.\n"
                                     "    muls: Scale multiplier / bandwidth. Default: 2.\n"
                                     "    kernel_spacing: Frequency spacing. Default: 1.\n"
                                     "    int_lower: Lower input intensity bound.\n"
                                     "    int_upper: Upper input intensity bound.\n\n"
                                     "ValueManager keys:\n"
                                     "    'grad_kernel_spacing', 'grad_scale', 'grad_muls'")
                                        .c_str())

          .def(py::init<FT, FT, FT, IT, IT>(), py::arg("scale") = static_cast<FT>(1),
               py::arg("muls") = static_cast<FT>(2), py::arg("kernel_spacing") = static_cast<FT>(1),
               py::arg("int_lower") = std::numeric_limits<IT>::lowest(),
               py::arg("int_upper") = std::numeric_limits<IT>::max(), "Construct a frequency-domain quadrature filter.")

          // --- Kernel spacing ---
          .def("kernel_spacing", static_cast<FT (QSF::*)() const>(&QSF::kernelSpacing), "Get the frequency spacing.")
          .def("set_kernel_spacing", static_cast<void (QSF::*)(FT)>(&QSF::kernelSpacing), py::arg("ks"),
               "Set the frequency spacing (must be positive).")

          // --- Scale ---
          .def("scale", static_cast<FT (QSF::*)() const>(&QSF::scale), "Get the filter scale.")
          .def("set_scale", static_cast<void (QSF::*)(FT)>(&QSF::scale), py::arg("s"),
               "Set the filter scale (must be positive).")

          // --- Muls (scale multiplier) ---
          .def("muls", static_cast<FT (QSF::*)() const>(&QSF::muls), "Get the scale multiplier.")
          .def("set_muls", static_cast<void (QSF::*)(FT)>(&QSF::muls), py::arg("m"),
               "Set the scale multiplier (must be positive).");

  bind_quadrature_common<QSF>(cls);
}


// ============================================================================
// Preset binding (all four quadrature types for one type combination)
// ============================================================================

template <class IT, class FT>
void bind_quadrature_preset(py::module_& m, const std::string& suffix) {
  bind_quadrature_g2<IT, FT>(m, suffix);
  bind_quadrature_lgf<IT, FT>(m, suffix);
  bind_quadrature_s<IT, FT, FT>(m, suffix);
  bind_quadrature_sf<IT, FT>(m, suffix);
}


// ============================================================================
// Explicit template instantiations
// ============================================================================

// Default preset: IT=uchar, FT=float
template void bind_quadrature_preset<uchar, float>(py::module_&, const std::string&);

// 32-bit float preset: IT=float, FT=float
template void bind_quadrature_preset<float, float>(py::module_&, const std::string&);

// 64-bit double preset: IT=double, FT=double
template void bind_quadrature_preset<double, double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
