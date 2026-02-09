/// @file module_le_imgproc.cpp
/// @brief pybind11 module definition for le_imgproc.
///
/// This is the entry point for the le_imgproc Python extension module.
/// It registers all imgproc bindings: core types, filter interfaces,
/// and concrete filter implementations for multiple pixel type presets.
///
/// ## Presets
///
///   | Suffix | IT     | GT     | MT    | DT    | LT     | Python example          |
///   |--------|--------|--------|-------|-------|--------|-------------------------|
///   | (none) | uchar  | short  | float | float | int    | SobelGradient()         |
///   | _16u   | ushort | float  | float | float | int    | SobelGradient_16u()     |
///   | _f32   | float  | float  | float | float | float  | SobelGradient_f32()     |
///   | _f64   | double | double | double| double| double | SobelGradient_f64()     |

#include "filter_binding.hpp"
#include <opencv2/core.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(le_imgproc, m) {
  m.doc() =
      "LineExtraction image processing module.\n\n"
      "Provides Python bindings for image filter interfaces and concrete\n"
      "implementations including gradient (Sobel, Scharr, Prewitt) and\n"
      "Laplacian filters.\n\n"
      "Multiple pixel type presets are available:\n\n"
      "  - Default (uint8):  SobelGradient, ScharrGradient, ...\n"
      "  - 16-bit unsigned:  SobelGradient_16u, ScharrGradient_16u, ...\n"
      "  - 32-bit float:     SobelGradient_f32, ScharrGradient_f32, ...\n"
      "  - 64-bit double:    SobelGradient_f64, ScharrGradient_f64, ...\n\n"
      "Core types:\n"
      "    RangeD, RangeF, RangeI, RangeS      - Value range classes\n"
      "    RangeUChar, RangeUShort              - Unsigned value ranges\n"
      "    Value                                - Type-safe variant\n"
      "    ValueManager                         - Runtime parameter config\n"
      "    FilterData                           - Filter output + range\n\n"
      "Example:\n"
      "    import numpy as np\n"
      "    import le_imgproc\n\n"
      "    # 8-bit image (default preset)\n"
      "    img = np.zeros((100, 100), dtype=np.uint8)\n"
      "    img[30:70, 30:70] = 255\n"
      "    grad = le_imgproc.SobelGradient()\n"
      "    grad.process(img)\n"
      "    mag = grad.magnitude()\n\n"
      "    # Float image (_f32 preset)\n"
      "    img_f = img.astype(np.float32) / 255.0\n"
      "    grad_f = le_imgproc.SobelGradient_f32(0.0, 1.0)\n"
      "    grad_f.process(img_f)";

  // Shared core types (Range variants, Value, ValueManager, FilterData).
  lsfm::python::bind_core_types(m);

  // Default preset: 8-bit unsigned (no suffix — backwards compatible).
  lsfm::python::bind_filter_preset<uchar, short, float, float, int>(m, "");

  // 16-bit unsigned preset.
  lsfm::python::bind_filter_preset<ushort, float, float, float, int>(m, "_16u");

  // 32-bit float preset.
  lsfm::python::bind_filter_preset<float, float, float, float, float>(m, "_f32");

  // 64-bit double preset.
  lsfm::python::bind_filter_preset<double, double, double, double, double>(m, "_f64");
}
