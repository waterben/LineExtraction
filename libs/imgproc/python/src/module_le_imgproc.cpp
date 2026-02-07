/// @file module_le_imgproc.cpp
/// @brief pybind11 module definition for le_imgproc.
///
/// This is the entry point for the le_imgproc Python extension module.
/// It registers all imgproc bindings: core types, filter interfaces,
/// and concrete filter implementations.

#include "filter_binding.hpp"
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(le_imgproc, m) {
  m.doc() =
      "LineExtraction image processing module.\n\n"
      "Provides Python bindings for image filter interfaces and concrete\n"
      "implementations including gradient (Sobel, Scharr, Prewitt),\n"
      "Laplacian, and quadrature filters.\n\n"
      "Core types:\n"
      "    RangeD, RangeI, RangeF, RangeUChar - Value range classes\n"
      "    Value - Type-safe variant for parameters\n"
      "    ValueManager - Runtime parameter configuration base\n"
      "    FilterData - Filter output with value range\n\n"
      "Filter interfaces:\n"
      "    FilterI - Abstract base for all filters\n"
      "    GradientI - Abstract gradient computation interface\n"
      "    LaplaceI - Abstract Laplacian filter interface\n\n"
      "Concrete filters:\n"
      "    SobelGradient - Sobel derivative gradient\n"
      "    ScharrGradient - Scharr derivative gradient\n"
      "    PrewittGradient - Prewitt derivative gradient\n\n"
      "Example:\n"
      "    import numpy as np\n"
      "    import le_imgproc\n\n"
      "    img = np.zeros((100, 100), dtype=np.uint8)\n"
      "    img[30:70, 30:70] = 255  # White square\n\n"
      "    grad = le_imgproc.SobelGradient()\n"
      "    grad.process(img)\n"
      "    magnitude = grad.magnitude()\n"
      "    direction = grad.direction()";

  lsfm::python::bind_core_types(m);
  lsfm::python::bind_filter_interface(m);
  lsfm::python::bind_gradient_interface(m);
  lsfm::python::bind_laplace_interface(m);
  lsfm::python::bind_derivative_gradient(m);
}
