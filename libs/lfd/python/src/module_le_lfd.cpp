/// @file module_le_lfd.cpp
/// @brief pybind11 module definition for le_lfd.
///
/// This is the entry point for the le_lfd Python extension module.
/// It registers all line feature descriptor bindings: match types,
/// descriptor types, descriptor creators, feature filters, and
/// matchers for multiple floating-point type presets.
///
/// Line and LineSegment geometry types are provided by le_geometry
/// (via le_lsd) and re-exported here for convenience.
///
/// ## Presets
///
///   | Suffix | FT     | Python example                     |
///   |--------|--------|------------------------------------|
///   | (none) | float  | FdcLBD(dx, dy)                     |
///   | _f64   | double | FdcLBD_f64(dx, dy)                 |

#include "lfd_binding.hpp"
#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

PYBIND11_MODULE(le_lfd, m) {
  // Import upstream modules so geometry, imgproc, and lsd types
  // are available as base/return types.
  auto geometry = py::module_::import("le_geometry");
  py::module_::import("le_imgproc");
  py::module_::import("le_edge");
  py::module_::import("le_lsd");

  m.doc() =
      "LineExtraction line feature descriptor module.\n\n"
      "Provides Python bindings for line feature descriptors,\n"
      "matchers, and outlier filters.\n\n"
      "Multiple floating-point type presets are available:\n\n"
      "  - Default (float):   FdcLBD, BruteForceLBD, ...\n"
      "  - 64-bit (double):   FdcLBD_f64, BruteForceLBD_f64, ...\n\n"
      "Descriptor types:\n"
      "    FdMat            - Simple cv::Mat feature descriptor\n"
      "    FdLBD            - LBD (Line Band Descriptor)\n\n"
      "Descriptor creators:\n"
      "    FdcLBD           - LBD descriptor creator from gradients\n"
      "    FdcGenericLR     - Left-Right descriptor with gradient+image bands\n\n"
      "Matchers:\n"
      "    BruteForceLBD    - Brute force matcher for LBD\n"
      "    BruteForceLR     - Brute force matcher for LR descriptors\n\n"
      "Filters:\n"
      "    GlobalRotationFilter - Rotation-based match filter\n"
      "    StereoLineFilter     - Stereo geometry match filter\n\n"
      "Match types:\n"
      "    FeatureMatch         - Index pair with filter state\n"
      "    DescriptorMatch      - Index pair + distance\n\n"
      "Example:\n"
      "    import numpy as np\n"
      "    import cv2\n"
      "    import le_lsd\n"
      "    import le_lfd\n\n"
      "    img = cv2.imread('image.png', cv2.IMREAD_GRAYSCALE)\n"
      "    detector = le_lsd.LsdCC()\n"
      "    detector.detect(img)\n"
      "    segments = detector.line_segments()\n\n"
      "    # Build descriptors from gradients\n"
      "    gx = detector.image_data_by_name('gx')\n"
      "    gy = detector.image_data_by_name('gy')\n"
      "    lbd = le_lfd.FdcLBD(gx, gy)\n"
      "    descriptors = lbd.create_list(segments)\n\n"
      "    # Match descriptors from two images\n"
      "    matcher = le_lfd.BruteForceLBD()\n"
      "    matcher.train(descs1, descs2)\n"
      "    matches = matcher.best()";

  // ---- Re-export Line/LineSegment from le_geometry for convenience ----
  m.attr("Line") = geometry.attr("Line");
  m.attr("LineSegment") = geometry.attr("LineSegment");
  m.attr("Line_f64") = geometry.attr("Line_f64");
  m.attr("LineSegment_f64") = geometry.attr("LineSegment_f64");

  // ---- Core types (filter-state constants) ----
  lsfm::python::bind_lfd_core_types(m);

  // ---- Default preset: FT=float, suffix="" ----
  lsfm::python::bind_lfd_preset<float>(m, "");

  // ---- Double preset: FT=double, suffix="_f64" ----
  lsfm::python::bind_lfd_preset<double>(m, "_f64");
}
