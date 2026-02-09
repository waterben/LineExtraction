/// @file module_le_lsd.cpp
/// @brief pybind11 module definition for le_lsd.
///
/// This is the entry point for the le_lsd Python extension module.
/// It registers all line segment detector bindings: core types
/// (DataDescriptorEntry, flags), base classes (LdBase, LsdBase),
/// and concrete detectors for multiple floating-point type presets.
///
/// Line and LineSegment geometry types are provided by le_geometry
/// and re-exported here for backward compatibility.
///
/// ## Presets
///
///   | Suffix | FT     | Python example              |
///   |--------|--------|-----------------------------|
///   | (none) | float  | LsdCC()                     |
///   | _f64   | double | LsdCC_f64()                 |

#include "lsd_binding.hpp"
#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

PYBIND11_MODULE(le_lsd, m) {
  // Import le_geometry so that Line and LineSegment type bindings
  // are available as base/return types for LSD detectors.
  auto geometry = py::module_::import("le_geometry");

  // Import le_imgproc so that ValueManager, Value, Range, FilterData
  // type bindings are available as base classes for LSD types.
  py::module_::import("le_imgproc");

  // Import le_edge so EdgeSegment and related types are available.
  py::module_::import("le_edge");

  m.doc() =
      "LineExtraction line segment detection module.\n\n"
      "Provides Python bindings for line segment detector interfaces\n"
      "and concrete implementations including CC, CP, Burns, FBW,\n"
      "FGioi, EDLines, EL, EP, and HoughP detectors.\n\n"
      "Multiple floating-point type presets are available:\n\n"
      "  - Default (float):   LsdCC, LsdBurns, LsdFGioi, ...\n"
      "  - 64-bit (double):   LsdCC_f64, LsdBurns_f64, ...\n\n"
      "Core types:\n"
      "    Line             - 2D line in Hesse normal form (from le_geometry)\n"
      "    LineSegment      - 2D line segment with endpoints (from le_geometry)\n"
      "    DataDescriptorEntry - Auxiliary image data layer descriptor\n\n"
      "Detector flags:\n"
      "    CC_FIND_NEAR_COMPLEX, CC_CORNER_RULE, CC_ADD_THICK_PIXELS\n"
      "    BURNS_NMS, FBW_NMS, FBW_PATAN\n"
      "    EL_USE_NFA, EL_USE_PRECISE_SPE, EP_USE_PRECISE_SPE\n\n"
      "Example:\n"
      "    import numpy as np\n"
      "    import le_lsd\n\n"
      "    img = np.zeros((100, 100), dtype=np.uint8)\n"
      "    img[20:80, 40:60] = 255\n\n"
      "    detector = le_lsd.LsdCC()\n"
      "    detector.detect(img)\n"
      "    segments = detector.line_segments()\n"
      "    for seg in segments:\n"
      "        print(seg)";

  // ---- Re-export Line/LineSegment from le_geometry for backward compat ----
  m.attr("Line") = geometry.attr("Line");
  m.attr("LineSegment") = geometry.attr("LineSegment");
  m.attr("Line_f64") = geometry.attr("Line_f64");
  m.attr("LineSegment_f64") = geometry.attr("LineSegment_f64");

  // ---- Core types (DataDescriptorEntry, flag constants) ----
  lsfm::python::bind_lsd_core_types(m);

  // ---- Default preset: FT=float, suffix="" ----
  lsfm::python::bind_lsd_preset<float>(m, "");

  // ---- Double preset: FT=double, suffix="_f64" ----
  lsfm::python::bind_lsd_preset<double>(m, "_f64");
}
