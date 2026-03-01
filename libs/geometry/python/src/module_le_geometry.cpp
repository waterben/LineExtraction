/// @file module_le_geometry.cpp
/// @brief PYBIND11_MODULE entry point for le_geometry Python bindings.
///
/// Exposes geometry types (Line, LineSegment, Polygon, Line3, LineSegment3,
/// Plane, Pose, Camera hierarchy) and drawing utilities for float and double
/// presets.

#include "geometry_binding.hpp"
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(le_geometry, m) {
  m.doc() =
      "LineExtraction geometry library — 2D/3D line, segment, polygon,\n"
      "camera models, and drawing utilities.";

  // Visualization helpers (non-templated, called once)
  lsfm::python::bind_visualization(m);

  // Float preset (default, no suffix)
  lsfm::python::bind_geometry_preset<float>(m, "");
  lsfm::python::bind_geometry3d_preset<float>(m, "");

  // Double preset
  lsfm::python::bind_geometry_preset<double>(m, "_f64");
  lsfm::python::bind_geometry3d_preset<double>(m, "_f64");
}
