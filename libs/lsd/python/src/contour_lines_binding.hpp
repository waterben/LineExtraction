/// @file contour_lines_binding.hpp
/// @brief pybind11 binding for contour-to-line-segment conversion.
///
/// Exposes the C++ RamerSplit + EigenFit pipeline to Python so that
/// raw contour point sequences (e.g. from AI segmentation) can be
/// converted to LineSegment objects without running a full LSD detector.

#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace lsfm {
namespace python {

/// @brief Bind the contour-to-line-segments utility functions.
/// @param m The pybind11 module to add bindings to
void bind_contour_lines(py::module_& m);

}  // namespace python
}  // namespace lsfm
