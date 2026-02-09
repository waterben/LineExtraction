/// @file geometry_binding.hpp
/// @brief Declarations for geometry pybind11 bindings.
///
/// Provides templated binding functions for Line, LineSegment, Polygon,
/// and drawing utilities from the geometry library.

#pragma once

#include <pybind11/pybind11.h>

#include <string>

namespace lsfm {
namespace python {

/// @brief Bind Line<FT> class with full geometric API.
template <class FT>
void bind_line(pybind11::module_& m, const std::string& suffix);

/// @brief Bind LineSegment<FT> class with full geometric API.
template <class FT>
void bind_line_segment(pybind11::module_& m, const std::string& suffix);

/// @brief Bind Polygon<FT> class.
template <class FT>
void bind_polygon(pybind11::module_& m, const std::string& suffix);

/// @brief Bind drawing utilities (drawLines, drawLinesR, quiver, etc.).
template <class FT>
void bind_draw(pybind11::module_& m, const std::string& suffix);

/// @brief Bind LineOptimizer functions for gradient-magnitude-based line refinement.
template <class FT>
void bind_line_optimizer(pybind11::module_& m, const std::string& suffix);

/// @brief Bind visualization helper functions (dirColor, createNMS, etc.).
void bind_visualization(pybind11::module_& m);

/// @brief Convenience: bind all geometry types for a given FT preset.
template <class FT>
void bind_geometry_preset(pybind11::module_& m, const std::string& suffix);

}  // namespace python
}  // namespace lsfm
