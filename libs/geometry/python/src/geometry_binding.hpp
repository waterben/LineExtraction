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

// --- 3D geometry types ---

/// @brief Bind Line3<FT> (3D infinite line).
template <class FT>
void bind_line3(pybind11::module_& m, const std::string& suffix);

/// @brief Bind LineSegment3<FT> (3D line segment).
template <class FT>
void bind_line_segment3(pybind11::module_& m, const std::string& suffix);

/// @brief Bind Plane<FT> (3D plane in Hesse normal form).
template <class FT>
void bind_plane(pybind11::module_& m, const std::string& suffix);

/// @brief Bind Pose<FT> (6-DOF rigid body pose).
template <class FT>
void bind_pose(pybind11::module_& m, const std::string& suffix);

/// @brief Bind Camera<FT> (pinhole camera model).
template <class FT>
void bind_camera(pybind11::module_& m, const std::string& suffix);

/// @brief Bind CameraHom<FT> (cached projection matrix).
template <class FT>
void bind_camera_hom(pybind11::module_& m, const std::string& suffix);

/// @brief Bind CameraPluecker<FT> (Plücker line projection).
template <class FT>
void bind_camera_pluecker(pybind11::module_& m, const std::string& suffix);

/// @brief Bind Camera2P<FT> (two-point line projection).
template <class FT>
void bind_camera_2p(pybind11::module_& m, const std::string& suffix);

/// @brief Bind CameraCV<FT> (OpenCV projection).
template <class FT>
void bind_camera_cv(pybind11::module_& m, const std::string& suffix);

/// @brief Convenience: bind all 3D geometry types for a given FT preset.
template <class FT>
void bind_geometry3d_preset(pybind11::module_& m, const std::string& suffix);

// --- Stereo triangulation ---

/// @brief Bind Stereo<FT> (ray-intersection stereo triangulation).
template <class FT>
void bind_stereo(pybind11::module_& m, const std::string& suffix);

/// @brief Bind StereoPlane<FT> (plane-intersection stereo triangulation).
template <class FT>
void bind_stereo_plane(pybind11::module_& m, const std::string& suffix);

/// @brief Bind StereoCV<FT> (OpenCV-based stereo triangulation).
template <class FT>
void bind_stereo_cv(pybind11::module_& m, const std::string& suffix);

/// @brief Convenience: bind all stereo triangulation types for a given FT preset.
template <class FT>
void bind_stereo_preset(pybind11::module_& m, const std::string& suffix);

}  // namespace python
}  // namespace lsfm
