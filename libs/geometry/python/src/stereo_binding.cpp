/// @file stereo_binding.cpp
/// @brief Templated pybind11 bindings for stereo triangulation classes.
///
/// Implements the templated binding functions for stereo vision:
///   - Stereo — ray-intersection triangulation
///   - StereoPlane — plane-intersection triangulation (non-rectified)
///   - StereoCV — OpenCV-based triangulation
///
///   | Suffix  | FT     |
///   |---------|--------|
///   | (none)  | float  |
///   | _f64    | double |

#include "geometry_binding.hpp"
#include <geometry/stereo.hpp>
#include <geometry/stereocv.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>
#include <string>
#include <vector>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Stereo<FT> binding — ray-intersection triangulation
// ============================================================================

template <class FT>
void bind_stereo(py::module_& m, const std::string& suffix) {
  using StereoT = Stereo<FT>;
  using CamT = Camera<FT>;
  using L = Line<FT, Vec2>;
  using LS = LineSegment<FT, Vec2>;
  using L3 = Line3<FT>;
  using LS3 = LineSegment3<FT>;
  const std::string cls = "Stereo" + suffix;

  py::class_<StereoT>(m, cls.c_str(),
                      ("Stereo triangulation via ray intersection" + suffix +
                       ".\n\n"
                       "Triangulates points, lines, and line segments from\n"
                       "corresponding observations in a rectified stereo pair.\n"
                       "Points are found at the midpoint of closest approach\n"
                       "between camera rays.")
                          .c_str())
      // --- Constructors ---
      .def(py::init<const CamT&, const CamT&>(), py::arg("cam_left"), py::arg("cam_right"),
           "Construct from left and right Camera objects.")

      // --- Point triangulation ---
      .def(
          "triangulate_point",
          [](const StereoT& s, FT lx, FT ly, FT rx, FT ry) {
            auto p = s.triangulate(Vec2<FT>(lx, ly), Vec2<FT>(rx, ry));
            return py::make_tuple(p[0], p[1], p[2]);
          },
          py::arg("left_x"), py::arg("left_y"), py::arg("right_x"), py::arg("right_y"),
          "Triangulate a single 3D point from stereo pixel correspondences.\n\n"
          "Returns (x, y, z) tuple.")
      .def(
          "triangulate_points",
          [](const StereoT& s, const std::vector<std::tuple<FT, FT>>& left,
             const std::vector<std::tuple<FT, FT>>& right) {
            std::vector<Vec2<FT>> lpts, rpts;
            lpts.reserve(left.size());
            rpts.reserve(right.size());
            for (const auto& t : left) lpts.emplace_back(std::get<0>(t), std::get<1>(t));
            for (const auto& t : right) rpts.emplace_back(std::get<0>(t), std::get<1>(t));
            std::vector<Vec3<FT>> out;
            s.triangulate(lpts, rpts, out);
            py::list result;
            for (const auto& p : out) result.append(py::make_tuple(p[0], p[1], p[2]));
            return result;
          },
          py::arg("left_points"), py::arg("right_points"),
          "Triangulate a list of point correspondences.\n\n"
          "Each point is a (x, y) tuple. Returns list of (x, y, z) tuples.")

      // --- Line triangulation ---
      .def(
          "triangulate_line",
          [](const StereoT& s, const L& left, const L& right) { return s.triangulate(left, right); },
          py::arg("left_line"), py::arg("right_line"),
          "Triangulate a 3D line from stereo line correspondences.\n\n"
          "Returns Line3 (empty if degenerate). Requires rectified setup.")
      .def(
          "triangulate_lines",
          [](const StereoT& s, const std::vector<L>& left, const std::vector<L>& right) {
            std::vector<L3> ret;
            s.triangulate(left, right, ret);
            return ret;
          },
          py::arg("left_lines"), py::arg("right_lines"), "Triangulate a list of line correspondences.")

      // --- Line segment triangulation ---
      .def(
          "triangulate_segment",
          [](const StereoT& s, const LS& left, const LS& right) { return s.triangulate(left, right); },
          py::arg("left_segment"), py::arg("right_segment"),
          "Triangulate a 3D line segment from stereo correspondences.\n\n"
          "Returns LineSegment3 (empty if degenerate). Requires rectified setup.")
      .def(
          "triangulate_segments",
          [](const StereoT& s, const std::vector<LS>& left, const std::vector<LS>& right) {
            std::vector<LS3> ret;
            s.triangulate(left, right, ret);
            return ret;
          },
          py::arg("left_segments"), py::arg("right_segments"), "Triangulate a list of line segment correspondences.")

      .def("__repr__", [cls](const StereoT&) { return cls + "(ray-intersection)"; });
}

// ============================================================================
// StereoPlane<FT> binding — plane-intersection triangulation
// ============================================================================

template <class FT>
void bind_stereo_plane(py::module_& m, const std::string& suffix) {
  using SPT = StereoPlane<FT>;
  using StereoT = Stereo<FT>;
  using CamT = Camera<FT>;
  using L = Line<FT, Vec2>;
  using LS = LineSegment<FT, Vec2>;
  using L3 = Line3<FT>;
  using LS3 = LineSegment3<FT>;
  const std::string cls = "StereoPlane" + suffix;

  py::class_<SPT, StereoT>(m, cls.c_str(),
                           ("Stereo triangulation via plane intersection" + suffix +
                            ".\n\n"
                            "Triangulates lines by intersecting the interpretation\n"
                            "planes from each camera. Works for non-rectified stereo\n"
                            "pairs. Inherits point triangulation from Stereo.")
                               .c_str())
      // --- Constructors ---
      .def(py::init<const CamT&, const CamT&>(), py::arg("cam_left"), py::arg("cam_right"),
           "Construct from left and right Camera objects.")

      // --- Line triangulation (overrides Stereo) ---
      .def(
          "triangulate_line", [](const SPT& s, const L& left, const L& right) { return s.triangulate(left, right); },
          py::arg("left_line"), py::arg("right_line"),
          "Triangulate a 3D line via interpretation plane intersection.\n\n"
          "Returns Line3 (empty if planes parallel).")
      .def(
          "triangulate_lines",
          [](const SPT& s, const std::vector<L>& left, const std::vector<L>& right) {
            std::vector<L3> ret;
            s.triangulate(left, right, ret);
            return ret;
          },
          py::arg("left_lines"), py::arg("right_lines"),
          "Triangulate a list of line correspondences via plane intersection.")

      // --- Line segment triangulation (overrides Stereo) ---
      .def(
          "triangulate_segment",
          [](const SPT& s, const LS& left, const LS& right) { return s.triangulate(left, right); },
          py::arg("left_segment"), py::arg("right_segment"),
          "Triangulate a 3D line segment via plane intersection.\n\n"
          "Returns LineSegment3 (empty if degenerate).")
      .def(
          "triangulate_segments",
          [](const SPT& s, const std::vector<LS>& left, const std::vector<LS>& right) {
            std::vector<LS3> ret;
            s.triangulate(left, right, ret);
            return ret;
          },
          py::arg("left_segments"), py::arg("right_segments"),
          "Triangulate a list of line segment correspondences via plane intersection.")

      .def("__repr__", [cls](const SPT&) { return cls + "(plane-intersection)"; });
}

// ============================================================================
// StereoCV<FT> binding — OpenCV triangulation
// ============================================================================

template <class FT>
void bind_stereo_cv(py::module_& m, const std::string& suffix) {
  using SCVT = StereoCV<FT>;
  using CamT = Camera<FT>;
  using L = Line<FT, Vec2>;
  using LS = LineSegment<FT, Vec2>;
  using L3 = Line3<FT>;
  using LS3 = LineSegment3<FT>;
  const std::string cls = "StereoCV" + suffix;

  py::class_<SCVT>(m, cls.c_str(),
                   ("OpenCV-based stereo triangulation" + suffix +
                    ".\n\n"
                    "Uses cv::triangulatePoints() for point triangulation.\n"
                    "Lines and segments are triangulated by triangulating\n"
                    "two points on each line.")
                       .c_str())
      // --- Constructors ---
      .def(py::init<const CamT&, const CamT&>(), py::arg("cam_left"), py::arg("cam_right"),
           "Construct from left and right Camera objects.")

      // --- Point triangulation ---
      .def(
          "triangulate_point",
          [](const SCVT& s, FT lx, FT ly, FT rx, FT ry) {
            auto p = s.triangulate(Vec2<FT>(lx, ly), Vec2<FT>(rx, ry));
            return py::make_tuple(p[0], p[1], p[2]);
          },
          py::arg("left_x"), py::arg("left_y"), py::arg("right_x"), py::arg("right_y"),
          "Triangulate a single 3D point using OpenCV.\n\n"
          "Returns (x, y, z) tuple.")
      .def(
          "triangulate_points",
          [](const SCVT& s, const std::vector<std::tuple<FT, FT>>& left, const std::vector<std::tuple<FT, FT>>& right) {
            std::vector<Vec2<FT>> lpts, rpts;
            lpts.reserve(left.size());
            rpts.reserve(right.size());
            for (const auto& t : left) lpts.emplace_back(std::get<0>(t), std::get<1>(t));
            for (const auto& t : right) rpts.emplace_back(std::get<0>(t), std::get<1>(t));
            std::vector<Vec3<FT>> out;
            s.triangulate(lpts, rpts, out);
            py::list result;
            for (const auto& p : out) result.append(py::make_tuple(p[0], p[1], p[2]));
            return result;
          },
          py::arg("left_points"), py::arg("right_points"),
          "Triangulate a list of point correspondences using OpenCV.\n\n"
          "Each point is a (x, y) tuple. Returns list of (x, y, z) tuples.")

      // --- Line triangulation ---
      .def(
          "triangulate_line", [](const SCVT& s, const L& left, const L& right) { return s.triangulate(left, right); },
          py::arg("left_line"), py::arg("right_line"),
          "Triangulate a 3D line using OpenCV.\n\n"
          "Returns Line3 (empty if degenerate).")
      .def(
          "triangulate_lines",
          [](const SCVT& s, const std::vector<L>& left, const std::vector<L>& right) {
            std::vector<L3> ret;
            s.triangulate(left, right, ret);
            return ret;
          },
          py::arg("left_lines"), py::arg("right_lines"), "Triangulate a list of line correspondences using OpenCV.")

      // --- Line segment triangulation ---
      .def(
          "triangulate_segment",
          [](const SCVT& s, const LS& left, const LS& right) { return s.triangulate(left, right); },
          py::arg("left_segment"), py::arg("right_segment"),
          "Triangulate a 3D line segment using OpenCV.\n\n"
          "Returns LineSegment3 (empty if degenerate).")
      .def(
          "triangulate_segments",
          [](const SCVT& s, const std::vector<LS>& left, const std::vector<LS>& right) {
            std::vector<LS3> ret;
            s.triangulate(left, right, ret);
            return ret;
          },
          py::arg("left_segments"), py::arg("right_segments"),
          "Triangulate a list of line segment correspondences using OpenCV.")

      .def("__repr__", [cls](const SCVT&) { return cls + "(opencv)"; });
}

// ============================================================================
// Preset convenience — bind all stereo types for a given FT
// ============================================================================

template <class FT>
void bind_stereo_preset(py::module_& m, const std::string& suffix) {
  bind_stereo<FT>(m, suffix);
  bind_stereo_plane<FT>(m, suffix);
  bind_stereo_cv<FT>(m, suffix);
}

// --- Explicit template instantiations ---
template void bind_stereo<float>(py::module_&, const std::string&);
template void bind_stereo<double>(py::module_&, const std::string&);

template void bind_stereo_plane<float>(py::module_&, const std::string&);
template void bind_stereo_plane<double>(py::module_&, const std::string&);

template void bind_stereo_cv<float>(py::module_&, const std::string&);
template void bind_stereo_cv<double>(py::module_&, const std::string&);

template void bind_stereo_preset<float>(py::module_&, const std::string&);
template void bind_stereo_preset<double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
