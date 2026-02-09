/// @file geometry_binding.cpp
/// @brief Templated pybind11 bindings for geometry types and drawing utilities.
///
/// Implements the templated binding functions declared in geometry_binding.hpp
/// and provides explicit template instantiations for the standard presets:
///
///   | Suffix  | FT     |
///   |---------|--------|
///   | (none)  | float  |
///   | _f64    | double |

#include "geometry_binding.hpp"

#include <cvnp/cvnp.h>
#include <eval/results.hpp>
#include <geometry/draw.hpp>
#include <geometry/line.hpp>
#include <geometry/line_optimizer.hpp>
#include <geometry/polygon.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Line<FT> binding — full geometric API
// ============================================================================

template <class FT>
void bind_line(py::module_& m, const std::string& suffix) {
  using LineT = Line<FT, Vec2>;
  const std::string cls = "Line" + suffix;

  py::class_<LineT>(m, cls.c_str(),
                    ("2D line in Hesse normal form (n*p = d)" + suffix +
                     ".\n\n"
                     "Represents a 2D infinite line via unit normal (nx, ny) and\n"
                     "signed perpendicular distance d to the origin. The normal\n"
                     "direction corresponds to the gradient direction (dark to bright).")
                        .c_str())
      // --- Constructors ---
      .def(py::init<>(), "Construct a zero (degenerate) line.")
      .def(py::init<FT, FT, FT>(), py::arg("normal_x"), py::arg("normal_y"), py::arg("distance"),
           "Construct from normal components and signed distance.")
      .def(py::init<FT, FT, FT, FT>(), py::arg("normal_x"), py::arg("normal_y"), py::arg("point_x"), py::arg("point_y"),
           "Construct from normal and a point on the line.")
      .def_static(
          "from_angle", [](FT angle, FT dist) { return LineT(angle, dist); }, py::arg("normal_angle"),
          py::arg("distance"), "Construct from normal angle (radians) and distance.")

      // --- Attribute accessors ---
      .def_property_readonly(
          "normal_x", [](const LineT& l) { return l.normalX(); }, "X-component of the unit normal.")
      .def_property_readonly(
          "normal_y", [](const LineT& l) { return l.normalY(); }, "Y-component of the unit normal.")
      .def_property_readonly(
          "origin_dist", [](const LineT& l) { return l.originDist(); }, "Signed perpendicular distance to origin.")
      .def_property_readonly(
          "normal_angle", [](const LineT& l) { return l.normalAngle(); },
          "Angle of the normal vector in radians [-PI, PI).")
      .def_property_readonly(
          "angle", [](const LineT& l) { return l.angle(); }, "Precise line angle to x-axis in radians [-PI, PI).")
      .def_property_readonly(
          "gradient_angle", [](const LineT& l) { return l.gradientAngle(); }, "Gradient angle in radians [-PI, PI).")
      .def_property_readonly(
          "origin_x", [](const LineT& l) { return l.originX(); }, "X-coordinate of closest point to origin.")
      .def_property_readonly(
          "origin_y", [](const LineT& l) { return l.originY(); }, "Y-coordinate of closest point to origin.")
      .def(
          "origin_point", [](const LineT& l) { return py::make_tuple(l.originX(), l.originY()); },
          "Get closest point to origin as (x, y) tuple.")
      .def_property_readonly(
          "direction_x", [](const LineT& l) { return l.directionX(); }, "X-component of line direction.")
      .def_property_readonly(
          "direction_y", [](const LineT& l) { return l.directionY(); }, "Y-component of line direction.")
      .def(
          "direction", [](const LineT& l) { return py::make_tuple(l.directionX(), l.directionY()); },
          "Get line direction as (dx, dy) tuple.")
      .def(
          "normal", [](const LineT& l) { return py::make_tuple(l.normalX(), l.normalY()); },
          "Get normal as (nx, ny) tuple.")
      .def(
          "valid", [](const LineT& l) { return l.valid(); }, "True if the normal is unit length (non-degenerate).")
      .def(
          "empty", [](const LineT& l) { return l.empty(); }, "True if the normal is zero (degenerate line).")

      // --- Distance and projection ---
      .def(
          "distance", [](const LineT& l, FT x, FT y) { return l.distance(x, y); }, py::arg("x"), py::arg("y"),
          "Signed perpendicular distance from point (x, y) to this line.")
      .def(
          "project", [](const LineT& l, FT x, FT y) { return l.project(x, y); }, py::arg("x"), py::arg("y"),
          "Project point onto line direction; returns signed distance along line.")
      .def(
          "normal_project", [](const LineT& l, FT x, FT y) { return l.normalProject(x, y); }, py::arg("x"),
          py::arg("y"), "Project point onto normal direction; returns signed distance along normal.")

      // --- Line evaluation ---
      .def(
          "x_at", [](const LineT& l, FT y) { return l.x(y); }, py::arg("y"), "Compute x on line for given y.")
      .def(
          "y_at", [](const LineT& l, FT x) { return l.y(x); }, py::arg("x"), "Compute y on line for given x.")

      // --- Coordinate transforms ---
      .def(
          "line2world",
          [](const LineT& l, FT x, FT y) {
            auto p = l.line2world(x, y);
            return py::make_tuple(getX(p), getY(p));
          },
          py::arg("x"), py::arg("y"), "Transform point from line coordinates to world coordinates.")
      .def(
          "world2line",
          [](const LineT& l, FT x, FT y) {
            auto p = l.world2line(x, y);
            return py::make_tuple(getX(p), getY(p));
          },
          py::arg("x"), py::arg("y"), "Transform point from world coordinates to line coordinates.")

      // --- Intersection ---
      .def(
          "is_parallel", [](const LineT& l, const LineT& other) { return l.isParallel(other); }, py::arg("other"),
          "Check if another line is parallel to this one.")
      .def(
          "intersection",
          [](const LineT& l, const LineT& other) -> py::object {
            FT x, y;
            if (l.intersection(other, x, y)) {
              return py::make_tuple(x, y);
            }
            return py::none();
          },
          py::arg("other"), "Compute intersection with another line. Returns (x, y) or None if parallel.")
      .def(
          "angle_to", [](const LineT& l, const LineT& other) { return l.angle(other); }, py::arg("other"),
          "Angle between this line and another (radians).")

      // --- Transforms (return new line) ---
      .def(
          "translated_ortho",
          [](const LineT& l, FT dist) {
            LineT copy(l);
            copy.translateOrtho(dist);
            return copy;
          },
          py::arg("distance"), "Return new line translated orthogonally by distance.")
      .def(
          "translated_to",
          [](const LineT& l, FT x, FT y) {
            LineT copy(l);
            copy.translateTo(x, y);
            return copy;
          },
          py::arg("x"), py::arg("y"), "Return new line passing through (x, y).")
      .def(
          "rotated",
          [](const LineT& l, FT angle) {
            LineT copy(l);
            copy.rotate(angle);
            return copy;
          },
          py::arg("angle"), "Return new line rotated by angle (radians) about its origin.")
      .def(
          "rotated_around",
          [](const LineT& l, FT angle, FT x, FT y) {
            LineT copy(l);
            copy.rotate(angle, x, y);
            return copy;
          },
          py::arg("angle"), py::arg("x"), py::arg("y"), "Return new line rotated about (x, y).")
      .def(
          "scaled",
          [](const LineT& l, FT s) {
            LineT copy(l);
            copy.scale(s);
            return copy;
          },
          py::arg("scale"), "Return new line scaled about origin.")
      .def(
          "normal_flipped",
          [](const LineT& l) {
            LineT copy(l);
            copy.normalFlip();
            return copy;
          },
          "Return new line with flipped normal direction.")

      // --- In-place transforms ---
      .def(
          "translate_ortho", [](LineT& l, FT dist) { l.translateOrtho(dist); }, py::arg("distance"),
          "Translate line orthogonally in-place.")
      .def(
          "translate_to", [](LineT& l, FT x, FT y) { l.translateTo(x, y); }, py::arg("x"), py::arg("y"),
          "Move line to pass through (x, y) in-place.")
      .def(
          "rotate", [](LineT& l, FT angle) { l.rotate(angle); }, py::arg("angle"), "Rotate line about origin in-place.")
      .def(
          "scale", [](LineT& l, FT s) { l.scale(s); }, py::arg("scale"), "Scale line about origin in-place.")
      .def("normal_flip", &LineT::normalFlip, "Flip normal direction in-place.")

      // --- Drawing ---
      .def(
          "draw",
          [](const LineT& l, cv::Mat& img, const std::vector<double>& color, int thickness, int line_type,
             double normal_length, double tip_length) {
            cv::Scalar c(color.size() > 0 ? color[0] : 0, color.size() > 1 ? color[1] : 0,
                         color.size() > 2 ? color[2] : 255);
            l.draw(img, c, thickness, line_type, normal_length, tip_length);
          },
          py::arg("image"), py::arg("color") = std::vector<double>{0, 0, 255}, py::arg("thickness") = 1,
          py::arg("line_type") = 8, py::arg("normal_length") = 0.0, py::arg("tip_length") = 0.0,
          "Draw this line on an image.")

      .def("__repr__", [](const LineT& l) {
        std::ostringstream os;
        os << "Line(nx=" << l.normalX() << ", ny=" << l.normalY() << ", d=" << l.originDist() << ")";
        return os.str();
      });
}

// ============================================================================
// LineSegment<FT> binding — full geometric API
// ============================================================================

template <class FT>
void bind_line_segment(py::module_& m, const std::string& suffix) {
  using LineT = Line<FT, Vec2>;
  using LST = LineSegment<FT, Vec2>;
  const std::string cls = "LineSegment" + suffix;

  py::class_<LST, LineT>(m, cls.c_str(),
                         ("2D line segment" + suffix +
                          ".\n\n"
                          "Extends Line with start/end distances along the line\n"
                          "direction, defining a finite segment. Provides endpoint\n"
                          "coordinates, length, trimming, overlap, and transforms.")
                             .c_str())
      // --- Constructors ---
      .def(py::init<>(), "Construct a zero (degenerate) segment.")
      .def(py::init<FT, FT, FT, FT, FT, int>(), py::arg("normal_x"), py::arg("normal_y"), py::arg("distance"),
           py::arg("line_beg"), py::arg("line_end"), py::arg("octave") = 0,
           "Construct from normal, distance, and line extents.")
      .def_static(
          "from_endpoints",
          [](FT x1, FT y1, FT x2, FT y2, int octave) {
            Vec4<FT> pts(x1, y1, x2, y2);
            return LST(pts, octave);
          },
          py::arg("x1"), py::arg("y1"), py::arg("x2"), py::arg("y2"), py::arg("octave") = 0,
          "Construct from endpoint coordinates.")

      // --- Segment attributes ---
      .def_property_readonly(
          "start", [](const LST& ls) { return ls.start(); }, "Start distance along line direction.")
      .def_property_readonly(
          "end", [](const LST& ls) { return ls.end(); }, "End distance along line direction.")
      .def_property_readonly(
          "center_dist", [](const LST& ls) { return ls.center(); }, "Center distance along line direction.")
      .def_property_readonly(
          "length", [](const LST& ls) { return ls.length(); }, "Length of the segment.")
      .def_property_readonly(
          "octave", [](const LST& ls) { return ls.octave(); }, "Detection octave level.")

      // --- Point accessors ---
      .def(
          "start_point",
          [](const LST& ls) {
            auto p = ls.startPoint();
            return py::make_tuple(getX(p), getY(p));
          },
          "Get start point as (x, y) tuple.")
      .def(
          "end_point",
          [](const LST& ls) {
            auto p = ls.endPoint();
            return py::make_tuple(getX(p), getY(p));
          },
          "Get end point as (x, y) tuple.")
      .def(
          "center_point",
          [](const LST& ls) {
            auto p = ls.centerPoint();
            return py::make_tuple(getX(p), getY(p));
          },
          "Get center point as (x, y) tuple.")
      .def(
          "end_points",
          [](const LST& ls) {
            auto ep = ls.endPoints();
            return py::make_tuple(ep[0], ep[1], ep[2], ep[3]);
          },
          "Get endpoints as (x1, y1, x2, y2) tuple.")

      // --- Range checking ---
      .def(
          "in_range", [](const LST& ls, FT dist) { return ls.inRange(dist); }, py::arg("distance"),
          "Check if a line-direction distance is within the segment range.")
      .def(
          "in_range_point", [](const LST& ls, FT x, FT y) { return ls.inRange(x, y); }, py::arg("x"), py::arg("y"),
          "Check if a point's projection falls within the segment.")
      .def(
          "in_range_tol", [](const LST& ls, FT x, FT y, FT tol) { return ls.inRangeTol(x, y, tol); }, py::arg("x"),
          py::arg("y"), py::arg("tolerance"), "Check if a point's projection falls within the segment with tolerance.")

      // --- Trimming and overlap ---
      .def(
          "trim_to_box",
          [](const LST& ls, FT max_x, FT max_y, FT min_x, FT min_y) {
            LST copy(ls);
            copy.trim2Box(max_x, max_y, min_x, min_y);
            return copy;
          },
          py::arg("max_x"), py::arg("max_y"), py::arg("min_x") = FT(0), py::arg("min_y") = FT(0),
          "Return segment trimmed to bounding box.")
      .def(
          "check_overlap", [](LST& ls, LST other) { return ls.checkOverlap(other); }, py::arg("other"),
          "Check if this segment overlaps with another.")
      .def(
          "error", [](const LST& ls, const LineT& ref) { return ls.error(ref); }, py::arg("reference_line"),
          "Compute error of this segment's endpoints w.r.t. a reference line.")

      // --- Transforms (return new segment) ---
      .def(
          "translated",
          [](const LST& ls, FT dist) {
            LST copy(ls);
            copy.translate(dist);
            return copy;
          },
          py::arg("distance"), "Return segment translated along line direction.")
      .def(
          "rotated",
          [](const LST& ls, FT angle, FT pivot) {
            LST copy(ls);
            copy.rotate(angle, pivot);
            return copy;
          },
          py::arg("angle"), py::arg("pivot"), "Return segment rotated about a pivot distance along line direction.")
      .def(
          "scaled",
          [](const LST& ls, FT s) {
            LST copy(ls);
            copy.scale(s);
            return copy;
          },
          py::arg("scale"), "Return segment scaled about origin.")
      .def(
          "scaled_around",
          [](const LST& ls, FT s, FT pivot) {
            LST copy(ls);
            copy.scale(s, pivot);
            return copy;
          },
          py::arg("scale"), py::arg("pivot"), "Return segment scaled about a pivot distance along line.")
      .def(
          "normal_flipped",
          [](const LST& ls) {
            LST copy(ls);
            copy.normalFlip();
            return copy;
          },
          "Return segment with flipped normal direction.")
      .def(
          "endpoint_swapped",
          [](const LST& ls) {
            LST copy(ls);
            copy.endPointSwap();
            return copy;
          },
          "Return segment with swapped start/end points.")

      // --- Drawing ---
      .def(
          "draw",
          [](const LST& ls, cv::Mat& img, const std::vector<double>& color, int thickness, int line_type,
             double normal_length, double tip_length) {
            cv::Scalar c(color.size() > 0 ? color[0] : 0, color.size() > 1 ? color[1] : 0,
                         color.size() > 2 ? color[2] : 255);
            ls.draw(img, c, thickness, line_type, normal_length, tip_length);
          },
          py::arg("image"), py::arg("color") = std::vector<double>{0, 0, 255}, py::arg("thickness") = 1,
          py::arg("line_type") = 8, py::arg("normal_length") = 0.0, py::arg("tip_length") = 0.0,
          "Draw this segment on an image.")

      .def("__repr__", [](const LST& ls) {
        auto sp = ls.startPoint();
        auto ep = ls.endPoint();
        std::ostringstream os;
        os << "LineSegment((" << getX(sp) << ", " << getY(sp) << ") -> (" << getX(ep) << ", " << getY(ep)
           << "), len=" << ls.length() << ")";
        return os.str();
      });
}

// ============================================================================
// Polygon<FT> binding
// ============================================================================

template <class FT>
void bind_polygon(py::module_& m, const std::string& suffix) {
  using PolyT = Polygon<FT, Vec2>;
  const std::string cls = "Polygon" + suffix;

  py::class_<PolyT>(m, cls.c_str(),
                    ("2D polygon" + suffix +
                     ".\n\n"
                     "A polygon defined by vertices relative to a pivot point.\n"
                     "Supports construction from line segments, transforms, and drawing.")
                        .c_str())
      .def(py::init<>(), "Construct an empty polygon at origin.")
      .def(
          "add_vertex",
          [](PolyT& p, FT x, FT y) {
            typename PolyT::point_type pt(x, y);
            p.push_back(pt);
          },
          py::arg("x"), py::arg("y"), "Add a vertex in local coordinates.")
      .def(
          "add_world_vertex",
          [](PolyT& p, FT x, FT y) {
            typename PolyT::point_type pt(x, y);
            p.push_back_world(pt);
          },
          py::arg("x"), py::arg("y"), "Add a vertex in world coordinates.")
      .def_property_readonly(
          "size", [](const PolyT& p) { return p.size(); }, "Number of vertices.")
      .def_property_readonly(
          "empty", [](const PolyT& p) { return p.empty(); }, "True if polygon has no vertices.")
      .def_property_readonly(
          "is_convex", [](const PolyT& p) { return p.isConvex(); }, "True if polygon is convex.")
      .def(
          "pivot",
          [](const PolyT& p) {
            auto pv = p.piviot();
            return py::make_tuple(getX(pv), getY(pv));
          },
          "Get pivot point as (x, y) tuple.")
      .def(
          "vertices",
          [](const PolyT& p) {
            py::list result;
            for (const auto& v : p.verticies()) {
              result.append(py::make_tuple(getX(v), getY(v)));
            }
            return result;
          },
          "Get local vertices as list of (x, y) tuples.")
      .def(
          "world_vertices",
          [](const PolyT& p) {
            py::list result;
            // Manually compute world vertices (add pivot to each vertex)
            // because wolrdVerticies() has a const-correctness issue.
            auto pivot = p.piviot();
            for (const auto& v : p.verticies()) {
              auto wx = getX(v) + getX(pivot);
              auto wy = getY(v) + getY(pivot);
              result.append(py::make_tuple(wx, wy));
            }
            return result;
          },
          "Get world vertices as list of (x, y) tuples.")
      .def(
          "draw",
          [](const PolyT& p, cv::Mat& img, const std::vector<double>& color, int thickness, int line_type) {
            cv::Scalar c(color.size() > 0 ? color[0] : 0, color.size() > 1 ? color[1] : 0,
                         color.size() > 2 ? color[2] : 255);
            // Use the global draw function
            lsfm::draw(p, img, c, thickness, line_type);
          },
          py::arg("image"), py::arg("color") = std::vector<double>{0, 0, 255}, py::arg("thickness") = 1,
          py::arg("line_type") = 8, "Draw polygon outline on image.")
      .def(
          "fill",
          [](const PolyT& p, cv::Mat& img, const std::vector<double>& color) {
            cv::Scalar c(color.size() > 0 ? color[0] : 0, color.size() > 1 ? color[1] : 0,
                         color.size() > 2 ? color[2] : 255);
            lsfm::fill(p, img, c);
          },
          py::arg("image"), py::arg("color") = std::vector<double>{0, 255, 0}, "Fill polygon on image.")
      .def("__repr__", [](const PolyT& p) {
        std::ostringstream os;
        os << "Polygon(vertices=" << p.size() << ", convex=" << (p.isConvex() ? "True" : "False") << ")";
        return os.str();
      });
}

// ============================================================================
// Drawing utilities
// ============================================================================

template <class FT>
void bind_draw(py::module_& m, const std::string& suffix) {
  using LST = LineSegment<FT, Vec2>;
  using LSVec = std::vector<LST>;

  // --- Image-creating drawing functions ---
  m.def(("draw_lines" + suffix).c_str(),
        [](const cv::Mat& img, const LSVec& segs, bool ids, const std::vector<double>& color, int thickness,
           int line_type) {
          cv::Scalar c(color.size() > 0 ? color[0] : 0, color.size() > 1 ? color[1] : 0,
                       color.size() > 2 ? color[2] : 255);
          return drawLines<FT>(img, segs, ids, c, thickness, line_type, FT(0), FT(0));
        },
        py::arg("image"), py::arg("segments"), py::arg("show_ids") = false,
        py::arg("color") = std::vector<double>{0, 0, 255}, py::arg("thickness") = 1, py::arg("line_type") = 8,
        "Draw line segments on a copy of the image with uniform color.");

  m.def(("draw_lines_labeled" + suffix).c_str(),
        [](const cv::Mat& img, const LSVec& segs, const std::vector<std::string>& labels,
           const std::vector<double>& color, int thickness, int line_type) {
          cv::Scalar c(color.size() > 0 ? color[0] : 0, color.size() > 1 ? color[1] : 0,
                       color.size() > 2 ? color[2] : 255);
          return drawLines<FT>(img, segs, labels, c, thickness, line_type, FT(0), FT(0));
        },
        py::arg("image"), py::arg("segments"), py::arg("labels"), py::arg("color") = std::vector<double>{0, 0, 255},
        py::arg("thickness") = 1, py::arg("line_type") = 8, "Draw line segments with string labels.");

  m.def(("draw_lines_random" + suffix).c_str(),
        [](const cv::Mat& img, const LSVec& segs, bool ids, int thickness, int line_type) {
          return drawLinesR<FT>(img, segs, ids, thickness, line_type, FT(0), FT(0));
        },
        py::arg("image"), py::arg("segments"), py::arg("show_ids") = false, py::arg("thickness") = 1,
        py::arg("line_type") = 8, "Draw line segments with random colors.");

  // --- In-place drawing ---
  m.def(("draw_lines_inplace" + suffix).c_str(),
        [](cv::Mat& img, const LSVec& segs, const std::vector<double>& color, int thickness, int line_type) {
          cv::Scalar c(color.size() > 0 ? color[0] : 0, color.size() > 1 ? color[1] : 0,
                       color.size() > 2 ? color[2] : 255);
          lines(img, segs, c, thickness, line_type);
        },
        py::arg("image"), py::arg("segments"), py::arg("color") = std::vector<double>{0, 0, 255},
        py::arg("thickness") = 1, py::arg("line_type") = 8, "Draw line segments in-place on the image.");

  m.def(("draw_lines_random_inplace" + suffix).c_str(),
        [](cv::Mat& img, const LSVec& segs, int thickness, int line_type) { linesR(img, segs, thickness, line_type); },
        py::arg("image"), py::arg("segments"), py::arg("thickness") = 1, py::arg("line_type") = 8,
        "Draw line segments with random colors in-place.");

  // --- Trim free function ---
  m.def(("trim_to_box" + suffix).c_str(),
        [](const Line<FT, Vec2>& l, FT max_x, FT max_y, FT min_x, FT min_y) {
          return trim2Box<FT, Vec2>(l, max_x, max_y, min_x, min_y);
        },
        py::arg("line"), py::arg("max_x"), py::arg("max_y"), py::arg("min_x") = FT(0), py::arg("min_y") = FT(0),
        "Trim infinite line to bounding box, returning a LineSegment.");
}

// ============================================================================
// LineOptimizer — gradient-magnitude-based line refinement
// ============================================================================

template <class FT>
void bind_line_optimizer(py::module_& m, const std::string& suffix) {
  using LST = LineSegment<FT, Vec2>;
  using LSVec = std::vector<LST>;
  // Use FT as the mat_type so CV_32F lines use float mag images, etc.
  using Optimizer = LineOptimizer<FT>;
  using MeanOp = typename MeanHelper<double, Vec2>::func_type;
  // Default mean operator: linear interpolation along the segment.
  static const MeanOp kDefaultMeanOp = Mean<double, FT, LinearInterpolator<double, FT>>::process;

  m.def(("optimize_line_segment" + suffix).c_str(),
        [](const cv::Mat& mag, const LST& seg, double d_lower, double d_upper, double r_lower, double r_upper,
           double mean_param, double derivative_prec) -> py::tuple {
          FT d = 0;
          FT r = 0;
          double err = Optimizer::optimize(mag, seg, d, r, d_lower, d_upper, r_lower, r_upper, mean_param,
                                           derivative_prec, kDefaultMeanOp);
          return py::make_tuple(err, static_cast<double>(d), static_cast<double>(r));
        },
        py::arg("magnitude"), py::arg("segment"), py::arg("d_lower") = -1.0, py::arg("d_upper") = 1.0,
        py::arg("r_lower") = -CV_PI / 180, py::arg("r_upper") = CV_PI / 180, py::arg("mean_param") = 1.0,
        py::arg("derivative_prec") = 1e-7,
        "Optimize line segment vs gradient magnitude.\n\n"
        "Finds optimal orthogonal translation (d) and rotation (r) that\n"
        "maximize gradient magnitude response along the segment.\n"
        "Returns (error, d, r). The segment is NOT modified.");

  m.def(("optimize_line_segment_inplace" + suffix).c_str(),
        [](const cv::Mat& mag, LST& seg, double d_lower, double d_upper, double r_lower, double r_upper,
           double mean_param, double derivative_prec) -> double {
          FT d = 0;
          FT r = 0;
          double err = Optimizer::optimize(mag, seg, d, r, d_lower, d_upper, r_lower, r_upper, mean_param,
                                           derivative_prec, kDefaultMeanOp);
          seg.translateOrtho(d);
          seg.rotate(r, seg.center());
          return err;
        },
        py::arg("magnitude"), py::arg("segment"), py::arg("d_lower") = -1.0, py::arg("d_upper") = 1.0,
        py::arg("r_lower") = -CV_PI / 180, py::arg("r_upper") = CV_PI / 180, py::arg("mean_param") = 1.0,
        py::arg("derivative_prec") = 1e-7,
        "Optimize line segment in-place vs gradient magnitude.\n\n"
        "Modifies the segment by applying the optimal translation and rotation.\n"
        "Returns the optimization error.");

  m.def(("optimize_line_segments" + suffix).c_str(),
        [](const cv::Mat& mag, const LSVec& segments, double d_lower, double d_upper, double r_lower, double r_upper,
           double mean_param, double derivative_prec) -> py::tuple {
          LSVec out(segments);
          std::vector<double> errors(out.size());
          for (size_t i = 0; i < out.size(); ++i) {
            FT d = 0;
            FT r = 0;
            errors[i] = Optimizer::optimize(mag, out[i], d, r, d_lower, d_upper, r_lower, r_upper, mean_param,
                                            derivative_prec, kDefaultMeanOp);
            out[i].translateOrtho(d);
            out[i].rotate(r, out[i].center());
          }
          return py::make_tuple(out, errors);
        },
        py::arg("magnitude"), py::arg("segments"), py::arg("d_lower") = -1.0, py::arg("d_upper") = 1.0,
        py::arg("r_lower") = -CV_PI / 180, py::arg("r_upper") = CV_PI / 180, py::arg("mean_param") = 1.0,
        py::arg("derivative_prec") = 1e-7,
        "Optimize a batch of line segments vs gradient magnitude.\n\n"
        "Returns (optimized_segments, errors). Input is not modified.");
}

// ============================================================================
// Visualization helpers (non-templated)
// ============================================================================

void bind_visualization(py::module_& m) {
  m.def(
      "create_nms_color", [](const cv::Mat& emap) { return createNMS(emap); }, py::arg("edge_map"),
      "Color-code an NMS edge map (8 direction classes to colors).");

  m.def(
      "apply_border",
      [](const cv::Mat& img, int border, int border_type) { return applyBorderCopy(img, border, border_type); },
      py::arg("image"), py::arg("border"), py::arg("border_type") = static_cast<int>(cv::BORDER_CONSTANT),
      "Apply border padding to image, returning a copy.");

  m.def(
      "save_normalized",
      [](const cv::Mat& data, const std::string& name, int border) { saveNormalized(data, name, border); },
      py::arg("data"), py::arg("filename"), py::arg("border") = 0, "Save matrix as normalized grayscale PNG.");

  m.def(
      "save_edge", [](const cv::Mat& data, const std::string& name, int border) { saveEdge(data, name, border); },
      py::arg("data"), py::arg("filename"), py::arg("border") = 0, "Save edge map as colored PNG.");

  m.def("random_color", &randomColor, py::arg("lowest") = 20.0, py::arg("highest") = 225.0,
        "Generate a random BGR color as (b, g, r, a) tuple.");
}

// ============================================================================
// Preset convenience
// ============================================================================

template <class FT>
void bind_geometry_preset(py::module_& m, const std::string& suffix) {
  bind_line<FT>(m, suffix);
  bind_line_segment<FT>(m, suffix);
  bind_polygon<FT>(m, suffix);
  bind_draw<FT>(m, suffix);
  bind_line_optimizer<FT>(m, suffix);
}

// --- Explicit instantiations ---
template void bind_line<float>(py::module_&, const std::string&);
template void bind_line<double>(py::module_&, const std::string&);

template void bind_line_segment<float>(py::module_&, const std::string&);
template void bind_line_segment<double>(py::module_&, const std::string&);

template void bind_polygon<float>(py::module_&, const std::string&);
template void bind_polygon<double>(py::module_&, const std::string&);

template void bind_draw<float>(py::module_&, const std::string&);
template void bind_draw<double>(py::module_&, const std::string&);

template void bind_line_optimizer<float>(py::module_&, const std::string&);
template void bind_line_optimizer<double>(py::module_&, const std::string&);

template void bind_geometry_preset<float>(py::module_&, const std::string&);
template void bind_geometry_preset<double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
