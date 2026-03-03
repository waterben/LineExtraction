/// @file geometry3d_binding.cpp
/// @brief Templated pybind11 bindings for 3D geometry types and camera models.
///
/// Implements the templated binding functions for 3D geometry:
///   - Line3, LineSegment3 — 3D line and segment
///   - Plane — 3D plane in Hesse normal form
///   - Pose — 6-DOF rigid body pose
///   - Camera, CameraHom, CameraPluecker, Camera2P, CameraCV — camera models
///
///   | Suffix  | FT     |
///   |---------|--------|
///   | (none)  | float  |
///   | _f64    | double |

#include "geometry_binding.hpp"
#include <cvnp/cvnp.h>
#include <geometry/camera.hpp>
#include <geometry/cameracv.hpp>
#include <geometry/line3.hpp>
#include <geometry/plane.hpp>
#include <geometry/pose.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>
#include <string>
#include <vector>

namespace py = pybind11;

namespace lsfm {
namespace python {

// ============================================================================
// Line3<FT> binding — 3D infinite line
// ============================================================================

template <class FT>
void bind_line3(py::module_& m, const std::string& suffix) {
  using L3 = Line3<FT>;
  const std::string cls = "Line3" + suffix;

  py::class_<L3>(m, cls.c_str(),
                 ("3D infinite line in point-direction form" + suffix +
                  ".\n\n"
                  "Represents a 3D line via origin point p and unit direction\n"
                  "vector v: P(t) = p + t*v. Supports Plücker and Cayley\n"
                  "representations for optimization and projection.")
                     .c_str())
      // --- Constructors ---
      .def(py::init<>(), "Construct an empty (degenerate) line.")
      .def(py::init([](FT px, FT py_, FT pz, FT vx, FT vy, FT vz) {
             return L3(Vec3<FT>(px, py_, pz), Vec3<FT>(vx, vy, vz));
           }),
           py::arg("px"), py::arg("py"), py::arg("pz"), py::arg("vx"), py::arg("vy"), py::arg("vz"),
           "Construct from point (px,py,pz) and direction (vx,vy,vz). Direction is normalized.")
      .def_static(
          "two_point",
          [](FT bx, FT by, FT bz, FT ex, FT ey, FT ez) {
            return L3::twoPoint(Vec3<FT>(bx, by, bz), Vec3<FT>(ex, ey, ez));
          },
          py::arg("bx"), py::arg("by"), py::arg("bz"), py::arg("ex"), py::arg("ey"), py::arg("ez"),
          "Create line through two points.")

      // --- Attribute accessors ---
      .def_property_readonly(
          "origin", [](const L3& l) { return py::make_tuple(l.origin()[0], l.origin()[1], l.origin()[2]); },
          "Origin point as (x, y, z) tuple.")
      .def_property_readonly(
          "direction", [](const L3& l) { return py::make_tuple(l.direction()[0], l.direction()[1], l.direction()[2]); },
          "Unit direction vector as (x, y, z) tuple.")
      .def_property_readonly(
          "momentum",
          [](const L3& l) {
            auto mom = l.momentum();
            return py::make_tuple(mom[0], mom[1], mom[2]);
          },
          "Plücker momentum vector m = p × v as (x, y, z) tuple.")
      .def(
          "valid", [](const L3& l) { return l.valid(); }, "True if direction is unit length.")
      .def(
          "empty", [](const L3& l) { return l.empty(); }, "True if direction is zero (degenerate).")

      // --- Distance and projection ---
      .def(
          "distance_to_point", [](const L3& l, FT x, FT y, FT z) { return l.distance(Vec3<FT>(x, y, z)); },
          py::arg("x"), py::arg("y"), py::arg("z"), "Shortest Euclidean distance from point to line.")
      .def(
          "distance_to_line", [](const L3& l, const L3& other) { return l.distance(other); }, py::arg("other"),
          "Shortest Euclidean distance between two lines.")
      .def(
          "normal_distance", [](const L3& l, FT x, FT y, FT z) { return l.normalDistance(Vec3<FT>(x, y, z)); },
          py::arg("x"), py::arg("y"), py::arg("z"),
          "Signed distance along direction from origin plane to projected point.")
      .def(
          "nearest_point",
          [](const L3& l, FT x, FT y, FT z) {
            auto np = l.nearestPointOnLine(Vec3<FT>(x, y, z));
            return py::make_tuple(np[0], np[1], np[2]);
          },
          py::arg("x"), py::arg("y"), py::arg("z"), "Nearest point on line to given point, as (x,y,z) tuple.")
      .def(
          "distance_origin",
          [](const L3& l, FT d) {
            auto p = l.distanceOrigin(d);
            return py::make_tuple(p[0], p[1], p[2]);
          },
          py::arg("d"), "Point at distance d from origin along direction, as (x,y,z) tuple.")

      // --- Geometric properties ---
      .def(
          "angle_to", [](const L3& l, const L3& other) { return l.angle(other); }, py::arg("other"),
          "Angle between two lines in radians [0, π].")
      .def("flip", &L3::flip, "Flip the line direction in-place.")

      // --- Transformations ---
      .def(
          "translate", [](L3& l, FT tx, FT ty, FT tz) { l.translate(Vec3<FT>(tx, ty, tz)); }, py::arg("tx"),
          py::arg("ty"), py::arg("tz"), "Translate line by vector in-place.")
      .def(
          "rotate", [](L3& l, FT rx, FT ry, FT rz) { l.rotate(Vec3<FT>(rx, ry, rz)); }, py::arg("rx"), py::arg("ry"),
          py::arg("rz"), "Rotate line around origin (Rodrigues vector) in-place.")

      // --- Cayley representation ---
      .def(
          "cayley",
          [](const L3& l) {
            auto c = l.cayley();
            return py::make_tuple(c[0], c[1], c[2], c[3]);
          },
          "Get Cayley representation as (w, s0, s1, s2) tuple.")
      .def_static(
          "from_cayley", [](FT w, FT s0, FT s1, FT s2) { return L3::lineFromCayley(w, Vec3<FT>(s0, s1, s2)); },
          py::arg("w"), py::arg("s0"), py::arg("s1"), py::arg("s2"), "Create line from Cayley representation.")

      .def("__repr__", [](const L3& l) {
        std::ostringstream os;
        os << "Line3(origin=(" << l.origin()[0] << ", " << l.origin()[1] << ", " << l.origin()[2] << "), dir=("
           << l.direction()[0] << ", " << l.direction()[1] << ", " << l.direction()[2] << "))";
        return os.str();
      });
}

// ============================================================================
// LineSegment3<FT> binding — 3D line segment
// ============================================================================

template <class FT>
void bind_line_segment3(py::module_& m, const std::string& suffix) {
  using L3 = Line3<FT>;
  using LS3 = LineSegment3<FT>;
  const std::string cls = "LineSegment3" + suffix;

  py::class_<LS3, L3>(m, cls.c_str(),
                      ("3D line segment" + suffix +
                       ".\n\n"
                       "Extends Line3 with start/end distances along the line\n"
                       "direction, defining a finite segment in 3D space.")
                          .c_str())
      // --- Constructors ---
      .def(py::init<>(), "Construct an empty (degenerate) segment.")
      .def_static(
          "from_endpoints",
          [](FT x1, FT y1, FT z1, FT x2, FT y2, FT z2) { return LS3(Vec3<FT>(x1, y1, z1), Vec3<FT>(x2, y2, z2)); },
          py::arg("x1"), py::arg("y1"), py::arg("z1"), py::arg("x2"), py::arg("y2"), py::arg("z2"),
          "Construct from two endpoints.")

      // --- Endpoint accessors ---
      .def(
          "start_point",
          [](const LS3& ls) {
            auto p = ls.startPoint();
            return py::make_tuple(p[0], p[1], p[2]);
          },
          "Start point as (x, y, z) tuple.")
      .def(
          "end_point",
          [](const LS3& ls) {
            auto p = ls.endPoint();
            return py::make_tuple(p[0], p[1], p[2]);
          },
          "End point as (x, y, z) tuple.")
      .def(
          "center_point",
          [](const LS3& ls) {
            auto p = ls.centerPoint();
            return py::make_tuple(p[0], p[1], p[2]);
          },
          "Center point as (x, y, z) tuple.")
      .def_property_readonly(
          "length", [](const LS3& ls) { return ls.length(); }, "Length of the segment.")

      // --- Distance ---
      .def(
          "nearest_point_on_segment",
          [](const LS3& ls, const L3& other) {
            auto np = ls.nearestPointOnLineSegment(other);
            return py::make_tuple(np[0], np[1], np[2]);
          },
          py::arg("line"), "Nearest point on this segment to another line (clamped to endpoints).")

      // --- Error ---
      .def(
          "error", [](const LS3& ls, const L3& gt) { return ls.error(gt); }, py::arg("ground_truth_line"),
          "Sum of squared endpoint distances to a ground truth line.")

      // --- Transforms ---
      .def("flip", &LS3::flip, "Flip the segment direction in-place.")
      .def("endpoint_swap", &LS3::endPointSwap, "Swap start and end points in-place.")

      .def("__repr__", [](const LS3& ls) {
        auto sp = ls.startPoint();
        auto ep = ls.endPoint();
        std::ostringstream os;
        os << "LineSegment3((" << sp[0] << ", " << sp[1] << ", " << sp[2] << ") -> (" << ep[0] << ", " << ep[1] << ", "
           << ep[2] << "), len=" << ls.length() << ")";
        return os.str();
      });
}

// ============================================================================
// Plane<FT> binding — 3D plane in Hesse normal form
// ============================================================================

template <class FT>
void bind_plane(py::module_& m, const std::string& suffix) {
  using PlaneT = Plane<FT>;
  using L3 = Line3<FT>;
  const std::string cls = "Plane" + suffix;

  py::class_<PlaneT>(m, cls.c_str(),
                     ("3D plane in Hesse normal form (n·x = d)" + suffix +
                      ".\n\n"
                      "Represents a plane via unit normal n and signed distance d\n"
                      "from the origin. Supports intersection tests with lines and planes.")
                         .c_str())
      // --- Constructors ---
      .def(py::init<>(), "Construct an invalid (degenerate) plane.")
      .def(py::init([](FT px, FT py_, FT pz, FT nx, FT ny, FT nz) {
             return PlaneT(Vec3<FT>(px, py_, pz), Vec3<FT>(nx, ny, nz));
           }),
           py::arg("px"), py::arg("py"), py::arg("pz"), py::arg("nx"), py::arg("ny"), py::arg("nz"),
           "Construct from a point on the plane and a normal direction (normalized automatically).")
      .def_static(
          "from_three_points",
          [](FT x1, FT y1, FT z1, FT x2, FT y2, FT z2, FT x3, FT y3, FT z3) {
            return PlaneT(Vec3<FT>(x1, y1, z1), Vec3<FT>(x2, y2, z2), Vec3<FT>(x3, y3, z3));
          },
          py::arg("x1"), py::arg("y1"), py::arg("z1"), py::arg("x2"), py::arg("y2"), py::arg("z2"), py::arg("x3"),
          py::arg("y3"), py::arg("z3"), "Construct plane through three non-collinear points.")

      // --- Attribute accessors ---
      .def_property_readonly(
          "normal", [](const PlaneT& p) { return py::make_tuple(p.normal()[0], p.normal()[1], p.normal()[2]); },
          "Unit normal vector as (x, y, z) tuple.")
      .def_property_readonly(
          "dist_to_origin", [](const PlaneT& p) { return p.dist2origin(); },
          "Signed distance from origin to plane along the normal.")
      .def(
          "valid", [](const PlaneT& p) { return p.valid(); }, "True if normal is unit length.")
      .def(
          "empty", [](const PlaneT& p) { return p.empty(); }, "True if normal is zero.")

      // --- Distance and projection ---
      .def(
          "distance", [](const PlaneT& pl, FT x, FT y, FT z) { return pl.distance(Vec3<FT>(x, y, z)); }, py::arg("x"),
          py::arg("y"), py::arg("z"), "Signed distance from point to plane.")
      .def(
          "nearest_point",
          [](const PlaneT& pl, FT x, FT y, FT z) {
            auto np = pl.nearestPointOnPlane(Vec3<FT>(x, y, z));
            return py::make_tuple(np[0], np[1], np[2]);
          },
          py::arg("x"), py::arg("y"), py::arg("z"), "Project point onto plane; returns nearest point as (x,y,z) tuple.")

      // --- Intersection ---
      .def(
          "intersects_line", [](const PlaneT& pl, const L3& l) { return pl.intersection(l); }, py::arg("line"),
          "Check if a line intersects this plane (not parallel).")
      .def(
          "intersection_with_line",
          [](const PlaneT& pl, const L3& l) -> py::object {
            Vec3<FT> pt;
            if (pl.intersection(l, pt)) {
              return py::make_tuple(pt[0], pt[1], pt[2]);
            }
            return py::none();
          },
          py::arg("line"), "Compute line-plane intersection point. Returns (x,y,z) or None.")
      .def(
          "intersection_with_plane",
          [](const PlaneT& pl, const PlaneT& other) -> py::object {
            L3 line;
            if (pl.intersection(other, line)) {
              return py::cast(line);
            }
            return py::none();
          },
          py::arg("other"), "Compute plane-plane intersection line. Returns Line3 or None.")

      // --- Transforms ---
      .def(
          "translate", [](PlaneT& p, FT tx, FT ty, FT tz) { p.translate(Vec3<FT>(tx, ty, tz)); }, py::arg("tx"),
          py::arg("ty"), py::arg("tz"), "Translate plane by vector in-place.")
      .def(
          "translate_ortho", [](PlaneT& p, FT d) { p.translate(d); }, py::arg("distance"),
          "Translate plane along its normal in-place.")
      .def("flip", &PlaneT::flip, "Flip normal direction in-place.")

      // --- Geometric properties ---
      .def(
          "angle_to_plane", [](const PlaneT& p, PlaneT& other) { return p.angle(other); }, py::arg("other"),
          "Angle between two planes in radians [0, π].")

      .def("__repr__", [](const PlaneT& p) {
        std::ostringstream os;
        os << "Plane(n=(" << p.normal()[0] << ", " << p.normal()[1] << ", " << p.normal()[2]
           << "), d=" << p.dist2origin() << ")";
        return os.str();
      });
}

// ============================================================================
// Pose<FT> binding — 6-DOF rigid body pose
// ============================================================================

template <class FT>
void bind_pose(py::module_& m, const std::string& suffix) {
  using PoseT = Pose<FT>;
  const std::string cls = "Pose" + suffix;

  py::class_<PoseT>(m, cls.c_str(),
                    ("6-DOF rigid body pose" + suffix +
                     ".\n\n"
                     "Represents position and orientation in 3D space using\n"
                     "a translation vector and a Rodrigues rotation vector.")
                        .c_str())
      // --- Constructors ---
      .def(py::init<FT, FT, FT, FT, FT, FT>(), py::arg("tx") = FT(0), py::arg("ty") = FT(0), py::arg("tz") = FT(0),
           py::arg("rx") = FT(0), py::arg("ry") = FT(0), py::arg("rz") = FT(0),
           "Construct from translation and Rodrigues rotation components.")

      // --- Accessors ---
      .def_property(
          "origin", [](const PoseT& p) { return py::make_tuple(p.origin()[0], p.origin()[1], p.origin()[2]); },
          [](PoseT& p, const py::tuple& t) { p.origin(Vec3<FT>(t[0].cast<FT>(), t[1].cast<FT>(), t[2].cast<FT>())); },
          "Translation (position) as (x, y, z) tuple.")
      .def_property(
          "orientation",
          [](const PoseT& p) { return py::make_tuple(p.orientation()[0], p.orientation()[1], p.orientation()[2]); },
          [](PoseT& p, const py::tuple& t) {
            p.orientation(Vec3<FT>(t[0].cast<FT>(), t[1].cast<FT>(), t[2].cast<FT>()));
          },
          "Rodrigues rotation vector as (rx, ry, rz) tuple.")

      // --- Matrix representations ---
      .def(
          "rot_matrix",
          [](const PoseT& p) {
            auto rm = p.rotM();
            // Return as 3x3 nested tuple
            py::list rows;
            for (int r = 0; r < 3; ++r) {
              rows.append(py::make_tuple(rm(r, 0), rm(r, 1), rm(r, 2)));
            }
            return rows;
          },
          "Get 3x3 rotation matrix as list of row tuples.")
      .def(
          "hom_matrix",
          [](const PoseT& p) {
            auto hm = p.homM();
            py::list rows;
            for (int r = 0; r < 4; ++r) {
              rows.append(py::make_tuple(hm(r, 0), hm(r, 1), hm(r, 2), hm(r, 3)));
            }
            return rows;
          },
          "Get 4x4 homogeneous transformation matrix as list of row tuples.")

      // --- Transforms ---
      .def(
          "translate", [](PoseT& p, FT tx, FT ty, FT tz) { p.translate(Vec3<FT>(tx, ty, tz)); }, py::arg("tx"),
          py::arg("ty"), py::arg("tz"), "Translate pose by vector in-place.")
      .def(
          "rotate", [](PoseT& p, FT rx, FT ry, FT rz) { p.rotate(Vec3<FT>(rx, ry, rz)); }, py::arg("rx"), py::arg("ry"),
          py::arg("rz"), "Rotate around pose origin (Rodrigues vector) in-place.")
      .def(
          "rotate_around",
          [](PoseT& p, FT rx, FT ry, FT rz, FT px, FT py_, FT pz) {
            p.rotate(Vec3<FT>(rx, ry, rz), Vec3<FT>(px, py_, pz));
          },
          py::arg("rx"), py::arg("ry"), py::arg("rz"), py::arg("px"), py::arg("py"), py::arg("pz"),
          "Rotate around given pivot (Rodrigues vector) in-place.")
      .def(
          "concat", [](PoseT& p, const PoseT& other) { p.concat(other); }, py::arg("other"),
          "Concatenate another pose (chain transformations) in-place.")

      .def("__repr__", [](const PoseT& p) {
        std::ostringstream os;
        os << "Pose(t=(" << p.origin()[0] << ", " << p.origin()[1] << ", " << p.origin()[2] << "), r=("
           << p.orientation()[0] << ", " << p.orientation()[1] << ", " << p.orientation()[2] << "))";
        return os.str();
      });
}

// ============================================================================
// Camera<FT> binding — pinhole camera model
// ============================================================================

template <class FT>
void bind_camera(py::module_& m, const std::string& suffix) {
  using CamT = Camera<FT>;
  using PoseT = Pose<FT>;
  const std::string cls = "Camera" + suffix;

  py::class_<CamT, PoseT>(m, cls.c_str(),
                          ("Pinhole camera model" + suffix +
                           ".\n\n"
                           "Models a camera using intrinsic parameters (focal length,\n"
                           "principal point) and extrinsic parameters (pose in world).\n"
                           "Inherits from Pose for position/orientation.")
                              .c_str())
      .def(py::init<FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT>(), py::arg("focal_x") = FT(0),
           py::arg("focal_y") = FT(0), py::arg("offset_x") = FT(0), py::arg("offset_y") = FT(0),
           py::arg("width") = FT(0), py::arg("height") = FT(0), py::arg("tx") = FT(0), py::arg("ty") = FT(0),
           py::arg("tz") = FT(0), py::arg("rx") = FT(0), py::arg("ry") = FT(0), py::arg("rz") = FT(0),
           "Construct from individual intrinsic and extrinsic parameters.")

      // --- Accessors ---
      .def_property_readonly(
          "empty", [](const CamT& c) { return c.empty(); }, "True if both focal lengths are zero.")
      .def_property_readonly(
          "focal", [](const CamT& c) { return py::make_tuple(c.focal().x(), c.focal().y()); },
          "Focal length as (fx, fy) tuple.")
      .def_property_readonly(
          "offset", [](const CamT& c) { return py::make_tuple(c.offset().x(), c.offset().y()); },
          "Principal point as (cx, cy) tuple.")
      .def_property_readonly(
          "focal_length", [](const CamT& c) { return c.focalLength(); }, "Composed focal length (sqrt(fx*fy)).")
      .def_property_readonly(
          "fov",
          [](const CamT& c) {
            auto f = c.fov();
            return py::make_tuple(f.x(), f.y());
          },
          "Field of view as (fov_x, fov_y) in radians.")
      .def_property_readonly(
          "width", [](const CamT& c) { return c.width(); }, "Image width.")
      .def_property_readonly(
          "height", [](const CamT& c) { return c.height(); }, "Image height.")
      .def_property_readonly(
          "image_size", [](const CamT& c) { return py::make_tuple(c.imageSize().x(), c.imageSize().y()); },
          "Image dimensions as (width, height) tuple.")
      .def_property_readonly(
          "aspect_ratio", [](const CamT& c) { return c.aspectRatio(); }, "Image aspect ratio (w/h).")
      .def(
          "set_image_size", [](CamT& c, int w, int h) { c.imageSize(w, h); }, py::arg("width"), py::arg("height"),
          "Set image dimensions.")

      // --- Static helpers ---
      .def_static(
          "fov_to_focal", [](FT fov, FT size) { return CamT::fov2Focal(fov, size); }, py::arg("fov"), py::arg("size"),
          "Convert field of view (radians) to focal length.")
      .def_static(
          "focal_to_fov", [](FT focal, FT offset) { return CamT::focal2Fov(focal, offset); }, py::arg("focal"),
          py::arg("offset"), "Convert focal length to field of view (radians).")

      .def("__repr__", [](const CamT& c) {
        std::ostringstream os;
        os << "Camera(focal=(" << c.focal().x() << ", " << c.focal().y() << "), offset=(" << c.offset().x() << ", "
           << c.offset().y() << "), size=(" << c.width() << "x" << c.height() << "))";
        return os.str();
      });
}

// ============================================================================
// CameraHom<FT> binding — cached projection matrix
// ============================================================================

template <class FT>
void bind_camera_hom(py::module_& m, const std::string& suffix) {
  using CamH = CameraHom<FT>;
  using CamT = Camera<FT>;
  const std::string cls = "CameraHom" + suffix;

  py::class_<CamH, CamT>(m, cls.c_str(),
                         ("Camera with cached projection matrix" + suffix +
                          ".\n\n"
                          "Extends Camera with pre-computed projection matrix\n"
                          "for efficient repeated point projection.")
                             .c_str())
      .def(py::init<FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT>(), py::arg("focal_x") = FT(0),
           py::arg("focal_y") = FT(0), py::arg("offset_x") = FT(0), py::arg("offset_y") = FT(0),
           py::arg("width") = FT(0), py::arg("height") = FT(0), py::arg("tx") = FT(0), py::arg("ty") = FT(0),
           py::arg("tz") = FT(0), py::arg("rx") = FT(0), py::arg("ry") = FT(0), py::arg("rz") = FT(0),
           "Construct from individual intrinsic and extrinsic parameters.")
      .def(py::init<const CamT&>(), py::arg("camera"), "Construct from base Camera.")

      // --- Point projection ---
      .def(
          "project_point",
          [](const CamH& c, FT x, FT y, FT z) {
            auto r = c.project(Vec3<FT>(x, y, z));
            return py::make_tuple(r.x(), r.y());
          },
          py::arg("x"), py::arg("y"), py::arg("z"), "Project 3D point to 2D image coordinates.")
      .def(
          "project_points",
          [](const CamH& c, const std::vector<std::tuple<FT, FT, FT>>& pts) {
            std::vector<Vec3<FT>> in;
            in.reserve(pts.size());
            for (const auto& t : pts) {
              in.emplace_back(std::get<0>(t), std::get<1>(t), std::get<2>(t));
            }
            std::vector<Vec2<FT>> out;
            c.project(in, out);
            py::list result;
            for (const auto& p : out) {
              result.append(py::make_tuple(p.x(), p.y()));
            }
            return result;
          },
          py::arg("points"), "Project list of (x,y,z) tuples to list of (u,v) tuples.");
}

// ============================================================================
// CameraPluecker<FT> binding — Plücker line projection
// ============================================================================

template <class FT>
void bind_camera_pluecker(py::module_& m, const std::string& suffix) {
  using CamP = CameraPluecker<FT>;
  using CamT = Camera<FT>;
  using CamH = CameraHom<FT>;
  using L3 = Line3<FT>;
  using LS3 = LineSegment3<FT>;
  using LineT = Line<FT, Vec2>;
  using LST = LineSegment<FT, Vec2>;
  const std::string cls = "CameraPluecker" + suffix;

  py::class_<CamP, CamH>(m, cls.c_str(),
                         ("Camera with Plücker line projection" + suffix +
                          ".\n\n"
                          "Extends CameraHom with Plücker coordinate-based\n"
                          "line projection for accurate 3D → 2D line transforms.")
                             .c_str())
      .def(py::init<FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT>(), py::arg("focal_x") = FT(0),
           py::arg("focal_y") = FT(0), py::arg("offset_x") = FT(0), py::arg("offset_y") = FT(0),
           py::arg("width") = FT(0), py::arg("height") = FT(0), py::arg("tx") = FT(0), py::arg("ty") = FT(0),
           py::arg("tz") = FT(0), py::arg("rx") = FT(0), py::arg("ry") = FT(0), py::arg("rz") = FT(0),
           "Construct from individual intrinsic and extrinsic parameters.")
      .def(py::init<const CamT&>(), py::arg("camera"), "Construct from base Camera.")

      // --- Line projection ---
      .def(
          "project_line", [](const CamP& c, const L3& line) { return c.project(line); }, py::arg("line3"),
          "Project 3D line to 2D using Plücker coordinates.")
      .def(
          "project_line_segment", [](const CamP& c, const LS3& ls) { return c.project(ls); }, py::arg("segment3"),
          "Project 3D line segment to 2D.")
      .def(
          "project_lines",
          [](const CamP& c, const std::vector<L3>& lines3) {
            std::vector<LineT> ret;
            c.project(lines3, ret);
            return ret;
          },
          py::arg("lines3"), "Project list of 3D lines to 2D lines.")
      .def(
          "project_line_segments",
          [](const CamP& c, const std::vector<LS3>& segs3) {
            std::vector<LST> ret;
            c.project(segs3, ret);
            return ret;
          },
          py::arg("segments3"), "Project list of 3D line segments to 2D.");
}

// ============================================================================
// Camera2P<FT> binding — two-point line projection
// ============================================================================

template <class FT>
void bind_camera_2p(py::module_& m, const std::string& suffix) {
  using Cam2P = Camera2P<FT>;
  using CamT = Camera<FT>;
  using CamH = CameraHom<FT>;
  using L3 = Line3<FT>;
  using LS3 = LineSegment3<FT>;
  using LineT = Line<FT, Vec2>;
  using LST = LineSegment<FT, Vec2>;
  const std::string cls = "Camera2P" + suffix;

  py::class_<Cam2P, CamH>(m, cls.c_str(),
                          ("Camera with two-point line projection" + suffix +
                           ".\n\n"
                           "Projects 3D lines by projecting two points and constructing\n"
                           "a 2D line through them. Simpler alternative to Plücker.")
                              .c_str())
      .def(py::init<FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT>(), py::arg("focal_x") = FT(0),
           py::arg("focal_y") = FT(0), py::arg("offset_x") = FT(0), py::arg("offset_y") = FT(0),
           py::arg("width") = FT(0), py::arg("height") = FT(0), py::arg("tx") = FT(0), py::arg("ty") = FT(0),
           py::arg("tz") = FT(0), py::arg("rx") = FT(0), py::arg("ry") = FT(0), py::arg("rz") = FT(0),
           "Construct from individual intrinsic and extrinsic parameters.")
      .def(py::init<const CamT&>(), py::arg("camera"), "Construct from base Camera.")

      // --- Line projection ---
      .def(
          "project_line", [](const Cam2P& c, const L3& line) { return c.project(line); }, py::arg("line3"),
          "Project 3D line to 2D via two-point projection.")
      .def(
          "project_line_segment", [](const Cam2P& c, const LS3& ls) { return c.project(ls); }, py::arg("segment3"),
          "Project 3D line segment to 2D via endpoint projection.")
      .def(
          "project_lines",
          [](const Cam2P& c, const std::vector<L3>& lines3) {
            std::vector<LineT> ret;
            c.project(lines3, ret);
            return ret;
          },
          py::arg("lines3"), "Project list of 3D lines to 2D.")
      .def(
          "project_line_segments",
          [](const Cam2P& c, const std::vector<LS3>& segs3) {
            std::vector<LST> ret;
            c.project(segs3, ret);
            return ret;
          },
          py::arg("segments3"), "Project list of 3D line segments to 2D.");
}

// ============================================================================
// CameraCV<FT> binding — OpenCV projection
// ============================================================================

template <class FT>
void bind_camera_cv(py::module_& m, const std::string& suffix) {
  using CamCV = CameraCV<FT>;
  using CamT = Camera<FT>;
  using CamH = CameraHom<FT>;
  using L3 = Line3<FT>;
  using LS3 = LineSegment3<FT>;
  using LineT = Line<FT, Vec2>;
  using LST = LineSegment<FT, Vec2>;
  const std::string cls = "CameraCV" + suffix;

  py::class_<CamCV, CamH>(m, cls.c_str(),
                          ("Camera using OpenCV projection" + suffix +
                           ".\n\n"
                           "Uses cv::projectPoints() for projection, providing\n"
                           "compatibility with OpenCV calibration workflows.")
                              .c_str())
      .def(py::init<FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT, FT>(), py::arg("focal_x") = FT(0),
           py::arg("focal_y") = FT(0), py::arg("offset_x") = FT(0), py::arg("offset_y") = FT(0),
           py::arg("width") = FT(0), py::arg("height") = FT(0), py::arg("tx") = FT(0), py::arg("ty") = FT(0),
           py::arg("tz") = FT(0), py::arg("rx") = FT(0), py::arg("ry") = FT(0), py::arg("rz") = FT(0),
           "Construct from individual intrinsic and extrinsic parameters.")
      .def(py::init<const CamT&>(), py::arg("camera"), "Construct from base Camera.")

      // --- Point projection ---
      .def(
          "project_point",
          [](const CamCV& c, FT x, FT y, FT z) {
            auto r = c.project(Vec3<FT>(x, y, z));
            return py::make_tuple(r.x(), r.y());
          },
          py::arg("x"), py::arg("y"), py::arg("z"), "Project 3D point to 2D using OpenCV.")

      // --- Line projection ---
      .def(
          "project_line", [](const CamCV& c, const L3& line) { return c.project(line); }, py::arg("line3"),
          "Project 3D line to 2D via OpenCV two-point projection.")
      .def(
          "project_line_segment", [](const CamCV& c, const LS3& ls) { return c.project(ls); }, py::arg("segment3"),
          "Project 3D line segment to 2D via OpenCV endpoint projection.")
      .def(
          "project_lines",
          [](const CamCV& c, const std::vector<L3>& lines3) {
            std::vector<LineT> ret;
            c.project(lines3, ret);
            return ret;
          },
          py::arg("lines3"), "Project list of 3D lines to 2D via OpenCV.")
      .def(
          "project_line_segments",
          [](const CamCV& c, const std::vector<LS3>& segs3) {
            std::vector<LST> ret;
            c.project(segs3, ret);
            return ret;
          },
          py::arg("segments3"), "Project list of 3D line segments to 2D via OpenCV.");
}

// ============================================================================
// Preset convenience — bind all 3D types for a given FT
// ============================================================================

template <class FT>
void bind_geometry3d_preset(py::module_& m, const std::string& suffix) {
  bind_line3<FT>(m, suffix);
  bind_line_segment3<FT>(m, suffix);
  bind_plane<FT>(m, suffix);
  bind_pose<FT>(m, suffix);
  bind_camera<FT>(m, suffix);
  bind_camera_hom<FT>(m, suffix);
  bind_camera_pluecker<FT>(m, suffix);
  bind_camera_2p<FT>(m, suffix);
  bind_camera_cv<FT>(m, suffix);
}

// --- Explicit template instantiations ---
template void bind_line3<float>(py::module_&, const std::string&);
template void bind_line3<double>(py::module_&, const std::string&);

template void bind_line_segment3<float>(py::module_&, const std::string&);
template void bind_line_segment3<double>(py::module_&, const std::string&);

template void bind_plane<float>(py::module_&, const std::string&);
template void bind_plane<double>(py::module_&, const std::string&);

template void bind_pose<float>(py::module_&, const std::string&);
template void bind_pose<double>(py::module_&, const std::string&);

template void bind_camera<float>(py::module_&, const std::string&);
template void bind_camera<double>(py::module_&, const std::string&);

template void bind_camera_hom<float>(py::module_&, const std::string&);
template void bind_camera_hom<double>(py::module_&, const std::string&);

template void bind_camera_pluecker<float>(py::module_&, const std::string&);
template void bind_camera_pluecker<double>(py::module_&, const std::string&);

template void bind_camera_2p<float>(py::module_&, const std::string&);
template void bind_camera_2p<double>(py::module_&, const std::string&);

template void bind_camera_cv<float>(py::module_&, const std::string&);
template void bind_camera_cv<double>(py::module_&, const std::string&);

template void bind_geometry3d_preset<float>(py::module_&, const std::string&);
template void bind_geometry3d_preset<double>(py::module_&, const std::string&);

}  // namespace python
}  // namespace lsfm
