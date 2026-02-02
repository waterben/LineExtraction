/*M///////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// C by Benjamin Wassermann
//M*/

/// @file stereo.hpp
/// @brief Stereo vision triangulation utilities.
/// Provides classes and functions for stereo triangulation of points
/// and lines from corresponding 2D observations in rectified stereo pairs.

#pragma once

#include <geometry/camera.hpp>
#include <geometry/plane.hpp>


namespace lsfm {

/// @name Coordinate Conversion Helpers
/// @{

/// @brief Convert pixel to 3D point on camera image plane.
/// @tparam FT Floating-point type.
/// @param focal Focal lengths (fx, fy).
/// @param offset Principal point (cx, cy).
/// @param p 2D pixel coordinates.
/// @return 3D point on camera plane at z = focal.x().
template <class FT>
inline Vec3<FT> pixel2Cam3dPoint(const Vec2<FT>& focal, const Vec2<FT>& offset, const Vec2<FT>& p) {
  return Vec3<FT>(p.x() - offset.x(), p.y() - offset.y(), focal.x());
}

/// @brief Convert pixel to 3D point on camera image plane (isotropic focal).
/// @tparam FT Floating-point type.
/// @param focal Isotropic focal length.
/// @param offset Principal point (cx, cy).
/// @param p 2D pixel coordinates.
/// @return 3D point on camera plane at z = focal.
template <class FT>
inline Vec3<FT> pixel2Cam3dPoint(FT focal, const Vec2<FT>& offset, const Vec2<FT>& p) {
  return Vec3<FT>(p.x() - offset.x(), p.y() - offset.y(), focal);
}

/// @brief Convert 2D line to 3D line on camera image plane.
/// @tparam FT Floating-point type.
/// @param focal Focal lengths (fx, fy).
/// @param offset Principal point (cx, cy).
/// @param l 2D line.
/// @return 3D line on camera plane (z-direction = 0).
template <class FT>
inline Line3<FT> line2Cam3dLine(const Vec2<FT>& focal, const Vec2<FT>& offset, const Line<FT>& l) {
  return Line3<FT>(pixel2Cam3dPoint(focal, offset, l.origin()), Vec3<FT>(l.directionX(), l.directionY(), 0), true);
}

/// @brief Convert 2D line to 3D line on camera image plane (isotropic focal).
/// @tparam FT Floating-point type.
/// @param focal Isotropic focal length.
/// @param offset Principal point (cx, cy).
/// @param l 2D line.
/// @return 3D line on camera plane (z-direction = 0).
template <class FT>
inline Line3<FT> line2Cam3dLine(FT focal, const Vec2<FT>& offset, const Line<FT>& l) {
  return Line3<FT>(pixel2Cam3dPoint(focal, offset, l.origin()), Vec3<FT>(l.directionX(), l.directionY(), 0), true);
}

/// @brief Convert image pixel to 3D point in world coordinates.
/// @tparam FT Floating-point type.
/// @param focal Focal lengths (fx, fy).
/// @param offset Principal point (cx, cy).
/// @param rot Rotation matrix (camera to world).
/// @param trans Translation vector (camera origin).
/// @param p 2D pixel coordinates.
/// @return 3D point in world coordinates.
template <class FT>
inline Vec3<FT> pointFromPixel(const Vec2<FT>& focal,
                               const Vec2<FT>& offset,
                               const Matx<FT, 3, 3>& rot,
                               const Vec3<FT>& trans,
                               const Vec2<FT>& p) {
  return trans + rot * pixel2Cam3dPoint(focal, offset, p);
}

/// @brief Convert image pixel to 3D point in world coordinates (isotropic focal).
/// @tparam FT Floating-point type.
/// @param focal Isotropic focal length.
/// @param offset Principal point (cx, cy).
/// @param rot Rotation matrix (camera to world).
/// @param trans Translation vector (camera origin).
/// @param p 2D pixel coordinates.
/// @return 3D point in world coordinates.
template <class FT>
inline Vec3<FT> pointFromPixel(
    FT focal, const Vec2<FT>& offset, const Matx<FT, 3, 3>& rot, const Vec3<FT>& trans, const Vec2<FT>& p) {
  return trans + rot * pixel2Cam3dPoint(focal, offset, p);
}

/// @brief Construct ray from camera origin through pixel.
/// @tparam FT Floating-point type.
/// @param focal Focal lengths (fx, fy).
/// @param offset Principal point (cx, cy).
/// @param rot Rotation matrix (camera to world).
/// @param trans Camera origin in world coordinates.
/// @param p 2D pixel coordinates.
/// @return Ray from camera origin through the pixel.
template <class FT>
inline Line3<FT> lineFromPixel(const Vec2<FT>& focal,
                               const Vec2<FT>& offset,
                               const Matx<FT, 3, 3>& rot,
                               const Vec3<FT>& trans,
                               const Vec2<FT>& p) {
  return Line3<FT>(trans, rot * pixel2Cam3dPoint(focal, offset, p));
}

/// @brief Construct ray from camera origin through pixel (isotropic focal).
/// @tparam FT Floating-point type.
/// @param focal Isotropic focal length.
/// @param offset Principal point (cx, cy).
/// @param rot Rotation matrix (camera to world).
/// @param trans Camera origin in world coordinates.
/// @param p 2D pixel coordinates.
/// @return Ray from camera origin through the pixel.
template <class FT>
inline Line3<FT> lineFromPixel(
    FT focal, const Vec2<FT>& offset, const Matx<FT, 3, 3>& rot, const Vec3<FT>& trans, const Vec2<FT>& p) {
  return Line3<FT>(trans, rot * pixel2Cam3dPoint(focal, offset, p));
}

/// @brief Construct plane from camera origin containing 2D line.
/// @tparam FT Floating-point type.
/// @param focal Focal lengths (fx, fy).
/// @param offset Principal point (cx, cy).
/// @param rot Rotation matrix (camera to world).
/// @param trans Camera origin in world coordinates.
/// @param l 2D line in image.
/// @return Plane through camera origin containing the line.
template <class FT>
inline Plane<FT> planeFromLine(const Vec2<FT>& focal,
                               const Vec2<FT>& offset,
                               const Matx<FT, 3, 3>& rot,
                               const Vec3<FT>& trans,
                               const Line<FT>& l) {
  return Plane<FT>(trans, line2Cam3dLine(focal, offset, l).rotate(rot).translate(trans));
}

/// @brief Construct plane from camera origin containing 2D line (isotropic focal).
/// @tparam FT Floating-point type.
/// @param focal Isotropic focal length.
/// @param offset Principal point (cx, cy).
/// @param rot Rotation matrix (camera to world).
/// @param trans Camera origin in world coordinates.
/// @param l 2D line in image.
/// @return Plane through camera origin containing the line.
template <class FT>
inline Plane<FT> planeFromLine(
    FT focal, const Vec2<FT>& offset, const Matx<FT, 3, 3>& rot, const Vec3<FT>& trans, const Line<FT>& l) {
  return Plane<FT>(trans, line2Cam3dLine(focal, offset, l).rotate(rot).translate(trans));
}

/// @}

/// @brief Stereo triangulation class using ray intersection.
/// Triangulates points and lines from corresponding observations
/// in a rectified stereo pair. Points are triangulated by finding
/// the midpoint of the closest approach between rays. Lines are
/// triangulated by triangulating two points on the line.
/// @tparam FT Floating-point type.
/// @note Requires rectified stereo setup for line triangulation.
template <class FT>
class Stereo {
 protected:
  Camera<FT> camL_, camR_;  ///< Left and right cameras.
  Matx33<FT> rotL_, rotR_;  ///< Cached rotation matrices.

 public:
  typedef FT float_type;

  /// @brief Construct from projection matrices.
  /// @param projL Left camera projection matrix (3x4).
  /// @param projR Right camera projection matrix (3x4).
  Stereo(const Matx34<FT>& projL, const Matx34<FT>& projR) : camL_(projL), camR_(projR) {
    rotL_ = camL_.rotM();
    rotR_ = camR_.rotM();
  }

  /// @brief Construct from Camera objects.
  /// @param camL Left camera.
  /// @param camR Right camera.
  Stereo(const Camera<FT>& camL, const Camera<FT>& camR)
      : camL_(camL), camR_(camR), rotL_(camL_.rotM()), rotR_(camR_.rotM()) {}

  /// @name Point Triangulation
  /// @{

  /// @brief Triangulate 3D point from stereo correspondences.
  /// Computes 3D point by finding the midpoint of closest approach
  /// between rays from left and right camera origins through the
  /// corresponding pixels.
  /// @param pointL Left image pixel.
  /// @param pointR Right image pixel.
  /// @return Triangulated 3D point.
  inline Vec3<FT> triangulate(const Vec2<FT>& pointL, const Vec2<FT>& pointR) const {
    Line3<FT> ray = lineFromPixel(camL_.focal(), camL_.offset(), rotL_, camL_.origin(), pointL);
#ifdef STEREO_FAST
    return ray.nearestPointOnLine(lineFromPixel(camR_.focal(), camR_.offset(), rotR_, camR_.origin(), pointR));
#else
    Vec3<FT> a, b;
    ray.nearestPointOnLine(lineFromPixel(camR_.focal(), camR_.offset(), rotR_, camR_.origin(), pointR), a, b);
    return (a + b) * static_cast<FT>(0.5);
#endif
  }

  /// @brief Triangulate vector of point correspondences.
  /// @tparam V Container type.
  /// @tparam V1Args Template args for input container.
  /// @tparam V2Args Template args for output container.
  /// @param pointL Left image pixels.
  /// @param pointR Right image pixels.
  /// @param[out] ret Triangulated 3D points.
  template <template <class, class...> class V, class... V1Args, class... V2Args>
  inline void triangulate(const V<Vec2<FT>, V1Args...>& pointL,
                          const V<Vec2<FT>, V1Args...>& pointR,
                          V<Vec3<FT>, V2Args...>& ret) const {
    CV_Assert(pointL.size() == pointR.size());
    size_t size = pointL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) {
      ret.push_back(triangulate(pointL[i], pointR[i]));
    }
  }

  /// @}

  /// @name Line Triangulation
  /// @{

  /// @brief Triangulate 3D line from stereo line correspondences.
  /// Triangulates by computing 3D points at two y-coordinates
  /// on the line and constructing a line through them.
  /// @param lineL Left image line.
  /// @param lineR Right image line.
  /// @return Triangulated 3D line (empty if degenerate).
  /// @note Requires rectified setup (epipolar lines horizontal).
  inline Line3<FT> triangulate(const Line<FT>& lineL, const Line<FT>& lineR) const {
    // prevent div by zero from normal x in .x()() call
    if (detail::abs(lineL.normalX()) < LIMITS<FT>::tau() || detail::abs(lineR.normalX()) < LIMITS<FT>::tau())
      return Line3<FT>();
    return Line3<FT>::twoPoint(triangulate(Vec2<FT>(lineL.x(0), 0), Vec2<FT>(lineR.x(0), 0)),
                               triangulate(Vec2<FT>(lineL.x(100), 100), Vec2<FT>(lineR.x(100), 100)));
  }

  /// @brief Triangulate vector of line correspondences.
  /// @tparam V Container type.
  /// @tparam V1Args Template args for input container.
  /// @tparam V2Args Template args for output container.
  /// @param lineL Left image lines.
  /// @param lineR Right image lines.
  /// @param[out] ret Triangulated 3D lines.
  template <template <class, class...> class V, class... V1Args, class... V2Args>
  inline void triangulate(const V<Line<FT>, V1Args...>& lineL,
                          const V<Line<FT>, V1Args...>& lineR,
                          V<Line3<FT>, V2Args...>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }

  /// @brief Triangulate generic line vector correspondences.
  /// @tparam LV Container type (must contain Line-compatible objects).
  /// @param lineL Left image lines.
  /// @param lineR Right image lines.
  /// @param[out] ret Triangulated 3D lines.
  template <class LV>
  inline void triangulateV(const LV& lineL, const LV& lineR, std::vector<Line3<FT>>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }

  /// @}

  /// @name Line Segment Triangulation
  /// @{

  /// @brief Triangulate 3D line segment from stereo correspondences.
  /// Computes endpoints by triangulating at the union of y-extents
  /// from both image segments.
  /// @param lineL Left image line segment.
  /// @param lineR Right image line segment.
  /// @return Triangulated 3D line segment (empty if degenerate).
  /// @note Requires rectified setup.
  inline LineSegment3<FT> triangulate(const LineSegment<FT>& lineL, const LineSegment<FT>& lineR) const {
    // prevent div by zero from normal x in .x()() call
    if (detail::abs(lineL.normalX()) < LIMITS<FT>::tau() || detail::abs(lineR.normalX()) < LIMITS<FT>::tau())
      return LineSegment3<FT>();

    Vec2<FT> lstart = lineL.startPoint(), lend = lineL.endPoint(), rstart = lineR.startPoint(), rend = lineR.endPoint();

    FT starty = std::min(std::min(std::min(lstart.y(), lend.y()), rstart.y()), rend.y());
    FT endy = std::max(std::max(std::max(lstart.y(), lend.y()), rstart.y()), rend.y());

    if (lstart.y() > lend.y()) std::swap(starty, endy);

    return LineSegment3<FT>(triangulate(Vec2<FT>(lineL.x(starty), starty), Vec2<FT>(lineR.x(starty), starty)),
                            triangulate(Vec2<FT>(lineL.x(endy), endy), Vec2<FT>(lineR.x(endy), endy)));
  }

  /// @brief Triangulate vector of line segment correspondences.
  /// @tparam V Container type.
  /// @tparam V1Args Template args for input container.
  /// @tparam V2Args Template args for output container.
  /// @param lineL Left image line segments.
  /// @param lineR Right image line segments.
  /// @param[out] ret Triangulated 3D line segments.
  template <template <class, class...> class V, class... V1Args, class... V2Args>
  inline void triangulate(const V<LineSegment<FT>, V1Args...>& lineL,
                          const V<LineSegment<FT>, V1Args...>& lineR,
                          V<LineSegment3<FT>, V2Args...>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }

  /// @brief Triangulate generic line segment vector correspondences.
  /// @tparam LV Container type (must contain LineSegment-compatible objects).
  /// @param lineL Left image line segments.
  /// @param lineR Right image line segments.
  /// @param[out] ret Triangulated 3D line segments.
  template <class LV>
  inline void triangulateV(const LV& lineL, const LV& lineR, std::vector<LineSegment3<FT>>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }

  /// @}
};

/// @brief Single-precision stereo triangulation.
typedef Stereo<float> Stereof;

/// @brief Double-precision stereo triangulation.
typedef Stereo<double> Stereod;

/// @brief Stereo triangulation using plane intersection for lines.
/// Triangulates lines by intersecting the interpretation planes
/// from each camera. Works for non-rectified stereo pairs.
/// @tparam FT Floating-point type.
template <class FT>
class StereoPlane : public Stereo<FT> {
 protected:
  using Stereo<FT>::camL_;
  using Stereo<FT>::camR_;
  using Stereo<FT>::rotL_;
  using Stereo<FT>::rotR_;

 public:
  typedef FT float_type;

  /// @brief Construct from projection matrices.
  /// @param projL Left camera projection matrix (3x4).
  /// @param projR Right camera projection matrix (3x4).
  StereoPlane(const Matx34<FT>& projL, const Matx34<FT>& projR) : Stereo<FT>(projL, projR) {}

  /// @brief Construct from Camera objects.
  /// @param camL Left camera.
  /// @param camR Right camera.
  StereoPlane(const Camera<FT>& camL, const Camera<FT>& camR) : Stereo<FT>(camL, camR) {}

  using Stereo<FT>::triangulate;

  /// @name Line Triangulation
  /// @{

  /// @brief Triangulate 3D line via plane intersection.
  /// Constructs interpretation planes from each camera through
  /// the 2D lines and finds their intersection line.
  /// @param lineL Left image line.
  /// @param lineR Right image line.
  /// @return Triangulated 3D line (empty if planes parallel).
  inline Line3<FT> triangulate(const Line<FT>& lineL, const Line<FT>& lineR) const {
    Line3<FT> ret;
    planeFromLine(camL_.focal(), camL_.offset(), rotL_, camL_.origin(), lineL)
        .intersection(planeFromLine(camR_.focal(), camR_.offset(), rotR_, camR_.origin(), lineR), ret);

    return ret;
  }

  /// @brief Triangulate vector of line correspondences.
  /// @tparam V Container type.
  /// @tparam V1Args Template args for input container.
  /// @tparam V2Args Template args for output container.
  /// @param lineL Left image lines.
  /// @param lineR Right image lines.
  /// @param[out] ret Triangulated 3D lines.
  template <template <class, class...> class V, class... V1Args, class... V2Args>
  inline void triangulate(const V<Line<FT>, V1Args...>& lineL,
                          const V<Line<FT>, V1Args...>& lineR,
                          V<Line3<FT>, V2Args...>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }

  /// @brief Triangulate generic line vector correspondences.
  /// @tparam LV Container type (must contain Line-compatible objects).
  /// @param lineL Left image lines.
  /// @param lineR Right image lines.
  /// @param[out] ret Triangulated 3D lines.
  template <class LV>
  inline void triangulateV(const LV& lineL, const LV& lineR, std::vector<Line3<FT>>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }

  /// @}

  /// @name Line Segment Triangulation
  /// @{

  /// @brief Triangulate 3D line segment via plane intersection.
  /// Triangulates the supporting line via plane intersection,
  /// then computes segment endpoints by projecting the 2D
  /// endpoints onto the 3D line.
  /// @param lineL Left image line segment.
  /// @param lineR Right image line segment.
  /// @return Triangulated 3D line segment (empty if degenerate).
  inline LineSegment3<FT> triangulate(const LineSegment<FT>& lineL, const LineSegment<FT>& lineR) const {
    Line3<FT> l = triangulate(static_cast<const Line<FT>>(lineL), static_cast<const Line<FT>>(lineR));

    if (l.empty() || detail::abs(lineL.normalX()) < LIMITS<FT>::tau() ||
        detail::abs(lineR.normalX()) < LIMITS<FT>::tau())
      return LineSegment3<FT>();

    Vec4<FT> tmp(
        l.normalDistance(lineFromPixel(camL_.focal(), camL_.offset(), rotL_, camL_.origin(), lineL.startPoint())),
        l.normalDistance(lineFromPixel(camL_.focal(), camL_.offset(), rotL_, camL_.origin(), lineL.endPoint())),
        l.normalDistance(lineFromPixel(camR_.focal(), camR_.offset(), rotR_, camR_.origin(), lineR.startPoint())),
        l.normalDistance(lineFromPixel(camR_.focal(), camR_.offset(), rotR_, camR_.origin(), lineR.endPoint())));

    if (tmp[0] < tmp[1])
      return LineSegment3<FT>(l, tmp.minCoeff(), tmp.maxCoeff());
    else
      return LineSegment3<FT>(l, tmp.maxCoeff(), tmp.minCoeff());
  }

  /// @brief Triangulate vector of line segment correspondences.
  /// @tparam V Container type.
  /// @tparam V1Args Template args for input container.
  /// @tparam V2Args Template args for output container.
  /// @param lineL Left image line segments.
  /// @param lineR Right image line segments.
  /// @param[out] ret Triangulated 3D line segments.
  template <template <class, class...> class V, class... V1Args, class... V2Args>
  inline void triangulate(const V<LineSegment<FT>, V1Args...>& lineL,
                          const V<LineSegment<FT>, V1Args...>& lineR,
                          V<LineSegment3<FT>, V2Args...>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }

  /// @brief Triangulate generic line segment vector correspondences.
  /// @tparam LV Container type (must contain LineSegment-compatible objects).
  /// @param lineL Left image line segments.
  /// @param lineR Right image line segments.
  /// @param[out] ret Triangulated 3D line segments.
  template <class LV>
  inline void triangulateV(const LV& lineL, const LV& lineR, std::vector<LineSegment3<FT>>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }

  /// @}
};

/// @brief Single-precision plane-intersection stereo.
typedef StereoPlane<float> Stereo2Planef;

/// @brief Double-precision plane-intersection stereo.
typedef StereoPlane<double> Stereo2Planed;

}  // namespace lsfm
