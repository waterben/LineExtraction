//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann & Manuel Lange
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file stereocv.hpp
/// @brief OpenCV-based stereo triangulation implementation.
///
/// Provides StereoCV class that uses OpenCV's cv::triangulatePoints()
/// for point and line triangulation from stereo correspondences.

#pragma once

#include <geometry/cameracv.hpp>
#include <geometry/stereo.hpp>


namespace lsfm {

/// @brief Stereo triangulation using OpenCV functions.
///
/// Uses cv::triangulatePoints() for point triangulation. Lines are
/// triangulated by triangulating two points on each line.
/// @tparam FT Floating-point type.
/// @note Requires rectified stereo setup for line triangulation.
template <class FT>
class StereoCV {
 protected:
  cv::Matx34<FT> projL_, projR_;  ///< OpenCV projection matrices.

 public:
  typedef FT float_type;

  /// @brief Construct from projection matrices.
  /// @param projL Left camera projection matrix (3x4).
  /// @param projR Right camera projection matrix (3x4).
  StereoCV(const Matx34<FT>& projL, const Matx34<FT>& projR) : projL_(projL.data()), projR_(projR.data()) {}

  /// @brief Construct from Camera objects.
  /// @param camL Left camera.
  /// @param camR Right camera.
  StereoCV(const Camera<FT>& camL, const Camera<FT>& camR) : projL_(camL.projM().data()), projR_(camR.projM().data()) {}

  /// @name Point Triangulation
  /// @{

  /// @brief Triangulate single 3D point using OpenCV.
  /// @param pointL Left image pixel.
  /// @param pointR Right image pixel.
  /// @return Triangulated 3D point.
  inline Vec3<FT> triangulate(const Vec2<FT>& pointL, const Vec2<FT>& pointR) const {
    cv::Mat_<FT> v;
    std::vector<Vec2<FT>> camLpnts, camRpnts;
    camLpnts.reserve(1);
    camRpnts.reserve(1);

    camLpnts.push_back(pointL);
    camRpnts.push_back(pointR);

    triangulatePoints(projL_, projR_, cv::Mat(camLpnts), cv::Mat(camRpnts), v);

    if (detail::abs(v(3)) < LIMITS<FT>::tau()) return Vec3<FT>(0, 0, 0);
    v /= v(3);
    return Vec3<FT>(v(0), v(1), v(2));
  }

  /// @brief Triangulate vector of point correspondences using OpenCV.
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
    cv::Mat_<FT> v;
    triangulatePoints(projL_, projR_, cv::Mat(pointL), cv::Mat(pointR), v);

    ret.resize(pointL.size());
    cv::Mat tmp(ret);
    cv::fromHomogeneous<FT>(v, tmp, true);
  }

  /// @}

  /// @name Line Triangulation
  /// @{

  /// @brief Triangulate 3D line using OpenCV point triangulation.
  ///
  /// Triangulates two points at y=0 and y=100 on each line,
  /// then constructs a 3D line through them.
  /// @param lineL Left image line.
  /// @param lineR Right image line.
  /// @return Triangulated 3D line (empty if degenerate).
  /// @note Requires rectified setup.
  inline Line3<FT> triangulate(const Line<FT>& lineL, const Line<FT>& lineR) const {
    if (detail::abs(lineL.normalX()) < LIMITS<FT>::tau() || detail::abs(lineR.normalX()) < LIMITS<FT>::tau())
      return Line3<FT>();

    cv::Mat_<FT> points4Dresult;
    std::vector<Vec2<FT>> camLpnts, camRpnts;
    camLpnts.reserve(2);
    camRpnts.reserve(2);

    camLpnts.push_back(Vec2<FT>(lineL.x()(0), 0));
    camLpnts.push_back(Vec2<FT>(lineL.x()(100), 100));
    camRpnts.push_back(Vec2<FT>(lineR.x()(0), 0));
    camRpnts.push_back(Vec2<FT>(lineR.x()(100), 100));

    triangulatePoints(projL_, projR_, cv::Mat(camLpnts), cv::Mat(camRpnts), points4Dresult);


    cv::Mat_<FT> v1 = points4Dresult.col(0), v2 = points4Dresult.col(1);
    if (detail::abs(v1(3)) < LIMITS<FT>::tau() || detail::abs(v2(3)) < LIMITS<FT>::tau()) return Line3<FT>();
    v1 /= v1(3);
    v2 /= v2(3);
    return Line3<FT>(Vec3<FT>(v1(0), v1(1), v1(2)), Vec3<FT>(v2(0) - v1(0), v2(1) - v1(1), v2(2) - v1(2)));
  }

  /// @brief Triangulate vector of line correspondences using OpenCV.
  /// @tparam V Container type.
  /// @tparam V1Args Template args for input container.
  /// @tparam V2Args Template args for output container.
  /// @param linesL Left image lines.
  /// @param linesR Right image lines.
  /// @param[out] ret Triangulated 3D lines.
  template <template <class, class...> class V, class... V1Args, class... V2Args>
  inline void triangulate(const V<Line<FT>, V1Args...>& linesL,
                          const V<Line<FT>, V1Args...>& linesR,
                          V<Line3<FT>, V2Args...>& ret) const {
    CV_Assert(linesL.size() == linesR.size());

    size_t size = linesL.size();
    std::vector<Vec3<FT>> vp3;
    std::vector<Vec2<FT>> camLpnts, camRpnts;
    camLpnts.reserve(size * 2);
    camRpnts.reserve(size * 2);

    for (size_t i = 0; i != size; ++i) {
      if (detail::abs(linesL[i].normalX()) < LIMITS<FT>::tau() ||
          detail::abs(linesR[i].normalX()) < LIMITS<FT>::tau()) {
        camLpnts.push_back(Vec2<FT>(0, 0));
        camLpnts.push_back(Vec2<FT>(0, 0));
        camRpnts.push_back(Vec2<FT>(0, 0));
        camRpnts.push_back(Vec2<FT>(0, 0));
        continue;
      }


      camLpnts.push_back(Vec2<FT>(linesL[i].x(0), 0));
      camLpnts.push_back(Vec2<FT>(linesL[i].x(100), 100));
      camRpnts.push_back(Vec2<FT>(linesR[i].x(0), 0));
      camRpnts.push_back(Vec2<FT>(linesR[i].x(100), 100));
    }

    triangulate(camLpnts, camRpnts, vp3);
    ret.clear();
    ret.reserve(size);

    size = vp3.size();
    for (size_t i = 0; i != size; i += 2) {
      ret.push_back(Line3<FT>::twoPoint(vp3[i], vp3[i + 1]));
    }
  }

  /// @brief Triangulate generic line vector correspondences using OpenCV.
  /// @tparam LV Container type (must contain Line-compatible objects).
  /// @param linesL Left image lines.
  /// @param linesR Right image lines.
  /// @param[out] ret Triangulated 3D lines.
  template <class LV>
  inline void triangulateV(const LV& linesL, const LV& linesR, std::vector<Line3<FT>>& ret) const {
    CV_Assert(linesL.size() == linesR.size());

    size_t size = linesL.size();
    std::vector<Vec3<FT>> vp3;
    std::vector<Vec2<FT>> camLpnts, camRpnts;
    camLpnts.reserve(size * 2);
    camRpnts.reserve(size * 2);

    for (size_t i = 0; i != size; ++i) {
      if (detail::abs(linesL[i].normalX()) < LIMITS<FT>::tau() ||
          detail::abs(linesR[i].normalX()) < LIMITS<FT>::tau()) {
        camLpnts.push_back(Vec2<FT>(0, 0));
        camLpnts.push_back(Vec2<FT>(0, 0));
        camRpnts.push_back(Vec2<FT>(0, 0));
        camRpnts.push_back(Vec2<FT>(0, 0));
        continue;
      }


      camLpnts.push_back(Vec2<FT>(linesL[i].x(0), 0));
      camLpnts.push_back(Vec2<FT>(linesL[i].x(100), 100));
      camRpnts.push_back(Vec2<FT>(linesR[i].x(0), 0));
      camRpnts.push_back(Vec2<FT>(linesR[i].x(100), 100));
    }

    triangulate(camLpnts, camRpnts, vp3);
    ret.clear();
    ret.reserve(size);

    size = vp3.size();
    for (size_t i = 0; i != size; i += 2) {
      ret.push_back(Line3<FT>::twoPoint(vp3[i], vp3[i + 1]));
    }
  }

  /// @}

  /// @name Line Segment Triangulation
  /// @{

  /// @brief Triangulate 3D line segment using OpenCV.
  ///
  /// Computes segment endpoints by triangulating at the union
  /// of y-extents from both image segments.
  /// @param lineL Left image line segment.
  /// @param lineR Right image line segment.
  /// @return Triangulated 3D line segment (empty if degenerate).
  /// @note Requires rectified setup.
  inline LineSegment3<FT> triangulate(const LineSegment<FT>& lineL, const LineSegment<FT>& lineR) const {
    if (detail::abs(lineL.normalX()) < LIMITS<FT>::tau() || detail::abs(lineR.normalX()) < LIMITS<FT>::tau())
      return LineSegment3<FT>();

    Vec2<FT> lstart = lineL.startPoint(), lend = lineL.endPoint(), rstart = lineR.startPoint(), rend = lineR.endPoint();

    FT starty = std::min(std::min(std::min(lstart.y(), lend.y()), rstart.y()), rend.y());
    FT endy = std::max(std::max(std::max(lstart.y(), lend.y()), rstart.y()), rend.y());

    if (lstart.y() > lend.y()) std::swap(starty, endy);


    cv::Mat_<FT> points4Dresult;
    std::vector<Vec2<FT>> camLpnts, camRpnts;
    camLpnts.reserve(2);
    camRpnts.reserve(2);

    camLpnts.push_back(Vec2<FT>(lineL.x(starty), starty));
    camLpnts.push_back(Vec2<FT>(lineL.x(endy), endy));
    camRpnts.push_back(Vec2<FT>(lineR.x(starty), starty));
    camRpnts.push_back(Vec2<FT>(lineR.x(endy), endy));

    triangulatePoints(projL_, projR_, camLpnts, camRpnts, points4Dresult);


    cv::Mat_<FT> v1 = points4Dresult.col(0), v2 = points4Dresult.col(1);
    if (detail::abs(v1(3)) < LIMITS<FT>::tau() || detail::abs(v2(3)) < LIMITS<FT>::tau()) return LineSegment3<FT>();
    v1 /= v1(3);
    v2 /= v2(3);
    return LineSegment3<FT>(Vec3<FT>(v1(0), v1(1), v1(2)), Vec3<FT>(v2(0) - v1(0), v2(1) - v1(1), v2(2) - v1(2)));
  }

  /// @brief Triangulate vector of line segment correspondences using OpenCV.
  /// @tparam V Container type.
  /// @tparam V1Args Template args for input container.
  /// @tparam V2Args Template args for output container.
  /// @param linesL Left image line segments.
  /// @param linesR Right image line segments.
  /// @param[out] ret Triangulated 3D line segments.
  template <template <class, class...> class V, class... V1Args, class... V2Args>
  inline void triangulate(const V<LineSegment<FT>, V1Args...>& linesL,
                          const V<LineSegment<FT>, V1Args...>& linesR,
                          V<LineSegment3<FT>, V2Args...>& ret) const {
    CV_Assert(linesL.size() == linesR.size());

    size_t size = linesL.size();
    std::vector<Vec3<FT>> vp3;
    std::vector<Vec2<FT>> camLpnts, camRpnts;
    camLpnts.reserve(size * 2);
    camRpnts.reserve(size * 2);

    for (size_t i = 0; i != size; ++i) {
      if (detail::abs(linesL[i].normalX()) < LIMITS<FT>::tau() ||
          detail::abs(linesR[i].normalX()) < LIMITS<FT>::tau()) {
        camLpnts.push_back(Vec2<FT>(0, 0));
        camLpnts.push_back(Vec2<FT>(0, 0));
        camRpnts.push_back(Vec2<FT>(0, 0));
        camRpnts.push_back(Vec2<FT>(0, 0));
        continue;
      }

      Vec2<FT> lstart = linesL[i].startPoint(), lend = linesL[i].endPoint(), rstart = linesR[i].startPoint(),
               rend = linesR[i].endPoint();

      FT starty = std::min(std::min(std::min(lstart.y(), lend.y()), rstart.y()), rend.y());
      FT endy = std::max(std::max(std::max(lstart.y(), lend.y()), rstart.y()), rend.y());

      if (lstart.y() > lend.y()) std::swap(starty, endy);


      camLpnts.push_back(Vec2<FT>(linesL[i].x(starty), starty));
      camLpnts.push_back(Vec2<FT>(linesL[i].x(endy), endy));
      camRpnts.push_back(Vec2<FT>(linesR[i].x(starty), starty));
      camRpnts.push_back(Vec2<FT>(linesR[i].x(endy), endy));
    }

    triangulate(camLpnts, camRpnts, vp3);
    ret.clear();
    ret.reserve(size);

    size = vp3.size();
    for (size_t i = 0; i != size; i += 2) {
      ret.push_back(LineSegment3<FT>(vp3[i], vp3[i + 1]));
    }
  }

  /// @brief Triangulate generic line segment vector correspondences using OpenCV.
  /// @tparam LV Container type (must contain LineSegment-compatible objects).
  /// @param linesL Left image line segments.
  /// @param linesR Right image line segments.
  /// @param[out] ret Triangulated 3D line segments.
  template <class LV>
  inline void triangulateV(const LV& linesL, const LV& linesR, std::vector<LineSegment3<FT>>& ret) const {
    CV_Assert(linesL.size() == linesR.size());

    size_t size = linesL.size();
    std::vector<Vec3<FT>> vp3;
    std::vector<Vec2<FT>> camLpnts, camRpnts;
    camLpnts.reserve(size * 2);
    camRpnts.reserve(size * 2);

    for (size_t i = 0; i != size; ++i) {
      if (detail::abs(linesL[i].normalX()) < LIMITS<FT>::tau() ||
          detail::abs(linesR[i].normalX()) < LIMITS<FT>::tau()) {
        camLpnts.push_back(Vec2<FT>(0, 0));
        camLpnts.push_back(Vec2<FT>(0, 0));
        camRpnts.push_back(Vec2<FT>(0, 0));
        camRpnts.push_back(Vec2<FT>(0, 0));
        continue;
      }

      Vec2<FT> lstart = linesL[i].startPoint(), lend = linesL[i].endPoint(), rstart = linesR[i].startPoint(),
               rend = linesR[i].endPoint();

      FT starty = std::min(std::min(std::min(lstart.y(), lend.y()), rstart.y()), rend.y());
      FT endy = std::max(std::max(std::max(lstart.y(), lend.y()), rstart.y()), rend.y());

      if (lstart.y() > lend.y()) std::swap(starty, endy);


      camLpnts.push_back(Vec2<FT>(linesL[i].x(starty), starty));
      camLpnts.push_back(Vec2<FT>(linesL[i].x(endy), endy));
      camRpnts.push_back(Vec2<FT>(linesR[i].x(starty), starty));
      camRpnts.push_back(Vec2<FT>(linesR[i].x(endy), endy));
    }

    triangulate(camLpnts, camRpnts, vp3);
    ret.clear();
    ret.reserve(size);

    size = vp3.size();
    for (size_t i = 0; i != size; i += 2) {
      ret.push_back(LineSegment<FT>(vp3[i], vp3[i + 1]));
    }
  }

  /// @}
};

/// @brief Single-precision OpenCV stereo triangulation.
typedef StereoCV<float> StereoCVf;

/// @brief Double-precision OpenCV stereo triangulation.
typedef StereoCV<double> StereoCVd;

}  // namespace lsfm
