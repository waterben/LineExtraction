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


#pragma once

#include <geometry/camera.hpp>
#include <geometry/plane.hpp>


namespace lsfm {
// some basic helpers

//! convert pixel to 3d point on camera plane
template <class FT>
inline Vec3<FT> pixel2Cam3dPoint(const Vec2<FT>& focal, const Vec2<FT>& offset, const Vec2<FT>& p) {
  return Vec3<FT>(p.x() - offset.x(), p.y() - offset.y(), focal.x());
}

//! convert pixel to 3d point on camera plane
template <class FT>
inline Vec3<FT> pixel2Cam3dPoint(FT focal, const Vec2<FT>& offset, const Vec2<FT>& p) {
  return Vec3<FT>(p.x() - offset.x(), p.y() - offset.y(), focal);
}

//! convert 2d line to 3d line on camera plane
template <class FT>
inline Line3<FT> line2Cam3dLine(const Vec2<FT>& focal, const Vec2<FT>& offset, const Line<FT>& l) {
  return Line3<FT>(pixel2Cam3dPoint(focal, offset, l.origin()), Vec3<FT>(l.directionX(), l.directionY(), 0), true);
}

//! convert 2d line to 3d line on camera plane
template <class FT>
inline Line3<FT> line2Cam3dLine(FT focal, const Vec2<FT>& offset, const Line<FT>& l) {
  return Line3<FT>(pixel2Cam3dPoint(focal, offset, l.origin()), Vec3<FT>(l.directionX(), l.directionY(), 0), true);
}

//! convert point on image plane to 3d point in world coords
template <class FT>
inline Vec3<FT> pointFromPixel(const Vec2<FT>& focal,
                               const Vec2<FT>& offset,
                               const Matx<FT, 3, 3>& rot,
                               const Vec3<FT>& trans,
                               const Vec2<FT>& p) {
  return trans + rot * pixel2Cam3dPoint(focal, offset, p);
}

//! convert point on image plane to 3d point in world coords
template <class FT>
inline Vec3<FT> pointFromPixel(
    FT focal, const Vec2<FT>& offset, const Matx<FT, 3, 3>& rot, const Vec3<FT>& trans, const Vec2<FT>& p) {
  return trans + rot * pixel2Cam3dPoint(focal, offset, p);
}

//! convert ray through camera origin and 3d point on image plane (given by image pixel)
template <class FT>
inline Line3<FT> lineFromPixel(const Vec2<FT>& focal,
                               const Vec2<FT>& offset,
                               const Matx<FT, 3, 3>& rot,
                               const Vec3<FT>& trans,
                               const Vec2<FT>& p) {
  return Line3<FT>(trans, rot * pixel2Cam3dPoint(focal, offset, p));
}

//! convert ray through camera origin and 3d point on image plane (given by image pixel)
template <class FT>
inline Line3<FT> lineFromPixel(
    FT focal, const Vec2<FT>& offset, const Matx<FT, 3, 3>& rot, const Vec3<FT>& trans, const Vec2<FT>& p) {
  return Line3<FT>(trans, rot * pixel2Cam3dPoint(focal, offset, p));
}

//! convert plane through camera origin and 3d line on image plane (given by image 2d line)
template <class FT>
inline Plane<FT> planeFromLine(const Vec2<FT>& focal,
                               const Vec2<FT>& offset,
                               const Matx<FT, 3, 3>& rot,
                               const Vec3<FT>& trans,
                               const Line<FT>& l) {
  return Plane<FT>(trans, line2Cam3dLine(focal, offset, l).rotate(rot).translate(trans));
}

//! convert plane through camera origin and 3d line on image plane (given by image 2d line)
template <class FT>
inline Plane<FT> planeFromLine(
    FT focal, const Vec2<FT>& offset, const Matx<FT, 3, 3>& rot, const Vec3<FT>& trans, const Line<FT>& l) {
  return Plane<FT>(trans, line2Cam3dLine(focal, offset, l).rotate(rot).translate(trans));
}

//! Stereo class for traingulation (uses only points, rectified setup required)
template <class FT>
class Stereo {
 protected:
  Camera<FT> camL_, camR_;
  Matx33<FT> rotL_, rotR_;

 public:
  typedef FT float_type;

  Stereo(const Matx34<FT>& projL, const Matx34<FT>& projR) : camL_(projL), camR_(projR) {
    rotL_ = camL_.rotM();
    rotR_ = camR_.rotM();
  }

  Stereo(const Camera<FT>& camL, const Camera<FT>& camR)
      : camL_(camL), camR_(camR), rotL_(camL_.rotM()), rotR_(camR_.rotM()) {}

  //! compute 3d point from two rays from camera origin through corresponding stereo pixels
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

  //! compute 3d points from point vector
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

  //! compute 3d points from cv::Mat (input: points as rows, output: points as rows)
  /*inline cv::Mat triangulate(const cv::Mat &pointL, const cv::Mat &pointR) const {
      CV_Assert(pointL.rows == pointR.rows && pointL.type() == pointR.type());

      cv::Mat_<FT> ret(pointL.rows,2);
      for (int i = 0; i != pointL.rows; ++i) {
          (*reinterpret_cast<Vec2<FT>*>(&ret(i))) = triangulate(*reinterpret_cast<const
  Vec3<FT>*>(&pointL.at<FT>(i)),*reinterpret_cast<const Vec3<FT>*>(&pointR.at<FT>(i)));
      }
      return ret;
  }*/

  //! compute 3d line from two planes from camera origin through corresponding stereo lines
  inline Line3<FT> triangulate(const Line<FT>& lineL, const Line<FT>& lineR) const {
    // prevent div by zero from normal x in .x()() call
    if (detail::abs(lineL.normalX()) < LIMITS<FT>::tau() || detail::abs(lineR.normalX()) < LIMITS<FT>::tau())
      return Line3<FT>();
    return Line3<FT>::twoPoint(triangulate(Vec2<FT>(lineL.x(0), 0), Vec2<FT>(lineR.x(0), 0)),
                               triangulate(Vec2<FT>(lineL.x(100), 100), Vec2<FT>(lineR.x(100), 100)));
  }

  //! compute 2d line pairs to 3d line vector
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


  //! compute 2d line (subclass) pairs to 3d line vector
  template <class LV>
  inline void triangulateV(const LV& lineL, const LV& lineR, std::vector<Line3<FT>>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }


  //! compute 3d line segment from two planes from camera origin through corresponding stereo line segments
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


  //! compute 2d line segment pairs to 3d line segment vector
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


  //! compute 2d line segment (subclass) pairs to 3d line segment vector
  template <class LV>
  inline void triangulateV(const LV& lineL, const LV& lineR, std::vector<LineSegment3<FT>>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }
};

typedef Stereo<float> Stereof;
typedef Stereo<double> Stereod;

//! Stereo class for traingulation (uses planes for lines)
//! line and line segments can also be triangulated for non rectified camera pairs
template <class FT>
class StereoPlane : public Stereo<FT> {
 protected:
  using Stereo<FT>::camL_;
  using Stereo<FT>::camR_;
  using Stereo<FT>::rotL_;
  using Stereo<FT>::rotR_;

 public:
  typedef FT float_type;

  StereoPlane(const Matx34<FT>& projL, const Matx34<FT>& projR) : Stereo<FT>(projL, projR) {}

  StereoPlane(const Camera<FT>& camL, const Camera<FT>& camR) : Stereo<FT>(camL, camR) {}

  using Stereo<FT>::triangulate;

  //! compute 3d line from two planes from camera origin through corresponding stereo lines
  inline Line3<FT> triangulate(const Line<FT>& lineL, const Line<FT>& lineR) const {
    Line3<FT> ret;
    planeFromLine(camL_.focal(), camL_.offset(), rotL_, camL_.origin(), lineL)
        .intersection(planeFromLine(camR_.focal(), camR_.offset(), rotR_, camR_.origin(), lineR), ret);

    return ret;
  }

  //! compute 2d line pairs to 3d line vector
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


  //! compute 2d line (subclass) pairs to 3d line vector
  template <class LV>
  inline void triangulateV(const LV& lineL, const LV& lineR, std::vector<Line3<FT>>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }


  //! compute 3d line segment from two planes from camera origin through corresponding stereo line segments
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

    /*return
       LineSegment3<FT>(l,l.normalDistance(lineFromPixel(camL_.focal(),camL_.offset(),rotL_,camL_.origin(),lineL.startPoint())),
                              l.normalDistance(lineFromPixel(camL_.focal(),camL_.offset(),rotL_,camL_.origin(),lineL.endPoint())));*/
  }


  //! compute 2d line segment pairs to 3d line segment vector
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


  //! compute 2d line segment (subclass) pairs to 3d line segment vector
  template <class LV>
  inline void triangulateV(const LV& lineL, const LV& lineR, std::vector<LineSegment3<FT>>& ret) const {
    CV_Assert(lineL.size() == lineR.size());
    size_t size = lineL.size();

    ret.clear();
    ret.reserve(size);
    for (size_t i = 0; i != size; ++i) ret.push_back(triangulate(lineL[i], lineR[i]));
  }

  ///////////////////////////////// static helpers
};

typedef StereoPlane<float> Stereo2Planef;
typedef StereoPlane<double> Stereo2Planed;

}  // namespace lsfm
