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

/// @file cameracv.hpp
/// @brief OpenCV-based camera projection implementation.
///
/// Provides CameraCV class that uses OpenCV's cv::projectPoints()
/// for 3D-to-2D projection. Offers better accuracy for distorted
/// projections while maintaining interface compatibility.

#pragma once

#include <geometry/camera.hpp>
#include <geometry/eigen2cv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace lsfm {

/// @brief Camera using OpenCV projection methods.
///
/// Uses cv::projectPoints() for point projection, providing compatibility
/// with OpenCV's camera model and distortion handling. Projects lines
/// using two-point method through OpenCV's projection pipeline.
/// @tparam FT Floating-point type.
template <class FT>
class CameraCV : public CameraHom<FT> {
 protected:
  cv::Matx33<FT> camM_;   ///< OpenCV camera matrix.
  cv::Vec3<FT> transCV_;  ///< Translation in OpenCV convention.
  using CameraHom<FT>::trans_;
  using CameraHom<FT>::rot_;
  using CameraHom<FT>::focal_;
  using CameraHom<FT>::offset_;
  using CameraHom<FT>::proj_;

  virtual void updateInternals() {
    CameraHom<FT>::updateInternals();
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

 public:
  typedef FT float_type;

  /// @brief Default/parameter constructor.
  CameraCV(FT focal_x = FT(0),
           FT focal_y = FT(0),
           FT offset_x = FT(0),
           FT offset_y = FT(0),
           FT width = FT(0),
           FT height = FT(0),
           FT trans_x = FT(0),
           FT trans_y = FT(0),
           FT trans_z = FT(0),
           FT rot_x = FT(0),
           FT rot_y = FT(0),
           FT rot_z = FT(0))
      : CameraHom<FT>(
            focal_x, focal_y, offset_x, offset_y, width, height, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z) {
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  /// @brief Construct from FOV and image size.
  CameraCV(FT fov,
           const Vec2<FT>& imageSize,
           const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(fov, imageSize, trans, rot),
        camM_(cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data())),
        transCV_(cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data())) {}

  /// @brief Construct from intrinsic/extrinsic vectors.
  /// @param focal Focal lengths (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @param imageSize Image dimensions.
  /// @param trans Translation vector.
  /// @param rot Rodrigues rotation vector.
  CameraCV(const Vec2<FT>& focal,
           const Vec2<FT>& offset,
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)),
           const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(focal, offset, imageSize, trans, rot) {
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  /// @brief Construct from intrinsic/extrinsic vectors with rotation matrix.
  /// @param focal Focal lengths (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @param imageSize Image dimensions.
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  CameraCV(const Vec2<FT>& focal,
           const Vec2<FT>& offset,
           const Vec2<FT>& imageSize,
           const Vec3<FT>& trans,
           const Matx33<FT>& rot)
      : CameraHom<FT>(focal, offset, imageSize, trans, rot) {
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  /// @brief Construct from camera matrix and extrinsics.
  /// @param cam Camera intrinsic matrix (3x3).
  /// @param trans Translation vector.
  /// @param rot Rodrigues rotation vector.
  /// @param imageSize Image dimensions.
  CameraCV(const Matx33<FT>& cam,
           const Vec3<FT>& trans,
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize), camM_(cam.data()) {}

  /// @brief Construct from camera and rotation matrices.
  /// @param cam Camera intrinsic matrix (3x3).
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  /// @param imageSize Image dimensions.
  CameraCV(const Matx33<FT>& cam,
           const Vec3<FT>& trans,
           const Matx33<FT>& rot,
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize), camM_(cam.data()) {
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  /// @brief Construct from projection matrix.
  /// @param proj 3x4 projection matrix.
  /// @param imageSize Image dimensions.
  CameraCV(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(proj, imageSize) {
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  /// @brief Copy construct from base Camera.
  /// @param cam Source camera.
  CameraCV(const Camera<FT>& cam) : CameraHom<FT>(cam) {
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  /// @name Projection Methods
  /// @{

  using CameraHom<FT>::project;

  /// @brief Project vector of 3D points using OpenCV.
  /// @param vh Input 3D point vector.
  /// @param ret Output 2D point vector.
  inline void project(const std::vector<Vec3<FT>>& vh, std::vector<Vec2<FT>>& ret) const {
    Vec3<FT> r = -rot_;
    ret.resize(vh.size());
    if (vh.size() > 0) cv::projectPoints(cv::Mat(vh), cv::Vec3<FT>(r.data()), transCV_, camM_, cv::Mat(), cv::Mat(ret));
  }

  /// @brief Project single 3D point using OpenCV.
  /// @param p 3D point.
  /// @return 2D projected point.
  inline Vec2<FT> project(const Vec3<FT>& p) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.push_back(p);
    project(endpoints3, endpoints2);
    return endpoints2[0];
  }

  /// @brief Project points from OpenCV Mat using OpenCV.
  /// @param points_as_cols Points stored as columns.
  /// @return Projected 2D points as Mat.
  inline cv::Mat project(const cv::Mat& points_as_cols) const {
    cv::Mat ret;
    Vec3<FT> r = -rot_;
    cv::projectPoints(points_as_cols, cv::Vec3<FT>(r.data()), transCV_, camM_, cv::Mat(), ret);
    return ret;
  }

  /// @brief Project 3D line to 2D via two-point projection using OpenCV.
  /// @param line 3D line.
  /// @return 2D projected line.
  inline Line<FT> project(const Line3<FT>& line) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.push_back(line.origin());
    endpoints3.push_back(line.distanceOrigin(1));

    project(endpoints3, endpoints2);
    return Line<FT>(endpoints2[0], endpoints2[1]);
  }

  /// @brief Project vector of 3D lines to 2D using OpenCV.
  /// @tparam V1 Input container type.
  /// @tparam LV Output container type.
  /// @param vl3 Input 3D line vector.
  /// @param ret Output 2D line vector.
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<Line3<FT>, V1Args...>& vl3, LV& ret) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) {
      endpoints3.push_back(l.origin());
      endpoints3.push_back(l.distanceOrigin(1));
    });
    ret.clear();
    ret.reserve(vl3.size());
    project(endpoints3, endpoints2);
    for (size_t i = 0; i != endpoints2.size(); i += 2) {
      ret.push_back(Line<FT>(endpoints2[i], endpoints2[i + 1]));
    }
  }

  /// @brief Project generic 3D line vector to 2D using OpenCV.
  /// @tparam LV Input container type (must contain Line3-compatible objects).
  /// @param vl3 Input 3D line vector.
  /// @param ret Output 2D line vector.
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<Line<FT>>& ret) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) {
      endpoints3.push_back(l.origin());
      endpoints3.push_back(l.distanceOrigin(1));
    });
    ret.clear();
    ret.reserve(vl3.size());
    project(endpoints3, endpoints2);
    for (size_t i = 0; i != endpoints2.size(); i += 2) {
      ret.push_back(Line<FT>(endpoints2[i], endpoints2[i + 1]));
    }
  }

  /// @brief Project 3D line segment to 2D using OpenCV.
  /// @param line 3D line segment.
  /// @return 2D line segment.
  inline LineSegment<FT> project(const LineSegment3<FT>& line) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.push_back(line.startPoint());
    endpoints3.push_back(line.endPoint());

    project(endpoints3, endpoints2);
    return LineSegment<FT>(endpoints2[0], endpoints2[1]);
  }

  /// @brief Project vector of 3D line segments to 2D using OpenCV.
  /// @tparam V1 Input container type.
  /// @tparam LV Output container type.
  /// @param vl3 Input 3D line segment vector.
  /// @param ret Output 2D line segment vector.
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<LineSegment3<FT>, V1Args...>& vl3, LV& ret) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.reserve(vl3.size() * 2);
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) {
      endpoints3.push_back(l.startPoint());
      endpoints3.push_back(l.endPoint());
    });
    ret.clear();
    ret.reserve(vl3.size());
    project(endpoints3, endpoints2);
    for (size_t i = 0; i != endpoints2.size(); i += 2) {
      ret.push_back(LineSegment<FT>(endpoints2[i], endpoints2[i + 1]));
    }
  }

  /// @brief Project generic 3D line segment vector to 2D using OpenCV.
  /// @tparam LV Input container type (must contain LineSegment3-compatible objects).
  /// @param vl3 Input 3D line segment vector.
  /// @param ret Output 2D line segment vector.
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<LineSegment<FT>>& ret) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.reserve(vl3.size() * 2);
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) {
      endpoints3.push_back(l.startPoint());
      endpoints3.push_back(l.endPoint());
    });
    ret.clear();
    ret.reserve(vl3.size());
    project(endpoints3, endpoints2);
    for (size_t i = 0; i != endpoints2.size(); i += 2) {
      ret.push_back(LineSegment<FT>(endpoints2[i], endpoints2[i + 1]));
    }
  }

  /// @}

  /// @brief Virtual destructor for proper inheritance.
  virtual ~CameraCV() = default;
};

/// @brief Single-precision OpenCV camera.
typedef CameraCV<float> CameraCVf;

/// @brief Double-precision OpenCV camera.
typedef CameraCV<double> CameraCVd;

}  // namespace lsfm
