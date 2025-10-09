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
#include <geometry/eigen2cv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace lsfm {


//! camera CV uses projection methods of opencv + 2point line projection
template <class FT>
class CameraCV : public CameraHom<FT> {
 protected:
  cv::Matx33<FT> camM_;
  cv::Vec3<FT> transCV_;
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

  CameraCV(FT fov,
           const Vec2<FT>& imageSize,
           const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(fov, imageSize, trans, rot),
        camM_(cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data())),
        transCV_(cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data())) {}

  CameraCV(const Vec2<FT>& focal,
           const Vec2<FT>& offset,
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)),
           const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(focal, offset, imageSize, trans, rot) {
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  CameraCV(const Vec2<FT>& focal,
           const Vec2<FT>& offset,
           const Vec2<FT>& imageSize,
           const Vec3<FT>& trans,
           const Matx33<FT>& rot)
      : CameraHom<FT>(focal, offset, imageSize, trans, rot) {
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  CameraCV(const Matx33<FT>& cam,
           const Vec3<FT>& trans,
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize), camM_(cam.data()) {}

  CameraCV(const Matx33<FT>& cam,
           const Vec3<FT>& trans,
           const Matx33<FT>& rot,
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize), camM_(cam.data()) {
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  CameraCV(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(proj, imageSize) {
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }

  CameraCV(const Camera<FT>& cam) : CameraHom<FT>(cam) {
    camM_ = cv::Matx33<FT>(Camera<FT>::composeCameraMatrix(focal_, offset_).data());
    transCV_ = cv::Vec3<FT>(Vec3<FT>(-rodrigues(rot_).transpose() * trans_).data());
  }


  ///////////////////////////////// projection methods

  using CameraHom<FT>::project;

  //! project point using opencv
  inline void project(const std::vector<Vec3<FT>>& vh, std::vector<Vec2<FT>>& ret) const {
    Vec3<FT> r = -rot_;
    ret.resize(vh.size());
    if (vh.size() > 0) cv::projectPoints(cv::Mat(vh), cv::Vec3<FT>(r.data()), transCV_, camM_, cv::Mat(), cv::Mat(ret));
  }


  //! project 3d point to 2d point using opencv
  inline Vec2<FT> project(const Vec3<FT>& p) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.push_back(p);
    project(endpoints3, endpoints2);
    return endpoints2[0];
  }

  //! project point using opencv
  inline cv::Mat project(const cv::Mat& points_as_cols) const {
    cv::Mat ret;
    Vec3<FT> r = -rot_;
    cv::projectPoints(points_as_cols, cv::Vec3<FT>(r.data()), transCV_, camM_, cv::Mat(), ret);
    return ret;
  }

  //! project 3d line to 2d line as two points using opencv
  inline Line<FT> project(const Line3<FT>& line) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.push_back(line.origin());
    endpoints3.push_back(line.distanceOrigin(1));

    project(endpoints3, endpoints2);
    return Line<FT>(endpoints2[0], endpoints2[1]);
  }

  //! project 3d line vector to 2d line vector as two points using opencv
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

  //! project 3d line (subclass) vector to 2d line vector as two points using opencv
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

  //! project 3d line segment to 2d line segment as two points using opencv
  inline LineSegment<FT> project(const LineSegment3<FT>& line) const {
    std::vector<Vec3<FT>> endpoints3;
    std::vector<Vec2<FT>> endpoints2;
    endpoints3.push_back(line.startPoint());
    endpoints3.push_back(line.endPoint());

    project(endpoints3, endpoints2);
    return LineSegment<FT>(endpoints2[0], endpoints2[1]);
  }

  //! project 3d line segment vector to 2d segment line vector using opencv
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

  //! project 3d line segment (subclass) vector to 2d line segment vector using opencv
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

  //! Virtual destructor for proper inheritance
  virtual ~CameraCV() = default;
};

typedef CameraCV<float> CameraCVf;
typedef CameraCV<double> CameraCVd;

}  // namespace lsfm
