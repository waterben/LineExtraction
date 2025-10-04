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

#include <geometry/line3.hpp>
#include <geometry/pose.hpp>
#include <geometry/vision.hpp>

namespace lsfm {

//! basic camera with no projection
template <class FT>
class Camera : public Pose<FT> {
 protected:
  Vec2<FT> focal_, offset_, imageSize_;

  using Pose<FT>::trans_;
  using Pose<FT>::rot_;

  virtual void updateInternals() { Pose<FT>::updateInternals(); }

  void initSize() {
    if (imageSize_.x() <= FT(0)) {
      imageSize_.x() = detail::abs(offset_.x() * FT(2));
    }
    if (imageSize_.y() <= FT(0)) {
      imageSize_.y() = detail::abs(offset_.y() * FT(2));
    }
  }

 public:
  typedef FT float_type;

  //! Init by intrinsics and extrinsics and image size
  Camera(FT focal_x = FT(0),
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
      : Pose<FT>(trans_x, trans_y, trans_z, rot_x, rot_y, rot_z),
        focal_(focal_x, focal_y),
        offset_(offset_x, offset_y),
        imageSize_(width, height) {
    initSize();
  }

  //! Init by field of view in x (rad), image size, and camera pose, it is expected that the offset is in the middle of
  //! the image
  Camera(FT fov,
         const Vec2<FT>& imageSize,
         const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
         const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : Pose<FT>(trans, rot),
        focal_(fov2Focal(fov, imageSize.x()), fov2Focal(fov, imageSize.x())),
        offset_(imageSize * FT(0.5)),
        imageSize_(imageSize) {}


  //! init by intrinsics and extrinsics
  Camera(const Vec2<FT>& focal,
         const Vec2<FT>& offset,
         const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)),
         const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
         const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : Pose<FT>(trans, rot), focal_(focal), offset_(offset), imageSize_(imageSize) {
    initSize();
  }

  //! init by intrinsics and extrinsics
  Camera(const Vec2<FT>& focal,
         const Vec2<FT>& offset,
         const Vec2<FT>& imageSize,
         const Vec3<FT>& trans,
         const Matx33<FT>& rot)
      : Pose<FT>(trans, rot), focal_(focal), offset_(offset), imageSize_(imageSize) {
    initSize();
  }

  //! init by intrinsics and extrinsics
  Camera(const Matx33<FT>& cam,
         const Vec3<FT>& trans,
         const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)),
         const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : Pose<FT>(trans, rot), imageSize_(imageSize) {
    decomposeCameraMatrix(cam, focal_, offset_);
    initSize();
  }

  //! init by intrinsics and extrinsics
  Camera(const Matx33<FT>& cam,
         const Vec3<FT>& trans,
         const Matx33<FT>& rot,
         const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : Pose<FT>(trans, rot), imageSize_(imageSize) {
    decomposeCameraMatrix(cam, focal_, offset_);
    initSize();
  }

  //! init by projection matrix - Careful with image Size! initSize() just multiplies the origin times two!
  Camera(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0))) : imageSize_(imageSize) {
    decomposeProjectionMatrix(proj, focal_, offset_, trans_, rot_);
    initSize();
  }

  Camera(const Camera<FT>& cam)
      : Pose<FT>(cam.trans_, cam.rot_), focal_(cam.focal_), offset_(cam.offset_), imageSize_(cam.imageSize_) {}

  //! test for empty camera (f = 0)
  virtual bool empty() const { return (focal_.x() == 0 && focal_.y() == 0); }

  //! get camera focal
  inline const Vec2<FT>& focal() const { return focal_; }

  //! get camera offset
  inline const Vec2<FT>& offset() const { return offset_; }

  //! get camera focal length
  virtual const FT focalLength() const { return composeFocalLength(focal_); }

  //! get camera field of view in rad
  virtual Vec2<FT> fov() const { return focal2Fov(focal_, offset_); }

  //! get camera width
  virtual FT width() const { return imageSize_.x(); }

  //! get camera height
  virtual FT height() const { return imageSize_.y(); }

  //! get camera size
  inline Vec2<FT> imageSize() const { return imageSize_; }

  //! get camera size
  virtual FT aspectRatio() const { return composeAspectRatio(imageSize_); }

  //! get camera calibration matrix
  virtual Matx33<FT> camM() const { return composeCameraMatrix(focal_, offset_); }

  //! get camera calibration matrix COF
  virtual Matx33<FT> camMCOF() const { return composeCameraMatrixCOF(focal_, offset_); }

  //! get projection matrix
  virtual Matx34<FT> projM() const { return composeProjectionMatrix(focal_, offset_, trans_, rot_); }

  //! get homogeneous projection matrix
  virtual Matx44<FT> projMH() const { return composeProjectionMatrixH(focal_, offset_, trans_, rot_); }

  //! set camera size
  void imageSize(const int width, const int height) {
    imageSize_.x() = width;
    imageSize_.y() = height;
  }

  ///////////////////////////////// static helpers

  //! compute focal from focal length and aspect ratio (x/y)
  static inline Vec2<FT> composeFocal(FT focalLength, FT aspectRatio = 1) {
    return Vec2<FT>(focalLength * aspectRatio, focalLength);
  }

  //! compute focal from focal length and pixel size
  static inline Vec2<FT> composeFocal(FT focalLength, const Vec2<FT>& pixelSize) {
    return Vec2<FT>(focalLength * pixelSize.x(), focalLength * pixelSize.y());
  }

  //! compute focal length from focal
  static inline FT composeFocalLength(const Vec2<FT>& focal) { return focal.y(); }

  //! compute focal length from focal
  static inline FT composeFocalLength(const Vec2<FT>& focal, const Vec2<FT>& pixelSize) {
    return focal.x() / pixelSize.x();
  }

  //! compute aspect ration form size
  static inline FT composeAspectRatio(const Vec2<FT>& size) { return size.x() / size.y(); }

  //! compute focal from field of view and size
  static inline FT fov2Focal(FT fov, FT size) { return size / (2 * detail::tan(fov / 2)); }

  //! compute focal from field of view and image size
  static inline Vec2<FT> fov2Focal(const Vec2<FT>& fov, const Vec2<FT>& size) {
    return Vec2<FT>(size.x() / (2 * detail::tan(fov.x() / 2)), size.y() / (2 * detail::tan(fov.y() / 2)));
  }

  //! compute field of view from focal and offset
  static inline FT focal2Fov(FT focal, FT offset) { fov = 2 * detail::atan(offset / (2 * focal)); }

  //! compute field of view from focal and offset
  static inline Vec2<FT> focal2Fov(const Vec2<FT>& focal, const Vec2<FT>& offset) {
    return Vec2<FT>(2 * detail::atan(detail::abs(offset.x()) / focal.x()),
                    2 * detail::atan(detail::abs(offset.y()) / focal.y()));
  }

  //! get camera calibration matrix
  static inline Matx33<FT> composeCameraMatrix(const Vec2<FT>& focal, const Vec2<FT>& offset) {
    return Matx<FT, 3, 3>(focal.x(), FT(0), offset.x(), FT(0), focal.y(), offset.y(), FT(0), FT(0), FT(1));
  }


  //! get camera calibration matrix COF
  static inline Matx33<FT> composeCameraMatrixCOF(const Vec2<FT>& focal, const Vec2<FT>& offset) {
    return Matx<FT, 3, 3>(focal.y(), FT(0), FT(0), FT(0), focal.x(), FT(0), -focal.y() * offset.x(),
                          -focal.x() * offset.y(), focal.x() * focal.y());
  }

  //! get camera calibration matrix COF
  static inline Matx33<FT> composeCameraMatrixCOF(const Matx33<FT>& cam) {
    return Matx33<FT>(cam(1, 1), FT(0), FT(0), FT(0), cam(0, 0), FT(0), -cam(1, 1) * cam(0, 2), -cam(0, 0) * cam(1, 2),
                      cam(0, 0) * cam(1, 1));
  }

  //! compose camera projection matrix
  static inline Matx34<FT> composeProjectionMatrix(const Vec2<FT>& focal,
                                                   const Vec2<FT>& offset,
                                                   const Vec3<FT>& trans,
                                                   const Vec3<FT>& rot) {
    return composeProjectionMatrix(composeCameraMatrix(focal, offset), trans, rot);
  }

  //! compose camera projection matrix
  static inline Matx34<FT> composeProjectionMatrix(const Vec2<FT>& focal,
                                                   const Vec2<FT>& offset,
                                                   const Vec3<FT>& trans,
                                                   const Matx33<FT>& rot) {
    return composeProjectionMatrix(composeCameraMatrix(focal, offset), trans, rot);
  }

  //! compose camera projection matrix
  static inline Matx34<FT> composeProjectionMatrix(const Matx33<FT>& cam, const Vec3<FT>& trans, const Vec3<FT>& rot) {
    return composeProjectionMatrix(cam, trans, rodrigues(rot));
  }

  //! compose camera projection matrix
  static inline Matx34<FT> composeProjectionMatrix(const Matx33<FT>& cam,
                                                   const Vec3<FT>& trans,
                                                   const Matx33<FT>& rot) {
    // Matx44<FT> camH = composeProjectionMatrixH(cam,trans,rot);
    // return Matx34<FT>(camH(0,0),camH(0,1),camH(0,2),camH(0,3),
    // camH(1,0),camH(1,1),camH(1,2),camH(1,3),
    // camH(2,0),camH(2,1),camH(2,2),camH(2,3));
    return Matx34<FT>(composeProjectionMatrixH(cam, trans, rot).data());
  }

  //! compose camera projection matrix
  static inline Matx44<FT> composeProjectionMatrixH(const Vec2<FT>& focal,
                                                    const Vec2<FT>& offset,
                                                    const Vec3<FT>& trans,
                                                    const Vec3<FT>& rot) {
    return composeProjectionMatrixH(composeCameraMatrix(focal, offset), trans, rot);
  }

  //! compose camera projection matrix
  static inline Matx44<FT> composeProjectionMatrixH(const Vec2<FT>& focal,
                                                    const Vec2<FT>& offset,
                                                    const Vec3<FT>& trans,
                                                    const Matx33<FT>& rot) {
    return composeProjectionMatrixH(composeCameraMatrix(focal, offset), trans, rot);
  }

  //! compose camera projection matrix
  static inline Matx44<FT> composeProjectionMatrixH(const Matx33<FT>& cam, const Vec3<FT>& trans, const Vec3<FT>& rot) {
    return composeProjectionMatrixH(cam, trans, rodrigues(rot));
  }

  //! compose camera projection matrix
  static inline Matx44<FT> composeProjectionMatrixH(const Matx33<FT>& cam,
                                                    const Vec3<FT>& trans,
                                                    const Matx33<FT>& rot) {
    Matx33<FT> R = rot.transpose();
    Vec3<FT> t = -R * trans;

    Matx44<FT> E(composeHom(t, R));
    Matx44<FT> P(composeHom(cam));

    return P * E;
  }

  //! decompose camera matrix to focal and offset
  static inline void decomposeCameraMatrix(const Matx33<FT>& cam, Vec2<FT>& focal, Vec2<FT>& offset) {
    FT n = cam(2, 2);
    focal = Vec2<FT>(cam(0, 0) / n, cam(1, 1) / n);
    offset = Vec2<FT>(cam(0, 2) / n, cam(1, 2) / n);
  }

  //! decompose camera projection matrix
  static inline void decomposeProjectionMatrix(const Matx34<FT>& proj,
                                               Matx33<FT>& cam,
                                               Vec3<FT>& trans,
                                               Matx33<FT>& rot) {
    lsfm::decomposeProjectionMatrix(proj, cam, trans, rot);
  }

  //! decompose camera projection matrix
  static inline void decomposeProjectionMatrix(const Matx34<FT>& proj,
                                               Matx33<FT>& cam,
                                               Vec3<FT>& trans,
                                               Vec3<FT>& rot) {
    Matx33<FT> rTmp;
    decomposeProjectionMatrix(proj, cam, trans, rTmp);
    rot = rodrigues(rTmp);
  }

  //! decompose camera projection matrix
  static inline void decomposeProjectionMatrix(
      const Matx34<FT>& proj, Vec2<FT>& focal, Vec2<FT>& offset, Vec3<FT>& trans, Matx33<FT>& rot) {
    Matx33<FT> tmp;
    decomposeProjectionMatrix(proj, tmp, trans, rot);
    decomposeCameraMatrix(tmp, focal, offset);
  }

  //! decompose camera projection matrix
  static inline void decomposeProjectionMatrix(
      const Matx34<FT>& proj, Vec2<FT>& focal, Vec2<FT>& offset, Vec3<FT>& trans, Vec3<FT>& rot) {
    Matx33<FT> tmp;
    decomposeProjectionMatrix(proj, tmp, trans, rot);
    decomposeCameraMatrix(tmp, focal, offset);
  }

  //! project 3d point
  static inline bool project(const Matx34<FT>& projM, const Vec3<FT>& h, Vec2<FT>& ret) {
    Vec3<FT> tmp = projM * h.homogeneous();
    // Vec2<FT> tmp2 = Vec2<FT>(tmp[0]/tmp[2],tmp[1]/tmp[2]);
    if (detail::abs(tmp[2]) < LIMITS<FT>::tau()) {
      ret = Vec2<FT>(FT(0), FT(0));
      return false;
    } else {
      ret = Vec2<FT>(tmp[0] / tmp[2], tmp[1] / tmp[2]);
      return true;
    }
  }
};

template <class FT>
inline Camera<FT> operator*(const Matx44<FT>& m, const Camera<FT>& cam) {
  Camera<FT> c = cam;
  c.transform(m);
  return c;
}

typedef Camera<float> Cameraf;
typedef Camera<double> Camerad;

//! camera with homogeneous projection using projection matrix (no line support)
template <class FT>
class CameraHom : public Camera<FT> {
 protected:
  Matx34<FT> proj_;
  using Camera<FT>::trans_;
  using Camera<FT>::rot_;
  using Camera<FT>::focal_;
  using Camera<FT>::offset_;

  virtual void updateInternals() {
    Camera<FT>::updateInternals();
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_);
  }

 public:
  typedef FT float_type;

  CameraHom(FT focal_x = 0,
            FT focal_y = 0,
            FT offset_x = 0,
            FT offset_y = 0,
            FT width = 0,
            FT height = 0,
            FT trans_x = 0,
            FT trans_y = 0,
            FT trans_z = 0,
            FT rot_x = 0,
            FT rot_y = 0,
            FT rot_z = 0)
      : Camera<FT>(
            focal_x, focal_y, offset_x, offset_y, width, height, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_);
  }

  CameraHom(FT fov,
            const Vec2<FT>& imageSize,
            const Vec3<FT>& trans = Vec3<FT>(0, 0, 0),
            const Vec3<FT>& rot = Vec3<FT>(0, 0, 0))
      : Camera<FT>(fov, imageSize, trans, rot) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_);
  }

  CameraHom(const Vec2<FT>& focal,
            const Vec2<FT>& offset,
            const Vec2<FT>& imageSize = Vec2<FT>(0, 0),
            const Vec3<FT>& trans = Vec3<FT>(0, 0, 0),
            const Vec3<FT>& rot = Vec3<FT>(0, 0, 0))
      : Camera<FT>(focal, offset, imageSize, trans, rot) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_);
  }

  CameraHom(const Vec2<FT>& focal,
            const Vec2<FT>& offset,
            const Vec2<FT>& imageSize,
            const Vec3<FT>& trans,
            const Matx33<FT>& rot)
      : Camera<FT>(focal, offset, imageSize, trans, rot) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot);
  }

  CameraHom(const Matx33<FT>& cam,
            const Vec3<FT>& trans,
            const Vec3<FT>& rot = Vec3<FT>(0, 0, 0),
            const Vec2<FT>& imageSize = Vec2<FT>(0, 0))
      : Camera<FT>(cam, trans, rot, imageSize) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_);
  }

  CameraHom(const Matx33<FT>& cam,
            const Vec3<FT>& trans,
            const Matx33<FT>& rot,
            const Vec2<FT>& imageSize = Vec2<FT>(0, 0))
      : Camera<FT>(cam, trans, rot, imageSize) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot);
  }

  CameraHom(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(0, 0))
      : Camera<FT>(proj, imageSize), proj_(proj) {}

  CameraHom(const Camera<FT>& cam) : Camera<FT>(cam) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_);
  }

  //! overwrite projM
  virtual Matx34<FT> projM() const { return proj_; }

  ///////////////////////////////// projection methods

  //! project homogeneous point
  inline Vec3<FT> projectH(const Vec4<FT>& h) const { return proj_ * h; }

  //! project homogeneous point mat (points as rows), results in mat with points as rows
  inline void projectHRow(const Eigen::Matrix<FT, Eigen::Dynamic, 4>& points,
                          Eigen::Matrix<FT, Eigen::Dynamic, 3>& res) const {
    res = points * proj_.transpose();
  }

  //! project homogeneous point mat (points as cols), results in mat with points as cols
  inline void projectHCol(const Eigen::Matrix<FT, 4, Eigen::Dynamic>& points,
                          Eigen::Matrix<FT, 3, Eigen::Dynamic>& res) const {
    res = proj_ * points;
  }

  //! project homogeneous point mat (points as rows), results in mat with points as rows
  inline void projectHRowMap(const Eigen::Map<const Eigen::Matrix<FT, Eigen::Dynamic, 4>> points,
                             Eigen::Map<Eigen::Matrix<FT, Eigen::Dynamic, 3>> res) const {
    res = points * proj_.transpose();
  }

  //! project homogeneous point mat (points as cols), results in mat with points as cols
  inline void projectHColMap(const Eigen::Map<const Eigen::Matrix<FT, 4, Eigen::Dynamic>> points,
                             Eigen::Map<Eigen::Matrix<FT, 3, Eigen::Dynamic>> res) const {
    res = proj_ * points;
  }

  //! project homogeneous point vector
  template <template <class, class...> class V, class... V1Args, class... V2Args>
  inline void projectH(const V<Vec4<FT>, V1Args...>& vh, V<Vec3<FT>, V2Args...>& ret) const {
    /*ret.clear();
    ret.reserve(vh.size());
    for_each(vh.begin(),vh.end(),[&](const Vec4<FT>& v){
        ret.push_back(projectH(v));
    });*/
    ret.resize(vh.size());
    projectHRowMap(Eigen::Map<const Eigen::Matrix<FT, Eigen::Dynamic, 4>>(vh[0][0], vh.size(), 4),
                   Eigen::Map<Eigen::Matrix<FT, Eigen::Dynamic, 3>>(ret[0][0], ret.size(), 3));
  }


  //! project homogeneous point mat (points as rows), results in mat with points as rows
  inline void projectRow(const Eigen::Matrix<FT, Eigen::Dynamic, 3>& points,
                         Eigen::Matrix<FT, Eigen::Dynamic, 2>& res) const {
    res = (points.rowwise().homogeneous() * proj_.transpose()).rowwise().hnormalized();
  }

  //! project homogeneous point mat (points as cols), results in mat with points as cols
  inline void projectCol(const Eigen::Matrix<FT, 3, Eigen::Dynamic>& points,
                         Eigen::Matrix<FT, 2, Eigen::Dynamic>& res) const {
    res = (proj_ * points.colwise().homogeneous()).colwise().hnormalized();
  }

  //! project homogeneous point mat (points as rows), results in mat with points as rows
  inline void projectRowMap(const Eigen::Map<const Eigen::Matrix<FT, Eigen::Dynamic, 3>> points,
                            Eigen::Map<Eigen::Matrix<FT, Eigen::Dynamic, 2>> res) const {
    res = (points.rowwise().homogeneous() * proj_.transpose()).rowwise().hnormalized();
  }

  //! project homogeneous point mat (points as cols), results in mat with points as cols
  inline void projectColMap(const Eigen::Map<const Eigen::Matrix<FT, 3, Eigen::Dynamic>> points,
                            Eigen::Map<Eigen::Matrix<FT, 2, Eigen::Dynamic>> res) const {
    res = (proj_ * points.colwise().homogeneous()).colwise().hnormalized();
  }


  //! project 3d point
  inline Vec2<FT> project(const Vec3<FT>& h) const {
    return projectH(h.homogeneous()).hnormalized();
    // Vec3<FT> tmp = projectH(h.homogeneous());
    // Vec2<FT> tmp2 = Vec2<FT>(tmp[0]/tmp[2],tmp[1]/tmp[2]);
    // return detail::abs(tmp[2]) < LIMITS<FT>::TAU ? Vec2<FT>(FT(0),FT(0)) : Vec2<FT>(tmp[0]/tmp[2],tmp[1]/tmp[2]);
  }

  //! project point vector
  template <template <class, class...> class V1, class... V1Args, template <class, class...> class V2, class... V2Args>
  inline void project(const V1<Vec3<FT>, V1Args...>& vh, V2<Vec2<FT>, V2Args...>& ret) const {
    ret.resize(vh.size());

    if (vh.size() > 0) {
      // Create temporary matrices with correct layout
      Eigen::Matrix<FT, Eigen::Dynamic, 3> pts_matrix(vh.size(), 3);
      for (size_t i = 0; i < vh.size(); ++i) {
        pts_matrix.row(i) << vh[i].x(), vh[i].y(), vh[i].z();
      }

      Eigen::Matrix<FT, Eigen::Dynamic, 2> res_matrix(vh.size(), 2);

      // Use the optimized matrix projection
      res_matrix = (pts_matrix.rowwise().homogeneous() * proj_.transpose()).rowwise().hnormalized();

      // Copy back to result vector
      for (size_t i = 0; i < vh.size(); ++i) {
        ret[i] = Vec2<FT>(res_matrix(i, 0), res_matrix(i, 1));
      }
    }
  }
};

typedef CameraHom<float> CameraHomf;
typedef CameraHom<double> CameraHomd;


//! camera with pluecker (lines) and homogeneous (points) projection
template <class FT>
class CameraPluecker : public CameraHom<FT> {
 protected:
  Matx33<FT> rotMt_;
  Matx33<FT> camMCOF_;
  using CameraHom<FT>::trans_;
  using CameraHom<FT>::rot_;
  using CameraHom<FT>::focal_;
  using CameraHom<FT>::offset_;
  using CameraHom<FT>::proj_;

  virtual void updateInternals() {
    CameraHom<FT>::updateInternals();
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

 public:
  typedef FT float_type;

  CameraPluecker(FT focal_x = FT(0),
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
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  CameraPluecker(FT fov,
                 const Vec2<FT>& imageSize,
                 const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
                 const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(fov, imageSize, trans, rot) {
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  CameraPluecker(const Vec2<FT>& focal,
                 const Vec2<FT>& offset,
                 const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)),
                 const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
                 const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(focal, offset, imageSize, trans, rot) {
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  CameraPluecker(const Vec2<FT>& focal,
                 const Vec2<FT>& offset,
                 const Vec2<FT>& imageSize,
                 const Vec3<FT>& trans,
                 const Matx33<FT>& rot)
      : CameraHom<FT>(focal, offset, imageSize, trans, rot), rotMt_(rot.transpose()) {
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  CameraPluecker(const Matx33<FT>& cam,
                 const Vec3<FT>& trans,
                 const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)),
                 const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize) {
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  CameraPluecker(const Matx33<FT>& cam,
                 const Vec3<FT>& trans,
                 const Matx33<FT>& rot,
                 const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize), rotMt_(rot.t()) {
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  CameraPluecker(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(proj, imageSize) {
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  CameraPluecker(const Camera<FT>& cam) : CameraHom<FT>(cam) {
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  //! overwrite camMCOF
  virtual Matx33<FT> camMCOF() const { return camMCOF_; }

  ///////////////////////////////// projection methods
  using CameraHom<FT>::project;

  //! project 3d line to 2d line
  inline Line<FT> project(const Line3<FT>& line) const {
    Vec3<FT> tmp = (line.origin() - trans_).cross(line.direction());
    tmp = camMCOF_ * rotMt_ * tmp;

    FT xyNorm = std::sqrt(tmp.x() * tmp.x() + tmp.y() * tmp.y());
    if (xyNorm < LIMITS<FT>::tau()) return Line<FT>();

    return Line<FT>(tmp.x() / xyNorm, tmp.y() / xyNorm, -tmp.z() / xyNorm);
    // return projectPlueckerCof(camMCOF(),trans_,this->rotM(),line.momentum(),line.direction());
  }

  //! project 3d line vector to 2d line vector
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<Line3<FT>, V1Args...>& vl3, LV& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) { ret.push_back(project(l)); });
  }

  //! project 3d line (subclass) vector to 2d line vector
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<Line<FT>>& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) { ret.push_back(project(l)); });
  }

  //! project 3d line segment to 2d line segment
  inline LineSegment<FT> project(const LineSegment3<FT>& line) const {
    Line<FT> l = project(static_cast<const Line3<FT>>(line));
    if (l.empty()) return LineSegment<FT>();
    return LineSegment<FT>(l, project(line.startPoint()), project(line.endPoint()));
  }

  //! project 3d line segment vector to 2d segment line vector
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<LineSegment3<FT>, V1Args...>& vl3, LV& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) { ret.push_back(project(l)); });
  }

  //! project 3d line segment (subclass) vector to 2d line segment vector
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<LineSegment<FT>>& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) { ret.push_back(project(l)); });
  }

  ///////////////////////////////////// static helpers

  static inline Line2<FT> projectLine(const Matx34<FT> proj, const Line3<FT>& l) {
    CameraPluecker<FT> cam(proj);
    return cam.project(l);
  }

  static inline LineSegment2<FT> projectLineSegment(const Matx34<FT> proj, const LineSegment3<FT>& ls) {
    CameraPluecker<FT> cam(proj);
    return cam.project(ls);
  }

  //! Project Pluecker line, with trans being the translation vector, p being the point instead of the momentum m and l
  //! is the normed direction vector
  static inline Line<FT> projectPlueckerP(
      const Matx33<FT>& cam, const Vec3<FT>& trans, const Matx33<FT>& rot, const Vec3<FT>& p, const Vec3<FT>& l) {
    return projectPlueckerM(cam, trans, rot, p.cross(l), l);
  }

  //! Project Pluecker line, with c being the translation vector, m being the momentum and l is the normed direction
  //! vector K - camera matrix, R - rotation matrix
  static inline Line<FT> projectPlueckerM(
      const Matx33<FT>& cam, const Vec3<FT>& trans, const Matx33<FT>& rot, const Vec3<FT>& m, const Vec3<FT>& l) {
    return projectPlueckerCof(Camera<FT>::composeCameraMatrixCOF(cam), trans, rot, m, l);
  }

  //! Project Pluecker line, with c being the translation vector, m being the momentum and l is the normed direction
  //! vector
  static inline Line<FT> projectPlueckerCof(
      const Matx33<FT>& camCOF, const Vec3<FT>& trans, const Matx33<FT>& rot, const Vec3<FT>& m, const Vec3<FT>& l) {
    /*if (sizeof(FT) == 8)
    std::cout << "camCof:" << std::endl << camCOF[0] << " " << camCOF[1] << " " << camCOF[2] << std::endl
                                        << camCOF[3] << " " << camCOF[4] << " " << camCOF[5] << std::endl
                                        << camCOF[6] << " " << camCOF[7] << " " << camCOF[8] << std::endl <<
                 "rot:" << std::endl << rot[0] << " " << rot[1] << " " << rot[2] << std::endl
                                     << rot[3] << " " << rot[4] << " " << rot[5] << std::endl
                                     << rot[6] << " " << rot[7] << " " << rot[8] << std::endl <<
                 "trans:" << trans[0] << " " << trans[1] << " " << trans[2] << std::endl <<
                 "m:" << m[0] << " " << m[1] << " " << m[2] << std::endl <<
                 "l:" << l[0] << " " << l[1] << " " << l[2] << std::endl << std::endl;
    */
    Vec3<FT> tmp = m - (trans.cross(l));
    tmp = camCOF * rot.transpose() * tmp;

    FT xyNorm = detail::sqrt(tmp.x() * tmp.x() + tmp.y() * tmp.y());
    if (xyNorm < LIMITS<FT>::tau()) return Line<FT>();

    return Line<FT>(tmp.x() / xyNorm, tmp.y() / xyNorm, -tmp.z() / xyNorm);
  }


  //! Project Cayley line
  static inline Line<FT> projectCayley(const Matx33<FT>& cam,
                                       const Vec3<FT>& trans,
                                       const Matx33<FT>& rot,
                                       const Vec4<FT>& cayley) {
    return projectCayleyCOF(Camera<FT>::composeCameraMatrixCOF(cam), trans, rot, cayley);
  }

  //! Project Cayley line with COF cam
  static inline Line<FT> projectCayleyCof(const Matx33<FT>& camCOF,
                                          const Vec3<FT>& trans,
                                          const Matx33<FT>& rot,
                                          const Vec4<FT>& cayley) {
    Vec3<FT> m, l;
    Line3<FT>::plueckerCoordinatesFromCayley(cayley, m, l);
    return projectPlueckerCof(camCOF, trans, rot, m, l);
  }
};

typedef CameraPluecker<float> CameraPlueckerf;
typedef CameraPluecker<double> CameraPlueckerd;


//! camera 2P uses homogeneous point projection and for lines 2 point projection
template <class FT>
class Camera2P : public CameraHom<FT> {
 protected:
  using CameraHom<FT>::trans_;
  using CameraHom<FT>::rot_;
  using CameraHom<FT>::focal_;
  using CameraHom<FT>::offset_;
  using CameraHom<FT>::proj_;

  virtual void updateInternals() { CameraHom<FT>::updateInternals(); }

 public:
  typedef FT float_type;

  Camera2P(FT focal_x = FT(0),
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
            focal_x, focal_y, offset_x, offset_y, width, height, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z) {}

  Camera2P(FT fov,
           const Vec2<FT>& imageSize,
           const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(fov, imageSize, trans, rot) {}

  Camera2P(const Vec2<FT>& focal,
           const Vec2<FT>& offset,
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)),
           const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(focal, offset, imageSize, trans, rot) {}

  Camera2P(const Vec2<FT>& focal,
           const Vec2<FT>& offset,
           const Vec2<FT>& imageSize,
           const Vec3<FT>& trans,
           const Matx33<FT>& rot)
      : CameraHom<FT>(focal, offset, imageSize, trans, rot) {}

  Camera2P(const Matx33<FT>& cam,
           const Vec3<FT>& trans,
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize) {}

  Camera2P(const Matx33<FT>& cam,
           const Vec3<FT>& trans,
           const Matx33<FT>& rot,
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize) {}

  Camera2P(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(proj, imageSize) {}

  Camera2P(const Camera<FT>& cam) : CameraHom<FT>(cam) {}

  ///////////////////////////////// projection methods
  using CameraHom<FT>::project;


  //! project 3d line to 2d line as two points
  inline Line<FT> project(const Line3<FT>& line) const {
    return Line<FT>(project(line.origin()), project(line.distanceOrigin(1000)));
  }

  //! project 3d line vector to 2d line vector as two points
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<Line3<FT>, V1Args...>& vl3, LV& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) { ret.push_back(project(l)); });
  }

  //! project 3d line (subclass) vector to 2d line vector as two points
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<Line<FT>>& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) { ret.push_back(project(l)); });
  }

  //! project 3d line segment to 2d line segment as two points
  inline LineSegment<FT> project(const LineSegment3<FT>& line) const {
    return LineSegment<FT>(project(line.startPoint()), project(line.endPoint()));
  }

  //! project 3d line segment vector to 2d segment line vector
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<LineSegment3<FT>, V1Args...>& vl3, LV& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) { ret.push_back(project(l)); });
  }

  //! project 3d line segment (subclass) vector to 2d line segment vector
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<LineSegment<FT>>& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) { ret.push_back(project(l)); });
  }
};

typedef Camera2P<float> Camera2Pf;
typedef Camera2P<double> Camera2Pd;
}  // namespace lsfm
