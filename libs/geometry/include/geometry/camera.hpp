//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann & Manuel Lange
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file camera.hpp
/// @brief Camera models for 3D-to-2D projection using Eigen-based types.
///
/// This file provides camera classes that model the projection from 3D world
/// coordinates to 2D image coordinates using pinhole camera models. The cameras
/// combine intrinsic parameters (focal length, principal point) with extrinsic
/// parameters (position and orientation via Pose).
/// Main classes:
/// - Camera: Base pinhole camera with no distortion
/// - CameraLens: Pinhole camera with polynomial lens distortion
/// The cameras support:
/// - Forward projection (world to image)
/// - Inverse projection (image to ray)
/// - Transformation of points, lines, and line segments
/// - Field of view calculations

#pragma once

#include <geometry/line3.hpp>
#include <geometry/pose.hpp>
#include <geometry/vision.hpp>

namespace lsfm {

/// @brief Pinhole camera model without lens distortion.
///
/// Models a camera as the combination of intrinsic parameters (focal length,
/// principal point, image size) and extrinsic parameters (pose in world).
/// Inherits from Pose to provide position and orientation.
/// The camera uses the standard pinhole model:
/// - x_img = fx * (X_cam / Z_cam) + cx
/// - y_img = fy * (Y_cam / Z_cam) + cy
/// @tparam FT Floating-point type (float or double).
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

  /// @brief Construct camera from individual parameters.
  /// @param focal_x Focal length in x (pixels).
  /// @param focal_y Focal length in y (pixels).
  /// @param offset_x Principal point x (pixels).
  /// @param offset_y Principal point y (pixels).
  /// @param width Image width (0 = auto from offset).
  /// @param height Image height (0 = auto from offset).
  /// @param trans_x Camera position x.
  /// @param trans_y Camera position y.
  /// @param trans_z Camera position z.
  /// @param rot_x Rotation angle x (radians).
  /// @param rot_y Rotation angle y (radians).
  /// @param rot_z Rotation angle z (radians).
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

  /// @brief Construct camera from field of view and image size.
  ///
  /// Creates a camera with the principal point at the image center.
  /// @param fov Horizontal field of view (radians).
  /// @param imageSize Image dimensions (width, height).
  /// @param trans Camera position (world coordinates).
  /// @param rot Camera rotation (Rodrigues vector).
  Camera(FT fov,
         const Vec2<FT>& imageSize,
         const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
         const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : Pose<FT>(trans, rot),
        focal_(fov2Focal(fov, imageSize.x()), fov2Focal(fov, imageSize.x())),
        offset_(imageSize * FT(0.5)),
        imageSize_(imageSize) {}

  /// @brief Construct camera from intrinsic and extrinsic vectors.
  /// @param focal Focal length (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @param imageSize Image dimensions (width, height).
  /// @param trans Camera position.
  /// @param rot Camera rotation (Rodrigues vector).
  Camera(const Vec2<FT>& focal,
         const Vec2<FT>& offset,
         const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)),
         const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
         const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : Pose<FT>(trans, rot), focal_(focal), offset_(offset), imageSize_(imageSize) {
    initSize();
  }

  /// @brief Construct camera from intrinsic vectors and rotation matrix.
  /// @param focal Focal length (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @param imageSize Image dimensions (width, height).
  /// @param trans Camera position.
  /// @param rot Camera rotation matrix.
  Camera(const Vec2<FT>& focal,
         const Vec2<FT>& offset,
         const Vec2<FT>& imageSize,
         const Vec3<FT>& trans,
         const Matx33<FT>& rot)
      : Pose<FT>(trans, rot), focal_(focal), offset_(offset), imageSize_(imageSize) {
    initSize();
  }

  /// @brief Construct camera from camera matrix and pose.
  /// @param cam 3x3 intrinsic camera matrix.
  /// @param trans Camera position.
  /// @param rot Camera rotation (Rodrigues vector).
  /// @param imageSize Image dimensions.
  Camera(const Matx33<FT>& cam,
         const Vec3<FT>& trans,
         const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)),
         const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : Pose<FT>(trans, rot), imageSize_(imageSize) {
    decomposeCameraMatrix(cam, focal_, offset_);
    initSize();
  }

  /// @brief Construct camera from camera matrix and rotation matrix.
  /// @param cam 3x3 intrinsic camera matrix.
  /// @param trans Camera position.
  /// @param rot Camera rotation matrix.
  /// @param imageSize Image dimensions.
  Camera(const Matx33<FT>& cam,
         const Vec3<FT>& trans,
         const Matx33<FT>& rot,
         const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : Pose<FT>(trans, rot), imageSize_(imageSize) {
    decomposeCameraMatrix(cam, focal_, offset_);
    initSize();
  }

  /// @brief Construct camera from 3x4 projection matrix.
  ///
  /// Decomposes P = K[R|t] to extract intrinsics and extrinsics.
  /// @param proj 3x4 projection matrix.
  /// @param imageSize Image dimensions.
  /// @note Be careful with imageSize - initSize() just multiplies origin by two.
  Camera(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0))) : imageSize_(imageSize) {
    decomposeProjectionMatrix(proj, focal_, offset_, trans_, rot_);
    initSize();
  }

  /// @brief Copy constructor.
  /// @param cam Camera to copy.
  Camera(const Camera<FT>& cam)
      : Pose<FT>(cam.trans_, cam.rot_), focal_(cam.focal_), offset_(cam.offset_), imageSize_(cam.imageSize_) {}

  /// @brief Assignment operator.
  /// @param cam Camera to assign from.
  /// @return Reference to this camera.
  Camera& operator=(const Camera<FT>& cam) {
    if (this != &cam) {
      this->pose(cam);
      focal_ = cam.focal_;
      offset_ = cam.offset_;
      imageSize_ = cam.imageSize_;
    }
    return *this;
  }

  /// @name Accessors
  /// @{

  /// @brief Test if camera is empty (no focal length).
  /// @return True if both focal lengths are zero.
  virtual bool empty() const { return (focal_.x() == 0 && focal_.y() == 0); }

  /// @brief Get focal length vector (fx, fy).
  inline const Vec2<FT>& focal() const { return focal_; }

  /// @brief Get principal point (cx, cy).
  inline const Vec2<FT>& offset() const { return offset_; }

  /// @brief Get composed focal length (sqrt(fx*fy)).
  virtual const FT focalLength() const { return composeFocalLength(focal_); }

  /// @brief Get field of view in radians.
  virtual Vec2<FT> fov() const { return focal2Fov(focal_, offset_); }

  /// @brief Get image width.
  virtual FT width() const { return imageSize_.x(); }

  /// @brief Get image height.
  virtual FT height() const { return imageSize_.y(); }

  /// @brief Get image dimensions (width, height).
  inline Vec2<FT> imageSize() const { return imageSize_; }

  /// @brief Get image aspect ratio (width/height).
  virtual FT aspectRatio() const { return composeAspectRatio(imageSize_); }

  /// @brief Get 3x3 camera intrinsic matrix K.
  virtual Matx33<FT> camM() const { return composeCameraMatrix(focal_, offset_); }

  /// @brief Get camera matrix cofactor (inverse transpose without scaling).
  virtual Matx33<FT> camMCOF() const { return composeCameraMatrixCOF(focal_, offset_); }

  /// @brief Get 3x4 projection matrix P = K[R|t].
  virtual Matx34<FT> projM() const { return composeProjectionMatrix(focal_, offset_, trans_, rot_); }

  /// @brief Get 4x4 homogeneous projection matrix.
  virtual Matx44<FT> projMH() const { return composeProjectionMatrixH(focal_, offset_, trans_, rot_); }

  /// @brief Set image size.
  /// @param width Image width.
  /// @param height Image height.
  void imageSize(const int width, const int height) {
    imageSize_.x() = width;
    imageSize_.y() = height;
  }

  /// @}

  /// @name Static Helpers
  /// Utility functions for camera parameter conversions.
  /// @{

  /// @brief Compose focal vector from focal length and aspect ratio.
  /// @param focalLength Focal length.
  /// @param aspectRatio Aspect ratio (x/y).
  /// @return Focal vector (fx, fy).
  static inline Vec2<FT> composeFocal(FT focalLength, FT aspectRatio = 1) {
    return Vec2<FT>(focalLength * aspectRatio, focalLength);
  }

  /// @brief Compose focal vector from focal length and pixel size.
  /// @param focalLength Focal length in world units.
  /// @param pixelSize Pixel size (width, height) in world units.
  /// @return Focal vector (fx, fy).
  static inline Vec2<FT> composeFocal(FT focalLength, const Vec2<FT>& pixelSize) {
    return Vec2<FT>(focalLength * pixelSize.x(), focalLength * pixelSize.y());
  }

  /// @brief Extract focal length from focal vector.
  /// @param focal Focal vector (fx, fy).
  /// @return Focal length (fy).
  static inline FT composeFocalLength(const Vec2<FT>& focal) { return focal.y(); }

  /// @brief Extract focal length considering pixel size.
  /// @param focal Focal vector (fx, fy).
  /// @param pixelSize Pixel size.
  /// @return Focal length in world units.
  static inline FT composeFocalLength(const Vec2<FT>& focal, const Vec2<FT>& pixelSize) {
    return focal.x() / pixelSize.x();
  }

  /// @brief Compute aspect ratio from size.
  /// @param size Image size (width, height).
  /// @return Aspect ratio (width/height).
  static inline FT composeAspectRatio(const Vec2<FT>& size) { return size.x() / size.y(); }

  /// @brief Convert field of view to focal length.
  /// @param fov Field of view (radians).
  /// @param size Image dimension (width or height).
  /// @return Focal length in pixels.
  static inline FT fov2Focal(FT fov, FT size) { return size / (2 * detail::tan(fov / 2)); }

  /// @brief Convert field of view vector to focal vector.
  /// @param fov Field of view (x, y) in radians.
  /// @param size Image size (width, height).
  /// @return Focal vector (fx, fy).
  static inline Vec2<FT> fov2Focal(const Vec2<FT>& fov, const Vec2<FT>& size) {
    return Vec2<FT>(size.x() / (2 * detail::tan(fov.x() / 2)), size.y() / (2 * detail::tan(fov.y() / 2)));
  }

  /// @brief Convert focal length to field of view.
  /// @param focal Focal length.
  /// @param offset Principal point offset.
  /// @return Field of view (radians).
  static inline FT focal2Fov(FT focal, FT offset) { return 2 * detail::atan(offset / (2 * focal)); }

  /// @brief Convert focal vector to field of view vector.
  /// @param focal Focal vector (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @return Field of view (x, y) in radians.
  static inline Vec2<FT> focal2Fov(const Vec2<FT>& focal, const Vec2<FT>& offset) {
    return Vec2<FT>(2 * detail::atan(detail::abs(offset.x()) / focal.x()),
                    2 * detail::atan(detail::abs(offset.y()) / focal.y()));
  }

  /// @brief Compose 3x3 camera intrinsic matrix.
  /// @param focal Focal vector (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @return 3x3 camera matrix K.
  static inline Matx33<FT> composeCameraMatrix(const Vec2<FT>& focal, const Vec2<FT>& offset) {
    return Matx<FT, 3, 3>(focal.x(), FT(0), offset.x(), FT(0), focal.y(), offset.y(), FT(0), FT(0), FT(1));
  }

  /// @brief Compose camera matrix cofactor.
  /// @param focal Focal vector (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @return Camera matrix cofactor.
  static inline Matx33<FT> composeCameraMatrixCOF(const Vec2<FT>& focal, const Vec2<FT>& offset) {
    return Matx<FT, 3, 3>(focal.y(), FT(0), FT(0), FT(0), focal.x(), FT(0), -focal.y() * offset.x(),
                          -focal.x() * offset.y(), focal.x() * focal.y());
  }

  /// @brief Compose camera matrix cofactor from camera matrix.
  /// @param cam 3x3 camera matrix.
  /// @return Camera matrix cofactor.
  static inline Matx33<FT> composeCameraMatrixCOF(const Matx33<FT>& cam) {
    return Matx33<FT>(cam(1, 1), FT(0), FT(0), FT(0), cam(0, 0), FT(0), -cam(1, 1) * cam(0, 2), -cam(0, 0) * cam(1, 2),
                      cam(0, 0) * cam(1, 1));
  }

  /// @brief Compose 3x4 projection matrix from intrinsics and Rodrigues rotation.
  /// @param focal Focal vector.
  /// @param offset Principal point.
  /// @param trans Translation.
  /// @param rot Rodrigues rotation vector.
  /// @return 3x4 projection matrix.
  static inline Matx34<FT> composeProjectionMatrix(const Vec2<FT>& focal,
                                                   const Vec2<FT>& offset,
                                                   const Vec3<FT>& trans,
                                                   const Vec3<FT>& rot) {
    return composeProjectionMatrix(composeCameraMatrix(focal, offset), trans, rot);
  }

  /// @brief Compose 3x4 projection matrix from intrinsics and rotation matrix.
  /// @param focal Focal vector.
  /// @param offset Principal point.
  /// @param trans Translation.
  /// @param rot Rotation matrix.
  /// @return 3x4 projection matrix.
  static inline Matx34<FT> composeProjectionMatrix(const Vec2<FT>& focal,
                                                   const Vec2<FT>& offset,
                                                   const Vec3<FT>& trans,
                                                   const Matx33<FT>& rot) {
    return composeProjectionMatrix(composeCameraMatrix(focal, offset), trans, rot);
  }

  /// @brief Compose 3x4 projection matrix from camera matrix and Rodrigues rotation.
  /// @param cam Camera intrinsic matrix.
  /// @param trans Translation.
  /// @param rot Rodrigues rotation vector.
  /// @return 3x4 projection matrix.
  static inline Matx34<FT> composeProjectionMatrix(const Matx33<FT>& cam, const Vec3<FT>& trans, const Vec3<FT>& rot) {
    return composeProjectionMatrix(cam, trans, rodrigues(rot));
  }

  /// @brief Compose 3x4 projection matrix from camera matrix and rotation matrix.
  /// @param cam Camera intrinsic matrix.
  /// @param trans Translation.
  /// @param rot Rotation matrix.
  /// @return 3x4 projection matrix P = K[R|t].
  static inline Matx34<FT> composeProjectionMatrix(const Matx33<FT>& cam,
                                                   const Vec3<FT>& trans,
                                                   const Matx33<FT>& rot) {
    // Matx44<FT> camH = composeProjectionMatrixH(cam,trans,rot);
    // return Matx34<FT>(camH(0,0),camH(0,1),camH(0,2),camH(0,3),
    // camH(1,0),camH(1,1),camH(1,2),camH(1,3),
    // camH(2,0),camH(2,1),camH(2,2),camH(2,3));
    return Matx34<FT>(composeProjectionMatrixH(cam, trans, rot).data());
  }

  /// @brief Compose 4x4 homogeneous projection matrix.
  /// @param focal Focal vector.
  /// @param offset Principal point.
  /// @param trans Translation.
  /// @param rot Rodrigues rotation vector.
  /// @return 4x4 homogeneous projection matrix.
  static inline Matx44<FT> composeProjectionMatrixH(const Vec2<FT>& focal,
                                                    const Vec2<FT>& offset,
                                                    const Vec3<FT>& trans,
                                                    const Vec3<FT>& rot) {
    return composeProjectionMatrixH(composeCameraMatrix(focal, offset), trans, rot);
  }

  /// @brief Compose 4x4 homogeneous projection matrix.
  /// @param focal Focal vector.
  /// @param offset Principal point.
  /// @param trans Translation.
  /// @param rot Rotation matrix.
  /// @return 4x4 homogeneous projection matrix.
  static inline Matx44<FT> composeProjectionMatrixH(const Vec2<FT>& focal,
                                                    const Vec2<FT>& offset,
                                                    const Vec3<FT>& trans,
                                                    const Matx33<FT>& rot) {
    return composeProjectionMatrixH(composeCameraMatrix(focal, offset), trans, rot);
  }

  /// @brief Compose 4x4 homogeneous projection matrix.
  /// @param cam Camera intrinsic matrix.
  /// @param trans Translation.
  /// @param rot Rodrigues rotation vector.
  /// @return 4x4 homogeneous projection matrix.
  static inline Matx44<FT> composeProjectionMatrixH(const Matx33<FT>& cam, const Vec3<FT>& trans, const Vec3<FT>& rot) {
    return composeProjectionMatrixH(cam, trans, rodrigues(rot));
  }

  /// @brief Compose 4x4 homogeneous projection matrix.
  /// @param cam Camera intrinsic matrix.
  /// @param trans Translation.
  /// @param rot Rotation matrix.
  /// @return 4x4 homogeneous projection matrix.
  static inline Matx44<FT> composeProjectionMatrixH(const Matx33<FT>& cam,
                                                    const Vec3<FT>& trans,
                                                    const Matx33<FT>& rot) {
    Matx33<FT> R = rot.transpose();
    Vec3<FT> t = -R * trans;

    Matx44<FT> E(composeHom(t, R));
    Matx44<FT> P(composeHom(cam));

    return P * E;
  }

  /// @brief Decompose camera matrix to focal and offset.
  /// @param cam 3x3 camera matrix.
  /// @param[out] focal Focal vector.
  /// @param[out] offset Principal point.
  static inline void decomposeCameraMatrix(const Matx33<FT>& cam, Vec2<FT>& focal, Vec2<FT>& offset) {
    FT n = cam(2, 2);
    focal = Vec2<FT>(cam(0, 0) / n, cam(1, 1) / n);
    offset = Vec2<FT>(cam(0, 2) / n, cam(1, 2) / n);
  }

  /// @brief Decompose projection matrix to camera matrix and pose.
  /// @param proj 3x4 projection matrix.
  /// @param[out] cam 3x3 camera matrix.
  /// @param[out] trans Translation.
  /// @param[out] rot Rotation matrix.
  static inline void decomposeProjectionMatrix(const Matx34<FT>& proj,
                                               Matx33<FT>& cam,
                                               Vec3<FT>& trans,
                                               Matx33<FT>& rot) {
    lsfm::decomposeProjectionMatrix(proj, cam, trans, rot);
  }

  /// @brief Decompose projection matrix to camera matrix and Rodrigues rotation.
  /// @param proj 3x4 projection matrix.
  /// @param[out] cam 3x3 camera matrix.
  /// @param[out] trans Translation.
  /// @param[out] rot Rodrigues rotation vector.
  static inline void decomposeProjectionMatrix(const Matx34<FT>& proj,
                                               Matx33<FT>& cam,
                                               Vec3<FT>& trans,
                                               Vec3<FT>& rot) {
    Matx33<FT> rTmp;
    decomposeProjectionMatrix(proj, cam, trans, rTmp);
    rot = rodrigues(rTmp);
  }

  /// @brief Decompose projection matrix to focal, offset, and pose.
  /// @param proj 3x4 projection matrix.
  /// @param[out] focal Focal vector.
  /// @param[out] offset Principal point.
  /// @param[out] trans Translation.
  /// @param[out] rot Rotation matrix.
  static inline void decomposeProjectionMatrix(
      const Matx34<FT>& proj, Vec2<FT>& focal, Vec2<FT>& offset, Vec3<FT>& trans, Matx33<FT>& rot) {
    Matx33<FT> tmp;
    decomposeProjectionMatrix(proj, tmp, trans, rot);
    decomposeCameraMatrix(tmp, focal, offset);
  }

  /// @brief Decompose projection matrix to focal, offset, and Rodrigues rotation.
  /// @param proj 3x4 projection matrix.
  /// @param[out] focal Focal vector.
  /// @param[out] offset Principal point.
  /// @param[out] trans Translation.
  /// @param[out] rot Rodrigues rotation vector.
  static inline void decomposeProjectionMatrix(
      const Matx34<FT>& proj, Vec2<FT>& focal, Vec2<FT>& offset, Vec3<FT>& trans, Vec3<FT>& rot) {
    Matx33<FT> tmp;
    decomposeProjectionMatrix(proj, tmp, trans, rot);
    decomposeCameraMatrix(tmp, focal, offset);
  }

  /// @brief Project 3D point to 2D image coordinates.
  /// @param projM 3x4 projection matrix.
  /// @param h 3D point.
  /// @param[out] ret 2D projected point.
  /// @return True if projection successful (point not at infinity).
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

  /// @}

  /// @brief Virtual destructor for proper inheritance.
  virtual ~Camera() = default;
};

/// @brief Transform camera by 4x4 homogeneous matrix.
/// @tparam FT Floating-point type.
/// @param m Transformation matrix.
/// @param cam Camera to transform.
/// @return Transformed camera.
template <class FT>
inline Camera<FT> operator*(const Matx44<FT>& m, const Camera<FT>& cam) {
  Camera<FT> c = cam;
  c.transform(m);
  return c;
}

/// @brief Single-precision camera.
typedef Camera<float> Cameraf;

/// @brief Double-precision camera.
typedef Camera<double> Camerad;

/// @brief Camera with cached homogeneous projection matrix.
///
/// Extends Camera with pre-computed projection matrix for efficient
/// point projection. Does not support line transformations.
/// @tparam FT Floating-point type.
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

  /// @brief Default/parameter constructor.
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

  /// @brief Construct from FOV and image size.
  CameraHom(FT fov,
            const Vec2<FT>& imageSize,
            const Vec3<FT>& trans = Vec3<FT>(0, 0, 0),
            const Vec3<FT>& rot = Vec3<FT>(0, 0, 0))
      : Camera<FT>(fov, imageSize, trans, rot),
        proj_(Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_)) {}

  /// @brief Construct from intrinsics and Rodrigues rotation.
  CameraHom(const Vec2<FT>& focal,
            const Vec2<FT>& offset,
            const Vec2<FT>& imageSize = Vec2<FT>(0, 0),
            const Vec3<FT>& trans = Vec3<FT>(0, 0, 0),
            const Vec3<FT>& rot = Vec3<FT>(0, 0, 0))
      : Camera<FT>(focal, offset, imageSize, trans, rot),
        proj_(Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_)) {}

  /// @brief Construct from intrinsics and rotation matrix.
  CameraHom(const Vec2<FT>& focal,
            const Vec2<FT>& offset,
            const Vec2<FT>& imageSize,
            const Vec3<FT>& trans,
            const Matx33<FT>& rot)
      : Camera<FT>(focal, offset, imageSize, trans, rot),
        proj_(Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot)) {}

  /// @brief Construct from camera matrix and Rodrigues rotation.
  CameraHom(const Matx33<FT>& cam,
            const Vec3<FT>& trans,
            const Vec3<FT>& rot = Vec3<FT>(0, 0, 0),
            const Vec2<FT>& imageSize = Vec2<FT>(0, 0))
      : Camera<FT>(cam, trans, rot, imageSize) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_);
  }

  /// @brief Construct from camera matrix and rotation matrix.
  CameraHom(const Matx33<FT>& cam,
            const Vec3<FT>& trans,
            const Matx33<FT>& rot,
            const Vec2<FT>& imageSize = Vec2<FT>(0, 0))
      : Camera<FT>(cam, trans, rot, imageSize) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot);
  }

  /// @brief Construct from projection matrix directly.
  CameraHom(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(0, 0))
      : Camera<FT>(proj, imageSize), proj_(proj) {}

  /// @brief Construct from base Camera.
  CameraHom(const Camera<FT>& cam) : Camera<FT>(cam) {
    proj_ = Camera<FT>::composeProjectionMatrix(focal_, offset_, trans_, rot_);
  }

  /// @brief Get cached projection matrix.
  virtual Matx34<FT> projM() const { return proj_; }

  /// @name Projection Methods
  /// @{

  /// @brief Project homogeneous 4D point to homogeneous 3D.
  /// @param h Homogeneous 4D point.
  /// @return Homogeneous 3D result.
  inline Vec3<FT> projectH(const Vec4<FT>& h) const { return proj_ * h; }

  /// @brief Project homogeneous points (row matrix).
  /// @param points Points as rows (Nx4).
  /// @param[out] res Results as rows (Nx3).
  inline void projectHRow(const Eigen::Matrix<FT, Eigen::Dynamic, 4>& points,
                          Eigen::Matrix<FT, Eigen::Dynamic, 3>& res) const {
    res = points * proj_.transpose();
  }

  /// @brief Project homogeneous points (column matrix).
  /// @param points Points as columns (4xN).
  /// @param[out] res Results as columns (3xN).
  inline void projectHCol(const Eigen::Matrix<FT, 4, Eigen::Dynamic>& points,
                          Eigen::Matrix<FT, 3, Eigen::Dynamic>& res) const {
    res = proj_ * points;
  }

  /// @brief Project homogeneous points (row matrix, mapped).
  /// @param points Points as rows (Nx4).
  /// @param res Results as rows (Nx3).
  inline void projectHRowMap(const Eigen::Map<const Eigen::Matrix<FT, Eigen::Dynamic, 4>> points,
                             Eigen::Map<Eigen::Matrix<FT, Eigen::Dynamic, 3>> res) const {
    res = points * proj_.transpose();
  }

  /// @brief Project homogeneous points (column matrix, mapped).
  /// @param points Points as columns (4xN).
  /// @param res Results as columns (3xN).
  inline void projectHColMap(const Eigen::Map<const Eigen::Matrix<FT, 4, Eigen::Dynamic>> points,
                             Eigen::Map<Eigen::Matrix<FT, 3, Eigen::Dynamic>> res) const {
    res = proj_ * points;
  }

  /// @brief Project homogeneous point vector.
  /// @tparam V Container type.
  /// @tparam V1Args Additional template args for input container.
  /// @tparam V2Args Additional template args for output container.
  /// @param vh Input homogeneous point vector.
  /// @param[out] ret Output homogeneous result vector.
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

  /// @brief Project 3D points (row matrix) to normalized 2D.
  /// @param points Points as rows (Nx3).
  /// @param[out] res Results as rows (Nx2).
  inline void projectRow(const Eigen::Matrix<FT, Eigen::Dynamic, 3>& points,
                         Eigen::Matrix<FT, Eigen::Dynamic, 2>& res) const {
    res = (points.rowwise().homogeneous() * proj_.transpose()).rowwise().hnormalized();
  }

  /// @brief Project 3D points (column matrix) to normalized 2D.
  /// @param points Points as columns (3xN).
  /// @param[out] res Results as columns (2xN).
  inline void projectCol(const Eigen::Matrix<FT, 3, Eigen::Dynamic>& points,
                         Eigen::Matrix<FT, 2, Eigen::Dynamic>& res) const {
    res = (proj_ * points.colwise().homogeneous()).colwise().hnormalized();
  }

  /// @brief Project 3D points (row matrix, mapped) to normalized 2D.
  /// @param points Points as rows (Nx3).
  /// @param res Results as rows (Nx2).
  inline void projectRowMap(const Eigen::Map<const Eigen::Matrix<FT, Eigen::Dynamic, 3>> points,
                            Eigen::Map<Eigen::Matrix<FT, Eigen::Dynamic, 2>> res) const {
    res = (points.rowwise().homogeneous() * proj_.transpose()).rowwise().hnormalized();
  }

  /// @brief Project 3D points (column matrix, mapped) to normalized 2D.
  /// @param points Points as columns (3xN).
  /// @param res Results as columns (2xN).
  inline void projectColMap(const Eigen::Map<const Eigen::Matrix<FT, 3, Eigen::Dynamic>> points,
                            Eigen::Map<Eigen::Matrix<FT, 2, Eigen::Dynamic>> res) const {
    res = (proj_ * points.colwise().homogeneous()).colwise().hnormalized();
  }

  /// @brief Project single 3D point to normalized 2D.
  /// @param h 3D point.
  /// @return 2D projected point.
  inline Vec2<FT> project(const Vec3<FT>& h) const {
    return projectH(h.homogeneous()).hnormalized();
    // Vec3<FT> tmp = projectH(h.homogeneous());
    // Vec2<FT> tmp2 = Vec2<FT>(tmp[0]/tmp[2],tmp[1]/tmp[2]);
    // return detail::abs(tmp[2]) < LIMITS<FT>::TAU ? Vec2<FT>(FT(0),FT(0)) : Vec2<FT>(tmp[0]/tmp[2],tmp[1]/tmp[2]);
  }

  /// @brief Project vector of 3D points to 2D.
  /// @tparam V1 Input container type.
  /// @tparam V2 Output container type.
  /// @param vh Input 3D point vector.
  /// @param[out] ret Output 2D point vector.
  template <template <class, class...> class V1, class... V1Args, template <class, class...> class V2, class... V2Args>
  inline void project(const V1<Vec3<FT>, V1Args...>& vh, V2<Vec2<FT>, V2Args...>& ret) const {
    ret.resize(vh.size());

    if (vh.size() > 0) {
      // Create temporary matrices with correct layout
      Eigen::Matrix<FT, Eigen::Dynamic, 3> pts_matrix(vh.size(), 3);
      for (size_t i = 0; i < vh.size(); ++i) {
        pts_matrix.row(static_cast<Eigen::Index>(i)) << vh[i].x(), vh[i].y(), vh[i].z();
      }

      Eigen::Matrix<FT, Eigen::Dynamic, 2> res_matrix(vh.size(), 2);

      // Use the optimized matrix projection
      res_matrix = (pts_matrix.rowwise().homogeneous() * proj_.transpose()).rowwise().hnormalized();

      // Copy back to result vector
      for (size_t i = 0; i < vh.size(); ++i) {
        ret[i] = Vec2<FT>(res_matrix(static_cast<Eigen::Index>(i), 0), res_matrix(static_cast<Eigen::Index>(i), 1));
      }
    }
  }

  /// @}

  /// @brief Virtual destructor for proper inheritance.
  virtual ~CameraHom() = default;
};

/// @brief Single-precision homogeneous projection camera.
typedef CameraHom<float> CameraHomf;

/// @brief Double-precision homogeneous projection camera.
typedef CameraHom<double> CameraHomd;

/// @brief Camera with Plücker line and homogeneous point projection.
///
/// Extends CameraHom with support for 3D line projection using
/// Plücker coordinates. Can project both points and lines.
/// @tparam FT Floating-point type.
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

  /// @brief Default/parameter constructor.
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

  /// @brief Construct from FOV and image size.
  CameraPluecker(FT fov,
                 const Vec2<FT>& imageSize,
                 const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
                 const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(fov, imageSize, trans, rot),
        rotMt_(rodrigues(Vec3<FT>(-rot_))),
        camMCOF_(Camera<FT>::composeCameraMatrixCOF(focal_, offset_)) {}

  /// @brief Construct from intrinsic/extrinsic vectors.
  /// @param focal Focal lengths (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @param imageSize Image dimensions.
  /// @param trans Translation vector.
  /// @param rot Rodrigues rotation vector.
  CameraPluecker(const Vec2<FT>& focal,
                 const Vec2<FT>& offset,
                 const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)),
                 const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
                 const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(focal, offset, imageSize, trans, rot),
        rotMt_(rodrigues(Vec3<FT>(-rot_))),
        camMCOF_(Camera<FT>::composeCameraMatrixCOF(focal_, offset_)) {}

  /// @brief Construct from intrinsic/extrinsic vectors with rotation matrix.
  /// @param focal Focal lengths (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @param imageSize Image dimensions.
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  CameraPluecker(const Vec2<FT>& focal,
                 const Vec2<FT>& offset,
                 const Vec2<FT>& imageSize,
                 const Vec3<FT>& trans,
                 const Matx33<FT>& rot)
      : CameraHom<FT>(focal, offset, imageSize, trans, rot),
        rotMt_(rot.transpose()),
        camMCOF_(Camera<FT>::composeCameraMatrixCOF(focal_, offset_)) {}

  /// @brief Construct from camera matrix and extrinsics.
  /// @param cam Camera intrinsic matrix (3x3).
  /// @param trans Translation vector.
  /// @param rot Rodrigues rotation vector.
  /// @param imageSize Image dimensions.
  CameraPluecker(const Matx33<FT>& cam,
                 const Vec3<FT>& trans,
                 const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)),
                 const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize) {
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  /// @brief Construct from camera and rotation matrices.
  /// @param cam Camera intrinsic matrix (3x3).
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  /// @param imageSize Image dimensions.
  CameraPluecker(const Matx33<FT>& cam,
                 const Vec3<FT>& trans,
                 const Matx33<FT>& rot,
                 const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize), rotMt_(rot.t()) {
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  /// @brief Construct from projection matrix.
  /// @param proj 3x4 projection matrix.
  /// @param imageSize Image dimensions.
  CameraPluecker(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(proj, imageSize) {
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  /// @brief Copy construct from base Camera.
  /// @param cam Source camera.
  CameraPluecker(const Camera<FT>& cam) : CameraHom<FT>(cam) {
    rotMt_ = rodrigues(Vec3<FT>(-rot_));
    camMCOF_ = Camera<FT>::composeCameraMatrixCOF(focal_, offset_);
  }

  /// @brief Get camera matrix cofactors.
  /// @return Cofactor matrix (cached).
  virtual Matx33<FT> camMCOF() const { return camMCOF_; }

  /// @name Projection Methods
  /// @{

  using CameraHom<FT>::project;

  /// @brief Project 3D line to 2D using Plücker coordinates.
  /// @param line 3D line to project.
  /// @return 2D projected line (empty if degenerate).
  inline Line<FT> project(const Line3<FT>& line) const {
    Vec3<FT> tmp = (line.origin() - trans_).cross(line.direction());
    tmp = camMCOF_ * rotMt_ * tmp;

    FT xyNorm = std::sqrt(tmp.x() * tmp.x() + tmp.y() * tmp.y());
    if (xyNorm < LIMITS<FT>::tau()) return Line<FT>();

    return Line<FT>(tmp.x() / xyNorm, tmp.y() / xyNorm, -tmp.z() / xyNorm);
    // return projectPlueckerCof(camMCOF(),trans_,this->rotM(),line.momentum(),line.direction());
  }

  /// @brief Project vector of 3D lines to 2D.
  /// @tparam V1 Input container type.
  /// @tparam LV Output container type.
  /// @param vl3 Input 3D line vector.
  /// @param[out] ret Output 2D line vector.
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<Line3<FT>, V1Args...>& vl3, LV& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) { ret.push_back(project(l)); });
  }

  /// @brief Project generic 3D line vector to 2D.
  /// @tparam LV Input container type (must contain Line3-compatible objects).
  /// @param vl3 Input 3D line vector.
  /// @param[out] ret Output 2D line vector.
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<Line<FT>>& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) { ret.push_back(project(l)); });
  }

  /// @brief Project 3D line segment to 2D.
  /// @param line 3D line segment.
  /// @return 2D line segment (empty if degenerate).
  inline LineSegment<FT> project(const LineSegment3<FT>& line) const {
    Line<FT> l = project(static_cast<const Line3<FT>>(line));
    if (l.empty()) return LineSegment<FT>();
    return LineSegment<FT>(l, project(line.startPoint()), project(line.endPoint()));
  }

  /// @brief Project vector of 3D line segments to 2D.
  /// @tparam V1 Input container type.
  /// @tparam LV Output container type.
  /// @param vl3 Input 3D line segment vector.
  /// @param[out] ret Output 2D line segment vector.
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<LineSegment3<FT>, V1Args...>& vl3, LV& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) { ret.push_back(project(l)); });
  }

  /// @brief Project generic 3D line segment vector to 2D.
  /// @tparam LV Input container type (must contain LineSegment3-compatible objects).
  /// @param vl3 Input 3D line segment vector.
  /// @param[out] ret Output 2D line segment vector.
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<LineSegment<FT>>& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) { ret.push_back(project(l)); });
  }

  /// @}

  /// @name Static Helpers
  /// @{

  /// @brief Project 3D line using projection matrix.
  /// @param proj 3x4 projection matrix.
  /// @param l 3D line.
  /// @return 2D projected line.
  static inline Line2<FT> projectLine(const Matx34<FT> proj, const Line3<FT>& l) {
    CameraPluecker<FT> cam(proj);
    return cam.project(l);
  }

  /// @brief Project 3D line segment using projection matrix.
  /// @param proj 3x4 projection matrix.
  /// @param ls 3D line segment.
  /// @return 2D projected line segment.
  static inline LineSegment2<FT> projectLineSegment(const Matx34<FT> proj, const LineSegment3<FT>& ls) {
    CameraPluecker<FT> cam(proj);
    return cam.project(ls);
  }

  /// @brief Project Plücker line from point and direction.
  ///
  /// Projects a 3D line using Plücker coordinates where the line is
  /// defined by a point and direction vector.
  /// @param cam Camera intrinsic matrix (3x3).
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  /// @param p Point on the line.
  /// @param l Normalized direction vector.
  /// @return 2D projected line.
  static inline Line<FT> projectPlueckerP(
      const Matx33<FT>& cam, const Vec3<FT>& trans, const Matx33<FT>& rot, const Vec3<FT>& p, const Vec3<FT>& l) {
    return projectPlueckerM(cam, trans, rot, p.cross(l), l);
  }

  /// @brief Project Plücker line from momentum and direction.
  ///
  /// Projects a 3D line using Plücker coordinates (momentum m, direction l).
  /// @param cam Camera intrinsic matrix (3x3).
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  /// @param m Plücker momentum vector.
  /// @param l Normalized direction vector.
  /// @return 2D projected line.
  static inline Line<FT> projectPlueckerM(
      const Matx33<FT>& cam, const Vec3<FT>& trans, const Matx33<FT>& rot, const Vec3<FT>& m, const Vec3<FT>& l) {
    return projectPlueckerCof(Camera<FT>::composeCameraMatrixCOF(cam), trans, rot, m, l);
  }

  /// @brief Project Plücker line using camera cofactor matrix.
  ///
  /// Projects a 3D line using pre-computed camera matrix cofactors
  /// for efficiency when projecting multiple lines.
  /// @param camCOF Camera matrix cofactors.
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  /// @param m Plücker momentum vector.
  /// @param l Normalized direction vector.
  /// @return 2D projected line (empty if degenerate).
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


  /// @brief Project Cayley line representation.
  /// @param cam Camera intrinsic matrix (3x3).
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  /// @param cayley Cayley line representation (4D vector).
  /// @return 2D projected line.
  static inline Line<FT> projectCayley(const Matx33<FT>& cam,
                                       const Vec3<FT>& trans,
                                       const Matx33<FT>& rot,
                                       const Vec4<FT>& cayley) {
    return projectCayleyCOF(Camera<FT>::composeCameraMatrixCOF(cam), trans, rot, cayley);
  }

  /// @brief Project Cayley line using camera cofactor matrix.
  /// @param camCOF Camera matrix cofactors.
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  /// @param cayley Cayley line representation (4D vector).
  /// @return 2D projected line.
  static inline Line<FT> projectCayleyCof(const Matx33<FT>& camCOF,
                                          const Vec3<FT>& trans,
                                          const Matx33<FT>& rot,
                                          const Vec4<FT>& cayley) {
    Vec3<FT> m, l;
    Line3<FT>::plueckerCoordinatesFromCayley(cayley, m, l);
    return projectPlueckerCof(camCOF, trans, rot, m, l);
  }

  /// @}
};

/// @brief Single-precision Plücker projection camera.
typedef CameraPluecker<float> CameraPlueckerf;

/// @brief Double-precision Plücker projection camera.
typedef CameraPluecker<double> CameraPlueckerd;

/// @brief Camera projecting 3D lines via two-point projection.
///
/// Alternative to CameraPluecker that projects 3D lines by projecting
/// two points on the line and constructing a 2D line from them. Less
/// mathematically elegant but simpler to understand.
/// @tparam FT Floating-point type.
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

  /// @brief Default/parameter constructor.
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

  /// @brief Construct from FOV and image size.
  Camera2P(FT fov,
           const Vec2<FT>& imageSize,
           const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(fov, imageSize, trans, rot) {}

  /// @brief Construct from intrinsic/extrinsic vectors.
  /// @param focal Focal lengths (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @param imageSize Image dimensions.
  /// @param trans Translation vector.
  /// @param rot Rodrigues rotation vector.
  Camera2P(const Vec2<FT>& focal,
           const Vec2<FT>& offset,
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)),
           const Vec3<FT>& trans = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)))
      : CameraHom<FT>(focal, offset, imageSize, trans, rot) {}

  /// @brief Construct from intrinsic/extrinsic vectors with rotation matrix.
  /// @param focal Focal lengths (fx, fy).
  /// @param offset Principal point (cx, cy).
  /// @param imageSize Image dimensions.
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  Camera2P(const Vec2<FT>& focal,
           const Vec2<FT>& offset,
           const Vec2<FT>& imageSize,
           const Vec3<FT>& trans,
           const Matx33<FT>& rot)
      : CameraHom<FT>(focal, offset, imageSize, trans, rot) {}

  /// @brief Construct from camera matrix and extrinsics.
  /// @param cam Camera intrinsic matrix (3x3).
  /// @param trans Translation vector.
  /// @param rot Rodrigues rotation vector.
  /// @param imageSize Image dimensions.
  Camera2P(const Matx33<FT>& cam,
           const Vec3<FT>& trans,
           const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0)),
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize) {}

  /// @brief Construct from camera and rotation matrices.
  /// @param cam Camera intrinsic matrix (3x3).
  /// @param trans Translation vector.
  /// @param rot Rotation matrix.
  /// @param imageSize Image dimensions.
  Camera2P(const Matx33<FT>& cam,
           const Vec3<FT>& trans,
           const Matx33<FT>& rot,
           const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(cam, trans, rot, imageSize) {}

  /// @brief Construct from projection matrix.
  /// @param proj 3x4 projection matrix.
  /// @param imageSize Image dimensions.
  Camera2P(const Matx34<FT>& proj, const Vec2<FT>& imageSize = Vec2<FT>(FT(0), FT(0)))
      : CameraHom<FT>(proj, imageSize) {}

  /// @brief Copy construct from base Camera.
  /// @param cam Source camera.
  Camera2P(const Camera<FT>& cam) : CameraHom<FT>(cam) {}

  /// @name Projection Methods
  /// @{

  using CameraHom<FT>::project;

  /// @brief Project 3D line to 2D via two-point projection.
  ///
  /// Projects two points on the line (origin and a distant point)
  /// and constructs a 2D line through their projections.
  /// @param line 3D line to project.
  /// @return 2D projected line.
  inline Line<FT> project(const Line3<FT>& line) const {
    return Line<FT>(project(line.origin()), project(line.distanceOrigin(1000)));
  }

  /// @brief Project vector of 3D lines to 2D.
  /// @tparam V1 Input container type.
  /// @tparam LV Output container type.
  /// @param vl3 Input 3D line vector.
  /// @param[out] ret Output 2D line vector.
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<Line3<FT>, V1Args...>& vl3, LV& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) { ret.push_back(project(l)); });
  }

  /// @brief Project generic 3D line vector to 2D.
  /// @tparam LV Input container type (must contain Line3-compatible objects).
  /// @param vl3 Input 3D line vector.
  /// @param[out] ret Output 2D line vector.
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<Line<FT>>& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const Line3<FT>& l) { ret.push_back(project(l)); });
  }

  /// @brief Project 3D line segment to 2D via endpoint projection.
  /// @param line 3D line segment.
  /// @return 2D line segment.
  inline LineSegment<FT> project(const LineSegment3<FT>& line) const {
    return LineSegment<FT>(project(line.startPoint()), project(line.endPoint()));
  }

  /// @brief Project vector of 3D line segments to 2D.
  /// @tparam V1 Input container type.
  /// @tparam LV Output container type.
  /// @param vl3 Input 3D line segment vector.
  /// @param[out] ret Output 2D line segment vector.
  template <template <class, class...> class V1, class... V1Args, class LV>
  inline void project(const V1<LineSegment3<FT>, V1Args...>& vl3, LV& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) { ret.push_back(project(l)); });
  }

  /// @brief Project generic 3D line segment vector to 2D.
  /// @tparam LV Input container type (must contain LineSegment3-compatible objects).
  /// @param vl3 Input 3D line segment vector.
  /// @param[out] ret Output 2D line segment vector.
  template <class LV>
  inline void projectV(const LV& vl3, std::vector<LineSegment<FT>>& ret) const {
    ret.clear();
    ret.reserve(vl3.size());
    for_each(vl3.begin(), vl3.end(), [&](const LineSegment3<FT>& l) { ret.push_back(project(l)); });
  }

  /// @}
};

/// @brief Single-precision two-point projection camera.
typedef Camera2P<float> Camera2Pf;

/// @brief Double-precision two-point projection camera.
typedef Camera2P<double> Camera2Pd;

}  // namespace lsfm
