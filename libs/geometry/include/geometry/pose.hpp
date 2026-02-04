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

/// @file pose.hpp
/// @brief 3D pose representation with rotation and translation.
///
/// Provides Pose class representing position and orientation in 3D space
/// using Rodrigues rotation vectors and translation vectors.

#pragma once

#include <geometry/base.hpp>

namespace lsfm {

namespace detail {
#ifndef GEOMETRY_USE_CERES
using namespace std;
#endif
};  // namespace detail

/// @brief 3D pose with translation and rotation.
///
/// Represents a 6-DOF pose in 3D space using a translation vector
/// and a Rodrigues rotation vector. Supports transformation operations
/// and conversion to/from homogeneous matrices.
/// @tparam FT Floating-point type.
template <class FT>
class Pose {
 protected:
  Vec3<FT> trans_;  ///< Translation vector (position).
  Vec3<FT> rot_;    ///< Rodrigues rotation vector (orientation).

  /// @brief Update internal cached members.
  ///
  /// Override in derived classes to update cached values when pose changes.
  virtual void updateInternals() {}

 public:
  typedef FT float_type;

  /// @brief Construct from individual components.
  /// @param trans_x X translation (default 0).
  /// @param trans_y Y translation (default 0).
  /// @param trans_z Z translation (default 0).
  /// @param rot_x X rotation (Rodrigues, default 0).
  /// @param rot_y Y rotation (Rodrigues, default 0).
  /// @param rot_z Z rotation (Rodrigues, default 0).
  Pose(FT trans_x = FT(0), FT trans_y = FT(0), FT trans_z = FT(0), FT rot_x = FT(0), FT rot_y = FT(0), FT rot_z = FT(0))
      : trans_(trans_x, trans_y, trans_z), rot_(rot_x, rot_y, rot_z) {}

  /// @brief Construct from translation and rotation arrays.
  /// @param trans Translation array [x, y, z].
  /// @param rot Rotation array [rx, ry, rz] (Rodrigues).
  Pose(const FT* trans, const FT* rot) : trans_(trans[0], trans[1], trans[2]), rot_(rot[0], rot[1], rot[2]) {}

  /// @brief Construct from combined pose array.
  /// @param pose Combined array [tx, ty, tz, rx, ry, rz].
  Pose(const FT* pose) : trans_(pose[0], pose[1], pose[2]), rot_(pose[3], pose[4], pose[5]) {}

  /// @brief Construct from translation and rotation vectors.
  /// @param trans Translation vector.
  /// @param rot Rodrigues rotation vector.
  Pose(const Vec3<FT>& trans, const Vec3<FT>& rot) : trans_(trans), rot_(rot) {}

  /// @brief Construct from translation and rotation matrix.
  /// @param trans Translation vector.
  /// @param rot Rotation matrix (converted to Rodrigues internally).
  Pose(const Vec3<FT>& trans, const Matx33<FT>& rot) : trans_(trans), rot_(rodrigues(rot)) {}

  /// @name SLAM Interface
  /// @{

  /// @brief Get origin pointer for SLAM optimization.
  /// @return Mutable pointer to translation data.
  inline FT* slamOrigin() { return &trans_[0]; }

  /// @brief Get rotation pointer for SLAM optimization.
  /// @return Mutable pointer to rotation data.
  inline FT* slamRot() { return &rot_[0]; }

  /// @}

  /// @name Pose Accessors
  /// @{

  /// @brief Get this pose (const reference).
  /// @return Reference to this pose.
  const Pose<FT>& pose() const { return *this; }

  /// @brief Set pose from another pose.
  /// @param pose Pose to copy.
  inline void pose(const Pose<FT>& pose) {
    trans_ = pose.trans_;
    rot_ = pose.rot_;
    updateInternals();
  }

  /// @brief Get translation (origin position).
  /// @return Translation vector.
  inline const Vec3<FT>& origin() const { return trans_; }

  /// @brief Set translation (origin position).
  /// @param o New translation.
  inline void origin(const Vec3<FT>& o) {
    trans_ = o;
    updateInternals();
  }

  /// @brief Get Rodrigues rotation vector.
  /// @return Rotation vector.
  inline const Vec3<FT>& orientation() const { return rot_; }

  /// @brief Set Rodrigues rotation vector.
  /// @param o New rotation.
  inline void orientation(const Vec3<FT>& o) {
    rot_ = o;
    updateInternals();
  }

  /// @brief Set orientation from rotation matrix.
  /// @param rMat Rotation matrix (converted to Rodrigues internally).
  virtual void orientation(const Matx33<FT>& rMat) { orientation(rodrigues(rMat)); }

  /// @}

  /// @name Matrix Representations
  /// @{

  /// @brief Get rotation matrix.
  /// @return 3x3 rotation matrix from Rodrigues vector.
  virtual Matx33<FT> rotM() const { return rodrigues(rot_); }

  /// @brief Get homogeneous transformation matrix (pose to world).
  /// @return 4x4 homogeneous matrix [R|t; 0 0 0 1].
  virtual Matx44<FT> homM() const { return composeHom(trans_, rot_); }

  /// @brief Get base change homogeneous matrix (world to pose).
  /// @return 4x4 inverse transformation matrix.
  virtual Matx44<FT> baseH() const {
    Matx33<FT> rot = rodrigues(Vec3<FT>(-rot_));
    return composeHom(Vec3<FT>((-rot * trans_)), rot);
  }

  /// @}

  /// @name Transformation Operations
  /// @{

  /// @brief Translate pose by vector.
  /// @param trans Translation to add.
  /// @return Reference to this pose.
  virtual Pose<FT>& translate(const Vec3<FT>& trans) {
    trans_ += trans;
    updateInternals();
    return *this;
  }

  /// @brief Rotate around given pivot point.
  /// @param rotMat Rotation matrix.
  /// @param pivot Center of rotation.
  /// @return Reference to this pose.
  virtual Pose<FT>& rotate(const Matx33<FT>& rotMat, const Vec3<FT>& pivot) {
    Matx33<FT> r = rodrigues(rot_), rnew = r * rotMat;
    trans_ = (rnew * r.transpose() * (trans_ - pivot)) + pivot;
    rot_ = rodrigues(rnew);
    updateInternals();
    return *this;
  }

  /// @brief Rotate around pose origin.
  /// @param rotMat Rotation matrix.
  /// @return Reference to this pose.
  virtual Pose<FT>& rotate(const Matx33<FT>& rotMat) {
    rot_ = rodrigues(Matx33<FT>(rodrigues(rot_) * rotMat));
    updateInternals();
    return *this;
  }

  /// @brief Rotate around pose origin (Rodrigues vector).
  /// @param rot Rodrigues rotation vector.
  /// @return Reference to this pose.
  inline Pose<FT>& rotate(const Vec3<FT>& rot) { return rotate(rodrigues(rot)); }

  /// @brief Rotate around given pivot (Rodrigues vector).
  /// @param rot Rodrigues rotation vector.
  /// @param pivot Center of rotation.
  /// @return Reference to this pose.
  inline Pose<FT>& rotate(const Vec3<FT>& rot, const Vec3<FT>& pivot) { return rotate(rodrigues(rot), pivot); }


  /// @brief Transform pose by homogeneous matrix.
  /// @param m 4x4 transformation matrix.
  /// @return Reference to this pose.
  inline Pose<FT>& transform(const Matx44<FT>& m) {
    trans_ = matxMul(m, trans_);
    rot_ = matxMulDir(m, rot_);
    updateInternals();
    return *this;
  }

  /// @}

  /// @name Pose Composition
  /// @{

  /// @brief Concatenate pose (add pose "underneath" current one).
  ///
  /// Computes the composition of this pose with another, effectively
  /// chaining transformations.
  /// @param p Pose to concatenate.
  inline void concat(const Pose<FT>& p) {
    //            rotate(p.rotM());
    /*
                lsfm::Pose<FT> tmpPose;
                tmpPose.translate(p.origin());
                tmpPose.rotate(this->rotM(), lsfm::Vec3<FT>(0,0,0));
                translate(tmpPose.origin());
    */
    //            translate(p.origin());

    lsfm::Matx44<FT> homMat = p.homM() * this->pose().homM();
    // newPose.transform(homMat);
    Vec3<FT> trans;
    Matx33<FT> rot;
    decomposeHom(homMat, trans, rot);
    lsfm::Pose<FT> newPose;
    newPose.origin(trans);
    newPose.orientation(rot);
    this->pose(newPose);

    /*
    this->pose().origin(lsfm::Vec3<FT>(0,0,0));
    this->pose().orientation(rodrigues(lsfm::Vec3<FT>(0,0,0)));
    this->pose().transform(homMat);
*/
    /*
                rotate(p.rotM());
                translate(p.origin());
                */
  }

  /// @brief Concatenate inverse pose.
  ///
  /// Computes the composition of this pose with the inverse of another,
  /// effectively "subtracting" the other pose.
  /// @param p Pose whose inverse to concatenate.
  inline void concatInverse(const Pose<FT>& p) {  // TODO: Test!

    lsfm::Matx44<FT> homMat = p.homM().inverse() * this->pose().homM();
    // newPose.transform(homMat);
    Vec3<FT> trans;
    Matx33<FT> rot;
    decomposeHom(homMat, trans, rot);
    lsfm::Pose<FT> newPose;
    newPose.origin(trans);
    newPose.orientation(rot);
    this->pose(newPose);
  }

  /// @}

  /// @brief Virtual destructor for proper inheritance.
  virtual ~Pose() = default;
};

/// @brief Transform pose by homogeneous matrix (matrix * pose).
/// @tparam FT Floating-point type.
/// @param m 4x4 transformation matrix.
/// @param pose Pose to transform.
/// @return Transformed pose.
template <class FT>
inline Pose<FT> operator*(const Matx44<FT>& m, const Pose<FT>& pose) {
  Pose<FT> p = pose;
  p.transform(m);
  return p;
}

/// @brief Single precision pose.
typedef Pose<float> Posef;

/// @brief Double precision pose.
typedef Pose<double> Posed;
}  // namespace lsfm
