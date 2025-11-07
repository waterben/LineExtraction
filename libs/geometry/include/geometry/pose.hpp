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

#include <geometry/base.hpp>

namespace lsfm {

namespace detail {
#ifndef GEOMETRY_USE_CERES
using namespace std;
#endif
};  // namespace detail

//! Base class for poses in 3d space (rotation and translation)
template <class FT>
class Pose {
 protected:
  // trans and rot
  Vec3<FT> trans_, rot_;

  // this can be overwritten to update members if state of object changes, eg. rotate...
  virtual void updateInternals() {}

 public:
  typedef FT float_type;

  //! init pose by single values (default: (0,0,0), (0,0,0))
  Pose(FT trans_x = FT(0), FT trans_y = FT(0), FT trans_z = FT(0), FT rot_x = FT(0), FT rot_y = FT(0), FT rot_z = FT(0))
      : trans_(trans_x, trans_y, trans_z), rot_(rot_x, rot_y, rot_z) {}

  //! init pose by pointer rot trans and rot
  Pose(const FT* trans, const FT* rot) : trans_(trans[0], trans[1], trans[2]), rot_(rot[0], rot[1], rot[2]) {}

  //! init pose by pointer (t.x,t.y,t.z,d.x,d.y,d.z)
  Pose(const FT* pose) : trans_(pose[0], pose[1], pose[2]), rot_(pose[3], pose[4], pose[5]) {}

  //! init pose by translation and rotation
  Pose(const Vec3<FT>& trans, const Vec3<FT>& rot) : trans_(trans), rot_(rot) {}

  //! init pose by translation and rotation
  Pose(const Vec3<FT>& trans, const Matx33<FT>& rot) : trans_(trans), rot_(rodrigues(rot)) {}

  //! get origin as pointer for slam
  inline FT* slamOrigin() { return &trans_[0]; }

  //! get rotation as pointer for slam
  inline FT* slamRot() { return &rot_[0]; }

  /* //! get pose as pointer (t.x,t.y,t.z,r.x,r.y,r.z)
   inline const FT* pose() const{
       return &trans_[0];
   }

   //! get pose as pointer (t.x,t.y,t.z,r.x,r.y,r.z)
   inline FT* pose() {
       return &trans_[0];
   }

   //! set pose as pointer (t.x,t.y,t.z,r.x,r.y,r.z)
   inline void pose(const FT *pose) {
       trans_ = Vec3<FT>(pose);
       rot_ = Vec3<FT>(pose+3);
   }*/

  // get pose
  const Pose<FT>& pose() const { return *this; }

  //! set pose
  inline void pose(const Pose<FT>& pose) {
    trans_ = pose.trans_;
    rot_ = pose.rot_;
    updateInternals();
  }

  //! get pose origin
  inline const Vec3<FT>& origin() const { return trans_; }

  //! set pose origin
  inline void origin(const Vec3<FT>& o) {
    trans_ = o;
    updateInternals();
  }

  //! get pose orientation
  inline const Vec3<FT>& orientation() const { return rot_; }

  //! set pose orientation
  inline void orientation(const Vec3<FT>& o) {
    rot_ = o;
    updateInternals();
  }

  //!  set pose orientation by rotation matrix
  virtual void orientation(const Matx33<FT>& rMat) { orientation(rodrigues(rMat)); }

  //! get rotation matrix
  virtual Matx33<FT> rotM() const { return rodrigues(rot_); }

  //! get pose as homogeneous matrix (pose to world)
  virtual Matx44<FT> homM() const { return composeHom(trans_, rot_); }

  //! get base change homogeneous matrix (world to pose)
  virtual Matx44<FT> baseH() const {
    Matx33<FT> rot = rodrigues(Vec3<FT>(-rot_));
    return composeHom(Vec3<FT>((-rot * trans_)), rot);
  }

  //! translate line
  virtual Pose<FT>& translate(const Vec3<FT>& trans) {
    trans_ += trans;
    updateInternals();
    return *this;
  }

  //! rotate around given pivot
  virtual Pose<FT>& rotate(const Matx33<FT>& rotMat, const Vec3<FT>& pivot) {
    Matx33<FT> r = rodrigues(rot_), rnew = r * rotMat;
    trans_ = (rnew * r.transpose() * (trans_ - pivot)) + pivot;
    rot_ = rodrigues(rnew);
    updateInternals();
    return *this;
  }

  //! rotate around pose origin
  virtual Pose<FT>& rotate(const Matx33<FT>& rotMat) {
    rot_ = rodrigues(Matx33<FT>(rodrigues(rot_) * rotMat));
    updateInternals();
    return *this;
  }

  //! rotate around pose origin
  inline Pose<FT>& rotate(const Vec3<FT>& rot) { return rotate(rodrigues(rot)); }

  //! rotate around given pivot
  inline Pose<FT>& rotate(const Vec3<FT>& rot, const Vec3<FT>& pivot) { return rotate(rodrigues(rot), pivot); }


  //! transform pose by hom matrix
  inline Pose<FT>& transform(const Matx44<FT>& m) {
    trans_ = matxMul(m, trans_);
    rot_ = matxMulDir(m, rot_);
    updateInternals();
    return *this;
  }

  //! "add" pose to current pose, puts the new pose "underneath" the current one
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

  //! "subtract" pose to current pose, inverse
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

  //! Virtual destructor for proper inheritance
  virtual ~Pose() = default;
};

template <class FT>
inline Pose<FT> operator*(const Matx44<FT>& m, const Pose<FT>& pose) {
  Pose<FT> p = pose;
  p.transform(m);
  return p;
}

typedef Pose<float> Posef;
typedef Pose<double> Posed;
}  // namespace lsfm
