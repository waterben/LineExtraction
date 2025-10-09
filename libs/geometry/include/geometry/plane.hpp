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

namespace lsfm {

/**
 * plane object
 */
template <class FT>
class Plane {
 protected:
  // plane internals
  // normal
  Vec3<FT> n_;
  // distance of plane to origin
  FT d_;

 public:
  typedef FT value_type;
  virtual ~Plane() = default;

  Plane() : d_(0), n_(0, 0, 0) {}

  //! Init Plane by distance and normal (has to be normalized!)
  Plane(const FT& dist, const Vec3<FT>& n) : d_(dist), n_(n) {}

  //! Init Plane by Point and normal
  Plane(const Vec3<FT>& pnt, const Vec3<FT>& n) : n_(n), d_(0) {
    // normalize direction
    FT norm = std::sqrt(n_.dot(n_));
    if (norm < LIMITS<FT>::tau()) {
      n_ = Vec3<FT>(0, 0, 0);
      return;
    }
    n_ *= static_cast<FT>(1.0) / norm;
    // compute distance of plane from point on plane
    d_ = n_.dot(pnt);
  }

  //! Init Plane by three points
  Plane(const Vec3<FT>& p1, const Vec3<FT>& p2, const Vec3<FT>& p3) : n_(p2 - p1), d_(0) {
    // compute two directions and normal from this direction
    n_ = n_.cross(p3 - p1);
    if (n_.dot(n_) <= LIMITS<FT>::tau()) {
      n_ = Vec3<FT>(0, 0, 0);
      return;
    }
    n_ *= static_cast<FT>(1.0) / static_cast<FT>(std::sqrt(n_.dot(n_)));
    d_ = n_.dot(p1);
  }

  //! Init Plane by Line3 and point
  Plane(const Vec3<FT>& p, const Line3<FT>& l) : n_(l.origin() - p), d_(0) {
    n_ = n_.cross(l.direction());
    if (n_.dot(n_) <= LIMITS<FT>::tau()) {
      n_ = Vec3<FT>(0, 0, 0);
      return;
    }
    n_ *= 1.0 / std::sqrt(n_.dot(n_));
    d_ = n_.dot(p);
  }

  bool valid() const { return detail::abs(n_.dot(n_) - 1) <= LIMITS<FT>::tau(); }

  bool empty() const { return n_.dot(n_) == 0; }

  //! get line direction vector
  inline const Vec3<FT>& normal() const { return n_; }

  //! get line origin
  inline const Vec3<FT>& origin() const { return n_ * d_; }

  //! get distance to origin
  inline const FT dist2origin() const { return d_; }

  //! compute shortest distance between plane and point
  inline FT distance(const Vec3<FT>& p) const {
    // plane: nx = d -> d = (np - d)/|n| -> |n| = 1 -> d = np - d
    return n_.dot(p) - d_;
  }

  //! compute projected point on plane by shortest distance
  inline Vec3<FT> nearestPointOnPlane(const Vec3<FT>& p) const { return p - distance(p) * n_; }

  //! compute intersection between line and plane
  inline bool intersection(const Line3<FT>& l) const {
    FT nu = n_.dot(l.direction());
    return nu > LIMITS<FT>::tau();
  }

  //! compute intersection between two planes
  inline bool intersection(const Plane<FT>& p) const {
    // plane1: n1x = d1, plane2: n2x = d2 -> line direction u = n1xn2
    FT u = n_.cross(p.n_);
    return u.dot(u) > LIMITS<FT>::tau();
  }

  //! compute intersection between line and plane
  inline bool intersection(const Line3<FT>& l, Vec3<FT>& intp) const {
    // plane: nx = d, line: p + tu -> np+tnu = d -> t = (d-np/nu)
    FT nu = n_.dot(l.direction());
    if (nu <= LIMITS<FT>::tau()) return false;
    FT t = (d_ - n_.dot(l.origin())) / nu;
    intp = l.distanceOrigin(t);
    return true;
  }

  //! compute intersection between two planes
  inline bool intersection(const Plane<FT>& p, Line3<FT>& intl) const {
    // plane1: n1x = d1, plane2: n2x = d2 -> line direction u = n1xn2
    // std::cout << "pl: " << n_ << ", pr: " << p.n_ << std::endl;
    Vec3<FT> u = n_.cross(p.n_);
    if (u.dot(u) <= LIMITS<FT>::tau()) return false;

    // get point on line and both planes
    FT a = n_.dot(p.n_);
    FT n = 1 - a * a;
    Vec3<FT> q = n_ * ((d_ - p.d_ * a) / n) + p.n_ * ((p.d_ - d_ * a) / n);
    intl = Line3<FT>(q, u);
    return true;
  }

  //! rotate around origin
  inline Plane<FT>& rotate(const Matx33<FT>& rotMat) {
    n_ = rotMat * n_;
    d_ = n_.dot(rotMat * origin());
    return *this;
  }

  //! rotate around origin
  inline Plane<FT>& rotate(const Vec3<FT>& rot) {
    Matx33<FT> rotMat;
    Rodrigues(rot, rotMat);
    return rotate(rotMat);
  }

  //! rotate around given pivot
  inline Plane<FT>& rotate(const Matx33<FT>& rotMat, const Vec3<FT>& pivot) {
    n_ = rotMat * n_;
    d_ = n_.dot((rotMat * (origin() - pivot)) + pivot);
    return *this;
  }

  //! rotate around given pivot
  inline Plane<FT>& rotate(const Vec3<FT>& rot, const Vec3<FT>& pivot) {
    Matx33<FT> rotMat;
    Rodrigues(rot, rotMat);
    return rotate(rotMat);
  }

  //! change distance by translation
  inline Plane<FT>& translate(const Vec3<FT>& trans) {
    d_ = n_.dot(origin() + trans);
    return *this;
  }

  //! change distance by scalar (orthogonal translation)
  inline Plane<FT>& translate(FT d) {
    d_ += d;
    return *this;
  }

  //! compute angle between two planes
  inline FT angle(Plane<FT>& plane) const { return std::acos(n_.dot(plane.n_)); }

  //! compute angle plane and line
  inline FT angle(Line3<FT>& line) const { return std::acos(n_.dot(line.n_)); }


  //! flip direction
  virtual void flip() {
    n_ *= -1;
    d_ *= -1;
  }
};


typedef Plane<float> Planef;
typedef Plane<double> Planed;

}  // namespace lsfm
