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

/// @file plane.hpp
/// @brief 3D plane representation.
/// This file provides a class for representing 3D planes in Hesse normal form:
/// n · x = d, where n is the unit normal vector and d is the signed distance
/// to the origin. Features include:
/// - Multiple construction methods (point+normal, 3 points, line+point)
/// - Distance and projection computations
/// - Line-plane and plane-plane intersections
/// - Rotation and translation transformations
/// @see line3.hpp for 3D line representation.

#pragma once

#include <geometry/line3.hpp>

namespace lsfm {

/// @brief 3D plane representation in Hesse normal form.
/// Represents a plane as n · x = d, where:
/// - n is the unit normal vector
/// - d is the signed distance from origin to plane along the normal
/// @tparam FT Floating-point type for coordinates (float or double).
template <class FT>
class Plane {
 protected:
  Vec3<FT> n_;  //!< Unit normal vector.
  FT d_;        //!< Signed distance from origin to plane.

 public:
  typedef FT value_type;  //!< Value type alias.

  //! Virtual destructor for proper inheritance.
  virtual ~Plane() = default;

  /// @brief Default constructor creating an invalid plane.
  Plane() : d_(0), n_(0, 0, 0) {}

  /// @brief Construct plane from distance and normal.
  /// @param dist Distance from origin to plane.
  /// @param n Normal vector (must already be normalized).
  /// @note The normal must be pre-normalized for correct behavior.
  Plane(const FT& dist, const Vec3<FT>& n) : d_(dist), n_(n) {}

  /// @brief Construct plane from point and normal.
  /// Creates a plane passing through the given point with the
  /// specified normal direction. The normal is automatically normalized.
  /// @param pnt A point on the plane.
  /// @param n Normal direction (will be normalized).
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

  /// @brief Construct plane from three points.
  /// Creates a plane passing through all three points. The normal
  /// direction is computed from (p2-p1) × (p3-p1).
  /// @param p1 First point on the plane.
  /// @param p2 Second point on the plane.
  /// @param p3 Third point on the plane.
  /// @note Points must not be collinear.
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

  /// @brief Construct plane from a point and a line.
  /// Creates a plane containing both the point and the line.
  /// @param p A point on the plane.
  /// @param l A line contained in the plane.
  /// @note Point must not be on the line.
  Plane(const Vec3<FT>& p, const Line3<FT>& l) : n_(l.origin() - p), d_(0) {
    n_ = n_.cross(l.direction());
    if (n_.dot(n_) <= LIMITS<FT>::tau()) {
      n_ = Vec3<FT>(0, 0, 0);
      return;
    }
    n_ *= 1.0 / std::sqrt(n_.dot(n_));
    d_ = n_.dot(p);
  }

  /// @brief Check if the plane is valid (has unit normal).
  /// @return True if normal vector has unit length.
  bool valid() const { return detail::abs(n_.dot(n_) - 1) <= LIMITS<FT>::tau(); }

  /// @brief Check if the plane is empty (zero normal).
  /// @return True if normal vector is zero.
  bool empty() const { return n_.dot(n_) == 0; }

  /// @name Attribute Accessors
  /// @{

  /// @brief Get the plane normal vector.
  /// @return Const reference to the unit normal.
  inline const Vec3<FT>& normal() const { return n_; }

  /// @brief Get a point on the plane (closest to origin).
  /// @return The point on the plane nearest to the coordinate origin.
  inline const Vec3<FT>& origin() const { return n_ * d_; }

  /// @brief Get the signed distance from origin to plane.
  /// @return Signed distance along the normal direction.
  inline const FT dist2origin() const { return d_; }

  /// @}

  /// @name Distance and Projection
  /// @{

  /// @brief Compute signed distance from point to plane.
  /// Uses the plane equation n·x = d. Positive distance means the
  /// point is on the same side as the normal points.
  /// @param p Point to compute distance to.
  /// @return Signed distance from point to plane.
  inline FT distance(const Vec3<FT>& p) const {
    // plane: nx = d -> d = (np - d)/|n| -> |n| = 1 -> d = np - d
    return n_.dot(p) - d_;
  }

  /// @brief Project point onto plane.
  /// @param p Point to project.
  /// @return Nearest point on plane to p.
  inline Vec3<FT> nearestPointOnPlane(const Vec3<FT>& p) const { return p - distance(p) * n_; }

  /// @}

  /// @name Intersection Tests
  /// @{

  /// @brief Check if line intersects plane.
  /// @param l Line to test.
  /// @return True if line is not parallel to plane.
  inline bool intersection(const Line3<FT>& l) const {
    FT nu = n_.dot(l.direction());
    return nu > LIMITS<FT>::tau();
  }

  /// @brief Check if two planes intersect.
  /// @param p Other plane.
  /// @return True if planes are not parallel.
  inline bool intersection(const Plane<FT>& p) const {
    // plane1: n1x = d1, plane2: n2x = d2 -> line direction u = n1xn2
    FT u = n_.cross(p.n_);
    return u.dot(u) > LIMITS<FT>::tau();
  }

  /// @brief Compute line-plane intersection point.
  /// @param l Line to intersect.
  /// @param[out] intp Intersection point (if exists).
  /// @return True if intersection exists.
  inline bool intersection(const Line3<FT>& l, Vec3<FT>& intp) const {
    // plane: nx = d, line: p + tu -> np+tnu = d -> t = (d-np/nu)
    FT nu = n_.dot(l.direction());
    if (nu <= LIMITS<FT>::tau()) return false;
    FT t = (d_ - n_.dot(l.origin())) / nu;
    intp = l.distanceOrigin(t);
    return true;
  }

  /// @brief Compute plane-plane intersection line.
  /// @param p Other plane.
  /// @param[out] intl Intersection line (if exists).
  /// @return True if intersection exists (planes not parallel).
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

  /// @}

  /// @name Transformations
  /// @{

  /// @brief Rotate plane around coordinate origin.
  /// @param rotMat 3x3 rotation matrix.
  /// @return Reference to this plane for chaining.
  inline Plane<FT>& rotate(const Matx33<FT>& rotMat) {
    n_ = rotMat * n_;
    d_ = n_.dot(rotMat * origin());
    return *this;
  }

  /// @brief Rotate plane around coordinate origin using Rodrigues vector.
  /// @param rot Rodrigues rotation vector.
  /// @return Reference to this plane for chaining.
  inline Plane<FT>& rotate(const Vec3<FT>& rot) {
    Matx33<FT> rotMat;
    Rodrigues(rot, rotMat);
    return rotate(rotMat);
  }

  /// @brief Rotate plane around given pivot point.
  /// @param rotMat 3x3 rotation matrix.
  /// @param pivot Pivot point for rotation.
  /// @return Reference to this plane for chaining.
  inline Plane<FT>& rotate(const Matx33<FT>& rotMat, const Vec3<FT>& pivot) {
    n_ = rotMat * n_;
    d_ = n_.dot((rotMat * (origin() - pivot)) + pivot);
    return *this;
  }

  /// @brief Rotate plane around given pivot using Rodrigues vector.
  /// @param rot Rodrigues rotation vector.
  /// @param pivot Pivot point for rotation.
  /// @return Reference to this plane for chaining.
  inline Plane<FT>& rotate(const Vec3<FT>& rot, const Vec3<FT>& pivot) {
    Matx33<FT> rotMat;
    Rodrigues(rot, rotMat);
    return rotate(rotMat);
  }

  /// @brief Translate plane by a vector.
  /// Moves the plane along the translation vector by updating
  /// the distance parameter.
  /// @param trans Translation vector.
  /// @return Reference to this plane for chaining.
  inline Plane<FT>& translate(const Vec3<FT>& trans) {
    d_ = n_.dot(origin() + trans);
    return *this;
  }

  /// @brief Translate plane orthogonally by a scalar distance.
  /// Moves the plane along its normal direction.
  /// @param d Distance to move (positive = along normal).
  /// @return Reference to this plane for chaining.
  inline Plane<FT>& translate(FT d) {
    d_ += d;
    return *this;
  }

  /// @}

  /// @name Geometric Properties
  /// @{

  /// @brief Compute angle between two planes.
  /// @param plane Other plane.
  /// @return Angle in radians [0, π].
  inline FT angle(Plane<FT>& plane) const { return std::acos(n_.dot(plane.n_)); }

  /// @brief Compute angle between plane and line.
  /// @param line Line to compute angle with.
  /// @return Angle in radians.
  inline FT angle(Line3<FT>& line) const { return std::acos(n_.dot(line.n_)); }

  /// @brief Flip the plane normal direction.
  /// Reverses the normal and negates the distance to maintain
  /// the same geometric plane but with opposite orientation.
  virtual void flip() {
    n_ *= -1;
    d_ *= -1;
  }

  /// @}
};

/// @brief Single-precision 3D plane.
typedef Plane<float> Planef;

/// @brief Double-precision 3D plane.
typedef Plane<double> Planed;

}  // namespace lsfm
