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

/// @file line3.hpp
/// @brief 3D line and line segment representations.
///
/// This file provides classes for representing 3D infinite lines and line segments
/// using point-direction form. Features include:
/// - Point and direction vector representation
/// - Plücker coordinates for line representation
/// - Cayley representation for optimization
/// - Distance computations (point-to-line, line-to-line)
/// - Nearest point calculations
/// - Rotation and translation transformations
/// @see line.hpp for 2D line representations.

#pragma once

#include <geometry/line.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>


namespace lsfm {

/// @brief 3D infinite line representation.
///
/// Represents a 3D line using point-direction form: p + t*v,
/// where p is the origin point and v is the normalized direction vector.
/// Provides conversions to/from Plücker and Cayley representations
/// for use in optimization algorithms.
/// @tparam FT Floating-point type for coordinates (float or double).
template <class FT>
class Line3 {
 protected:
  Vec3<FT> p_;  //!< Origin point on the line.
  Vec3<FT> v_;  //!< Normalized direction vector.

 public:
  typedef FT float_type;  //!< Floating-point type alias.

  //! Virtual destructor for proper inheritance.
  virtual ~Line3() = default;

  /// @brief Default constructor creating an empty line.
  Line3() : p_(FT(0), FT(0), FT(0)), v_(FT(0), FT(0), FT(0)) {}

  /// @brief Construct line from point and direction without normalization.
  /// @param pnt Origin point on the line.
  /// @param vec Direction vector (used as-is, not normalized).
  /// @param[in] (unnamed) Tag parameter to distinguish from normalizing constructor.
  Line3(const Vec3<FT>& pnt, const Vec3<FT>& vec, bool) : p_(pnt), v_(vec) {}

  /// @brief Construct line from point and direction with normalization.
  ///
  /// Normalizes the direction vector. If the vector is too short (length < tau),
  /// creates an empty line.
  /// @param pnt Origin point on the line.
  /// @param vec Direction vector (will be normalized).
  Line3(const Vec3<FT>& pnt, const Vec3<FT>& vec) : p_(pnt), v_(vec) {
    // normalize direction
    FT n = detail::sqrt(v_.dot(v_));
    if (n < LIMITS<FT>::tau()) {
      v_ = Vec3<FT>(FT(0), FT(0), FT(0));
      p_ = v_;
      return;
    }
    v_ /= n;
  }

  /// @brief Convert to line with different floating-point type.
  /// @tparam newFT Target floating-point type.
  /// @return Line converted to new type.
  template <class newFT>
  Line3<newFT> convertTo() {
    return Line3<newFT>(Vec3<newFT>(static_cast<newFT>(p_[0]), static_cast<newFT>(p_[1]), static_cast<newFT>(p_[2])),
                        Vec3<newFT>(static_cast<newFT>(v_[0]), static_cast<newFT>(v_[1]), static_cast<newFT>(v_[2])));
  }

  /// @brief Create line from two points.
  /// @param beg Start point.
  /// @param end End point.
  /// @return Line passing through both points, directed from beg to end.
  static Line3<FT> twoPoint(const Vec3<FT>& beg, const Vec3<FT>& end) { return Line3<FT>(beg, end - beg); }

  /// @brief Check if the line is valid (has unit direction).
  /// @return True if direction vector has unit length.
  bool valid() const { return detail::abs(v_.dot(v_) - 1) <= LIMITS<FT>::tau(); }

  /// @brief Check if the line is empty (zero direction).
  /// @return True if direction vector is zero.
  bool empty() const { return v_.dot(v_) == FT(0); }

  /// @name Attribute Accessors
  /// @{

  /// @brief Get the line direction vector.
  /// @return Const reference to the unit direction vector.
  inline const Vec3<FT>& direction() const { return v_; }

  /// @brief Get the line origin point.
  /// @return Const reference to the origin point.
  inline const Vec3<FT>& origin() const { return p_; }

  /// @brief Compute the momentum vector for Plücker representation.
  ///
  /// The momentum m = p × v is used with the direction v to form
  /// the Plücker coordinates (v, m) of the line.
  /// @return Momentum vector (cross product of origin and direction).
  inline Vec3<FT> momentum() const { return p_.cross(v_); }

  /// @}

  /// @name Distance and Projection
  /// @{

  /// @brief Compute signed distance from point to perpendicular plane.
  ///
  /// Computes the distance from a point to the plane through the line
  /// origin with normal equal to the line direction.
  /// @param p Point to compute distance to.
  /// @return Signed distance along the line direction.
  inline FT normalDistance(const Vec3<FT>& p) const { return v_.dot(p - p_); }

  /// @brief Compute parameter for nearest point to another line.
  ///
  /// Computes the parameter t such that p_ + t*v_ is the point on this
  /// line nearest to the given line.
  /// @param l Other line to find distance to.
  /// @return Parameter t for the nearest point on this line.
  inline FT normalDistance(const Line3<FT>& l) const {
    Vec3<FT> w = p_ - l.p_;
    FT b = v_.dot(l.v_);
    FT D = 1 - b * b;

    return D > LIMITS<FT>::tau() ? (b * l.v_.dot(w) - v_.dot(w)) / D : 0;
  }

  /// @brief Compute nearest point on line to a given point.
  /// @param p Point to project onto line.
  /// @return Point on line nearest to p.
  /// @see http://geomalgorithms.com/a02-_lines.html
  inline Vec3<FT> nearestPointOnLine(const Vec3<FT>& p) const { return p_ + v_ * normalDistance(p); }

  /// @brief Compute nearest point on this line to another line.
  ///
  /// Finds the point on this line that is closest to the given line
  /// and returns the parameter sc (distance from origin along direction).
  /// @param l Other line.
  /// @param[out] sc Parameter for the nearest point (distance along direction).
  /// @return Point on this line nearest to line l.
  /// @see http://geomalgorithms.com/a07-_distance.html
  inline Vec3<FT> nearestPointOnLine(const Line3<FT>& l, FT& sc) const {
    Vec3<FT> w = p_ - l.p_;
    FT b = v_.dot(l.v_);
    FT D = 1 - b * b;
    // FT sc = 0;

    if (D > LIMITS<FT>::tau()) {
      sc = (b * l.v_.dot(w) - v_.dot(w)) / D;
    }

    return p_ + (v_ * sc);
  }

  /// @brief Compute nearest point on this line to another line.
  /// @param l Other line.
  /// @return Point on this line nearest to line l.
  inline Vec3<FT> nearestPointOnLine(const Line3<FT>& l) const {
    FT sc = 0;
    return nearestPointOnLine(l, sc);
  }

  /// @brief Compute nearest points on both lines.
  ///
  /// Finds the pair of points (one on each line) that represent
  /// the shortest distance between the two lines.
  /// @param l Other line.
  /// @param[out] p1 Nearest point on this line.
  /// @param[out] p2 Nearest point on line l.
  inline void nearestPointOnLine(const Line3<FT>& l, Vec3<FT>& p1, Vec3<FT>& p2) const {
    Vec3<FT> w = p_ - l.p_;
    FT b = v_.dot(l.v_);
    FT e = l.v_.dot(w);
    FT D = 1 - b * b;
    FT sc = 0, tc = e;

    if (D > LIMITS<FT>::tau()) {
      FT d = v_.dot(w);
      sc = (b * e - d) / D;
      tc = (e - b * d) / D;
    }

    p1 = p_ + (sc * v_);
    p2 = l.p_ + (tc * l.v_);
  }

  /// @brief Compute point at distance d from coordinate origin along line direction.
  /// @param d Distance along line direction from coordinate origin.
  /// @return Point at the specified distance.
  inline Vec3<FT> distance(FT d) const { return v_ * d; }

  /// @brief Compute point at distance d from given point along line direction.
  /// @param d Distance along line direction.
  /// @param p Starting point.
  /// @return Point at distance d from p along direction.
  inline Vec3<FT> distance(FT d, const Vec3<FT>& p) const { return p + v_ * d; }

  /// @brief Compute point at distance d from line origin along direction.
  /// @param d Distance along line direction from origin.
  /// @return Point at distance d from line origin.
  inline Vec3<FT> distanceOrigin(FT d) const { return distance(d, p_); }

  /// @brief Compute shortest distance between line and point.
  /// @param p Point to compute distance to.
  /// @return Euclidean distance from point to line.
  inline FT distance(const Vec3<FT>& p) const {
    Vec3<FT> d = p - nearestPointOnLine(p);
    return detail::sqrt(d.dot(d));
  }

  /// @brief Compute shortest distance between two lines.
  /// @param l Other line.
  /// @return Shortest Euclidean distance between the lines.
  inline FT distance(const Line3<FT>& l) const {
    Vec3<FT> w = p_ - l.p_;
    FT b = v_.dot(l.v_);
    FT e = l.v_.dot(w);
    FT D = FT(1) - b * b;
    FT sc = FT(0), tc = e;

    if (D > LIMITS<FT>::tau()) {
      FT d = v_.dot(w);
      sc = (b * e - d) / D;
      tc = (e - b * d) / D;
    }

    Vec3<FT> dP = w + (sc * v_) - (tc * l.v_);
    return detail::sqrt(dP.dot(dP));
  }

  /// @}

  /// @name Transformations
  /// @{

  /// @brief Rotate direction around line origin.
  ///
  /// Rotates only the direction vector, keeping the origin fixed.
  /// @param rotMat 3x3 rotation matrix.
  /// @return Reference to this line for chaining.
  inline Line3<FT>& rotateDirection(const Matx33<FT>& rotMat) {
    v_ = rotMat * v_;
    return *this;
  }

  /// @brief Rotate direction around line origin using Rodrigues vector.
  /// @param rot Rodrigues rotation vector.
  /// @return Reference to this line for chaining.
  inline Line3<FT>& rotateDirection(const Vec3<FT>& rot) { return rotate(rodrigues(rot)); }

  /// @brief Rotate line around coordinate origin.
  ///
  /// Rotates both the origin point and direction vector.
  /// @param rotMat 3x3 rotation matrix.
  /// @return Reference to this line for chaining.
  inline Line3<FT>& rotate(const Matx33<FT>& rotMat) {
    p_ = rotMat * p_;
    v_ = rotMat * v_;
    return *this;
  }

  /// @brief Rotate line around coordinate origin using Rodrigues vector.
  /// @param rot Rodrigues rotation vector.
  /// @return Reference to this line for chaining.
  inline Line3<FT>& rotate(const Vec3<FT>& rot) { return rotate(rodrigues(rot)); }

  /// @brief Rotate line around given pivot point.
  /// @param rotMat 3x3 rotation matrix.
  /// @param pivot Pivot point for rotation.
  /// @return Reference to this line for chaining.
  inline Line3<FT>& rotate(const Matx33<FT>& rotMat, const Vec3<FT>& pivot) {
    p_ = (rotMat * (p_ - pivot)) + pivot;
    v_ = rotMat * v_;
    return *this;
  }

  /// @brief Rotate line around given pivot using Rodrigues vector.
  /// @param rot Rodrigues rotation vector.
  /// @param pivot Pivot point for rotation.
  /// @return Reference to this line for chaining.
  inline Line3<FT>& rotate(const Vec3<FT>& rot, const Vec3<FT>& pivot) { return rotate(rodrigues(rot)); }

  /// @brief Translate line by a vector.
  /// @param trans Translation vector.
  /// @return Reference to this line for chaining.
  inline Line3<FT>& translate(const Vec3<FT>& trans) {
    p_ += trans;
    return *this;
  }

  /// @}

  /// @name Geometric Properties
  /// @{

  /// @brief Compute angle between two lines in radians.
  /// @param line3 Other line.
  /// @return Angle in radians [0, π].
  inline FT angle(const Line3<FT>& line3) const { return detail::acos(v_.dot(line3.v_)); }

  /// @brief Compute normal vector from two lines.
  /// @param line3 Other line.
  /// @return Cross product of direction vectors.
  inline Vec3<FT> normal(Line3<FT>& line3) const { return v_.cross(line3.v_); }

  /// @brief Flip the line direction.
  virtual void flip() { v_ *= FT(-1); }

  /// @}

  /// @name Cayley and Plücker Representations
  /// @{

  /// @brief Get Cayley representation of the line.
  ///
  /// The Cayley representation uses 4 parameters (w, s) where s is a 3-vector,
  /// suitable for optimization algorithms.
  /// @param[out] w Scalar component of Cayley representation.
  /// @param[out] s Vector component of Cayley representation.
  inline void cayley(FT& w, Vec3<FT>& s) const { cayleyRepresentationFromPluecker(momentum(), v_, w, s); }

  /// @brief Get Cayley representation as 4-vector.
  /// @return Vec4 containing (w, s[0], s[1], s[2]).
  inline Vec4<FT> cayley() const {
    Vec4<FT> ret;
    cayleyRepresentationFromPluecker(momentum(), v_, ret[0], *reinterpret_cast<Vec3<FT>*>(&ret[1]));
    return ret;
  }

  /// @brief Convert Plücker coordinates to Cayley representation (4-vector output).
  /// @param m Momentum vector (m = p × v).
  /// @param l Direction vector.
  /// @param[out] c Cayley representation as Vec4.
  static void cayleyRepresentationFromPluecker(const Vec3<FT>& m, const Vec3<FT>& l, Vec4<FT>& c) {
    cayleyRepresentationFromPluecker(m, l, c[0], *reinterpret_cast<Vec3<FT>*>(&c[1]));
  }

  /// @brief Compute pseudo-inverse of a matrix using SVD.
  /// @tparam T Scalar type.
  /// @tparam _Matrix_Type_ Eigen matrix type.
  /// @param a Matrix to invert.
  /// @param epsilon Tolerance for singular values.
  /// @return Pseudo-inverse of the matrix.
  template <typename T, typename _Matrix_Type_>
  static _Matrix_Type_ pseudoInverse(const _Matrix_Type_& a, T epsilon = lsfm::LIMITS<FT>::tau()) {
    Eigen::JacobiSVD<_Matrix_Type_> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    T tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() *
           (svd.singularValues().array().abs() > tolerance)
               .select(svd.singularValues().array().inverse(), 0)
               .matrix()
               .asDiagonal() *
           svd.matrixU().adjoint();
  }

  /// @brief Convert Plücker coordinates to Cayley representation.
  ///
  /// Converts from Plücker line representation (m, l) to the Cayley
  /// representation (w, s) which is better suited for optimization.
  /// @param m Momentum vector (m = p × v).
  /// @param l Direction vector.
  /// @param[out] w Scalar component of Cayley representation.
  /// @param[out] s Vector component of Cayley representation.
  static void cayleyRepresentationFromPluecker(const Vec3<FT>& m, const Vec3<FT>& l, FT& w, Vec3<FT>& s) {
    static const Matx33<FT> I = Matx33<FT>::Identity();

    // L2 norm
    w = detail::sqrt(m.dot(m));

    Matx33<FT> Q;
    Q(0, 0) = l[0];
    Q(1, 0) = l[1];
    Q(2, 0) = l[2];

    if (w <= LIMITS<FT>::tau()) {
      Vec3<FT> e1, e2, tmp;

      // create orthogonal base
      tmp[0] = -l[1];  // shift and negate
      tmp[1] = l[2];
      tmp[2] = l[0];
      e1 = l.cross(tmp);
      e2 = l.cross(e1);

      Q(0, 1) = e1[0];
      Q(1, 1) = e1[1];
      Q(2, 1) = e1[2];

      Q(0, 2) = e2[0];
      Q(1, 2) = e2[1];
      Q(2, 2) = e2[2];
      std::cout << "shift and negate reached!" << std::endl;

    } else {
      Vec3<FT> lXm = l.cross(m);
      FT lXmNorm = detail::sqrt(lXm.dot(lXm));

      Q(0, 1) = m[0] / w;
      Q(1, 1) = m[1] / w;
      Q(2, 1) = m[2] / w;

      /*if (lXmNorm < LIMITS<FT>::tau()) {
          // l is 0 -> set s to 0 and return to prevent div by zero
          s[0] = 0;
          s[1] = 0;
          s[2] = 0;
          return;
      } else {*/
      Q(0, 2) = lXm[0] / lXmNorm;
      Q(1, 2) = lXm[1] / lXmNorm;
      Q(2, 2) = lXm[2] / lXmNorm;
      //}
    }

    Matx33<FT> sx = (Q - I) * ((Q + I).inverse());

    if (!(sx.allFinite())) {  // happens only very rarely
      Eigen::Matrix<FT, Eigen::Dynamic, Eigen::Dynamic> QpI = (Q + I);
      Eigen::Matrix<FT, Eigen::Dynamic, Eigen::Dynamic> QpIPseudoInv = pseudoInverse<FT>(QpI);

      sx = (Q - I) * (QpIPseudoInv);
      w = w * -1;
    }

    s[0] = sx(2, 1);
    s[1] = sx(0, 2);
    s[2] = sx(1, 0);
  }

  /// @brief Convert Cayley representation (4-vector) to Plücker coordinates.
  /// @param c Cayley representation as Vec4.
  /// @param[out] m Momentum vector.
  /// @param[out] l Direction vector.
  inline static void plueckerCoordinatesFromCayley(const Vec4<FT>& c, Vec3<FT>& m, Vec3<FT>& l) {
    plueckerCoordinatesFromCayley(c[0], *reinterpret_cast<const Vec3<FT>*>(&c[1]), m, l);
  }

  /// @brief Convert Cayley representation to Plücker coordinates.
  ///
  /// Converts from Cayley representation (w, s) back to Plücker
  /// line representation (m, l).
  /// @param w Scalar component of Cayley representation.
  /// @param s Vector component of Cayley representation.
  /// @param[out] m Momentum vector.
  /// @param[out] l Direction vector.
  inline static void plueckerCoordinatesFromCayley(const FT w, const Vec3<FT>& s, Vec3<FT>& m, Vec3<FT>& l) {
    static const Matx33<FT> I = Matx33<FT>::Identity();

    Matx33<FT> Q, sx = Matx33<FT>::Zero();
    Vec3<FT> sv(s);

    FT sNormSquared = s.dot(s);

    sx(2, 1) = s[0];
    sx(0, 2) = s[1];
    sx(1, 0) = s[2];

    sx(1, 2) = -s[0];
    sx(2, 0) = -s[1];
    sx(0, 1) = -s[2];

    Q = ((FT(1) - sNormSquared) * I + FT(2) * sx + FT(2) * sv * sv.transpose()) / (FT(1) + sNormSquared);

    l[0] = Q(0, 0);
    l[1] = Q(1, 0);
    l[2] = Q(2, 0);

    m[0] = Q(0, 1) * w;
    m[1] = Q(1, 1) * w;
    m[2] = Q(2, 1) * w;
  }

  /// @brief Create 3D line from Plücker coordinates.
  /// @param m Momentum vector.
  /// @param l Direction vector.
  /// @return Line3 object.
  static Line3<FT> lineFromPluecker(Vec3<FT>& m, Vec3<FT>& l) { return Line3<FT>(l.cross(m), l, true); }

  /// @brief Create 3D line from Cayley representation (4-vector).
  /// @param c Cayley representation as Vec4.
  /// @return Line3 object.
  static Line3<FT> lineFromCayley(const Vec4<FT>& c) {
    return lineFromCayley(c[0], *reinterpret_cast<const Vec3<FT>*>(&c[1]));
  }

  /// @brief Create 3D line from Cayley representation.
  /// @param w Scalar component of Cayley representation.
  /// @param s Vector component of Cayley representation.
  /// @return Line3 object.
  static Line3<FT> lineFromCayley(FT w, const Vec3<FT>& s) {
    Vec3<FT> m, l;
    plueckerCoordinatesFromCayley(w, s, m, l);
    return lineFromPluecker(m, l);
  }

  /// @}
};


/// @brief Single-precision 3D line.
typedef Line3<float> Line3f;

/// @brief Double-precision 3D line.
typedef Line3<double> Line3d;

/// @brief 3D line segment.
///
/// Represents a finite portion of a 3D line defined by start and end
/// distances along the line direction from the origin point.
/// @tparam FT Floating-point type for coordinates.
template <class FT>
class LineSegment3 : public Line3<FT> {
 protected:
  using Line3<FT>::p_;
  using Line3<FT>::v_;

  FT beg_;  //!< Start distance from origin along direction.
  FT end_;  //!< End distance from origin along direction.

  /// @brief Swap direction and adjust distances.
  ///
  /// Internal helper to ensure beg_ <= end_ after construction.
  void swapDir() {
    Line3<FT>::flip();
    beg_ *= -1;
    end_ *= -1;
  }

 public:
  typedef FT float_type;  //!< Floating-point type alias.

  /// @brief Default constructor creating an empty segment.
  LineSegment3() : Line3<FT>(), beg_(0), end_(0) {}

  /// @brief Construct from point, direction, and distances.
  /// @param p Origin point on the line.
  /// @param dir Direction vector (will be normalized).
  /// @param beg Start distance from origin.
  /// @param end End distance from origin.
  LineSegment3(const Vec3<FT>& p, const Vec3<FT>& dir, FT beg, FT end) : Line3<FT>(p, dir), beg_(beg), end_(end) {
    if (beg > end) swapDir();
  }

  /// @brief Construct from two endpoints.
  /// @param line_begin Start point of segment.
  /// @param line_end End point of segment.
  LineSegment3(const Vec3<FT>& line_begin, const Vec3<FT>& line_end) : Line3<FT>(), beg_(0), end_(0) {
    // Eigen::DenseBase<FT,3,1> test = line_end - line_begin;
    //
    Vec3<FT> dir = Matx<FT, 3, 1>(line_end - line_begin);
    // Vec3<FT> dir = line_end - line_begin;
    FT n = detail::sqrt(dir.dot(dir));
    if (n < LIMITS<FT>::tau()) return;

    p_ = line_begin;
    v_ = dir * (1.0 / n);

    end_ = v_.dot(dir);
  }

  /// @brief Construct from point, direction, and projected endpoints.
  /// @param p Origin point on the line.
  /// @param dir Direction vector.
  /// @param beg Point to project as start.
  /// @param end Point to project as end.
  LineSegment3(const Vec3<FT>& p, const Vec3<FT>& dir, const Vec3<FT>& beg, const Vec3<FT>& end) : Line3<FT>(p, dir) {
    beg_ = normalDistance(beg);
    end_ = normalDistance(end);
    if (beg > end) swapDir();
  }

  /// @brief Construct from line and distances.
  /// @param l Base line.
  /// @param beg Start distance along line.
  /// @param end End distance along line.
  LineSegment3(const Line3<FT>& l, FT beg, FT end) : Line3<FT>(l), beg_(beg), end_(end) {
    if (beg > end) swapDir();
  }

  /// @brief Construct from line and projected endpoints.
  /// @param l Base line.
  /// @param beg Point to project as start.
  /// @param end Point to project as end.
  LineSegment3(const Line3<FT>& l, const Vec3<FT>& beg, const Vec3<FT>& end) : Line3<FT>(l) {
    beg_ = normalDistance(beg);
    end_ = normalDistance(end);
    if (beg > end) swapDir();
  }

  /// @brief Convert to segment with different floating-point type.
  /// @tparam newFT Target floating-point type.
  /// @return Converted line segment.
  template <class newFT>
  LineSegment3<newFT> convertTo() {
    return LineSegment3<newFT>(
        Vec3<newFT>(static_cast<newFT>(p_[0]), static_cast<newFT>(p_[1]), static_cast<newFT>(p_[2])),
        Vec3<newFT>(static_cast<newFT>(v_[0]), static_cast<newFT>(v_[1]), static_cast<newFT>(v_[2])),
        static_cast<newFT>(beg_), static_cast<newFT>(end_));
  }

  /// @name Endpoint Accessors
  /// @{

  /// @brief Get the start point of the segment.
  /// @return Start point as Vec3.
  Vec3<FT> startPoint() const { return Vec3<FT>(p_ + v_ * beg_); }

  /// @brief Get the start point as OpenCV Mat.
  /// @param type OpenCV data type (CV_32F or CV_64F).
  /// @return Start point as 3x1 Mat.
  cv::Mat startPoint_cv(int type) {
    Vec3<FT> sp(p_ + v_ * beg_);
    cv::Mat m(3, 1, type);
    m.at<FT>(0) = sp[0];
    m.at<FT>(1) = sp[1];
    m.at<FT>(2) = sp[2];
    return m;
  }

  /// @brief Get the end point of the segment.
  /// @return End point as Vec3.
  Vec3<FT> endPoint() const { return Vec3<FT>(p_ + v_ * end_); }

  /// @brief Get the end point as OpenCV Mat.
  /// @param type OpenCV data type (CV_32F or CV_64F).
  /// @return End point as 3x1 Mat.
  cv::Mat endPoint_cv(int type) {
    Vec3<FT> ep(p_ + v_ * end_);
    cv::Mat m(3, 1, type);
    m.at<FT>(0) = ep[0];
    m.at<FT>(1) = ep[1];
    m.at<FT>(2) = ep[2];
    return m;
  }

  /// @brief Get the center point of the segment.
  /// @return Midpoint of the segment.
  Vec3<FT> centerPoint() const { return Vec3<FT>(p_ + v_ * beg_ + ((end_ - beg_) / 2) * v_); }

  /// @brief Get the center point as OpenCV Mat.
  /// @param type OpenCV data type (CV_32F or CV_64F).
  /// @return Center point as 3x1 Mat.
  cv::Mat centerPoint_cv(int type) {
    Vec3<FT> cp(p_ + v_ * beg_ + ((end_ - beg_) / 2) * v_);
    cv::Mat m(3, 1, type);
    m.at<FT>(0) = cp[0];
    m.at<FT>(1) = cp[1];
    m.at<FT>(2) = cp[2];
    return m;
  }

  /// @brief Get the length of the segment.
  /// @return Segment length.
  inline FT length() const { return detail::abs(end_ - beg_); }

  /// @brief Get internal begin distance value.
  /// @return Start distance from origin.
  FT getBegin() { return beg_; }

  /// @brief Get internal end distance value.
  /// @return End distance from origin.
  FT getEnd() { return end_; }

  /// @}

  /// @name Projection and Distance
  /// @{

  /// @brief Find nearest point on segment to another line.
  ///
  /// Similar to nearestPointOnLine but clamps the result to segment bounds.
  /// @param l Other line to find distance to.
  /// @return Nearest point on this segment (clamped to endpoints if needed).
  /// @see http://geomalgorithms.com/a07-_distance.html
  inline Vec3<FT> nearestPointOnLineSegment(const Line3<FT>& l) const {
    FT a = beg_, b = end_, sc = 0;
    this->nearestPointOnLine(l, sc);

    if (a > b) std::swap(a, b);
    if (sc < a)
      sc = a;
    else if (sc > b)
      sc = b;

    return p_ + (v_ * sc);
  }

  /// @}

  /// @name Transformations
  /// @{

  /// @brief Swap start and end points.
  inline void endPointSwap() { std::swap(beg_, end_); }

  /// @brief Flip the segment direction.
  ///
  /// Reverses the direction vector and adjusts distances to maintain
  /// the same physical segment.
  virtual void flip() {
    Line3<FT>::flip();
    beg_ *= FT(-1);
    end_ *= FT(-1);
    std::swap(beg_, end_);
  }

  /// @}

  /// @name Error Measurement
  /// @{

  /// @brief Compute squared endpoint distance error.
  ///
  /// Calculates the sum of squared distances from both endpoints
  /// to a ground truth line. Useful for evaluating segment accuracy.
  /// @param gtLine Ground truth line to measure against.
  /// @return Sum of squared endpoint distances.
  inline FT error(const Line3<FT>& gtLine) const {
    if (this->empty() || gtLine.empty()) return FT(0);
    FT ds = gtLine.distance(this->startPoint());
    FT de = gtLine.distance(this->endPoint());
    return (ds * ds + de * de);
  }

  /// @}

  /// @brief Stream output operator for line segments.
  /// @tparam U Floating-point type.
  /// @param os Output stream.
  /// @param ls3 Line segment to output.
  /// @return Reference to the output stream.
  template <class U>
  friend std::ostream& operator<<(std::ostream& os, const LineSegment3<U>& ls3);
};

/// @brief Stream output operator for LineSegment3.
///
/// Outputs the segment in format: "S: sx, sy, sz  E: ex, ey, ez,"
/// @tparam FT Floating-point type.
/// @param os Output stream.
/// @param ls3 Line segment to output.
/// @return Reference to the output stream.
template <class FT>
std::ostream& operator<<(std::ostream& os, const LineSegment3<FT>& ls3) {
  return os << "S: " << ls3.startPoint()[0] << ", " << ls3.startPoint()[1] << ", " << ls3.startPoint()[2]
            << "  E: " << ls3.endPoint()[0] << ", " << ls3.endPoint()[1] << ", " << ls3.endPoint()[2] << ", ";
}

/// @brief Single-precision 3D line segment.
typedef LineSegment3<float> LineSegment3f;

/// @brief Double-precision 3D line segment.
typedef LineSegment3<double> LineSegment3d;

}  // namespace lsfm
