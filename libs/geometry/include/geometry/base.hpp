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

/// @file base.hpp
/// @brief Base geometric types built on Eigen library.
/// Provides fundamental matrix and vector types for geometric computations,
/// wrapping Eigen matrices with convenient interfaces. Includes:
/// - Matx: Fixed-size matrix template (up to 4x4)
/// - Vec/RowVec: Column and row vector aliases
/// - Rodrigues rotation conversions
/// - Homogeneous matrix composition/decomposition utilities
/// All types are designed for use with the geometry library and are
/// compatible with Eigen operations.

#pragma once

// #define EIGEN_MATRIXBASE_PLUGIN "MatrixBaseAddons.h"
#include <utility/limit.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstdlib>

namespace lsfm {

/// @brief Internal math function imports for ADL-safe usage.
namespace detail {
using std::abs;
using std::acos;
using std::asin;
using std::atan;
using std::atan2;
using std::cos;
using std::cosh;
using std::exp;
using std::hypot;
using std::log;
using std::pow;
using std::round;
using std::sin;
using std::sinh;
using std::sqrt;
using std::tan;
using std::tanh;
}  // namespace detail


/// @brief Extract scalar value from a scalar type (identity operation).
/// @tparam FT Scalar type.
/// @param val Input value.
/// @return The same value.
template <class FT>
FT getScalar(FT val) {
  return val;
}

/// @brief Determine storage order based on matrix dimensions.
/// Uses row-major storage for matrices with multiple rows and columns,
/// and disables alignment for STL container compatibility.
/// @tparam rows Number of rows.
/// @tparam _cols Number of columns.
// note -> use Eigen::DontAlign for eigen matrix to prevent align problems, eg. transpose etc.
// eigen seems to have problems wiht stl vector -> Eigen::aligned_allocator has to be used or
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION or best swith off alignment?!
template <int rows, int _cols>
struct Major {
  enum {
    type = ((rows > 1 || rows == Eigen::Dynamic) * (_cols > 1 || _cols == Eigen::Dynamic) * Eigen::RowMajor) |
           Eigen::DontAlign
  };
  // static constexpr int type = ((rows > 1 || rows == Eigen::Dynamic) * (_cols > 1 || _cols == Eigen::Dynamic) *
  // Eigen::RowMajor) | Eigen::DontAlign ;
};


/// @brief Fixed-size matrix class extending Eigen::Matrix.
/// Provides a convenient wrapper around Eigen matrices with:
/// - Direct element access via x(), y(), z(), w() for vectors
/// - Linear indexing via operator[]
/// - Multiple constructors for various element counts
/// - STL container compatibility (alignment disabled)
/// @tparam FT Floating-point or scalar type.
/// @tparam _rows Number of rows (compile-time constant).
/// @tparam _cols Number of columns (compile-time constant).
// matrix base, up to 4x4
template <class FT, int _rows, int _cols>
class Matx : public Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type> {
 public:
  typedef FT float_type;  ///< Scalar element type.

  typedef Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type> MatrixBase;  ///< Underlying Eigen matrix type.
  typedef typename MatrixBase::Base Base;                                         ///< Eigen base class.
  typedef typename MatrixBase::Index Index;                                       ///< Index type for element access.

  // typedef typename Base::PlainObject PlainObject;
  // using Base::base;
  // using Base::coeffRef;

  // EIGEN_DENSE_PUBLIC_INTERFACE(MyMatrixBase)

  // Matx(const MyMatrixBase& other) : Eigen::Matrix<FT,_rows,_cols>(other) {}

  /// @brief Construct from any Eigen matrix expression.
  /// @tparam OtherDerived Eigen expression type.
  /// @param other Source expression.
  template <typename OtherDerived>
  Matx(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(other) {}

  /// @brief Construct from Eigen return-by-value expression.
  /// @tparam OtherDerived Eigen expression type.
  /// @param other Source expression.
  template <typename OtherDerived>
  Matx(const Eigen::ReturnByValue<OtherDerived>& other)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(other) {}

  /// @brief Construct from any Eigen base type.
  /// @tparam OtherDerived Eigen expression type.
  /// @param other Source expression.
  template <typename OtherDerived>
  Matx(const Eigen::EigenBase<OtherDerived>& other)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(other) {}

  /// @brief Default constructor (uninitialized).
  Matx() : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {}

  /// @brief Construct 2-element matrix/vector.
  Matx(const FT& v0, const FT& v1) : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(v0, v1) {}

  /// @brief Construct 3-element matrix/vector.
  Matx(const FT& v0, const FT& v1, const FT& v2)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(v0, v1, v2) {}

  /// @brief Construct 4-element matrix/vector.
  Matx(const FT& v0, const FT& v1, const FT& v2, const FT& v3)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    if (_rows == 4 || _cols == 4) {
      this->m_storage.data()[0] = v0;
      this->m_storage.data()[1] = v1;
      this->m_storage.data()[2] = v2;
      this->m_storage.data()[3] = v3;
      return;
    }
    *this << v0, v1, v2, v3;
  }

  /// @brief Construct 5-element matrix/vector.
  Matx(const FT& v0, const FT& v1, const FT& v2, const FT& v3, const FT& v4)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4;
  }

  /// @brief Construct 6-element matrix/vector (e.g., 2x3 or 3x2).
  Matx(const FT& v0, const FT& v1, const FT& v2, const FT& v3, const FT& v4, const FT& v5)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4, v5;
  }

  /// @brief Construct 7-element matrix/vector.
  Matx(const FT& v0, const FT& v1, const FT& v2, const FT& v3, const FT& v4, const FT& v5, const FT& v6)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4, v5, v6;
  }

  /// @brief Construct 8-element matrix/vector.
  Matx(const FT& v0, const FT& v1, const FT& v2, const FT& v3, const FT& v4, const FT& v5, const FT& v6, const FT& v7)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4, v5, v6, v7;
  }

  /// @brief Construct 9-element matrix (e.g., 3x3).
  Matx(const FT& v0,
       const FT& v1,
       const FT& v2,
       const FT& v3,
       const FT& v4,
       const FT& v5,
       const FT& v6,
       const FT& v7,
       const FT& v8)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4, v5, v6, v7, v8;
  }

  /// @brief Construct 12-element matrix (e.g., 3x4 or 4x3).
  Matx(const FT& v0,
       const FT& v1,
       const FT& v2,
       const FT& v3,
       const FT& v4,
       const FT& v5,
       const FT& v6,
       const FT& v7,
       const FT& v8,
       const FT& v9,
       const FT& v10,
       const FT& v11)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11;
  }

  /// @brief Construct 16-element matrix (e.g., 4x4).
  Matx(const FT& v0,
       const FT& v1,
       const FT& v2,
       const FT& v3,
       const FT& v4,
       const FT& v5,
       const FT& v6,
       const FT& v7,
       const FT& v8,
       const FT& v9,
       const FT& v10,
       const FT& v11,
       const FT& v12,
       const FT& v13,
       const FT& v14,
       const FT& v15)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15;
  }

  /// @brief Construct from raw data pointer.
  /// @param vals Pointer to array with at least rows*cols elements.
  explicit Matx(const FT* vals) : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(vals) {}

  /*Matx& operator=(const MyMatrixBase& other) {
      this->Base::_set(other);
      return *this;
  }*/

  /// @brief Assignment from Eigen matrix expression.
  /// @tparam OtherDerived Eigen expression type.
  /// @param other Source expression.
  /// @return Reference to this matrix.
  template <typename OtherDerived>
  Matx& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
    this->Base::operator=(other);
    return *this;
  }

  /// @brief Assignment from Eigen base type.
  /// @tparam OtherDerived Eigen expression type.
  /// @param other Source expression.
  /// @return Reference to this matrix.
  template <typename OtherDerived>
  Matx& operator=(const Eigen::EigenBase<OtherDerived>& other) {
    this->Base::operator=(other);
    return *this;
  }

  /// @brief Assignment from Eigen return-by-value expression.
  /// @tparam OtherDerived Eigen expression type.
  /// @param func Return-by-value functor.
  /// @return Reference to this matrix.
  template <typename OtherDerived>
  Matx& operator=(const Eigen::ReturnByValue<OtherDerived>& func) {
    this->Base::operator=(func);
    return *this;
  }

  /// @name Linear element access
  /// @{
  /// @brief Access element by linear index.
  /// @param idx Linear index (0 to rows*cols-1).
  /// @return Reference to element.
  inline FT& operator[](Index idx) { return this->m_storage.data()[idx]; }

  /// @brief Access element by linear index (const).
  /// @param idx Linear index (0 to rows*cols-1).
  /// @return Const reference to element.
  inline const FT& operator[](Index idx) const { return this->m_storage.data()[idx]; }
  /// @}

  /// @name Named coordinate accessors
  /// @{
  inline FT& x() { return this->m_storage.data()[0]; }              ///< Access first element (x coordinate).
  inline const FT& x() const { return this->m_storage.data()[0]; }  ///< Access first element (const).

  inline FT& y() { return this->m_storage.data()[1]; }              ///< Access second element (y coordinate).
  inline const FT& y() const { return this->m_storage.data()[1]; }  ///< Access second element (const).

  inline FT& z() { return this->m_storage.data()[2]; }              ///< Access third element (z coordinate).
  inline const FT& z() const { return this->m_storage.data()[2]; }  ///< Access third element (const).

  inline FT& w() { return this->m_storage.data()[3]; }              ///< Access fourth element (w/homogeneous).
  inline const FT& w() const { return this->m_storage.data()[3]; }  ///< Access fourth element (const).
  /// @}
};

/// @name Matrix type aliases
/// @{
template <class FT>
using Matx22 = Matx<FT, 2, 2>;  ///< 2x2 matrix alias.

template <class FT>
using Matx23 = Matx<FT, 2, 3>;  ///< 2x3 matrix alias.

template <class FT>
using Matx32 = Matx<FT, 3, 2>;  ///< 3x2 matrix alias.

template <class FT>
using Matx33 = Matx<FT, 3, 3>;  ///< 3x3 matrix alias.

template <class FT>
using Matx34 = Matx<FT, 3, 4>;  ///< 3x4 matrix alias.


template <class FT>
using Matx43 = Matx<FT, 4, 3>;  ///< 4x3 matrix alias.

template <class FT>
using Matx44 = Matx<FT, 4, 4>;  ///< 4x4 matrix alias.

typedef Matx22<float> Matx22f;   ///< 2x2 float matrix.
typedef Matx22<double> Matx22d;  ///< 2x2 double matrix.

typedef Matx23<float> Matx23f;   ///< 2x3 float matrix.
typedef Matx23<double> Matx23d;  ///< 2x3 double matrix.

typedef Matx32<float> Matx32f;   ///< 3x2 float matrix.
typedef Matx32<double> Matx32d;  ///< 3x2 double matrix.

typedef Matx33<float> Matx33f;   ///< 3x3 float matrix.
typedef Matx33<double> Matx33d;  ///< 3x3 double matrix.

typedef Matx34<float> Matx34f;   ///< 3x4 float matrix.
typedef Matx34<double> Matx34d;  ///< 3x4 double matrix.

typedef Matx43<float> Matx43f;   ///< 4x3 float matrix.
typedef Matx43<double> Matx43d;  ///< 4x3 double matrix.

typedef Matx44<float> Matx44f;   ///< 4x4 float matrix.
typedef Matx44<double> Matx44d;  ///< 4x4 double matrix.
/// @}

/// @name Vector type aliases
/// @{
template <class FT, int rows>
using Vec = Matx<FT, rows, 1>;  ///< Column vector alias.

template <class FT>
using Vec2 = Vec<FT, 2>;  ///< 2D column vector alias.

template <class FT>
using Vec3 = Vec<FT, 3>;  ///< 3D column vector alias.

template <class FT>
using Vec4 = Vec<FT, 4>;  ///< 4D column vector alias.

typedef Vec2<float> Vec2f;   ///< 2D float vector.
typedef Vec2<double> Vec2d;  ///< 2D double vector.
typedef Vec2<int> Vec2i;     ///< 2D integer vector.
typedef Vec3<float> Vec3f;   ///< 3D float vector.
typedef Vec3<double> Vec3d;  ///< 3D double vector.
typedef Vec3<int> Vec3i;     ///< 3D integer vector.
typedef Vec4<float> Vec4f;   ///< 4D float vector.
typedef Vec4<double> Vec4d;  ///< 4D double vector.
typedef Vec4<int> Vec4i;     ///< 4D integer vector.
/// @}

/// @name Row vector type aliases
/// @{
template <class FT, int cols>
using RowVec = Matx<FT, 1, cols>;  ///< Row vector alias.

template <class FT>
using RowVec2 = RowVec<FT, 2>;  ///< 2D row vector alias.

template <class FT>
using RowVec3 = RowVec<FT, 3>;  ///< 3D row vector alias.

template <class FT>
using RowVec4 = RowVec<FT, 4>;  ///< 4D row vector alias.

typedef RowVec2<float> RowVec2f;   ///< 2D float row vector.
typedef RowVec2<double> RowVec2d;  ///< 2D double row vector.
typedef RowVec3<float> RowVec3f;   ///< 3D float row vector.
typedef RowVec3<double> RowVec3d;  ///< 3D double row vector.
typedef RowVec4<float> RowVec4f;   ///< 4D float row vector.
typedef RowVec4<double> RowVec4d;  ///< 4D double row vector.
/// @}

/// @name Rodrigues rotation conversions
/// @{

/// @brief Convert rotation vector to rotation matrix (Rodrigues formula).
/// The rotation vector encodes both the axis and angle: the direction
/// is the rotation axis, and the magnitude is the rotation angle in radians.
/// @tparam FT Floating point type.
/// @param r Rotation vector (axis * angle).
/// @return 3x3 rotation matrix.
template <class FT>
inline Matx33<FT> rodrigues(const Vec3<FT>& r) {
  Vec3<FT> axis;
  FT angle = rodrigues(r, axis);
  return Eigen::AngleAxis<FT>(angle, axis).matrix();
}

/// @brief Convert rotation matrix to rotation vector (inverse Rodrigues).
/// @tparam FT Floating point type.
/// @param r 3x3 rotation matrix.
/// @return Rotation vector (axis * angle).
template <class FT>
inline Vec3<FT> rodrigues(const Matx33<FT>& r) {
  Eigen::AngleAxis<FT> axis(r);
  return axis.axis() * axis.angle();
}

/// @brief Extract rotation axis and angle from rotation vector.
/// @tparam FT Floating point type.
/// @param r Rotation vector (axis * angle).
/// @param[out] axis Normalized rotation axis (unit vector).
/// @return Rotation angle in radians.
template <class FT>
inline FT rodrigues(const Vec3<FT> r, Vec3<FT>& axis) {
  FT n = r.norm();
  if (n < LIMITS<FT>::tau()) {
    axis = Vec3<FT>(FT(1), FT(0), FT(0));
    return FT(0);
  }
  axis = r / n;
  return n;
}
/// @}

/// @name Homogeneous matrix composition
/// @{

/// @brief Compose 4x4 homogeneous transformation matrix from translation and rotation vector.
/// @tparam FT Floating point type.
/// @param trans Translation vector.
/// @param rot Rotation vector (axis * angle, default zero).
/// @return 4x4 homogeneous transformation matrix.
template <class FT>
Matx44<FT> composeHom(const Vec3<FT>& trans, const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0))) {
  return composeHom(trans, rodrigues(rot));
}

// clang-format off
/// @brief Compose 4x4 homogeneous transformation matrix from translation and rotation matrix.
/// @tparam FT Floating point type.
/// @param trans Translation vector.
/// @param rot 3x3 rotation matrix.
/// @return 4x4 homogeneous transformation matrix.
template <class FT>
Matx44<FT> composeHom(const Vec3<FT>& trans, const Matx33<FT>& rot) {
  return Matx44<FT>(
    rot(0, 0), rot(0, 1), rot(0, 2), trans.x(),
    rot(1, 0), rot(1, 1), rot(1, 2), trans.y(),
    rot(2, 0), rot(2, 1), rot(2, 2), trans.z(),
    FT(0),    FT(0),      FT(0),     FT(1)
  );
}

/// @brief Compose 4x4 homogeneous matrix from 3x3 matrix (no translation).
/// @tparam FT Floating point type.
/// @param m 3x3 rotation/transformation matrix.
/// @return 4x4 homogeneous matrix with zero translation.
template <class FT>
Matx44<FT> composeHom(const Matx33<FT>& m) {
  return Matx44<FT>(
    m(0, 0), m(0, 1), m(0, 2), FT(0),
    m(1, 0), m(1, 1), m(1, 2), FT(0),
    m(2, 0), m(2, 1), m(2, 2), FT(0),
    FT(0),   FT(0),   FT(0),   FT(1)
);
}
// clang-format on
/// @}

/// @name Homogeneous matrix decomposition
/// @{

/// @brief Decompose 4x4 homogeneous matrix into translation and rotation vectors.
/// @tparam FT Floating point type.
/// @param m 4x4 homogeneous transformation matrix.
/// @param[out] trans Translation vector.
/// @param[out] rot Rotation vector (axis * angle).
template <class FT>
void decomposeHom(const Matx44<FT>& m, Vec3<FT>& trans, Vec3<FT>& rot) {
  Matx33<FT> r;
  decomposeHom(m, trans, r);
  rot = rodrigues(r);
}

/// @brief Decompose 4x4 homogeneous matrix into translation vector and rotation matrix.
/// @tparam FT Floating point type.
/// @param m 4x4 homogeneous transformation matrix.
/// @param[out] trans Translation vector.
/// @param[out] rot 3x3 rotation matrix.
template <class FT>
void decomposeHom(const Matx44<FT>& m, Vec3<FT>& trans, Matx33<FT>& rot) {
  trans = Vec3<FT>(m(0, 3), m(1, 3), m(2, 3));
  decomposeHom(m, rot);
}

// clang-format off
/// @brief Extract 3x3 rotation matrix from 4x4 homogeneous matrix.
/// @tparam FT Floating point type.
/// @param m 4x4 homogeneous transformation matrix.
/// @param[out] m33 3x3 rotation matrix.
template <class FT>
void decomposeHom(const Matx44<FT>& m, Matx33<FT>& m33) {
  m33 = Matx33<FT>(
    m(0, 0), m(0, 1), m(0, 2),
    m(1, 0), m(1, 1), m(1, 2),
    m(2, 0), m(2, 1), m(2, 2)
  );
}


/// @brief Compose 3x3 homogeneous matrix from 2D translation and rotation (2D case).
/// @tparam FT Floating point type.
/// @param trans 2D translation vector.
/// @param rot 2x2 rotation matrix.
/// @return 3x3 homogeneous transformation matrix.
template <class FT>
Matx33<FT> composeHom(const Vec2<FT>& trans, const Matx22<FT>& rot) {
  return Matx33<FT>(
    rot(0, 0), rot(0, 1), trans.x(),
    rot(1, 0), rot(1, 1), trans.y(),
    FT(0),     FT(0),     FT(1)
  );
}

/// @brief Compose 3x3 homogeneous matrix from 2x2 matrix (2D case, no translation).
/// @tparam FT Floating point type.
/// @param m 2x2 rotation/transformation matrix.
/// @return 3x3 homogeneous matrix with zero translation.
template <class FT>
Matx33<FT> composeHom(const Matx22<FT>& m) {
  return Matx33<FT>(
    m(0, 0), m(0, 1), FT(0),
    m(1, 0), m(1, 1), FT(0),
    FT(0), FT(0), FT(1)
  );
}
// clang-format on

/// @brief Decompose 3x3 homogeneous matrix into 2D translation and rotation (2D case).
/// @tparam FT Floating point type.
/// @param m 3x3 homogeneous transformation matrix.
/// @param[out] trans 2D translation vector.
/// @param[out] rot 2x2 rotation matrix.
template <class FT>
void decomposeHom(const Matx33<FT>& m, Vec2<FT>& trans, Matx22<FT>& rot) {
  trans = Vec3<FT>(m(0, 2), m(1, 2));
  decomposeHom(m, rot);
}

/// @brief Extract 2x2 rotation matrix from 3x3 homogeneous matrix (2D case).
/// @tparam FT Floating point type.
/// @param m 3x3 homogeneous transformation matrix.
/// @param[out] m22 2x2 rotation matrix.
template <class FT>
void decomposeHom(const Matx33<FT>& m, Matx22<FT>& m22) {
  m22 = Matx33<FT>(m(0, 0), m(0, 1), m(1, 0), m(1, 1));
}
/// @}
}  // namespace lsfm

/// @brief Eigen traits specialization for lsfm::Matx.
/// Enables seamless integration of Matx with Eigen's expression templates.
namespace Eigen {
namespace internal {

/// @brief Traits specialization for lsfm::Matx matrix type.
/// @tparam _Scalar Scalar element type.
/// @tparam _Rows Number of rows at compile time.
/// @tparam _Cols Number of columns at compile time.
template <typename _Scalar, int _Rows, int _Cols>
struct traits<lsfm::Matx<_Scalar, _Rows, _Cols> > {
  typedef _Scalar Scalar;           ///< Scalar element type.
  typedef Dense StorageKind;        ///< Dense storage.
  typedef DenseIndex StorageIndex;  ///< Index type.
  typedef MatrixXpr XprKind;        ///< Expression kind.
  enum {
    RowsAtCompileTime = _Rows,       ///< Compile-time rows.
    ColsAtCompileTime = _Cols,       ///< Compile-time columns.
    MaxRowsAtCompileTime = Dynamic,  ///< Maximum rows (dynamic).
    MaxColsAtCompileTime = Dynamic,  ///< Maximum columns (dynamic).
    Flags = compute_matrix_flags<_Scalar, _Rows, _Cols, lsfm::Major<_Rows, _Cols>::type, Dynamic, Dynamic>::ret,
    CoeffReadCost = NumTraits<Scalar>::ReadCost,
    Options = lsfm::Major<_Rows, _Cols>::type,
    InnerStrideAtCompileTime = 1,
    OuterStrideAtCompileTime = (Options & RowMajor) ? ColsAtCompileTime : RowsAtCompileTime
  };
};
}  // namespace internal
}  // namespace Eigen
