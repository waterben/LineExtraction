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

// #define EIGEN_MATRIXBASE_PLUGIN "MatrixBaseAddons.h"
#include <utility/limit.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstdlib>

namespace lsfm {
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


template <class FT>
FT getScalar(FT val) {
  return val;
}

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


// matrix base, up to 4x4
template <class FT, int _rows, int _cols>
class Matx : public Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type> {
 public:
  typedef FT float_type;

  typedef Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type> MatrixBase;
  typedef typename MatrixBase::Base Base;
  typedef typename MatrixBase::Index Index;

  // typedef typename Base::PlainObject PlainObject;
  // using Base::base;
  // using Base::coeffRef;

  // EIGEN_DENSE_PUBLIC_INTERFACE(MyMatrixBase)

  // Matx(const MyMatrixBase& other) : Eigen::Matrix<FT,_rows,_cols>(other) {}

  template <typename OtherDerived>
  Matx(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(other) {}

  template <typename OtherDerived>
  Matx(const Eigen::ReturnByValue<OtherDerived>& other)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(other) {}

  template <typename OtherDerived>
  Matx(const Eigen::EigenBase<OtherDerived>& other)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(other) {}

  Matx() : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {}
  Matx(const FT& v0, const FT& v1) : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(v0, v1) {}
  Matx(const FT& v0, const FT& v1, const FT& v2)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(v0, v1, v2) {}
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
  Matx(const FT& v0, const FT& v1, const FT& v2, const FT& v3, const FT& v4)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4;
  }
  Matx(const FT& v0, const FT& v1, const FT& v2, const FT& v3, const FT& v4, const FT& v5)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4, v5;
  }
  Matx(const FT& v0, const FT& v1, const FT& v2, const FT& v3, const FT& v4, const FT& v5, const FT& v6)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4, v5, v6;
  }
  Matx(const FT& v0, const FT& v1, const FT& v2, const FT& v3, const FT& v4, const FT& v5, const FT& v6, const FT& v7)
      : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>() {
    *this << v0, v1, v2, v3, v4, v5, v6, v7;
  }
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

  explicit Matx(const FT* vals) : Eigen::Matrix<FT, _rows, _cols, Major<_rows, _cols>::type>(vals) {}

  /*Matx& operator=(const MyMatrixBase& other) {
      this->Base::_set(other);
      return *this;
  }*/

  template <typename OtherDerived>
  Matx& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
    this->Base::operator=(other);
    return *this;
  }

  template <typename OtherDerived>
  Matx& operator=(const Eigen::EigenBase<OtherDerived>& other) {
    this->Base::operator=(other);
    return *this;
  }

  template <typename OtherDerived>
  Matx& operator=(const Eigen::ReturnByValue<OtherDerived>& func) {
    this->Base::operator=(func);
    return *this;
  }

  // linear access
  inline FT& operator[](Index idx) { return this->m_storage.data()[idx]; }
  inline const FT& operator[](Index idx) const { return this->m_storage.data()[idx]; }

  // point access
  inline FT& x() { return this->m_storage.data()[0]; }
  inline const FT& x() const { return this->m_storage.data()[0]; }

  inline FT& y() { return this->m_storage.data()[1]; }
  inline const FT& y() const { return this->m_storage.data()[1]; }

  inline FT& z() { return this->m_storage.data()[2]; }
  inline const FT& z() const { return this->m_storage.data()[2]; }

  inline FT& w() { return this->m_storage.data()[3]; }
  inline const FT& w() const { return this->m_storage.data()[3]; }
};


template <class FT>
using Matx22 = Matx<FT, 2, 2>;

template <class FT>
using Matx23 = Matx<FT, 2, 3>;

template <class FT>
using Matx32 = Matx<FT, 3, 2>;

template <class FT>
using Matx33 = Matx<FT, 3, 3>;

template <class FT>
using Matx34 = Matx<FT, 3, 4>;

template <class FT>
using Matx43 = Matx<FT, 4, 3>;

template <class FT>
using Matx44 = Matx<FT, 4, 4>;

typedef Matx22<float> Matx22f;
typedef Matx22<double> Matx22d;

typedef Matx23<float> Matx23f;
typedef Matx23<double> Matx23d;

typedef Matx32<float> Matx32f;
typedef Matx32<double> Matx32d;

typedef Matx33<float> Matx33f;
typedef Matx33<double> Matx33d;

typedef Matx34<float> Matx34f;
typedef Matx34<double> Matx34d;

typedef Matx43<float> Matx43f;
typedef Matx43<double> Matx43d;

typedef Matx44<float> Matx44f;
typedef Matx44<double> Matx44d;

template <class FT, int rows>
using Vec = Matx<FT, rows, 1>;

template <class FT>
using Vec2 = Vec<FT, 2>;

template <class FT>
using Vec3 = Vec<FT, 3>;

template <class FT>
using Vec4 = Vec<FT, 4>;

typedef Vec2<float> Vec2f;
typedef Vec2<double> Vec2d;
typedef Vec2<int> Vec2i;
typedef Vec3<float> Vec3f;
typedef Vec3<double> Vec3d;
typedef Vec3<int> Vec3i;
typedef Vec4<float> Vec4f;
typedef Vec4<double> Vec4d;
typedef Vec4<int> Vec4i;

template <class FT, int cols>
using RowVec = Matx<FT, 1, cols>;

template <class FT>
using RowVec2 = RowVec<FT, 2>;

template <class FT>
using RowVec3 = RowVec<FT, 3>;

template <class FT>
using RowVec4 = RowVec<FT, 4>;

typedef RowVec2<float> RowVec2f;
typedef RowVec2<double> RowVec2d;
typedef RowVec3<float> RowVec3f;
typedef RowVec3<double> RowVec3d;
typedef RowVec4<float> RowVec4f;
typedef RowVec4<double> RowVec4d;

//! convert rot vec to rot matrix
template <class FT>
inline Matx33<FT> rodrigues(const Vec3<FT>& r) {
  Vec3<FT> axis;
  FT angle = rodrigues(r, axis);
  return Eigen::AngleAxis<FT>(angle, axis).matrix();
}

//! convert rot matrix to rot vec
template <class FT>
inline Vec3<FT> rodrigues(const Matx33<FT>& r) {
  Eigen::AngleAxis<FT> axis(r);
  return axis.axis() * axis.angle();
}

//! convert rot point to rot axis and angle
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


//! compose homogeneouse matrix from trans vector and rot vector
template <class FT>
Matx44<FT> composeHom(const Vec3<FT>& trans, const Vec3<FT>& rot = Vec3<FT>(FT(0), FT(0), FT(0))) {
  return composeHom(trans, rodrigues(rot));
}

//! compose homogeneouse matrix from trans vector and rot matrix
template <class FT>
Matx44<FT> composeHom(const Vec3<FT>& trans, const Matx33<FT>& rot) {
  return Matx44<FT>(rot(0, 0), rot(0, 1), rot(0, 2), trans.x(), rot(1, 0), rot(1, 1), rot(1, 2), trans.y(), rot(2, 0),
                    rot(2, 1), rot(2, 2), trans.z(), FT(0), FT(0), FT(0), FT(1));
}

//! compose homogeneouse matrix from other matrix
template <class FT>
Matx44<FT> composeHom(const Matx33<FT>& m) {
  return Matx44<FT>(m(0, 0), m(0, 1), m(0, 2), FT(0), m(1, 0), m(1, 1), m(1, 2), FT(0), m(2, 0), m(2, 1), m(2, 2),
                    FT(0), FT(0), FT(0), FT(0), FT(1));
}

//! decompose homogeneouse matrix to trans vector and rot vector
template <class FT>
void decomposeHom(const Matx44<FT>& m, Vec3<FT>& trans, Vec3<FT>& rot) {
  Matx33<FT> r;
  decomposeHom(m, trans, r);
  rot = rodrigues(r);
}

//! decompose homogeneouse matrix to trans vector and rot matrix
template <class FT>
void decomposeHom(const Matx44<FT>& m, Vec3<FT>& trans, Matx33<FT>& rot) {
  trans = Vec3<FT>(m(0, 3), m(1, 3), m(2, 3));
  decomposeHom(m, rot);
}

//! decompose homogeneouse matrix and  matrix
template <class FT>
void decomposeHom(const Matx44<FT>& m, Matx33<FT>& m33) {
  m33 = Matx33<FT>(m(0, 0), m(0, 1), m(0, 2), m(1, 0), m(1, 1), m(1, 2), m(2, 0), m(2, 1), m(2, 2));
}

//! compose homogeneouse matrix from trans vector and rot matrix
template <class FT>
Matx33<FT> composeHom(const Vec2<FT>& trans, const Matx22<FT>& rot) {
  return Matx33<FT>(rot(0, 0), rot(0, 1), trans.x(), rot(1, 0), rot(1, 1), trans.y(), FT(0), FT(0), FT(1));
}

//! compose homogeneouse matrix from other matrix
template <class FT>
Matx33<FT> composeHom(const Matx22<FT>& m) {
  return Matx33<FT>(m(0, 0), m(0, 1), FT(0), m(1, 0), m(1, 1), FT(0), FT(0), FT(0), FT(1));
}

//! decompose homogeneouse matrix to trans vector and rot matrix
template <class FT>
void decomposeHom(const Matx33<FT>& m, Vec2<FT>& trans, Matx22<FT>& rot) {
  trans = Vec3<FT>(m(0, 2), m(1, 2));
  decomposeHom(m, rot);
}

//! decompose homogeneouse matrix to matrix
template <class FT>
void decomposeHom(const Matx33<FT>& m, Matx22<FT>& m22) {
  m22 = Matx33<FT>(m(0, 0), m(0, 1), m(1, 0), m(1, 1));
}
}  // namespace lsfm

namespace Eigen {
namespace internal {
template <typename _Scalar, int _Rows, int _Cols>
struct traits<lsfm::Matx<_Scalar, _Rows, _Cols> > {
  typedef _Scalar Scalar;
  typedef Dense StorageKind;
  typedef DenseIndex StorageIndex;
  typedef MatrixXpr XprKind;
  enum {
    RowsAtCompileTime = _Rows,
    ColsAtCompileTime = _Cols,
    MaxRowsAtCompileTime = Dynamic,
    MaxColsAtCompileTime = Dynamic,
    Flags = compute_matrix_flags<_Scalar, _Rows, _Cols, lsfm::Major<_Rows, _Cols>::type, Dynamic, Dynamic>::ret,
    CoeffReadCost = NumTraits<Scalar>::ReadCost,
    Options = lsfm::Major<_Rows, _Cols>::type,
    InnerStrideAtCompileTime = 1,
    OuterStrideAtCompileTime = (Options & RowMajor) ? ColsAtCompileTime : RowsAtCompileTime
  };
};
}  // namespace internal
}  // namespace Eigen
