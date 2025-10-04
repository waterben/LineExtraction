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
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>

namespace cv {

//! basic 2d vector type
template <class FT>
using Vec2 = Vec<FT, 2>;

//! basic 3d vector type
template <class FT>
using Vec3 = Vec<FT, 3>;

//! basic homogeneous vector type
template <class FT>
using Vec4 = Vec<FT, 4>;

//! basic 2d point type
template <class FT>
using Point2 = Point_<FT>;

//! basic 3d point type
template <class FT>
using Point3 = Point3_<FT>;

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


//! convert rot point to rot matrix
template <class FT>
inline Matx33<FT> rodrigues(const Point3<FT> r) {
  Matx33<FT> m;
  Rodrigues(*reinterpret_cast<const Vec3<FT>*>(&r), m);
  return m;
}

//! convert rot vec to rot matrix
template <class FT>
inline Matx33<FT> rodrigues(const Vec3<FT> r) {
  Matx33<FT> m;
  Rodrigues(r, m);
  return m;
}

//! convert rot matrix to rot point
template <class FT>
inline Vec3<FT> rodrigues(const Matx33<FT> r) {
  Vec3<FT> v;
  Rodrigues(r, v);
  return v;
}

//! convert rot matrix to rot vec
template <class FT>
inline Vec3<FT> rodriguesV(const Matx33<FT> r) {
  Vec3<FT> v;
  Rodrigues(r, v);
  return v;
}

//! convert rot point to rot axis and angle
template <class FT>
inline FT rodrigues(const Vec3<FT> r, Point3<FT>& axis) {
  FT n = norm(r);
  if (n < lsfm::LIMITS<FT>::tau()) {
    axis = Point3<FT>(1, 0, 0);
    return 0;
  }
  axis = r * (1 / n);
  return n;
}

//! convert rot point to rot axis and angle
template <class FT>
inline Matx33<FT> rodrigues(const Vec3<FT> r, Vec3<FT>& axis) {
  FT n = norm(r);
  if (n < lsfm::LIMITS<FT>::tau()) {
    axis = Vec3<FT>(1, 0, 0);
    return 0;
  }
  axis = r * (1 / n);
  return n;
}

//! convert quaternion (x,y,z,w) to rot matrix
template <class FT>
Matx33<FT> quaternion(const Vec4<FT>& q) {
  Matx33<FT> ret;

  Vec4<FT> sq(q[0] * q[0], q[1] * q[1], q[2] * q[2], q[3] * q[3]);

  // invs (inverse square length) is only required if quaternion is not already normalised
  FT invs = 1 / (sq[0] + sq[1] + sq[2] + sq[3]);
  ret(0, 0) = (sq[0] - sq[1] - sq[2] + sq[3]) * invs;  // since sqw + sqx + sqy + sqz =1/invs*invs
  ret(1, 1) = (-sq[0] + sq[1] - sq[2] + sq[3]) * invs;
  ret(2, 2) = (-sq[0] - sq[1] + sq[2] + sq[3]) * invs;

  FT tmp1 = q[0] * q[1];
  FT tmp2 = q[2] * q[3];
  ret(1, 0) = 2 * (tmp1 + tmp2) * invs;
  ret(0, 1) = 2 * (tmp1 - tmp2) * invs;

  tmp1 = q[0] * q[2];
  tmp2 = q[1] * q[3];
  ret(2, 0) = 2 * (tmp1 - tmp2) * invs;
  ret(0, 2) = 2 * (tmp1 + tmp2) * invs;

  tmp1 = q[1] * q[2];
  tmp2 = q[0] * q[3];
  ret(2, 1) = 2 * (tmp1 + tmp2) * invs;
  ret(1, 2) = 2 * (tmp1 - tmp2) * invs;

  return ret;
}

//! convert rot matrix to quaternion (x,y,z,w)
template <class FT>
Vec4<FT> quaternion(const Matx33<FT>& m) {
  FT tr = m(0, 0) + m(1, 1) + m(2, 2);

  if (tr > 0) {
    FT S = std::sqrt(tr + 1) * 2;  // S=4*qw
    return Vec4<FT>((m(2, 1) - m(1, 2)) / S, (m(0, 2) - m(2, 0)) / S, (m(1, 0) - m(0, 1)) / S, S / 4);
  }

  if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
    FT S = std::sqrt(1 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;  // S=4*qx
    return Vec4<FT>(S / 4, (m(0, 1) + m(1, 0)) / S, (m(0, 2) + m(2, 0)) / S, (m(2, 1) - m(1, 2)) / S);
  }

  if (m(1, 1) > m(2, 2)) {
    FT S = std::sqrt(1 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;  // S=4*qy
    return Vec4<FT>((m(0, 1) + m(1, 0)) / S, S / 4, (m(1, 2) + m(2, 1)) / S, (m(0, 2) - m(2, 0)) / S);
  }

  FT S = std::sqrt(1 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;  // S=4*qz
  return Vec4<FT>((m(0, 2) + m(2, 0)) / S, (m(1, 2) + m(2, 1)) / S, S / 4, (m(1, 0) - m(0, 1)) / S);
}

//! convert euler (yaw,pitch,roll) to rot matrix
template <class FT>
Matx33<FT> euler(const Vec3<FT>& e) {
  // Assuming the angles are in radians.
  FT ch = std::cos(e[0]);
  FT sh = std::sin(e[0]);
  FT ca = std::cos(e[1]);
  FT sa = std::sin(e[1]);
  FT cb = std::cos(e[2]);
  FT sb = std::sin(e[2]);

  Matx33<FT> ret(ch * ca, sh * sb - ch * sa * cb, ch * sa * sb + sh * cb, sa, ca * cb, -ca * sb, -sh * ca,
                 sh * sa * cb + ch * sb, -sh * sa * sb + ch * cb);
  return ret;
}

//! convert euler (yaw,pitch,roll) to rot matrix
template <class FT>
Matx33<FT> euler(const Point3<FT>& e) {
  // Assuming the angles are in radians.
  FT ch = std::cos(e.x);
  FT sh = std::sin(e.x);
  FT ca = std::cos(e.y);
  FT sa = std::sin(e.y);
  FT cb = std::cos(e.z);
  FT sb = std::sin(e.z);

  Matx33<FT> ret(ch * ca, sh * sb - ch * sa * cb, ch * sa * sb + sh * cb, sa, ca * cb, -ca * sb, -sh * ca,
                 sh * sa * cb + ch * sb, -sh * sa * sb + ch * cb);
  return ret;
}

//! convert rot matrix to euler (yaw,pitch,roll)
template <class FT>
Vec3<FT> euler(const Matx33<FT>& rot) {
  FT sing = rot(1, 0);
  // Assuming the angles are in radians.
  if (sing > 0.999)  // singularity at north pole
    return Vec3<FT>(std::atan2(rot(0, 2), rot(2, 2)), static_cast<FT>(CV_PI / 2), 0);

  if (sing < -0.999)  // singularity at south pole
    return Vec3<FT>(std::atan2(rot(0, 2), rot(2, 2)), -static_cast<FT>(CV_PI / 2), 0);

  return Vec3<FT>(std::atan2(-rot(2, 0), rot(0, 0)), std::asin(rot(1, 0)), std::atan2(-rot(1, 2), rot(1, 1)));
}

//! convert mat of homogoneous row vectors to mat of normalized row vectors (2d or 3d)
//! if col is true, input vectors are interpreted as cols vectors, else as row vectors
//! output are always row vectors
template <class FT>
void fromHomogeneous(const cv::Mat& in, cv::Mat& out, bool cols = false) {
  if (DataType<FT>::type == CV_32F) {
    convertPointsFromHomogeneous((cols ? in.t() : in), out);
    return;
  }

  int shape = 0;
  cv::Mat tmp = in.reshape(1);

  int ncols = tmp.cols;
  int nrows = tmp.rows;

  if (cols) {
    ncols = tmp.rows;
    nrows = tmp.cols;
  }

  if (out.empty()) {
    out = cv::Mat(nrows, ncols - 1, DataType<FT>::type);
  } else {
    shape = out.channels();
    out = out.reshape(1);
    if (out.cols != ncols - 1 || out.rows != nrows) out = cv::Mat(nrows, ncols - 1, DataType<FT>::type);
  }

  if (cols) {
    if (ncols == 4) {
      out = out.reshape(3);
      for (int i = 0; i != in.cols; ++i) {
        FT n = in.at<FT>(3, i);
        if (std::abs(n) < lsfm::LIMITS<FT>::tau())
          out.at<Vec3<FT>>(i, 0) = Vec3<FT>(0, 0, 0);
        else
          out.at<Vec3<FT>>(i, 0) = Vec3<FT>(in.at<FT>(0, i) / n, in.at<FT>(1, i) / n, in.at<FT>(2, i) / n);
      }
    } else {
      out = out.reshape(2);
      for (int i = 0; i != in.cols; ++i) {
        FT n = in.at<FT>(2, i);
        if (std::abs(n) < lsfm::LIMITS<FT>::tau())
          out.at<Vec2<FT>>(i, 0) = Vec2<FT>(0, 0);
        else
          out.at<Vec2<FT>>(i, 0) = Vec2<FT>(in.at<FT>(0, i) / n, in.at<FT>(1, i) / n);
      }
    }
  } else {
    if (ncols == 4) {
      out = out.reshape(3);
      tmp = tmp.reshape(4);
      for (int i = 0; i != tmp.rows; ++i) {
        const Vec4<FT>& v = tmp.at<Vec4<FT>>(i, 0);
        FT n = v[3];
        if (std::abs(n) < lsfm::LIMITS<FT>::tau())
          out.at<Vec3<FT>>(i, 0) = Vec3<FT>(0, 0, 0);
        else
          out.at<Vec3<FT>>(i, 0) = Vec3<FT>(v[0] / n, v[1] / n, v[2] / n);
      }
    } else {
      out = out.reshape(2);
      tmp = tmp.reshape(3);
      for (int i = 0; i != tmp.rows; ++i) {
        const Vec3<FT>& v = tmp.at<Vec3<FT>>(i, 0);
        FT n = v[2];
        if (std::abs(n) < lsfm::LIMITS<FT>::tau())
          out.at<Vec2<FT>>(i, 0) = Vec2<FT>(0, 0);
        else
          out.at<Vec2<FT>>(i, 0) = Vec2<FT>(v[0] / n, v[1] / n);
      }
    }
  }
  // undo of reshape
  if (shape) out = out.reshape(shape);
}

//! convert mat of homogoneous row vectors to mat of normalized row vectors (2d or 3d)
//! if col is true, input vectors are interpreted as cols vectors, else as row vectors
//! output are always row vectors
template <class FT>
inline void fromHomogeneousT(const cv::Mat_<FT>& in, cv::Mat_<FT>& out, bool cols = false) {
  fromHomogeneous<FT>(in, out, cols);
}

//! convert vector of homogneous point 3 to normalized point 2
template <class FT>
inline void fromHomogeneous(const std::vector<Point3<FT>>& in, std::vector<Point2<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  fromHomogeneous<FT>(matIn, matOut);
}

//! convert vector of homogneous vec 3 to normalized vec 2
template <class FT>
inline void fromHomogeneous(const std::vector<Vec3<FT>>& in, std::vector<Vec2<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  fromHomogeneous<FT>(matIn, matOut);
}

//! convert vector of homogneous vec 4 to normalized point 3
template <class FT>
inline void fromHomogeneous(const std::vector<Vec4<FT>>& in, std::vector<Point3<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  fromHomogeneous<FT>(matIn, matOut);
}

//! convert vector of homogneous vec 4 to normalized vec 3
template <class FT>
inline void fromHomogeneous(const std::vector<Vec4<FT>>& in, std::vector<Vec3<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  fromHomogeneous<FT>(matIn, matOut);
}

//! convert mat of row vectors to mat of homogeneous row vectors (2d or 3d)
//! if col is true, input vectors are interpreted as cols vectors, else as row vectors
//! output are always row vectors
template <class FT>
void toHomogeneous(const cv::Mat& in, cv::Mat& out, bool cols = false) {
  cv::Mat tmp = (cols ? in.t() : in);
  if (DataType<FT>::type == CV_32F) {
    convertPointsToHomogeneous(tmp, out);
    return;
  }

  int shape = 0;
  tmp = tmp.reshape(1);

  int ncols = tmp.cols;
  int nrows = tmp.rows;


  if (out.empty()) {
    out = cv::Mat(nrows, ncols + 1, DataType<FT>::type);
  } else {
    shape = out.channels();
    out = out.reshape(1);
    if (out.cols != ncols + 1 || out.rows != nrows) out = cv::Mat(nrows, ncols + 1, DataType<FT>::type);
  }

  tmp.copyTo(out.colRange(0, ncols));
  out.col(ncols).setTo(1);

  // undo of reshape
  if (shape) out = out.reshape(shape);
}

//! convert mat of row vectors to mat of homogeneous row vectors (2d or 3d)
//! if col is true, input vectors are interpreted as cols vectors, else as row vectors
//! output are always row vectors
template <class FT>
inline void toHomogeneousT(const cv::Mat_<FT>& in, cv::Mat_<FT>& out, bool cols = false) {
  toHomogeneous<FT>(in, out, cols);
}

//! convert vector of point 2 to homogeneous point 3
template <class FT>
inline void toHomogeneous(const std::vector<Point3<FT>>& in, std::vector<Point2<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  toHomogeneous<FT>(matIn, matOut);
}

//! convert vector of vec 2 to homogeneous vec 3
template <class FT>
inline void toHomogeneous(const std::vector<Vec3<FT>>& in, std::vector<Vec2<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  toHomogeneous<FT>(matIn, matOut);
}

//! convert vector of point 3 to homogeneous vec 4
template <class FT>
inline void toHomogeneous(const std::vector<Point3<FT>>& in, std::vector<Vec4<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  toHomogeneous<FT>(matIn, matOut);
}

//! convert vector of vec 3 to homogeneous vec 4
template <class FT>
inline void toHomogeneous(const std::vector<Vec3<FT>>& in, std::vector<Vec4<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  toHomogeneous<FT>(matIn, matOut);
}


//! matrix multiplication of vectors as row in data -> also results in vectors of rows
template <class FT>
inline void matxMul(cv::Mat mtx, cv::Mat data, cv::Mat& out) {
  if (data.channels() > 1) data = data.reshape(1);
  out = data * mtx.t();
}


template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Vec4<FT>>& data, std::vector<Vec4<FT>>& out) {
  CV_Assert(mtx.cols == 4 && mtx.rows == 4);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Vec3<FT>>& data, std::vector<Vec3<FT>>& out) {
  CV_Assert(mtx.cols == 3 && mtx.rows == 3);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Vec2<FT>>& data, std::vector<Vec2<FT>>& out) {
  CV_Assert(mtx.cols == 2 && mtx.rows == 2);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Point3<FT>>& data, std::vector<Point3<FT>>& out) {
  CV_Assert(mtx.cols == 3 && mtx.rows == 3);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Point2<FT>>& data, std::vector<Point2<FT>>& out) {
  CV_Assert(mtx.cols == 2 && mtx.rows == 2);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

//! auto convert to hom and back
template <class FT>
inline Vec3<FT> matxMul(const Matx44<FT>& mtx, const Vec3<FT>& data) {
  Vec4<FT> tmp = mtx * Vec4<FT>(data[0], data[1], data[2], 1);
  FT n = tmp[3];
  return std::abs(n) < lsfm::LIMITS<FT>::tau() ? Vec3<FT>(0, 0, 0) : Vec3<FT>(tmp[0] / n, tmp[1] / n, tmp[2] / n);
}

//! auto convert to hom and back
template <class FT>
inline Vec3<FT> matxMul(const Matx44<FT>& mtx, const Point3<FT>& data) {
  Vec4<FT> tmp = mtx * Vec4<FT>(data.x, data.y, data.z, 1);
  FT n = tmp[3];
  return std::abs(n) < lsfm::LIMITS<FT>::tau() ? Vec3<FT>(0, 0, 0) : Vec3<FT>(tmp[0] / n, tmp[1] / n, tmp[2] / n);
}

//! auto convert to hom and back
template <class FT>
inline Vec2<FT> matxMul(const Matx33<FT>& mtx, const Vec2<FT>& data) {
  Vec3<FT> tmp = mtx * Vec3<FT>(data[0], data[1], 1);
  FT n = tmp[2];
  return std::abs(n) < lsfm::LIMITS<FT>::tau() ? Vec2<FT>(0, 0) : Vec2<FT>(tmp[0] / n, tmp[1] / n);
}

//! auto convert to hom and back
template <class FT>
inline Vec2<FT> matxMul(const Matx33<FT>& mtx, const Point2<FT>& data) {
  Vec3<FT> tmp = mtx * Vec3<FT>(data.x, data.y, 1);
  FT n = tmp[2];
  return std::abs(n) < lsfm::LIMITS<FT>::tau() ? Vec2<FT>(0, 0) : Vec2<FT>(tmp[0] / n, tmp[1] / n);
}

//! auto convert to hom and back
template <class FT>
inline Vec3<FT> matxMulDir(const Matx44<FT>& mtx, const Vec3<FT>& data) {
  Vec4<FT> tmp = mtx * Vec4<FT>(data[0], data[1], data[2], 0);
  return Vec3<FT>(tmp[0], tmp[1]);
}

//! auto convert to hom and back
template <class FT>
inline Vec3<FT> matxMulDir(const Matx44<FT>& mtx, const Point3<FT>& data) {
  Vec4<FT> tmp = mtx * Vec4<FT>(data.x, data.y, data.z, 0);
  return Vec3<FT>(tmp[0], tmp[1]);
}

//! auto convert to hom and back
template <class FT>
inline Vec2<FT> matxMulDir(const Matx33<FT>& mtx, const Vec2<FT>& data) {
  Vec3<FT> tmp = mtx * Vec3<FT>(data[0], data[1], 0);
  return Vec2<FT>(tmp[0], tmp[1]);
}

//! auto convert to hom and back
template <class FT>
inline Vec2<FT> matxMulDir(const Matx33<FT>& mtx, const Point2<FT>& data) {
  Vec3<FT> tmp = mtx * Vec3<FT>(data.x, data.y, 0);
  return Vec2<FT>(tmp[0], tmp[1]);
}


template <class FT>
inline std::vector<Vec4<FT>> operator*(const Matx44<FT>& mtx, const std::vector<Vec4<FT>>& data) {
  std::vector<Vec4<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}

template <class FT>
inline std::vector<Vec3<FT>> operator*(const Matx33<FT>& mtx, const std::vector<Vec3<FT>>& data) {
  std::vector<Vec3<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}

template <class FT>
inline std::vector<Vec2<FT>> operator*(const Matx22<FT>& mtx, const std::vector<Vec2<FT>>& data) {
  std::vector<Vec2<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}

template <class FT>
inline std::vector<Point3<FT>> operator*(const Matx33<FT>& mtx, const std::vector<Point3<FT>>& data) {
  std::vector<Point3<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}

template <class FT>
inline std::vector<Point2<FT>> operator*(const Matx22<FT>& mtx, const std::vector<Point2<FT>>& data) {
  std::vector<Point2<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}


template <typename FT, int m, int n>
class DataType<lsfm::Matx<FT, m, n>> {
 public:
  typedef lsfm::Matx<FT, m, n> value_type;
  typedef lsfm::Matx<typename DataType<FT>::work_type, m, n> work_type;
  typedef FT channel_type;
  enum {
    generic_type = 0,
    depth = DataDepth<channel_type>::value,
    channels = m * n,
    fmt = ((channels - 1) << 8) + DataDepth<channel_type>::fmt,
    type = CV_MAKETYPE(depth, channels)
  };
  typedef Matx<channel_type, m, n> vec_type;
};

}  // namespace cv

namespace lsfm {
template <class FT, int rows, int cols>
Matx<FT, rows, cols> cv2eigen(const cv::Matx<FT, rows, cols>& m) {
  return Matx<FT, rows, cols>(&m(0));
}

template <class FT, int rows, int cols>
cv::Matx<FT, rows, cols> cv2eigen(const Matx<FT, rows, cols>& m) {
  return cv::Matx<FT, rows, cols>(m.data());
}
}  // namespace lsfm
