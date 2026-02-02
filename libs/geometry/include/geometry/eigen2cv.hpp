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

/// @file eigen2cv.hpp
/// @brief OpenCV type aliases and conversion utilities.
/// This file provides OpenCV-based type aliases and rotation conversion
/// functions as an alternative to the Eigen-based types in base.hpp.
/// Includes:
/// - Type aliases: Vec2, Vec3, Vec4, Point2, Point3, Matx templates
/// - Rotation conversions: Rodrigues, quaternion, Euler angles
/// - Homogeneous coordinate transformations
/// - Projection and calibration matrix operations
/// Use this header when working primarily with OpenCV types rather than
/// Eigen types.
/// @see base.hpp for Eigen-based types.

#pragma once

#include <geometry/base.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>

namespace cv {

/// @name Type Aliases
/// OpenCV-based vector and matrix type aliases for common dimensions.
/// @{

/// @brief 2D vector type.
/// @tparam FT Floating-point type.
template <class FT>
using Vec2 = Vec<FT, 2>;

/// @brief 3D vector type.
/// @tparam FT Floating-point type.
template <class FT>
using Vec3 = Vec<FT, 3>;

/// @brief 4D (homogeneous) vector type.
/// @tparam FT Floating-point type.
template <class FT>
using Vec4 = Vec<FT, 4>;

/// @brief 2D point type.
/// @tparam FT Floating-point type.
template <class FT>
using Point2 = Point_<FT>;

/// @brief 3D point type.
/// @tparam FT Floating-point type.
template <class FT>
using Point3 = Point3_<FT>;

/// @brief 2x2 matrix type.
/// @tparam FT Floating-point type.
template <class FT>
using Matx22 = Matx<FT, 2, 2>;

/// @brief 2x3 matrix type (2D affine transform).
/// @tparam FT Floating-point type.
template <class FT>
using Matx23 = Matx<FT, 2, 3>;

/// @brief 3x2 matrix type.
/// @tparam FT Floating-point type.
template <class FT>
using Matx32 = Matx<FT, 3, 2>;

/// @brief 3x3 matrix type (rotation, homography).
/// @tparam FT Floating-point type.
template <class FT>
using Matx33 = Matx<FT, 3, 3>;

/// @brief 3x4 matrix type (projection matrix).
/// @tparam FT Floating-point type.
template <class FT>
using Matx34 = Matx<FT, 3, 4>;

/// @brief 4x3 matrix type.
/// @tparam FT Floating-point type.
template <class FT>
using Matx43 = Matx<FT, 4, 3>;

/// @brief 4x4 matrix type (homogeneous transform).
/// @tparam FT Floating-point type.
template <class FT>
using Matx44 = Matx<FT, 4, 4>;

/// @}

/// @name Rotation Conversions
/// Functions for converting between different rotation representations:
/// Rodrigues vector, rotation matrix, quaternion, Euler angles.
/// @{

/// @brief Convert Rodrigues point to rotation matrix.
/// @tparam FT Floating-point type.
/// @param r Rodrigues rotation vector as Point3.
/// @return 3x3 rotation matrix.
template <class FT>
inline Matx33<FT> rodrigues(const Point3<FT> r) {
  Matx33<FT> m;
  Rodrigues(*reinterpret_cast<const Vec3<FT>*>(&r), m);
  return m;
}

/// @brief Convert Rodrigues vector to rotation matrix.
/// @tparam FT Floating-point type.
/// @param r Rodrigues rotation vector.
/// @return 3x3 rotation matrix.
template <class FT>
inline Matx33<FT> rodrigues(const Vec3<FT> r) {
  Matx33<FT> m;
  Rodrigues(r, m);
  return m;
}

/// @brief Convert rotation matrix to Rodrigues vector.
/// @tparam FT Floating-point type.
/// @param r 3x3 rotation matrix.
/// @return Rodrigues rotation vector.
template <class FT>
inline Vec3<FT> rodrigues(const Matx33<FT> r) {
  Vec3<FT> v;
  Rodrigues(r, v);
  return v;
}

/// @brief Convert rotation matrix to Rodrigues vector (explicit name).
/// @tparam FT Floating-point type.
/// @param r 3x3 rotation matrix.
/// @return Rodrigues rotation vector.
template <class FT>
inline Vec3<FT> rodriguesV(const Matx33<FT> r) {
  Vec3<FT> v;
  Rodrigues(r, v);
  return v;
}

/// @brief Convert Rodrigues vector to axis-angle representation.
/// @tparam FT Floating-point type.
/// @param r Rodrigues rotation vector.
/// @param[out] axis Rotation axis (unit vector).
/// @return Rotation angle in radians.
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

/// @brief Convert Rodrigues vector to axis-angle representation (Vec3 output).
/// @tparam FT Floating-point type.
/// @param r Rodrigues rotation vector.
/// @param[out] axis Rotation axis (unit vector).
/// @return Rotation angle in radians.
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

/// @brief Convert quaternion (x,y,z,w) to rotation matrix.
/// @tparam FT Floating-point type.
/// @param q Quaternion as Vec4 (x,y,z,w).
/// @return 3x3 rotation matrix.
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

/// @brief Convert rotation matrix to quaternion (x,y,z,w).
/// @tparam FT Floating-point type.
/// @param m 3x3 rotation matrix.
/// @return Quaternion as Vec4 (x,y,z,w).
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

/// @brief Convert Euler angles (yaw,pitch,roll) to rotation matrix.
/// @tparam FT Floating-point type.
/// @param e Euler angles as Vec3 (yaw,pitch,roll) in radians.
/// @return 3x3 rotation matrix.
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

/// @brief Convert Euler angles (yaw,pitch,roll) to rotation matrix (Point3 input).
/// @tparam FT Floating-point type.
/// @param e Euler angles as Point3 (yaw,pitch,roll) in radians.
/// @return 3x3 rotation matrix.
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

/// @brief Convert rotation matrix to Euler angles (yaw,pitch,roll).
/// @tparam FT Floating-point type.
/// @param rot 3x3 rotation matrix.
/// @return Euler angles as Vec3 (yaw,pitch,roll) in radians.
/// @note Handles singularities at north/south poles (gimbal lock).
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

/// @}

/// @name Homogeneous Coordinate Conversions
/// Functions for converting between Euclidean and homogeneous coordinates.
/// @{

/// @brief Convert homogeneous row/col vectors to normalized Euclidean vectors.
/// Converts a matrix of homogeneous row or column vectors to normalized
/// Euclidean coordinates by dividing by the last component. Handles 2D (3->2)
/// and 3D (4->3) conversions.
/// @tparam FT Floating-point type.
/// @param in Input matrix of homogeneous vectors.
/// @param[out] out Output matrix of normalized vectors.
/// @param cols If true, input vectors are columns; otherwise rows.
/// Output is always row vectors.
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

/// @brief Convert homogeneous vectors to normalized vectors (typed version).
/// @tparam FT Floating-point type.
/// @param in Input matrix of homogeneous vectors.
/// @param[out] out Output matrix of normalized vectors.
/// @param cols If true, input vectors are columns; otherwise rows.
template <class FT>
inline void fromHomogeneousT(const cv::Mat_<FT>& in, cv::Mat_<FT>& out, bool cols = false) {
  fromHomogeneous<FT>(in, out, cols);
}

/// @brief Convert homogeneous Point3 vector to normalized Point2 vector.
/// @tparam FT Floating-point type.
/// @param in Input vector of homogeneous 3D points.
/// @param[out] out Output vector of normalized 2D points.
template <class FT>
inline void fromHomogeneous(const std::vector<Point3<FT>>& in, std::vector<Point2<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  fromHomogeneous<FT>(matIn, matOut);
}

/// @brief Convert homogeneous Vec3 vector to normalized Vec2 vector.
/// @tparam FT Floating-point type.
/// @param in Input vector of homogeneous 3D vectors.
/// @param[out] out Output vector of normalized 2D vectors.
template <class FT>
inline void fromHomogeneous(const std::vector<Vec3<FT>>& in, std::vector<Vec2<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  fromHomogeneous<FT>(matIn, matOut);
}

/// @brief Convert homogeneous Vec4 vector to normalized Point3 vector.
/// @tparam FT Floating-point type.
/// @param in Input vector of homogeneous 4D vectors.
/// @param[out] out Output vector of normalized 3D points.
template <class FT>
inline void fromHomogeneous(const std::vector<Vec4<FT>>& in, std::vector<Point3<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  fromHomogeneous<FT>(matIn, matOut);
}

/// @brief Convert homogeneous Vec4 vector to normalized Vec3 vector.
/// @tparam FT Floating-point type.
/// @param in Input vector of homogeneous 4D vectors.
/// @param[out] out Output vector of normalized 3D vectors.
template <class FT>
inline void fromHomogeneous(const std::vector<Vec4<FT>>& in, std::vector<Vec3<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  fromHomogeneous<FT>(matIn, matOut);
}

/// @brief Convert Euclidean row/col vectors to homogeneous vectors.
/// Converts a matrix of Euclidean row or column vectors to homogeneous
/// coordinates by appending 1 as the last component. Handles 2D (2->3)
/// and 3D (3->4) conversions.
/// @tparam FT Floating-point type.
/// @param in Input matrix of Euclidean vectors.
/// @param[out] out Output matrix of homogeneous vectors.
/// @param cols If true, input vectors are columns; otherwise rows.
/// Output is always row vectors.
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

/// @brief Convert Euclidean vectors to homogeneous vectors (typed version).
/// @tparam FT Floating-point type.
/// @param in Input matrix of Euclidean vectors.
/// @param[out] out Output matrix of homogeneous vectors.
/// @param cols If true, input vectors are columns; otherwise rows.
template <class FT>
inline void toHomogeneousT(const cv::Mat_<FT>& in, cv::Mat_<FT>& out, bool cols = false) {
  toHomogeneous<FT>(in, out, cols);
}

/// @brief Convert Point3 vector to homogeneous Point2 vector.
/// @tparam FT Floating-point type.
/// @param in Input vector of 3D points.
/// @param[out] out Output vector of 2D homogeneous points.
template <class FT>
inline void toHomogeneous(const std::vector<Point3<FT>>& in, std::vector<Point2<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  toHomogeneous<FT>(matIn, matOut);
}

/// @brief Convert Vec3 vector to homogeneous Vec2 vector.
/// @tparam FT Floating-point type.
/// @param in Input vector of 3D vectors.
/// @param[out] out Output vector of 2D homogeneous vectors.
template <class FT>
inline void toHomogeneous(const std::vector<Vec3<FT>>& in, std::vector<Vec2<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  toHomogeneous<FT>(matIn, matOut);
}

/// @brief Convert Point3 vector to homogeneous Vec4 vector.
/// @tparam FT Floating-point type.
/// @param in Input vector of 3D points.
/// @param[out] out Output vector of 4D homogeneous vectors.
template <class FT>
inline void toHomogeneous(const std::vector<Point3<FT>>& in, std::vector<Vec4<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  toHomogeneous<FT>(matIn, matOut);
}

/// @brief Convert Vec3 vector to homogeneous Vec4 vector.
/// @tparam FT Floating-point type.
/// @param in Input vector of 3D vectors.
/// @param[out] out Output vector of 4D homogeneous vectors.
template <class FT>
inline void toHomogeneous(const std::vector<Vec3<FT>>& in, std::vector<Vec4<FT>>& out) {
  out.resize(in.size());
  cv::Mat matOut(out), matIn(in);
  toHomogeneous<FT>(matIn, matOut);
}

/// @}

/// @name Matrix-Vector Multiplication
/// Functions for transforming vectors by matrix multiplication.
/// @{

/// @brief Multiply matrix with row vectors from data.
/// @tparam FT Floating-point type.
/// @param mtx Transformation matrix.
/// @param data Input data matrix (row vectors).
/// @param[out] out Output matrix (transformed row vectors).
template <class FT>
inline void matxMul(cv::Mat mtx, cv::Mat data, cv::Mat& out) {
  if (data.channels() > 1) data = data.reshape(1);
  out = data * mtx.t();
}

/// @brief Multiply 4x4 matrix with Vec4 vector.
/// @tparam FT Floating-point type.
/// @param mtx 4x4 transformation matrix.
/// @param data Input Vec4 vector.
/// @param[out] out Output Vec4 vector.
template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Vec4<FT>>& data, std::vector<Vec4<FT>>& out) {
  CV_Assert(mtx.cols == 4 && mtx.rows == 4);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

/// @brief Multiply 3x3 matrix with Vec3 vector.
/// @tparam FT Floating-point type.
/// @param mtx 3x3 transformation matrix.
/// @param data Input Vec3 vector.
/// @param[out] out Output Vec3 vector.
template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Vec3<FT>>& data, std::vector<Vec3<FT>>& out) {
  CV_Assert(mtx.cols == 3 && mtx.rows == 3);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

/// @brief Multiply 2x2 matrix with Vec2 vector.
/// @tparam FT Floating-point type.
/// @param mtx 2x2 transformation matrix.
/// @param data Input Vec2 vector.
/// @param[out] out Output Vec2 vector.
template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Vec2<FT>>& data, std::vector<Vec2<FT>>& out) {
  CV_Assert(mtx.cols == 2 && mtx.rows == 2);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

/// @brief Multiply 3x3 matrix with Point3 vector.
/// @tparam FT Floating-point type.
/// @param mtx 3x3 transformation matrix.
/// @param data Input Point3 vector.
/// @param[out] out Output Point3 vector.
template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Point3<FT>>& data, std::vector<Point3<FT>>& out) {
  CV_Assert(mtx.cols == 3 && mtx.rows == 3);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

/// @brief Multiply 2x2 matrix with Point2 vector.
/// @tparam FT Floating-point type.
/// @param mtx 2x2 transformation matrix.
/// @param data Input Point2 vector.
/// @param[out] out Output Point2 vector.
template <class FT>
inline void matxMul(cv::Mat mtx, const std::vector<Point2<FT>>& data, std::vector<Point2<FT>>& out) {
  CV_Assert(mtx.cols == 2 && mtx.rows == 2);
  cv::Mat matData = cv::Mat(data);
  matData.reshape(1);
  cv::Mat tmp = matData * mtx.t();
  tmp.copyTo(out);
}

/// @brief Multiply 4x4 matrix with 3D vector (auto-convert to/from homogeneous).
/// @tparam FT Floating-point type.
/// @param mtx 4x4 homogeneous transformation matrix.
/// @param data Input Vec3 vector.
/// @return Transformed and normalized Vec3.
template <class FT>
inline Vec3<FT> matxMul(const Matx44<FT>& mtx, const Vec3<FT>& data) {
  Vec4<FT> tmp = mtx * Vec4<FT>(data[0], data[1], data[2], 1);
  FT n = tmp[3];
  return std::abs(n) < lsfm::LIMITS<FT>::tau() ? Vec3<FT>(0, 0, 0) : Vec3<FT>(tmp[0] / n, tmp[1] / n, tmp[2] / n);
}

/// @brief Multiply 4x4 matrix with 3D point (auto-convert to/from homogeneous).
/// @tparam FT Floating-point type.
/// @param mtx 4x4 homogeneous transformation matrix.
/// @param data Input Point3.
/// @return Transformed and normalized Vec3.
template <class FT>
inline Vec3<FT> matxMul(const Matx44<FT>& mtx, const Point3<FT>& data) {
  Vec4<FT> tmp = mtx * Vec4<FT>(data.x, data.y, data.z, 1);
  FT n = tmp[3];
  return std::abs(n) < lsfm::LIMITS<FT>::tau() ? Vec3<FT>(0, 0, 0) : Vec3<FT>(tmp[0] / n, tmp[1] / n, tmp[2] / n);
}

/// @brief Multiply 3x3 matrix with 2D vector (auto-convert to/from homogeneous).
/// @tparam FT Floating-point type.
/// @param mtx 3x3 homogeneous transformation matrix.
/// @param data Input Vec2 vector.
/// @return Transformed and normalized Vec2.
template <class FT>
inline Vec2<FT> matxMul(const Matx33<FT>& mtx, const Vec2<FT>& data) {
  Vec3<FT> tmp = mtx * Vec3<FT>(data[0], data[1], 1);
  FT n = tmp[2];
  return std::abs(n) < lsfm::LIMITS<FT>::tau() ? Vec2<FT>(0, 0) : Vec2<FT>(tmp[0] / n, tmp[1] / n);
}

/// @brief Multiply 3x3 matrix with 2D point (auto-convert to/from homogeneous).
/// @tparam FT Floating-point type.
/// @param mtx 3x3 homogeneous transformation matrix.
/// @param data Input Point2.
/// @return Transformed and normalized Vec2.
template <class FT>
inline Vec2<FT> matxMul(const Matx33<FT>& mtx, const Point2<FT>& data) {
  Vec3<FT> tmp = mtx * Vec3<FT>(data.x, data.y, 1);
  FT n = tmp[2];
  return std::abs(n) < lsfm::LIMITS<FT>::tau() ? Vec2<FT>(0, 0) : Vec2<FT>(tmp[0] / n, tmp[1] / n);
}

/// @brief Multiply 4x4 matrix with 3D direction vector (w=0).
/// Direction vectors are not translated, only rotated and scaled.
/// @tparam FT Floating-point type.
/// @param mtx 4x4 transformation matrix.
/// @param data Input direction Vec3.
/// @return Transformed direction Vec3.
template <class FT>
inline Vec3<FT> matxMulDir(const Matx44<FT>& mtx, const Vec3<FT>& data) {
  Vec4<FT> tmp = mtx * Vec4<FT>(data[0], data[1], data[2], 0);
  return Vec3<FT>(tmp[0], tmp[1]);
}

/// @brief Multiply 4x4 matrix with 3D direction point (w=0).
/// @tparam FT Floating-point type.
/// @param mtx 4x4 transformation matrix.
/// @param data Input direction Point3.
/// @return Transformed direction Vec3.
template <class FT>
inline Vec3<FT> matxMulDir(const Matx44<FT>& mtx, const Point3<FT>& data) {
  Vec4<FT> tmp = mtx * Vec4<FT>(data.x, data.y, data.z, 0);
  return Vec3<FT>(tmp[0], tmp[1]);
}

/// @brief Multiply 3x3 matrix with 2D direction vector (w=0).
/// @tparam FT Floating-point type.
/// @param mtx 3x3 transformation matrix.
/// @param data Input direction Vec2.
/// @return Transformed direction Vec2.
template <class FT>
inline Vec2<FT> matxMulDir(const Matx33<FT>& mtx, const Vec2<FT>& data) {
  Vec3<FT> tmp = mtx * Vec3<FT>(data[0], data[1], 0);
  return Vec2<FT>(tmp[0], tmp[1]);
}

/// @brief Multiply 3x3 matrix with 2D direction point (w=0).
/// @tparam FT Floating-point type.
/// @param mtx 3x3 transformation matrix.
/// @param data Input direction Point2.
/// @return Transformed direction Vec2.
template <class FT>
inline Vec2<FT> matxMulDir(const Matx33<FT>& mtx, const Point2<FT>& data) {
  Vec3<FT> tmp = mtx * Vec3<FT>(data.x, data.y, 0);
  return Vec2<FT>(tmp[0], tmp[1]);
}

/// @}

/// @name Operator Overloads
/// Multiplication operators for matrix-vector operations.
/// @{

/// @brief Multiply 4x4 matrix with vector of Vec4.
/// @tparam FT Floating-point type.
/// @param mtx Transformation matrix.
/// @param data Input vector.
/// @return Transformed vector.
template <class FT>
inline std::vector<Vec4<FT>> operator*(const Matx44<FT>& mtx, const std::vector<Vec4<FT>>& data) {
  std::vector<Vec4<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}

/// @brief Multiply 3x3 matrix with vector of Vec3.
/// @tparam FT Floating-point type.
/// @param mtx Transformation matrix.
/// @param data Input vector.
/// @return Transformed vector.
template <class FT>
inline std::vector<Vec3<FT>> operator*(const Matx33<FT>& mtx, const std::vector<Vec3<FT>>& data) {
  std::vector<Vec3<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}

/// @brief Multiply 2x2 matrix with vector of Vec2.
/// @tparam FT Floating-point type.
/// @param mtx Transformation matrix.
/// @param data Input vector.
/// @return Transformed vector.
template <class FT>
inline std::vector<Vec2<FT>> operator*(const Matx22<FT>& mtx, const std::vector<Vec2<FT>>& data) {
  std::vector<Vec2<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}

/// @brief Multiply 3x3 matrix with vector of Point3.
/// @tparam FT Floating-point type.
/// @param mtx Transformation matrix.
/// @param data Input vector.
/// @return Transformed vector.
template <class FT>
inline std::vector<Point3<FT>> operator*(const Matx33<FT>& mtx, const std::vector<Point3<FT>>& data) {
  std::vector<Point3<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}

/// @brief Multiply 2x2 matrix with vector of Point2.
/// @tparam FT Floating-point type.
/// @param mtx Transformation matrix.
/// @param data Input vector.
/// @return Transformed vector.
template <class FT>
inline std::vector<Point2<FT>> operator*(const Matx22<FT>& mtx, const std::vector<Point2<FT>>& data) {
  std::vector<Point2<FT>> ret;
  matxMul(mtx, data, ret);
  return ret;
}

/// @}

/// @brief OpenCV DataType specialization for lsfm::Matx.
/// Enables using lsfm::Matx types with OpenCV functions that require
/// DataType traits.
/// @tparam FT Element type.
/// @tparam m Number of rows.
/// @tparam n Number of columns.
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

/// @name Eigen/OpenCV Conversions
/// Functions for converting between Eigen-style and OpenCV matrix types.
/// @{

/// @brief Convert OpenCV Matx to lsfm Matx (Eigen-style).
/// @tparam FT Element type.
/// @tparam rows Number of rows.
/// @tparam cols Number of columns.
/// @param m OpenCV matrix.
/// @return lsfm matrix.
template <class FT, int rows, int cols>
Matx<FT, rows, cols> cv2eigen(const cv::Matx<FT, rows, cols>& m) {
  return Matx<FT, rows, cols>(&m(0));
}

/// @brief Convert lsfm Matx (Eigen-style) to OpenCV Matx.
/// @tparam FT Element type.
/// @tparam rows Number of rows.
/// @tparam cols Number of columns.
/// @param m lsfm matrix.
/// @return OpenCV matrix.
template <class FT, int rows, int cols>
cv::Matx<FT, rows, cols> cv2eigen(const Matx<FT, rows, cols>& m) {
  return cv::Matx<FT, rows, cols>(m.data());
}

/// @}

}  // namespace lsfm
