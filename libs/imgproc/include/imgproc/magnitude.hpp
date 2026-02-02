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

#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <string>

namespace lsfm {

/// @brief Enumeration of magnitude norm types.
///
/// Indicates how magnitude was computed from gradient components.
enum NormType {
  NONE = 0,   ///< Norm doesn't correspond to gx, gy
  NORM_L1,    ///< L1 norm: |gx| + |gy|
  NORM_L2,    ///< L2 norm: sqrt(gx² + gy²)
  NORM_L2SQR  ///< Squared L2 norm: gx² + gy²
};

/// @brief Quadratic (squared L2) magnitude computation.
///
/// Computes gx² + gy² without square root for speed.
/// Useful when relative magnitude ordering matters but not actual distance.
/// @tparam GT Gradient input type (short, float, or double)
/// @tparam MT Magnitude output type (int, float, or double)
template <class GT = short, class MT = int>
struct QuadraticMagnitude {
  QuadraticMagnitude() {}

  typedef GT grad_type;  ///< Gradient input type
  typedef MT mag_type;   ///< Magnitude output type

  /// @brief Compute squared magnitude of single value.
  /// @param val Input gradient value
  /// @return val² as magnitude type
  inline static MT single(GT val) { return static_cast<MT>(val) * val; }

  /// @brief Compute squared magnitude (double overload for thresholds).
  /// @param val Input value
  /// @return val²
  inline static double singled(double val) { return val * val; }

  /// @brief Compute quadratic magnitude from gradient components.
  /// @param gx X-direction gradient
  /// @param gy Y-direction gradient
  /// @return gx² + gy²
  inline static MT process(GT gx, GT gy) { return static_cast<MT>(gx) * gx + static_cast<MT>(gy) * gy; }

  /// @brief Compute quadratic magnitude for arrays.
  /// @param gx Pointer to X-gradient array
  /// @param gy Pointer to Y-gradient array
  /// @param[out] qmag Output magnitude array
  /// @param size Number of elements to process
  inline static void process(const GT* gx, const GT* gy, MT* qmag, size_t size) {
    for (size_t i = 0; i != size; ++i) qmag[i] = QuadraticMagnitude<GT, MT>::process(gx[i], gy[i]);
  }

  /// @brief Compute quadratic magnitude from cv::Mat gradient images.
  /// @param gx X-direction gradient matrix
  /// @param gy Y-direction gradient matrix
  /// @param[out] qmag Output magnitude matrix
  inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& qmag) {
    CV_Assert(gx.type() == gy.type() && (gx.type() & CV_MAT_DEPTH_MASK) == cv::DataType<GT>::type &&
              gx.rows == gy.rows && gx.cols == gy.cols);
    qmag.create(gx.size(), CV_MAKETYPE(cv::DataType<MT>::type, gx.channels()));
    QuadraticMagnitude<GT, MT>::process(
        gx.ptr<GT>(), gy.ptr<GT>(), qmag.ptr<MT>(),
        static_cast<size_t>(gx.rows) * static_cast<size_t>(gx.cols) * static_cast<size_t>(gx.channels()));
  }

  /// @brief Compute maximum magnitude from derivative max values.
  /// @param dm DerivativeMax containing maximum gradient values
  /// @param intensity Input intensity change (default: 1)
  /// @return Maximum possible magnitude
  inline static MT max(const DerivativeMax<GT>& dm, GT intensity = 1) {
    return QuadraticMagnitude<GT, MT>::process(dm.max_1st * intensity, dm.max_3rd * intensity);
  }

  /// @brief Get magnitude operator name.
  /// @return "qmag"
  inline static const std::string name() { return "qmag"; }

  /// @brief Get norm type.
  /// @return NormType::NORM_L2SQR
  inline static NormType normType() { return NormType::NORM_L2SQR; }
};

/// @brief Euclidean (L2) magnitude computation.
///
/// Computes sqrt(gx² + gy²) for true distance magnitude.
/// @tparam GT Gradient input type (short, float, or double)
/// @tparam MT Magnitude output type (must be float or double)
template <class GT = short, class MT = float>
struct Magnitude {
  Magnitude() {}

  typedef GT grad_type;  ///< Gradient input type
  typedef MT mag_type;   ///< Magnitude output type

  /// @brief Compute absolute magnitude of single value.
  /// @param val Input gradient value
  /// @return sqrt(val²) = |val|
  inline static MT single(GT val) { return std::sqrt(static_cast<MT>(val) * val); }

  /// @brief Compute absolute magnitude (double overload for thresholds).
  /// @param val Input value
  /// @return |val|
  inline static double singled(double val) { return std::sqrt(val * val); }

  /// @brief Compute Euclidean magnitude from gradient components.
  /// @param gx X-direction gradient
  /// @param gy Y-direction gradient
  /// @return sqrt(gx² + gy²)
  inline static MT process(GT gx, GT gy) { return std::sqrt(static_cast<MT>(gx) * gx + static_cast<MT>(gy) * gy); }

  /// @brief Compute Euclidean magnitude for arrays.
  /// @param gx Pointer to X-gradient array
  /// @param gy Pointer to Y-gradient array
  /// @param[out] mag Output magnitude array
  /// @param size Number of elements to process
  inline static void process(const GT* gx, const GT* gy, MT* mag, size_t size) {
    for (size_t i = 0; i != size; ++i) mag[i] = process(gx[i], gy[i]);
  }

  /// @brief Compute Euclidean magnitude from cv::Mat gradient images.
  ///
  /// Uses cv::magnitude() for float types for efficiency.
  /// @param gx X-direction gradient matrix
  /// @param gy Y-direction gradient matrix
  /// @param[out] mag Output magnitude matrix (float or double)
  inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& mag) {
    int type = gx.type() & CV_MAT_DEPTH_MASK;
    if ((type == CV_32F || type == CV_64F) && type == cv::DataType<MT>::type)
      cv::magnitude(gx, gy, mag);
    else {
      QuadraticMagnitude<GT, MT>::process(gx, gy, mag);
      cv::sqrt(mag, mag);
    }
  }

  /// @brief Compute maximum magnitude from derivative max values.
  /// @param dm DerivativeMax containing maximum gradient values
  /// @param intensity Input intensity change (default: 1)
  /// @return Maximum possible magnitude
  inline static MT max(const DerivativeMax<GT>& dm, GT intensity = 1) {
    return Magnitude<GT, MT>::process(dm.max_1st * intensity, dm.max_3rd * intensity);
  }

  /// @brief Get magnitude operator name.
  /// @return "mag"
  inline static const std::string name() { return "mag"; }

  /// @brief Get norm type.
  /// @return NormType::NORM_L2
  inline static NormType normType() { return NormType::NORM_L2; }
};


/// @brief Absolute (L1) magnitude computation.
///
/// Computes |gx| + |gy| for Manhattan distance magnitude.
/// Faster than L2 but provides less accurate distance.
/// @tparam GT Gradient input type (short, float, or double)
/// @tparam MT Magnitude output type (int, float, or double)
template <class GT = short, class MT = int>
struct AbsoluteMagnitude {
 public:
  AbsoluteMagnitude() {}

  typedef GT grad_type;  ///< Gradient input type
  typedef MT mag_type;   ///< Magnitude output type

  /// @brief Compute absolute magnitude of single value.
  /// @param val Input gradient value
  /// @return |val|
  inline static MT single(GT val) { return std::abs(static_cast<MT>(val)); }

  /// @brief Compute absolute magnitude (double overload for thresholds).
  /// @param val Input value
  /// @return |val|
  inline static double singled(double val) { return std::abs(val); }

  /// @brief Compute absolute magnitude from gradient components.
  /// @param gx X-direction gradient
  /// @param gy Y-direction gradient
  /// @return |gx| + |gy|
  inline static MT process(GT gx, GT gy) { return std::abs(static_cast<MT>(gx)) + std::abs(static_cast<MT>(gy)); }

  /// @brief Compute absolute magnitude for arrays.
  /// @param gx Pointer to X-gradient array
  /// @param gy Pointer to Y-gradient array
  /// @param[out] amag Output magnitude array
  /// @param size Number of elements to process
  inline static void process(const GT* gx, const GT* gy, MT* amag, size_t size) {
    for (size_t i = 0; i != size; ++i) amag[i] = AbsoluteMagnitude<GT, MT>::process(gx[i], gy[i]);
  }

  /// @brief Compute absolute magnitude from cv::Mat gradient images.
  /// @param gx X-direction gradient matrix
  /// @param gy Y-direction gradient matrix
  /// @param[out] amag Output magnitude matrix
  inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& amag) {
    CV_Assert(gx.type() == gy.type() && (gx.type() & CV_MAT_DEPTH_MASK) == cv::DataType<GT>::type &&
              gx.rows == gy.rows && gx.cols == gy.cols);
    // amag.create(gx.size(), CV_MAKETYPE(cv::DataType<MT>::type, gx.channels()));
    // AbsoluteMagnitude<GT, MT>::process(gx.ptr<GT>(), gy.ptr<GT>(), amag.ptr<MT>(), gx.rows*gx.cols*gx.channels());
    cv::add(cv::abs(gx), cv::abs(gy), amag);
  }

  /// @brief Compute maximum magnitude from derivative max values.
  /// @param dm DerivativeMax containing maximum gradient values
  /// @param intensity Input intensity change (default: 1)
  /// @return Maximum possible magnitude
  inline static MT max(const DerivativeMax<GT>& dm, GT intensity = 1) {
    return std::max(AbsoluteMagnitude<GT, MT>::process(dm.max_1st, dm.max_3rd),
                    AbsoluteMagnitude<GT, MT>::process(dm.max_2nd * intensity, dm.max_2nd * intensity));
  }

  /// @brief Get magnitude operator name.
  /// @return "amag"
  inline const std::string name() { return "amag"; }

  /// @brief Get norm type.
  /// @return NormType::NORM_L1
  inline static NormType normType() { return NormType::NORM_L1; }
};


}  // namespace lsfm
