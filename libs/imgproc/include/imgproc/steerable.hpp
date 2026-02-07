//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file steerable.hpp
/// @brief Steerable Gaussian derivative filter generation.
///
/// This file provides functions for generating steerable Gaussian derivative
/// filters, which can be oriented to any angle by linear combination of
/// basis filters. Steerable filters are useful for detecting oriented
/// features at arbitrary angles without recomputing the filter.
/// @see Freeman, W.T. and Adelson, E.H., "The design and use of steerable
/// filters", IEEE PAMI, 1991.

#pragma once

#include <imgproc/gaussian.hpp>

namespace lsfm {

/// @brief Generate a steerable first-derivative Gaussian filter at a specified angle.
///
/// Creates a 2D kernel that responds to edges at the given orientation by
/// combining basis filters (G0 and G90) using the steering equation.
/// @tparam FT Floating-point type for kernel values (e.g., float, double).
/// @param[in] angle Desired filter orientation in radians.
/// @param[in] size Kernel size in pixels (should be odd, default: 5).
/// @param[in] range Standard deviation range for Gaussian (default: 3).
/// @return Steerable first-derivative Gaussian kernel oriented at the specified angle.
template <class FT>
cv::Mat_<FT> SteerGaussianD1(FT angle, int size = 5, FT range = 3) {
  cv::Mat_<FT> g = gaussian<FT>(size, range);
  cv::Mat_<FT> gd1 = gaussianD1<FT>(size, range);

  return cv::Mat_<FT>(cos(-angle) * (g.t() * gd1) + sin(-angle) * (gd1.t() * g));
}

/// @brief Steer a first-derivative Gaussian filter using precomputed basis filters.
///
/// Uses precomputed basis filter responses (k0 = G0, k1 = G90) to compute
/// the response at an arbitrary angle without recomputing the full convolution.
/// @tparam FT Floating-point type for calculations.
/// @param[in] angle Desired filter orientation in radians.
/// @param[in] Ik0 Pre-filtered image with horizontal basis filter (G0).
/// @param[in] Ik1 Pre-filtered image with vertical basis filter (G90).
/// @return Steered filter response at the specified angle.
template <class FT>
cv::Mat_<FT> SteerGaussianD1(double angle, const cv::Mat& Ik0, const cv::Mat& Ik1) {
  cv::Mat_<FT> k0, k1;

  if (cv::DataType<FT>::type() != Ik0.type())
    Ik0.convertTo(k0, cv::DataType<FT>::type());
  else
    k0 = Ik0;

  if (cv::DataType<FT>::type() != Ik1.type())
    Ik1.convertTo(k0, cv::DataType<FT>::type());
  else
    k1 = Ik1;

  return cos(angle) * k0 + sin(angle) * k1;
}

/// @brief Generate a steerable second-derivative Gaussian filter at a specified angle.
///
/// Creates a 2D kernel that responds to ridges/valleys at the given orientation
/// by combining three basis filters using the second-order steering equation.
/// @tparam FT Floating-point type for kernel values (e.g., float, double).
/// @param[in] angle Desired filter orientation in radians.
/// @param[in] size Kernel size in pixels (should be odd, default: 5).
/// @param[in] range Standard deviation range for Gaussian (default: 3).
/// @return Steerable second-derivative Gaussian kernel oriented at the specified angle.
template <class FT>
cv::Mat_<FT> SteerGaussianD2(FT angle, int size = 5, FT range = 3) {
  cv::Mat_<FT> g = gaussian<FT>(size, range);
  cv::Mat_<FT> gd1 = gaussianD1<FT>(size, range);
  cv::Mat_<FT> gd2 = gaussianD2<FT>(size, range);

  FT c = static_cast<FT>(cos(angle)), s = static_cast<FT>(sin(angle));

  return cv::Mat_<FT>((c * c) * (g.t() * gd2) + (s * s) * (gd2.t() * g) + (c * s * -2) * (gd1.t() * gd1));
}

/// @brief Steer a second-derivative Gaussian filter using precomputed basis filters.
///
/// Uses precomputed basis filter responses (k0 = G0°, k1 = G45°, k2 = G90°) to
/// compute the response at an arbitrary angle without recomputing the full convolution.
/// @tparam FT Floating-point type for calculations.
/// @param[in] angle Desired filter orientation in radians.
/// @param[in] Ik0 Pre-filtered image with 0° basis filter.
/// @param[in] Ik1 Pre-filtered image with 45° basis filter.
/// @param[in] Ik2 Pre-filtered image with 90° basis filter.
/// @return Steered filter response at the specified angle.
template <class FT>
cv::Mat_<FT> SteerGaussianD2(double angle, const cv::Mat& Ik0, const cv::Mat& Ik1, const cv::Mat& Ik2) {
  cv::Mat_<FT> k0, k1, k2;

  if (cv::DataType<FT>::type() != Ik0.type())
    Ik0.convertTo(k0, cv::DataType<FT>::type());
  else
    k0 = Ik0;

  if (cv::DataType<FT>::type() != Ik1.type())
    Ik1.convertTo(k1, cv::DataType<FT>::type());
  else
    k1 = Ik1;
  if (cv::DataType<FT>::type() != Ik2.type())
    Ik2.convertTo(k2, cv::DataType<FT>::type());
  else
    k2 = Ik2;

  FT c = cos(angle), s = sin(angle);

  return (c * c) * k0 + (s * s) * k2 + (c * s * -2) * k1;
}


}  // namespace lsfm
