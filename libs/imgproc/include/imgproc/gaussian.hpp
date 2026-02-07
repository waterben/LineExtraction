//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file gaussian.hpp
/// @brief Gaussian kernel generation utilities.

#pragma once

#include <opencv2/core/core.hpp>

#include <cmath>

namespace lsfm {

/// @brief Generate a 1D Gaussian kernel.
///
/// Creates a discrete Gaussian with sigma = 1 and no normalization.
/// The range parameter controls the effective spread (larger range = narrower curve).
/// @tparam FT Floating-point type for kernel values (float or double)
/// @param size Kernel size (must be odd, minimum 3, default: 5)
/// @param range Value range for Gaussian curve (larger = narrower, default: 3)
/// @return 1xN matrix containing Gaussian values
/// @note Size is adjusted to be odd if even value is provided
template <class FT>
cv::Mat_<FT> gaussian(int size = 5, FT range = 3) {
  // size have to be odd and in sane range
  if (size < 3) size = 3;
  if (size % 2 == 0) ++size;

  int s2 = (size - 1) / 2;

  FT step = range / static_cast<FT>(s2);

  cv::Mat_<FT> g(1, size, cv::DataType<FT>::type);

  FT* pg = &g(s2);
  pg[0] = std::exp(FT(0));

  for (int i = 1; i <= s2; ++i) {
    FT x = static_cast<FT>(i) * step;
    pg[-i] = pg[i] = std::exp(-x * x / 2);
  }

  return g;
}

/// @brief Generate a 1D first derivative of Gaussian kernel.
///
/// Creates the first derivative of a Gaussian (edge detection kernel).
/// Antisymmetric kernel with zero at center.
/// @tparam FT Floating-point type for kernel values (float or double)
/// @param size Kernel size (must be odd, minimum 3, default: 5)
/// @param range Value range for Gaussian curve (larger = narrower, default: 3)
/// @return 1xN matrix containing first derivative of Gaussian values
template <class FT>
cv::Mat_<FT> gaussianD1(int size = 5, FT range = 3) {
  // size have to be odd and in sane range
  if (size < 3) size = 3;
  if (size % 2 == 0) ++size;

  int s2 = (size - 1) / 2;

  FT step = range / static_cast<FT>(s2);

  cv::Mat_<FT> g(1, size, cv::DataType<FT>::type);

  FT* pg = &g(s2);
  pg[0] = 0;

  for (int i = 1; i <= s2; ++i) {
    FT x = static_cast<FT>(i) * step;
    x *= std::exp(-x * x / 2);
    pg[-i] = -x;
    pg[i] = x;
  }

  return g;
}


/// @brief Generate a 1D second derivative of Gaussian kernel (Laplacian of Gaussian).
///
/// Creates the second derivative of a Gaussian (blob/ridge detection kernel).
/// Symmetric kernel with negative value at center (Mexican hat shape in 2D).
/// @tparam FT Floating-point type for kernel values (float or double)
/// @param size Kernel size (must be odd, minimum 3, default: 5)
/// @param range Value range for Gaussian curve (larger = narrower, default: 3)
/// @return 1xN matrix containing second derivative of Gaussian values
template <class FT>
cv::Mat_<FT> gaussianD2(int size = 5, FT range = 3) {
  // size have to be odd and in sane range
  if (size < 3) size = 3;
  if (size % 2 == 0) ++size;

  int s2 = (size - 1) / 2;

  FT step = range / static_cast<FT>(s2);

  cv::Mat_<FT> g(1, size, cv::DataType<FT>::type);

  FT* pg = &g(s2);
  pg[0] = -std::exp(FT(0));

  for (int i = 1; i <= s2; ++i) {
    FT x2 = static_cast<FT>(i) * step;
    x2 *= x2;
    pg[-i] = pg[i] = (x2 - 1) * std::exp(-x2 / 2);
  }

  return g;
}

}  // namespace lsfm
