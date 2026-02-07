//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file response_convert.cpp
/// @brief Response conversion utilities implementation.

#include <utility/response_convert.hpp>

#include <limits>

namespace lsfm {

cv::Mat convertMag(const cv::Mat& src) {
  double vmin, vmax;
  cv::minMaxIdx(src, &vmin, &vmax);
  return convertMag(src, vmin, vmax);
}

cv::Mat convertMag(const cv::Mat& src, const Range<double>& range) { return convertMag(src, range.lower, range.upper); }

cv::Mat convertMag(const cv::Mat& src, double vmin, double vmax) {
  cv::Mat mag;
  src.convertTo(mag, CV_32F);

  const double range = vmax - vmin;
  if (range > std::numeric_limits<double>::epsilon()) {
    mag -= static_cast<float>(vmin);
    mag /= static_cast<float>(range);
  } else {
    // Degenerate range: we have no contrast to scale, so return zeros for now.
    // Alternative options: keep the original values or warn the caller to provide
    // a sensible range if preserving contrast is required.
    mag.setTo(0);
  }

  cv::max(mag, 0.f, mag);
  cv::min(mag, 1.f, mag);
  mag *= 255.f;
  mag.convertTo(mag, CV_8U);
  return mag;
}

cv::Mat convertLaplace(const cv::Mat& src) {
  cv::Mat lap = cv::abs(src);
  return convertMag(lap);
}

cv::Mat convertLaplace(const cv::Mat& src, const Range<double>& range) {
  return convertLaplace(src, range.lower, range.upper);
}

cv::Mat convertLaplace(const cv::Mat& src, double vmin, double vmax) {
  cv::Mat lap = cv::abs(src);
  return convertMag(lap, 0, std::max(std::abs(vmin), std::abs(vmax)));
}

}  // namespace lsfm
