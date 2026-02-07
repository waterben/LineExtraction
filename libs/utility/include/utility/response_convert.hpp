//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file response_convert.hpp
/// @brief Image response visualization utilities.

#pragma once


#include <opencv2/imgproc.hpp>
#include <utility/range.hpp>


namespace lsfm {

/// @brief Convert magnitude image to 8-bit for visualization.
/// @param src Input magnitude image.
/// @return 8-bit visualization image.
cv::Mat convertMag(const cv::Mat& src);

/// @brief Convert magnitude image with specified range.
/// @param src Input magnitude image.
/// @param range Min/max range for normalization.
/// @return 8-bit visualization image.
cv::Mat convertMag(const cv::Mat& src, const Range<double>& range);

/// @brief Convert magnitude image with explicit min/max.
/// @param src Input magnitude image.
/// @param vmin Minimum value for normalization.
/// @param vmax Maximum value for normalization.
/// @return 8-bit visualization image.
cv::Mat convertMag(const cv::Mat& src, double vmin, double vmax);

/// @brief Convert Laplacian response to 8-bit for visualization.
/// @param src Input Laplacian image.
/// @return 8-bit visualization image.
cv::Mat convertLaplace(const cv::Mat& src);

/// @brief Convert Laplacian response with specified range.
/// @param src Input Laplacian image.
/// @param range Min/max range for normalization.
/// @return 8-bit visualization image.
cv::Mat convertLaplace(const cv::Mat& src, const Range<double>& range);

/// @brief Convert Laplacian response with explicit min/max.
/// @param src Input Laplacian image.
/// @param vmin Minimum value for normalization.
/// @param vmax Maximum value for normalization.
/// @return 8-bit visualization image.
cv::Mat convertLaplace(const cv::Mat& src, double vmin, double vmax);

}  // namespace lsfm
