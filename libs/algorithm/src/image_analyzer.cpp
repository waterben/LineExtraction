//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file image_analyzer.cpp
/// @brief Implementation of ImageAnalyzer and ImageProperties::suggest_profile.

#include <algorithm/image_analyzer.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace lsfm {

// ============================================================================
// ImageAnalyzer
// ============================================================================

ImageProperties ImageAnalyzer::analyze(const cv::Mat& image) {
  if (image.empty()) {
    throw std::invalid_argument("ImageAnalyzer::analyze: image is empty");
  }

  // Convert to 8-bit grayscale if needed
  cv::Mat gray;
  if (image.channels() > 1) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }

  if (gray.depth() != CV_8U) {
    cv::Mat tmp;
    gray.convertTo(tmp, CV_8U, 255.0);
    gray = tmp;
  }

  ImageProperties props;
  props.contrast = compute_contrast(gray);
  props.noise_level = compute_noise(gray);
  props.edge_density = compute_edge_density(gray);
  props.dynamic_range = compute_dynamic_range(gray);

  return props;
}

double ImageAnalyzer::compute_contrast(const cv::Mat& gray) {
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(gray, mean, stddev);

  // Normalize: max possible stddev for 8-bit is ~127.5
  constexpr double kMaxStddev = 127.5;
  return std::clamp(stddev[0] / kMaxStddev, 0.0, 1.0);
}

double ImageAnalyzer::compute_noise(const cv::Mat& gray) {
  // Robust noise estimate: sigma = MAD(Laplacian) * 1.4826
  // Reference: Immerkær "Fast Noise Variance Estimation" (1996)
  cv::Mat laplacian;
  cv::Laplacian(gray, laplacian, CV_64F);

  // Compute absolute values
  cv::Mat abs_lap;
  laplacian = cv::abs(laplacian);
  laplacian.convertTo(abs_lap, CV_64F);

  // Sort to find median
  cv::Mat flat = abs_lap.reshape(1, 1);
  cv::Mat sorted;
  cv::sort(flat, sorted, cv::SORT_ASCENDING);
  double median = sorted.at<double>(0, sorted.cols / 2);

  // MAD * 1.4826 gives robust sigma estimate
  double sigma = median * 1.4826;

  // Normalize: a sigma of ~30 on 8-bit is already very noisy
  constexpr double kMaxNoiseSigma = 30.0;
  return std::clamp(sigma / kMaxNoiseSigma, 0.0, 1.0);
}

double ImageAnalyzer::compute_edge_density(const cv::Mat& gray) {
  // Compute gradient magnitude via Sobel
  cv::Mat grad_x;
  cv::Mat grad_y;
  cv::Sobel(gray, grad_x, CV_64F, 1, 0, 3);
  cv::Sobel(gray, grad_y, CV_64F, 0, 1, 3);

  cv::Mat magnitude;
  cv::magnitude(grad_x, grad_y, magnitude);

  // Threshold: pixels with gradient > mean + stddev are "edge" pixels
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(magnitude, mean, stddev);
  double threshold = mean[0] + stddev[0];

  int edge_pixels = cv::countNonZero(magnitude > threshold);
  int total_pixels = gray.rows * gray.cols;

  if (total_pixels == 0) {
    return 0.0;
  }

  // Edge density typically 5–25% for natural images
  // Normalize so that ~30% edge pixels → 1.0
  constexpr double kMaxEdgeFraction = 0.30;
  double fraction = static_cast<double>(edge_pixels) / static_cast<double>(total_pixels);
  return std::clamp(fraction / kMaxEdgeFraction, 0.0, 1.0);
}

double ImageAnalyzer::compute_dynamic_range(const cv::Mat& gray) {
  // Compute histogram
  constexpr int kHistSize = 256;
  float range[] = {0.0F, 256.0F};
  const float* hist_range = range;
  cv::Mat hist;
  cv::calcHist(&gray, 1, nullptr, cv::Mat(), hist, 1, &kHistSize, &hist_range);

  int total = gray.rows * gray.cols;
  if (total == 0) {
    return 0.0;
  }

  // Find 5th and 95th percentile
  int p5_idx = 0;
  int p95_idx = 255;
  constexpr double kP5Frac = 0.05;
  constexpr double kP95Frac = 0.95;
  double cumsum = 0;

  for (int i = 0; i < kHistSize; ++i) {
    cumsum += static_cast<double>(hist.at<float>(i));
    if (cumsum / static_cast<double>(total) >= kP5Frac) {
      p5_idx = i;
      break;
    }
  }

  cumsum = 0;
  for (int i = 0; i < kHistSize; ++i) {
    cumsum += static_cast<double>(hist.at<float>(i));
    if (cumsum / static_cast<double>(total) >= kP95Frac) {
      p95_idx = i;
      break;
    }
  }

  // Normalize spread to [0, 1]
  constexpr double kMaxSpread = 255.0;
  double spread = static_cast<double>(p95_idx - p5_idx);
  return std::clamp(spread / kMaxSpread, 0.0, 1.0);
}

// ============================================================================
// ImageProperties::suggest_profile
// ============================================================================

ProfileHints ImageProperties::suggest_profile() const {
  ProfileHints hints;

  // Detail: high contrast + low noise → suggest more detail
  // Low contrast or high noise → suggest less detail (fewer false detections)
  hints.detail = std::clamp(50.0 + (contrast - 0.3) * 100.0 - noise_level * 60.0, 10.0, 90.0);

  // Gap tolerance: noisy images need more gap tolerance;
  // high edge density means real edges are dense, less gap tolerance needed
  hints.gap_tolerance = std::clamp(50.0 + noise_level * 40.0 - edge_density * 30.0, 10.0, 90.0);

  // Min length: noisy or edge-dense images → prefer longer segments (filter noise)
  // Clean, sparse images → shorter segments are fine
  hints.min_length = std::clamp(30.0 + noise_level * 40.0 + edge_density * 20.0, 10.0, 90.0);

  // Precision: high dynamic range + low noise → high precision feasible
  hints.precision = std::clamp(50.0 + dynamic_range * 30.0 - noise_level * 40.0, 10.0, 90.0);

  // Adaptive factors
  // Low contrast → raise thresholds (factor > 1), high contrast → lower them
  hints.contrast_factor = std::clamp(1.0 + (0.4 - contrast) * 1.5, 0.5, 2.0);

  // High noise → raise thresholds (factor > 1)
  hints.noise_factor = std::clamp(1.0 + noise_level * 1.5, 0.5, 2.0);

  hints.clamp();
  return hints;
}

}  // namespace lsfm
