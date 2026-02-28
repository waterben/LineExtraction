//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file image_analyzer.hpp
/// @brief Image analysis for adaptive detector profile parameter estimation.
///
/// Provides ImageProperties (contrast, noise, edge density, dynamic range)
/// computed from a given image, and ProfileHints suggesting intuitive knob
/// values for a DetectorProfile.

#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lsfm {

/// @brief Measured image properties, each normalized to [0, 1].
///
/// These properties characterize the image content and are used
/// to suggest adaptive DetectorProfile knob values.
struct ImageProperties {
  /// @brief Contrast: normalized standard deviation of intensity.
  /// 0 = flat / uniform image, 1 = maximum contrast.
  double contrast = 0;

  /// @brief Noise level: normalized median absolute deviation of the Laplacian.
  /// 0 = noise-free, 1 = extremely noisy.
  double noise_level = 0;

  /// @brief Edge density: fraction of strong-gradient pixels.
  /// 0 = no edges, 1 = edges everywhere.
  double edge_density = 0;

  /// @brief Dynamic range: normalized percentile spread (5th–95th).
  /// 0 = narrow histogram, 1 = full dynamic range.
  double dynamic_range = 0;

  /// @brief Suggest default profile knobs based on these image properties.
  ///
  /// Returns a set of ProfileHints with suggested values for each knob.
  /// The hints are heuristic and can be overridden by the user.
  /// @return Suggested ProfileHints.
  struct ProfileHints suggest_profile() const;
};

/// @brief Suggested intuitive knob values for a DetectorProfile.
///
/// Each knob is in [0, 100] percent. Additionally, adaptive scaling
/// factors modulate the parameter translation in DetectorProfile.
struct ProfileHints {
  /// @brief Detail level: 0 = coarse / salient only, 100 = fine details.
  double detail = 50;

  /// @brief Gap tolerance: 0 = no gaps allowed, 100 = very tolerant of gaps.
  double gap_tolerance = 50;

  /// @brief Minimum length: 0 = keep all (even tiny), 100 = only long segments.
  double min_length = 50;

  /// @brief Precision: 0 = rough / fast, 100 = sub-pixel accurate.
  double precision = 50;

  /// @brief Adaptive contrast factor: scales threshold parameters.
  /// Derived from ImageProperties::contrast. Range [0.5, 2.0].
  double contrast_factor = 1.0;

  /// @brief Adaptive noise factor: scales threshold parameters.
  /// Derived from ImageProperties::noise_level. Range [0.5, 2.0].
  double noise_factor = 1.0;

  /// @brief Clamp all knob values to [0, 100] and factors to [0.5, 2.0].
  void clamp() {
    detail = std::clamp(detail, 0.0, 100.0);
    gap_tolerance = std::clamp(gap_tolerance, 0.0, 100.0);
    min_length = std::clamp(min_length, 0.0, 100.0);
    precision = std::clamp(precision, 0.0, 100.0);
    contrast_factor = std::clamp(contrast_factor, 0.5, 2.0);
    noise_factor = std::clamp(noise_factor, 0.5, 2.0);
  }
};

/// @brief Analyze an image to extract measurable properties.
///
/// Uses simple OpenCV operations (meanStdDev, Laplacian, Sobel)
/// to compute contrast, noise, edge density, and dynamic range.
/// All outputs are normalized to [0, 1].
///
/// @example
/// @code{cpp}
/// cv::Mat img = cv::imread("photo.png", cv::IMREAD_GRAYSCALE);
/// auto props = ImageAnalyzer::analyze(img);
/// auto hints = props.suggest_profile();
/// // Use hints.detail, hints.gap_tolerance, etc. with DetectorProfile
/// @endcode
class ImageAnalyzer {
 public:
  ImageAnalyzer() = delete;

  /// @brief Analyze a grayscale image and return normalized properties.
  ///
  /// The input image must be single-channel (CV_8U or CV_32F/CV_64F).
  /// If 8-bit, values are normalized to [0, 255] range internally.
  ///
  /// @param image Input grayscale image (single channel).
  /// @return Computed ImageProperties with all fields in [0, 1].
  /// @throws std::invalid_argument if image is empty or not single-channel.
  static ImageProperties analyze(const cv::Mat& image);

 private:
  /// @brief Compute contrast as normalized intensity standard deviation.
  /// @param gray Grayscale image (CV_8U).
  /// @return Contrast in [0, 1].
  static double compute_contrast(const cv::Mat& gray);

  /// @brief Compute noise level via median absolute deviation of Laplacian.
  /// @param gray Grayscale image (CV_8U).
  /// @return Noise level in [0, 1].
  static double compute_noise(const cv::Mat& gray);

  /// @brief Compute edge density as fraction of strong-gradient pixels.
  /// @param gray Grayscale image (CV_8U).
  /// @return Edge density in [0, 1].
  static double compute_edge_density(const cv::Mat& gray);

  /// @brief Compute dynamic range from 5th-to-95th percentile spread.
  /// @param gray Grayscale image (CV_8U).
  /// @return Dynamic range in [0, 1].
  static double compute_dynamic_range(const cv::Mat& gray);
};

}  // namespace lsfm
