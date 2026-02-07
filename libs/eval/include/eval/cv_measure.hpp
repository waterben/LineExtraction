//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file cv_measure.hpp
/// @brief OpenCV-related measurement structures for evaluation.
/// Provides measurement types for computer vision evaluation metrics.

#pragma once

#include <eval/performance_measure.hpp>
#include <opencv2/core.hpp>

#include <cstdint>

namespace lsfm {

/// @brief OpenCV-based timing strategy using cv::getTickCount()
///
/// Uses OpenCV's high-resolution timer which is optimized for
/// computer vision benchmarking.
struct CVTimingStrategy : public TimingStrategy {
  std::uint64_t now() const override { return static_cast<std::uint64_t>(cv::getTickCount()); }

  double to_milliseconds(std::uint64_t ticks) const override {
    return static_cast<double>(ticks * 1000) / cv::getTickFrequency();
  }
};

/// @brief Default CV timing strategy instance
inline const CVTimingStrategy& cvTimingStrategy() {
  static CVTimingStrategy strategy;
  return strategy;
}

/// @brief CV-specific performance measure with image dimensions
///
/// This class adds computer vision specific fields (width, height)
/// that are needed for mega-pixel calculations in CV benchmarks.
///
/// For backward compatibility, this is aliased as PerformanceMeasure
/// since the original PerformanceMeasure always included CV fields.
struct CVPerformanceMeasure : public PerformanceMeasureBase {
  /// Default constructor
  CVPerformanceMeasure() = default;

  /// @brief Constructor with names only (for accumulation)
  CVPerformanceMeasure(const std::string& sn, const std::string& tn) : PerformanceMeasureBase(sn, tn) {}

  /// @brief Constructor with all parameters
  CVPerformanceMeasure(const std::string& sn, const std::string& tn, double w, double h)
      : PerformanceMeasureBase(sn, tn), width(w), height(h) {}

  /// @brief Constructor with int dimensions
  CVPerformanceMeasure(const std::string& sn, const std::string& tn, int w, int h)
      : PerformanceMeasureBase(sn, tn), width(static_cast<double>(w)), height(static_cast<double>(h)) {}

  /// Image dimensions (for mega pixel calculation)
  double width{0.0};
  double height{0.0};

  /// @brief Compute result using CV timing strategy
  PerformanceResult computeResult() const { return PerformanceMeasureBase::computeResult(cvTimingStrategy()); }

  /// @brief Static version using CV timing
  static PerformanceResult computeResult(const std::vector<std::uint64_t>& data) {
    return PerformanceMeasureBase::computeResult(data, cvTimingStrategy());
  }

  /// @brief Clear all data including dimensions.
  void clear() override {
    PerformanceMeasureBase::clear();
    width = 0.0;
    height = 0.0;
  }

  /// @brief Append a single duration (override to use correct type)
  void append(std::uint64_t duration) { durations.push_back(duration); }

  /// @brief Append from another CV measure (accumulates dimensions)
  void append(const CVPerformanceMeasure& other) {
    PerformanceMeasureBase::append(other);
    width += other.width;
    height += other.height;
  }

  /// @brief Calculate mega pixels
  double megaPixels() const { return (width * height) / 1'000'000.0; }
};

using CVPerformanceMeasureVector = std::vector<CVPerformanceMeasure>;

// Legacy compatibility: PerformanceMeasure was historically CV-specific
// New code should use CVPerformanceMeasure explicitly or the generic PerformanceMeasureBase
using PerformanceMeasure = CVPerformanceMeasure;
using PerformanceMeasureVector = CVPerformanceMeasureVector;

/// @brief Accumulate multiple CV performance measures into one
inline CVPerformanceMeasure accumulateCVMeasures(const CVPerformanceMeasureVector& measures,
                                                 const std::string& source_name = std::string(),
                                                 const std::string& task_name = std::string()) {
  CVPerformanceMeasure ret{source_name, task_name};
  if (measures.empty()) return ret;

  for (const auto& pm : measures) {
    ret.height += pm.height;
    ret.width += pm.width;
    ret.durations.insert(ret.durations.end(), pm.durations.begin(), pm.durations.end());
  }
  const double count = static_cast<double>(measures.size());
  ret.width /= count;
  ret.height /= count;
  return ret;
}

}  // namespace lsfm
