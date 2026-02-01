#pragma once

#include <eval/measure.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace lsfm {

/// @brief Performance results in milliseconds
struct PerformanceResult {
  double total{};
  double mean{};
  double stddev{};
};

/// @brief Timing strategy interface (for dependency injection)
///
/// Abstracts the timing mechanism, allowing different backends:
/// - std::chrono (default, portable)
/// - OpenCV cv::getTickCount() (for CV applications)
/// - Custom high-resolution timers
struct TimingStrategy {
  virtual ~TimingStrategy() = default;

  /// @brief Get current timestamp in nanoseconds
  virtual std::uint64_t now() const = 0;

  /// @brief Convert ticks to milliseconds
  virtual double to_milliseconds(std::uint64_t ticks) const = 0;
};

/// @brief Standard chrono-based timing strategy
struct ChronoTimingStrategy : public TimingStrategy {
  std::uint64_t now() const override {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::high_resolution_clock::now().time_since_epoch())
                                          .count());
  }

  double to_milliseconds(std::uint64_t ticks) const override {
    return static_cast<double>(ticks) / 1'000'000.0;  // ns to ms
  }
};

/// @brief Generic performance measure (no CV-specific fields)
///
/// Design principles:
/// - Contains only timing-related data
/// - Domain-specific data (width/height) goes into derived classes or metadata
/// - Supports pluggable timing strategies
///
/// @note For backward compatibility, use PerformanceMeasure (alias to CVPerformanceMeasure)
///       when CV-specific features (width/height) are needed.
struct PerformanceMeasureBase : public Measure {
  using Result = PerformanceResult;

  /// Default constructor
  PerformanceMeasureBase() = default;

  /// @brief Constructor with names only (for accumulation)
  PerformanceMeasureBase(const std::string& sn, const std::string& tn) {
    source_name = sn;
    task_name = tn;
  }

  /// Measurements in ticks (strategy-specific units)
  /// @note Legacy code uses 'measures' - now renamed to 'durations' for clarity
  std::vector<std::uint64_t> durations{};

  /// @brief Compute result using given timing strategy
  PerformanceResult computeResult(const TimingStrategy& strategy) const { return computeResult(durations, strategy); }

  /// @brief Compute result from duration vector using given strategy
  static PerformanceResult computeResult(const std::vector<std::uint64_t>& data, const TimingStrategy& strategy) {
    PerformanceResult ret{};
    if (data.empty()) return ret;

    double sqr_sum = 0;
    for (auto duration : data) {
      double ms = strategy.to_milliseconds(duration);
      ret.total += ms;
      sqr_sum += ms * ms;
    }
    ret.mean = ret.total / static_cast<double>(data.size());
    ret.stddev = std::sqrt(std::abs(sqr_sum / static_cast<double>(data.size()) - ret.mean * ret.mean));
    return ret;
  }

  void clear() override {
    Measure::clear();
    durations.clear();
  }

  /// @brief Append a single duration
  void append(std::uint64_t duration) { durations.push_back(duration); }

  /// @brief Append durations from another measure
  void append(const PerformanceMeasureBase& other) {
    durations.insert(durations.end(), other.durations.begin(), other.durations.end());
  }
};

using PerformanceMeasureBaseVector = std::vector<PerformanceMeasureBase>;

/// @brief Accumulate multiple performance measures into one
inline PerformanceMeasureBase accumulateBaseMeasures(const PerformanceMeasureBaseVector& measures,
                                                     const std::string& source_name = std::string(),
                                                     const std::string& task_name = std::string()) {
  PerformanceMeasureBase ret{source_name, task_name};
  if (measures.empty()) return ret;

  for (const auto& pm : measures) {
    ret.durations.insert(ret.durations.end(), pm.durations.begin(), pm.durations.end());
  }
  return ret;
}

/// @brief Default timing strategy (chrono-based)
inline const ChronoTimingStrategy& defaultTimingStrategy() {
  static ChronoTimingStrategy strategy;
  return strategy;
}

}  // namespace lsfm
