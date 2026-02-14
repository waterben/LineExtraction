//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file accuracy_measure.hpp
/// @brief Accuracy metrics for line segment detection evaluation.
///
/// Provides precision, recall, F1-score, and structural AP (sAP) metrics
/// for comparing detected line segments against ground truth.

#pragma once

#include <geometry/line.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace lsfm {

/// @brief Result of accuracy evaluation.
struct AccuracyResult {
  double precision{0};     ///< True positives / (true positives + false positives).
  double recall{0};        ///< True positives / (true positives + false negatives).
  double f1{0};            ///< Harmonic mean of precision and recall.
  int true_positives{0};   ///< Number of detected segments matching ground truth.
  int false_positives{0};  ///< Number of detected segments without match.
  int false_negatives{0};  ///< Number of ground truth segments without match.

  /// @brief Compute F1 from current precision and recall values.
  void compute_f1() {
    if (precision + recall > 0) {
      f1 = 2.0 * precision * recall / (precision + recall);
    } else {
      f1 = 0;
    }
  }
};

/// @brief Compute accuracy metrics for line segment detection.
///
/// Matches detected line segments to ground truth using endpoint distance.
/// A detected segment matches a ground truth segment if the sum of
/// endpoint-to-endpoint distances (best permutation) is below the threshold.
///
/// @tparam FT Floating-point type.
template <class FT>
class AccuracyMeasure {
 public:
  using LineSegmentType = LineSegment<FT>;
  using LineSegmentVector = std::vector<LineSegmentType>;

  /// @brief Construct with matching threshold.
  /// @param threshold Maximum average endpoint distance for a match (pixels).
  explicit AccuracyMeasure(double threshold = 5.0) : threshold_(threshold) {}

  /// @brief Get/set the matching threshold.
  /// @return Current threshold in pixels.
  double threshold() const { return threshold_; }

  /// @brief Set the matching threshold.
  /// @param t New threshold in pixels.
  void set_threshold(double t) { threshold_ = t; }

  /// @brief Evaluate accuracy of detected lines against ground truth.
  /// @param detected Detected line segments.
  /// @param ground_truth Ground truth line segments.
  /// @return AccuracyResult with precision, recall, and F1.
  AccuracyResult evaluate(const LineSegmentVector& detected, const LineSegmentVector& ground_truth) const {
    AccuracyResult result;
    if (ground_truth.empty() && detected.empty()) {
      result.precision = 1.0;
      result.recall = 1.0;
      result.f1 = 1.0;
      return result;
    }
    if (detected.empty()) {
      result.false_negatives = static_cast<int>(ground_truth.size());
      return result;
    }
    if (ground_truth.empty()) {
      result.false_positives = static_cast<int>(detected.size());
      return result;
    }

    // Greedy matching: for each ground truth, find best unmatched detection
    std::vector<bool> det_matched(detected.size(), false);
    std::vector<bool> gt_matched(ground_truth.size(), false);

    // Build distance matrix
    std::vector<std::tuple<double, std::size_t, std::size_t>> pairs;
    pairs.reserve(detected.size() * ground_truth.size());

    for (std::size_t gi = 0; gi < ground_truth.size(); ++gi) {
      for (std::size_t di = 0; di < detected.size(); ++di) {
        double d = segment_distance(detected[di], ground_truth[gi]);
        if (d <= threshold_) {
          pairs.emplace_back(d, gi, di);
        }
      }
    }

    // Sort by distance, greedily assign matches
    std::sort(pairs.begin(), pairs.end());

    for (const auto& [dist, gi, di] : pairs) {
      if (!gt_matched[gi] && !det_matched[di]) {
        gt_matched[gi] = true;
        det_matched[di] = true;
        result.true_positives++;
      }
    }

    result.false_positives = static_cast<int>(detected.size()) - result.true_positives;
    result.false_negatives = static_cast<int>(ground_truth.size()) - result.true_positives;

    if (result.true_positives + result.false_positives > 0) {
      result.precision = static_cast<double>(result.true_positives) /
                         static_cast<double>(result.true_positives + result.false_positives);
    }
    if (result.true_positives + result.false_negatives > 0) {
      result.recall = static_cast<double>(result.true_positives) /
                      static_cast<double>(result.true_positives + result.false_negatives);
    }
    result.compute_f1();
    return result;
  }

  /// @brief Compute structural AP (sAP) at multiple thresholds.
  ///
  /// Evaluates accuracy at the given thresholds and averages the AP scores.
  /// @param detected Detected line segments.
  /// @param ground_truth Ground truth line segments.
  /// @param thresholds Vector of distance thresholds to evaluate at.
  /// @return Average AP across all thresholds.
  double structural_ap(const LineSegmentVector& detected,
                       const LineSegmentVector& ground_truth,
                       const std::vector<double>& thresholds = {5, 10, 15}) const {
    if (thresholds.empty()) return 0;
    double sum = 0;
    AccuracyMeasure<FT> eval(threshold_);
    for (double t : thresholds) {
      eval.set_threshold(t);
      AccuracyResult r = eval.evaluate(detected, ground_truth);
      sum += r.f1;
    }
    return sum / static_cast<double>(thresholds.size());
  }

 private:
  double threshold_;  ///< Maximum average endpoint distance for match.

  /// @brief Compute distance between two line segments.
  ///
  /// Returns the minimum of the two possible endpoint-to-endpoint
  /// distance sums (forward and reversed), averaged.
  /// @param a First segment.
  /// @param b Second segment.
  /// @return Average endpoint distance.
  static double segment_distance(const LineSegmentType& a, const LineSegmentType& b) {
    auto a_s = a.startPoint(), a_e = a.endPoint();
    auto b_s = b.startPoint(), b_e = b.endPoint();
    double d_forward = endpoint_dist(a_s, b_s) + endpoint_dist(a_e, b_e);
    double d_reverse = endpoint_dist(a_s, b_e) + endpoint_dist(a_e, b_s);
    return std::min(d_forward, d_reverse) / 2.0;
  }

  /// @brief Euclidean distance between two points.
  /// @tparam PT Point type supporting getX/getY accessors.
  template <class PT>
  static double endpoint_dist(const PT& a, const PT& b) {
    double dx = static_cast<double>(getX(a)) - static_cast<double>(getX(b));
    double dy = static_cast<double>(getY(a)) - static_cast<double>(getY(b));
    return std::sqrt(dx * dx + dy * dy);
  }
};

}  // namespace lsfm
