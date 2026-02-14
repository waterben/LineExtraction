//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file param_search.hpp
/// @brief Parameter search optimizer for line detection algorithms.
///
/// Provides ParamOptimizer that systematically evaluates parameter
/// combinations using a SearchStrategy and AccuracyMeasure, finding
/// optimal parameters for any algorithm implementing ValueManager.

#pragma once

#include <algorithm/accuracy_measure.hpp>
#include <algorithm/ground_truth.hpp>
#include <algorithm/search_strategy.hpp>
#include <eval/cv_performance_task.hpp>
#include <eval/data_provider.hpp>
#include <utility/value_manager.hpp>

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace lsfm {

/// @brief Metric to optimize for.
enum class OptimMetric : int {
  F1 = 0,         ///< Optimize for F1 score.
  PRECISION = 1,  ///< Optimize for precision.
  RECALL = 2      ///< Optimize for recall.
};

/// @brief Result of a single parameter evaluation.
struct EvalResult {
  ParamConfig params{};       ///< Parameter configuration used.
  AccuracyResult accuracy{};  ///< Accuracy result for this configuration.
  double score{0};            ///< The metric value being optimized.
};

/// @brief Complete search result.
struct SearchResult {
  ParamConfig best_params{};              ///< Best parameter configuration found.
  double best_score{0};                   ///< Best score achieved.
  std::string strategy_name{};            ///< Name of the search strategy used.
  std::string metric_name{};              ///< Name of the metric optimized.
  int total_configs{0};                   ///< Total configurations evaluated.
  std::vector<EvalResult> all_results{};  ///< All evaluation results.

  /// @brief Sort results by score (descending).
  void sort_by_score() {
    std::sort(all_results.begin(), all_results.end(),
              [](const EvalResult& a, const EvalResult& b) { return a.score > b.score; });
  }

  /// @brief Get top N results.
  /// @param n Number of results to return.
  /// @return Vector of top N results, sorted by score (descending).
  std::vector<EvalResult> top_n(int n) const {
    auto sorted = all_results;
    std::sort(sorted.begin(), sorted.end(), [](const EvalResult& a, const EvalResult& b) { return a.score > b.score; });
    int count = std::min(n, static_cast<int>(sorted.size()));
    return std::vector<EvalResult>(sorted.begin(), sorted.begin() + count);
  }
};

/// @brief Callback type for line detection execution.
///
/// Given a source image and a ValueManager-compatible algorithm,
/// runs detection and returns the detected line segments.
using DetectionFunc = std::function<std::vector<LineSegment<double>>(const cv::Mat& src, const ParamConfig& params)>;

/// @brief Parameter optimizer for line detection algorithms.
///
/// Systematically evaluates parameter combinations using a search strategy,
/// running detection on a set of images and comparing against ground truth
/// to find the best parameters.
///
/// @code{cpp}
/// // Define search space
/// SearchSpace space = {
///     ParamRange("quant_error", 1.0, 4.0, 0.5),
///     ParamRange("angle_th", 15.0, 30.0, 5.0),
/// };
///
/// // Define detection function
/// auto detect_fn = [&](const cv::Mat& src, const ParamConfig& params) {
///     LsdFGioi<double> lsd;
///     lsd.value(params);
///     lsd.detect(src);
///     return lsd.lineSegments();
/// };
///
/// // Load ground truth
/// auto gt = GroundTruthLoader::load_csv("ground_truth.csv");
///
/// // Run optimization
/// ParamOptimizer optimizer;
/// GridSearchStrategy strategy;
/// SearchResult result = optimizer.optimize(
///     strategy, space, images, gt, detect_fn);
/// @endcode
class ParamOptimizer {
 public:
  /// @brief Construct with default settings (optimize F1, threshold 5px).
  ParamOptimizer() : metric_(OptimMetric::F1), match_threshold_(5.0), verbose_(false) {}

  /// @brief Construct with specific metric and threshold.
  /// @param metric Metric to optimize for.
  /// @param match_threshold Endpoint matching threshold in pixels.
  /// @param verbose Print progress information.
  ParamOptimizer(OptimMetric metric, double match_threshold = 5.0, bool verbose = false)
      : metric_(metric), match_threshold_(match_threshold), verbose_(verbose) {}

  /// @brief Get the optimization metric.
  OptimMetric metric() const { return metric_; }

  /// @brief Set the optimization metric.
  void set_metric(OptimMetric m) { metric_ = m; }

  /// @brief Get the matching threshold.
  double match_threshold() const { return match_threshold_; }

  /// @brief Set the matching threshold.
  void set_match_threshold(double t) { match_threshold_ = t; }

  /// @brief Get verbose flag.
  bool verbose() const { return verbose_; }

  /// @brief Set verbose flag.
  void set_verbose(bool v) { verbose_ = v; }

  /// @brief Run parameter optimization.
  ///
  /// Evaluates all parameter configurations generated by the strategy,
  /// running the detection function on each image and comparing against
  /// ground truth.
  /// @param strategy Search strategy to generate configurations.
  /// @param space Parameter search space.
  /// @param images Vector of (image_name, image) pairs to evaluate on.
  /// @param ground_truth Ground truth entries (matched by image_name).
  /// @param detect Detection function callback.
  /// @param progress Optional progress callback. Return false to cancel.
  /// @return SearchResult with best parameters and all evaluations.
  SearchResult optimize(const SearchStrategy& strategy,
                        const SearchSpace& space,
                        const std::vector<std::pair<std::string, cv::Mat>>& images,
                        const std::vector<GroundTruthEntry>& ground_truth,
                        const DetectionFunc& detect,
                        const ProgressCallback& progress = nullptr) const {
    // Generate parameter configurations
    auto configs = strategy.generate(space);

    SearchResult result;
    result.strategy_name = strategy.name();
    result.metric_name = metric_string();
    result.total_configs = static_cast<int>(configs.size());
    result.all_results.reserve(configs.size());

    AccuracyMeasure<double> accuracy(match_threshold_);

    // Build ground truth lookup
    std::map<std::string, const GroundTruthEntry*> gt_map;
    for (const auto& entry : ground_truth) {
      gt_map[entry.image_name] = &entry;
    }

    double best_score = -1;
    int step = 0;

    for (const auto& config : configs) {
      EvalResult eval;
      eval.params = config;

      // Evaluate this configuration across all images
      double total_precision = 0;
      double total_recall = 0;
      double total_f1 = 0;
      int n_evaluated = 0;

      for (const auto& [img_name, img] : images) {
        auto gt_it = gt_map.find(img_name);
        if (gt_it == gt_map.end()) continue;

        auto detected = detect(img, config);
        AccuracyResult acc = accuracy.evaluate(detected, gt_it->second->segments);

        total_precision += acc.precision;
        total_recall += acc.recall;
        total_f1 += acc.f1;
        ++n_evaluated;
      }

      if (n_evaluated > 0) {
        eval.accuracy.precision = total_precision / n_evaluated;
        eval.accuracy.recall = total_recall / n_evaluated;
        eval.accuracy.f1 = total_f1 / n_evaluated;
      }

      eval.score = extract_metric(eval.accuracy);
      result.all_results.push_back(eval);

      if (eval.score > best_score) {
        best_score = eval.score;
        result.best_params = config;
        result.best_score = best_score;
      }

      ++step;

      if (verbose_) {
        std::cout << "[" << step << "/" << result.total_configs << "] " << metric_name() << "=" << eval.score
                  << " (best=" << best_score << ")" << std::endl;
      }

      if (progress && !progress(step, result.total_configs, best_score)) {
        break;  // Cancelled
      }
    }

    result.sort_by_score();
    return result;
  }

 private:
  OptimMetric metric_;      ///< Metric to optimize.
  double match_threshold_;  ///< Endpoint matching threshold.
  bool verbose_;            ///< Verbose output flag.

  /// @brief Extract the target metric from an accuracy result.
  double extract_metric(const AccuracyResult& acc) const {
    switch (metric_) {
      case OptimMetric::PRECISION:
        return acc.precision;
      case OptimMetric::RECALL:
        return acc.recall;
      case OptimMetric::F1:
      default:
        return acc.f1;
    }
  }

  /// @brief Get metric name string.
  std::string metric_string() const {
    switch (metric_) {
      case OptimMetric::PRECISION:
        return "precision";
      case OptimMetric::RECALL:
        return "recall";
      case OptimMetric::F1:
      default:
        return "f1";
    }
  }

  /// @brief Get metric name string (alias for metric_string).
  std::string metric_name() const { return metric_string(); }
};

}  // namespace lsfm
