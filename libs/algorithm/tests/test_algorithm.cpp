//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************

#include <algorithm/accuracy_measure.hpp>
#include <algorithm/ground_truth.hpp>
#include <algorithm/line_connect.hpp>
#include <algorithm/line_merge.hpp>
#include <algorithm/param_search.hpp>
#include <algorithm/search_strategy.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace lsfm;

// =============================================================================
// LineMerge Tests
// =============================================================================

TEST(LineMergeTest, MergesCollinearSegments) {
  LineMerge<double> merger(20.0, 10.0, 3.0, 15.0, MergeType::STANDARD);

  LineMerge<double>::LineSegmentVector input = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
      LineSegment2d(cv::Point2d(12, 0), cv::Point2d(22, 0)),
  };

  LineMerge<double>::LineSegmentVector output;
  merger.merge_lines(input, output);

  // Two close collinear segments should merge into one
  ASSERT_EQ(output.size(), 1u);
  // The merged segment should span from (0,0) to (22,0)
  EXPECT_NEAR(output[0].length(), 22.0, 1.0);
}

TEST(LineMergeTest, PreservesDistantSegments) {
  LineMerge<double> merger(5.0, 5.0, 2.0, 5.0, MergeType::STANDARD);

  LineMerge<double>::LineSegmentVector input = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
      LineSegment2d(cv::Point2d(100, 100), cv::Point2d(110, 100)),
  };

  LineMerge<double>::LineSegmentVector output;
  merger.merge_lines(input, output);

  // Distant segments should not merge
  ASSERT_EQ(output.size(), 2u);
}

TEST(LineMergeTest, PreservesPerpendicularSegments) {
  LineMerge<double> merger(20.0, 3.0, 3.0, 10.0, MergeType::STANDARD);

  LineMerge<double>::LineSegmentVector input = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
      LineSegment2d(cv::Point2d(10, 0), cv::Point2d(10, 10)),
  };

  LineMerge<double>::LineSegmentVector output;
  merger.merge_lines(input, output);

  // Perpendicular segments should not merge (angle > threshold)
  ASSERT_EQ(output.size(), 2u);
}

TEST(LineMergeTest, EmptyInput) {
  LineMerge<double> merger;
  LineMerge<double>::LineSegmentVector input, output;
  merger.merge_lines(input, output);
  EXPECT_TRUE(output.empty());
}

TEST(LineMergeTest, SingleSegment) {
  LineMerge<double> merger;
  LineMerge<double>::LineSegmentVector input = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
  };
  LineMerge<double>::LineSegmentVector output;
  merger.merge_lines(input, output);
  ASSERT_EQ(output.size(), 1u);
}

TEST(LineMergeTest, ValueManagerIntegration) {
  LineMerge<double> merger;
  // Check default values are accessible
  auto vals = merger.values();
  EXPECT_GE(vals.size(), 5u);

  // Set via ValueManager interface
  merger.value("max_dist", Value(30.0));
  EXPECT_DOUBLE_EQ(merger.value("max_dist").getDouble(), 30.0);

  merger.value("merge_type", Value(1));
  EXPECT_EQ(merger.value("merge_type").getInt(), 1);
}

TEST(LineMergeTest, AvgMerge) {
  LineMerge<double> merger(20.0, 10.0, 5.0, 15.0, MergeType::AVG);

  LineMerge<double>::LineSegmentVector input = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
      LineSegment2d(cv::Point2d(2, 1), cv::Point2d(12, 1)),
  };

  LineMerge<double>::LineSegmentVector output;
  merger.merge_lines(input, output);

  ASSERT_EQ(output.size(), 1u);
  // Average: start=(1, 0.5), end=(11, 0.5)
  auto sp = output[0].startPoint();
  EXPECT_NEAR(getX(sp), 1.0, 0.1);
  EXPECT_NEAR(getY(sp), 0.5, 0.1);
}

// =============================================================================
// LineConnect Tests
// =============================================================================

TEST(LineConnectTest, EmptyInput) {
  LineConnect<double> connector;
  LineConnect<double>::LineSegmentVector input, output;
  cv::Mat mag = cv::Mat::zeros(100, 100, CV_32F);
  connector.connect_lines(input, output, mag);
  EXPECT_TRUE(output.empty());
}

TEST(LineConnectTest, SingleSegment) {
  LineConnect<double> connector;
  LineConnect<double>::LineSegmentVector input = {
      LineSegment2d(cv::Point2d(10, 10), cv::Point2d(20, 10)),
  };
  LineConnect<double>::LineSegmentVector output;
  cv::Mat mag = cv::Mat::zeros(100, 100, CV_32F);
  connector.connect_lines(input, output, mag);
  ASSERT_EQ(output.size(), 1u);
}

TEST(LineConnectTest, ValueManagerIntegration) {
  LineConnect<double> connector;
  auto vals = connector.values();
  EXPECT_GE(vals.size(), 3u);

  connector.value("max_radius", Value(25.0));
  EXPECT_DOUBLE_EQ(connector.value("max_radius").getDouble(), 25.0);
}

// =============================================================================
// AccuracyMeasure Tests
// =============================================================================

TEST(AccuracyMeasureTest, PerfectMatch) {
  AccuracyMeasure<double> measure(5.0);

  std::vector<LineSegment2d> detected = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
      LineSegment2d(cv::Point2d(0, 10), cv::Point2d(10, 10)),
  };
  auto ground_truth = detected;  // Same segments

  auto result = measure.evaluate(detected, ground_truth);
  EXPECT_DOUBLE_EQ(result.precision, 1.0);
  EXPECT_DOUBLE_EQ(result.recall, 1.0);
  EXPECT_DOUBLE_EQ(result.f1, 1.0);
  EXPECT_EQ(result.true_positives, 2);
  EXPECT_EQ(result.false_positives, 0);
  EXPECT_EQ(result.false_negatives, 0);
}

TEST(AccuracyMeasureTest, NoDetections) {
  AccuracyMeasure<double> measure(5.0);

  std::vector<LineSegment2d> detected;
  std::vector<LineSegment2d> ground_truth = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
  };

  auto result = measure.evaluate(detected, ground_truth);
  EXPECT_DOUBLE_EQ(result.precision, 0.0);
  EXPECT_DOUBLE_EQ(result.recall, 0.0);
  EXPECT_EQ(result.false_negatives, 1);
}

TEST(AccuracyMeasureTest, AllFalsePositives) {
  AccuracyMeasure<double> measure(2.0);  // Tight threshold

  std::vector<LineSegment2d> detected = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
  };
  std::vector<LineSegment2d> ground_truth = {
      // Far away from detected
      LineSegment2d(cv::Point2d(100, 100), cv::Point2d(110, 100)),
  };

  auto result = measure.evaluate(detected, ground_truth);
  EXPECT_DOUBLE_EQ(result.precision, 0.0);
  EXPECT_DOUBLE_EQ(result.recall, 0.0);
  EXPECT_EQ(result.false_positives, 1);
  EXPECT_EQ(result.false_negatives, 1);
}

TEST(AccuracyMeasureTest, PartialMatch) {
  AccuracyMeasure<double> measure(5.0);

  std::vector<LineSegment2d> detected = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),     // matches gt[0]
      LineSegment2d(cv::Point2d(50, 50), cv::Point2d(60, 50)),  // false positive
  };
  std::vector<LineSegment2d> ground_truth = {
      LineSegment2d(cv::Point2d(0, 1), cv::Point2d(10, 1)),         // matches det[0]
      LineSegment2d(cv::Point2d(100, 100), cv::Point2d(110, 100)),  // false negative
  };

  auto result = measure.evaluate(detected, ground_truth);
  EXPECT_DOUBLE_EQ(result.precision, 0.5);
  EXPECT_DOUBLE_EQ(result.recall, 0.5);
  EXPECT_EQ(result.true_positives, 1);
  EXPECT_EQ(result.false_positives, 1);
  EXPECT_EQ(result.false_negatives, 1);
}

TEST(AccuracyMeasureTest, BothEmpty) {
  AccuracyMeasure<double> measure(5.0);
  std::vector<LineSegment2d> detected, ground_truth;
  auto result = measure.evaluate(detected, ground_truth);
  EXPECT_DOUBLE_EQ(result.precision, 1.0);
  EXPECT_DOUBLE_EQ(result.recall, 1.0);
  EXPECT_DOUBLE_EQ(result.f1, 1.0);
}

TEST(AccuracyMeasureTest, ReversedEndpoints) {
  // Detected has reversed endpoints compared to ground truth
  AccuracyMeasure<double> measure(2.0);

  std::vector<LineSegment2d> detected = {
      LineSegment2d(cv::Point2d(10, 0), cv::Point2d(0, 0)),  // Reversed
  };
  std::vector<LineSegment2d> ground_truth = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
  };

  auto result = measure.evaluate(detected, ground_truth);
  EXPECT_DOUBLE_EQ(result.precision, 1.0);
  EXPECT_DOUBLE_EQ(result.recall, 1.0);
}

TEST(AccuracyMeasureTest, StructuralAP) {
  AccuracyMeasure<double> measure(5.0);

  std::vector<LineSegment2d> detected = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
  };
  std::vector<LineSegment2d> ground_truth = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 0)),
  };

  double sap = measure.structural_ap(detected, ground_truth, {5, 10, 15});
  EXPECT_DOUBLE_EQ(sap, 1.0);
}

// =============================================================================
// SearchStrategy Tests
// =============================================================================

TEST(GridSearchTest, SingleParam) {
  SearchSpace space = {
      ParamRange("param_a", 0.0, 1.0, 0.5),
  };
  GridSearchStrategy strategy;
  auto configs = strategy.generate(space);

  // 0.0, 0.5, 1.0 = 3 values
  ASSERT_EQ(configs.size(), 3u);
  EXPECT_DOUBLE_EQ(configs[0][0].value.getDouble(), 0.0);
  EXPECT_DOUBLE_EQ(configs[1][0].value.getDouble(), 0.5);
  EXPECT_DOUBLE_EQ(configs[2][0].value.getDouble(), 1.0);
}

TEST(GridSearchTest, TwoParams) {
  SearchSpace space = {
      ParamRange("a", 0.0, 1.0, 1.0),     // 2 values: 0, 1
      ParamRange("b", 10.0, 20.0, 10.0),  // 2 values: 10, 20
  };
  GridSearchStrategy strategy;
  auto configs = strategy.generate(space);

  // Cartesian product: 2 * 2 = 4
  ASSERT_EQ(configs.size(), 4u);
}

TEST(GridSearchTest, EmptySpace) {
  SearchSpace space;
  GridSearchStrategy strategy;
  auto configs = strategy.generate(space);
  EXPECT_TRUE(configs.empty());
}

TEST(GridSearchTest, IntParam) {
  SearchSpace space = {
      ParamRange::make_int("k", 3, 7, 2),  // 3, 5, 7 = 3 values
  };
  GridSearchStrategy strategy;
  auto configs = strategy.generate(space);

  ASSERT_EQ(configs.size(), 3u);
  EXPECT_EQ(configs[0][0].value.getInt(), 3);
  EXPECT_EQ(configs[1][0].value.getInt(), 5);
  EXPECT_EQ(configs[2][0].value.getInt(), 7);
}

TEST(GridSearchTest, BoolParam) {
  SearchSpace space = {
      ParamRange::make_bool("flag"),  // false, true = 2 values
  };
  GridSearchStrategy strategy;
  auto configs = strategy.generate(space);
  ASSERT_EQ(configs.size(), 2u);
}

TEST(RandomSearchTest, GeneratesCorrectCount) {
  SearchSpace space = {
      ParamRange("a", 0.0, 10.0, 1.0),
  };
  RandomSearchStrategy strategy(50, 42);
  auto configs = strategy.generate(space);
  ASSERT_EQ(configs.size(), 50u);
}

TEST(RandomSearchTest, Reproducible) {
  SearchSpace space = {
      ParamRange("a", 0.0, 10.0, 1.0),
  };
  RandomSearchStrategy s1(10, 42);
  RandomSearchStrategy s2(10, 42);
  auto c1 = s1.generate(space);
  auto c2 = s2.generate(space);

  ASSERT_EQ(c1.size(), c2.size());
  for (std::size_t i = 0; i < c1.size(); ++i) {
    EXPECT_DOUBLE_EQ(c1[i][0].value.getDouble(), c2[i][0].value.getDouble());
  }
}

TEST(RandomSearchTest, WithinBounds) {
  SearchSpace space = {
      ParamRange("a", 5.0, 15.0, 1.0),
  };
  RandomSearchStrategy strategy(100, 123);
  auto configs = strategy.generate(space);

  for (const auto& config : configs) {
    double val = config[0].value.getDouble();
    EXPECT_GE(val, 5.0);
    EXPECT_LE(val, 15.0);
  }
}

// =============================================================================
// ParamOptimizer Tests
// =============================================================================

TEST(ParamOptimizerTest, BasicOptimization) {
  // Simple synthetic test: the detection function returns more lines
  // when "sensitivity" is higher, but ground truth has exactly 2 lines.
  std::vector<LineSegment2d> gt_lines = {
      LineSegment2d(cv::Point2d(10, 10), cv::Point2d(90, 10)),
      LineSegment2d(cv::Point2d(10, 50), cv::Point2d(90, 50)),
  };

  std::vector<GroundTruthEntry> ground_truth = {GroundTruthLoader::make_entry("test.png", gt_lines)};

  cv::Mat test_img = cv::Mat::zeros(100, 100, CV_8UC1);
  std::vector<std::pair<std::string, cv::Mat>> images = {{"test.png", test_img}};

  // Detection function: returns ground truth lines when sensitivity is 0.5
  auto detect_fn = [&](const cv::Mat& /*src*/, const ParamConfig& params) {
    double sensitivity = params[0].value.getDouble();
    std::vector<LineSegment2d> detected;
    if (sensitivity >= 0.3) {
      detected.push_back(gt_lines[0]);
      detected.push_back(gt_lines[1]);
    }
    if (sensitivity >= 0.7) {
      // Add false positive
      detected.emplace_back(cv::Point2d(50, 80), cv::Point2d(90, 80));
    }
    return detected;
  };

  SearchSpace space = {
      ParamRange("sensitivity", 0.0, 1.0, 0.1),
  };

  ParamOptimizer optimizer(OptimMetric::F1, 5.0, false);
  GridSearchStrategy strategy;

  SearchResult result = optimizer.optimize(strategy, space, images, ground_truth, detect_fn);

  // Best F1 should be at sensitivity in [0.3, 0.6] range (all 2 detected, no FP)
  EXPECT_GT(result.best_score, 0.0);
  EXPECT_GT(result.total_configs, 0);
  EXPECT_FALSE(result.all_results.empty());

  // Check that best params exist
  EXPECT_FALSE(result.best_params.empty());

  // The best sensitivity should give F1=1.0 (2 TP, 0 FP, 0 FN)
  EXPECT_DOUBLE_EQ(result.best_score, 1.0);
}

TEST(ParamOptimizerTest, ProgressCallback) {
  std::vector<GroundTruthEntry> ground_truth;
  std::vector<std::pair<std::string, cv::Mat>> images;
  auto detect_fn = [](const cv::Mat&, const ParamConfig&) { return std::vector<LineSegment2d>{}; };

  SearchSpace space = {ParamRange("a", 0.0, 1.0, 0.5)};
  ParamOptimizer optimizer;
  GridSearchStrategy strategy;

  int progress_calls = 0;
  auto progress = [&](int step, int total, double) -> bool {
    ++progress_calls;
    EXPECT_GT(total, 0);
    EXPECT_LE(step, total);
    return true;
  };

  auto result = optimizer.optimize(strategy, space, images, ground_truth, detect_fn, progress);
  EXPECT_EQ(progress_calls, result.total_configs);
}

TEST(ParamOptimizerTest, CancellableSearch) {
  std::vector<GroundTruthEntry> ground_truth;
  std::vector<std::pair<std::string, cv::Mat>> images;
  auto detect_fn = [](const cv::Mat&, const ParamConfig&) { return std::vector<LineSegment2d>{}; };

  SearchSpace space = {ParamRange("a", 0.0, 10.0, 0.1)};  // 101 configs
  ParamOptimizer optimizer;
  GridSearchStrategy strategy;

  int progress_calls = 0;
  auto progress = [&](int, int, double) -> bool {
    ++progress_calls;
    return progress_calls < 5;  // Cancel after 5
  };

  auto result = optimizer.optimize(strategy, space, images, ground_truth, detect_fn, progress);
  EXPECT_EQ(progress_calls, 5);
  EXPECT_LT(static_cast<int>(result.all_results.size()), result.total_configs);
}

// =============================================================================
// GroundTruthLoader Tests
// =============================================================================

TEST(GroundTruthTest, MakeEntry) {
  std::vector<LineSegment2d> segs = {
      LineSegment2d(cv::Point2d(0, 0), cv::Point2d(10, 10)),
  };
  auto entry = GroundTruthLoader::make_entry("test.png", segs);
  EXPECT_EQ(entry.image_name, "test.png");
  ASSERT_EQ(entry.segments.size(), 1u);
}

// =============================================================================
// SearchResult Tests
// =============================================================================

TEST(SearchResultTest, TopN) {
  SearchResult result;
  for (int i = 0; i < 10; ++i) {
    EvalResult eval;
    eval.score = static_cast<double>(i) / 10.0;
    result.all_results.push_back(eval);
  }

  auto top3 = result.top_n(3);
  ASSERT_EQ(top3.size(), 3u);
  EXPECT_DOUBLE_EQ(top3[0].score, 0.9);
  EXPECT_DOUBLE_EQ(top3[1].score, 0.8);
  EXPECT_DOUBLE_EQ(top3[2].score, 0.7);
}
