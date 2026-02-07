//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_performance.cpp
/// @brief Unit tests for performance evaluation.
#include <eval/cv_measure.hpp>

#include <gtest/gtest.h>

using lsfm::CVPerformanceMeasure;
using lsfm::cvTimingStrategy;
using lsfm::PerformanceMeasure;

TEST(PerformanceTest, ComputeResult) {
  double freq = cv::getTickFrequency();
  std::vector<uint64> ticks = {static_cast<uint64>(freq), static_cast<uint64>(2 * freq)};
  auto res = CVPerformanceMeasure::computeResult(ticks);
  // Expect approximately 1000ms and 2000ms => mean ~1500ms
  EXPECT_NEAR(res.total, 3000.0, 1.0);
  EXPECT_NEAR(res.mean, 1500.0, 1.0);
  EXPECT_GE(res.stddev, 0.0);
}

TEST(PerformanceTest, AccumulatedMeasure) {
  CVPerformanceMeasure a("src", "alg", 100, 200);
  a.durations = {10, 20};
  CVPerformanceMeasure b("src", "alg", 110, 210);
  b.durations = {30};
  auto acc = lsfm::accumulateCVMeasures({a, b}, "src", "alg");
  EXPECT_DOUBLE_EQ(acc.width, (100 + 110) / 2.0);
  EXPECT_DOUBLE_EQ(acc.height, (200 + 210) / 2.0);
  ASSERT_EQ(acc.durations.size(), 3u);
}

TEST(PerformanceTest, LegacyAlias) {
  // Test that PerformanceMeasure alias works
  PerformanceMeasure pm("source", "task", 640, 480);
  pm.durations.push_back(1000);
  EXPECT_EQ(pm.source_name, "source");
  EXPECT_EQ(pm.task_name, "task");
  EXPECT_DOUBLE_EQ(pm.width, 640.0);
  EXPECT_DOUBLE_EQ(pm.height, 480.0);
  EXPECT_NEAR(pm.megaPixels(), 0.3072, 0.0001);
}
