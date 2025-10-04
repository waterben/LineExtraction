#include <utility/performance.hpp>

#include <gtest/gtest.h>

using lsfm::PerformanceMeasure;

TEST(PerformanceTest, ComputeResult) {
  double freq = cv::getTickFrequency();
  std::vector<uint64> ticks = {static_cast<uint64>(freq), static_cast<uint64>(2 * freq)};
  auto res = PerformanceMeasure::computeResult(ticks);
  // Expect approximately 1000ms and 2000ms => mean ~1500ms
  EXPECT_NEAR(res.total, 3000.0, 1.0);
  EXPECT_NEAR(res.mean, 1500.0, 1.0);
  EXPECT_GE(res.stddev, 0.0);
}

TEST(PerformanceTest, AccumulatedMeasure) {
  PerformanceMeasure a("src", "alg", 100, 200);
  a.measures = {10, 20};
  PerformanceMeasure b("src", "alg", 110, 210);
  b.measures = {30};
  auto acc = lsfm::PerformanceTaskBase::accumulatedMeasure({a, b}, "src", "alg");
  EXPECT_DOUBLE_EQ(acc.width, (100 + 110) / 2.0);
  EXPECT_DOUBLE_EQ(acc.height, (200 + 210) / 2.0);
  ASSERT_EQ(acc.measures.size(), 3u);
}
