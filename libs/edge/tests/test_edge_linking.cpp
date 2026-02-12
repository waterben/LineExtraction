//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_edge_linking.cpp
/// @brief Unit tests for edge linking detector (EsdLinking).
/// Tests gap bridging, segment extraction, and parameter handling.
/// Specifically verifies that gap=0 (no gap bridging) terminates correctly,
/// addressing a historical infinite-loop / crash bug.

#include <edge/edge_linking.hpp>
#include <opencv2/core.hpp>

#include <gtest/gtest.h>

#include <chrono>

using namespace lsfm;

// ---------------------------------------------------------------------------
// Helper: create a synthetic direction + magnitude map with a horizontal edge
// line that contains a gap of `gap_size` pixels in the middle.
//
// Layout (for cols=20, row=5, gap_start=9, gap_size=2):
//   cols 2..8  : direction=0 (right), magnitude=100
//   cols 9..10 : direction=-1 (no edge), magnitude=0  <-- gap
//   cols 11..17: direction=0 (right), magnitude=100
//
// Returns the seed index at the center of the left edge segment.
// ---------------------------------------------------------------------------

/// @brief Parameters for building a synthetic edge row with an optional gap.
struct SyntheticEdgeParams {
  int rows = 12;       ///< Image height
  int cols = 20;       ///< Image width
  int edge_row = 5;    ///< Row in which the edge is placed
  int edge_start = 2;  ///< Column where the edge begins
  int edge_end = 17;   ///< Column where the edge ends (inclusive)
  int gap_start = 9;   ///< Column where the gap begins (-1 for no gap)
  int gap_size = 0;    ///< Number of gap pixels (0 = no gap)
};

/// @brief Build direction and magnitude maps for a horizontal edge with an
///        optional gap, and return a seed index inside the left part.
/// @param p Parameters describing the synthetic edge layout
/// @param[out] dir Direction map  (CV_8SC1, non-edge = -1)
/// @param[out] mag Magnitude map  (CV_32SC1)
/// @return Seed index (flattened) inside the left edge segment
static index_type build_edge_with_gap(const SyntheticEdgeParams& p, cv::Mat& dir, cv::Mat& mag) {
  dir = cv::Mat(p.rows, p.cols, CV_8SC1);
  dir.setTo(cv::Scalar(-1));
  mag = cv::Mat::zeros(p.rows, p.cols, CV_32SC1);

  for (int col = p.edge_start; col <= p.edge_end; ++col) {
    // Skip gap region
    if (p.gap_size > 0 && col >= p.gap_start && col < p.gap_start + p.gap_size) {
      continue;
    }
    dir.at<char>(p.edge_row, col) = 0;  // Direction: right
    mag.at<int>(p.edge_row, col) = 100;
  }

  // Seed in the middle of the left portion
  int seed_col = (p.edge_start + std::min(p.edge_end, p.gap_start - 1)) / 2;
  return static_cast<index_type>(p.edge_row * p.cols + seed_col);
}

// ===========================================================================
// Construction & parameter tests
// ===========================================================================

/// @brief Default construction uses expected defaults.
TEST(EsdLinkingTest, DefaultConstruction) {
  EsdLinking<int> link;
  EXPECT_EQ(link.minPixels(), 10);
  EXPECT_EQ(link.maxGap(), 3);
}

/// @brief Custom constructor parameters are stored correctly.
TEST(EsdLinkingTest, CustomConstruction) {
  EsdLinking<int> link(5, 7, 2.0f, 10.0f);
  EXPECT_EQ(link.minPixels(), 5);
  EXPECT_EQ(link.maxGap(), 7);
  EXPECT_FLOAT_EQ(link.magMul(), 2.0f);
  EXPECT_FLOAT_EQ(link.magThresold(), 10.0f);
}

/// @brief Setters and getters for maxGap work.
TEST(EsdLinkingTest, SetMaxGap) {
  EsdLinking<int> link;
  link.maxGap(0);
  EXPECT_EQ(link.maxGap(), 0);
  link.maxGap(5);
  EXPECT_EQ(link.maxGap(), 5);
}

// ===========================================================================
// Gap=0  –  the former infinite-loop / crash regression
// ===========================================================================

/// @brief With maxGap=0 and no gap in the edge, detection must terminate and
///        produce a valid segment (regression test for infinite-loop bug).
TEST(EsdLinkingTest, GapZeroNoGapTerminates) {
  EsdLinking<int> link(1, /*maxGap=*/0);

  SyntheticEdgeParams p;
  p.gap_size = 0;  // Continuous edge, no gap
  cv::Mat dir, mag;
  index_type seed = build_edge_with_gap(p, dir, mag);

  IndexVector seeds = {seed};

  // Must complete within a reasonable time (no infinite loop)
  auto t0 = std::chrono::steady_clock::now();
  link.detect(dir, mag, seeds);
  auto t1 = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

  // Generous limit – even on slow CI this should be <100 ms for 20x12 image
  EXPECT_LT(elapsed_ms, 2000) << "detect() with gap=0 took too long – "
                                 "possible infinite loop regression";

  EXPECT_GT(link.points().size(), 0u);
  EXPECT_GT(link.segments().size(), 0u);
}

/// @brief With maxGap=0 and a 1-pixel gap, detection must terminate normally
///        but the gap should NOT be bridged (two separate segments or one
///        shorter segment).
TEST(EsdLinkingTest, GapZeroWithSmallGapTerminates) {
  EsdLinking<int> link(1, /*maxGap=*/0);

  SyntheticEdgeParams p;
  p.gap_start = 10;
  p.gap_size = 1;  // 1-pixel gap – cannot be bridged with maxGap=0
  cv::Mat dir, mag;
  index_type seed = build_edge_with_gap(p, dir, mag);

  IndexVector seeds = {seed};

  auto t0 = std::chrono::steady_clock::now();
  link.detect(dir, mag, seeds);
  auto t1 = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

  EXPECT_LT(elapsed_ms, 2000) << "detect() with gap=0, gap_size=1 took too "
                                 "long – possible infinite loop regression";

  // Detection should complete; we expect at least one segment
  EXPECT_GT(link.points().size(), 0u);
}

// ===========================================================================
// Varying gap sizes – parameterised sanity tests
// ===========================================================================

/// @brief Test fixture that runs detection with different maxGap values.
class EsdLinkingGapTest : public ::testing::TestWithParam<int> {};

/// @brief Detection with a 2-pixel gap and varying maxGap must always
///        terminate.  When maxGap >= 2 the gap can be bridged so segments
///        may be longer.
TEST_P(EsdLinkingGapTest, TerminatesWithGap2) {
  const int max_gap = GetParam();
  EsdLinking<int> link(1, max_gap);

  SyntheticEdgeParams p;
  p.gap_start = 10;
  p.gap_size = 2;  // 2-pixel gap in the edge
  cv::Mat dir, mag;
  index_type seed = build_edge_with_gap(p, dir, mag);

  IndexVector seeds = {seed};

  auto t0 = std::chrono::steady_clock::now();
  link.detect(dir, mag, seeds);
  auto t1 = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

  EXPECT_LT(elapsed_ms, 2000) << "detect() with maxGap=" << max_gap << " took too long – possible infinite loop";

  EXPECT_GT(link.points().size(), 0u);
}

INSTANTIATE_TEST_SUITE_P(GapSizes, EsdLinkingGapTest, ::testing::Values(0, 1, 2, 3, 5));

// ===========================================================================
// Corner rule variant – same gap=0 regression check
// ===========================================================================

/// @brief EsdLinking with corner rule enabled and maxGap=0 must terminate.
TEST(EsdLinkingCornerRuleTest, GapZeroTerminates) {
  EsdLinking<int, 8, true> link(1, /*maxGap=*/0);

  SyntheticEdgeParams p;
  p.gap_size = 0;
  cv::Mat dir, mag;
  index_type seed = build_edge_with_gap(p, dir, mag);

  IndexVector seeds = {seed};

  auto t0 = std::chrono::steady_clock::now();
  link.detect(dir, mag, seeds);
  auto t1 = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

  EXPECT_LT(elapsed_ms, 2000) << "detect() with corner rule, gap=0 took too long";

  EXPECT_GT(link.points().size(), 0u);
}

/// @brief EsdLinking with corner rule, varying gap sizes must terminate.
class EsdLinkingCornerGapTest : public ::testing::TestWithParam<int> {};

TEST_P(EsdLinkingCornerGapTest, TerminatesWithGap2) {
  const int max_gap = GetParam();
  EsdLinking<int, 8, true> link(1, max_gap);

  SyntheticEdgeParams p;
  p.gap_start = 10;
  p.gap_size = 2;
  cv::Mat dir, mag;
  index_type seed = build_edge_with_gap(p, dir, mag);

  IndexVector seeds = {seed};

  auto t0 = std::chrono::steady_clock::now();
  link.detect(dir, mag, seeds);
  auto t1 = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

  EXPECT_LT(elapsed_ms, 2000) << "detect() with corner rule, maxGap=" << max_gap << " took too long";

  EXPECT_GT(link.points().size(), 0u);
}

INSTANTIATE_TEST_SUITE_P(CornerGapSizes, EsdLinkingCornerGapTest, ::testing::Values(0, 1, 2, 3, 5));

// ===========================================================================
// Empty seeds – must not crash
// ===========================================================================

/// @brief Empty seeds produce no output.
TEST(EsdLinkingTest, EmptySeeds) {
  EsdLinking<int> link;

  cv::Mat dir(10, 10, CV_8SC1);
  dir.setTo(cv::Scalar(-1));
  cv::Mat mag = cv::Mat::zeros(10, 10, CV_32SC1);
  IndexVector seeds;

  link.detect(dir, mag, seeds);

  EXPECT_EQ(link.points().size(), 0u);
  EXPECT_EQ(link.segments().size(), 0u);
}

// ===========================================================================
// Results cleared between calls
// ===========================================================================

/// @brief Consecutive detect() calls do not accumulate results.
TEST(EsdLinkingTest, ResultsCleared) {
  EsdLinking<int> link(1, 0);

  SyntheticEdgeParams p;
  cv::Mat dir, mag;
  index_type seed = build_edge_with_gap(p, dir, mag);
  IndexVector seeds = {seed};

  link.detect(dir, mag, seeds);
  size_t pts_first = link.points().size();
  EXPECT_GT(pts_first, 0u);

  // Second call with empty seeds – results must be cleared
  IndexVector empty;
  link.detect(dir, mag, empty);
  EXPECT_EQ(link.points().size(), 0u);
  EXPECT_EQ(link.segments().size(), 0u);
}
