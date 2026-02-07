//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_hysteresis.cpp
/// @brief Unit tests for hysteresis thresholding.

#include <edge/hysteresis.hpp>
#include <opencv2/core.hpp>

#include <gtest/gtest.h>

namespace {

/// @brief Test hysteresis with single seed pixel.
TEST(HysteresisTest, SingleSeedNoExpansion) {
  // Create 5x5 map with single seed pixel
  // IMPORTANT: Non-edges must be negative (-1), edges positive
  cv::Mat dmap(5, 5, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));  // Initialize with -1 for non-edges
  dmap.at<char>(2, 2) = 1;     // Single edge pixel

  lsfm::IndexVector seeds;
  seeds.push_back(2 * 5 + 2);  // Center pixel (row 2, col 2)

  cv::Mat result = lsfm::hysteresis_binary(dmap, seeds);

  // Only seed pixel should be marked
  EXPECT_EQ(cv::countNonZero(result), 1);
  EXPECT_EQ(result.at<uchar>(2, 2), 1);
}

/// @brief Test hysteresis with horizontal line.
TEST(HysteresisTest, HorizontalLineExpansion) {
  // Create map with horizontal line of edge pixels
  cv::Mat dmap(5, 5, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  for (int col = 1; col < 4; ++col) {
    dmap.at<char>(2, col) = 1;
  }

  lsfm::IndexVector seeds;
  seeds.push_back(2 * 5 + 2);  // Middle of line (row 2, col 2)

  cv::Mat result = lsfm::hysteresis_binary(dmap, seeds);

  // All 3 pixels should be connected
  EXPECT_EQ(cv::countNonZero(result), 3);
  EXPECT_EQ(result.at<uchar>(2, 1), 1);
  EXPECT_EQ(result.at<uchar>(2, 2), 1);
  EXPECT_EQ(result.at<uchar>(2, 3), 1);
}

/// @brief Test hysteresis with vertical line.
TEST(HysteresisTest, VerticalLineExpansion) {
  // Create map with vertical line of edge pixels
  cv::Mat dmap(5, 5, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  for (int row = 1; row < 4; ++row) {
    dmap.at<char>(row, 2) = 1;
  }

  lsfm::IndexVector seeds;
  seeds.push_back(2 * 5 + 2);  // Middle of line (row 2, col 2)

  cv::Mat result = lsfm::hysteresis_binary(dmap, seeds);

  // All 3 pixels should be connected
  EXPECT_EQ(cv::countNonZero(result), 3);
  EXPECT_EQ(result.at<uchar>(1, 2), 1);
  EXPECT_EQ(result.at<uchar>(2, 2), 1);
  EXPECT_EQ(result.at<uchar>(3, 2), 1);
}

/// @brief Test hysteresis with diagonal line.
TEST(HysteresisTest, DiagonalLineExpansion) {
  // Create map with diagonal line
  cv::Mat dmap(5, 5, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  dmap.at<char>(1, 1) = 1;
  dmap.at<char>(2, 2) = 1;
  dmap.at<char>(3, 3) = 1;

  lsfm::IndexVector seeds;
  seeds.push_back(2 * 5 + 2);  // Middle (row 2, col 2)

  cv::Mat result = lsfm::hysteresis_binary(dmap, seeds);

  // All 3 diagonal pixels should be connected
  EXPECT_EQ(cv::countNonZero(result), 3);
  EXPECT_EQ(result.at<uchar>(1, 1), 1);
  EXPECT_EQ(result.at<uchar>(2, 2), 1);
  EXPECT_EQ(result.at<uchar>(3, 3), 1);
}

/// @brief Test hysteresis with disconnected regions.
TEST(HysteresisTest, DisconnectedRegionsNoExpansion) {
  // Create map with two separate regions
  cv::Mat dmap(7, 7, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  dmap.at<char>(1, 1) = 1;
  dmap.at<char>(1, 2) = 1;
  dmap.at<char>(5, 5) = 1;
  dmap.at<char>(5, 6) = 1;

  lsfm::IndexVector seeds;
  seeds.push_back(1 * 7 + 1);  // Top-left region seed

  cv::Mat result = lsfm::hysteresis_binary(dmap, seeds);

  // Only first region (2 pixels) should be marked
  EXPECT_EQ(cv::countNonZero(result), 2);
  EXPECT_EQ(result.at<uchar>(1, 1), 1);
  EXPECT_EQ(result.at<uchar>(1, 2), 1);
  EXPECT_EQ(result.at<uchar>(5, 5), 0);
  EXPECT_EQ(result.at<uchar>(5, 6), 0);
}

/// @brief Test hysteresis with square region.
TEST(HysteresisTest, SquareRegionExpansion) {
  // Create 3x3 square of edge pixels
  cv::Mat dmap(7, 7, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  for (int row = 2; row < 5; ++row) {
    for (int col = 2; col < 5; ++col) {
      dmap.at<char>(row, col) = 1;
    }
  }

  lsfm::IndexVector seeds;
  seeds.push_back(3 * 7 + 3);  // Center of square

  cv::Mat result = lsfm::hysteresis_binary(dmap, seeds);

  // All 9 pixels should be connected
  EXPECT_EQ(cv::countNonZero(result), 9);
}

/// @brief Test hysteresis with multiple seeds.
TEST(HysteresisTest, MultipleSeedsExpansion) {
  // Create U-shaped edge region
  cv::Mat dmap(7, 7, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  for (int row = 1; row < 6; ++row) {
    dmap.at<char>(row, 1) = 1;  // Left vertical
    dmap.at<char>(row, 5) = 1;  // Right vertical
  }
  for (int col = 1; col < 6; ++col) {
    dmap.at<char>(5, col) = 1;  // Bottom horizontal
  }

  lsfm::IndexVector seeds;
  seeds.push_back(1 * 7 + 1);  // Top-left
  seeds.push_back(1 * 7 + 5);  // Top-right

  cv::Mat result = lsfm::hysteresis_binary(dmap, seeds);

  // All U-shape pixels should be connected (5 left + 5 right + 3 bottom unique = 13)
  EXPECT_EQ(cv::countNonZero(result), 13);
}

/// @brief Test hysteresis_binary with custom value.
TEST(HysteresisTest, BinaryCustomValue) {
  cv::Mat dmap(3, 3, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  dmap.at<char>(1, 1) = 1;

  lsfm::IndexVector seeds;
  seeds.push_back(1 * 3 + 1);

  cv::Mat result = lsfm::hysteresis_binary(dmap, seeds, 255);

  EXPECT_EQ(result.at<uchar>(1, 1), 255);
  EXPECT_EQ(cv::countNonZero(result), 1);
}

// Note: The non-binary hysteresis() function has implementation issues
// with pointer arithmetic and is not safely testable in current form.
// Tests focus on the working hysteresis_binary() implementation.

/// @brief Test hysteresis_edgels modifying edgels vector.
TEST(HysteresisTest, EdgelsVectorExpansion) {
  cv::Mat dmap(5, 5, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  for (int col = 1; col < 4; ++col) {
    dmap.at<char>(2, col) = 5;  // Positive value for valid edge
  }

  lsfm::IndexVector edgels;
  edgels.push_back(2 * 5 + 2);  // Start with 1 seed

  size_t initial_size = edgels.size();
  lsfm::hysteresis_edgels(dmap, edgels);

  // Edgels should have expanded to include neighbors
  EXPECT_GT(edgels.size(), initial_size);
  EXPECT_EQ(edgels.size(), 3u);  // 3 connected pixels
}

/// @brief Test empty seed vector.
TEST(HysteresisTest, EmptySeedVector) {
  cv::Mat dmap(5, 5, CV_8SC1);
  dmap.setTo(cv::Scalar(1));  // All edges but no seeds

  lsfm::IndexVector seeds;  // Empty

  cv::Mat result = lsfm::hysteresis_binary(dmap, seeds);

  // No seeds -> no marked pixels
  EXPECT_EQ(cv::countNonZero(result), 0);
}

/// @brief Test hysteresis_const (const version).
TEST(HysteresisTest, ConstVersion) {
  // IMPORTANT: Non-edges must be negative (-1), edges positive
  cv::Mat dmap(3, 3, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  dmap.at<char>(1, 1) = 1;

  const lsfm::IndexVector seeds = {1 * 3 + 1};

  cv::Mat result = lsfm::hysteresis_const(dmap, seeds);

  EXPECT_EQ(result.at<char>(1, 1), 1);
  // Original seeds should be unchanged
  EXPECT_EQ(seeds.size(), 1u);
}

/// @brief Test hysteresis_binary_const (const version).
TEST(HysteresisTest, BinaryConstVersion) {
  cv::Mat dmap(3, 3, CV_8SC1);
  dmap.setTo(cv::Scalar(-1));
  dmap.at<char>(1, 1) = 1;

  const lsfm::IndexVector seeds = {1 * 3 + 1};

  cv::Mat result = lsfm::hysteresis_binary_const(dmap, seeds);

  EXPECT_EQ(result.at<uchar>(1, 1), 1);
  EXPECT_EQ(seeds.size(), 1u);
}

}  // namespace
