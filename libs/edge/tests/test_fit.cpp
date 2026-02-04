/// @file test_fit.cpp
/// @brief Unit tests for curve fitting utilities.

#include <edge/fit.hpp>
#include <opencv2/core.hpp>

#include <gtest/gtest.h>
#include <vector>
#include <cmath>

namespace {

/// @brief Simple point structure for testing.
struct TestPoint {
  float x;
  float y;

  TestPoint(float x_val, float y_val) : x(x_val), y(y_val) {}
};

/// @brief Accessor functions for TestPoint.
inline float getX(const TestPoint& p) { return p.x; }
inline float getY(const TestPoint& p) { return p.y; }

/// @brief Test covariance with single point.
TEST(FitTest, CovarianceSinglePoint) {
  TestPoint points[] = {TestPoint(5.0f, 10.0f)};

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float>(points, points + 1, sx, sy, sxy, cx, cy);

  // Centroid should be at the point
  EXPECT_NEAR(cx, 5.0f, 1e-6f);
  EXPECT_NEAR(cy, 10.0f, 1e-6f);

  // Variances should be zero (single point)
  EXPECT_NEAR(sx, 0.0f, 1e-6f);
  EXPECT_NEAR(sy, 0.0f, 1e-6f);
  EXPECT_NEAR(sxy, 0.0f, 1e-6f);
}

/// @brief Test covariance with two points on horizontal line.
TEST(FitTest, CovarianceTwoPointsHorizontal) {
  TestPoint points[] = {TestPoint(0.0f, 5.0f), TestPoint(10.0f, 5.0f)};

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float>(points, points + 2, sx, sy, sxy, cx, cy);

  // Centroid at midpoint
  EXPECT_NEAR(cx, 5.0f, 1e-6f);
  EXPECT_NEAR(cy, 5.0f, 1e-6f);

  // Variance in X, none in Y
  EXPECT_GT(sx, 0.0f);
  EXPECT_NEAR(sy, 0.0f, 1e-6f);
  EXPECT_NEAR(sxy, 0.0f, 1e-6f);
}

/// @brief Test covariance with two points on vertical line.
TEST(FitTest, CovarianceTwoPointsVertical) {
  TestPoint points[] = {TestPoint(5.0f, 0.0f), TestPoint(5.0f, 10.0f)};

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float>(points, points + 2, sx, sy, sxy, cx, cy);

  // Centroid at midpoint
  EXPECT_NEAR(cx, 5.0f, 1e-6f);
  EXPECT_NEAR(cy, 5.0f, 1e-6f);

  // Variance in Y, none in X
  EXPECT_NEAR(sx, 0.0f, 1e-6f);
  EXPECT_GT(sy, 0.0f);
  EXPECT_NEAR(sxy, 0.0f, 1e-6f);
}

/// @brief Test covariance with diagonal points (positive correlation).
TEST(FitTest, CovarianceDiagonalPositive) {
  TestPoint points[] = {TestPoint(0.0f, 0.0f), TestPoint(10.0f, 10.0f)};

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float>(points, points + 2, sx, sy, sxy, cx, cy);

  // Centroid at midpoint
  EXPECT_NEAR(cx, 5.0f, 1e-6f);
  EXPECT_NEAR(cy, 5.0f, 1e-6f);

  // Positive correlation: sxy should be negative (formula uses -dx*dy)
  EXPECT_GT(sx, 0.0f);
  EXPECT_GT(sy, 0.0f);
  EXPECT_LT(sxy, 0.0f);  // Note: formula has negative sign

  // For perfect diagonal, sx == sy
  EXPECT_NEAR(sx, sy, 1e-5f);
}

/// @brief Test covariance with diagonal points (negative correlation).
TEST(FitTest, CovarianceDiagonalNegative) {
  TestPoint points[] = {TestPoint(0.0f, 10.0f), TestPoint(10.0f, 0.0f)};

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float>(points, points + 2, sx, sy, sxy, cx, cy);

  // Centroid at midpoint
  EXPECT_NEAR(cx, 5.0f, 1e-6f);
  EXPECT_NEAR(cy, 5.0f, 1e-6f);

  // Negative correlation: sxy should be positive (formula uses -dx*dy)
  EXPECT_GT(sx, 0.0f);
  EXPECT_GT(sy, 0.0f);
  EXPECT_GT(sxy, 0.0f);
}

/// @brief Test covariance with square corners.
TEST(FitTest, CovarianceSquare) {
  TestPoint points[] = {TestPoint(0.0f, 0.0f), TestPoint(10.0f, 0.0f), TestPoint(10.0f, 10.0f), TestPoint(0.0f, 10.0f)};

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float>(points, points + 4, sx, sy, sxy, cx, cy);

  // Centroid at center of square
  EXPECT_NEAR(cx, 5.0f, 1e-6f);
  EXPECT_NEAR(cy, 5.0f, 1e-6f);

  // Symmetric square -> sx == sy, sxy == 0
  EXPECT_GT(sx, 0.0f);
  EXPECT_GT(sy, 0.0f);
  EXPECT_NEAR(sx, sy, 1e-5f);
  EXPECT_NEAR(sxy, 0.0f, 1e-5f);
}

/// @brief Test covariance with double precision.
TEST(FitTest, CovarianceDoublePrecision) {
  TestPoint points[] = {TestPoint(1.0f, 2.0f), TestPoint(3.0f, 4.0f), TestPoint(5.0f, 6.0f)};

  double sx, sy, sxy, cx, cy;
  lsfm::covariance<double>(points, points + 3, sx, sy, sxy, cx, cy);

  // Centroid at mean (3, 4)
  EXPECT_NEAR(cx, 3.0, 1e-10);
  EXPECT_NEAR(cy, 4.0, 1e-10);

  // Points on line y = x + 1, so perfect correlation
  EXPECT_GT(sx, 0.0);
  EXPECT_GT(sy, 0.0);
  EXPECT_LT(sxy, 0.0);  // Negative due to formula
}

/// @brief Test weighted covariance with empty weight matrix.
TEST(FitTest, WeightedCovarianceEmptyWeights) {
  TestPoint points[] = {TestPoint(0.0f, 0.0f), TestPoint(10.0f, 10.0f)};

  cv::Mat empty_weights;  // Empty matrix -> use uniform weights

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float, TestPoint, float>(points, points + 2, sx, sy, sxy, cx, cy, empty_weights);

  // Should give same result as unweighted version
  EXPECT_NEAR(cx, 5.0f, 1e-6f);
  EXPECT_NEAR(cy, 5.0f, 1e-6f);
  EXPECT_GT(sx, 0.0f);
  EXPECT_GT(sy, 0.0f);
}

/// @brief Test weighted covariance with uniform weights.
TEST(FitTest, WeightedCovarianceUniformWeights) {
  TestPoint points[] = {TestPoint(0.0f, 0.0f), TestPoint(10.0f, 0.0f)};

  // Create weight matrix with proper size (needs to cover point locations)
  cv::Mat weights = cv::Mat::ones(1, 11, CV_32F);

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float, TestPoint, float>(points, points + 2, sx, sy, sxy, cx, cy, weights);

  // Uniform weights -> same as unweighted
  EXPECT_NEAR(cx, 5.0f, 1e-5f);
  EXPECT_NEAR(cy, 0.0f, 1e-5f);
}

/// @brief Test weighted covariance with non-uniform weights.
TEST(FitTest, WeightedCovarianceNonUniform) {
  // Two points with different weights
  TestPoint points[] = {TestPoint(0.0f, 0.0f), TestPoint(10.0f, 0.0f)};

  cv::Mat weights = cv::Mat::ones(1, 11, CV_32F);
  weights.at<float>(0, 10) = 3.0f;  // Second point has 3x weight

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float, TestPoint, float>(points, points + 2, sx, sy, sxy, cx, cy, weights);

  // Centroid should shift toward heavier point (x=10)
  // Weight 1 at x=0, weight 3 at x=10 -> cx = (0*1 + 10*3) / (1+3) = 30/4 = 7.5
  EXPECT_GT(cx, 5.0f);
  EXPECT_NEAR(cx, 7.5f, 1e-5f);
  EXPECT_NEAR(cy, 0.0f, 1e-5f);
}

/// @brief Test covariance centroid calculation accuracy.
TEST(FitTest, CovarianceCentroidAccuracy) {
  // Many points on a circle to test averaging
  std::vector<TestPoint> points;
  const int n = 100;
  for (int i = 0; i < n; ++i) {
    float angle = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(n);
    points.emplace_back(std::cos(angle) + 5.0f, std::sin(angle) + 10.0f);
  }

  float sx, sy, sxy, cx, cy;
  lsfm::covariance<float>(points.data(), points.data() + points.size(), sx, sy, sxy, cx, cy);

  // Centroid should be at circle center (5, 10)
  EXPECT_NEAR(cx, 5.0f, 0.01f);
  EXPECT_NEAR(cy, 10.0f, 0.01f);

  // Symmetric circle -> sx == sy, sxy ~= 0
  EXPECT_NEAR(sx, sy, 0.1f);
  EXPECT_NEAR(sxy, 0.0f, 0.01f);
}

}  // namespace
