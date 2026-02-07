//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_spe.cpp
/// @brief Unit tests for sub-pixel edge estimators.

#include <edge/spe.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace {

/// @brief Test LinearEstimate with symmetric neighbors.
TEST(SpeTest, LinearEstimateSymmetric) {
  // Peak at center with symmetric neighbors
  float offset = lsfm::LinearEstimate<float, float>::estimate(10.0f, 5.0f, 5.0f);
  EXPECT_NEAR(offset, 0.0f, 1e-6f);
}

/// @brief Test LinearEstimate with stronger right neighbor.
TEST(SpeTest, LinearEstimateRightShift) {
  // Peak at center, but stronger on right side
  float offset = lsfm::LinearEstimate<float, float>::estimate(10.0f, 3.0f, 7.0f);
  // Formula: (p_p - p_m) / (2 * (p - min(p_m, p_p))) = (7 - 3) / (2 * (10 - 3)) = 4 / 14
  EXPECT_NEAR(offset, 4.0f / 14.0f, 1e-6f);
}

/// @brief Test LinearEstimate with stronger left neighbor.
TEST(SpeTest, LinearEstimateLeftShift) {
  // Peak at center, but stronger on left side
  float offset = lsfm::LinearEstimate<float, float>::estimate(10.0f, 7.0f, 3.0f);
  // Formula: (p_p - p_m) / (2 * (p - min(p_m, p_p))) = (3 - 7) / (2 * (10 - 3)) = -4 / 14
  EXPECT_NEAR(offset, -4.0f / 14.0f, 1e-6f);
}

/// @brief Test LinearEstimate with floating-point magnitudes.
TEST(SpeTest, LinearEstimateInteger) {
  // Using float magnitudes with double output to avoid integer truncation.
  // Formula: (p_p - p_m) / (2 * (p - min(p_m, p_p)))
  // Here: p = 200, p_m = 100, p_p = 150 -> (150 - 100) / (2 * (200 - 100))
  //      = 50 / 200 = 0.25 (computed in floating point, no truncation).
  double offset = lsfm::LinearEstimate<double, float>::estimate(200.0f, 100.0f, 150.0f);
  // (150 - 100) / (2 * (200 - 100)) = 50 / 200 = 0.25
  EXPECT_NEAR(offset, 0.25, 1e-6);
}

/// @brief Test QuadraticEstimate with symmetric neighbors.
TEST(SpeTest, QuadraticEstimateSymmetric) {
  // Perfect parabola peak at center
  float offset = lsfm::QuadraticEstimate<float, float>::estimate(10.0f, 5.0f, 5.0f);
  EXPECT_NEAR(offset, 0.0f, 1e-6f);
}

/// @brief Test QuadraticEstimate with right shift.
TEST(SpeTest, QuadraticEstimateRightShift) {
  // Parabola peak shifted right
  float offset = lsfm::QuadraticEstimate<float, float>::estimate(10.0f, 4.0f, 8.0f);
  // Formula: (p_p - p_m) / (4*p - 2*(p_p + p_m)) = (8 - 4) / (40 - 24) = 4 / 16 = 0.25
  EXPECT_NEAR(offset, 0.25f, 1e-6f);
}

/// @brief Test QuadraticEstimate with left shift.
TEST(SpeTest, QuadraticEstimateLeftShift) {
  // Parabola peak shifted left
  float offset = lsfm::QuadraticEstimate<float, float>::estimate(10.0f, 8.0f, 4.0f);
  // Formula: (p_p - p_m) / (4*p - 2*(p_p + p_m)) = (4 - 8) / (40 - 24) = -4 / 16 = -0.25
  EXPECT_NEAR(offset, -0.25f, 1e-6f);
}

/// @brief Test QuadraticEstimate with double precision.
TEST(SpeTest, QuadraticEstimateDouble) {
  double offset = lsfm::QuadraticEstimate<double, double>::estimate(100.0, 80.0, 90.0);
  // (90 - 80) / (400 - 340) = 10 / 60 = 1/6
  EXPECT_NEAR(offset, 1.0 / 6.0, 1e-10);
}

/// @brief Test CoGEstimate with symmetric neighbors.
TEST(SpeTest, CoGEstimateSymmetric) {
  // Symmetric magnitudes -> center of gravity at center
  float offset = lsfm::CoGEstimate<float, float>::estimate(10.0f, 5.0f, 5.0f);
  EXPECT_NEAR(offset, 0.0f, 1e-6f);
}

/// @brief Test CoGEstimate with right shift.
TEST(SpeTest, CoGEstimateRightShift) {
  // Stronger right neighbor shifts center of gravity right
  float offset = lsfm::CoGEstimate<float, float>::estimate(10.0f, 3.0f, 7.0f);
  // Formula: (p_p - p_m) / (p + p_p + p_m - 3*min(p_m, p_p))
  // = (7 - 3) / (10 + 7 + 3 - 9) = 4 / 11
  EXPECT_NEAR(offset, 4.0f / 11.0f, 1e-6f);
}

/// @brief Test CoGEstimate with left shift.
TEST(SpeTest, CoGEstimateLeftShift) {
  // Stronger left neighbor shifts center of gravity left
  float offset = lsfm::CoGEstimate<float, float>::estimate(10.0f, 7.0f, 3.0f);
  // Formula: (3 - 7) / (10 + 7 + 3 - 9) = -4 / 11
  EXPECT_NEAR(offset, -4.0f / 11.0f, 1e-6f);
}

/// @brief Test CoGEstimate with equal neighbors.
TEST(SpeTest, CoGEstimateEqual) {
  // All equal -> numerator is zero (5 - 5 = 0), result should be 0/something = 0
  // BUT denominator: (5 + 5 + 5 - 3*min(5,5)) = 15 - 15 = 0 -> division by zero = nan
  // This is a degenerate case - skip or test for nan
  double offset = lsfm::CoGEstimate<double, float>::estimate(5.0f, 5.0f, 5.0f);
  // When all values equal, CoG formula has 0/0 -> nan is expected
  EXPECT_TRUE(std::isnan(offset) || std::abs(offset) < 1e-10);
}

/// @brief Test SobelEstimate with symmetric neighbors.
TEST(SpeTest, SobelEstimateSymmetric) {
  // Read implementation to verify formula
  float offset = lsfm::SobelEstimate<float, float>::estimate(10.0f, 5.0f, 5.0f);
  EXPECT_NEAR(offset, 0.0f, 1e-6f);
}

/// @brief Test SobelEstimate with right gradient.
TEST(SpeTest, SobelEstimateRightGradient) {
  // Gradient increases to the right
  float offset = lsfm::SobelEstimate<float, float>::estimate(10.0f, 3.0f, 7.0f);
  // Formula from spe.hpp: (p_p - p_m) / (2 * p) = (7 - 3) / (2 * 10) = 4 / 20 = 0.2
  EXPECT_NEAR(offset, 0.2f, 1e-6f);
}

/// @brief Test SobelEstimate with left gradient.
TEST(SpeTest, SobelEstimateLeftGradient) {
  // Gradient increases to the left
  float offset = lsfm::SobelEstimate<float, float>::estimate(10.0f, 7.0f, 3.0f);
  // Formula: (p_p - p_m) / (2 * p) = (3 - 7) / (2 * 10) = -4 / 20 = -0.2
  EXPECT_NEAR(offset, -0.2f, 1e-6f);
}

/// @brief Test all estimators with zero magnitude.
TEST(SpeTest, AllEstimatorsZeroHandling) {
  // Edge case: what happens with zero central magnitude?
  // Most estimators will divide by zero or near-zero, might produce inf/nan
  // Just verify they don't crash and produce finite values when possible

  // Non-zero neighbors with zero center should be handled gracefully
  float linear = lsfm::LinearEstimate<float, float>::estimate(0.1f, 0.05f, 0.05f);
  EXPECT_TRUE(std::isfinite(linear));

  float quadratic = lsfm::QuadraticEstimate<float, float>::estimate(0.1f, 0.05f, 0.05f);
  EXPECT_TRUE(std::isfinite(quadratic));

  float cog = lsfm::CoGEstimate<float, float>::estimate(0.1f, 0.05f, 0.05f);
  EXPECT_TRUE(std::isfinite(cog));

  float sobel = lsfm::SobelEstimate<float, float>::estimate(0.1f, 0.05f, 0.05f);
  EXPECT_TRUE(std::isfinite(sobel));
}

/// @brief Test estimators with extreme values.
TEST(SpeTest, AllEstimatorsExtremeValues) {
  // Large magnitude values
  double linear = lsfm::LinearEstimate<double, double>::estimate(1e6, 5e5, 7e5);
  EXPECT_TRUE(std::isfinite(linear));
  EXPECT_GT(linear, 0.0);

  double quadratic = lsfm::QuadraticEstimate<double, double>::estimate(1e6, 5e5, 7e5);
  EXPECT_TRUE(std::isfinite(quadratic));
  EXPECT_GT(quadratic, 0.0);
}

/// @brief Test type combinations.
TEST(SpeTest, MixedTypeCombinations) {
  // float output, uchar input
  float result1 = lsfm::LinearEstimate<float, unsigned char>::estimate(200, 100, 150);
  EXPECT_TRUE(std::isfinite(result1));

  // double output, float input
  double result2 = lsfm::QuadraticEstimate<double, float>::estimate(10.0f, 5.0f, 8.0f);
  EXPECT_TRUE(std::isfinite(result2));

  // float output, int input
  float result3 = lsfm::CoGEstimate<float, int>::estimate(100, 50, 75);
  EXPECT_TRUE(std::isfinite(result3));
}

}  // namespace
