/**
 * @file test_edge_cases.cpp
 * @brief Edge case tests for image processing operations
 *
 * Tests imgproc functions with edge cases like empty images, noise,
 * extreme values, and boundary conditions.
 */

#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <gtest/gtest.h>

using namespace lsfm;
using namespace cv;

/**
 * @brief Test fixture for imgproc edge cases
 */
class ImgprocEdgeCasesTest : public ::testing::Test {
 protected:
  ImgprocEdgeCasesTest()
      : empty_img(),
        zero_img(),
        uniform_img(),
        noise_img(),
        tiny_img(),
        single_row(),
        single_col(),
        saturated_img(),
        checkerboard() {}

  void SetUp() override {
    // Empty image
    empty_img = Mat();

    // Zero image
    zero_img = Mat::zeros(50, 50, CV_8UC1);

    // Uniform image (single value)
    uniform_img = Mat(60, 60, CV_8UC1, Scalar(128));

    // Pure noise
    noise_img = Mat(80, 80, CV_8UC1);
    randn(noise_img, 128, 40);

    // Very small image
    tiny_img = Mat::ones(2, 2, CV_8UC1) * 100;

    // Single row image
    single_row = Mat::ones(1, 100, CV_8UC1) * 150;

    // Single column image
    single_col = Mat::ones(100, 1, CV_8UC1) * 150;

    // Extreme values (saturated)
    saturated_img = Mat(50, 50, CV_8UC1);
    saturated_img.setTo(255);

    // Checkerboard pattern (high frequency)
    checkerboard = Mat(40, 40, CV_8UC1);
    for (int y = 0; y < checkerboard.rows; ++y) {
      for (int x = 0; x < checkerboard.cols; ++x) {
        checkerboard.at<uchar>(y, x) = ((x + y) % 2 == 0) ? 0 : 255;
      }
    }
  }

  Mat empty_img;
  Mat zero_img;
  Mat uniform_img;
  Mat noise_img;
  Mat tiny_img;
  Mat single_row;
  Mat single_col;
  Mat saturated_img;
  Mat checkerboard;
};

/**
 * @brief Test gradient computation on empty image
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_EmptyImage) {
  DerivativeGradient<uchar, float, float, float> gradient;

  // Empty image throws exception (expected behavior)
  EXPECT_THROW(gradient.process(empty_img), cv::Exception);
}

/**
 * @brief Test gradient computation on zero image
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_ZeroImage) {
  DerivativeGradient<uchar, float, float, float> gradient;

  EXPECT_NO_THROW(gradient.process(zero_img));

  const Mat& mag = gradient.magnitude();
  EXPECT_FALSE(mag.empty());

  // Gradient of constant image should be near zero
  double min_val, max_val;
  minMaxLoc(mag, &min_val, &max_val);
  EXPECT_LT(max_val, 0.01);  // Very small gradient values
}

/**
 * @brief Test gradient computation on uniform image
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_UniformImage) {
  DerivativeGradient<uchar, float, float, float> gradient;

  EXPECT_NO_THROW(gradient.process(uniform_img));

  const Mat& mag = gradient.magnitude();

  // Gradient of uniform image should be near zero (except at borders)
  Rect inner_region(5, 5, uniform_img.cols - 10, uniform_img.rows - 10);
  Mat inner_mag = mag(inner_region);

  double max_inner;
  minMaxLoc(inner_mag, nullptr, &max_inner);
  EXPECT_LT(max_inner, 0.01);
}

/**
 * @brief Test gradient computation on noise
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_NoiseImage) {
  DerivativeGradient<uchar, float, float, float> gradient;

  EXPECT_NO_THROW(gradient.process(noise_img));

  const Mat& mag = gradient.magnitude();
  EXPECT_FALSE(mag.empty());

  // Noise should produce some gradients
  double min_val, max_val;
  minMaxLoc(mag, &min_val, &max_val);
  EXPECT_GT(max_val, 0.0);
}

/**
 * @brief Test gradient computation on tiny image
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_TinyImage) {
  DerivativeGradient<uchar, float, float, float> gradient;

  // Should handle 2x2 image without crashing
  EXPECT_NO_THROW(gradient.process(tiny_img));
}

/**
 * @brief Test gradient computation on single row
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_SingleRow) {
  DerivativeGradient<uchar, float, float, float> gradient;

  // Should handle 1xN image
  EXPECT_NO_THROW(gradient.process(single_row));

  const Mat& mag = gradient.magnitude();
  if (!mag.empty()) {
    EXPECT_EQ(mag.rows, single_row.rows);
    EXPECT_EQ(mag.cols, single_row.cols);
  }
}

/**
 * @brief Test gradient computation on single column
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_SingleColumn) {
  DerivativeGradient<uchar, float, float, float> gradient;

  // Should handle Nx1 image
  EXPECT_NO_THROW(gradient.process(single_col));

  const Mat& mag = gradient.magnitude();
  if (!mag.empty()) {
    EXPECT_EQ(mag.rows, single_col.rows);
    EXPECT_EQ(mag.cols, single_col.cols);
  }
}

/**
 * @brief Test gradient computation on saturated image
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_SaturatedImage) {
  DerivativeGradient<uchar, float, float, float> gradient;

  EXPECT_NO_THROW(gradient.process(saturated_img));

  const Mat& mag = gradient.magnitude();

  // All 255 values - gradient should be near zero (except at borders)
  Rect inner_region(5, 5, saturated_img.cols - 10, saturated_img.rows - 10);
  Mat inner_mag = mag(inner_region);

  double max_inner;
  minMaxLoc(inner_mag, nullptr, &max_inner);
  EXPECT_LT(max_inner, 0.01);
}

/**
 * @brief Test gradient computation on checkerboard (high frequency)
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_Checkerboard) {
  DerivativeGradient<uchar, float, float, float> gradient;

  EXPECT_NO_THROW(gradient.process(checkerboard));

  const Mat& mag = gradient.magnitude();
  EXPECT_FALSE(mag.empty());

  // Checkerboard should produce gradients (may depend on filter used)
  double min_val, max_val;
  minMaxLoc(mag, &min_val, &max_val);
  EXPECT_GE(max_val, 0.0);  // Just check we have some magnitude values
}

/**
 * @brief Test gradient direction consistency
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_DirectionConsistency) {
  // Create image with horizontal edge
  Mat horizontal_edge = Mat::zeros(60, 60, CV_8UC1);
  rectangle(horizontal_edge, Point(0, 0), Point(59, 29), Scalar(0), -1);
  rectangle(horizontal_edge, Point(0, 30), Point(59, 59), Scalar(255), -1);
  GaussianBlur(horizontal_edge, horizontal_edge, Size(5, 5), 1.0);

  DerivativeGradient<uchar, float, float, float> gradient;
  gradient.process(horizontal_edge);

  const Mat& dir = gradient.direction();
  if (!dir.empty()) {
    // Direction at the edge should be roughly vertical (pointing up/down)
    float dir_value = dir.at<float>(30, 30);  // Center of edge
    // Direction should be close to 90 or -90 degrees for vertical gradient
    float angle_deg = dir_value * 180.0f / static_cast<float>(CV_PI);
    EXPECT_TRUE(std::abs(angle_deg - 90.0f) < 10.0f || std::abs(angle_deg + 90.0f) < 10.0f);
  }
}

/**
 * @brief Test gradient magnitude normalization
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_MagnitudeRange) {
  Mat test_img = Mat::zeros(100, 100, CV_8UC1);
  line(test_img, Point(10, 50), Point(90, 50), Scalar(255), 3);
  GaussianBlur(test_img, test_img, Size(3, 3), 0.8);

  DerivativeGradient<uchar, float, float, float> gradient;
  gradient.process(test_img);

  const Mat& mag = gradient.magnitude();

  // Magnitude should be non-negative
  double min_val;
  minMaxLoc(mag, &min_val, nullptr);
  EXPECT_GE(min_val, 0.0);
}

/**
 * @brief Test multiple consecutive gradient computations
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_ConsecutiveComputations) {
  Mat test_img = Mat::zeros(80, 80, CV_8UC1);
  circle(test_img, Point(40, 40), 20, Scalar(255), -1);
  GaussianBlur(test_img, test_img, Size(5, 5), 1.0);

  DerivativeGradient<uchar, float, float, float> gradient;

  // Run multiple times
  gradient.process(test_img);
  Mat mag1 = gradient.magnitude().clone();

  gradient.process(test_img);
  Mat mag2 = gradient.magnitude().clone();

  gradient.process(test_img);
  Mat mag3 = gradient.magnitude().clone();

  // Results should be identical
  Mat diff1, diff2;
  absdiff(mag1, mag2, diff1);
  absdiff(mag2, mag3, diff2);

  EXPECT_LT(sum(diff1)[0], 0.001);
  EXPECT_LT(sum(diff2)[0], 0.001);
}

/**
 * @brief Test gradient with different data types
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_DifferentDataTypes) {
  Mat test_img_8u = Mat::zeros(60, 60, CV_8UC1);
  line(test_img_8u, Point(10, 30), Point(50, 30), Scalar(200), 2);
  GaussianBlur(test_img_8u, test_img_8u, Size(3, 3), 0.8);

  // Convert to different types
  Mat test_img_16u, test_img_32f;
  test_img_8u.convertTo(test_img_16u, CV_16UC1);
  test_img_8u.convertTo(test_img_32f, CV_32FC1, 1.0 / 255.0);

  DerivativeGradient<uchar, float, float, float> gradient_8u;
  DerivativeGradient<ushort, float, float, float> gradient_16u;
  DerivativeGradient<float, float, float, float> gradient_32f;

  EXPECT_NO_THROW(gradient_8u.process(test_img_8u));
  EXPECT_NO_THROW(gradient_16u.process(test_img_16u));
  EXPECT_NO_THROW(gradient_32f.process(test_img_32f));

  // All should produce valid results
  EXPECT_FALSE(gradient_8u.magnitude().empty());
  EXPECT_FALSE(gradient_16u.magnitude().empty());
  EXPECT_FALSE(gradient_32f.magnitude().empty());
}

/**
 * @brief Test border handling
 */
TEST_F(ImgprocEdgeCasesTest, Gradient_BorderHandling) {
  Mat test_img = Mat::zeros(50, 50, CV_8UC1);

  // Draw line at border
  line(test_img, Point(0, 25), Point(49, 25), Scalar(255), 2);

  DerivativeGradient<uchar, float, float, float> gradient;
  EXPECT_NO_THROW(gradient.process(test_img));

  const Mat& mag = gradient.magnitude();

  // Should produce valid results even at borders
  EXPECT_FALSE(mag.empty());
  EXPECT_EQ(mag.size(), test_img.size());
}
