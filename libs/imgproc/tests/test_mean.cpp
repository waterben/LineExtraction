#include <imgproc/mean.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

using namespace lsfm;
using namespace cv;

class MeanTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create test image with gradient
    test_img = Mat::zeros(10, 10, CV_32F);
    for (int y = 0; y < 10; ++y) {
      for (int x = 0; x < 10; ++x) {
        test_img.at<float>(y, x) = static_cast<float>(x) + static_cast<float>(y) * 0.1f;  // Simple gradient
      }
    }
  }

  Mat test_img;
};

TEST_F(MeanTest, HorizontalLineSegment) {
  // Create horizontal line segment from (1,5) to (8,5)
  LineSegment<float> line(Point2f(1, 5), Point2f(8, 5));

  // Calculate mean along the line
  float mean_val = Mean<float, float>::process(test_img, line, 1.0f);

  // Expected mean should be around the middle x coordinate at y=5
  float expected = (1 + 8) / 2.0f + 5 * 0.1f;  // x_avg + y * 0.1

  EXPECT_NEAR(mean_val, expected, 0.5f);
}

TEST_F(MeanTest, VerticalLineSegment) {
  // Create vertical line segment from (5,1) to (5,8)
  LineSegment<float> line(Point2f(5, 1), Point2f(5, 8));

  // Calculate mean along the line
  float mean_val = Mean<float, float>::process(test_img, line, 1.0f);

  // Expected mean should be around x=5 and middle y coordinate
  float expected = 5 + (1 + 8) / 2.0f * 0.1f;  // x + y_avg * 0.1

  EXPECT_NEAR(mean_val, expected, 0.5f);
}

TEST_F(MeanTest, DiagonalLineSegment) {
  // Create diagonal line segment from (2,2) to (7,7)
  LineSegment<float> line(Point2f(2, 2), Point2f(7, 7));

  // Calculate mean along the line
  float mean_val = Mean<float, float>::process(test_img, line, 1.0f);

  // For diagonal line, x and y change proportionally
  float expected = (2 + 7) / 2.0f + (2 + 7) / 2.0f * 0.1f;  // avg_x + avg_y * 0.1

  EXPECT_NEAR(mean_val, expected, 0.5f);
}

TEST_F(MeanTest, MeanAndVariance) {
  // Create horizontal line with uniform values for simple variance test
  Mat uniform_img = Mat::ones(5, 10, CV_32F) * 3.0f;  // All pixels = 3.0

  LineSegment<float> line(Point2f(1, 2), Point2f(8, 2));

  float variance;
  float mean_val = Mean<float, float>::process(variance, uniform_img, line, 1.0f);

  EXPECT_NEAR(mean_val, 3.0f, 1e-6);
  EXPECT_NEAR(variance, 0.0f, 1e-6);  // Variance should be 0 for uniform values
}

TEST_F(MeanTest, ShortLineSegment) {
  // Create very short line segment
  LineSegment<float> line(Point2f(5, 5), Point2f(5.1f, 5.1f));

  // Calculate mean along the line
  float mean_val = Mean<float, float>::process(test_img, line, 0.1f);

  // Should be close to the value at (5,5)
  float expected = 5 + 5 * 0.1f;

  EXPECT_NEAR(mean_val, expected, 0.2f);
}

TEST_F(MeanTest, LineOutsideBounds) {
  // Create line segment that goes outside image bounds
  LineSegment<float> line(Point2f(-1, 5), Point2f(15, 5));

  // Should still work due to trimming to box
  float mean_val = Mean<float, float>::process(test_img, line, 1.0f);

  // Should be a reasonable value (not NaN or infinite)
  EXPECT_TRUE(std::isfinite(mean_val));
  EXPECT_GE(mean_val, 0.0f);
}
