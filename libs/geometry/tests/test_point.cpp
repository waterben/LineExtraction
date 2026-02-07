//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_point.cpp
/// @brief Unit tests for point geometry.

#include <geometry/point.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

using namespace lsfm;

class PointUtilitiesTest : public ::testing::Test {
 protected:
  PointUtilitiesTest() : test_mat(cv::Mat::zeros(10, 10, CV_32F)) {}

  void SetUp() override {
    // Create a test image/matrix
    for (int i = 0; i < test_mat.rows; ++i) {
      for (int j = 0; j < test_mat.cols; ++j) {
        test_mat.at<float>(i, j) = static_cast<float>(i * 10 + j);  // Set unique values
      }
    }
  }

  cv::Mat test_mat;
};

TEST_F(PointUtilitiesTest, SetGetWithCvPoint) {
  cv::Point p(3, 4);  // x=3, y=4

  // Set value using Point
  set(test_mat, p, 999.5f);

  // Verify the value was set correctly
  EXPECT_FLOAT_EQ(test_mat.at<float>(4, 3), 999.5f);

  // Get value using Point
  float retrieved;
  get(test_mat, p, retrieved);
  EXPECT_FLOAT_EQ(retrieved, 999.5f);
}

TEST_F(PointUtilitiesTest, SetGetWithFloatingPointCvPoint) {
  cv::Point2f p(3.7f, 4.2f);  // Should round to (4, 4)

  // Set value using floating point Point
  set(test_mat, p, 123.4f);

  // Verify the value was set at rounded coordinates
  EXPECT_FLOAT_EQ(test_mat.at<float>(4, 4), 123.4f);

  // Get value using floating point Point
  float retrieved;
  get(test_mat, p, retrieved);
  EXPECT_FLOAT_EQ(retrieved, 123.4f);
}

TEST_F(PointUtilitiesTest, SetGetWithVec2i) {
  Vec2i v(5, 6);  // x=5, y=6

  // Set value using Vec2i
  set(test_mat, v, 555.0f);

  // Verify the value was set correctly (y=6, x=5)
  EXPECT_FLOAT_EQ(test_mat.at<float>(6, 5), 555.0f);

  // Get value using Vec2i
  float retrieved;
  get(test_mat, v, retrieved);
  EXPECT_FLOAT_EQ(retrieved, 555.0f);
}

TEST_F(PointUtilitiesTest, SetGetWithFloatingVec2) {
  Vec2<float> v(2.3f, 7.8f);  // Should round to (2, 8)

  // Set value using floating Vec2
  set(test_mat, v, 777.7f);

  // Verify the value was set at rounded coordinates (y=8, x=2)
  EXPECT_FLOAT_EQ(test_mat.at<float>(8, 2), 777.7f);

  // Get value using floating Vec2
  float retrieved;
  get(test_mat, v, retrieved);
  EXPECT_FLOAT_EQ(retrieved, 777.7f);
}

TEST_F(PointUtilitiesTest, RoundingBehavior) {
  // Test rounding behavior for edge cases
  cv::Point2f p1(1.4f, 2.4f);  // Should round to (1, 2)
  cv::Point2f p2(1.6f, 2.6f);  // Should round to (2, 3)

  set(test_mat, p1, 100.0f);
  set(test_mat, p2, 200.0f);

  EXPECT_FLOAT_EQ(test_mat.at<float>(2, 1), 100.0f);
  EXPECT_FLOAT_EQ(test_mat.at<float>(3, 2), 200.0f);
}

TEST_F(PointUtilitiesTest, BoundaryValues) {
  // Test at matrix boundaries
  cv::Point corner(0, 0);
  cv::Point edge(9, 9);

  set(test_mat, corner, -1.0f);
  set(test_mat, edge, -2.0f);

  float val1, val2;
  get(test_mat, corner, val1);
  get(test_mat, edge, val2);

  EXPECT_FLOAT_EQ(val1, -1.0f);
  EXPECT_FLOAT_EQ(val2, -2.0f);
}

TEST_F(PointUtilitiesTest, DifferentMatrixTypes) {
  // Test with different matrix types
  cv::Mat int_mat = cv::Mat::zeros(5, 5, CV_32S);
  cv::Point p(2, 3);

  set(int_mat, p, 42);

  int retrieved;
  get(int_mat, p, retrieved);
  EXPECT_EQ(retrieved, 42);
}
