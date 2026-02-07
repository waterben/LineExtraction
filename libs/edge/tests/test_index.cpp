//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_index.cpp
/// @brief Unit tests for index utilities.

#include <edge/index.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

using namespace lsfm;

class IndexTest : public ::testing::Test {
 protected:
  IndexTest() : test_mat(), cols(0), rows(0) {}

  void SetUp() override {
    // Create test matrix 5x4 (5 rows, 4 columns)
    test_mat = cv::Mat::zeros(5, 4, CV_32F);
    cols = test_mat.cols;  // 4
    rows = test_mat.rows;  // 5

    // Fill with unique values
    for (std::size_t i = 0; i < test_mat.total(); ++i) {
      test_mat.ptr<float>()[i] = static_cast<float>(i);
    }
  }

  cv::Mat test_mat;
  int cols, rows;
};

TEST_F(IndexTest, NegSignFunction) {
  // Test integer version
  EXPECT_TRUE(neg_sign(-5, 3));
  EXPECT_TRUE(neg_sign(5, -3));
  EXPECT_FALSE(neg_sign(5, 3));
  EXPECT_FALSE(neg_sign(-5, -3));
  EXPECT_FALSE(neg_sign(0, 0));

  // Test float version
  EXPECT_TRUE(neg_sign(-2.5f, 1.5f));
  EXPECT_TRUE(neg_sign(2.5f, -1.5f));
  EXPECT_FALSE(neg_sign(2.5f, 1.5f));
  EXPECT_FALSE(neg_sign(-2.5f, -1.5f));

  // Test double version
  EXPECT_TRUE(neg_sign(-2.5, 1.5));
  EXPECT_TRUE(neg_sign(2.5, -1.5));
  EXPECT_FALSE(neg_sign(2.5, 1.5));
  EXPECT_FALSE(neg_sign(-2.5, -1.5));
}

TEST_F(IndexTest, SetGetByIndex) {
  index_type idx = 7;  // Some index in matrix

  // Set value by index
  set(test_mat, idx, 999.5f);
  EXPECT_FLOAT_EQ(test_mat.ptr<float>()[idx], 999.5f);

  // Get value by index (reference version)
  float retrieved;
  get(test_mat, idx, retrieved);
  EXPECT_FLOAT_EQ(retrieved, 999.5f);

  // Get value by index (return version)
  float retrieved2 = get<float>(test_mat, idx);
  EXPECT_FLOAT_EQ(retrieved2, 999.5f);
}

TEST_F(IndexTest, GetXYFromIndex) {
  // Matrix is 5x4, so:
  // index 0 = (0,0), index 1 = (1,0), index 4 = (0,1), index 5 = (1,1)

  EXPECT_EQ(getX<int>(0, cols), 0);
  EXPECT_EQ(getY<int>(0, cols), 0);

  EXPECT_EQ(getX<int>(1, cols), 1);
  EXPECT_EQ(getY<int>(1, cols), 0);

  EXPECT_EQ(getX<int>(4, cols), 0);
  EXPECT_EQ(getY<int>(4, cols), 1);

  EXPECT_EQ(getX<int>(5, cols), 1);
  EXPECT_EQ(getY<int>(5, cols), 1);

  EXPECT_EQ(getX<int>(7, cols), 3);
  EXPECT_EQ(getY<int>(7, cols), 1);
}

TEST_F(IndexTest, Index2Point) {
  cv::Point p;

  // Test various indices
  index2Point(0, p, cols);
  EXPECT_EQ(p.x, 0);
  EXPECT_EQ(p.y, 0);

  index2Point(1, p, cols);
  EXPECT_EQ(p.x, 1);
  EXPECT_EQ(p.y, 0);

  index2Point(4, p, cols);
  EXPECT_EQ(p.x, 0);
  EXPECT_EQ(p.y, 1);

  index2Point(7, p, cols);
  EXPECT_EQ(p.x, 3);
  EXPECT_EQ(p.y, 1);
}

TEST_F(IndexTest, Point2Index) {
  index_type idx;

  // Test cv::Point version
  cv::Point p1(0, 0);
  point2Index(p1, idx, cols);
  EXPECT_EQ(idx, 0);

  cv::Point p2(1, 0);
  point2Index(p2, idx, cols);
  EXPECT_EQ(idx, 1);

  cv::Point p3(3, 1);
  point2Index(p3, idx, cols);
  EXPECT_EQ(idx, 7);

  // Test Vec2i version
  Vec2i v(2, 2);
  point2Index(v, idx, cols);
  EXPECT_EQ(idx, 10);  // 2*4 + 2 = 10
}

TEST_F(IndexTest, Point2IndexReturnVersion) {
  // Test cv::Point version
  EXPECT_EQ(point2Index(cv::Point(0, 0), cols), 0);
  EXPECT_EQ(point2Index(cv::Point(1, 0), cols), 1);
  EXPECT_EQ(point2Index(cv::Point(3, 1), cols), 7);

  // Test Vec2i version
  EXPECT_EQ(point2Index(Vec2i(2, 2), cols), 10);
}

TEST_F(IndexTest, FloatingPointConversion) {
  cv::Point2f pf(1.7f, 2.3f);  // Should round to (2, 2)
  index_type idx = point2Index(pf, cols);
  EXPECT_EQ(idx, 10);  // 2*4 + 2 = 10

  // Test rounding behavior
  cv::Point2f pf2(1.4f, 2.6f);  // Should round to (1, 3)
  index_type idx2 = point2Index(pf2, cols);
  EXPECT_EQ(idx2, 13);  // 3*4 + 1 = 13
}

TEST_F(IndexTest, BulkConversion) {
  // Test bulk index to point conversion
  std::vector<index_type> indices = {0, 1, 4, 7, 10};
  std::vector<cv::Point> points(indices.size());

  index2Point(indices.data(), indices.data() + indices.size(), points.data(), cols);

  EXPECT_EQ(points[0], cv::Point(0, 0));
  EXPECT_EQ(points[1], cv::Point(1, 0));
  EXPECT_EQ(points[2], cv::Point(0, 1));
  EXPECT_EQ(points[3], cv::Point(3, 1));
  EXPECT_EQ(points[4], cv::Point(2, 2));
}

TEST_F(IndexTest, RoundTripConversion) {
  // Test that index -> point -> index gives same result
  index_type original_idx = 13;
  cv::Point p;
  index_type converted_idx;

  index2Point(original_idx, p, cols);
  point2Index(p, converted_idx, cols);

  EXPECT_EQ(original_idx, converted_idx);
}

TEST_F(IndexTest, EdgeCases) {
  // Test first and last valid indices
  index_type last_idx = static_cast<index_type>(rows * cols - 1);  // 19 for 5x4 matrix

  cv::Point p;
  index2Point(last_idx, p, cols);
  EXPECT_EQ(p.x, 3);  // cols-1
  EXPECT_EQ(p.y, 4);  // rows-1

  index_type converted = point2Index(p, cols);
  EXPECT_EQ(converted, last_idx);
}
