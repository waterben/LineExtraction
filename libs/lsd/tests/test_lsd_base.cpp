/**
 * @file test_lsd_base.cpp
 * @brief Unit tests for LsdBase class interface
 *
 * Tests the base class API methods including:
 * - lineSegments() getter
 * - endPoints() conversion and getter
 * - lines() conversion from segments
 * - Various detect() overloads
 */

#include <lsd/lsd_cc.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

#include <gtest/gtest.h>

using namespace lsfm;
using namespace cv;

/// @brief Test fixture for LsdBase API tests
class LsdBaseTest : public ::testing::Test {
 protected:
  LsdBaseTest() : test_img() {}

  void SetUp() override {
    // Create simple test image with clear horizontal and vertical lines
    test_img = Mat::zeros(100, 100, CV_8UC1);

    // Horizontal line
    line(test_img, Point(20, 30), Point(80, 30), Scalar(255), 2);

    // Vertical line
    line(test_img, Point(50, 20), Point(50, 80), Scalar(255), 2);

    GaussianBlur(test_img, test_img, Size(3, 3), 0.8);
  }

  Mat test_img;
};

/// @brief Test basic lineSegments() getter
TEST_F(LsdBaseTest, LineSegmentsGetter) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  const auto& segments = lsd.lineSegments();

  // Should return a vector
  EXPECT_GT(segments.size(), 0u);

  // Each segment should have valid length
  for (const auto& seg : segments) {
    EXPECT_GT(seg.length(), 0.0f);
    EXPECT_FALSE(std::isnan(seg.length()));
    EXPECT_FALSE(std::isinf(seg.length()));
  }
}

/// @brief Test endPoints() conversion and getter
TEST_F(LsdBaseTest, EndPointsConversion) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  const auto& segments = lsd.lineSegments();
  const auto& endpoints = lsd.endPoints();

  // Same number of endpoints as segments
  EXPECT_EQ(endpoints.size(), segments.size());

  // Check endpoint format (x1, y1, x2, y2)
  for (size_t i = 0; i < endpoints.size(); ++i) {
    const auto& ep = endpoints[i];
    const auto& seg = segments[i];

    // Endpoints should be within image bounds
    EXPECT_GE(ep[0], 0.0f);  // x1
    EXPECT_GE(ep[1], 0.0f);  // y1
    EXPECT_GE(ep[2], 0.0f);  // x2
    EXPECT_GE(ep[3], 0.0f);  // y2

    EXPECT_LT(ep[0], static_cast<float>(test_img.cols));
    EXPECT_LT(ep[1], static_cast<float>(test_img.rows));
    EXPECT_LT(ep[2], static_cast<float>(test_img.cols));
    EXPECT_LT(ep[3], static_cast<float>(test_img.rows));

    // Endpoints should match segment endpoints
    EXPECT_FLOAT_EQ(ep[0], seg.startPoint().x());
    EXPECT_FLOAT_EQ(ep[1], seg.startPoint().y());
    EXPECT_FLOAT_EQ(ep[2], seg.endPoint().x());
    EXPECT_FLOAT_EQ(ep[3], seg.endPoint().y());
  }
}

/// @brief Test lines() conversion from segments
TEST_F(LsdBaseTest, LinesConversion) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  const auto& segments = lsd.lineSegments();
  const auto& lines = lsd.lines();

  // Same number of lines as segments
  EXPECT_EQ(lines.size(), segments.size());

  // Each line should have valid parameters
  for (const auto& l : lines) {
    EXPECT_FALSE(std::isnan(l.normalAngle()));
    EXPECT_FALSE(std::isinf(l.normalAngle()));
  }
}

/// @brief Test that we can manually copy segments via getter
TEST_F(LsdBaseTest, ManualSegmentCopy) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  // Manually copy via getter
  auto output_segments = lsd.lineSegments();

  // Should have copied data
  EXPECT_GT(output_segments.size(), 0u);

  for (size_t i = 0; i < output_segments.size(); ++i) {
    EXPECT_GT(output_segments[i].length(), 0.0f);
  }
}

/// @brief Test that we can manually copy endpoints via getter
TEST_F(LsdBaseTest, ManualEndPointCopy) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  // Manually copy via getter
  auto output_endpoints = lsd.endPoints();

  // Should have copied data
  EXPECT_GT(output_endpoints.size(), 0u);

  for (const auto& ep : output_endpoints) {
    EXPECT_FALSE(std::isnan(ep[0]));
    EXPECT_FALSE(std::isnan(ep[1]));
    EXPECT_FALSE(std::isnan(ep[2]));
    EXPECT_FALSE(std::isnan(ep[3]));
  }
}

/// @brief Test imageData() getter
TEST_F(LsdBaseTest, ImageDataGetter) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  // Get image data via imageData() method
  const auto& data = lsd.imageData();

  // Should have some auxiliary data
  EXPECT_GT(data.size(), 0u);

  // Check that we can access gradient data by name
  const Mat& gx = lsd.imageData("gx");
  const Mat& gy = lsd.imageData("gy");

  EXPECT_FALSE(gx.empty());
  EXPECT_FALSE(gy.empty());
}

/// @brief Test imageDataDescriptor()
TEST_F(LsdBaseTest, ImageDataDescriptor) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  // Get image data descriptor
  const auto& descriptor = lsd.imageDataDescriptor();

  // Should have descriptors for gradient data
  EXPECT_GT(descriptor.size(), 0u);
}

/// @brief Test multiple consecutive detections
TEST_F(LsdBaseTest, ConsecutiveDetections) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  // First detection
  lsd.detect(test_img);
  size_t count1 = lsd.lineSegments().size();

  // Second detection - should clear previous results
  lsd.detect(test_img);
  size_t count2 = lsd.lineSegments().size();

  // Should produce consistent results
  EXPECT_EQ(count1, count2);
}

/// @brief Test that endPoints() is lazily computed
TEST_F(LsdBaseTest, LazyEndPointComputation) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  // First call computes endpoints
  const auto& endpoints1 = lsd.endPoints();
  size_t size1 = endpoints1.size();

  // Second call returns cached value
  const auto& endpoints2 = lsd.endPoints();
  size_t size2 = endpoints2.size();

  EXPECT_EQ(size1, size2);
  EXPECT_GT(size1, 0u);
}

/// @brief Test that lines() is lazily computed from segments
TEST_F(LsdBaseTest, LazyLinesComputation) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  // First call computes lines from segments
  const auto& lines1 = lsd.lines();
  size_t size1 = lines1.size();

  // Second call returns cached value
  const auto& lines2 = lsd.lines();
  size_t size2 = lines2.size();

  EXPECT_EQ(size1, size2);
  EXPECT_GT(size1, 0u);
}

/// @brief Test detection with different image types
TEST_F(LsdBaseTest, DifferentImageTypes) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  // 8-bit grayscale
  Mat img_8u = test_img.clone();
  EXPECT_NO_THROW(lsd.detect(img_8u));
  size_t count_8u = lsd.lineSegments().size();
  EXPECT_GT(count_8u, 0u);

  // 32-bit float grayscale (keep 0-255 range, not normalized)
  Mat img_32f;
  test_img.convertTo(img_32f, CV_32F);  // Direct conversion, not normalized
  EXPECT_NO_THROW(lsd.detect(img_32f));
  size_t count_32f = lsd.lineSegments().size();
  EXPECT_GT(count_32f, 0u);

  // Results should be similar (not necessarily identical due to precision)
  EXPECT_NEAR(static_cast<double>(count_8u), static_cast<double>(count_32f),
              static_cast<double>(count_8u) * 0.3);  // Within 30%
}

/// @brief Test segment properties
TEST_F(LsdBaseTest, SegmentProperties) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  lsd.detect(test_img);

  const auto& segments = lsd.lineSegments();
  ASSERT_GT(segments.size(), 0u);

  for (const auto& seg : segments) {
    // Check basic properties
    EXPECT_FALSE(std::isnan(seg.length()));
    EXPECT_FALSE(std::isnan(seg.angle()));
    EXPECT_FALSE(std::isnan(seg.centerPoint().x()));
    EXPECT_FALSE(std::isnan(seg.centerPoint().y()));

    // Length should match distance between endpoints
    float dx = seg.endPoint().x() - seg.startPoint().x();
    float dy = seg.endPoint().y() - seg.startPoint().y();
    float computed_length = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(seg.length(), computed_length, 0.01f);
  }
}

/// @brief Test empty image handling
TEST_F(LsdBaseTest, EmptyImageHandling) {
  LsdCC<float> lsd(0.004f, 0.012f, 10, 2, 2, CC_FIND_NEAR_COMPLEX);

  Mat empty_img = Mat::zeros(50, 50, CV_8UC1);

  EXPECT_NO_THROW(lsd.detect(empty_img));

  const auto& segments = lsd.lineSegments();
  const auto& endpoints = lsd.endPoints();
  const auto& lines = lsd.lines();

  // Should all be empty or have matching sizes
  EXPECT_EQ(endpoints.size(), segments.size());
  EXPECT_EQ(lines.size(), segments.size());
}
