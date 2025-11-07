#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

using namespace lsfm;
using namespace cv;

class NMSTest : public ::testing::Test {
 protected:
  NMSTest() : test_img(), horizontal_edge_img(), gradient() {}

  void SetUp() override {
    // Create test images with synthetic edges

    // Vertical edge at column 3
    test_img = Mat::zeros(8, 8, CV_8UC1);
    for (int y = 0; y < test_img.rows; ++y) {
      test_img.at<uchar>(y, 2) = 50;
      test_img.at<uchar>(y, 3) = 200;
      test_img.at<uchar>(y, 4) = 50;
    }

    // Horizontal edge at row 3
    horizontal_edge_img = Mat::zeros(8, 8, CV_8UC1);
    for (int x = 0; x < horizontal_edge_img.cols; ++x) {
      horizontal_edge_img.at<uchar>(2, x) = 50;
      horizontal_edge_img.at<uchar>(3, x) = 200;
      horizontal_edge_img.at<uchar>(4, x) = 50;
    }

    // Initialize gradient calculator with float types
    gradient = std::make_unique<DerivativeGradient<uchar, float, float, float>>();
  }

  Mat test_img;
  Mat horizontal_edge_img;
  std::unique_ptr<DerivativeGradient<uchar, float, float, float>> gradient;
};

TEST_F(NMSTest, VerticalEdgeDetection) {
  // Calculate gradient
  gradient->process(test_img);

  // Create NMS with float template arguments
  NonMaximaSuppression<float, float, float, FastNMS8<float, float, float>> nms;
  nms.threshold(0.05, 0.1);  // Use relative thresholds (5% and 10% of max magnitude)

  // Process NMS
  nms.process(*gradient);

  const cv::Mat& edges = nms.directionMap();

  // Check that we found edges
  EXPECT_FALSE(edges.empty());
  EXPECT_EQ(edges.size(), test_img.size());

  // The vertical edge transitions should be detected at columns 2 and 4 (not at the peak at column 3)
  bool edge_found = false;
  for (int y = 1; y < edges.rows - 1; ++y) {
    // Check column 2 or column 4 for edge transitions
    if (edges.at<char>(y, 2) >= 0 || edges.at<char>(y, 4) >= 0) {
      edge_found = true;
      break;
    }
  }
  EXPECT_TRUE(edge_found);
}

TEST_F(NMSTest, HorizontalEdgeDetection) {
  // Calculate gradient
  gradient->process(horizontal_edge_img);

  // Create NMS with float template arguments
  NonMaximaSuppression<float, float, float, FastNMS8<float, float, float>> nms;
  nms.threshold(0.05, 0.1);  // Use relative thresholds (5% and 10% of max magnitude)

  // Process NMS
  nms.process(*gradient);

  const cv::Mat& edges = nms.directionMap();

  // Check that we found edges
  EXPECT_FALSE(edges.empty());

  // The horizontal edge transitions should be detected at rows 2 and 4 (not at the peak at row 3)
  bool edge_found = false;
  for (int x = 1; x < edges.cols - 1; ++x) {
    // Check row 2 or row 4 for edge transitions
    if (edges.at<char>(2, x) >= 0 || edges.at<char>(4, x) >= 0) {
      edge_found = true;
      break;
    }
  }
  EXPECT_TRUE(edge_found);
}

TEST_F(NMSTest, ThresholdSettings) {
  // Calculate gradient
  gradient->process(test_img);

  // Create NMS with high thresholds - using float template arguments
  NonMaximaSuppression<float, float, float, FastNMS8<float, float, float>> nms;
  nms.threshold(500.0f, 600.0f);

  // Process NMS - should find no edges due to high threshold
  nms.process(*gradient);

  const IndexVector& seeds = nms.seeds();

  // Should find no seeds with very high threshold
  EXPECT_TRUE(seeds.empty());
}

TEST_F(NMSTest, EdgeIndex) {
  // Calculate gradient
  gradient->process(test_img);

  NonMaximaSuppression<float, float, float, FastNMS8<float, float, float>> nms;
  nms.threshold(0.005, 0.015);  // Use relative thresholds to detect edges in test image

  // Process NMS
  nms.process(*gradient);

  const IndexVector& seeds = nms.seeds();

  // Edge detection may or may not find seeds depending on algorithm parameters
  // The important thing is that it runs without error and produces valid results
  EXPECT_TRUE(seeds.size() > 0U);  // Should be a valid vector (always true, but tests access)

  // If there are seeds, validate they're in bounds
  for (const auto& seed : seeds) {
    EXPECT_LT(seed, test_img.rows * test_img.cols);
  }
}

TEST_F(NMSTest, EmptyImage) {
  Mat empty_img = Mat::zeros(5, 5, CV_8UC1);

  DerivativeGradient<uchar, float, float, float> empty_gradient;
  empty_gradient.process(empty_img);

  NonMaximaSuppression<float, float, float, FastNMS8<float, float, float>> nms;
  EXPECT_NO_THROW(nms.process(empty_gradient));
}

TEST_F(NMSTest, UniformImage) {
  Mat uniform_img = Mat::ones(8, 8, CV_8UC1) * 128;

  gradient->process(uniform_img);

  NonMaximaSuppression<float, float, float, FastNMS8<float, float, float>> nms;
  nms.threshold(10.0f, 20.0f);

  // Process NMS
  nms.process(*gradient);

  const IndexVector& seeds = nms.seeds();

  // Uniform image should have no edges
  EXPECT_TRUE(seeds.empty());
}
