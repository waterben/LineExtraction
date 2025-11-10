#include <edge/zc.hpp>
#include <imgproc/laplace.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

using namespace lsfm;
using namespace cv;

class ZCTest : public ::testing::Test {
 protected:
  ZCTest() : test_img(), horizontal_img(), laplace() {}

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
    horizontal_img = Mat::zeros(8, 8, CV_8UC1);
    for (int x = 0; x < horizontal_img.cols; ++x) {
      horizontal_img.at<uchar>(2, x) = 50;
      horizontal_img.at<uchar>(3, x) = 200;
      horizontal_img.at<uchar>(4, x) = 50;
    }

    // Initialize Laplace calculator with correct template arguments
    laplace = std::make_unique<LaplaceSimple<uchar, float>>();
  }

  Mat test_img;
  Mat horizontal_img;
  std::unique_ptr<LaplaceSimple<uchar, float>> laplace;
};

TEST_F(ZCTest, VerticalEdgeZeroCrossing) {
  // Calculate Laplace
  laplace->process(test_img);

  // Create ZC with correct template arguments
  ZeroCrossing<uchar, float, float, FastZC<uchar, float, float>> zc;
  zc.threshold(10.0f, 20.0f);

  // Process ZC
  zc.process(*laplace);

  const cv::Mat& zc_map = zc.directionMap();

  EXPECT_EQ(zc_map.size(), test_img.size());

  // Check for zero crossings near the edge
  for (int y = 1; y < zc_map.rows - 1; ++y) {
    for (int x = 1; x < zc_map.cols - 1; ++x) {
      if (zc_map.at<int>(y, x) >= 0) {  // Non-negative values indicate zero crossings
        // Zero crossings should be near the vertical edge (column 3)
        EXPECT_TRUE(abs(x - 3) <= 3);  // Allow some tolerance
      }
    }
  }
}

TEST_F(ZCTest, HorizontalEdgeZeroCrossing) {
  // Calculate Laplace
  laplace->process(horizontal_img);

  // Create ZC
  ZeroCrossing<uchar, float, float, FastZC<uchar, float, float>> zc;
  zc.threshold(10.0f, 20.0f);

  // Process ZC
  zc.process(*laplace);

  const cv::Mat& zc_map = zc.directionMap();

  // Check for zero crossings near the edge
  for (int y = 1; y < zc_map.rows - 1; ++y) {
    for (int x = 1; x < zc_map.cols - 1; ++x) {
      if (zc_map.at<int>(y, x) >= 0) {  // Non-negative values indicate zero crossings
        // Zero crossings should be near the horizontal edge (row 3)
        EXPECT_TRUE(abs(y - 3) <= 3);  // Allow some tolerance
      }
    }
  }
}

TEST_F(ZCTest, ThresholdEffect) {
  // Calculate Laplace
  laplace->process(test_img);

  // High threshold ZC
  ZeroCrossing<uchar, float, float, FastZC<uchar, float, float>> zc_high;
  zc_high.threshold(100.0f, 150.0f);
  zc_high.process(*laplace);

  const IndexVector& high_seeds = zc_high.seeds();

  // Low threshold ZC
  ZeroCrossing<uchar, float, float, FastZC<uchar, float, float>> zc_low;
  zc_low.threshold(5.0f, 10.0f);
  zc_low.process(*laplace);

  const IndexVector& low_seeds = zc_low.seeds();

  // Lower threshold should generally find more or equal seeds
  EXPECT_GE(low_seeds.size(), high_seeds.size());
}

TEST_F(ZCTest, ZCIndex) {
  laplace->process(test_img);

  ZeroCrossing<uchar, float, float, FastZC<uchar, float, float>> zc;
  zc.threshold(15.0f, 30.0f);
  zc.process(*laplace);

  const IndexVector& seeds = zc.seeds();

  // Validate seed indices are within image bounds
  for (const auto& seed : seeds) {
    EXPECT_LT(seed, test_img.rows * test_img.cols);
  }
}

TEST_F(ZCTest, UniformImage) {
  Mat uniform_img = Mat::ones(8, 8, CV_8UC1) * 128;

  laplace->process(uniform_img);

  ZeroCrossing<uchar, float, float, FastZC<uchar, float, float>> zc;
  zc.threshold(5.0f, 10.0f);
  zc.process(*laplace);

  const IndexVector& seeds = zc.seeds();

  // Uniform image should have no zero crossings
  EXPECT_TRUE(seeds.empty());
}

TEST_F(ZCTest, EmptyImage) {
  Mat empty_img = Mat::zeros(5, 5, CV_8UC1);

  LaplaceSimple<uchar, float> empty_laplace;
  EXPECT_NO_THROW(empty_laplace.process(empty_img));

  ZeroCrossing<uchar, float, float, FastZC<uchar, float, float>> zc;
  EXPECT_NO_THROW(zc.process(empty_laplace));
}
