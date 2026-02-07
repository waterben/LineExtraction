//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_derivative.cpp
/// @brief Unit tests for derivative operators.

#include <imgproc/derivative.hpp>

#include <gtest/gtest.h>

using lsfm::GaussianDerivative;
using lsfm::PrewittDerivative;
using lsfm::RobertsDerivative;
using lsfm::ScharrDerivative;
using lsfm::SobelDerivative;

static cv::Mat makeRamp3x3() {
  // 3x3 ramp image
  return (cv::Mat_<uchar>(3, 3) << 10, 20, 30, 40, 50, 60, 70, 80, 90);
}

static cv::Mat makeStep() {
  // Vertical step: left half 0, right half 255
  cv::Mat img(7, 8, CV_8U, cv::Scalar(0));
  img.colRange(img.cols / 2, img.cols).setTo(255);
  return img;
}

TEST(DerivativeTest, RobertsProcess) {
  RobertsDerivative<uchar, short> rob;
  cv::Mat gx, gy;
  rob.process(makeStep(), gx, gy);
  ASSERT_EQ(gx.type(), CV_16S);
  ASSERT_EQ(gy.type(), CV_16S);
  // x-gradient along step column should have non-zero energy
  int stepCol = gx.cols / 2;
  cv::Mat m =
      cv::abs(gx.col(stepCol)) + cv::abs(gy.col(stepCol)) + cv::abs(gx.col(stepCol - 1)) + cv::abs(gy.col(stepCol - 1));
  cv::Scalar s = cv::sum(m);
  EXPECT_GT(s[0], 0.0);
}

TEST(DerivativeTest, PrewittProcess) {
  PrewittDerivative<uchar, short> prew;
  cv::Mat gx, gy;
  prew.process(makeStep(), gx, gy);
  ASSERT_EQ(gx.type(), CV_16S);
  ASSERT_EQ(gy.type(), CV_16S);
  int stepCol2 = gx.cols / 2;
  cv::Scalar s2 = cv::sum(cv::abs(gx.col(stepCol2)));
  EXPECT_GT(s2[0], 0.0);
}

TEST(DerivativeTest, SobelProcessAndKernelSize) {
  SobelDerivative<uchar, short> sobel(2);  // even will be corrected
  EXPECT_EQ(sobel.kernelSize(), 3);

  cv::Mat gx, gy;
  sobel.process(makeRamp3x3(), gx, gy);
  ASSERT_EQ(gx.type(), CV_16S);
  ASSERT_EQ(gy.type(), CV_16S);
  ASSERT_EQ(gx.size(), gy.size());
  // central pixel should have some gradient
  EXPECT_NE(gx.at<short>(1, 1), 0);
  EXPECT_NE(gy.at<short>(1, 1), 0);
}

TEST(DerivativeTest, ScharrProcess) {
  ScharrDerivative<uchar, short> scharr;
  cv::Mat gx, gy;
  scharr.process(makeRamp3x3(), gx, gy);
  ASSERT_EQ(gx.type(), CV_16S);
  ASSERT_EQ(gy.type(), CV_16S);
}

TEST(DerivativeTest, GaussianDerivativeParams) {
  GaussianDerivative<uchar, float> gauss(5, 3.0, 1.0);
  EXPECT_EQ(gauss.kernelSize(), 5);
  EXPECT_NEAR(gauss.range(), 3.0, 1e-9);
  EXPECT_NEAR(gauss.scale(), 1.0, 1e-9);

  // Changing parameters should not throw and should update
  gauss.kernelSize(6);
  EXPECT_EQ(gauss.kernelSize(), 7);
  gauss.range(2.0);
  EXPECT_NEAR(gauss.range(), 2.0, 1e-9);
  gauss.scale(2.0);
  EXPECT_NEAR(gauss.scale(), 2.0, 1e-9);
}

TEST(DerivativeTest, GaussianProcessOnStep) {
  GaussianDerivative<uchar, float> gauss(5, 3.0, 1.0);
  cv::Mat gx, gy;
  gauss.process(makeStep(), gx, gy);
  ASSERT_EQ(gx.type(), CV_32F);
  ASSERT_EQ(gy.type(), CV_32F);
  int stepCol3 = gx.cols / 2;
  cv::Mat m3 = cv::abs(gx.col(stepCol3)) + cv::abs(gy.col(stepCol3)) + cv::abs(gx.col(stepCol3 - 1)) +
               cv::abs(gy.col(stepCol3 - 1));
  cv::Scalar s3 = cv::sum(m3);
  EXPECT_GT(s3[0], 0.0);
}
