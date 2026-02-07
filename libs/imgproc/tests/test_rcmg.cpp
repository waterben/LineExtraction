//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_rcmg.cpp
/// @brief Unit tests for RCMG filtering.

#include <imgproc/rcmg.hpp>

#include <gtest/gtest.h>

static cv::Mat makeColorStep(int rows = 12, int cols = 12) {
  cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
  // left half blue, right half red
  img.colRange(cols / 2, cols).setTo(cv::Scalar(0, 0, 255));
  img.colRange(0, cols / 2).setTo(cv::Scalar(255, 0, 0));
  return img;
}

TEST(RCMGTest, Smoke) {
  using RCMG = lsfm::RCMGradient<uchar, 3, short, int, float>;
  RCMG rcmg(3, 0);  // mask 3x3, no rejection
  auto img = makeColorStep();
  cv::Mat gx, gy, mag, dir;
  rcmg.process(img, gx, gy, mag, dir);
  ASSERT_EQ(gx.type(), CV_16S);
  ASSERT_EQ(gy.type(), CV_16S);
  ASSERT_EQ(mag.type(), CV_32S);
  ASSERT_EQ(dir.type(), CV_32F);

  // energy along the step should be non-zero
  // Convert to double to avoid overflow in sum computation
  int stepCol = mag.cols / 2;
  cv::Mat magDouble;
  mag.convertTo(magDouble, CV_64F);
  cv::Scalar s = cv::sum(cv::abs(magDouble.col(stepCol)) + cv::abs(magDouble.col(stepCol - 1)));
  EXPECT_GT(s[0], 0.0);

  auto gr = rcmg.gradientRange();
  EXPECT_GT(gr.upper, 0);
  auto mr = rcmg.magnitudeRange();
  EXPECT_GT(mr.upper, 0);
}
