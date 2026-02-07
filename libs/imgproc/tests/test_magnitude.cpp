//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_magnitude.cpp
/// @brief Unit tests for magnitude computations.

#include <imgproc/derivative.hpp>
#include <imgproc/magnitude.hpp>

#include <gtest/gtest.h>

using lsfm::AbsoluteMagnitude;
using lsfm::Magnitude;
using lsfm::QuadraticMagnitude;

TEST(MagnitudeTest, QuadraticScalar) {
  using QMag = QuadraticMagnitude<short, int>;
  EXPECT_EQ((QMag::single(3)), 9);
  EXPECT_EQ((QMag::process(3, 4)), 25);
}

TEST(MagnitudeTest, QuadraticMat) {
  cv::Mat gx = (cv::Mat_<short>(1, 3) << 3, 0, -1);
  cv::Mat gy = (cv::Mat_<short>(1, 3) << 4, 5, 2);
  cv::Mat q;
  QuadraticMagnitude<short, int>::process(gx, gy, q);
  ASSERT_EQ(q.type(), CV_32S);
  EXPECT_EQ(q.at<int>(0, 0), 25);
  EXPECT_EQ(q.at<int>(0, 1), 25);
  EXPECT_EQ(q.at<int>(0, 2), 5);
}

TEST(MagnitudeTest, L2Scalar) {
  using L2 = Magnitude<short, float>;
  EXPECT_FLOAT_EQ((L2::single(3)), 3.0f);
  EXPECT_FLOAT_EQ((L2::process(3, 4)), 5.0f);
}

TEST(MagnitudeTest, L2Mat) {
  cv::Mat gx = (cv::Mat_<short>(1, 2) << 3, 0);
  cv::Mat gy = (cv::Mat_<short>(1, 2) << 4, 5);
  cv::Mat m;
  Magnitude<short, float>::process(gx, gy, m);
  ASSERT_EQ(m.type(), CV_32F);
  EXPECT_NEAR((m.at<float>(0, 0)), 5.0f, 1e-6f);
  EXPECT_NEAR((m.at<float>(0, 1)), 5.0f, 1e-6f);
}

TEST(MagnitudeTest, L1Scalar) {
  using L1 = AbsoluteMagnitude<short, int>;
  EXPECT_EQ((L1::single(-3)), 3);
  EXPECT_EQ((L1::process(-3, 4)), 7);
}

TEST(MagnitudeTest, L1Mat) {
  cv::Mat gx = (cv::Mat_<short>(1, 3) << -3, 1, -2);
  cv::Mat gy = (cv::Mat_<short>(1, 3) << 4, -5, 2);
  cv::Mat a;
  AbsoluteMagnitude<short, int>::process(gx, gy, a);
  ASSERT_EQ(a.type(), CV_16S) << "cv::add(cv::abs(), cv::abs()) keeps input depth";
  EXPECT_EQ(a.at<short>(0, 0), 7);
  EXPECT_EQ(a.at<short>(0, 1), 6);
  EXPECT_EQ(a.at<short>(0, 2), 4);
}
