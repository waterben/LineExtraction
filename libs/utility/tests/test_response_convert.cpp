//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_response_convert.cpp
/// @brief Unit tests for response conversion utilities.

#include <utility/response_convert.hpp>

#include <gtest/gtest.h>

using lsfm::convertLaplace;
using lsfm::convertMag;
using lsfm::Range;

TEST(ResponseConvertTest, ConvertMagBasic) {
  cv::Mat m = (cv::Mat_<float>(2, 2) << 0.f, 1.f, 2.f, 0.5f);
  // using explicit vmin/vmax so test is stable
  cv::Mat u8 = convertMag(m, 0.0, 2.0);
  ASSERT_EQ(u8.type(), CV_8U);
  EXPECT_EQ(u8.at<uchar>(0, 0), 0);
  EXPECT_EQ(u8.at<uchar>(0, 1), 128);
  EXPECT_EQ(u8.at<uchar>(1, 0), 255);
}

TEST(ResponseConvertTest, ConvertLaplaceAbs) {
  cv::Mat m = (cv::Mat_<float>(1, 3) << -2.f, 0.f, 2.f);
  cv::Mat u8 = convertLaplace(m, Range<double>(-2.0, 2.0));
  EXPECT_EQ(u8.at<uchar>(0, 0), 255);
  EXPECT_EQ(u8.at<uchar>(0, 1), 0);
  EXPECT_EQ(u8.at<uchar>(0, 2), 255);
}
