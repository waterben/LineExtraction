//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_quadrature_variants.cpp
/// @brief Unit tests for quadrature filter variants.

#include <imgproc/quadratureLGF.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>

#include <gtest/gtest.h>

static cv::Mat makeSinus(int rows = 32, int cols = 32, double period = 8.0) {
  cv::Mat img(rows, cols, CV_8U);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      double v = 128.0 + 127.0 * std::sin(2.0 * CV_PI * c / period);
      img.at<uchar>(r, c) = static_cast<uchar>(std::clamp(v, 0.0, 255.0));
    }
  }
  return img;
}

template <typename Q>
static void quadSmoke() {
  Q q;
  auto img = makeSinus();
  q.process(img);
  auto e = q.even();
  auto o = q.odd();
  auto en = q.energy();
  auto ph = q.phase();
  auto dir = q.direction();
  ASSERT_FALSE(e.empty());
  ASSERT_FALSE(o.empty());
  ASSERT_FALSE(en.empty());
  ASSERT_FALSE(ph.empty());
  ASSERT_FALSE(dir.empty());
  EXPECT_EQ(e.type(), CV_32F);
  EXPECT_EQ(o.type(), CV_32F);
  EXPECT_EQ(en.type(), CV_32F);
  EXPECT_EQ(ph.type(), CV_32F);
  EXPECT_EQ(dir.type(), CV_32F);
  auto er = q.evenRange();
  auto orr = q.oddRange();
  auto enr = q.energyRange();
  EXPECT_GT(er.upper, 0.0f);
  EXPECT_GT(orr.upper, 0.0f);
  EXPECT_GT(enr.upper, 0.0f);
}

TEST(QuadratureVariantsTest, QuadratureS) { quadSmoke<lsfm::QuadratureS<uchar, float, float>>(); }

TEST(QuadratureVariantsTest, QuadratureSF) { quadSmoke<lsfm::QuadratureSF<uchar, float>>(); }

TEST(QuadratureVariantsTest, QuadratureLGF) { quadSmoke<lsfm::QuadratureLGF<uchar, float>>(); }
