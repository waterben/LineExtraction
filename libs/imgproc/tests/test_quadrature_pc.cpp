//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_quadrature_pc.cpp
/// @brief Unit tests for quadrature phase congruency.

#include <imgproc/gradient_adapter.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/quadratureG2.hpp>

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

TEST(QuadratureG2Test, Smoke) {
  using Quad = lsfm::QuadratureG2<uchar, float>;
  Quad q(9, 0.782f);
  auto img = makeSinus();
  q.process(img);

  cv::Mat e = q.even();
  cv::Mat o = q.odd();
  cv::Mat en = q.energy();
  cv::Mat ph = q.phase();
  cv::Mat dir = q.direction();
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

  // verify helpers
  cv::Mat e2, o2, en2, ph2;
  q.evenOdd(e2, o2);
  q.energyPhase(en2, ph2);
  ASSERT_FALSE(e2.empty());
  ASSERT_FALSE(o2.empty());
  ASSERT_FALSE(en2.empty());
  ASSERT_FALSE(ph2.empty());

  // norm type should be L2
  EXPECT_EQ(q.normType(), lsfm::NormType::NORM_L2);
}

TEST(GradientAdapterTest, OddAndEnergyAdapters) {
  using Quad = lsfm::QuadratureG2<uchar, float>;
  lsfm::GradientOdd<Quad> g_odd;
  lsfm::GradientEnergy<Quad> g_en;
  auto img = makeSinus();
  g_odd.process(img);
  g_en.process(img);
  auto m1 = g_odd.magnitude();
  auto m2 = g_en.magnitude();
  ASSERT_FALSE(m1.empty());
  ASSERT_FALSE(m2.empty());
  EXPECT_EQ(m1.type(), CV_32F);
  EXPECT_EQ(m2.type(), CV_32F);
}

TEST(GradientAdapterTest, PhaseCongruencyAdapter) {
  using PC = lsfm::PCSqf<uchar, float>;
  lsfm::GradientPC<PC> g_pc;
  auto img = makeSinus();
  g_pc.process(img);
  auto m = g_pc.magnitude();
  ASSERT_FALSE(m.empty());
  EXPECT_EQ(m.type(), CV_32F);
  auto mr = g_pc.magnitudeRange();
  EXPECT_GE(mr.upper, mr.lower);
}

TEST(PhaseCongruencyTest, PCSqfSmoke) {
  using PC = lsfm::PCSqf<uchar, float>;
  PC pc(1.0f, 2.0f, 1.0f, 2);  // fewer scales to be fast
  auto img = makeSinus();
  pc.process(img);
  auto pcmap = pc.phaseCongruency();
  ASSERT_FALSE(pcmap.empty());
  EXPECT_EQ(pcmap.type(), CV_32F);
  double minv, maxv;
  cv::minMaxIdx(pcmap, &minv, &maxv);
  EXPECT_GE(minv, 0.0);
  EXPECT_LE(maxv, 1.0);
}
