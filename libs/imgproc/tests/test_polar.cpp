#include <imgproc/polar.hpp>

#include <gtest/gtest.h>

using lsfm::Polar;
using lsfm::PolarCV;

TEST(PolarTest, PhaseAndMagnitude) {
  cv::Mat dx = (cv::Mat_<short>(1, 2) << 1, 0);
  cv::Mat dy = (cv::Mat_<short>(1, 2) << 0, 1);
  cv::Mat phase, mag;
  Polar<short, float>::phase(dx, dy, phase);
  Polar<short, float>::magnitude(dx, dy, mag);
  ASSERT_EQ(phase.type(), CV_32F);
  ASSERT_EQ(mag.type(), CV_32F);
  EXPECT_NEAR(phase.at<float>(0, 0), 0.0f, 1e-6f);
  EXPECT_NEAR(phase.at<float>(0, 1), static_cast<float>(CV_PI / 2), 1e-6f);
  EXPECT_NEAR(mag.at<float>(0, 0), 1.0f, 1e-6f);
  EXPECT_NEAR(mag.at<float>(0, 1), 1.0f, 1e-6f);
}

TEST(PolarTest, Cart2PolarAndBack) {
  cv::Mat dx = (cv::Mat_<short>(1, 2) << 3, 0);
  cv::Mat dy = (cv::Mat_<short>(1, 2) << 4, -5);
  cv::Mat phase, mag, rdx, rdy;
  Polar<short, float>::cart2Polar(dx, dy, mag, phase);
  Polar<short, float>::polar2Cart(mag, phase, rdx, rdy);
  ASSERT_EQ(rdx.type(), CV_16S);
  ASSERT_EQ(rdy.type(), CV_16S);
  EXPECT_NEAR(static_cast<float>(rdx.at<short>(0, 0)), 3.0f, 1e-2f);
  EXPECT_NEAR(static_cast<float>(rdy.at<short>(0, 0)), 4.0f, 1e-2f);
}

TEST(PolarTest, Wrap) {
  cv::Mat ang = (cv::Mat_<float>(1, 2) << static_cast<float>(CV_PI * 1.5), static_cast<float>(CV_PI / 2));
  cv::Mat wrapped;
  PolarCV<short, float>::wrap(ang, wrapped);
  // angle > pi should be wrapped into (-pi, pi]
  EXPECT_LT(wrapped.at<float>(0, 0), static_cast<float>(CV_PI));
  EXPECT_NEAR(wrapped.at<float>(0, 1), static_cast<float>(CV_PI / 2), 1e-6f);
}
