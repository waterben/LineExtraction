#include <imgproc/derivative.hpp>
#include <imgproc/direction.hpp>

#include <gtest/gtest.h>

using lsfm::Direction;
using lsfm::FastDirection;

TEST(DirectionTest, ScalarRadians) {
  using Dir = Direction<short, float>;
  EXPECT_NEAR((Dir::process(1, 0)), 0.0f, 1e-6f);
  EXPECT_NEAR((Dir::process(0, 1)), static_cast<float>(CV_PI / 2), 1e-6f);
  EXPECT_NEAR((Dir::process(-1, 0)), static_cast<float>(CV_PI), 1e-6f);
  EXPECT_NEAR((Dir::process(0, -1)), static_cast<float>(-CV_PI / 2), 1e-6f);
}

TEST(DirectionTest, MatRadians) {
  cv::Mat gx = (cv::Mat_<short>(1, 2) << 1, 0);
  cv::Mat gy = (cv::Mat_<short>(1, 2) << 0, 1);
  cv::Mat d;
  Direction<short, float>::process(gx, gy, d);
  ASSERT_EQ(d.type(), CV_32F);
  EXPECT_NEAR(d.at<float>(0, 0), 0.0f, 1e-6f);
  EXPECT_NEAR(d.at<float>(0, 1), static_cast<float>(CV_PI / 2), 1e-6f);
}

TEST(DirectionTest, FastDegrees) {
  using FDir = FastDirection<short, float>;
  EXPECT_NEAR((FDir::process(1, 0)), 0.0f, 1e-3f);
  EXPECT_NEAR((FDir::process(0, 1)), 90.0f, 1e-3f);
  EXPECT_NEAR((FDir::process(-1, 0)), 180.0f, 1e-3f);
  EXPECT_NEAR((FDir::process(0, -1)), 270.0f, 1e-3f);
}
