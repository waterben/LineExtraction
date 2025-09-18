#include <gtest/gtest.h>
#include <imgproc/derivative_gradient.hpp>

// Use float for magnitude type (MT) so cv::sqrt/cv::pow paths accept depth
using Grad = lsfm::DerivativeGradient<uchar, short, float, float>;

static cv::Mat makeImg()
{
  return (cv::Mat_<uchar>(2,3) << 0, 50, 100,
                                  150, 200, 250);
}

TEST(GradientTest, ProcessAndAccessors)
{
  Grad g;
  cv::Mat gx, gy, mag, dir;
  g.process(makeImg(), gx, gy, mag, dir);

  ASSERT_FALSE(gx.empty());
  ASSERT_FALSE(gy.empty());
  ASSERT_FALSE(mag.empty());
  ASSERT_FALSE(dir.empty());

  // Norm type provided by Magnitude (L2)
  EXPECT_EQ(g.normType(), lsfm::NormType::NORM_L2);

  auto mr = g.magnitudeRange();
  EXPECT_GE(mr.upper, mr.lower);

  auto gr = g.gradientRange();
  EXPECT_GE(gr.upper, -gr.lower);

  // Threshold mapping is monotonic and in range
  auto th = g.magnitudeThreshold(0.5);
  EXPECT_GT(th, 0);
  EXPECT_LE(th, mr.upper);
}
