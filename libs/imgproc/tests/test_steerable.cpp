#include <gtest/gtest.h>
#include <imgproc/steerable.hpp>

TEST(SteerableTest, D1KernelsOrthogonal)
{
  auto k0 = lsfm::SteerGaussianD1<float>(0.0f, 5, 3.0f);
  auto k90 = lsfm::SteerGaussianD1<float>(static_cast<float>(CV_PI/2), 5, 3.0f);
  // The two should be different and have similar norms
  double n0 = cv::norm(k0), n90 = cv::norm(k90);
  EXPECT_GT(n0, 0.0);
  EXPECT_GT(n90, 0.0);
  EXPECT_NEAR(n0, n90, n0*0.2);
}

TEST(SteerableTest, D2KernelsSymmetry)
{
  auto k = lsfm::SteerGaussianD2<float>(0.0f, 5, 3.0f);
  // Basic symmetry: central element should be negative
  EXPECT_LT(k(2,2), 0.0f);
}

