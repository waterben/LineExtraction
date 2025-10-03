#include <imgproc/gaussian.hpp>

#include <gtest/gtest.h>

using lsfm::gaussian;
using lsfm::gaussianD1;
using lsfm::gaussianD2;

TEST(GaussianKernelsTest, ShapesAndCenter) {
  auto g = gaussian<float>(5, 3.0f);
  ASSERT_EQ(g.rows, 1);
  ASSERT_EQ(g.cols, 5);
  // center is exp(0)=1
  EXPECT_NEAR(g(0, 2), 1.0f, 1e-6f);

  auto d1 = gaussianD1<float>(5, 3.0f);
  ASSERT_EQ(d1.cols, 5);
  // center of first derivative is 0
  EXPECT_NEAR(d1(0, 2), 0.0f, 1e-6f);
  // antisymmetric: left = -right
  EXPECT_NEAR(d1(0, 1), -d1(0, 3), 1e-6f);

  auto d2 = gaussianD2<float>(5, 3.0f);
  ASSERT_EQ(d2.cols, 5);
  // center negative
  EXPECT_LT(d2(0, 2), 0.0f);
  // symmetric ends
  EXPECT_NEAR(d2(0, 0), d2(0, 4), 1e-6f);
}

TEST(GaussianKernelsTest, SizeCorrection) {
  // even size request should get corrected to next odd
  auto g = gaussian<double>(4, 2.0);
  EXPECT_EQ(g.cols, 5);
  auto d1 = gaussianD1<double>(2, 2.0);
  EXPECT_EQ(d1.cols, 3);
}
