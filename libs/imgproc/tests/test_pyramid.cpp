#include <imgproc/pyramid.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <gtest/gtest.h>

static cv::Mat makeGradient(int rows = 8, int cols = 8) {
  cv::Mat img(rows, cols, CV_32F);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) img.at<float>(r, c) = static_cast<float>(r * cols + c);
  return img;
}

TEST(PyramidTest, BuildUntilMinSize) {
  auto base = makeGradient(8, 8);
  lsfm::Pyramid<float> pyr(base);
  // default builds until area < 2
  ASSERT_GE(pyr.size(), 3u);
  EXPECT_EQ(pyr[0].rows, 8);
  EXPECT_EQ(pyr[1].rows, 4);
  EXPECT_EQ(pyr[2].rows, 2);
}

TEST(PyramidTest, AlgebraOps) {
  lsfm::Pyramid<float> a(makeGradient(8, 8));
  lsfm::Pyramid<float> b(makeGradient(8, 8));
  auto s = a + b;
  ASSERT_EQ(s.size(), a.size());
  EXPECT_NEAR(s[0].at<float>(0, 0), 0.0f + 0.0f, 1e-5f);
  auto m = a * 2.0;
  EXPECT_NEAR(m[0].at<float>(1, 1), a[0].at<float>(1, 1) * 2.0f, 1e-5f);
}
