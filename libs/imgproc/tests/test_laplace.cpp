#include <imgproc/laplace.hpp>

#include <gtest/gtest.h>

static cv::Mat makeDot(int rows = 5, int cols = 5) {
  cv::Mat img = cv::Mat::zeros(rows, cols, CV_8U);
  img.at<uchar>(rows / 2, cols / 2) = 255;
  return img;
}

TEST(LaplaceTest, LaplaceSimple) {
  lsfm::LaplaceSimple<uchar, short> lap;
  auto img = makeDot();
  lap.process(img);
  cv::Mat L = lap.laplace();
  ASSERT_EQ(L.type(), CV_16S);
  // center should be negative large, neighbors positive
  short c = L.at<short>(2, 2);
  EXPECT_LT(c, 0);
  EXPECT_GT(L.at<short>(2, 1), 0);
  auto r = lap.laplaceRange();
  EXPECT_GT(r.upper, 0);
  EXPECT_LT(r.lower, 0);
}

TEST(LaplaceTest, LaplaceCV) {
  lsfm::LaplaceCV<uchar, short> lap(3);
  auto img = makeDot();
  lap.process(img);
  auto L = lap.laplace();
  ASSERT_EQ(L.type(), CV_16S);
  auto r = lap.laplaceRange();
  EXPECT_GT(r.upper, 0);
  EXPECT_LT(r.lower, 0);
}

TEST(LaplaceTest, LoGKernelAndRange) {
  lsfm::LoG<uchar, float> log(5, 1.0, 1.0);
  cv::Mat k = log.kernel();
  ASSERT_EQ(k.rows, 5);
  ASSERT_EQ(k.cols, 5);
  // Kernel should sum to ~0 after DC zero fix
  double s = cv::sum(k)[0];
  EXPECT_NEAR(s, 0.0, 1e-4);
  auto r = log.laplaceRange();
  EXPECT_LT(r.lower, 0.0f);
  EXPECT_GT(r.upper, 0.0f);
}
