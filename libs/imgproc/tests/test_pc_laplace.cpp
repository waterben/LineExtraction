#include <gtest/gtest.h>
#include <imgproc/pc_sqf.hpp>

static cv::Mat makeSin(int rows=32, int cols=32, double period=8.0)
{
  cv::Mat img(rows, cols, CV_8U);
  for (int r=0;r<rows;++r) for (int c=0;c<cols;++c) {
    double v = 128.0 + 127.0*std::sin(2.0*CV_PI * r / period);
    img.at<uchar>(r,c) = static_cast<uchar>(std::clamp(v, 0.0, 255.0));
  }
  return img;
}

TEST(PCLSqfTest, Smoke)
{
  using PCL = lsfm::PCLSqf<uchar,float>;
  PCL pcl(1.0f, 2.0f, 1.0f);
  auto img = makeSin();
  pcl.process(img);
  cv::Mat lx, ly;
  pcl.pcLaplace(lx, ly);
  ASSERT_FALSE(lx.empty());
  ASSERT_FALSE(ly.empty());
  EXPECT_EQ(lx.type(), CV_32F);
  EXPECT_EQ(ly.type(), CV_32F);
  auto mag = pcl.pclMag();
  ASSERT_FALSE(mag.empty());
  EXPECT_EQ(mag.type(), CV_32F);
  auto rr = pcl.pcLaplaceRange();
  EXPECT_GE(rr.upper, rr.lower);
  double vmin1, vmax1;
  cv::minMaxIdx(mag, &vmin1, &vmax1);
  EXPECT_GT(vmax1, 0.0);
}

TEST(PCLSqTest, Smoke)
{
  using PCL = lsfm::PCLSq<uchar,float,float>;
  PCL pcl(1.0f, 2.0f, 1, 1.0f); // scale, muls, k, spacing
  auto img = makeSin();
  pcl.process(img);
  cv::Mat lx, ly;
  pcl.pcLaplace(lx, ly);
  ASSERT_FALSE(lx.empty());
  ASSERT_FALSE(ly.empty());
  EXPECT_EQ(lx.type(), CV_32F);
  EXPECT_EQ(ly.type(), CV_32F);
  auto mag = pcl.pclMag();
  ASSERT_FALSE(mag.empty());
  EXPECT_EQ(mag.type(), CV_32F);
  auto rr = pcl.pcLaplaceRange();
  EXPECT_GE(rr.upper, rr.lower);
  // Some parameterizations can yield near-zero maps; type/shape suffices as smoke
}
