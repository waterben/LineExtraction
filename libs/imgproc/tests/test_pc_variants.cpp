#include <gtest/gtest.h>
#include <imgproc/pc_lgf.hpp>
#include <imgproc/pc_matlab.hpp>

static cv::Mat makeSin2D(int rows=32, int cols=32, double period=8.0)
{
  cv::Mat img(rows, cols, CV_8U);
  for (int r=0;r<rows;++r) for (int c=0;c<cols;++c) {
    double v = 128.0 + 127.0*std::sin(2.0*CV_PI * (r+c) / (2*period));
    img.at<uchar>(r,c) = static_cast<uchar>(std::clamp(v, 0.0, 255.0));
  }
  return img;
}

TEST(PCVariantsTest, PCLgf)
{
  using PC = lsfm::PCLgf<uchar,float>;
  PC pc(2, 3.0f, 2.0f, 0.55f); // fewer Scales
  auto img = makeSin2D();
  pc.process(img);
  auto e = pc.even();
  auto o = pc.odd();
  auto en = pc.energy();
  ASSERT_FALSE(e.empty());
  ASSERT_FALSE(o.empty());
  ASSERT_FALSE(en.empty());
  EXPECT_EQ(e.type(), CV_32F);
  EXPECT_EQ(o.type(), CV_32F);
  EXPECT_EQ(en.type(), CV_32F);
  auto pcmap = pc.phaseCongruency();
  ASSERT_FALSE(pcmap.empty());
  double minv, maxv;
  cv::minMaxIdx(pcmap, &minv, &maxv);
  EXPECT_GE(minv, 0.0);
  EXPECT_LE(maxv, 1.0);
}

TEST(PCVariantsTest, PCMatlab)
{
  using PC = lsfm::PCMatlab<uchar>;
  PC pc(2, 3, 2.1, 0.55); // fewer Scales
  auto img = makeSin2D();
  pc.process(img);
  auto e = pc.even();
  auto o = pc.odd();
  auto en = pc.energy();
  ASSERT_FALSE(e.empty());
  ASSERT_FALSE(o.empty());
  ASSERT_FALSE(en.empty());
  EXPECT_EQ(e.type(), CV_64F);
  EXPECT_EQ(o.type(), CV_64F);
  EXPECT_EQ(en.type(), CV_64F);
  auto pcmap = pc.phaseCongruency();
  ASSERT_FALSE(pcmap.empty());
}

