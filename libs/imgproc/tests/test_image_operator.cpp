#include <gtest/gtest.h>
#include <imgproc/image_operator.hpp>

static cv::Mat makeGray(int rows=16, int cols=16)
{
  cv::Mat img(rows, cols, CV_8U);
  for (int r=0;r<rows;++r)
    for (int c=0;c<cols;++c)
      img.at<uchar>(r,c) = static_cast<uchar>((r*cols + c) % 255);
  return img;
}

TEST(ImageOperatorTest, GeometryOps)
{
  auto img = makeGray();
  auto rot = lsfm::RotateOperator<double>::create(CV_PI/4);
  auto scl = lsfm::ScaleOperator<double>::create(0.5);
  auto trs = lsfm::TranslateOperator<double>::create({2.0, 3.0});
  auto aff = lsfm::AffineOperator<double>::create(cv::Matx<double,2,3>(1,0,1, 0,1,1));
  auto per = lsfm::PerspectiveOperator<double>::create(cv::Matx<double,3,3>(1,0,0, 0,1,0, 0,0,1));

  EXPECT_NO_THROW((*rot)(img));
  EXPECT_NO_THROW((*scl)(img));
  EXPECT_NO_THROW((*trs)(img));
  EXPECT_NO_THROW((*aff)(img));
  EXPECT_NO_THROW((*per)(img));
}

TEST(ImageOperatorTest, BlurAndNoise)
{
  auto img = makeGray();
  auto nb = lsfm::NoOp::create();
  auto rs = lsfm::ResizeOperator::create(8, 8);
  auto bl = lsfm::BlurOperator::create(3);
  auto gbl = lsfm::GaussianBlurOperator::create(1.0, cv::Size(3,3));
  auto mbl = lsfm::MedianBlurOperator::create(3);
  auto bil = lsfm::BilateralOperator::create(5, 10, 10);
  auto nlm = lsfm::FastNlMeansOperator::create(1.0f, 3, 7);
  auto uno = lsfm::UniformNoiseOperator::create(-5.0, 5.0);
  auto gno = lsfm::GaussianNoiseOperator::create(5.0, 0.0);

  cv::Mat out;
  EXPECT_NO_THROW((*nb)(img, out));
  EXPECT_NO_THROW((*rs)(img, out));
  EXPECT_EQ(out.rows, 8);
  EXPECT_EQ(out.cols, 8);

  EXPECT_NO_THROW((*bl)(img));
  EXPECT_NO_THROW((*gbl)(img));
  // median requires odd kernel size; already set to 3
  EXPECT_NO_THROW((*mbl)(img));
  EXPECT_NO_THROW((*bil)(img));
  EXPECT_NO_THROW((*nlm)(img));
  EXPECT_NO_THROW((*uno)(img));
  EXPECT_NO_THROW((*gno)(img));
}

TEST(ImageOperatorTest, Pipeline)
{
  lsfm::PipelineOperator pipe;
  pipe.push(lsfm::BlurOperator::create(3));
  pipe.push(lsfm::GaussianBlurOperator::create(1.0, cv::Size(3,3)));
  auto img = makeGray();
  EXPECT_NO_THROW(pipe.apply(img));
}

