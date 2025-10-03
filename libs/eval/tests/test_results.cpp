#include <eval/results.hpp>

#include <gtest/gtest.h>

using lsfm::applyBorderCopy;
using lsfm::createNMS;

TEST(ResultsTest, ApplyBorderCopyPositive) {
  cv::Mat m = cv::Mat::ones(2, 2, CV_8U) * 9;
  cv::Mat out = applyBorderCopy(m, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
  EXPECT_EQ(out.rows, 4);
  EXPECT_EQ(out.cols, 4);
  // inner remains 9
  EXPECT_EQ(out.at<uchar>(1, 1), 9);
  // border is 0
  EXPECT_EQ(out.at<uchar>(0, 0), 0);
}

TEST(ResultsTest, ApplyBorderCopyNegative) {
  cv::Mat m = cv::Mat::ones(3, 3, CV_8U);
  cv::Mat out = applyBorderCopy(m, -1, cv::BORDER_CONSTANT, cv::Scalar(5));
  ASSERT_EQ(out.size(), m.size());
  // corners colored
  EXPECT_EQ(out.at<uchar>(0, 0), 5);
  EXPECT_EQ(out.at<uchar>(0, 2), 5);
  EXPECT_EQ(out.at<uchar>(2, 0), 5);
  EXPECT_EQ(out.at<uchar>(2, 2), 5);
  // center preserved
  EXPECT_EQ(out.at<uchar>(1, 1), 1);
}

TEST(ResultsTest, CreateNMSColors) {
  cv::Mat emap = (cv::Mat_<uchar>(1, 8) << 0, 1, 2, 3, 4, 5, 6, 7);
  cv::Mat img = createNMS(emap);
  ASSERT_EQ(img.type(), CV_8UC3);
  // check a couple of known mappings (see results.cpp)
  EXPECT_EQ(img.at<cv::Vec3b>(0, 0), (cv::Vec3b{0, 0, 255}));    // 0 -> red
  EXPECT_EQ(img.at<cv::Vec3b>(0, 4), (cv::Vec3b{255, 255, 0}));  // 4 -> cyan
}
