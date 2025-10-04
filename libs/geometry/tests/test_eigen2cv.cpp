// eigen2cv adapter tests: euler, quaternion, homogeneous, matxMul
#include <geometry/eigen2cv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <gtest/gtest.h>

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

TEST(Eigen2CvTest, EulerRoundTrip) {
  cv::Vec3f e(0.3f, -0.4f, 1.2f);
  cv::Matx33f R = cv::euler(e);
  cv::Vec3f e2 = cv::euler(R);
  EXPECT_NEAR(e2[0], e[0], 1e-4f);
  EXPECT_NEAR(e2[1], e[1], 1e-4f);
  EXPECT_NEAR(e2[2], e[2], 1e-4f);
}

TEST(Eigen2CvTest, QuaternionRoundTrip) {
  cv::Matx33f R = cv::euler(cv::Vec3f(0.1f, 0.2f, -0.3f));
  cv::Vec4f q = cv::quaternion(R);
  cv::Matx33f R2 = cv::quaternion(q);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) EXPECT_NEAR(R(i, j), R2(i, j), 1e-5f);
}

TEST(Eigen2CvTest, Homogeneous2DRoundTripMat) {
  std::vector<cv::Point2f> pts = {{1.f, 2.f}, {3.f, 4.f}, {-5.f, 6.f}};
  cv::Mat matPts(pts);
  cv::Mat matH, matOut;
  cv::toHomogeneous<float>(matPts, matH);
  cv::fromHomogeneous<float>(matH, matOut);
  std::vector<cv::Point2f> pts2;
  matOut.copyTo(pts2);
  ASSERT_EQ(pts2.size(), pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    EXPECT_NEAR(pts2[i].x, pts[i].x, 1e-5f);
    EXPECT_NEAR(pts2[i].y, pts[i].y, 1e-5f);
  }
}

TEST(Eigen2CvTest, MatxMul2DSingle) {
  float c = std::cos(kPi / 2.f), s = std::sin(kPi / 2.f);
  cv::Matx33f H(c, -s, 0, s, c, 0, 0, 0, 1);
  cv::Vec2f p(1.f, 0.f);
  auto p2 = cv::matxMul(H, p);
  EXPECT_NEAR(p2[0], 0.f, 1e-5f);
  EXPECT_NEAR(p2[1], 1.f, 1e-5f);
}
