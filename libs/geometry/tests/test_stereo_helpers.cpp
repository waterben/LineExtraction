// Stereo helpers tests: pixel/line conversions and ray
#include <gtest/gtest.h>

#include <geometry/stereo.hpp>

using lsfm::Vec2f;
using lsfm::Vec3f;
using lsfm::Matx33f;
using lsfm::Line;
using LineF = lsfm::Line<float>;

TEST(StereoHelpersTest, PixelToCam3DPoint)
{
  Vec2f focal(100.f, 100.f);
  Vec2f offset(50.f, 60.f);
  Vec2f p(55.f, 65.f);
  auto P = lsfm::pixel2Cam3dPoint(focal, offset, p);
  EXPECT_NEAR(P.x(), 5.f, 1e-6f);
  EXPECT_NEAR(P.y(), 5.f, 1e-6f);
  EXPECT_NEAR(P.z(), 100.f, 1e-6f);
}

TEST(StereoHelpersTest, LineToCam3DLine)
{
  // 2D line y = 10 -> normal (0,1), d = 10
  LineF l(0.f, 1.f, 10.f);
  Vec2f focal(100.f, 100.f), offset(50.f, 50.f);
  auto L3 = lsfm::line2Cam3dLine(focal, offset, l);
  // Direction should be along x axis and z=0
  EXPECT_NEAR(L3.direction().x(), 1.f, 1e-6f);
  EXPECT_NEAR(L3.direction().y(), 0.f, 1e-6f);
  EXPECT_NEAR(L3.direction().z(), 0.f, 1e-6f);
}

TEST(StereoHelpersTest, LineFromPixelRay)
{
  Vec2f focal(120.f, 120.f), offset(40.f, 30.f);
  Matx33f I = Matx33f::Identity();
  Vec3f t(1.f, 2.f, 3.f);
  Vec2f p(45.f, 31.f);
  auto ray = lsfm::lineFromPixel(focal, offset, I, t, p);
  // Origin at camera translation; direction equals rotated pixel2Cam3dPoint
  EXPECT_NEAR(ray.origin().x(), t.x(), 1e-6f);
  auto d = lsfm::pixel2Cam3dPoint(focal, offset, p);
  EXPECT_NEAR(ray.direction().x(), d.x()/d.norm(), 1e-6f);
}

