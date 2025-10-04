// 3D line tests
#include <geometry/line3.hpp>

#include <gtest/gtest.h>

using lsfm::Vec3f;
using Line3f = lsfm::Line3<float>;
using Seg3f = lsfm::LineSegment3<float>;

TEST(Line3Test, ConstructAndBasics) {
  Line3f l(Vec3f(0, 0, 0), Vec3f(1, 0, 0));
  ASSERT_TRUE(l.valid());
  EXPECT_NEAR(l.direction().x(), 1.f, 1e-6f);
  EXPECT_NEAR(l.direction().y(), 0.f, 1e-6f);
  EXPECT_NEAR(l.direction().z(), 0.f, 1e-6f);
}

TEST(Line3Test, DistancePointAndNearestPoint) {
  Line3f l(Vec3f(0, 0, 0), Vec3f(1, 0, 0));
  Vec3f p(0, 2, 0);
  EXPECT_NEAR(l.distance(p), 2.f, 1e-6f);
  auto np = l.nearestPointOnLine(p);
  EXPECT_NEAR(np.x(), 0.f, 1e-6f);
  EXPECT_NEAR(np.y(), 0.f, 1e-6f);
}

TEST(Line3Test, SkewLinesDistance) {
  Line3f l1(Vec3f(0, 0, 0), Vec3f(1, 0, 0));
  Line3f l2(Vec3f(0, 0, 1), Vec3f(0, 1, 0));
  // They are skew; shortest distance should be 1 (z offset)
  EXPECT_NEAR(l1.distance(l2), 1.f, 1e-6f);
}

TEST(LineSegment3Test, LengthAndFlip) {
  Seg3f s(Vec3f(0, 0, 0), Vec3f(1, 0, 0), 0.f, 5.f);
  EXPECT_NEAR(s.length(), 5.f, 1e-6f);
  auto sp = s.startPoint();
  auto ep = s.endPoint();
  EXPECT_NEAR(sp.x(), 0.f, 1e-6f);
  EXPECT_NEAR(ep.x(), 5.f, 1e-6f);
  s.flip();
  // After flip, direction reverses and endpoints swap accordingly
  EXPECT_NEAR(s.startPoint().x(), 5.f, 1e-6f);
  EXPECT_NEAR(s.endPoint().x(), 0.f, 1e-6f);
}
