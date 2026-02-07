//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_line.cpp
/// @brief Unit tests for 2D line geometry.

#include <geometry/line.hpp>

#include <gtest/gtest.h>

#include <cmath>
namespace {
constexpr float kPi = 3.14159265358979323846f;
}

using LineF = lsfm::Line<float>;
using SegF = lsfm::LineSegment<float>;
using P2f = lsfm::Vec2<float>;

static inline P2f P(float x, float y) { return P2f(x, y); }

TEST(LineTest, ConstructFromPointsAndBasics) {
  // Horizontal segment from (0,0) to (10,0) -> normal points up (0,1), distance 0
  LineF l(P(0, 0), P(10, 0));
  ASSERT_TRUE(l.valid());
  EXPECT_NEAR(l.normalX(), 0.f, 1e-6f);
  EXPECT_NEAR(l.normalY(), 1.f, 1e-6f);
  EXPECT_NEAR(l.originDist(), 0.f, 1e-6f);

  // Angle of line (direction along +x) should be 0 rad; gradient angle pi/2 (downwards normal gives -pi/2)
  EXPECT_NEAR(l.angle(), 0.f, 1e-6f);
  EXPECT_NEAR(l.normalAngle(), kPi / 2.f, 1e-5f);
}

TEST(LineTest, DistanceAndProjection) {
  // Vertical line x = 2 -> normal (1,0), d = 2
  LineF l(1.f, 0.f, 2.f);
  // Point (5,3): distance is x - 2
  EXPECT_NEAR(l.distance(5.f, 3.f), 3.f, 1e-6f);
  // Line projection (along direction) of (5,3) should be ny*x - nx*y = 0*5 - 1*3 = -3
  EXPECT_NEAR(l.project(5.f, 3.f), -3.f, 1e-6f);
}

TEST(LineTest, Intersection) {
  // x=2 and y=3 lines intersect at (2,3)
  LineF lx(1.f, 0.f, 2.f);
  LineF ly(0.f, 1.f, 3.f);
  float x, y;
  ASSERT_TRUE(lx.intersection(ly, x, y));
  EXPECT_NEAR(x, 2.f, 1e-6f);
  EXPECT_NEAR(y, 3.f, 1e-6f);
}

TEST(LineTest, TranslateRotateScale) {
  // Start with y=0
  LineF l(P(0, 0), P(10, 0));
  // Translate up to y=2
  l.translateTo(0.f, 2.f);
  EXPECT_NEAR(l.originDist(), 2.f * l.normalY() + 0.f * l.normalX(), 1e-6f);

  // Rotate by 90 degrees around origin point
  auto beforeNormal = l.normal();
  l.rotate(kPi / 2.f);
  // Normal should rotate by +90 degrees
  EXPECT_NEAR(l.normalX(), -beforeNormal.y(), 1e-5f);
  EXPECT_NEAR(l.normalY(), beforeNormal.x(), 1e-5f);

  // Scaling at origin scales the distance
  float d0 = l.originDist();
  l.scale(2.f);
  EXPECT_NEAR(l.originDist(), 2.f * d0, 1e-5f);
}

TEST(LineSegmentTest, BasicsAndLength) {
  SegF s(P(1.f, 0.f), P(6.f, 0.f));
  EXPECT_NEAR(s.length(), 5.f, 1e-6f);
  auto sp = s.startPoint();
  auto ep = s.endPoint();
  EXPECT_NEAR(sp[0], 1.f, 1e-6f);
  EXPECT_NEAR(ep[0], 6.f, 1e-6f);
}

TEST(LineSegmentTest, InRangeAndTrim) {
  // Line y=2 from x=-10..10, then trim to box [0,0]-[5,5]
  LineF l(0.f, 1.f, 2.f);  // y = 2
  SegF s(P(-10.f, 2.f), P(10.f, 2.f));
  // In-range checks
  EXPECT_TRUE(s.inRange(0.f, 2.f));
  EXPECT_FALSE(s.inRange(-100.f, 2.f));

  auto trimmed = lsfm::trim2Box(l, 5.f, 5.f, 0.f, 0.f);
  // Should lie from (0,2) to (5,2)
  auto sp = trimmed.startPoint();
  auto ep = trimmed.endPoint();
  EXPECT_NEAR(sp[0], 0.f, 1e-6f);
  EXPECT_NEAR(sp[1], 2.f, 1e-6f);
  EXPECT_NEAR(ep[0], 5.f, 1e-6f);
  EXPECT_NEAR(ep[1], 2.f, 1e-6f);
}

TEST(LineSegmentTest, OverlapCheck) {
  SegF a(P(0.f, 0.f), P(10.f, 0.f));
  SegF b(P(5.f, 0.f), P(15.f, 0.f));
  EXPECT_TRUE(a.checkOverlap(b));

  SegF c(P(11.f, 0.f), P(20.f, 0.f));
  EXPECT_FALSE(a.checkOverlap(c));
}
