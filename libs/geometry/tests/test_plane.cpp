//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_plane.cpp
/// @brief Unit tests for plane geometry.

#include <geometry/plane.hpp>

#include <gtest/gtest.h>

using lsfm::Vec3f;
using Planef = lsfm::Plane<float>;
using Line3f = lsfm::Line3<float>;

TEST(PlaneTest, FromThreePointsAndDistance) {
  // XY plane z=0
  Planef p(Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 1, 0));
  ASSERT_TRUE(p.valid());
  // Normal points along +Z
  EXPECT_NEAR(p.normal().x(), 0.f, 1e-6f);
  EXPECT_NEAR(p.normal().y(), 0.f, 1e-6f);
  EXPECT_NEAR(p.normal().z(), 1.f, 1e-6f);
  // Distance of point (0,0,2) is 2
  EXPECT_NEAR(p.distance(Vec3f(0, 0, 2)), 2.f, 1e-6f);
  // Nearest projection returns z=0
  auto proj = p.nearestPointOnPlane(Vec3f(5, -3, 7));
  EXPECT_NEAR(proj.z(), 0.f, 1e-6f);
}

TEST(PlaneTest, LineIntersection) {
  Planef p(Vec3f(0, 0, 0), Vec3f(0, 0, 1));  // z=0
  Line3f l(Vec3f(0, 0, -1), Vec3f(0, 0, 1));
  Vec3f ip;
  ASSERT_TRUE(p.intersection(l, ip));
  EXPECT_NEAR(ip.x(), 0.f, 1e-6f);
  EXPECT_NEAR(ip.y(), 0.f, 1e-6f);
  EXPECT_NEAR(ip.z(), 0.f, 1e-6f);
}

TEST(PlaneTest, PlaneIntersection) {
  // z=0 and y=0 intersect in x-axis direction
  Planef p1(Vec3f(0, 0, 0), Vec3f(0, 0, 1));
  Planef p2(Vec3f(0, 0, 0), Vec3f(0, 1, 0));
  Line3f line;
  ASSERT_TRUE(p1.intersection(p2, line));
  // Direction should align with x-axis (or negative)
  EXPECT_NEAR(std::fabs(line.direction().x()), 1.f, 1e-6f);
  EXPECT_NEAR(line.direction().y(), 0.f, 1e-6f);
  EXPECT_NEAR(line.direction().z(), 0.f, 1e-6f);
}
