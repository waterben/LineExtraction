// Polygon tests: edges, convexity, transforms
#include <geometry/polygon.hpp>

#include <gtest/gtest.h>

using P2f = lsfm::Vec2<float>;
using Poly = lsfm::Polygon<float>;

TEST(PolygonTest, EdgesAndConvexity) {
  Poly poly;
  poly.push_back(P2f(0, 0));
  poly.push_back(P2f(1, 0));
  poly.push_back(P2f(1, 1));
  poly.push_back(P2f(0, 1));

  auto edges = poly.edges();
  ASSERT_EQ(edges.size(), 4u);
  EXPECT_TRUE(poly.isConvex());

  // Manual world vertices = local + pivot (pivot is 0,0 here)
  ASSERT_EQ(poly.verticies().size(), 4u);
  EXPECT_NEAR(poly.verticies()[2].x(), 1.f, 1e-6f);
  EXPECT_NEAR(poly.verticies()[2].y(), 1.f, 1e-6f);
}

TEST(PolygonTest, TranslateRotateScale) {
  Poly poly(P2f(0, 0));
  poly.push_back(P2f(1, 0));
  poly.push_back(P2f(0, 1));
  // translate pivot; world position equals local + pivot
  poly.translate(P2f(2, 3));
  auto pv = poly.piviot();
  auto v0w = poly.verticies()[0] + pv;
  EXPECT_NEAR(v0w.x(), 1.f + 2.f, 1e-6f);
  EXPECT_NEAR(v0w.y(), 0.f + 3.f, 1e-6f);

  // rotate 90 deg around pivot (2,3)
  poly.rotate(static_cast<float>(3.14159265358979323846 / 2));
  auto verts = poly.verticies();
  // Original local point (1,0)-> after 90deg becomes (0,1)
  // So world is pivot + (0,1)
  EXPECT_NEAR((verts[0] + pv).x(), 2.f + 0.f, 1e-5f);
  EXPECT_NEAR((verts[0] + pv).y(), 3.f + 1.f, 1e-5f);

  // scale by 2 around pivot
  poly.scale(2.f);
  verts = poly.verticies();
  pv = poly.piviot();
  EXPECT_NEAR((verts[0] + pv).x(), 2.f + 0.f * 2.f, 1e-5f);
  EXPECT_NEAR((verts[0] + pv).y(), 3.f + 1.f * 2.f, 1e-5f);
}
