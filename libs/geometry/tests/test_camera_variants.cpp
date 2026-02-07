//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_camera_variants.cpp
/// @brief Unit tests for camera model variants.
// Verify multiple camera projection variants agree (Pluecker, CV, 2P)

#include <geometry/camera.hpp>
#include <geometry/cameracv.hpp>

#include <gtest/gtest.h>

using FT = float;
using lsfm::Line3;
using lsfm::LineSegment3;
using lsfm::Vec2;
using lsfm::Vec3;
using CamP = lsfm::CameraPluecker<FT>;
using CamCV = lsfm::CameraCV<FT>;
using Cam2P = lsfm::Camera2P<FT>;

static std::vector<Vec3<FT>> makeCube() {
  return {{-1, 1, 1}, {1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {-1, 1, -1}, {1, 1, -1}, {1, -1, -1}, {-1, -1, -1}};
}

TEST(CameraVariantsTest, ProjectPointsAgree) {
  Vec3<FT> origin(0, 0, 10);
  Vec3<FT> orientation(3.14159265f, 0, 0);  // flip to look towards -Z
  FT fov = static_cast<FT>(50.0 / 180.0 * 3.14159265);
  Vec2<FT> size(800, 400);

  CamP camP(fov, size, origin, orientation);
  CamCV camCV(fov, size, origin, orientation);
  Cam2P cam2P(fov, size, origin, orientation);

  auto verts = makeCube();
  std::vector<Vec2<FT>> pP, pCV, p2P;
  pP.reserve(verts.size());
  pCV.reserve(verts.size());
  p2P.reserve(verts.size());
  for (auto& v : verts) {
    pP.push_back(camP.project(v));
    pCV.push_back(camCV.project(v));
    p2P.push_back(cam2P.project(v));
  }
  ASSERT_EQ(pP.size(), verts.size());
  ASSERT_EQ(pCV.size(), verts.size());
  ASSERT_EQ(p2P.size(), verts.size());
  for (size_t i = 0; i < verts.size(); ++i) {
    EXPECT_NEAR(pP[i].x(), pCV[i].x(), 1e-3f);
    EXPECT_NEAR(pP[i].y(), pCV[i].y(), 1e-3f);
    EXPECT_NEAR(pP[i].x(), p2P[i].x(), 1e-3f);
    EXPECT_NEAR(pP[i].y(), p2P[i].y(), 1e-3f);
  }
}

TEST(CameraVariantsTest, ProjectLinesAgree) {
  Vec3<FT> origin(0, 0, 10);
  Vec3<FT> orientation(3.14159265f, 0, 0);
  FT fov = static_cast<FT>(50.0 / 180.0 * 3.14159265);
  Vec2<FT> size(800, 400);

  CamP camP(fov, size, origin, orientation);
  CamCV camCV(fov, size, origin, orientation);
  Cam2P cam2P(fov, size, origin, orientation);

  auto verts = makeCube();
  std::vector<std::pair<size_t, size_t>> edges = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                                                  {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  std::vector<Line3<FT>> lines;
  lines.reserve(edges.size());
  for (auto e : edges) lines.emplace_back(LineSegment3<FT>(verts[e.first], verts[e.second]));

  std::vector<lsfm::Line<FT>> lP, lCV, l2P;
  camP.project(lines, lP);
  camCV.project(lines, lCV);
  cam2P.project(lines, l2P);
  ASSERT_EQ(lP.size(), lines.size());
  for (size_t i = 0; i < lines.size(); ++i) {
    // Compare normals and distances
    EXPECT_NEAR(lP[i].normalX(), lCV[i].normalX(), 1e-3f);
    EXPECT_NEAR(lP[i].normalY(), lCV[i].normalY(), 1e-3f);
    EXPECT_NEAR(lP[i].originDist(), lCV[i].originDist(), 1e-2f);
    EXPECT_NEAR(lP[i].normalX(), l2P[i].normalX(), 1e-2f);
    EXPECT_NEAR(lP[i].normalY(), l2P[i].normalY(), 1e-2f);
    EXPECT_NEAR(lP[i].originDist(), l2P[i].originDist(), 2e-1f);
  }
}
