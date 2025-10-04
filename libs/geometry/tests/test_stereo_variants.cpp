// Verify triangulation via StereoCV, StereoPlane, and Stereo (2P) agree
#include <geometry/camera.hpp>
#include <geometry/cameracv.hpp>
#include <geometry/stereocv.hpp>

#include <gtest/gtest.h>

using FT = float;
using lsfm::Line;
using lsfm::Line3;
using lsfm::Vec2;
using lsfm::Vec3;
using CamP = lsfm::CameraPluecker<FT>;
using CamCV = lsfm::CameraCV<FT>;
using Cam2P = lsfm::Camera2P<FT>;

static std::vector<Vec3<FT>> makeCube() {
  return {{-1, 1, 1}, {1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {-1, 1, -1}, {1, 1, -1}, {1, -1, -1}, {-1, -1, -1}};
}

TEST(StereoVariantsTest, TriangulatePointsAgree) {
  Vec3<FT> originL(-1, 0, 10), originR(1, 0, 10);
  Vec3<FT> orientation(3.14159265f, 0, 0);
  FT fov = static_cast<FT>(50.0 / 180.0 * 3.14159265);
  Vec2<FT> size(800, 400);

  CamP camLP(fov, size, originL, orientation);
  CamCV camLCV(fov, size, originL, orientation);
  Cam2P camL2P(fov, size, originL, orientation);
  CamP camRP(fov, size, originR, orientation);
  CamCV camRCV(fov, size, originR, orientation);
  Cam2P camR2P(fov, size, originR, orientation);

  lsfm::StereoCV<FT> stereoCV(camLCV, camRCV);
  lsfm::StereoPlane<FT> stereoP(camLP, camRP);
  lsfm::Stereo<FT> stereo2P(camL2P, camR2P);

  auto verts = makeCube();
  std::vector<Vec2<FT>> pL, pR, pLCV, pRCV, pL2P, pR2P;
  camLP.project(verts, pL);
  camRP.project(verts, pR);
  camLCV.project(verts, pLCV);
  camRCV.project(verts, pRCV);
  camL2P.project(verts, pL2P);
  camR2P.project(verts, pR2P);

  std::vector<Vec3<FT>> tCV, tP, t2P;
  stereoCV.triangulate(pLCV, pRCV, tCV);
  stereoP.triangulate(pL, pR, tP);
  stereo2P.triangulate(pL2P, pR2P, t2P);
  ASSERT_EQ(tCV.size(), verts.size());
  ASSERT_EQ(tP.size(), verts.size());
  ASSERT_EQ(t2P.size(), verts.size());
  for (size_t i = 0; i < verts.size(); ++i) {
    // Original points are in camera space; expect consistency up to small error
    EXPECT_NEAR(tP[i].x(), tCV[i].x(), 1e-2f);
    EXPECT_NEAR(tP[i].y(), tCV[i].y(), 1e-2f);
    EXPECT_NEAR(tP[i].z(), tCV[i].z(), 5e-2f);
    EXPECT_NEAR(tP[i].x(), t2P[i].x(), 1e-1f);
    EXPECT_NEAR(tP[i].y(), t2P[i].y(), 1e-1f);
    EXPECT_NEAR(tP[i].z(), t2P[i].z(), 2e-1f);
  }
}
