// Camera tests: camera matrix and projection matrix decompose
#include <gtest/gtest.h>

#include <geometry/camera.hpp>

using lsfm::Vec2f;
using lsfm::Vec3f;
using lsfm::Matx33f;
using lsfm::Matx34f;

TEST(CameraTest, ComposeAndDecomposeCamMatrix)
{
  Vec2f focal(800.f, 820.f);
  Vec2f offset(320.f, 240.f);
  Matx33f K = lsfm::Camera<float>::composeCameraMatrix(focal, offset);
  Vec2f focal_out, offset_out;
  lsfm::Camera<float>::decomposeCameraMatrix(K, focal_out, offset_out);
  EXPECT_NEAR(focal_out.x(), focal.x(), 1e-5f);
  EXPECT_NEAR(focal_out.y(), focal.y(), 1e-5f);
  EXPECT_NEAR(offset_out.x(), offset.x(), 1e-5f);
  EXPECT_NEAR(offset_out.y(), offset.y(), 1e-5f);
}

TEST(CameraTest, ComposeAndDecomposeProjectionMatrix)
{
  Vec2f focal(700.f, 710.f);
  Vec2f offset(320.f, 200.f);
  Vec3f t(1.f, 2.f, 3.f);
  Vec3f r(0.1f, -0.2f, 0.3f);
  Matx34f P = lsfm::Camera<float>::composeProjectionMatrix(
      lsfm::Camera<float>::composeCameraMatrix(focal, offset), t, lsfm::rodrigues(r));

  Matx33f K_out; Vec3f t_out; Matx33f R_out;
  lsfm::Camera<float>::decomposeProjectionMatrix(P, K_out, t_out, R_out);

  // Intrinsics should match
  Vec2f focal_out, offset_out;
  lsfm::Camera<float>::decomposeCameraMatrix(K_out, focal_out, offset_out);
  EXPECT_NEAR(focal_out.x(), focal.x(), 1e-3f);
  EXPECT_NEAR(focal_out.y(), focal.y(), 1e-3f);
  EXPECT_NEAR(offset_out.x(), offset.x(), 1e-3f);
  EXPECT_NEAR(offset_out.y(), offset.y(), 1e-3f);

  // Recompose and compare P up to scale; normalize both by P(2,3)
  Matx34f P2 = lsfm::Camera<float>::composeProjectionMatrix(K_out, t_out, R_out);
  float s1 = P(2,3) == 0 ? 1.f : P(2,3);
  float s2 = P2(2,3) == 0 ? 1.f : P2(2,3);
  for (int i=0;i<3;i++) for (int j=0;j<4;j++)
    EXPECT_NEAR(P(i,j)/s1, P2(i,j)/s2, 1e-3f);
}

