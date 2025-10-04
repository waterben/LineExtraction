// Geometry base (matrix/vec) utilities tests
#include <geometry/base.hpp>

#include <gtest/gtest.h>

#include <cmath>

using lsfm::Matx33f;
using lsfm::Matx44f;
using lsfm::Vec3f;
namespace {
constexpr float kPi = 3.14159265358979323846f;
}

TEST(GeometryBaseTest, RodriguesVectorToMatrixAndBack) {
  // 90 deg around Z axis
  Vec3f r(0.f, 0.f, kPi / 2.f);
  Matx33f R = lsfm::rodrigues(r);
  // Expected rotation matrix around Z
  EXPECT_NEAR(R(0, 0), 0.f, 1e-5f);
  EXPECT_NEAR(R(0, 1), -1.f, 1e-5f);
  EXPECT_NEAR(R(1, 0), 1.f, 1e-5f);
  EXPECT_NEAR(R(1, 1), 0.f, 1e-5f);
  EXPECT_NEAR(R(2, 2), 1.f, 1e-5f);

  // Convert back to axis-angle vector
  Vec3f r_back = lsfm::rodrigues(R);
  EXPECT_NEAR(r_back[0], r[0], 1e-5f);
  EXPECT_NEAR(r_back[1], r[1], 1e-5f);
  EXPECT_NEAR(r_back[2], r[2], 1e-5f);
}

TEST(GeometryBaseTest, ComposeAndDecomposeHomogeneous3D) {
  Vec3f t(1.f, 2.f, 3.f);
  Vec3f r(0.f, kPi / 4.f, 0.f);  // rotate around Y
  Matx44f H = lsfm::composeHom(t, lsfm::rodrigues(r));

  lsfm::Vec3f t_out;
  lsfm::Matx33f R_out;
  lsfm::decomposeHom(H, t_out, R_out);

  // translation recovered
  EXPECT_NEAR(t_out[0], t[0], 1e-6f);
  EXPECT_NEAR(t_out[1], t[1], 1e-6f);
  EXPECT_NEAR(t_out[2], t[2], 1e-6f);

  // rotation recovered
  Matx33f R_in = lsfm::rodrigues(r);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) EXPECT_NEAR(R_out(i, j), R_in(i, j), 1e-5f);
}
