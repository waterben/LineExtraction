//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_pose.cpp
/// @brief Unit tests for pose representations.
// Pose tests: composition, inversion, concat

#include <geometry/pose.hpp>

#include <gtest/gtest.h>

using lsfm::Matx33f;
using lsfm::Matx44f;
using lsfm::Posef;
using lsfm::Vec3f;

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

static Matx44f matMul(const Matx44f& A, const Matx44f& B) {
  Matx44f C = Matx44f::Zero();
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++) C(i, j) += A(i, k) * B(k, j);
  return C;
}

static bool isIdentity(const Matx44f& M, float eps = 1e-5f) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float v = M(i, j) - (i == j ? 1.f : 0.f);
      if (std::fabs(v) > eps) return false;
    }
  return true;
}

TEST(PoseTest, HomAndBaseInverse) {
  Posef p(Vec3f(1.f, 2.f, 3.f), Vec3f(0.1f, -0.2f, 0.3f));
  Matx44f H = p.homM();
  Matx44f B = p.baseH();
  auto I = matMul(H, B);
  ASSERT_TRUE(isIdentity(I));
}

TEST(PoseTest, OrientationAndHom) {
  Vec3f t(0.5f, -1.f, 2.f);
  Vec3f r(0.f, kPi / 6.f, -kPi / 3.f);
  Posef p(t, r);
  Matx44f H = p.homM();
  // Compose directly and compare
  Matx44f Hd = lsfm::composeHom(t, lsfm::rodrigues(r));
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) EXPECT_NEAR(H(i, j), Hd(i, j), 1e-5f);
}

TEST(PoseTest, ConcatAndConcatInverse) {
  Posef p1(Vec3f(1, 0, 0), Vec3f(0, 0, kPi / 4.f));
  Posef p2(Vec3f(0, 1, 0), Vec3f(0, kPi / 6.f, 0));
  Posef pc = p1;
  pc.concat(p2);

  // Expect pc.homM() == p2.homM() * p1.homM()
  Matx44f expected = matMul(p2.homM(), p1.homM());
  Matx44f actual = pc.homM();
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) EXPECT_NEAR(actual(i, j), expected(i, j), 1e-5f);

  // Undo with inverse concat
  Posef back = pc;
  back.concatInverse(p2);
  Matx44f backH = back.homM();
  Matx44f p1H = p1.homM();
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) EXPECT_NEAR(backH(i, j), p1H(i, j), 1e-5f);
}
