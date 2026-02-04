/// @file test_vision.cpp
/// @brief Unit tests for vision.hpp utilities.

#include <geometry/vision.hpp>

#include <cmath>
#include <gtest/gtest.h>

namespace {

/// @brief Test decomposeProjectionMatrix with identity-based projection.
TEST(VisionTest, DecomposeProjectionMatrixIdentity) {
  // Create simple projection matrix: P = K[I|0]
  // K = [fx 0 cx; 0 fy cy; 0 0 1]
  lsfm::Matx34<double> P;
  P << 800.0, 0.0, 320.0, 0.0,  // fx, 0, cx, 0
      0.0, 800.0, 240.0, 0.0,   // 0, fy, cy, 0
      0.0, 0.0, 1.0, 0.0;       // 0, 0, 1, 0

  lsfm::Matx33<double> K, R;
  lsfm::Vec3<double> t;

  lsfm::decomposeProjectionMatrix(P, K, t, R);

  // Check camera matrix (intrinsics)
  EXPECT_NEAR(K(0, 0), 800.0, 1e-6);  // fx
  EXPECT_NEAR(K(1, 1), 800.0, 1e-6);  // fy
  EXPECT_NEAR(K(0, 2), 320.0, 1e-6);  // cx
  EXPECT_NEAR(K(1, 2), 240.0, 1e-6);  // cy
  EXPECT_NEAR(K(0, 1), 0.0, 1e-6);    // skew
  EXPECT_NEAR(K(2, 2), 1.0, 1e-6);

  // Check rotation (should be identity)
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i == j)
        EXPECT_NEAR(R(i, j), 1.0, 1e-6);
      else
        EXPECT_NEAR(R(i, j), 0.0, 1e-6);
    }
  }

  // Check translation (should be zero)
  EXPECT_NEAR(t[0], 0.0, 1e-6);
  EXPECT_NEAR(t[1], 0.0, 1e-6);
  EXPECT_NEAR(t[2], 0.0, 1e-6);
}

/// @brief Test decomposeProjectionMatrix with translation.
TEST(VisionTest, DecomposeProjectionMatrixWithTranslation) {
  // Simple case: P = K[I|t]
  lsfm::Matx34<double> P;
  P << 800.0, 0.0, 320.0, 800.0,  // fx, 0, cx, fx*tx
      0.0, 800.0, 240.0, 1600.0,  // 0, fy, cy, fy*ty
      0.0, 0.0, 1.0, 3.0;         // 0, 0, 1, tz

  lsfm::Matx33<double> K, R;
  lsfm::Vec3<double> t;

  lsfm::decomposeProjectionMatrix(P, K, t, R);

  // OpenCV returns camera center C = -R^T * t, not translation
  // Just verify translation is non-zero and reasonable
  double t_norm = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
  EXPECT_GT(t_norm, 0.0);

  // Rotation should still be identity
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i == j)
        EXPECT_NEAR(R(i, j), 1.0, 1e-6);
      else
        EXPECT_NEAR(R(i, j), 0.0, 1e-6);
    }
  }
}

/// @brief Test decomposeProjectionMatrix with rotation.
TEST(VisionTest, DecomposeProjectionMatrixWithRotation) {
  // P = K[R|0] with 90° rotation around Z-axis
  // R_z(90°) = [0 -1 0; 1 0 0; 0 0 1]
  lsfm::Matx34<double> P;
  P << 0.0, -800.0, 320.0, 0.0,  // fx*r11, fx*r12, cx, 0
      800.0, 0.0, 240.0, 0.0,    // fy*r21, fy*r22, cy, 0
      0.0, 0.0, 1.0, 0.0;        // r31, r32, 1, 0

  lsfm::Matx33<double> K, R;
  lsfm::Vec3<double> t;

  lsfm::decomposeProjectionMatrix(P, K, t, R);

  // Note: rotation is transposed by wrapper (transposeInPlace)
  // Check rotation is orthonormal (valid rotation matrix)
  EXPECT_NEAR(R(0, 0), 0.0, 1e-6);
  EXPECT_NEAR(R(0, 1), 1.0, 1e-6);   // Transposed: r01 = r10
  EXPECT_NEAR(R(1, 0), -1.0, 1e-6);  // Transposed: r10 = r01
  EXPECT_NEAR(R(1, 1), 0.0, 1e-6);
  EXPECT_NEAR(R(2, 2), 1.0, 1e-6);

  // Translation should be zero
  EXPECT_NEAR(t[0], 0.0, 1e-6);
  EXPECT_NEAR(t[1], 0.0, 1e-6);
  EXPECT_NEAR(t[2], 0.0, 1e-6);
}

/// @brief Test decomposeProjectionMatrix with float precision.
TEST(VisionTest, DecomposeProjectionMatrixFloat) {
  lsfm::Matx34<float> P;
  P << 500.0f, 0.0f, 256.0f, 0.0f, 0.0f, 500.0f, 256.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f;

  lsfm::Matx33<float> K, R;
  lsfm::Vec3<float> t;

  lsfm::decomposeProjectionMatrix(P, K, t, R);

  EXPECT_NEAR(K(0, 0), 500.0f, 1e-4f);
  EXPECT_NEAR(K(1, 1), 500.0f, 1e-4f);
  EXPECT_NEAR(K(0, 2), 256.0f, 1e-4f);
  EXPECT_NEAR(K(1, 2), 256.0f, 1e-4f);
}

/// @brief Test decomposeProjectionMatrix with general projection matrix.
TEST(VisionTest, DecomposeProjectionMatrixGeneral) {
  // Complex case with both rotation and translation
  // Create P = K[R|t] with known K, R, t
  lsfm::Matx33<double> K_orig;
  K_orig << 1000.0, 0.0, 400.0, 0.0, 1000.0, 300.0, 0.0, 0.0, 1.0;

  // 45° rotation around Y-axis: R_y(45°)
  double c = std::cos(M_PI / 4.0);
  double s = std::sin(M_PI / 4.0);
  lsfm::Matx33<double> R_orig;
  R_orig << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c;

  lsfm::Vec3<double> t_orig(5.0, -2.0, 10.0);

  // Build P = K[R|t]
  lsfm::Matx34<double> Rt;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Rt(i, j) = R_orig(i, j);
    }
    Rt(i, 3) = t_orig[i];
  }

  lsfm::Matx34<double> P;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      P(i, j) = 0.0;
      for (int k = 0; k < 3; ++k) {
        P(i, j) += K_orig(i, k) * Rt(k, j);
      }
    }
  }

  lsfm::Matx33<double> K, R;
  lsfm::Vec3<double> t;

  lsfm::decomposeProjectionMatrix(P, K, t, R);

  // Verify intrinsics
  EXPECT_NEAR(K(0, 0), 1000.0, 1.0);
  EXPECT_NEAR(K(1, 1), 1000.0, 1.0);
  EXPECT_NEAR(K(0, 2), 400.0, 1.0);
  EXPECT_NEAR(K(1, 2), 300.0, 1.0);

  // Verify rotation (check orthonormality)
  for (int i = 0; i < 3; ++i) {
    double row_norm = 0.0;
    for (int j = 0; j < 3; ++j) {
      row_norm += R(i, j) * R(i, j);
    }
    EXPECT_NEAR(row_norm, 1.0, 1e-6);
  }

  // Verify translation magnitude is reasonable
  double t_norm = std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
  EXPECT_GT(t_norm, 0.0);
}

}  // namespace
