//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_projection_analysis.cpp
/// @brief Unit tests for projection analysis utilities.

#include <geometry/camera.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <vector>

using FT = float;
using lsfm::CameraHom;
using lsfm::Vec2;
using lsfm::Vec3;

TEST(ProjectionAnalysis, CompareProjectionMethods) {
  // Create a simple camera
  Vec3<FT> origin(0, 0, 0);
  Vec3<FT> orientation(0, 0, 0);
  FT fov = static_cast<FT>(50.0 / 180.0 * 3.14159265);
  lsfm::Vec2<FT> size(800, 400);

  CameraHom<FT> cam(fov, size, origin, orientation);

  // Test points
  std::vector<Vec3<FT>> test_points = {{1.0f, 2.0f, 5.0f}, {-1.0f, 1.0f, 3.0f}, {0.5f, -0.5f, 4.0f}};

  std::cout << "=== Input Analysis ===" << std::endl;
  for (size_t i = 0; i < test_points.size(); ++i) {
    std::cout << "Point [" << i << "]: (" << test_points[i].x() << ", " << test_points[i].y() << ", "
              << test_points[i].z() << ")" << std::endl;
  }

  // Method 1: Element-wise projection (using individual calls)
  std::vector<Vec2<FT>> result1(test_points.size());
  for (size_t i = 0; i < test_points.size(); ++i) {
    result1[i] = cam.project(test_points[i]);
  }

  // Method 2: Vector projection (using the new optimized project() method)
  std::vector<Vec2<FT>> result2;
  cam.project(test_points, result2);

  std::cout << "\n=== Results Comparison ===" << std::endl;
  std::cout << "Method 1 (element-wise calls):" << std::endl;
  for (size_t i = 0; i < result1.size(); ++i) {
    std::cout << "  [" << i << "]: (" << result1[i].x() << ", " << result1[i].y() << ")" << std::endl;
  }

  std::cout << "\nMethod 2 (optimized vector projection):" << std::endl;
  for (size_t i = 0; i < result2.size(); ++i) {
    std::cout << "  [" << i << "]: (" << result2[i].x() << ", " << result2[i].y() << ")" << std::endl;
  }

  std::cout << "\n=== Differences ===" << std::endl;
  bool all_match = true;
  for (size_t i = 0; i < result1.size(); ++i) {
    FT diff_x = std::abs(result1[i].x() - result2[i].x());
    FT diff_y = std::abs(result1[i].y() - result2[i].y());
    std::cout << "  [" << i << "]: diff_x=" << diff_x << ", diff_y=" << diff_y << std::endl;

    if (diff_x > 1e-6f || diff_y > 1e-6f) {
      std::cout << "    WARNING: Significant difference detected!" << std::endl;
      all_match = false;
    }
  }

  if (all_match) {
    std::cout << "SUCCESS: Both methods produce identical results!" << std::endl;
  }

  // Compare results with reasonable tolerance for floating point arithmetic
  for (size_t i = 0; i < result1.size(); ++i) {
    EXPECT_NEAR(result1[i].x(), result2[i].x(), 1e-4f) << "X coordinate mismatch at index " << i;
    EXPECT_NEAR(result1[i].y(), result2[i].y(), 1e-4f) << "Y coordinate mismatch at index " << i;
  }
}
