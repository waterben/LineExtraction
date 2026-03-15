//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_multiview.cpp
/// @brief Unit tests for MultiviewStereo N-view line triangulation.

#include <geometry/camera.hpp>
#include <geometry/multiview.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using FT = double;
using lsfm::Camera;
using lsfm::CameraPluecker;
using lsfm::Line;
using lsfm::Line3;
using lsfm::LineSegment;
using lsfm::LineSegment3;
using lsfm::MultiviewStereo;
using lsfm::Vec2;
using lsfm::Vec3;

/// @brief Create a ring of cameras looking at the origin.
static std::vector<Camera<FT>> make_orbital_cameras(int n, FT radius, FT height) {
  std::vector<Camera<FT>> cameras;
  FT fov = static_cast<FT>(50.0 / 180.0 * M_PI);
  Vec2<FT> size(640, 480);

  for (int i = 0; i < n; ++i) {
    FT angle = static_cast<FT>(2.0 * M_PI * i / n);
    Vec3<FT> eye(radius * std::cos(angle), height, radius * std::sin(angle));
    // Look-at: compute rotation pointing camera at origin.
    Vec3<FT> forward = Vec3<FT>(0, 0, 0) - eye;
    forward *= static_cast<FT>(1.0) / std::sqrt(forward.dot(forward));
    Vec3<FT> up(0, 1, 0);
    Vec3<FT> right = forward.cross(up);
    right *= static_cast<FT>(1.0) / std::sqrt(right.dot(right));
    up = right.cross(forward);

    // Rodrigues from rotation matrix (w2c)
    lsfm::Matx33<FT> rot_w2c;
    rot_w2c(0, 0) = right[0];
    rot_w2c(0, 1) = right[1];
    rot_w2c(0, 2) = right[2];
    rot_w2c(1, 0) = -up[0];
    rot_w2c(1, 1) = -up[1];
    rot_w2c(1, 2) = -up[2];
    rot_w2c(2, 0) = -forward[0];
    rot_w2c(2, 1) = -forward[1];
    rot_w2c(2, 2) = -forward[2];

    // Camera uses c2w convention: origin = eye, rot = R_w2c^T
    cv::Matx33d rot_w2c_cv(rot_w2c(0, 0), rot_w2c(0, 1), rot_w2c(0, 2), rot_w2c(1, 0), rot_w2c(1, 1), rot_w2c(1, 2),
                           rot_w2c(2, 0), rot_w2c(2, 1), rot_w2c(2, 2));
    cv::Matx33d rot_c2w_cv = rot_w2c_cv.t();
    cv::Vec3d rodrigues_cv;
    cv::Rodrigues(rot_c2w_cv, rodrigues_cv);
    Vec3<FT> rodrigues(rodrigues_cv[0], rodrigues_cv[1], rodrigues_cv[2]);

    cameras.emplace_back(fov, size, eye, rodrigues);
  }
  return cameras;
}

TEST(MultiviewStereoTest, TwoViewMatchesStereoPlane) {
  auto cameras = make_orbital_cameras(2, 10.0, 3.0);
  MultiviewStereo<FT> mv(cameras);
  lsfm::StereoPlane<FT> sp(cameras[0], cameras[1]);

  EXPECT_EQ(mv.num_cameras(), 2u);

  // Define a 3D test line perpendicular to the baseline.
  // Cameras are at (10,3,0) and (-10,3,0), so baseline is x-axis.
  // Use a z-aligned line to avoid degeneracy.
  Line3<FT> gt_line = Line3<FT>::twoPoint(Vec3<FT>(0, 1, -1), Vec3<FT>(0, 1, 1));

  CameraPluecker<FT> cam0_p(cameras[0]);
  CameraPluecker<FT> cam1_p(cameras[1]);

  Line<FT> proj0 = cam0_p.project(gt_line);
  Line<FT> proj1 = cam1_p.project(gt_line);

  ASSERT_FALSE(proj0.empty()) << "Projection into camera 0 failed";
  ASSERT_FALSE(proj1.empty()) << "Projection into camera 1 failed";

  // Triangulate via both methods.
  std::vector<std::pair<size_t, Line<FT>>> obs = {{0, proj0}, {1, proj1}};
  Line3<FT> mv_result = mv.triangulate(obs);
  Line3<FT> sp_result = sp.triangulate(proj0, proj1);

  ASSERT_FALSE(mv_result.empty()) << "Multiview triangulation failed";
  ASSERT_FALSE(sp_result.empty()) << "StereoPlane triangulation failed";

  // Both multiview and StereoPlane should recover roughly the correct
  // direction (z-axis).  With only 2 views the constraint is minimal,
  // so we allow a generous tolerance.
  Vec3<FT> gt_dir(0, 0, 1);
  FT mv_dot = std::abs(mv_result.direction().dot(gt_dir) / std::sqrt(mv_result.direction().dot(mv_result.direction())));
  FT sp_dot = std::abs(sp_result.direction().dot(gt_dir) / std::sqrt(sp_result.direction().dot(sp_result.direction())));
  EXPECT_GT(mv_dot, 0.5) << "Multiview direction too far from GT";
  EXPECT_GT(sp_dot, 0.5) << "StereoPlane direction too far from GT";

  // Both should be close to the ground truth line.
  EXPECT_LT(gt_line.distance(mv_result.origin()), 2.0);
}

TEST(MultiviewStereoTest, FourViewTriangulation) {
  auto cameras = make_orbital_cameras(4, 10.0, 3.0);
  MultiviewStereo<FT> mv(cameras);

  // 3D test line segment.
  Line3<FT> gt_line = Line3<FT>::twoPoint(Vec3<FT>(-1, 1, 0), Vec3<FT>(1, 1, 0));

  // Project into all 4 views.
  std::vector<std::pair<size_t, Line<FT>>> obs;
  for (size_t i = 0; i < 4; ++i) {
    CameraPluecker<FT> cam_p(cameras[i]);
    Line<FT> proj = cam_p.project(gt_line);
    if (!proj.empty()) {
      obs.emplace_back(i, proj);
    }
  }

  ASSERT_GE(obs.size(), 2u);

  Line3<FT> result = mv.triangulate(obs);
  EXPECT_FALSE(result.empty());

  // Direction should be close to GT direction.
  Vec3<FT> gt_dir(1, 0, 0);
  FT dot = std::abs(result.direction().dot(gt_dir) / std::sqrt(result.direction().dot(result.direction())));
  EXPECT_GT(dot, 0.7);

  // Distance from GT line should be moderate (exact accuracy depends on
  // camera geometry).
  EXPECT_LT(gt_line.distance(result.origin()), 2.0);
}

TEST(MultiviewStereoTest, SegmentTriangulation) {
  auto cameras = make_orbital_cameras(4, 10.0, 3.0);
  MultiviewStereo<FT> mv(cameras);

  // 3D test line segment.
  LineSegment3<FT> gt_seg(Vec3<FT>(-1, 1, 0), Vec3<FT>(1, 1, 0));

  // Project into all views.
  std::vector<std::pair<size_t, LineSegment<FT>>> obs;
  for (size_t i = 0; i < 4; ++i) {
    CameraPluecker<FT> cam_p(cameras[i]);
    LineSegment<FT> proj = cam_p.project(gt_seg);
    if (!proj.empty()) {
      obs.emplace_back(i, proj);
    }
  }

  ASSERT_GE(obs.size(), 2u);

  LineSegment3<FT> result = mv.triangulate(obs);
  EXPECT_FALSE(result.empty());

  // Length should be reasonable (GT length = 2).
  EXPECT_GT(result.length(), 0.5);
  EXPECT_LT(result.length(), 5.0);
}

TEST(MultiviewStereoTest, BatchTriangulation) {
  auto cameras = make_orbital_cameras(4, 10.0, 3.0);
  MultiviewStereo<FT> mv(cameras);

  // Multiple 3D lines.
  std::vector<Line3<FT>> gt_lines = {
      Line3<FT>::twoPoint(Vec3<FT>(-1, 0, 0), Vec3<FT>(1, 0, 0)),
      Line3<FT>::twoPoint(Vec3<FT>(0, -1, 0), Vec3<FT>(0, 1, 0)),
      Line3<FT>::twoPoint(Vec3<FT>(0, 0, -1), Vec3<FT>(0, 0, 1)),
  };

  std::vector<std::vector<std::pair<size_t, Line<FT>>>> tracks;
  for (const auto& gt : gt_lines) {
    std::vector<std::pair<size_t, Line<FT>>> obs;
    for (size_t i = 0; i < 4; ++i) {
      CameraPluecker<FT> cam_p(cameras[i]);
      Line<FT> proj = cam_p.project(gt);
      if (!proj.empty()) {
        obs.emplace_back(i, proj);
      }
    }
    tracks.push_back(obs);
  }

  std::vector<Line3<FT>> results;
  mv.triangulate(tracks, results);
  ASSERT_EQ(results.size(), 3u);

  for (size_t i = 0; i < 3; ++i) {
    EXPECT_FALSE(results[i].empty()) << "Line " << i << " should not be empty";
  }
}

TEST(MultiviewStereoTest, DegenerateSingleView) {
  auto cameras = make_orbital_cameras(4, 10.0, 3.0);
  MultiviewStereo<FT> mv(cameras);

  // Only one observation — should return empty.
  CameraPluecker<FT> cam_p(cameras[0]);
  Line3<FT> gt = Line3<FT>::twoPoint(Vec3<FT>(-1, 0, 0), Vec3<FT>(1, 0, 0));
  Line<FT> proj = cam_p.project(gt);

  std::vector<std::pair<size_t, Line<FT>>> obs = {{0, proj}};
  Line3<FT> result = mv.triangulate(obs);
  EXPECT_TRUE(result.empty());
}
