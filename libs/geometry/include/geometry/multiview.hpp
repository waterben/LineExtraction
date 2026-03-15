//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann & Manuel Lange
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file multiview.hpp
/// @brief Multiview line triangulation via interpretation plane intersection.
///
/// Provides the MultiviewStereo class for triangulating 3D lines from
/// corresponding 2D line observations in N >= 2 views.  The algorithm
/// constructs an interpretation plane for each observation and recovers
/// the 3D line through SVD-based least squares on the plane normals.
///
/// @see Bartoli, A. & Sturm, P. "The 3D Line Motion Matrix and Alignment
///      of Line Reconstructions", IJCV 2004.
/// @see LIMAP: https://github.com/cvg/limap

#pragma once

#include <geometry/stereo.hpp>
#include <opencv2/core.hpp>

#include <utility>
#include <vector>


namespace lsfm {

/// @brief Multiview line triangulation using interpretation plane intersection.
///
/// Given N >= 2 cameras and a set of 2D line observations (one per view),
/// the algorithm:
/// 1. Constructs an interpretation plane through each camera center
///    containing the back-projected 2D line.
/// 2. Recovers the 3D line direction as the right singular vector of
///    the plane-normal matrix corresponding to the smallest singular value.
/// 3. Recovers a point on the 3D line via least-squares on the plane
///    offset equations, projected orthogonal to the recovered direction.
///
/// For exactly 2 views this reduces to the classical plane intersection
/// used by StereoPlane.
///
/// @tparam FT Floating-point type (float or double).
template <class FT>
class MultiviewStereo {
 protected:
  std::vector<Camera<FT>> cameras_;    ///< Registered cameras.
  std::vector<Matx33<FT>> rotations_;  ///< Cached rotation matrices.

 public:
  typedef FT float_type;

  /// @brief Construct from a vector of cameras.
  /// @param cameras N cameras in a common world frame.
  explicit MultiviewStereo(const std::vector<Camera<FT>>& cameras) : cameras_(cameras), rotations_() {
    rotations_.reserve(cameras_.size());
    for (const auto& cam : cameras_) {
      rotations_.push_back(cam.rotM());
    }
  }

  /// @brief Number of registered cameras.
  /// @return Camera count.
  size_t num_cameras() const { return cameras_.size(); }

  /// @brief Access a camera by index.
  /// @param idx Camera index.
  /// @return Const reference to the camera.
  const Camera<FT>& camera(size_t idx) const { return cameras_[idx]; }

  /// @name Line Triangulation
  /// @{

  /// @brief Triangulate a 3D line from observations in multiple views.
  ///
  /// Each observation is a ``(view_index, 2D line)`` pair.  At least
  /// two observations from different cameras are required.
  ///
  /// @param observations Vector of (camera_index, 2D_line) pairs.
  /// @return Triangulated 3D line (empty if degenerate).
  Line3<FT> triangulate(const std::vector<std::pair<size_t, Line<FT>>>& observations) const {
    const size_t n = observations.size();
    if (n < 2) return Line3<FT>();

    // Build plane-normal matrix A (n x 3) and offset vector b (n x 1).
    cv::Mat A(static_cast<int>(n), 3, cv::traits::Type<FT>::value);
    cv::Mat b(static_cast<int>(n), 1, cv::traits::Type<FT>::value);

    for (size_t i = 0; i < n; ++i) {
      const size_t idx = observations[i].first;
      const Line<FT>& line2d = observations[i].second;
      const auto& cam = cameras_[idx];
      const auto& rot = rotations_[idx];

      Plane<FT> plane = planeFromLine(cam.focal(), cam.offset(), rot, cam.origin(), line2d);

      if (plane.empty()) return Line3<FT>();

      const Vec3<FT>& normal = plane.normal();
      A.at<FT>(static_cast<int>(i), 0) = normal[0];
      A.at<FT>(static_cast<int>(i), 1) = normal[1];
      A.at<FT>(static_cast<int>(i), 2) = normal[2];
      b.at<FT>(static_cast<int>(i), 0) = plane.dist2origin();
    }

    // Direction: right singular vector for smallest singular value.
    cv::Mat w, u, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::FULL_UV);
    Vec3<FT> direction(vt.at<FT>(2, 0), vt.at<FT>(2, 1), vt.at<FT>(2, 2));

    FT dir_norm = std::sqrt(direction.dot(direction));
    if (dir_norm < LIMITS<FT>::tau()) return Line3<FT>();
    direction *= static_cast<FT>(1.0) / dir_norm;

    // Point on 3D line: solve min ||A*p - b||^2 via SVD pseudo-inverse.
    // With n < 3 this is under-determined; use A^+ = V * S^+ * U^T.
    int rows = A.rows;
    int cols = A.cols;
    int rank = std::min(rows, cols);
    cv::Mat s_inv = cv::Mat::zeros(cols, rows, cv::traits::Type<FT>::value);
    for (int i = 0; i < rank; ++i) {
      FT si = w.at<FT>(i, 0);
      if (si > LIMITS<FT>::tau()) {
        s_inv.at<FT>(i, i) = static_cast<FT>(1.0) / si;
      }
    }
    cv::Mat p_ls = vt.t() * s_inv * u.t() * b;
    Vec3<FT> point(p_ls.at<FT>(0, 0), p_ls.at<FT>(1, 0), p_ls.at<FT>(2, 0));

    // Remove component along direction to get the closest point to origin
    // on the 3D line.
    point -= direction * direction.dot(point);

    return Line3<FT>(point, direction);
  }

  /// @brief Batch triangulation of line tracks.
  /// @param tracks Vector of observation vectors (one per track).
  /// @param[out] results Triangulated 3D lines.
  void triangulate(const std::vector<std::vector<std::pair<size_t, Line<FT>>>>& tracks,
                   std::vector<Line3<FT>>& results) const {
    results.clear();
    results.reserve(tracks.size());
    for (const auto& obs : tracks) {
      results.push_back(triangulate(obs));
    }
  }

  /// @}

  /// @name Line Segment Triangulation
  /// @{

  /// @brief Triangulate a 3D line segment from segment observations.
  ///
  /// Triangulates the supporting line, then projects all 2D segment
  /// endpoints onto the 3D line to determine the segment extent.
  ///
  /// @param observations Vector of (camera_index, 2D_segment) pairs.
  /// @return Triangulated 3D line segment (empty if degenerate).
  LineSegment3<FT> triangulate(const std::vector<std::pair<size_t, LineSegment<FT>>>& observations) const {
    // Build Line observations for the supporting line.
    std::vector<std::pair<size_t, Line<FT>>> line_obs;
    line_obs.reserve(observations.size());
    for (const auto& [idx, seg] : observations) {
      line_obs.emplace_back(idx, static_cast<const Line<FT>&>(seg));
    }

    Line3<FT> line3d = triangulate(line_obs);
    if (line3d.empty()) return LineSegment3<FT>();

    // Project all 2D endpoints onto the 3D line to find the extent.
    FT t_min = std::numeric_limits<FT>::max();
    FT t_max = std::numeric_limits<FT>::lowest();

    for (const auto& [idx, seg] : observations) {
      const auto& cam = cameras_[idx];
      const auto& rot = rotations_[idx];

      // Project start and end points as rays, find nearest t on 3D line.
      Line3<FT> ray_start = lineFromPixel(cam.focal(), cam.offset(), rot, cam.origin(), seg.startPoint());
      Line3<FT> ray_end = lineFromPixel(cam.focal(), cam.offset(), rot, cam.origin(), seg.endPoint());

      FT t_s = line3d.normalDistance(ray_start);
      FT t_e = line3d.normalDistance(ray_end);

      t_min = std::min({t_min, t_s, t_e});
      t_max = std::max({t_max, t_s, t_e});
    }

    if (t_min >= t_max) return LineSegment3<FT>();

    return LineSegment3<FT>(line3d, t_min, t_max);
  }

  /// @brief Batch triangulation of segment tracks.
  /// @param tracks Vector of segment observation vectors.
  /// @param[out] results Triangulated 3D line segments.
  void triangulate(const std::vector<std::vector<std::pair<size_t, LineSegment<FT>>>>& tracks,
                   std::vector<LineSegment3<FT>>& results) const {
    results.clear();
    results.reserve(tracks.size());
    for (const auto& obs : tracks) {
      results.push_back(triangulate(obs));
    }
  }

  /// @}
};

/// @brief Single-precision multiview stereo.
typedef MultiviewStereo<float> MultiviewStereof;

/// @brief Double-precision multiview stereo.
typedef MultiviewStereo<double> MultiviewStereod;

}  // namespace lsfm
