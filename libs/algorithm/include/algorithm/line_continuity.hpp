//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file line_continuity.hpp
/// @brief Unified line segment continuity optimizer (merge + optional gradient).

#pragma once

#include <geometry/line.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <utility/value_manager.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace lsfm {

/// @brief Merge strategy enumeration.
///
/// Duplicated here for self-containment; identical to
/// the definition in line_merge.hpp.
enum class ContinuityMergeType : int {
  STANDARD = 0,  ///< Merge using furthest endpoints of the pair.
  AVG = 1        ///< Merge by averaging the endpoint positions.
};

/// @brief Unified line segment continuity optimizer.
///
/// Provides two operating modes that share the same geometric constraints:
///
/// **Variant 1 — geometry-only merge** (`optimize` without magnitude):
/// Merges collinear segment pairs that pass all four geometric checks
/// (angle, endpoint distance, perpendicular distance, parallel gap).
///
/// **Variant 2 — gradient-assisted merge** (`optimize` with magnitude):
/// Runs the same iterative loop but adds a second acceptance path.
/// When a pair passes the angle, endpoint distance, and perpendicular
/// distance checks but the parallel gap exceeds `parallel_error`, the
/// pair can still be merged if gradient evidence along the connecting
/// path is strong enough (average magnitude >= `threshold`).  This
/// allows the optimizer to bridge larger gaps without giving up the
/// geometric safeguards that prevent false connections.
///
/// @tparam FT Floating-point type (float or double).
template <class FT>
class LineContinuityOptimizer : public ValueManager {
 public:
  using LineSegmentType = LineSegment<FT>;
  using LineSegmentVector = std::vector<LineSegmentType>;

  /// @brief Construct with default parameters.
  LineContinuityOptimizer()
      : ValueManager(),
        max_dist_(20),
        angle_error_(5),
        distance_error_(3),
        parallel_error_(10),
        merge_type_(ContinuityMergeType::STANDARD),
        accuracy_(2),
        threshold_(10) {
    init();
  }

  /// @brief Construct with merge-only parameters.
  /// @param max_dist Maximum endpoint distance for merge candidates.
  /// @param angle_error Maximum angle difference in degrees.
  /// @param distance_error Maximum perpendicular distance.
  /// @param parallel_error Maximum parallel gap.
  /// @param mt Merge strategy (STANDARD or AVG).
  LineContinuityOptimizer(FT max_dist,
                          FT angle_error,
                          FT distance_error,
                          FT parallel_error,
                          ContinuityMergeType mt = ContinuityMergeType::STANDARD)
      : ValueManager(),
        max_dist_(max_dist),
        angle_error_(angle_error),
        distance_error_(distance_error),
        parallel_error_(parallel_error),
        merge_type_(mt),
        accuracy_(2),
        threshold_(10) {
    init();
  }

  /// @brief Construct with full parameters (merge + gradient).
  /// @param max_dist Maximum endpoint distance.
  /// @param angle_error Maximum angle difference in degrees.
  /// @param distance_error Maximum perpendicular distance.
  /// @param parallel_error Maximum parallel gap (geometry-only path).
  /// @param mt Merge strategy.
  /// @param accuracy Sampling step along the gradient path (pixels).
  /// @param threshold Minimum average gradient for gradient-assisted merge.
  LineContinuityOptimizer(FT max_dist,
                          FT angle_error,
                          FT distance_error,
                          FT parallel_error,
                          ContinuityMergeType mt,
                          FT accuracy,
                          FT threshold)
      : ValueManager(),
        max_dist_(max_dist),
        angle_error_(angle_error),
        distance_error_(distance_error),
        parallel_error_(parallel_error),
        merge_type_(mt),
        accuracy_(accuracy),
        threshold_(threshold) {
    init();
  }

  /// @brief Construct from initializer list of named parameters.
  explicit LineContinuityOptimizer(InitializerList il) : LineContinuityOptimizer() { value(il); }

  /// @brief Construct from vector of named parameters.
  explicit LineContinuityOptimizer(const NameValueVector& nv) : LineContinuityOptimizer() { value(nv); }

  virtual ~LineContinuityOptimizer() = default;

  // -----------------------------------------------------------------------
  // Variant 1: geometry-only merge
  // -----------------------------------------------------------------------

  /// @brief Merge line segments using geometry constraints only.
  ///
  /// Iteratively merges pairs of near-collinear line segments until no
  /// more candidates are found. No gradient data is used.
  /// @param input Input line segments (unchanged).
  /// @param output Output merged line segments.
  void optimize(const LineSegmentVector& input, LineSegmentVector& output) const { run_loop(input, output, cv::Mat()); }

  // -----------------------------------------------------------------------
  // Variant 2: gradient-assisted merge
  // -----------------------------------------------------------------------

  /// @brief Merge line segments with optional gradient-assisted bridging.
  ///
  /// Same geometric loop as variant 1, but pairs that pass angle,
  /// endpoint distance, and perpendicular distance checks — yet have a
  /// parallel gap exceeding `parallel_error` — may still be merged when
  /// gradient evidence along the connecting path meets `threshold`.
  /// @param input Input line segments (unchanged).
  /// @param output Output merged line segments.
  /// @param magnitude Gradient magnitude image.
  void optimize(const LineSegmentVector& input, LineSegmentVector& output, const cv::Mat& magnitude) const {
    run_loop(input, output, magnitude);
  }

  /// @brief Gradient-assisted merge with automatic magnitude computation.
  ///
  /// If `magnitude` is empty it is computed from `src` via Sobel
  /// gradients and returned for potential reuse.
  /// @param input Input line segments.
  /// @param output Output merged line segments.
  /// @param src Source grayscale image.
  /// @param magnitude [in/out] Gradient magnitude image.
  void optimize(const LineSegmentVector& input,
                LineSegmentVector& output,
                const cv::Mat& src,
                cv::Mat& magnitude) const {
    if (magnitude.empty()) {
      cv::Mat grad_x;
      cv::Mat grad_y;
      cv::Sobel(src, grad_x, CV_64F, 1, 0);
      cv::Sobel(src, grad_y, CV_64F, 0, 1);
      cv::magnitude(grad_x, grad_y, magnitude);
    }
    run_loop(input, output, magnitude);
  }

 private:
  // -- Merge parameters --
  FT max_dist_;                     ///< Maximum endpoint distance.
  FT angle_error_;                  ///< Maximum angle difference (degrees).
  FT distance_error_;               ///< Maximum perpendicular distance.
  FT parallel_error_;               ///< Maximum parallel gap (geometry path).
  ContinuityMergeType merge_type_;  ///< Merge strategy.

  // -- Gradient parameters --
  FT accuracy_;   ///< Sampling step along gradient path (pixels).
  FT threshold_;  ///< Minimum average gradient for acceptance.

  /// @brief Register parameters with ValueManager.
  void init() {
    this->add(
        "max_dist",
        [this](const Value& v) -> Value {
          if (v.type()) max_dist_ = static_cast<FT>(v.getDouble());
          return Value(static_cast<double>(max_dist_));
        },
        "Maximum endpoint distance for merge candidates");
    this->add(
        "angle_error",
        [this](const Value& v) -> Value {
          if (v.type()) angle_error_ = static_cast<FT>(v.getDouble());
          return Value(static_cast<double>(angle_error_));
        },
        "Maximum angle difference in degrees");
    this->add(
        "distance_error",
        [this](const Value& v) -> Value {
          if (v.type()) distance_error_ = static_cast<FT>(v.getDouble());
          return Value(static_cast<double>(distance_error_));
        },
        "Maximum perpendicular distance");
    this->add(
        "parallel_error",
        [this](const Value& v) -> Value {
          if (v.type()) parallel_error_ = static_cast<FT>(v.getDouble());
          return Value(static_cast<double>(parallel_error_));
        },
        "Maximum parallel gap (geometry-only path)");
    this->add(
        "merge_type",
        [this](const Value& v) -> Value {
          if (v.type()) merge_type_ = static_cast<ContinuityMergeType>(v.getInt());
          return Value(static_cast<int>(merge_type_));
        },
        "Merge type: 0 = STANDARD, 1 = AVG");
    this->add(
        "accuracy",
        [this](const Value& v) -> Value {
          if (v.type()) accuracy_ = static_cast<FT>(v.getDouble());
          return Value(static_cast<double>(accuracy_));
        },
        "Sampling step along the gradient path (pixels)");
    this->add(
        "threshold",
        [this](const Value& v) -> Value {
          if (v.type()) threshold_ = static_cast<FT>(v.getDouble());
          return Value(static_cast<double>(threshold_));
        },
        "Minimum average gradient magnitude for gradient-assisted merge");
  }

  // =====================================================================
  // Core loop — shared between both variants
  // =====================================================================

  /// @brief Run the iterative merge loop.
  ///
  /// When `magnitude` is empty, only the pure geometric path is used.
  /// When `magnitude` is provided, the gradient-assisted path is added.
  void run_loop(const LineSegmentVector& input, LineSegmentVector& output, const cv::Mat& magnitude) const {
    output.clear();
    if (input.empty()) return;

    const bool use_gradient = !magnitude.empty();
    std::vector<bool> used(input.size(), false);
    LineSegmentVector working(input);

    bool merged_any = true;
    while (merged_any) {
      merged_any = false;
      for (std::size_t i = 0; i < working.size(); ++i) {
        if (used[i]) continue;
        for (std::size_t j = i + 1; j < working.size(); ++j) {
          if (used[j]) continue;
          if (can_merge(working[i], working[j], magnitude, use_gradient)) {
            working[i] = do_merge(working[i], working[j]);
            used[j] = true;
            merged_any = true;
          }
        }
      }
      // Compact and restart if anything was merged
      if (merged_any) {
        LineSegmentVector next;
        next.reserve(working.size());
        for (std::size_t i = 0; i < working.size(); ++i) {
          if (!used[i]) {
            next.push_back(working[i]);
          }
        }
        working = std::move(next);
        used.assign(working.size(), false);
      }
    }
    output = std::move(working);
  }

  // =====================================================================
  // Geometric checks
  // =====================================================================

  /// @brief Decide whether two segments should be merged.
  ///
  /// The decision follows a two-path logic:
  /// 1. **Geometry path** — all four checks pass (angle, endpoint dist,
  ///    perpendicular dist, parallel gap).  Always tried first.
  /// 2. **Gradient path** — angle, endpoint dist, and perpendicular dist
  ///    pass, but parallel gap is too large.  Accepted only when
  ///    `use_gradient` is true and the average gradient along the best
  ///    connecting path meets `threshold_`.
  bool can_merge(const LineSegmentType& a,
                 const LineSegmentType& b,
                 const cv::Mat& magnitude,
                 bool use_gradient) const {
    using point_type = typename LineSegmentType::point_type;

    // --- Shared check 1: angle ---
    point_type as = a.startPoint(), ae = a.endPoint();
    point_type bs = b.startPoint(), be = b.endPoint();
    FT angle_a = std::atan2(getY(ae) - getY(as), getX(ae) - getX(as));
    FT angle_b = std::atan2(getY(be) - getY(bs), getX(be) - getX(bs));
    FT angle_diff = std::abs(angle_a - angle_b);
    if (angle_diff > static_cast<FT>(CV_PI)) {
      angle_diff = static_cast<FT>(2 * CV_PI) - angle_diff;
    }
    if (angle_diff > static_cast<FT>(CV_PI / 2)) {
      angle_diff = static_cast<FT>(CV_PI) - angle_diff;
    }
    FT angle_deg = angle_diff * static_cast<FT>(180.0 / CV_PI);
    if (angle_deg > angle_error_) return false;

    // --- Shared check 2: endpoint distance ---
    FT min_dist = min_endpoint_distance(a, b);
    if (min_dist > max_dist_) return false;

    // --- Shared check 3: perpendicular distance ---
    FT perp_dist = perpendicular_distance(a, b);
    if (perp_dist > distance_error_) return false;

    // --- Check 4: parallel gap ---
    FT par_gap = parallel_gap(a, b);
    if (par_gap <= parallel_error_) {
      // Geometry path: all four checks pass → merge.
      return true;
    }

    // Parallel gap too large for pure geometry.
    // Try gradient path if enabled.
    if (!use_gradient) return false;

    // --- Gradient-assisted path ---
    FT score = best_gradient_score(a, b, magnitude);
    return score >= threshold_;
  }

  // =====================================================================
  // Distance helpers
  // =====================================================================

  /// @brief Minimum distance between any pair of endpoints.
  static FT min_endpoint_distance(const LineSegmentType& a, const LineSegmentType& b) {
    using point_type = typename LineSegmentType::point_type;
    auto dist = [](const point_type& p, const point_type& q) -> FT {
      FT dx = getX(p) - getX(q);
      FT dy = getY(p) - getY(q);
      return std::sqrt(dx * dx + dy * dy);
    };
    FT d1 = dist(a.startPoint(), b.startPoint());
    FT d2 = dist(a.startPoint(), b.endPoint());
    FT d3 = dist(a.endPoint(), b.startPoint());
    FT d4 = dist(a.endPoint(), b.endPoint());
    return std::min({d1, d2, d3, d4});
  }

  /// @brief Perpendicular distance (midpoint of b onto line through a).
  static FT perpendicular_distance(const LineSegmentType& a, const LineSegmentType& b) {
    auto bs = b.startPoint();
    auto be = b.endPoint();
    FT mid_x = (getX(bs) + getX(be)) / 2;
    FT mid_y = (getY(bs) + getY(be)) / 2;
    auto as = a.startPoint();
    auto ae = a.endPoint();
    FT dx = getX(ae) - getX(as);
    FT dy = getY(ae) - getY(as);
    FT len = std::sqrt(dx * dx + dy * dy);
    if (len < static_cast<FT>(1e-10)) return static_cast<FT>(0);
    return std::abs(dy * (mid_x - getX(as)) - dx * (mid_y - getY(as))) / len;
  }

  /// @brief Parallel gap between two segments along a's direction.
  static FT parallel_gap(const LineSegmentType& a, const LineSegmentType& b) {
    auto as = a.startPoint();
    auto ae = a.endPoint();
    FT dx = getX(ae) - getX(as);
    FT dy = getY(ae) - getY(as);
    FT len = std::sqrt(dx * dx + dy * dy);
    if (len < static_cast<FT>(1e-10)) return static_cast<FT>(0);
    FT nx = dx / len;
    FT ny = dy / len;

    using point_type = typename LineSegmentType::point_type;
    auto proj = [&](const point_type& p) -> FT { return (getX(p) - getX(as)) * nx + (getY(p) - getY(as)) * ny; };

    FT pa1 = proj(a.startPoint());
    FT pa2 = proj(a.endPoint());
    FT pb1 = proj(b.startPoint());
    FT pb2 = proj(b.endPoint());

    FT a_min = std::min(pa1, pa2);
    FT a_max = std::max(pa1, pa2);
    FT b_min = std::min(pb1, pb2);
    FT b_max = std::max(pb1, pb2);

    FT gap = std::max(a_min - b_max, b_min - a_max);
    return std::max(gap, static_cast<FT>(0));
  }

  // =====================================================================
  // Gradient sampling
  // =====================================================================

  /// @brief Best average gradient magnitude among the four endpoint
  ///        configurations that are within `max_dist_`.
  FT best_gradient_score(const LineSegmentType& a, const LineSegmentType& b, const cv::Mat& magnitude) const {
    using point_type = typename LineSegmentType::point_type;

    struct PointPair {
      point_type first;
      point_type second;
    };
    std::array<PointPair, 4> configs = {{
        {a.endPoint(), b.startPoint()},
        {a.endPoint(), b.endPoint()},
        {a.startPoint(), b.startPoint()},
        {a.startPoint(), b.endPoint()},
    }};

    FT best = 0;
    for (int c = 0; c < 4; ++c) {
      const auto& pp = configs[static_cast<std::size_t>(c)];
      FT dx = getX(pp.second) - getX(pp.first);
      FT dy = getY(pp.second) - getY(pp.first);
      FT dist = std::sqrt(dx * dx + dy * dy);
      if (dist > max_dist_ || dist < static_cast<FT>(1e-6)) continue;
      FT score = sample_path(pp.first, pp.second, magnitude, dist);
      if (score > best) {
        best = score;
      }
    }
    return best;
  }

  /// @brief Sample gradient magnitude along a straight path.
  /// @param from Start point.
  /// @param to End point.
  /// @param magnitude Gradient magnitude image.
  /// @param dist Pre-computed distance between from and to.
  /// @return Average gradient magnitude along the path.
  FT sample_path(const typename LineSegmentType::point_type& from,
                 const typename LineSegmentType::point_type& to,
                 const cv::Mat& magnitude,
                 FT dist) const {
    FT step = std::max(accuracy_, static_cast<FT>(0.5));
    int n_samples = std::max(2, static_cast<int>(dist / step));
    FT sum = 0;
    int valid = 0;

    for (int s = 0; s <= n_samples; ++s) {
      FT t = static_cast<FT>(s) / static_cast<FT>(n_samples);
      FT x = getX(from) + t * (getX(to) - getX(from));
      FT y = getY(from) + t * (getY(to) - getY(from));

      int ix = static_cast<int>(std::round(static_cast<double>(x)));
      int iy = static_cast<int>(std::round(static_cast<double>(y)));

      if (ix >= 0 && ix < magnitude.cols && iy >= 0 && iy < magnitude.rows) {
        switch (magnitude.depth()) {
          case CV_8U:
            sum += static_cast<FT>(magnitude.at<uchar>(iy, ix));
            break;
          case CV_16U:
            sum += static_cast<FT>(magnitude.at<ushort>(iy, ix));
            break;
          case CV_16S:
            sum += static_cast<FT>(magnitude.at<short>(iy, ix));
            break;
          case CV_32S:
            sum += static_cast<FT>(magnitude.at<int>(iy, ix));
            break;
          case CV_32F:
            sum += static_cast<FT>(magnitude.at<float>(iy, ix));
            break;
          case CV_64F:
          default:
            sum += static_cast<FT>(magnitude.at<double>(iy, ix));
            break;
        }
        ++valid;
      }
    }
    return valid > 0 ? sum / static_cast<FT>(valid) : static_cast<FT>(0);
  }

  // =====================================================================
  // Merge execution
  // =====================================================================

  /// @brief Merge two segments into one.
  LineSegmentType do_merge(const LineSegmentType& a, const LineSegmentType& b) const {
    using point_type = typename LineSegmentType::point_type;

    if (merge_type_ == ContinuityMergeType::AVG) {
      auto as = a.startPoint(), ae = a.endPoint();
      auto bs = b.startPoint(), be = b.endPoint();
      point_type s((getX(as) + getX(bs)) / 2, (getY(as) + getY(bs)) / 2);
      point_type e((getX(ae) + getX(be)) / 2, (getY(ae) + getY(be)) / 2);
      return LineSegmentType(s, e);
    }

    // STANDARD: use the two furthest endpoints
    std::array<point_type, 4> pts = {{a.startPoint(), a.endPoint(), b.startPoint(), b.endPoint()}};

    FT max_d = 0;
    std::size_t pi = 0;
    std::size_t pj = 1;
    for (std::size_t i = 0; i < pts.size(); ++i) {
      for (std::size_t j = i + 1; j < pts.size(); ++j) {
        FT ddx = getX(pts[i]) - getX(pts[j]);
        FT ddy = getY(pts[i]) - getY(pts[j]);
        FT d = ddx * ddx + ddy * ddy;
        if (d > max_d) {
          max_d = d;
          pi = i;
          pj = j;
        }
      }
    }
    return LineSegmentType(pts[pi], pts[pj]);
  }
};

}  // namespace lsfm
