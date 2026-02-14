//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file line_merge.hpp
/// @brief Merge collinear or near-collinear line segments.

#pragma once

#include <geometry/line.hpp>
#include <utility/value_manager.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace lsfm {

/// @brief Merge strategy enumeration.
enum class MergeType : int {
  STANDARD = 0,  ///< Merge using furthest endpoints of the pair.
  AVG = 1        ///< Merge by averaging the endpoint positions.
};

/// @brief Merges collinear or near-collinear line segments into longer ones.
///
/// Two line segments are candidates for merging when:
/// 1. Their endpoint distance is below `max_dist`.
/// 2. Their relative angle is below `angle_error` (degrees).
/// 3. Their perpendicular distance is below `distance_error`.
/// 4. Their parallel gap is below `parallel_error`.
///
/// @tparam FT Floating-point type (float or double).
template <class FT>
class LineMerge : public ValueManager {
 public:
  using LineSegmentType = LineSegment<FT>;
  using LineSegmentVector = std::vector<LineSegmentType>;

  /// @brief Construct with default parameters.
  LineMerge()
      : ValueManager(),
        max_dist_(20),
        angle_error_(5),
        distance_error_(3),
        parallel_error_(10),
        merge_type_(MergeType::STANDARD) {
    init();
  }

  /// @brief Construct with explicit parameters.
  /// @param max_dist Maximum endpoint distance for merge candidates.
  /// @param angle_error Maximum angle difference in degrees.
  /// @param distance_error Maximum perpendicular distance.
  /// @param parallel_error Maximum parallel gap.
  /// @param mt Merge strategy (STANDARD or AVG).
  LineMerge(FT max_dist, FT angle_error, FT distance_error, FT parallel_error, MergeType mt = MergeType::STANDARD)
      : ValueManager(),
        max_dist_(max_dist),
        angle_error_(angle_error),
        distance_error_(distance_error),
        parallel_error_(parallel_error),
        merge_type_(mt) {
    init();
  }

  /// @brief Construct from initializer list of named parameters.
  /// @param il Initializer list of NameValuePair entries.
  explicit LineMerge(InitializerList il) : LineMerge() { value(il); }

  /// @brief Construct from vector of named parameters.
  /// @param nv Vector of NameValuePair entries.
  explicit LineMerge(const NameValueVector& nv) : LineMerge() { value(nv); }

  virtual ~LineMerge() = default;

  /// @brief Merge line segments from input into output.
  ///
  /// Iteratively merges pairs of near-collinear line segments until no
  /// more candidates are found.
  /// @param input Input line segments (unchanged).
  /// @param output Output merged line segments.
  void merge_lines(const LineSegmentVector& input, LineSegmentVector& output) const {
    output.clear();
    if (input.empty()) return;

    // Working copy with "used" flags
    std::vector<bool> used(input.size(), false);
    LineSegmentVector working(input);

    bool merged_any = true;
    while (merged_any) {
      merged_any = false;
      for (std::size_t i = 0; i < working.size(); ++i) {
        if (used[i]) continue;
        for (std::size_t j = i + 1; j < working.size(); ++j) {
          if (used[j]) continue;
          if (can_merge(working[i], working[j])) {
            working[i] = do_merge(working[i], working[j]);
            used[j] = true;
            merged_any = true;
          }
        }
      }
      // Compact and restart if we merged anything
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

 private:
  FT max_dist_;           ///< Maximum endpoint distance for merge candidates.
  FT angle_error_;        ///< Maximum angle difference in degrees.
  FT distance_error_;     ///< Maximum perpendicular distance.
  FT parallel_error_;     ///< Maximum parallel gap.
  MergeType merge_type_;  ///< Merge strategy.

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
        "Maximum parallel gap");
    this->add(
        "merge_type",
        [this](const Value& v) -> Value {
          if (v.type()) merge_type_ = static_cast<MergeType>(v.getInt());
          return Value(static_cast<int>(merge_type_));
        },
        "Merge type: 0 = STANDARD, 1 = AVG");
  }

  /// @brief Check whether two line segments are close and collinear enough to merge.
  /// @param a First line segment.
  /// @param b Second line segment.
  /// @return True if the segments should be merged.
  bool can_merge(const LineSegmentType& a, const LineSegmentType& b) const {
    using point_type = typename LineSegmentType::point_type;

    // Angle check (in degrees)
    point_type as = a.startPoint(), ae = a.endPoint();
    point_type bs = b.startPoint(), be = b.endPoint();
    FT angle_a = std::atan2(getY(ae) - getY(as), getX(ae) - getX(as));
    FT angle_b = std::atan2(getY(be) - getY(bs), getX(be) - getX(bs));
    FT angle_diff = std::abs(angle_a - angle_b);
    if (angle_diff > static_cast<FT>(CV_PI)) {
      angle_diff = static_cast<FT>(2 * CV_PI) - angle_diff;
    }
    // Normalize to 0..PI range (line segments have no direction)
    if (angle_diff > static_cast<FT>(CV_PI / 2)) {
      angle_diff = static_cast<FT>(CV_PI) - angle_diff;
    }
    FT angle_deg = angle_diff * static_cast<FT>(180.0 / CV_PI);
    if (angle_deg > angle_error_) return false;

    // Find closest endpoint pair
    FT min_dist = min_endpoint_distance(a, b);
    if (min_dist > max_dist_) return false;

    // Perpendicular distance: project midpoint of b onto line through a
    FT perp_dist = perpendicular_distance(a, b);
    if (perp_dist > distance_error_) return false;

    // Parallel gap: distance along a's direction between the segments
    FT par_gap = parallel_gap(a, b);
    if (par_gap > parallel_error_) return false;

    return true;
  }

  /// @brief Compute the minimum distance between any pair of endpoints.
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

  /// @brief Compute perpendicular distance between two segments.
  ///
  /// Projects the midpoint of b onto line through a and returns
  /// the perpendicular distance.
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

  /// @brief Compute the parallel gap between two segments.
  ///
  /// Projects all four endpoints onto the direction of segment a
  /// and checks whether the projections overlap or have a gap.
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

    // Gap is positive if segments don't overlap along the direction
    FT gap = std::max(a_min - b_max, b_min - a_max);
    return std::max(gap, static_cast<FT>(0));
  }

  /// @brief Merge two segments into one.
  /// @param a First line segment.
  /// @param b Second line segment.
  /// @return Merged line segment.
  LineSegmentType do_merge(const LineSegmentType& a, const LineSegmentType& b) const {
    using point_type = typename LineSegmentType::point_type;

    if (merge_type_ == MergeType::AVG) {
      // Average the endpoints
      auto as = a.startPoint(), ae = a.endPoint();
      auto bs = b.startPoint(), be = b.endPoint();
      point_type s((getX(as) + getX(bs)) / 2, (getY(as) + getY(bs)) / 2);
      point_type e((getX(ae) + getX(be)) / 2, (getY(ae) + getY(be)) / 2);
      return LineSegmentType(s, e);
    }

    // STANDARD: use the two furthest endpoints
    std::array<point_type, 4> pts = {{a.startPoint(), a.endPoint(), b.startPoint(), b.endPoint()}};

    FT max_d = 0;
    std::size_t pi = 0, pj = 1;
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
