//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file line_connect.hpp
/// @brief Connect nearby line segments using gradient magnitude.

#pragma once

#include <geometry/line.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <utility/value_manager.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace lsfm {

/// @brief Connects nearby line segments using gradient magnitude evidence.
///
/// Given a list of line segments and a gradient magnitude image, finds
/// pairs of nearby segment endpoints and connects them if the gradient
/// response along the connecting path is strong enough.
///
/// @tparam FT Floating-point type (float or double).
template <class FT>
class LineConnect : public ValueManager {
 public:
  using LineSegmentType = LineSegment<FT>;
  using LineSegmentVector = std::vector<LineSegmentType>;

  /// @brief Construct with default parameters.
  LineConnect() : ValueManager(), max_radius_(15), accuracy_(2), threshold_(10) { init(); }

  /// @brief Construct with explicit parameters.
  /// @param max_radius Maximum distance between endpoints to consider connection.
  /// @param accuracy Sampling step along the connecting path.
  /// @param threshold Minimum average gradient magnitude along the connection.
  LineConnect(FT max_radius, FT accuracy, FT threshold)
      : ValueManager(), max_radius_(max_radius), accuracy_(accuracy), threshold_(threshold) {
    init();
  }

  /// @brief Construct from initializer list of named parameters.
  /// @param il Initializer list of NameValuePair entries.
  explicit LineConnect(InitializerList il) : LineConnect() { value(il); }

  /// @brief Construct from vector of named parameters.
  /// @param nv Vector of NameValuePair entries.
  explicit LineConnect(const NameValueVector& nv) : LineConnect() { value(nv); }

  virtual ~LineConnect() = default;

  /// @brief Connect line segments using the provided magnitude map.
  /// @param input Input line segments.
  /// @param output Output connected line segments.
  /// @param magnitude Gradient magnitude image (CV_32F or CV_64F).
  void connect_lines(const LineSegmentVector& input, LineSegmentVector& output, const cv::Mat& magnitude) const {
    output.clear();
    if (input.empty()) return;

    std::vector<bool> used(input.size(), false);
    LineSegmentVector working(input);

    bool connected_any = true;
    while (connected_any) {
      connected_any = false;
      for (std::size_t i = 0; i < working.size(); ++i) {
        if (used[i]) continue;

        FT best_score = static_cast<FT>(0);
        std::size_t best_j = 0;
        int best_config = 0;  // Which endpoints to join

        for (std::size_t j = i + 1; j < working.size(); ++j) {
          if (used[j]) continue;

          int config = 0;
          FT score = evaluate_connection(working[i], working[j], magnitude, config);
          if (score > best_score) {
            best_score = score;
            best_j = j;
            best_config = config;
          }
        }

        if (best_score > threshold_) {
          working[i] = join_segments(working[i], working[best_j], best_config);
          used[best_j] = true;
          connected_any = true;
        }
      }

      if (connected_any) {
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

  /// @brief Connect line segments, computing magnitude from source image.
  /// @param input Input line segments.
  /// @param output Output connected line segments.
  /// @param src Source grayscale image.
  /// @param magnitude Output gradient magnitude image.
  void connect_lines(const LineSegmentVector& input,
                     LineSegmentVector& output,
                     const cv::Mat& src,
                     cv::Mat& magnitude) const {
    if (magnitude.empty()) {
      cv::Mat grad_x, grad_y;
      cv::Sobel(src, grad_x, CV_64F, 1, 0);
      cv::Sobel(src, grad_y, CV_64F, 0, 1);
      cv::magnitude(grad_x, grad_y, magnitude);
    }
    connect_lines(input, output, magnitude);
  }

 private:
  FT max_radius_;  ///< Maximum endpoint distance for connection candidates.
  FT accuracy_;    ///< Sampling step along the connecting path.
  FT threshold_;   ///< Minimum average gradient magnitude for connection.

  /// @brief Register parameters with ValueManager.
  void init() {
    this->add(
        "max_radius",
        [this](const Value& v) -> Value {
          if (v.type()) max_radius_ = static_cast<FT>(v.getDouble());
          return Value(static_cast<double>(max_radius_));
        },
        "Maximum endpoint distance for connection candidates");
    this->add(
        "accuracy",
        [this](const Value& v) -> Value {
          if (v.type()) accuracy_ = static_cast<FT>(v.getDouble());
          return Value(static_cast<double>(accuracy_));
        },
        "Sampling step along the connecting path");
    this->add(
        "threshold",
        [this](const Value& v) -> Value {
          if (v.type()) threshold_ = static_cast<FT>(v.getDouble());
          return Value(static_cast<double>(threshold_));
        },
        "Minimum average gradient magnitude for connection");
  }

  /// @brief Evaluate the gradient response along a potential connection.
  ///
  /// Tests all four endpoint combinations (start-start, start-end,
  /// end-start, end-end) and returns the best average gradient magnitude
  /// along the connecting path. Sets `config` to indicate which
  /// configuration was best.
  /// @param a First segment.
  /// @param b Second segment.
  /// @param magnitude Gradient magnitude image.
  /// @param[out] config Best endpoint configuration (0-3).
  /// @return Best average gradient response, or 0 if distance exceeds max_radius.
  FT evaluate_connection(const LineSegmentType& a,
                         const LineSegmentType& b,
                         const cv::Mat& magnitude,
                         int& config) const {
    using point_type = typename LineSegmentType::point_type;
    struct PointPair {
      point_type first;
      point_type second;
    };
    std::array<PointPair, 4> configs = {{{a.endPoint(), b.startPoint()},
                                         {a.endPoint(), b.endPoint()},
                                         {a.startPoint(), b.startPoint()},
                                         {a.startPoint(), b.endPoint()}}};

    FT best = 0;
    config = 0;

    for (int c = 0; c < 4; ++c) {
      const auto& pp = configs[static_cast<std::size_t>(c)];
      FT dx = getX(pp.second) - getX(pp.first);
      FT dy = getY(pp.second) - getY(pp.first);
      FT dist = std::sqrt(dx * dx + dy * dy);
      if (dist > max_radius_ || dist < static_cast<FT>(1e-6)) continue;

      FT score = sample_path(pp.first, pp.second, magnitude, dist);
      if (score > best) {
        best = score;
        config = c;
      }
    }
    return best;
  }

  /// @brief Sample gradient magnitude along a straight path between two points.
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
        if (magnitude.type() == CV_32F) {
          sum += static_cast<FT>(magnitude.at<float>(iy, ix));
        } else if (magnitude.type() == CV_64F) {
          sum += static_cast<FT>(magnitude.at<double>(iy, ix));
        } else {
          sum += static_cast<FT>(magnitude.at<uchar>(iy, ix));
        }
        ++valid;
      }
    }
    return valid > 0 ? sum / static_cast<FT>(valid) : static_cast<FT>(0);
  }

  /// @brief Join two segments at their best endpoint configuration.
  /// @param a First segment.
  /// @param b Second segment.
  /// @param config Endpoint join configuration (0-3).
  /// @return New segment spanning from one far endpoint to the other.
  static LineSegmentType join_segments(const LineSegmentType& a, const LineSegmentType& b, int config) {
    switch (config) {
      case 0:  // a.end -> b.start: a.start to b.end
        return LineSegmentType(a.startPoint(), b.endPoint());
      case 1:  // a.end -> b.end: a.start to b.start
        return LineSegmentType(a.startPoint(), b.startPoint());
      case 2:  // a.start -> b.start: a.end to b.end
        return LineSegmentType(a.endPoint(), b.endPoint());
      case 3:  // a.start -> b.end: a.end to b.start
        return LineSegmentType(a.endPoint(), b.startPoint());
      default:
        return a;
    }
  }
};

}  // namespace lsfm
