//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file precision_optimize.hpp
/// @brief Headless line precision optimization wrapper.
///
/// Provides a GUI-independent interface to the LineOptimizer from
/// libs/geometry/, encapsulating the search/stop strategy selection
/// and parameter configuration. Designed for use from both C++ and Python.

#pragma once

#include <dlib/optimization.h>
#include <geometry/line.hpp>
#include <geometry/line_optimizer.hpp>
#include <imgproc/mean.hpp>
#include <opencv2/core.hpp>
#include <utility/value_manager.hpp>

#include <functional>
#include <string>
#include <vector>

namespace lsfm {

/// @brief Search strategy enumeration for precision optimization.
enum class PrecisionSearchStrategy : int {
  BFGS = 0,   ///< BFGS quasi-Newton method.
  LBFGS = 1,  ///< Limited-memory BFGS.
  CG = 2      ///< Conjugate gradient.
};

/// @brief Stop strategy enumeration for precision optimization.
enum class PrecisionStopStrategy : int {
  DELTA = 0,     ///< Stop when objective delta < threshold.
  GRAD_NORM = 1  ///< Stop when gradient norm < threshold.
};

/// @brief Interpolation mode for gradient sampling.
enum class InterpolationMode : int {
  NEAREST = 0,        ///< Nearest neighbor.
  NEAREST_ROUND = 1,  ///< Fast rounded nearest neighbor.
  BILINEAR = 2,       ///< Bilinear interpolation.
  BICUBIC = 3         ///< Bicubic interpolation.
};

/// @brief Headless precision optimizer for line segments.
///
/// Wraps the LineOptimizer from libs/geometry/ with a clean interface
/// that encapsulates the strategy selection boilerplate. All parameters
/// are accessible via ValueManager for programmatic configuration.
///
/// @code{cpp}
/// PrecisionOptimize optimizer;
/// optimizer.value("search_strategy", 0);  // BFGS
/// optimizer.value("stop_strategy", 0);    // Delta
///
/// auto result = optimizer.optimize_all(magnitude, line_segments);
/// @endcode
class PrecisionOptimize : public ValueManager {
 public:
  using LineSegmentType = LineSegment<double>;
  using LineSegmentVector = std::vector<LineSegmentType>;
  using MeanFunc = MeanHelper<double, Vec2>::func_type;

  /// @brief Construct with default parameters.
  PrecisionOptimize()
      : ValueManager(),
        search_range_d_(1.0),
        search_range_r_(1.0),
        interpolation_(InterpolationMode::BILINEAR),
        search_strategy_(PrecisionSearchStrategy::BFGS),
        stop_strategy_(PrecisionStopStrategy::DELTA),
        stop_delta_(1e-7),
        max_iterations_(0),
        derivative_precision_(1e-7),
        mean_param_(1.0),
        use_sampled_(false),
        use_fast_(false) {
    init();
  }

  /// @brief Construct from initializer list.
  /// @param il Initializer list of named parameters.
  explicit PrecisionOptimize(InitializerList il) : PrecisionOptimize() { value(il); }

  /// @brief Construct from named value vector.
  /// @param nv Named value vector.
  explicit PrecisionOptimize(const NameValueVector& nv) : PrecisionOptimize() { value(nv); }

  virtual ~PrecisionOptimize() = default;

  /// @brief Optimize a single line segment.
  /// @param magnitude Gradient magnitude image.
  /// @param line Line segment to optimize (modified in place).
  /// @return Optimization error value (negative of maximized mean).
  double optimize_line(const cv::Mat& magnitude, LineSegmentType& line) const {
    double d = 0, r = 0;
    double d_lower = -search_range_d_;
    double d_upper = search_range_d_;
    double r_lower = -search_range_r_ * CV_PI / 180.0;
    double r_upper = search_range_r_ * CV_PI / 180.0;

    double result = dispatch_optimize(magnitude, line, d, r, d_lower, d_upper, r_lower, r_upper);
    line.translateOrtho(d);
    line.rotate(r, line.center());
    return result;
  }

  /// @brief Optimize all line segments in a vector.
  /// @param magnitude Gradient magnitude image.
  /// @param lines Line segments to optimize (modified in place).
  /// @return Vector of optimization error values.
  std::vector<double> optimize_all(const cv::Mat& magnitude, LineSegmentVector& lines) const {
    std::vector<double> errors(lines.size());
    for (std::size_t i = 0; i < lines.size(); ++i) {
      errors[i] = optimize_line(magnitude, lines[i]);
    }
    return errors;
  }

  /// @brief Optimize line segments to new output vector.
  /// @param magnitude Gradient magnitude image.
  /// @param input Input line segments (unchanged).
  /// @param output Output optimized line segments.
  /// @return Vector of optimization error values.
  std::vector<double> optimize_copy(const cv::Mat& magnitude,
                                    const LineSegmentVector& input,
                                    LineSegmentVector& output) const {
    output = input;
    return optimize_all(magnitude, output);
  }

 private:
  double search_range_d_;                    ///< Orthogonal distance search range (pixels).
  double search_range_r_;                    ///< Rotation search range (degrees).
  InterpolationMode interpolation_;          ///< Interpolation mode for gradient sampling.
  PrecisionSearchStrategy search_strategy_;  ///< Dlib search strategy.
  PrecisionStopStrategy stop_strategy_;      ///< Dlib stop strategy.
  double stop_delta_;                        ///< Stop criterion threshold.
  int max_iterations_;                       ///< Max iterations (0 = unlimited).
  double derivative_precision_;              ///< Numerical derivative delta.
  double mean_param_;                        ///< Mean calculation parameter.
  bool use_sampled_;                         ///< Use sampled mean (vs. continuous).
  bool use_fast_;                            ///< Use fast interpolation.

  /// @brief Register parameters with ValueManager.
  void init() {
    this->add(
        "search_range_d",
        [this](const Value& v) -> Value {
          if (v.type()) search_range_d_ = v.getDouble();
          return Value(search_range_d_);
        },
        "Orthogonal distance search range in pixels");
    this->add(
        "search_range_r",
        [this](const Value& v) -> Value {
          if (v.type()) search_range_r_ = v.getDouble();
          return Value(search_range_r_);
        },
        "Rotation search range in degrees");
    this->add(
        "interpolation",
        [this](const Value& v) -> Value {
          if (v.type()) interpolation_ = static_cast<InterpolationMode>(v.getInt());
          return Value(static_cast<int>(interpolation_));
        },
        "Interpolation: 0=nearest, 1=nearest_round, 2=bilinear, 3=bicubic");
    this->add(
        "search_strategy",
        [this](const Value& v) -> Value {
          if (v.type()) search_strategy_ = static_cast<PrecisionSearchStrategy>(v.getInt());
          return Value(static_cast<int>(search_strategy_));
        },
        "Search strategy: 0=BFGS, 1=LBFGS, 2=CG");
    this->add(
        "stop_strategy",
        [this](const Value& v) -> Value {
          if (v.type()) stop_strategy_ = static_cast<PrecisionStopStrategy>(v.getInt());
          return Value(static_cast<int>(stop_strategy_));
        },
        "Stop strategy: 0=delta, 1=gradient_norm");
    this->add(
        "stop_delta",
        [this](const Value& v) -> Value {
          if (v.type()) stop_delta_ = v.getDouble();
          return Value(stop_delta_);
        },
        "Stop criterion threshold");
    this->add(
        "max_iterations",
        [this](const Value& v) -> Value {
          if (v.type()) max_iterations_ = v.getInt();
          return Value(max_iterations_);
        },
        "Maximum iterations (0 = unlimited)");
    this->add(
        "derivative_precision",
        [this](const Value& v) -> Value {
          if (v.type()) derivative_precision_ = v.getDouble();
          return Value(derivative_precision_);
        },
        "Numerical derivative delta");
    this->add(
        "mean_param",
        [this](const Value& v) -> Value {
          if (v.type()) mean_param_ = v.getDouble();
          return Value(mean_param_);
        },
        "Mean calculation parameter");
    this->add(
        "use_sampled",
        [this](const Value& v) -> Value {
          if (v.type()) use_sampled_ = v.getBool();
          return Value(use_sampled_);
        },
        "Use sampled mean calculation");
    this->add(
        "use_fast",
        [this](const Value& v) -> Value {
          if (v.type()) use_fast_ = v.getBool();
          return Value(use_fast_);
        },
        "Use fast interpolation");
  }

  /// @brief Get mean function for the current interpolation settings.
  /// @tparam MT Matrix element type.
  /// @return Mean computation function pointer.
  template <class MT>
  MeanFunc get_mean_func() const {
    // Select interpolation type and mean computation variant.
    // Explicit template argument <Vec2> and static_cast are required because
    // process() is a member function template and the ternary operator cannot
    // deduce a common type from different class function pointers.
    int idx = static_cast<int>(interpolation_);
    switch (idx) {
      case 0:
        if (use_sampled_) {
          if (use_fast_) {
            return static_cast<MeanFunc>(
                FastMeanSampled<double, MT, NearestInterpolator<double, MT>>::template process<Vec2>);
          }
          return static_cast<MeanFunc>(
              MeanSampled<double, MT, NearestInterpolator<double, MT>>::template process<Vec2>);
        }
        if (use_fast_) {
          return static_cast<MeanFunc>(FastMean<double, MT, NearestInterpolator<double, MT>>::template process<Vec2>);
        }
        return static_cast<MeanFunc>(Mean<double, MT, NearestInterpolator<double, MT>>::template process<Vec2>);
      case 1:
        if (use_sampled_) {
          if (use_fast_) {
            return static_cast<MeanFunc>(
                FastMeanSampled<double, MT, FastRoundNearestInterpolator<double, MT>>::template process<Vec2>);
          }
          return static_cast<MeanFunc>(
              MeanSampled<double, MT, FastRoundNearestInterpolator<double, MT>>::template process<Vec2>);
        }
        if (use_fast_) {
          return static_cast<MeanFunc>(
              FastMean<double, MT, FastRoundNearestInterpolator<double, MT>>::template process<Vec2>);
        }
        return static_cast<MeanFunc>(
            Mean<double, MT, FastRoundNearestInterpolator<double, MT>>::template process<Vec2>);
      case 3:
        if (use_sampled_) {
          if (use_fast_) {
            return static_cast<MeanFunc>(
                FastMeanSampled<double, MT, CubicInterpolator<double, MT>>::template process<Vec2>);
          }
          return static_cast<MeanFunc>(MeanSampled<double, MT, CubicInterpolator<double, MT>>::template process<Vec2>);
        }
        if (use_fast_) {
          return static_cast<MeanFunc>(FastMean<double, MT, CubicInterpolator<double, MT>>::template process<Vec2>);
        }
        return static_cast<MeanFunc>(Mean<double, MT, CubicInterpolator<double, MT>>::template process<Vec2>);
      case 2:
      default:
        if (use_sampled_) {
          if (use_fast_) {
            return static_cast<MeanFunc>(
                FastMeanSampled<double, MT, LinearInterpolator<double, MT>>::template process<Vec2>);
          }
          return static_cast<MeanFunc>(MeanSampled<double, MT, LinearInterpolator<double, MT>>::template process<Vec2>);
        }
        if (use_fast_) {
          return static_cast<MeanFunc>(FastMean<double, MT, LinearInterpolator<double, MT>>::template process<Vec2>);
        }
        return static_cast<MeanFunc>(Mean<double, MT, LinearInterpolator<double, MT>>::template process<Vec2>);
    }
  }

  /// @brief Dispatch optimization with the correct template parameters.
  double dispatch_optimize(const cv::Mat& mag,
                           const LineSegmentType& line,
                           double& d,
                           double& r,
                           double d_lower,
                           double d_upper,
                           double r_lower,
                           double r_upper) const {
    if (mag.type() == CV_32F) {
      return dispatch_search<float>(mag, line, d, r, d_lower, d_upper, r_lower, r_upper);
    }
    return dispatch_search<int>(mag, line, d, r, d_lower, d_upper, r_lower, r_upper);
  }

  /// @brief Dispatch based on search and stop strategies.
  template <class MT>
  double dispatch_search(const cv::Mat& mag,
                         const LineSegmentType& line,
                         double& d,
                         double& r,
                         double d_lower,
                         double d_upper,
                         double r_lower,
                         double r_upper) const {
    MeanFunc mean_op = get_mean_func<MT>();

    switch (search_strategy_) {
      case PrecisionSearchStrategy::LBFGS:
        return dispatch_stop<MT, dlib::lbfgs_search_strategy>(mag, line, d, r, d_lower, d_upper, r_lower, r_upper,
                                                              mean_op, dlib::lbfgs_search_strategy(10));
      case PrecisionSearchStrategy::CG:
        return dispatch_stop<MT, dlib::cg_search_strategy>(mag, line, d, r, d_lower, d_upper, r_lower, r_upper, mean_op,
                                                           dlib::cg_search_strategy());
      case PrecisionSearchStrategy::BFGS:
      default:
        return dispatch_stop<MT, dlib::bfgs_search_strategy>(mag, line, d, r, d_lower, d_upper, r_lower, r_upper,
                                                             mean_op, dlib::bfgs_search_strategy());
    }
  }

  /// @brief Dispatch based on stop strategy.
  template <class MT, class SearchT>
  double dispatch_stop(const cv::Mat& mag,
                       const LineSegmentType& line,
                       double& d,
                       double& r,
                       double d_lower,
                       double d_upper,
                       double r_lower,
                       double r_upper,
                       MeanFunc mean_op,
                       SearchT search) const {
    switch (stop_strategy_) {
      case PrecisionStopStrategy::GRAD_NORM:
        if (max_iterations_ > 0) {
          return LineOptimizer<MT, SearchT, dlib::gradient_norm_stop_strategy>::template optimize<double>(
              mag, line, d, r, d_lower, d_upper, r_lower, r_upper, mean_param_, derivative_precision_, mean_op, search,
              dlib::gradient_norm_stop_strategy(stop_delta_, static_cast<unsigned long>(max_iterations_)));
        }
        return LineOptimizer<MT, SearchT, dlib::gradient_norm_stop_strategy>::template optimize<double>(
            mag, line, d, r, d_lower, d_upper, r_lower, r_upper, mean_param_, derivative_precision_, mean_op, search,
            dlib::gradient_norm_stop_strategy(stop_delta_));
      case PrecisionStopStrategy::DELTA:
      default:
        if (max_iterations_ > 0) {
          return LineOptimizer<MT, SearchT, dlib::objective_delta_stop_strategy>::template optimize<double>(
              mag, line, d, r, d_lower, d_upper, r_lower, r_upper, mean_param_, derivative_precision_, mean_op, search,
              dlib::objective_delta_stop_strategy(stop_delta_, static_cast<unsigned long>(max_iterations_)));
        }
        return LineOptimizer<MT, SearchT, dlib::objective_delta_stop_strategy>::template optimize<double>(
            mag, line, d, r, d_lower, d_upper, r_lower, r_upper, mean_param_, derivative_precision_, mean_op, search,
            dlib::objective_delta_stop_strategy(stop_delta_));
    }
  }
};

}  // namespace lsfm
