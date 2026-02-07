//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann & Manuel Lange
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file line_optimizer.hpp
/// @brief Line segment optimization using gradient magnitude.
///
/// Provides LineOptimizer for refining line segment positions by maximizing
/// gradient magnitude response. Uses dlib optimization with box constraints
/// for rotation and orthogonal translation.
/// @note Detects junction or corner areas and skips them for optimization
///
/// to reduce error for wide-range interpolation like cubic.

#pragma once

#include <dlib/optimization.h>
#include <imgproc/mean.hpp>

namespace lsfm {

/// @brief Line segment optimizer using gradient magnitude maximization.
///
/// Optimizes line segment position by adjusting rotation and orthogonal
/// translation to maximize gradient magnitude response along the line.
/// Uses dlib's box-constrained optimization with configurable search
/// and stop strategies.
/// @tparam mat_type OpenCV matrix element type for gradient magnitude.
/// @tparam search_strategy_type Dlib search strategy (default: BFGS).
/// @tparam stop_strategy_type Dlib stop strategy (default: objective delta).
template <class mat_type,
          class search_strategy_type = dlib::bfgs_search_strategy,
          class stop_strategy_type = dlib::objective_delta_stop_strategy>
struct LineOptimizer {
  /// @brief Optimize line parameters for gradient magnitude fit.
  ///
  /// Finds optimal rotation and orthogonal translation to maximize
  /// gradient magnitude response along the line segment.
  /// @tparam FT Floating-point type.
  /// @tparam LPT Line point type template.
  /// @param mag Gradient magnitude image.
  /// @param l Line segment to optimize.
  /// @param d Output orthogonal translation distance (pixels).
  /// @param r Output rotation angle (radians).
  /// @param d_lower Lower bound for distance search (default -1).
  /// @param d_upper Upper bound for distance search (default 1).
  /// @param r_lower Lower bound for rotation search (default -π/180).
  /// @param r_upper Upper bound for rotation search (default π/180).
  /// @param mean_param Parameter for mean calculation.
  /// @param derivative_prec Delta for numerical derivatives.
  /// @param mean_op Mean computation operator.
  /// @param search Dlib search strategy.
  /// @param stop Dlib stop strategy.
  /// @return Optimization error (negative of maximized value).
  template <class FT, template <class> class LPT>
  static inline double optimize(const cv::Mat& mag,
                                const LineSegment<FT, LPT>& l,
                                FT& d,
                                FT& r,
                                double d_lower = -1,
                                double d_upper = 1,
                                double r_lower = -CV_PI / 180,
                                double r_upper = CV_PI / 180,
                                double mean_param = 1,
                                double derivative_prec = 1e-7,
                                typename MeanHelper<double, LPT>::func_type mean_op =
                                    Mean<double, mat_type, LinearInterpolator<double, mat_type>>::process,
                                search_strategy_type search = dlib::bfgs_search_strategy(),
                                stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
    CV_Assert(cv::DataType<mat_type>::type == mag.type());

    typedef dlib::matrix<double, 0, 1> column_vector;
    auto eval = [&](const column_vector& v) -> double {
      LineSegment<double, LPT> tmp = l;
      tmp.translateOrtho(v(0));
      tmp.rotate(v(1), tmp.center());
      return mean_op(mag, tmp, mean_param);
    };

    column_vector starting_point(2), lower(2), upper(2);
    starting_point = d, r;
    lower = d_lower, r_lower;
    upper = d_upper, r_upper;
    double ret = dlib::find_max_box_constrained(search, stop, eval, dlib::derivative(eval, derivative_prec),
                                                starting_point, lower, upper);
    d = starting_point(0);
    r = starting_point(1);
    return ret;
  }

  /// @brief Optimize line segment in place.
  ///
  /// Same as optimize() but applies the optimized parameters directly
  /// to the line segment object.
  /// @tparam FT Floating-point type.
  /// @tparam LPT Line point type template.
  /// @param mag Gradient magnitude image.
  /// @param l Line segment to optimize (modified in place).
  /// @param d_lower Lower bound for distance search.
  /// @param d_upper Upper bound for distance search.
  /// @param r_lower Lower bound for rotation search.
  /// @param r_upper Upper bound for rotation search.
  /// @param mean_param Parameter for mean calculation.
  /// @param derivative_prec Delta for numerical derivatives.
  /// @param mean_op Mean computation operator.
  /// @param search Dlib search strategy.
  /// @param stop Dlib stop strategy.
  /// @return Optimization error.
  template <class FT, template <class> class LPT>
  static inline double optimize_line(const cv::Mat& mag,
                                     LineSegment<FT, LPT>& l,
                                     double d_lower = -1,
                                     double d_upper = 1,
                                     double r_lower = -CV_PI / 180,
                                     double r_upper = CV_PI / 180,
                                     double mean_param = 1,
                                     double derivative_prec = 1e-7,
                                     typename MeanHelper<double, LPT>::func_type mean_op =
                                         Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
                                     search_strategy_type search = dlib::bfgs_search_strategy(),
                                     stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
    FT d = 0, r = 0;
    double ret =
        optimize(mag, l, d, r, d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op, search, stop);
    l.translateOrtho(d);
    l.rotate(r, l.center());
    return ret;
  }

  /// @brief Optimize vector of line segments in place.
  ///
  /// Applies optimize_line() to each line segment in the vector.
  /// @tparam FT Floating-point type.
  /// @tparam LPT Line point type template.
  /// @tparam LV Line vector type.
  /// @param mag Gradient magnitude image.
  /// @param in Line segments to optimize (modified in place).
  /// @param d_lower Lower bound for distance search.
  /// @param d_upper Upper bound for distance search.
  /// @param r_lower Lower bound for rotation search.
  /// @param r_upper Upper bound for rotation search.
  /// @param mean_param Parameter for mean calculation.
  /// @param derivative_prec Delta for numerical derivatives.
  /// @param mean_op Mean computation operator.
  /// @param search Dlib search strategy.
  /// @param stop Dlib stop strategy.
  template <class FT, template <class> class LPT, class LV>
  static inline void optimizeLV(const cv::Mat& mag,
                                LV& in,
                                double d_lower = -1,
                                double d_upper = 1,
                                double r_lower = -CV_PI / 180,
                                double r_upper = CV_PI / 180,
                                FT mean_param = 1,
                                double derivative_prec = 1e-7,
                                typename MeanHelper<double, LPT>::func_type mean_op =
                                    Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
                                search_strategy_type search = dlib::bfgs_search_strategy(),
                                stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
    for (size_t i = 0; i != in.size(); ++i)
      optimize_line(mag, in[i], d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op, search, stop);
  }

  /// @brief Optimize vector of line segments with error output.
  ///
  /// Applies optimize_line() to each line segment, recording optimization errors.
  /// @tparam FT Floating-point type.
  /// @tparam LPT Line point type template.
  /// @tparam LV Line vector type.
  /// @param mag Gradient magnitude image.
  /// @param in Line segments to optimize (modified in place).
  /// @param err Output vector of optimization errors.
  /// @param d_lower Lower bound for distance search.
  /// @param d_upper Upper bound for distance search.
  /// @param r_lower Lower bound for rotation search.
  /// @param r_upper Upper bound for rotation search.
  /// @param mean_param Parameter for mean calculation.
  /// @param derivative_prec Delta for numerical derivatives.
  /// @param mean_op Mean computation operator.
  /// @param search Dlib search strategy.
  /// @param stop Dlib stop strategy.
  template <class FT, template <class> class LPT, class LV>
  static inline void optimizeLV(const cv::Mat& mag,
                                LV& in,
                                std::vector<double>& err,
                                double d_lower = -1,
                                double d_upper = 1,
                                double r_lower = -CV_PI / 180,
                                double r_upper = CV_PI / 180,
                                double mean_param = 1,
                                double derivative_prec = 1e-7,
                                typename MeanHelper<FT, LPT>::func_type mean_op =
                                    Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
                                search_strategy_type search = dlib::bfgs_search_strategy(),
                                stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
    err.resize(in.size());
    for (size_t i = 0; i != in.size(); ++i)
      err[i] = optimize_line(mag, in[i], d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op,
                             search, stop);
  }

  /// @brief Optimize line segments to new output vector.
  ///
  /// Copies input lines to output and optimizes the copies.
  /// @tparam FT Floating-point type.
  /// @tparam LPT Line point type template.
  /// @tparam LV Line vector type.
  /// @param mag Gradient magnitude image.
  /// @param in Input line segments (unchanged).
  /// @param out Output optimized line segments.
  /// @param d_lower Lower bound for distance search.
  /// @param d_upper Upper bound for distance search.
  /// @param r_lower Lower bound for rotation search.
  /// @param r_upper Upper bound for rotation search.
  /// @param mean_param Parameter for mean calculation.
  /// @param derivative_prec Delta for numerical derivatives.
  /// @param mean_op Mean computation operator.
  /// @param search Dlib search strategy.
  /// @param stop Dlib stop strategy.
  template <class FT, template <class> class LPT, class LV>
  static inline void optimizeLV(const cv::Mat& mag,
                                const LV& in,
                                LV& out,
                                double d_lower = -1,
                                double d_upper = 1,
                                double r_lower = -CV_PI / 180,
                                double r_upper = CV_PI / 180,
                                double mean_param = 1,
                                double derivative_prec = 1e-7,
                                typename MeanHelper<FT, LPT>::func_type mean_op =
                                    Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
                                search_strategy_type search = dlib::bfgs_search_strategy(),
                                stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
    out.resize(in.size());
    for (size_t i = 0; i != in.size(); ++i) {
      out[i] = in[i];
      optimize_line(mag, out[i], d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op, search,
                    stop);
    }
  }

  /// @brief Optimize line segments to output vector with errors.
  ///
  /// Copies input lines to output, optimizes the copies, and records errors.
  /// @tparam FT Floating-point type.
  /// @tparam LPT Line point type template.
  /// @tparam LV Line vector type.
  /// @param mag Gradient magnitude image.
  /// @param in Input line segments (unchanged).
  /// @param out Output optimized line segments.
  /// @param err Output vector of optimization errors.
  /// @param d_lower Lower bound for distance search.
  /// @param d_upper Upper bound for distance search.
  /// @param r_lower Lower bound for rotation search.
  /// @param r_upper Upper bound for rotation search.
  /// @param mean_param Parameter for mean calculation.
  /// @param derivative_prec Delta for numerical derivatives.
  /// @param mean_op Mean computation operator.
  /// @param search Dlib search strategy.
  /// @param stop Dlib stop strategy.
  template <class FT, template <class> class LPT, class LV>
  static inline void optimizeLV(const cv::Mat& mag,
                                const LV& in,
                                LV& out,
                                std::vector<double>& err,
                                double d_lower = -1,
                                double d_upper = 1,
                                double r_lower = -CV_PI / 180,
                                double r_upper = CV_PI / 180,
                                double mean_param = 1,
                                double derivative_prec = 1e-7,
                                typename MeanHelper<double, LPT>::func_type mean_op =
                                    Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
                                search_strategy_type search = dlib::bfgs_search_strategy(),
                                stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
    out.resize(in.size());
    err.resize(in.size());
    for (size_t i = 0; i != in.size(); ++i) {
      out[i] = in[i];
      err[i] = optimize_line(mag, out[i], d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op,
                             search, stop);
    }
  }
};


}  // namespace lsfm
