//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file fit.hpp
/// @brief Curve fitting algorithms for edge segments.
/// Provides methods to fit various curve models (lines, conic sections, splines)
/// to detected edge segments using least squares and robust estimation techniques.

#pragma once

#include <edge/edge_segment.hpp>
#include <edge/index.hpp>
#include <geometry/line.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/value_manager.hpp>

#include <string>
#include <vector>


namespace lsfm {

/// @brief Compute covariance matrix and centroid from point sequence.
/// Calculates second-moment statistics (covariance) and center of mass for a set of points.
/// @tparam FT Floating-point type for calculations
/// @tparam PT Point type with getX() and getY() accessors
/// @param beg Beginning iterator of points
/// @param end Ending iterator of points
/// @param sx Output X variance (Sxx)
/// @param sy Output Y variance (Syy)
/// @param sxy Output covariance (Sxy)
/// @param cx Output X centroid coordinate
/// @param cy Output Y centroid coordinate
template <class FT, class PT>
inline void covariance(const PT* beg, const PT* end, FT& sx, FT& sy, FT& sxy, FT& cx, FT& cy) {
  int count = 0;
  FT sum_x = 0, sum_y = 0;

  std::for_each(beg, end, [&](const PT& lp) {
    ++count;
    sum_x += static_cast<FT>(getX(lp));
    sum_y += static_cast<FT>(getY(lp));
  });

  cx = sum_x / static_cast<FT>(count);
  cy = sum_y / static_cast<FT>(count);
  sx = 0;
  sy = 0;
  sxy = 0;

  std::for_each(beg, end, [&](const PT& lp) {
    FT dx = static_cast<FT>(getX(lp)) - cx;
    FT dy = static_cast<FT>(getY(lp)) - cy;

    sxy -= dx * dy;
    sx += dx * dx;
    sy += dy * dy;
  });
}

/// @brief Compute weighted covariance matrix and centroid from point sequence.
/// Calculates weighted second-moment statistics using point magnitudes as weights.
/// @tparam FT Floating-point type for calculations
/// @tparam PT Point type with getX() and getY() accessors
/// @tparam DT Data element type in weight matrix
/// @param beg Beginning iterator of points
/// @param end Ending iterator of points
/// @param sx Output X variance (weighted Sxx)
/// @param sy Output Y variance (weighted Syy)
/// @param sxy Output covariance (weighted Sxy)
/// @param cx Output X centroid coordinate (weighted)
/// @param cy Output Y centroid coordinate (weighted)
/// @param data Optional weight map; if empty, unweighted statistics computed
template <class FT, class PT, class DT = FT>
inline void covariance(const PT* beg, const PT* end, FT& sx, FT& sy, FT& sxy, FT& cx, FT& cy, const cv::Mat& data) {
  FT count = 0;
  FT sum_x = 0, sum_y = 0;
  sx = 0;
  sy = 0;
  sxy = 0;

  if (data.empty()) {
    std::for_each(beg, end, [&](const PT& lp) {
      ++count;
      sum_x += static_cast<FT>(getX(lp));
      sum_y += static_cast<FT>(getY(lp));
    });

    cx = static_cast<FT>(sum_x) / count;
    cy = static_cast<FT>(sum_y) / count;

    std::for_each(beg, end, [&](const PT& lp) {
      FT dx = static_cast<FT>(getX(lp)) - cx;
      FT dy = static_cast<FT>(getY(lp)) - cy;

      sxy -= dx * dy;
      sx += dx * dx;
      sy += dy * dy;
    });
  } else {
    std::for_each(beg, end, [&](const PT& lp) {
      FT m = static_cast<FT>(data.at<DT>(getY(lp), getX(lp)));
      count += m;
      sum_x += static_cast<FT>(getX(lp)) * m;
      sum_y += static_cast<FT>(getY(lp)) * m;
    });

    cx = static_cast<FT>(sum_x) / count;
    cy = static_cast<FT>(sum_y) / count;

    std::for_each(beg, end, [&](const PT& lp) {
      FT m = static_cast<FT>(data.at<DT>(getY(lp), getX(lp)));

      FT dx = static_cast<FT>(getX(lp)) - cx;
      FT dy = static_cast<FT>(getY(lp)) - cy;

      sxy -= m * dx * dy;
      sx += m * dx * dx;
      sy += m * dy * dy;
    });
  }
}

/// @brief Line fitting using regression method.
/// Fits a line to points using least-squares regression on covariance matrix.
/// @tparam FT Floating-point type for calculations
/// @tparam PT Point type
/// @tparam DT Data element type for weighting (default: FT)
template <class FT, class PT, class DT = FT>
class RegressionFit {
 public:
  /// @typedef float_type
  /// @brief Floating-point type used in calculations
  typedef FT float_type;

  /// @typedef point_type
  /// @brief Point type
  typedef PT point_type;

  /// @typedef data_type
  /// @brief Data/weight element type
  typedef DT data_type;

  /// @brief Get the name of this fit method.
  /// @return String "Regression"
  static const std::string name() { return "Regression"; }

  /// @brief Fit normalized line normal vector from covariance components.
  /// Solves the eigenvalue problem to get the dominant direction.
  /// @param sx X variance (Sxx)
  /// @param sy Y variance (Syy)
  /// @param sxy Covariance (Sxy)
  /// @param nx Output X component of unit normal vector
  /// @param ny Output Y component of unit normal vector
  static inline void fit_unorm(FT sx, FT sy, FT sxy, FT& nx, FT& ny) {
    if (sx > sy) {
      nx = sxy;
      ny = sx;
    } else {
      nx = sy;
      ny = sxy;
    }
  }

  static inline void fit(FT sx, FT sy, FT sxy, FT& nx, FT& ny) {
    fit_unorm(sx, sy, sxy, nx, ny);
    FT norm = hypot(nx, ny);
    nx /= norm;
    ny /= norm;
  }

  static inline void fit_unorm(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny) {
    FT sx, sy, sxy;

    covariance(beg, end, sx, sy, sxy, cx, cy);
    fit_unorm(sx, sy, sxy, nx, ny);
  }

  static inline void fit(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny) {
    FT sx, sy, sxy;

    covariance(beg, end, sx, sy, sxy, cx, cy);
    fit(sx, sy, sxy, nx, ny);
  }

  static inline void fit(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny, const cv::Mat& data) {
    FT sx, sy, sxy;

    covariance<FT, PT, DT>(beg, end, sx, sy, sxy, cx, cy, data);
    fit(sx, sy, sxy, nx, ny);
  }

  template <template <class> class LPT>
  static inline void fit(const PT* beg, const PT* end, Line<FT, LPT>& l) {
    FT cx, cy, nx, ny;

    fit(beg, end, cx, cy, nx, ny);
    l = Line<FT, LPT>(nx, ny, cx, cy);
  }

  template <template <class> class LPT>
  static inline void fit(const PT* beg, const PT* end, Line<FT, LPT>& l, const cv::Mat& data) {
    FT cx, cy, nx, ny;

    fit(beg, end, cx, cy, nx, ny, data);
    l = Line<FT, LPT>(nx, ny, cx, cy);
  }
};


/// @brief Line fitting using eigenvalue decomposition.
/// Computes the smallest eigenvector of the covariance matrix to find
/// the best-fit line normal direction.
/// @tparam FT Floating-point type
/// @tparam PT Point type
/// @tparam DT Data/weight element type
template <class FT, class PT, class DT = FT>
class EigenFit {
 public:
  /// @typedef float_type
  /// @brief Floating-point type used in calculations
  typedef FT float_type;

  /// @typedef point_type
  /// @brief Point type
  typedef PT point_type;

  /// @typedef data_type
  /// @brief Data/weight element type
  typedef DT data_type;

  EigenFit() {}

  /// @brief Get the name of this fit method.
  /// @return "Eigen"
  static std::string name() { return "Eigen"; }

  /// @brief Compute eigenvalues from covariance components.
  /// @param sx X variance
  /// @param sy Y variance
  /// @param sxy Covariance
  /// @param e1 Output largest eigenvalue
  /// @param e2 Output smallest eigenvalue
  static inline void eigen(FT sx, FT sy, FT sxy, FT& e1, FT& e2) {
    FT lambda = static_cast<FT>(0.5) * std::sqrt((sx - sy) * (sx - sy) + 4 * sxy * sxy);
    FT p = static_cast<FT>(0.5) * (sx + sy);
    e1 = p + lambda;
    e2 = p - lambda;
  }

  // gives the direction of the smallest eigenvalue, the direction for largest is eigenvalue is dy,-dx
  static inline void eigen(FT sx, FT sy, FT sxy, FT& e1, FT& e2, FT& dx, FT& dy) {
    eigen(sx, sy, sxy, e1, e2);
    fit(sx, sy, sxy, dx, dy);
  }

  /// @brief Fit unnormalized line normal from covariance components.
  /// Uses eigenvalue decomposition to find the normal direction.
  /// @param sx X variance
  /// @param sy Y variance
  /// @param sxy Covariance
  /// @param nx Output X component of normal (unnormalized)
  /// @param ny Output Y component of normal (unnormalized)
  static inline void fit_unorm(FT sx, FT sy, FT sxy, FT& nx, FT& ny) {
    // Compute smallest eigenvalue
    FT lambda = static_cast<FT>(0.5) * (sx + sy - std::sqrt((sx - sy) * (sx - sy) + 4 * sxy * sxy));

    if (sx > sy) {
      nx = sxy;
      ny = sx - lambda;
    } else {
      nx = sy - lambda;
      ny = sxy;
    }
  }

  /// @brief Fit normalized line normal from covariance components.
  /// @param sx X variance
  /// @param sy Y variance
  /// @param sxy Covariance
  /// @param nx Output X component of unit normal
  /// @param ny Output Y component of unit normal
  static inline void fit(FT sx, FT sy, FT sxy, FT& nx, FT& ny) {
    fit_unorm(sx, sy, sxy, nx, ny);
    FT norm = hypot(nx, ny);
    nx /= norm;
    ny /= norm;
  }

  static inline void fit(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny) {
    FT sx, sy, sxy;

    covariance(beg, end, sx, sy, sxy, cx, cy);
    fit(sx, sy, sxy, nx, ny);
  }

  static inline void fit_unorm(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny) {
    FT sx, sy, sxy;

    covariance(beg, end, sx, sy, sxy, cx, cy);
    fit_unorm(sx, sy, sxy, nx, ny);
  }

  static inline void fit(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny, const cv::Mat& data) {
    FT sx, sy, sxy;

    covariance<FT, PT, DT>(beg, end, sx, sy, sxy, cx, cy, data);
    fit(sx, sy, sxy, nx, ny);
  }

  template <template <class> class LPT>
  static inline void fit(const PT* beg, const PT* end, Line<FT, LPT>& l) {
    FT cx, cy, nx, ny;

    fit(beg, end, cx, cy, nx, ny);
    l = Line<FT, LPT>(nx, ny, cx, cy);
  }

  template <template <class> class LPT>
  static inline void fit(const PT* beg, const PT* end, Line<FT, LPT>& l, const cv::Mat& data) {
    FT cx, cy, nx, ny;

    fit(beg, end, cx, cy, nx, ny, data);
    l = Line<FT, LPT>(nx, ny, cx, cy);
  }
};

/// @brief Line fitting using OpenCV eigenvalue decomposition.
/// Uses cv::eigen to solve the 2x2 covariance eigenvalue problem.
/// @tparam FT Floating-point type
/// @tparam PT Point type
/// @tparam DT Data/weight element type
template <class FT, class PT, class DT = FT>
class EigenCVFit {
 public:
  /// @typedef float_type
  /// @brief Floating-point type
  typedef FT float_type;

  /// @typedef point_type
  /// @brief Point type
  typedef PT point_type;

  /// @typedef data_type
  /// @brief Data/weight element type
  typedef DT data_type;

  EigenCVFit() {}

  /// @brief Get the name of this fit method.
  /// @return "EigenCV"
  static std::string name() { return "EigenCV"; }

  /// @brief Compute eigenvalues using OpenCV.
  /// @param sx X variance
  /// @param sy Y variance
  /// @param sxy Covariance
  /// @param e1 Output largest eigenvalue
  /// @param e2 Output smallest eigenvalue
  static inline void eigen(FT sx, FT sy, FT sxy, FT& e1, FT& e2) {
    cv::Matx<FT, 2, 2> M(sx, sxy, sxy, sy);

    cv::Mat E, V;
    cv::eigen(M, E, V);
    e1 = E.at<FT>(0, 0);
    e2 = E.at<FT>(1, 0);
  }

  // gives the direction of the smallest eigenvalue, the direction for largest is eigenvalue is dy,-dx
  static inline void eigen(FT sx, FT sy, FT sxy, FT& e1, FT& e2, FT& dx, FT& dy) {
    cv::Matx<FT, 2, 2> M(sx, sxy, sxy, sy);

    cv::Mat E, V;
    cv::eigen(M, E, V);
    e1 = E.at<FT>(0, 0);
    e2 = E.at<FT>(1, 0);
    dx = -V.at<FT>(1, 0);
    dy = V.at<FT>(1, 1);
  }

  static inline void fit(FT sx, FT sy, FT sxy, FT& nx, FT& ny) {
    cv::Matx<FT, 2, 2> M(sx, sxy, sxy, sy);

    cv::Mat E, V;
    cv::eigen(M, E, V);
    nx = -V.at<FT>(1, 0);
    ny = V.at<FT>(1, 1);
  }

  /// @brief Fit unnormalized line normal from covariance components.
  /// Since OpenCV eigen returns normalized vectors, this is equivalent to fit().
  static inline void fit_unorm(FT sx, FT sy, FT sxy, FT& nx, FT& ny) { fit(sx, sy, sxy, nx, ny); }

  static inline void fit(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny) {
    FT sx, sy, sxy;

    covariance(beg, end, sx, sy, sxy, cx, cy);
    fit(sx, sy, sxy, nx, ny);
  }

  static inline void fit_unorm(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny) {
    fit(beg, end, cx, cy, nx, ny);
  }

  static inline void fit(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny, const cv::Mat& data) {
    FT sx, sy, sxy;

    covariance<FT, PT, DT>(beg, end, sx, sy, sxy, cx, cy, data);
    fit(sx, sy, sxy, nx, ny);
  }

  template <template <class> class LPT>
  static inline void fit(const PT* beg, const PT* end, Line<FT, LPT>& l) {
    FT cx, cy, nx, ny;

    fit(beg, end, cx, cy, nx, ny);
    l = Line<FT, LPT>(nx, ny, cx, cy);
  }

  template <template <class> class LPT>
  static inline void fit(const PT* beg, const PT* end, Line<FT, LPT>& l, const cv::Mat& data) {
    FT cx, cy, nx, ny;

    fit(beg, end, cx, cy, nx, ny, data);
    l = Line<FT, LPT>(nx, ny, cx, cy);
  }
};

/// @brief Base class for line fitting operations.
/// Wraps a FIT implementation with various apply() overloads for different
/// point representations (points, indices, segments).
/// @tparam FIT Underlying fit implementation type
template <class FIT>
class FitLine : public ValueManager {
  FitLine(const FitLine&);

 protected:
  FIT fit_;

 public:
  /// @typedef fit_type
  /// @brief Underlying fit type
  typedef FIT fit_type;

  /// @typedef float_type
  /// @brief Floating-point type
  typedef typename fit_type::float_type float_type;

  /// @typedef point_type
  /// @brief Point type
  typedef typename fit_type::point_type point_type;

  FitLine() : fit_() {}

  FitLine(const ValueManager::NameValueVector& options) { static_cast<void>(options); }

  FitLine(ValueManager::InitializerList options) : fit_() { static_cast<void>(options); }

  template <class ITER, template <class> class LPT>
  inline void apply(ITER beg, ITER end, Line<float_type, LPT>& l) const {
    // type of ITER must have a conversion operator to Point
    std::vector<point_type> tmp(beg, end);
    fit_.fit(&tmp.front(), &tmp.back() + 1, l);
  }

  template <class ITER, template <class> class LPT>
  inline void apply(ITER beg, ITER end, Line<float_type, LPT>& l, const cv::Mat& data) const {
    // type of ITER must have a conversion operator to Point
    std::vector<point_type> tmp(beg, end);
    fit_.fit(&tmp.front(), &tmp.back() + 1, l, data);
  }

  template <template <class> class LPT>
  inline void apply(const point_type* beg, const point_type* end, Line<float_type, LPT>& l) const {
    fit_.fit(beg, end, l);
  }

  template <template <class> class LPT>
  inline void apply(const point_type* beg, const point_type* end, Line<float_type, LPT>& l, const cv::Mat& data) const {
    fit_.fit(beg, end, l, data);
  }

  template <template <class> class LPT>
  inline void apply(typename std::vector<index_type>::const_iterator beg,
                    typename std::vector<index_type>::const_iterator end,
                    Line<float_type, LPT>& l) const {
    std::vector<point_type> tmp;
    tmp.resize(std::distance(beg, end));
    index2Point(beg, end, &tmp.front());
    fit_.fit(&tmp.front(), &tmp.back() + 1, l);
  }

  template <template <class> class LPT>
  inline void apply(typename std::vector<index_type>::const_iterator beg,
                    typename std::vector<index_type>::const_iterator end,
                    Line<float_type, LPT>& l,
                    const cv::Mat& data) const {
    std::vector<point_type> tmp;
    tmp.resize(std::distance(beg, end));
    index2Point(beg, end, &tmp.front());
    fit_.fit(&tmp.front(), &tmp.back() + 1, l, data);
  }

  template <template <class> class LPT>
  inline void apply(typename std::vector<point_type>::const_iterator beg,
                    typename std::vector<point_type>::const_iterator end,
                    Line<float_type, LPT>& l) const {
    fit_.fit(&(*beg), &(*(end - 1)) + 1, l);
  }

  template <template <class> class LPT>
  inline void apply(typename std::vector<point_type>::const_iterator beg,
                    typename std::vector<point_type>::const_iterator end,
                    Line<float_type, LPT>& l,
                    const cv::Mat& data) const {
    fit_.fit(&(*beg), &(*(end - 1)) + 1, l, data);
  }


  template <class VEC, template <class> class LPT>
  inline void apply(const VEC& p, Line<float_type, LPT>& l) const {
    apply(&p[0], &p[0] + p.size(), l);
  }

  template <class VEC, template <class> class LPT>
  inline void apply(const VEC& p, Line<float_type, LPT>& l, const cv::Mat& data) const {
    apply(&p[0], &p[0] + p.size(), l, data);
  }

  template <template <class> class LPT>
  inline void apply(const EdgeSegmentVector& segments,
                    const std::vector<point_type>& points,
                    std::vector<Line<float_type, LPT>>& lines) const {
    lines.clear();
    lines.reserve(segments.size());
    for_each(segments.begin(), segments.end(), [&](const EdgeSegment& seg) {
      Line<float_type, LPT> l;
      fit_.fit(points.data() + seg.begin(), points.data() + seg.end(), l);
      lines.push_back(l);
    });
  }

  template <template <class> class LPT>
  inline void apply(const EdgeSegmentVector& segments,
                    const std::vector<point_type>& points,
                    std::vector<LineSegment<float_type, LPT>>& lines) const {
    lines.clear();
    lines.reserve(segments.size());
    for_each(segments.begin(), segments.end(), [&](const EdgeSegment& seg) {
      Line<float_type, LPT> l;
      fit_.fit(points.data() + seg.begin(), points.data() + seg.end(), l);
      const point_type& first = points[seg.first()];
      const point_type& last = points[seg.last()];
      lines.push_back(LineSegment<float_type, LPT>(l, first, last));
    });
  }

  /// @brief Get the name of this fit line operator.
  /// @return Name string from the underlying FIT type
  virtual const std::string name() const { return FIT::name(); }
};

/// @brief Line fitting using OpenCV M-Estimator (robust fitting).
/// Wraps cv::fitLine with configurable distance type and parameters.
/// @tparam FT Floating-point type
/// @tparam PT Point type
template <class FT, class PT>
class MEstimatorFit {
 public:
  /// @typedef float_type
  /// @brief Floating-point type
  typedef FT float_type;

  /// @typedef point_type
  /// @brief Point type
  typedef PT point_type;

  int dist;
  double param, reps, aeps;

  /// @brief Get the name of this fit method.
  /// @return "M-Estimator"
  static std::string name() { return "M-Estimator"; }

  MEstimatorFit(int d = cv::DIST_L2, double p = 0, double r = 0.001, double a = 0.001)
      : dist(d), param(p), reps(r), aeps(a) {}

  void fit(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny) const {
    cv::Vec<float, 4> res;
    std::vector<cv::Point> tmp;
    tmp.reserve(end - beg);
    for (const PT* i = beg; i != end; ++i) tmp.push_back(cv::Point(getX(*i), getY(*i)));
    cv::fitLine(cv::Mat(tmp), res, dist, param, reps, aeps);
    nx = -res[1];
    ny = res[0];
    cx = res[2];
    cy = res[3];
  }

  void fit(const PT* beg, const PT* end, FT& cx, FT& cy, FT& nx, FT& ny, const cv::Mat& data) const {
    fit(beg, end, cx, cy, nx, ny);
  }

  template <template <class> class LPT>
  void fit(const PT* beg, const PT* end, Line<FT, LPT>& l) const {
    cv::Vec<float, 4> res;
    std::vector<cv::Point> tmp;
    tmp.reserve(end - beg);
    for (const PT* i = beg; i != end; ++i) tmp.push_back(cv::Point(getX(*i), getY(*i)));
    cv::fitLine(cv::Mat(tmp), res, dist, param, reps, aeps);
    l = Line<FT, LPT>(-res[1], res[0], res[2], res[3]);
  }

  template <template <class> class LPT>
  void fit(const PT* beg, const PT* end, Line<FT, LPT>& l, const cv::Mat& data) const {
    fit(beg, end, l);
  }
};

/// @brief M-Estimator specialization for cv::Point.
/// Avoids copying by wrapping the point array directly as a cv::Mat.
/// @tparam FT Floating-point type
template <class FT>
class MEstimatorFit<FT, cv::Point> {
 public:
  int dist;
  double param, reps, aeps;

  /// @typedef float_type
  /// @brief Floating-point type
  typedef FT float_type;

  /// @typedef point_type
  /// @brief Point type (cv::Point)
  typedef cv::Point point_type;

  MEstimatorFit(int d = cv::DIST_L2, double p = 0, double r = 0.001, double a = 0.001)
      : dist(d), param(p), reps(r), aeps(a) {}

  static std::string name() { return "M-Estimator"; }

  void fit(const cv::Point* beg, const cv::Point* end, FT& cx, FT& cy, FT& nx, FT& ny) const {
    cv::Vec<float, 4> res;
    cv::Mat tmp(end - beg, 2, CV_32SC1, const_cast<cv::Point*>(beg));
    cv::fitLine(tmp, res, dist, param, reps, aeps);
    nx = -res[1];
    ny = res[0];
    cx = res[2];
    cy = res[3];
  }

  void fit(const cv::Point* beg, const cv::Point* end, FT& cx, FT& cy, FT& nx, FT& ny, const cv::Mat& data) const {
    fit(beg, end, cx, cy, nx, ny);
  }

  template <template <class> class LPT>
  void fit(const cv::Point* beg, const cv::Point* end, Line<FT, LPT>& l) const {
    cv::Vec<float, 4> res;
    cv::Mat tmp(static_cast<int>(end - beg), 2, CV_32SC1, const_cast<cv::Point*>(beg));
    cv::fitLine(tmp, res, dist, param, reps, aeps);
    l = Line<FT, LPT>(-res[1], res[0], res[2], res[3]);
  }

  template <template <class> class LPT>
  void fit(const cv::Point* beg, const cv::Point* end, Line<FT, LPT>& l, const cv::Mat& data) const {
    fit(beg, end, l);
  }
};

/// @brief FitLine wrapper for M-Estimator with configurable parameters.
/// Exposes distance type, parameter, reps, and aeps as value-managed options.
/// @tparam FT Floating-point type
/// @tparam PT Point type
template <class FT, class PT>
class MEstimatorFitLine : public FitLine<MEstimatorFit<FT, PT>> {
  using FitLine<MEstimatorFit<FT, PT>>::fit_;

  void init() {
    fit_.dist = cv::DIST_L2;
    fit_.param = 0;
    fit_.reps = 0.001;
    fit_.aeps = 0.001;

    this->add("fit_distance", std::bind(&MEstimatorFitLine<FT, PT>::valueDist, this, std::placeholders::_1),
              "Distance used by the M-estimator.");

    this->add("fit_param", std::bind(&MEstimatorFitLine<FT, PT>::valueParam, this, std::placeholders::_1),
              "Numerical parameter for some types of distances. If it is 0, an optimal value is chosen.");

    this->add("fit_reps", std::bind(&MEstimatorFitLine<FT, PT>::valueREps, this, std::placeholders::_1),
              "Sufficient accuracy for the radius (distance between the coordinate origin and the line)");

    this->add("fit_aeps", std::bind(&MEstimatorFitLine<FT, PT>::valueAEps, this, std::placeholders::_1),
              "Sufficient accuracy for the angle.");
  }

 public:
  using FitLine<MEstimatorFit<FT, PT>>::apply;

  MEstimatorFitLine(int d = cv::DIST_L2, double p = 0, double r = 0.001, double a = 0.001) {
    init();

    CV_Assert(d > 0 && d < 8 && p >= 0 && r > 0 && a > 0);

    fit_.dist = d;
    fit_.param = p;
    fit_.reps = r;
    fit_.aeps = a;
  }

  MEstimatorFitLine(const ValueManager::NameValueVector& options) {
    init();
    this->value(options);
  }

  MEstimatorFitLine(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  Value valueDist(const Value& d = Value::NAV()) {
    if (d.type()) distance(d.getInt());
    return fit_.dist;
  }

  int distance() const { return fit_.dist; }

  void distance(int d) {
    if (d > 0 && d < 8) fit_.dist = d;
  }

  Value valueParam(const Value& p = Value::NAV()) {
    if (p.type()) param(p.getDouble());
    return fit_.param;
  }

  double param() const { return fit_.param; }

  void param(double p) { fit_.param = p; }

  Value valueREps(const Value& reps = Value::NAV()) {
    if (reps.type()) rEps(reps.getDouble());
    return fit_.reps;
  }

  double rEps() const { return fit_.reps; }

  void rEps(double reps) {
    if (reps > 0) fit_.reps = reps;
  }

  Value valueAEps(const Value& aeps = Value::NAV()) {
    if (aeps.type()) aEps(aeps.getDouble());
    return fit_.aeps;
  }

  double aEps() const { return fit_.aeps; }

  void aEps(double aeps) {
    if (aeps > 0) fit_.aeps = aeps;
  }
};
}  // namespace lsfm
