//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file spe.hpp
/// @brief Sub-pixel edge localization estimators.
/// Provides various sub-pixel edge localization methods (Linear, Quadratic, CoG, Sobel)
/// that refine edge positions to sub-pixel accuracy using local magnitude derivatives.

#pragma once

#include <edge/edge_segment.hpp>
#include <edge/index.hpp>
#include <imgproc/interpolate.hpp>

#include <vector>

namespace lsfm {

/// @brief Linear sub-pixel edge localization estimator.
/// Uses linear interpolation (pyramid fitting) to refine edge position to sub-pixel accuracy.
/// @tparam FT Floating-point type for output
/// @tparam MT Magnitude/data element type
template <class FT, class MT>
struct LinearEstimate {
  /// @typedef mag_type
  /// @brief Magnitude element type
  typedef MT mag_type;

  /// @typedef float_type
  /// @brief Floating-point output type
  typedef FT float_type;

  /// @brief Estimate sub-pixel displacement along an edge direction.
  /// Uses linear (pyramid) fitting on three neighboring magnitude values.
  /// @param p Central pixel magnitude
  /// @param p_m Magnitude of pixel in negative direction
  /// @param p_p Magnitude of pixel in positive direction
  /// @return Sub-pixel displacement offset (typically in range [-0.5, 0.5])
  static inline FT estimate(MT p, MT p_m, MT p_p) { return (p_p - p_m) / (2 * (p - std::min(p_m, p_p))); }
};

/// @brief Quadratic (parabolic) sub-pixel edge localization estimator.
/// Fits a parabola to three neighboring points for high-accuracy sub-pixel localization.
/// @tparam FT Floating-point type for output
/// @tparam MT Magnitude/data element type
template <class FT, class MT>
struct QuadraticEstimate {
  /// @typedef mag_type
  /// @brief Magnitude element type
  typedef MT mag_type;

  /// @typedef float_type
  /// @brief Floating-point output type
  typedef FT float_type;

  /// @brief Estimate sub-pixel displacement using parabolic fitting.
  /// Fits a parabola to three neighboring magnitude values.
  /// @param p Central pixel magnitude
  /// @param p_m Magnitude of pixel in negative direction
  /// @param p_p Magnitude of pixel in positive direction
  /// @return Sub-pixel displacement offset
  static inline FT estimate(MT p, MT p_m, MT p_p) { return (p_p - p_m) / (4 * p - 2 * (p_p + p_m)); }
};

/// @brief Center-of-Gravity (CoG) sub-pixel edge localization estimator.
/// Uses weighted center of gravity of neighboring magnitudes for localization.
/// @tparam FT Floating-point type for output
/// @tparam MT Magnitude/data element type
template <class FT, class MT>
struct CoGEstimate {
  /// @typedef mag_type
  /// @brief Magnitude element type
  typedef MT mag_type;

  /// @typedef float_type
  /// @brief Floating-point output type
  typedef FT float_type;

  /// @brief Estimate sub-pixel displacement using center of gravity.
  /// Computes weighted center of mass of three neighboring pixels.
  /// @param p Central pixel magnitude
  /// @param p_m Magnitude of pixel in negative direction
  /// @param p_p Magnitude of pixel in positive direction
  /// @return Sub-pixel displacement offset
  static inline FT estimate(MT p, MT p_m, MT p_p) { return (p_p - p_m) / (p + p_p + p_m - 3 * std::min(p_m, p_p)); }
};

/// @brief Sobel-based sub-pixel edge localization estimator.
/// Uses Sobel-like derivative-based approach for sub-pixel refinement.
/// @tparam FT Floating-point type for output
/// @tparam MT Magnitude/data element type
template <class FT, class MT>
struct SobelEstimate {
  /// @typedef mag_type
  /// @brief Magnitude element type
  typedef MT mag_type;

  /// @typedef float_type
  /// @brief Floating-point output type
  typedef FT float_type;

  /// @brief Estimate sub-pixel displacement using Sobel-like method.
  /// Uses symmetric difference divided by central magnitude.
  /// @param p Central pixel magnitude
  /// @param p_m Magnitude of pixel in negative direction
  /// @param p_p Magnitude of pixel in positive direction
  /// @return Sub-pixel displacement offset
  static inline FT estimate(MT p, MT p_m, MT p_p) { return (p_p - p_m) / (2 * p); }
};

/// @brief Sobel-based zero-crossing sub-pixel edge localization estimator.
/// Specialized for zero-crossing detection using Laplacian values.
/// @tparam FT Floating-point type for output
/// @tparam LT Laplacian/data element type
template <class FT, class LT>
struct SobelZCEstimate {
  /// @typedef laplace_type
  /// @brief Laplacian element type
  typedef LT laplace_type;

  /// @typedef float_type
  /// @brief Floating-point output type
  typedef FT float_type;

  /// @brief Estimate sub-pixel zero-crossing position using Sobel method.
  /// Interpolates between sign changes in Laplacian values.
  /// @param p Central pixel Laplacian value
  /// @param p_m Laplacian of pixel in negative direction
  /// @param p_p Laplacian of pixel in positive direction
  /// @return Sub-pixel offset to zero-crossing
  static inline FT estimate(LT p, LT p_m, LT p_p) {
    return static_cast<FT>(p) / (neg_sign(p, p_p) ? (p - p_p) : (p_m - p));
  }
};

/// @brief Sub-pixel edge position estimator.
/// Refines discrete edge positions to sub-pixel accuracy using local magnitude
/// profiles. Supports multiple estimation methods (linear, quadratic, CoG, Sobel)
/// and interpolation strategies.
/// @tparam FT Floating-point output type
/// @tparam MT Magnitude element type
/// @tparam PT Point template (default: Vec2)
/// @tparam Estimator Sub-pixel estimation method (default: QuadraticEstimate)
/// @tparam Interpolator Interpolation method for sub-pixel access (default: LinearInterpolator)
template <class FT,
          class MT,
          template <class> class PT = Vec2,
          template <class, class> class Estimator = QuadraticEstimate,
          template <class, class> class Interpolator = LinearInterpolator>
struct SubPixelEstimator {
  /// @typedef mag_type
  /// @brief Magnitude element type
  typedef MT mag_type;

  /// @typedef float_type
  /// @brief Floating-point output type
  typedef FT float_type;

  /// @typedef point_type_int
  /// @brief Integer point type
  typedef PT<int> point_type_int;

  /// @typedef point_type_float
  /// @brief Floating-point point type
  typedef PT<FT> point_type_float;

  /// @typedef Pointi
  /// @brief Alias for integer point
  typedef PT<int> Pointi;

  /// @typedef Pointf
  /// @brief Alias for floating-point point
  typedef PT<FT> Pointf;

  /// @typedef estimator_type
  /// @brief The underlying estimation method
  typedef Estimator<FT, MT> estimator_type;

  /// @typedef interpolator_type
  /// @brief The interpolation method for sub-pixel magnitude access
  typedef Interpolator<FT, MT> interpolator_type;

  /// @brief Estimate sub-pixel position from integer point and direction.
  /// @param p Integer coordinates of central point
  /// @param dir Relative direction vector to neighboring pixels
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(const Pointi& p, const Pointi& dir, const cv::Mat& mag, Pointf& res) {
    Pointf dirf(getX(dir), getY(dir)), pf(getX(p), getY(p));
    res = pf + dirf * estimate(get<MT>(mag, p), get<MT>(mag, p - dir), get<MT>(mag, p + dir));
  }


  /// @brief Estimate sub-pixel position from float point and direction.
  /// Uses interpolation for sub-pixel magnitude access.
  /// @param p Floating-point coordinates of central point
  /// @param dir Direction vector to neighboring pixels
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(const Pointf& p, const Pointf& dir, const cv::Mat& mag, Pointf& res) {
    res = p + dir * estimate(interpolator_type::getNB(mag, p), interpolator_type::getNB(mag, p - dir),
                             interpolator_type::getNB(mag, p + dir));
  }


  /// @brief Estimate sub-pixel position from integer point and explicit neighbors.
  /// @param p Integer central point
  /// @param p_m Point in negative direction
  /// @param p_p Point in positive direction
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(const Pointi& p, const Pointi& p_m, const Pointi& p_p, const cv::Mat& mag, Pointf& res) {
    Pointi dir(p_p - p);
    Pointf pf(getX(p), getY(p)), dirf(getX(dir), getY(dir));
    res = pf + dirf * estimate(get<MT>(mag, p), get<MT>(mag, p_m), get<MT>(mag, p_p));
  }


  /// @brief Estimate sub-pixel position from float point and explicit neighbors.
  /// Uses interpolation for sub-pixel magnitude access.
  /// @param p Floating-point central point
  /// @param p_m Point in negative direction
  /// @param p_p Point in positive direction
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(const Pointf& p, const Pointf& p_m, const Pointf& p_p, const cv::Mat& mag, Pointf& res) {
    res = p + (p_p - p) * estimate(interpolator_type::getNB(mag, p), interpolator_type::getNB(mag, p_m),
                                   interpolator_type::getNB(mag, p_p));
  }

  /// @brief Estimate sub-pixel position from pixel indices and explicit neighbors.
  /// @param p Index of central point
  /// @param p_m Index of point in negative direction
  /// @param p_p Index of point in positive direction
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(index_type p, index_type p_m, index_type p_p, const cv::Mat& mag, Pointf& res) {
    Pointf pf, p_pf;
    index2Point(p, pf, mag.cols);
    index2Point(p_p, p_pf, mag.cols);

    res = pf + (p_pf - pf) * estimate(mag.ptr<MT>()[p], mag.ptr<MT>()[p_m], mag.ptr<MT>()[p_p]);
  }


  /// @brief Estimate sub-pixel position from index and integer direction.
  /// @param p Index of central point
  /// @param dir Integer direction vector to neighbors
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(index_type p, const Pointi& dir, const cv::Mat& mag, Pointf& res) {
    Pointi p2;
    index2Point(p, p2, mag.cols);
    Pointf pf(getX(p2), getY(p2)), dirf(getX(dir), getY(dir));

    res = pf + dirf * estimate(mag.ptr<MT>()[p], get<MT>(mag, p2 - dir), get<MT>(mag, p2 + dir));
  }


  /// @brief Estimate sub-pixel position from index and float direction.
  /// @param p Index of central point
  /// @param dir Float direction vector to neighbors
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(index_type p, const Pointf& dir, const cv::Mat& mag, Pointf& res) {
    Pointf pf;
    index2Point(p, pf, mag.cols);
    res = pf + dir * estimate(mag.ptr<MT>()[p], interpolator_type::getNB(mag, pf - dir),
                              interpolator_type::getNB(mag, pf + dir));
  }


  /// @brief Estimate sub-pixel position from index and angle in radians.
  /// @param p Index of central point
  /// @param dir Angle in radians defining the gradient direction
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(index_type p, FT dir, const cv::Mat& mag, Pointf& res) {
    Pointf pf, dirf(std::cos(dir), std::sin(dir));
    index2Point(p, pf, mag.cols);

    res = pf + dirf * estimate(mag.ptr<MT>()[p], interpolator_type::getNB(mag, pf - dirf),
                               interpolator_type::getNB(mag, pf + dirf));
  }


  /// @brief Estimate sub-pixel position from integer point and angle.
  /// @param p Integer point coordinates
  /// @param dir Angle in radians
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(const Pointi& p, FT dir, const cv::Mat& mag, Pointf& res) {
    int w = mag.cols;

    Pointf pf(getX(p), getY(p)), dirf(std::cos(dir), std::sin(dir));

    res = pf + dirf * estimate(get<MT>(mag, p), interpolator_type::getNB(mag, pf - dirf),
                               interpolator_type::getNB(mag, pf + dirf));
  }


  /// @brief Estimate sub-pixel position from float point and angle.
  /// @param p Float point coordinates
  /// @param dir Angle in radians
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(const Pointf& p, FT dir, const cv::Mat& mag, Pointf& res) {
    int w = mag.cols;

    Pointf dirf(std::cos(dir), std::sin(dir));

    res = p + dirf * estimate(interpolator_type::getNB(mag, p), interpolator_type::getNB(mag, p - dirf),
                              interpolator_type::getNB(mag, p + dirf));
  }

  /// @brief Estimate sub-pixel position from index and quantized direction.
  /// @param p Index of central point
  /// @param dir Quantized direction (0-7 for 8 orientations)
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(index_type p, char dir, const cv::Mat& mag, Pointf& res) {
    Pointi p2, dir2;
    index2Point(p, p2, mag.cols);
    dir2Vec4(dir + 2, dir2);
    Pointf pf(static_cast<float>(getX(p2)), static_cast<float>(getY(p2))),
        dirf(static_cast<float>(getX(dir2)), static_cast<float>(getY(dir2)));

    res = pf + dirf * estimate(mag.ptr<MT>()[p], get<MT>(mag, p2 - dir2), get<MT>(mag, p2 + dir2));
  }

  /// @brief Estimate sub-pixel position from integer point and quantized direction.
  /// @param p Integer point coordinates
  /// @param dir Quantized direction (0-7)
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(const Pointi& p, char dir, const cv::Mat& mag, Pointf& res) {
    Pointi dir2;
    dir2Vec4(dir + 2, dir2);
    Pointf pf(getX(p), getY(p)), dirf(getX(dir2), getY(dir2));

    res = pf + dirf * estimate(get<MT>(mag, p), get<MT>(mag, p - dir2), get<MT>(mag, p + dir2));
  }

  /// @brief Estimate sub-pixel position from float point and quantized direction.
  /// @param p Float point coordinates
  /// @param dir Quantized direction (0-7)
  /// @param mag Magnitude map
  /// @param res Output sub-pixel refined point
  static inline void estimate(const Pointf& p, char dir, const cv::Mat& mag, Pointf& res) {
    Pointf dirf;
    dir2Vec4(dir + 2, dirf);

    res = p + dirf * estimate(interpolator_type::getNB(mag, p), interpolator_type::getNB(mag, p - dirf),
                              interpolator_type::getNB(mag, p + dirf));
  }

  /// @brief Apply estimator to three magnitude values.
  /// Core worker function that delegates to the underlying Estimator.
  /// @param p Central pixel magnitude
  /// @param p_m Magnitude in negative direction
  /// @param p_p Magnitude in positive direction
  /// @return Sub-pixel displacement offset
  static inline FT estimate(MT p, MT p_m, MT p_p) { return estimator_type::estimate(p, p_m, p_p); }

  /// @brief Convert integer points to sub-pixel using direction map.
  /// @param beg Start of integer point array
  /// @param end End of integer point array
  /// @param data Output float point array
  /// @param mag Magnitude map
  /// @param dmap Quantized direction map (CV_8S)
  static inline void convertP(
      const Pointi* beg, const Pointi* end, Pointf* data, const cv::Mat& mag, const cv::Mat& dmap) {
    char dir;
    for (; beg != end; ++beg, ++data) {
      dir = get<char>(dmap, *beg);
      if (dir < 0) {
        *data = Pointf(0, 0);
        continue;
      }
      estimate(*beg, dir, mag, *data);
    }
  }

  template <class IVT, class VT>
  static inline void convertP(const IVT& pts, VT& res, const cv::Mat& mag, const cv::Mat& dmap) {
    if (pts.empty()) return;
    res.resize(pts.size());
    convertP(&pts[0], &pts[0] + pts.size(), &res[0], mag, dmap);
  }

  template <class IVT, class VT>
  static inline VT convertP(const IVT& pts, const cv::Mat& mag, const cv::Mat& dmap) {
    VT res;
    convertP(pts, res, mag, dmap);
    return res;
  }

  static inline void convertPDir(
      const Pointi* beg, const Pointi* end, Pointf* data, const cv::Mat& mag, const cv::Mat& dir) {
    FT d;
    for (; beg != end; ++beg, ++data) {
      d = get<FT>(dir, *beg);
      estimate(*beg, d, mag, *data);
    }
  }

  template <class IVT, class VT>
  static inline void convertPDir(const IVT& pts, VT& res, const cv::Mat& mag, const cv::Mat& dir) {
    if (pts.empty()) return;
    res.resize(pts.size());
    convertPDir(&pts[0], &pts[0] + pts.size(), &res[0], mag, dir);
  }

  template <class IVT, class VT>
  static inline VT convertPDir(const IVT& pts, const cv::Mat& mag, const cv::Mat& dir) {
    VT res;
    convertPDir(pts, res, mag, dir);
    return res;
  }

  static inline void convert(
      const index_type* beg, const index_type* end, Pointf* data, const cv::Mat& mag, const cv::Mat& dmap) {
    char dir;
    for (; beg != end; ++beg, ++data) {
      dir = get<char>(dmap, *beg);
      if (dir < 0) {
        *data = Pointf(0, 0);
        continue;
      }
      estimate(*beg, dir, mag, *data);
    }
  }

  template <class IVT, class VT>
  static inline void convert(const IVT& idxs, VT& res, const cv::Mat& mag, const cv::Mat& dmap) {
    if (idxs.empty()) return;
    res.resize(idxs.size());
    convert(&idxs[0], &idxs[0] + idxs.size(), &res[0], mag, dmap);
  }

  template <class IVT, class VT>
  static inline VT convert(const IVT& idxs, const cv::Mat& mag, const cv::Mat& dmap) {
    VT res;
    convert(idxs, res, mag, dmap);
    return res;
  }

  static inline void convertDir(
      const index_type* beg, const index_type* end, Pointf* data, const cv::Mat& mag, const cv::Mat& dir) {
    FT d;
    for (; beg != end; ++beg, ++data) {
      d = get<FT>(dir, *beg);
      estimate(*beg, d, mag, *data);
    }
  }

  template <class IVT, class VT>
  static inline void convertDir(const IVT& idxs, VT& res, const cv::Mat& mag, const cv::Mat& dir) {
    if (idxs.empty()) return;
    res.resize(idxs.size());
    convertDir(&idxs[0], &idxs[0] + idxs.size(), &res[0], mag, dir);
  }

  template <class IVT, class VT>
  static inline VT convertDir(const IVT& idxs, const cv::Mat& mag, const cv::Mat& dir) {
    VT res;
    convertDir(idxs, res, mag, dir);
    return res;
  }
};

/// @brief Pixel-level edge position estimator (no sub-pixel refinement).
/// Simply converts indices to points without any sub-pixel adjustment.
/// @tparam FT Floating-point type (unused but kept for interface compatibility)
/// @tparam PT Point type (default: Vec2i)
/// @tparam C Index-to-point converter type
template <class FT, class PT = Vec2i, class C = IndexConvert<PT>>
struct PixelEstimator {
  /// @typedef float_type
  /// @brief Floating-point type
  typedef FT float_type;

  /// @typedef point_type
  /// @brief Point type
  typedef PT point_type;

  /// @typedef converter_type
  /// @brief Index-to-point converter
  typedef C converter_type;

  static inline void convert(
      const index_type* beg, const index_type* end, point_type* data, const cv::Mat& mag, const cv::Mat& dmap) {
    converter_type::convert(beg, end, data, mag, dmap);
  }

  template <class IVT, class VT>
  static inline void convert(const IVT& idxs, VT& res, const cv::Mat& mag, const cv::Mat& dmap) {
    if (idxs.empty()) return;
    res.resize(idxs.size());
    convert(&idxs[0], &idxs[0] + idxs.size(), &res[0], mag, dmap);
  }

  template <class IVT, class VT>
  static inline VT convert(const IVT& idxs, const cv::Mat& mag, const cv::Mat& dmap) {
    VT res;
    convert(idxs, res, mag, dmap);
    return res;
  }

  static inline void convertDir(
      const index_type* beg, const index_type* end, point_type* data, const cv::Mat& mag, const cv::Mat& dir) {
    converter_type::convertDir(beg, end, data, mag, dir);
  }

  template <class IVT, class VT>
  static inline void convertDir(const IVT& idxs, VT& res, const cv::Mat& mag, const cv::Mat& dir) {
    if (idxs.empty()) return;
    res.resize(idxs.size());
    convertDir(&idxs[0], &idxs[0] + idxs.size(), &res[0], mag, dir);
  }

  template <class IVT, class VT>
  static inline VT convertDir(const IVT& idxs, const cv::Mat& mag, const cv::Mat& dir) {
    VT res;
    convertDir(idxs, res, mag, dir);
    return res;
  }
};

}  // namespace lsfm
