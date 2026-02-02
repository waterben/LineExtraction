/*M///////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// C by Benjamin Wassermann
//M*/

#pragma once

#include <geometry/line.hpp>
#include <imgproc/interpolate.hpp>

namespace lsfm {

/// @brief Compute mean value along a line segment with sub-pixel interpolation.
/// This struct provides methods to sample pixel values along a line segment and
/// compute statistical measures (mean, variance). The sampling uses a configurable
/// interpolator for sub-pixel accuracy and walks along the line at specified
/// distance intervals.
/// @tparam FT Floating-point type for calculations (e.g., float, double).
/// @tparam MT Matrix element type for the input image.
/// @tparam IP Interpolator type (defaults to LinearInterpolator).
template <class FT, class MT, class IP = LinearInterpolator<FT, MT>>
struct Mean {
  typedef FT float_type;          ///< Floating-point type for calculations.
  typedef MT mat_type;            ///< Matrix element type.
  typedef IP interpolation_type;  ///< Interpolation policy type.

  /// @brief Compute mean value along a line segment.
  /// Samples the image at regular intervals along the line segment and returns
  /// the arithmetic mean of all sampled values. The line is trimmed to fit
  /// within the image boundaries before sampling.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] dist Distance between sample points (default: 1 pixel).
  /// @return Mean value of all samples along the line.
  template <template <class> class PT>
  static inline FT process(const cv::Mat& mag, const LineSegment<FT, PT>& l, FT dist = 1) {
    CV_Assert(cv::DataType<MT>::type == mag.type());

    // make sure line is not ouside of mat
    LineSegment<FT, PT> tl = l;
    tl.trim2Box(static_cast<FT>(mag.cols - IP::BorderEnd), static_cast<FT>(mag.rows - IP::BorderEnd),
                static_cast<FT>(IP::BorderStart), static_cast<FT>(IP::BorderStart));

    FT length = tl.length();
    FT offset = static_cast<FT>(fmod(length, dist)) / static_cast<FT>(2);
    PT<FT> start = tl.start() > tl.end() ? tl.endPoint() : tl.startPoint();
    FT ret = 0;
    int n = 0;
    for (; offset <= length; offset += dist) {
      ret += IP::getNB(mag, tl.lineDist(offset, start));
      ++n;
    }

    // normalize
    if (n != 0) ret /= static_cast<FT>(n);
    return ret;
  }

  /// @brief Compute mean value and variance along a line segment.
  /// Samples the image at regular intervals along the line segment and computes
  /// both the arithmetic mean and variance of all sampled values.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[out] variance Computed variance of the sampled values.
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] dist Distance between sample points (default: 1 pixel).
  /// @return Mean value of all samples along the line.
  template <template <class> class PT>
  static inline FT process(FT& variance, const cv::Mat& mag, const LineSegment<FT, PT>& l, FT dist = 1) {
    CV_Assert(cv::DataType<MT>::type == mag.type());

    // make sure line is not ouside of mat
    LineSegment<FT, PT> tl = l;
    tl.trim2Box(static_cast<FT>(mag.cols - IP::BorderEnd), static_cast<FT>(mag.rows - IP::BorderEnd),
                static_cast<FT>(IP::BorderStart), static_cast<FT>(IP::BorderStart));

    FT length = tl.length();
    std::vector<FT> vals;
    vals.reserve(static_cast<size_t>(length / dist));
    FT offset = static_cast<FT>(fmod(length, dist)) / static_cast<FT>(2);
    PT<FT> start = tl.start() > tl.end() ? tl.endPoint() : tl.startPoint();
    FT ret = 0, ret2 = 0, val;
    variance = 0;
    int n = 0;
    for (; offset <= length; offset += dist) {
      val = IP::getNB(mag, tl.lineDist(offset, start));
      ret += val;
      ret2 += val * val;
      ++n;
    }

    if (n != 0) {
      // normalize
      ret /= static_cast<FT>(n);
      variance = ret2 / static_cast<FT>(n) - ret * ret;
    }
    return ret;
  }

  /// @brief Collect all sample values along a line segment.
  /// Samples the image at regular intervals along the line segment and stores
  /// all values in the provided container.
  /// @tparam V Container type for output values (e.g., std::vector<FT>).
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[out] ret Output container to store sampled values.
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] dist Distance between sample points (default: 1 pixel).
  template <class V, template <class> class PT>
  static inline void process(V& ret, const cv::Mat& mag, const LineSegment<FT, PT>& l, FT dist = 1) {
    CV_Assert(cv::DataType<MT>::type == mag.type());

    // make sure line is not ouside of mat
    LineSegment<FT, PT> tl = l;
    tl.trim2Box(static_cast<FT>(mag.cols - IP::BorderEnd), static_cast<FT>(mag.rows - IP::BorderEnd),
                static_cast<FT>(IP::BorderStart), static_cast<FT>(IP::BorderStart));

    FT length = tl.length();
    ret.clear();
    ret.reserve(static_cast<size_t>(length));
    FT offset = fmod(length, dist) / 2;
    PT<FT> start = tl.start() > tl.end() ? tl.endPoint() : tl.startPoint();
    for (; offset <= length; offset += dist) {
      ret.push_back(IP::get(mag, tl.lineDist(offset, start)));
    }
  }
};

/// @brief Compute mean value along a line segment using sample count.
/// This struct wraps Mean to provide sampling based on a specified number of
/// samples rather than a fixed distance. The distance between samples is
/// computed from the line length and desired sample count.
/// @tparam FT Floating-point type for calculations (e.g., float, double).
/// @tparam MT Matrix element type for the input image.
/// @tparam IP Interpolator type (defaults to LinearInterpolator).
template <class FT, class MT, class IP = LinearInterpolator<FT, MT>>
struct MeanSampled {
  typedef FT float_type;          ///< Floating-point type for calculations.
  typedef MT mat_type;            ///< Matrix element type.
  typedef IP interpolation_type;  ///< Interpolation policy type.

  /// @brief Compute mean value along a line segment.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] samples Number of samples to take along the line (default: 20).
  /// @return Mean value of all samples along the line.
  template <template <class> class PT>
  static inline FT process(const cv::Mat& mag, const LineSegment<FT, PT>& l, FT samples = 20) {
    return Mean<FT, MT, IP>::process(mag, l, l.length() / floor(samples) - 1);
  }

  /// @brief Compute mean value and variance along a line segment.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[out] variance Computed variance of the sampled values.
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] samples Number of samples to take along the line (default: 20).
  /// @return Mean value of all samples along the line.
  template <template <class> class PT>
  static inline FT process(FT& variance, const cv::Mat& mag, const LineSegment<FT, PT>& l, FT samples = 20) {
    return Mean<FT, MT, IP>::process(variance, mag, l, l.length() / floor(samples) - 1);
  }

  /// @brief Collect all sample values along a line segment.
  /// @tparam V Container type for output values.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[out] ret Output container to store sampled values.
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] samples Number of samples to take along the line (default: 20).
  template <class V, template <class> class PT>
  static inline void process(V& ret, const cv::Mat& mag, const LineSegment<FT, PT>& l, FT samples = 20) {
    Mean<FT, MT, IP>::process(ret, mag, l, l.length() / floor(samples) - 1);
  }
};

/// @brief Fast mean computation along a line segment using axis-aligned traversal.
/// This struct provides an optimized implementation of line sampling by walking
/// along the dominant axis (X or Y) rather than walking along the actual line
/// direction. This is faster because it avoids computing line distances for
/// each sample point.
/// @note This implementation may produce slightly different results than Mean
/// for diagonal lines due to the axis-aligned sampling strategy.
/// @tparam FT Floating-point type for calculations (e.g., float, double).
/// @tparam MT Matrix element type for the input image.
/// @tparam IP Interpolator type (defaults to LinearInterpolator).
template <class FT, class MT, class IP = LinearInterpolator<FT, MT>>
struct FastMean {
  typedef FT float_type;          ///< Floating-point type for calculations.
  typedef MT mat_type;            ///< Matrix element type.
  typedef IP interpolation_type;  ///< Interpolation policy type.

  /// @brief Compute mean value along a line segment using fast axis-aligned traversal.
  /// Walks along the dominant axis direction and interpolates the perpendicular
  /// coordinate for each sample. This is faster than walking along the actual
  /// line direction.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] dist Distance between sample points in pixels (default: 1).
  /// @return Mean value of all samples along the line.
  template <template <class> class PT>
  static inline FT process(const cv::Mat& mag, const LineSegment<FT, PT>& l, FT dist = 1) {
    CV_Assert(cv::DataType<MT>::type == mag.type());

    int step = static_cast<int>(dist);
    if (step < 1) step = 1;
    FT ret = 0, nstep, nx = fabs(l.normalX()), ny = fabs(l.normalY());
    int n = 0, cols = mag.cols - IP::BorderEnd, rows = mag.rows - IP::BorderEnd,
        matStepY = static_cast<int>(mag.step[0] / sizeof(MT));
    LineSegment<FT, PT> tl = l;
    // make sure line is not ouside of box
    tl.trim2Box(cols, rows, IP::BorderStart, IP::BorderStart);
    PT<FT> start = l.startPoint(), end = l.endPoint();

    const MT* data = nullptr;
    if (nx > ny) {
      if (getX(start) < 1) {
        getX(start) = 1;
        getY(start) = l.y(1);
      } else if (getX(start) > cols) {
        getX(start) = cols;
        getY(start) = l.y(cols);
      }

      if (getX(end) < 1) {
        getX(end) = 1;
        getY(end) = l.y(1);
      } else if (getX(end) > cols) {
        getX(end) = cols;
        getY(end) = l.y(cols);
      }

      if (getY(start) > getY(end)) std::swap(start, end);

      nstep = ny / nx;
      if (getX(start) > getX(end)) nstep = -nstep;

      int ybeg = static_cast<int>(getY(start) + 0.5), yend = static_cast<int>(getY(end) + 0.5);
      if (ybeg < 1) ybeg = 1;
      if (ybeg > rows) ybeg = rows;

      if (yend < 1) yend = 1;
      if (yend > rows) yend = rows;

      ybeg += ((yend - ybeg) % step) / 2;
      data = mag.template ptr<MT>(ybeg);
      FT xpos = l.x(ybeg);
      matStepY *= step;
      for (; ybeg < yend; ybeg += step, data += matStepY, xpos += nstep, ++n) {
        ret += IP::getXNB(data, xpos);
      }
    } else {
      if (getY(start) < 1) {
        getY(start) = 1;
        getX(start) = l.x(1);
      } else if (getY(start) > rows) {
        getY(start) = rows;
        getX(start) = l.x(rows);
      }

      if (getY(end) < 1) {
        getY(end) = 1;
        getX(end) = l.x(1);
      } else if (getY(end) > rows) {
        getY(end) = rows;
        getX(end) = l.x(rows);
      }

      if (getX(start) > getX(end)) std::swap(start, end);

      nstep = nx / ny;
      if (getY(start) > getY(end)) nstep = -nstep;

      int xbeg = static_cast<int>(getX(start) + 0.5), xend = static_cast<int>(getX(end) + 0.5);
      if (xbeg < 1) xbeg = 1;
      if (xbeg > cols) xbeg = cols;

      if (xend < 1) xend = 1;
      if (xend > cols) xend = cols;

      xbeg += ((xend - xbeg) % step) / 2;
      data = mag.template ptr<MT>(0, xbeg);
      FT ypos = l.y(xbeg);
      for (; xbeg < xend; xbeg += step, data += step, ypos += nstep, ++n) {
        ret += IP::getYNB(data, matStepY, ypos);
      }
    }

    // normalize
    if (n != 0) ret /= n;
    return ret;
  }

  /// @brief Compute mean and variance along a line using fast axis-aligned traversal.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[out] variance Computed variance of the sampled values.
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] dist Distance between sample points in pixels (default: 1).
  /// @return Mean value of all samples along the line.
  template <template <class> class PT>
  static inline FT process(FT& variance, const cv::Mat& mag, const LineSegment<FT, PT>& l, FT dist = 1) {
    CV_Assert(cv::DataType<MT>::type == mag.type());

    int step = static_cast<int>(dist);
    if (step < 1) step = 1;
    FT ret = 0, ret2 = 0, nstep, tmp, nx = fabs(l.normalX()), ny = fabs(l.normalY());
    int n = 0;
    const int cols = mag.cols - 2;
    const int rows = mag.rows - 2;
    int matStepY = static_cast<int>(mag.step[0] / sizeof(MT));
    PT<FT> start = l.startPoint(), end = l.endPoint();
    const MT* data = nullptr;
    if (nx > ny) {
      if (getX(start) < 1) {
        getX(start) = 1;
        getY(start) = l.y(1);
      } else if (getX(start) > cols) {
        getX(start) = cols;
        getY(start) = l.y(cols);
      }

      if (getX(end) < 1) {
        getX(end) = 1;
        getY(end) = l.y(1);
      } else if (getX(end) > cols) {
        getX(end) = cols;
        getY(end) = l.y(cols);
      }

      if (getY(start) > getY(end)) std::swap(start, end);

      nstep = ny / nx;
      if (getX(start) > getX(end)) nstep = -nstep;

      int ybeg = static_cast<int>(getY(start) + 0.5), yend = static_cast<int>(getY(end) + 0.5);
      if (ybeg < 1) ybeg = 1;
      if (ybeg > rows) ybeg = rows;

      if (yend < 1) yend = 1;
      if (yend > rows) yend = rows;

      ybeg += ((yend - ybeg) % step) / 2;
      data = mag.template ptr<MT>(ybeg);
      FT xpos = l.x(ybeg);
      matStepY *= step;
      for (; ybeg < yend; ybeg += step, data += matStepY, xpos += nstep, ++n) {
        tmp = IP::getXNB(data, xpos);
        ret += tmp;
        ret2 += tmp * tmp;
      }
    } else {
      if (getY(start) < 1) {
        getY(start) = 1;
        getX(start) = l.x(1);
      } else if (getY(start) > rows) {
        getY(start) = rows;
        getX(start) = l.x(rows);
      }

      if (getY(end) < 1) {
        getY(end) = 1;
        getX(end) = l.x(1);
      } else if (getY(end) > rows) {
        getY(end) = rows;
        getX(end) = l.x(rows);
      }

      if (getX(start) > getX(end)) std::swap(start, end);

      nstep = nx / ny;
      if (getY(start) > getY(end)) nstep = -nstep;

      int xbeg = static_cast<int>(getX(start) + 0.5), xend = static_cast<int>(getX(end) + 0.5);
      if (xbeg < 1) xbeg = 1;
      if (xbeg > cols) xbeg = cols;

      if (xend < 1) xend = 1;
      if (xend > cols) xend = cols;

      xbeg += ((xend - xbeg) % step) / 2;
      data = mag.template ptr<MT>(0, xbeg);
      FT ypos = l.y(xbeg);
      for (; xbeg < xend; xbeg += step, data += step, ypos += nstep, ++n) {
        tmp = IP::getYNB(data, matStepY, ypos);
        ret += tmp;
        ret2 += tmp * tmp;
      }
    }

    // normalize
    if (n != 0) {
      ret /= n;
      variance = ret2 / n - ret * ret;
    }
    return ret;
  }

  /// @brief Collect all sample values along a line using fast axis-aligned traversal.
  /// @tparam V Container type for output values (e.g., std::vector<FT>).
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[out] ret Output container to store sampled values.
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] dist Distance between sample points in pixels (default: 1).
  template <class V, template <class> class PT>
  static inline void process(V& ret, const cv::Mat& mag, const LineSegment<FT, PT>& l, FT dist = 1) {
    CV_Assert(cv::DataType<MT>::type == mag.type());

    ret.clear();
    int step = static_cast<int>(dist);
    if (step < 1) step = 1;
    FT nstep, nx = fabs(l.normalX()), ny = fabs(l.normalY());
    int cols = mag.cols - 2, rows = mag.rows - 2, matStepY = mag.step[0] / sizeof(MT);
    PT<FT> start = l.startPoint(), end = l.endPoint();
    const MT* data = nullptr;
    if (nx > ny) {
      if (getX(start) < 1) {
        getX(start) = 1;
        getY(start) = l.y(1);
      } else if (getX(start) > cols) {
        getX(start) = cols;
        getY(start) = l.y(cols);
      }

      if (getX(end) < 1) {
        getX(end) = 1;
        getY(end) = l.y(1);
      } else if (getX(end) > cols) {
        getX(end) = cols;
        getY(end) = l.y(cols);
      }

      if (getY(start) > getY(end)) std::swap(start, end);

      nstep = ny / nx;
      if (getX(start) > getX(end)) nstep = -nstep;

      int ybeg = static_cast<int>(getY(start) + 0.5), yend = static_cast<int>(getY(end) + 0.5);
      if (ybeg < 1) ybeg = 1;
      if (ybeg > rows) ybeg = rows;

      if (yend < 1) yend = 1;
      if (yend > rows) yend = rows;

      ybeg += ((yend - ybeg) % step) / 2;
      data = mag.template ptr<MT>(ybeg);
      FT xpos = l.x(ybeg);
      matStepY *= step;
      ret.reserve(yend - ybeg);
      for (; ybeg < yend; ybeg += step, data += matStepY, xpos += nstep) {
        ret.push_back(IP::getXNB(data, xpos));
      }
    } else {
      if (getY(start) < 1) {
        getY(start) = 1;
        getX(start) = l.x(1);
      } else if (getY(start) > rows) {
        getY(start) = rows;
        getX(start) = l.x(rows);
      }

      if (getY(end) < 1) {
        getY(end) = 1;
        getX(end) = l.x(1);
      } else if (getY(end) > rows) {
        getY(end) = rows;
        getX(end) = l.x(rows);
      }

      if (getX(start) > getX(end)) std::swap(start, end);

      nstep = nx / ny;
      if (getY(start) > getY(end)) nstep = -nstep;

      int xbeg = static_cast<int>(getX(start) + 0.5), xend = static_cast<int>(getX(end) + 0.5);
      if (xbeg < 1) xbeg = 1;
      if (xbeg > cols) xbeg = cols;

      if (xend < 1) xend = 1;
      if (xend > cols) xend = cols;

      xbeg += ((xend - xbeg) % step) / 2;
      data = mag.template ptr<MT>(0, xbeg);
      FT ypos = l.y(xbeg);
      ret.reserve(xend - xbeg);
      for (; xbeg < xend; xbeg += step, data += step, ypos += nstep) {
        ret.push_back(IP::getYNB(data, matStepY, ypos));
      }
    }
  }
};

/// @brief Fast mean computation along a line segment using sample count.
/// This struct wraps FastMean to provide sampling based on a specified number
/// of samples rather than a fixed distance.
/// @tparam FT Floating-point type for calculations (e.g., float, double).
/// @tparam MT Matrix element type for the input image.
/// @tparam IP Interpolator type (defaults to LinearInterpolator).
template <class FT, class MT, class IP = LinearInterpolator<FT, MT>>
struct FastMeanSampled {
  typedef FT float_type;          ///< Floating-point type for calculations.
  typedef MT mat_type;            ///< Matrix element type.
  typedef IP interpolation_type;  ///< Interpolation policy type.

  /// @brief Compute mean value along a line segment.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] samples Number of samples to take along the line (default: 20).
  /// @return Mean value of all samples along the line.
  template <template <class> class PT>
  static inline FT process(const cv::Mat& mag, const LineSegment<FT, PT>& l, FT samples = 20) {
    return FastMean<FT, MT, IP>::process(mag, l, l.length() / floor(samples) - 1);
  }

  /// @brief Compute mean value and variance along a line segment.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[out] variance Computed variance of the sampled values.
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] samples Number of samples to take along the line (default: 20).
  /// @return Mean value of all samples along the line.
  template <template <class> class PT>
  static inline FT process(FT& variance, const cv::Mat& mag, const LineSegment<FT, PT>& l, FT samples = 20) {
    return FastMean<FT, MT, IP>::process(variance, mag, l, l.length() / floor(samples) - 1);
  }

  /// @brief Collect all sample values along a line segment.
  /// @tparam V Container type for output values.
  /// @tparam PT Point template type (e.g., Vec2).
  /// @param[out] ret Output container to store sampled values.
  /// @param[in] mag Input image (magnitude or intensity).
  /// @param[in] l Line segment to sample along.
  /// @param[in] samples Number of samples to take along the line (default: 20).
  template <class V, template <class> class PT>
  static inline void process(V& ret, const cv::Mat& mag, const LineSegment<FT, PT>& l, FT samples = 20) {
    FastMean<FT, MT, IP>::process(ret, mag, l, l.length() / floor(samples) - 1);
  }
};


/// @brief Helper struct for obtaining function pointers to mean computation functions.
/// This helper provides type aliases for function pointers to the templated mean
/// functions, enabling their use in optimization routines that require function
/// pointers as parameters.
/// @tparam FT Floating-point type for calculations.
/// @tparam PT Point template type (e.g., Vec2).
template <class FT, template <class> class PT>
struct MeanHelper {
  /// Function pointer type for mean computation.
  typedef FT (*func_type)(const cv::Mat&, const LineSegment<FT, PT>&, FT param);

  /// Function pointer type for mean computation with variance output.
  typedef FT (*func_type_variance)(FT&, const cv::Mat&, const LineSegment<FT, PT>&, FT param);
};

}  // namespace lsfm
