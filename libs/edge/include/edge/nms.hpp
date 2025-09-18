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

#ifndef _NMS_HPP_
#define _NMS_HPP_
#ifdef __cplusplus

#  include <imgproc/gradient.hpp>
#  include <imgproc/interpolate.hpp>
#  include <edge/hysteresis.hpp>
#  include <edge/index.hpp>
#  include <opencv2/core/core.hpp>

#  include <vector>


namespace lsfm {

// expected gradient direction are:
//           bright
//             ^ gy < 0->-pi/2
//             |
//     gx < 0  |  gx > 0->0
// bright <-- dark --> bright
//             |
//             |                             gradient direction of edge
//             v gy > 0->pi/2                              ^
//           bright                                        |
//                                                         |
// The gradient is orthogonal to the line direction  -------------> line direction
// line direction = gradiend direction + pi/2
// Since the positive y axis is from top to bottom, the positive rotation is clockwise
// For 8 region NMS we define following tags:
//
//  5   6   7
//   \  |  /
//    \ | /
//  4 --+-- 0
//    / | \
    //   /  |  \
    //  3   2   1
//
//
// The 4 region NMS only uses the positive part
//
//  1   2   3
//   \  |  /
//    \ | /
//  0 --+-- 0
//    / | \
    //   /  |  \
    //  3   2   1
//
// gradient to map ids:
//
//     -pi/8 to   pi/8 = 2 right
//      pi/8 to  3pi/8 = 3 right down
//     3pi/8 to  5pi/8 = 4 down
//     5pi/8 to  7pi/8 = 5 down left
//     7pi/8 to -7pi/8 = 6 left
//    -5pi/8 to -7pi/8 = 7 left up
//    -3pi/8 to -5pi/8 = 0 up
//     -pi/8 to -3pi/8 = 1 up right


template <class DT>
struct EMap8 {
  static constexpr int NUM_DIR = 4;
  static inline void map(char& dmap, DT xs, DT ys) {
    // we are within +-pi/8 at x
    if (std::abs(xs) > static_cast<DT>(0.923879533)) {
      dmap = (xs > 0 ? 2 : 6);
      // we are within +-pi/8 at y
    } else if (std::abs(ys) > static_cast<DT>(0.923879533)) {
      dmap = (ys > 0 ? 4 : 0);
    } else {
      if (xs > 0)
        dmap = (ys > 0 ? 3 : 1);
      else
        dmap = (ys > 0 ? 5 : 7);
    }
  }

  static inline const char type() { return '8'; }
};

template <class DT>
struct EMap4 {
  static constexpr int NUM_DIR = 4;
  static inline void map(char& dmap, DT xs, DT ys) {
    // we are within +-pi/8 at x
    if (std::abs(xs) > static_cast<DT>(0.923879533)) {
      dmap = 2;
      // we are within +-pi/8 at y
    } else if (std::abs(ys) > static_cast<DT>(0.923879533)) {
      dmap = 0;
    } else {
      dmap = (xs * ys > 0 ? 3 : 1);
    }
  }

  static inline const char type() { return '4'; }
};

template <class MT>
inline void setBorder(cv::Mat& mat, int borderStart, int borderEnd, MT val) {
  mat.rowRange(0, borderStart).setTo(val);
  mat.rowRange(mat.rows - borderEnd, mat.rows).setTo(val);
  mat.colRange(0, borderStart).setTo(val);
  mat.colRange(mat.cols - borderEnd, mat.cols).setTo(val);
}

template <class MT>
inline void setBorder(cv::Mat& mat, int border, MT val) {
  mat.rowRange(0, border).setTo(val);
  mat.rowRange(mat.rows - border, mat.rows).setTo(val);
  mat.colRange(0, border).setTo(val);
  mat.colRange(mat.cols - border, mat.cols).setTo(val);
}

template <class GT,
          class MT,
          bool SQR,
          class DT = float,
          template <class> class EM = EMap8,
          template <class, class> class Interpolate = LinearInterpolator,
          template <class, class> class P = Polar>
struct PreciseNMS {
  static constexpr int NUM_DIR = EM<DT>::NUM_DIR;

  // compute non maxima supression by given derivative maps and magnitude map
  static MT process(const cv::Mat& gx,
                    const cv::Mat& gy,
                    const cv::Mat& mag,
                    MT low,
                    MT high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    cv::Mat magf, gxf, gyf;
    if (static_cast<int>(cv::DataType<MT>::type) == static_cast<int>(cv::DataType<DT>::type))
      magf = mag;
    else {
      mag.convertTo(magf, cv::DataType<DT>::type);
    }

    if (SQR) {
      cv::sqrt(magf, magf);
    }

    if (static_cast<int>(cv::DataType<GT>::type) == static_cast<int>(cv::DataType<DT>::type)) {
      cv::divide(gx, magf, gxf);
      cv::divide(gy, magf, gyf);
    } else {
      gx.convertTo(gxf, cv::DataType<DT>::type);
      gy.convertTo(gyf, cv::DataType<DT>::type);

      cv::divide(gxf, magf, gxf);
      cv::divide(gyf, magf, gyf);
    }

    return processf(gxf, gyf, mag, low, high, seeds, dmap, border);
  }

  // compute non maxima supression by given derivative maps and magnitude map
  static MT process(const cv::Mat& gx,
                    const cv::Mat& gy,
                    const cv::Mat& mag,
                    const cv::Mat& low,
                    const cv::Mat& high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    cv::Mat magf, gxf, gyf;
    if (cv::DataType<MT>::type == cv::DataType<DT>::type)
      magf = mag;
    else {
      mag.convertTo(magf, cv::DataType<DT>::type);
    }

    if (SQR) {
      cv::sqrt(magf, magf);
    }

    if (cv::DataType<GT>::type == cv::DataType<DT>::type) {
      cv::divide(gx, magf, gxf);
      cv::divide(gy, magf, gyf);
    } else {
      gx.convertTo(gxf, cv::DataType<DT>::type);
      gy.convertTo(gyf, cv::DataType<DT>::type);

      cv::divide(gxf, magf, gxf);
      cv::divide(gyf, magf, gyf);
    }

    return processf(gxf, gyf, mag, low, high, seeds, dmap, border);
  }

  // compute non maxima supression by given direction map and magnitude map
  static MT process(const cv::Mat& dir,
                    const cv::Mat& mag,
                    MT low,
                    MT high,
                    DT r_low,
                    DT r_high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    static DT pi2 = static_cast<DT>(2 * CV_PI), epsilon = static_cast<DT>(0.01);
    DT r_size = r_high - r_low;
    cv::Mat dirf = dir, gxf, gyf;
    if (r_size > pi2 + epsilon || r_size < pi2 - epsilon) {
      dirf = dir * (pi2 / r_size);
    }

    P<DT, DT>::polar2Cart(dirf, gxf, gyf);
    return processf(gxf, gyf, mag, low, high, seeds, dmap, border);
  }

  // compute non maxima supression by given direction map and magnitude map
  static MT process(const cv::Mat& dir,
                    const cv::Mat& mag,
                    const cv::Mat& low,
                    const cv::Mat& high,
                    DT r_low,
                    DT r_high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    static DT pi2 = static_cast<DT>(2 * CV_PI), epsilon = static_cast<DT>(0.01);
    DT r_size = r_high - r_low;
    cv::Mat dirf = dir, gxf, gyf;
    if (r_size > pi2 + epsilon || r_size < pi2 - epsilon) {
      dirf = dir * (pi2 / r_size);
    }

    P<DT, DT>::polar2Cart(dirf, gxf, gyf);
    return processf(gxf, gyf, mag, low, high, seeds, dmap, border);
  }

  static inline std::string name() { return std::string("PreciseNMS") + EM<DT>::type(); }

  static MT processf(const cv::Mat& gx,
                     const cv::Mat& gy,
                     const cv::Mat& mag,
                     MT low,
                     MT high,
                     IndexVector& seeds,
                     cv::Mat& dmap,
                     int border = 1) {
    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8S);
    int borderStart = std::max(Interpolate<DT, MT>::BorderStart + 1, border),
        borderEnd = std::max(Interpolate<DT, MT>::BorderEnd + 1, border);
    setBorder<char>(dmap, borderStart, borderEnd, -1);

    if (low > high) std::swap(high, low);

    const MT* pmag = mag.ptr<MT>();
    const DT* pgx = gx.ptr<DT>();
    const DT* pgy = gy.ptr<DT>();
    char* pdmap = dmap.template ptr<char>();
    MT mag_max = 0;

    int last_row = mag.rows - borderEnd, idx, r_end;
    for (int r = borderStart; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - borderEnd);
      idx += borderStart;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > low) {
          DT xs = pgx[idx];
          DT ys = pgy[idx];

          size_t x = idx % mag.cols;
          size_t y = idx / mag.cols;

          if (m > Interpolate<DT, MT>::getNB(mag, x - xs, y - ys) &&
              m >= Interpolate<DT, MT>::getNB(mag, x + xs, y + ys)) {
            if (m > high) seeds.push_back(idx);

            EM<DT>::map(pdmap[idx], xs, ys);
          }
        }
      }
    }
    return mag_max;
  }

  static MT processf(const cv::Mat& gx,
                     const cv::Mat& gy,
                     const cv::Mat& mag,
                     const cv::Mat& low,
                     const cv::Mat& high,
                     IndexVector& seeds,
                     cv::Mat& dmap,
                     int border = 1) {
    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8S);
    int borderStart = Interpolate<DT, MT>::BorderStart + border, borderEnd = Interpolate<DT, MT>::BorderEnd + border;
    setBorder<char>(dmap, borderStart, borderEnd, -1);

    const MT* pmag = mag.ptr<MT>();
    const MT* plow = low.ptr<MT>();
    const MT* phigh = high.ptr<MT>();
    const DT* pgx = gx.ptr<DT>();
    const DT* pgy = gy.ptr<DT>();
    char* pdmap = dmap.template ptr<char>();
    MT mag_max = 0;

    int last_row = mag.rows - borderEnd, idx, r_end;
    for (int r = borderStart; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - borderEnd);
      idx += borderStart;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > plow[idx]) {
          DT xs = pgx[idx];
          DT ys = pgy[idx];

          size_t x = idx % mag.cols;
          size_t y = idx / mag.cols;

          if (m > Interpolate<DT, MT>::getNB(mag, x - xs, y - ys) &&
              m >= Interpolate<DT, MT>::getNB(mag, x + xs, y + ys)) {
            if (m > phigh[idx]) seeds.push_back(idx);

            EM<DT>::map(pdmap[idx], xs, ys);
          }
        }
      }
    }
    return mag_max;
  }
};

template <class GT, class MT, class DT = float>
struct FastNMS8 {
  static constexpr int NUM_DIR = 8;
  // compute non maxima supression by given derivative maps and magnitude map
  static MT process(const cv::Mat& gx,
                    const cv::Mat& gy,
                    const cv::Mat& mag,
                    MT low,
                    MT high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    static const int TG22 = (int)(0.4142135623730950488016887242097 * (1 << 15) + 0.5);

    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8S);
    setBorder<char>(dmap, border, -1);

    if (low > high) std::swap(high, low);

    const MT* pmag = mag.ptr<MT>();
    const GT* pgx = gx.ptr<GT>();
    const GT* pgy = gy.ptr<GT>();
    char* pdmap = dmap.template ptr<char>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > low) {
          MT xs = static_cast<MT>(pgx[idx]);
          MT ys = static_cast<MT>(pgy[idx]);
          MT axs = std::abs(xs);
          // int ays = std::abs(ys) << 15;
          MT ays = std::abs(ys) * 32768;

          MT tg22x = axs * TG22;

          if (ays < tg22x)  // |
          {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = (xs < 0) ? 6 : 2;
              if (m > high) seeds.push_back(idx);
            }
          } else {
            // int tg67x = tg22x + (axs << 16));
            MT tg67x = tg22x + (axs * 65536);
            if (ays > tg67x)  // -
            {
              if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
                pdmap[idx] = (ys < 0) ? 0 : 4;
                if (m > high) seeds.push_back(idx);
              }
            } else {  // \ or /
                      // int s = (xs ^ ys) < 0 ? -1 : 1;
              int s = neg_sign(xs, ys) ? -1 : 1;
              if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
                pdmap[idx] = (s > 0 ? (xs < 0 ? 7 : 3) : (xs < 0 ? 5 : 1));
                if (m > high) seeds.push_back(idx);
              }
            }
          }
        }
      }
    }
    return mag_max;
  }

  // compute non maxima supression by given derivative maps and magnitude map
  static MT process(const cv::Mat& gx,
                    const cv::Mat& gy,
                    const cv::Mat& mag,
                    const cv::Mat& low,
                    const cv::Mat& high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    static const int TG22 = (int)(0.4142135623730950488016887242097 * (1 << 15) + 0.5);

    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8S);
    setBorder<char>(dmap, border, -1);

    const MT* pmag = mag.ptr<MT>();
    const MT* plow = low.ptr<MT>();
    const MT* phigh = high.ptr<MT>();
    const GT* pgx = gx.ptr<GT>();
    const GT* pgy = gy.ptr<GT>();
    char* pdmap = dmap.template ptr<char>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > plow[idx]) {
          MT high = phigh[idx];
          MT xs = static_cast<MT>(pgx[idx]);
          MT ys = static_cast<MT>(pgy[idx]);
          MT axs = std::abs(xs);
          // int ays = std::abs(ys) << 15;
          MT ays = std::abs(ys) * 32768;

          MT tg22x = axs * TG22;

          if (ays < tg22x)  // |
          {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = (xs < 0) ? 6 : 2;
              if (m > high) seeds.push_back(idx);
            }
          } else {
            // int tg67x = tg22x + (axs << 16));
            MT tg67x = tg22x + (axs * 65536);
            if (ays > tg67x)  // -
            {
              if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
                pdmap[idx] = (ys < 0) ? 0 : 4;
                if (m > high) seeds.push_back(idx);
              }
            } else {  // \ or /
                      // int s = (xs ^ ys) < 0 ? -1 : 1;
              int s = neg_sign(xs, ys) ? -1 : 1;
              if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
                pdmap[idx] = (s > 0 ? (xs < 0 ? 7 : 3) : (xs < 0 ? 5 : 1));
                if (m > high) seeds.push_back(idx);
              }
            }
          }
        }
      }
    }
    return mag_max;
  }

  // compute non maxima supression by given direction map and magnitude map
  static MT process(const cv::Mat& dir,
                    const cv::Mat& mag,
                    MT low,
                    MT high,
                    DT r_low,
                    DT r_high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8SC1);
    setBorder<char>(dmap, border, -1);

    if (low > high) std::swap(high, low);

    if (r_low > r_high) std::swap(r_high, r_low);

    DT r_size = r_high - r_low;
    if (r_low < 0)
      return processRotHalf(dir, mag, low, high, r_size, dmap.template ptr<char>(), seeds, border);
    else
      return processRotFull(dir, mag, low, high, r_size, dmap.template ptr<char>(), seeds, border);
  }

  // compute non maxima supression by given direction map and magnitude map
  static MT process(const cv::Mat& dir,
                    const cv::Mat& mag,
                    const cv::Mat& low,
                    const cv::Mat& high,
                    DT r_low,
                    DT r_high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8SC1);
    setBorder<char>(dmap, border, -1);

    if (r_low > r_high) std::swap(r_high, r_low);

    DT r_size = r_high - r_low;
    if (r_low < 0)
      return processRotHalf(dir, mag, low, high, r_size, dmap.template ptr<char>(), seeds, border);
    else
      return processRotFull(dir, mag, low, high, r_size, dmap.template ptr<char>(), seeds, border);
  }

  static inline const char* name() { return "FastNMS8"; }

 private:
  static MT processRotFull(
      const cv::Mat& dir, const cv::Mat& mag, MT low, MT high, DT r_size, char* pdmap, IndexVector& seeds, int border) {
    const DT step = r_size / 8;
    const DT border1 = step / 2;
    const DT border2 = step / 2 + step;
    const DT border3 = step / 2 + 2 * step;
    const DT border4 = step / 2 + 3 * step;
    const DT r2 = r_size / 2;

    const MT* pmag = mag.ptr<MT>();
    const DT* pdir = dir.ptr<DT>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > low) {
          DT d = pdir[idx];
          if (d > r2) d = d - r_size;
          DT ad = std::abs(d);

          if (ad < border1) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad >= border4) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 6;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad < border3 && ad >= border2) {
            if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
              pdmap[idx] = (ad == d ? 4 : 0);
              if (m > high) seeds.push_back(idx);
            }
          } else {
            int s = d > 0 ? (ad < border2 ? 1 : -1) : (ad < border2 ? -1 : 1);
            if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
              pdmap[idx] = (d > 0 ? (ad < border2 ? 3 : 5) : (ad < border2 ? 1 : 7));
              if (m > high) seeds.push_back(idx);
            }
          }
        }
      }
    }
    return mag_max;
  }

  static MT processRotFull(const cv::Mat& dir,
                           const cv::Mat& mag,
                           const cv::Mat& low,
                           const cv::Mat& high,
                           DT r_size,
                           char* pdmap,
                           IndexVector& seeds,
                           int border) {
    const DT step = r_size / 8;
    const DT border1 = step / 2;
    const DT border2 = step / 2 + step;
    const DT border3 = step / 2 + 2 * step;
    const DT border4 = step / 2 + 3 * step;
    const DT r2 = r_size / 2;

    const MT* pmag = mag.ptr<MT>();
    const MT* plow = low.ptr<MT>();
    const MT* phigh = high.ptr<MT>();
    const DT* pdir = dir.ptr<DT>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > plow[idx]) {
          MT high = phigh[idx];
          DT d = pdir[idx];
          if (d > r2) d = d - r_size;
          DT ad = std::abs(d);

          if (ad < border1) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad >= border4) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 6;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad < border3 && ad >= border2) {
            if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
              pdmap[idx] = (ad == d ? 4 : 0);
              if (m > high) seeds.push_back(idx);
            }
          } else {
            int s = d > 0 ? (ad < border2 ? 1 : -1) : (ad < border2 ? -1 : 1);
            if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
              pdmap[idx] = (d > 0 ? (ad < border2 ? 3 : 5) : (ad < border2 ? 1 : 7));
              if (m > high) seeds.push_back(idx);
            }
          }
        }
      }
    }
    return mag_max;
  }

  static MT processRotHalf(
      const cv::Mat& dir, const cv::Mat& mag, MT low, MT high, DT r_size, char* pdmap, IndexVector& seeds, int border) {
    const DT step = r_size / 8;
    const DT border1 = step / 2;
    const DT border2 = step / 2 + step;
    const DT border3 = step / 2 + 2 * step;
    const DT border4 = step / 2 + 3 * step;

    const MT* pmag = mag.ptr<MT>();
    const DT* pdir = dir.ptr<DT>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > low) {
          DT d = pdir[idx];
          DT ad = std::abs(d);

          if (ad < border1) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad >= border4) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 6;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad < border3 && ad >= border2) {
            if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
              pdmap[idx] = (ad == d ? 4 : 0);
              if (m > high) seeds.push_back(idx);
            }
          } else {
            int s = d > 0 ? (ad < border2 ? 1 : -1) : (ad < border2 ? -1 : 1);
            if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
              pdmap[idx] = (d > 0 ? (ad < border2 ? 3 : 5) : (ad < border2 ? 1 : 7));
              if (m > high) seeds.push_back(idx);
            }
          }
        }
      }
    }
    return mag_max;
  }

  static MT processRotHalf(const cv::Mat& dir,
                           const cv::Mat& mag,
                           const cv::Mat& low,
                           const cv::Mat& high,
                           DT r_size,
                           char* pdmap,
                           IndexVector& seeds,
                           int border) {
    const DT step = r_size / 8;
    const DT border1 = step / 2;
    const DT border2 = step / 2 + step;
    const DT border3 = step / 2 + 2 * step;
    const DT border4 = step / 2 + 3 * step;

    const MT* pmag = mag.ptr<MT>();
    const MT* plow = low.ptr<MT>();
    const MT* phigh = high.ptr<MT>();
    const DT* pdir = dir.ptr<DT>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > plow[idx]) {
          MT high = phigh[idx];
          DT d = pdir[idx];
          DT ad = std::abs(d);

          if (ad < border1) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad >= border4) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 6;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad < border3 && ad >= border2) {
            if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
              pdmap[idx] = (ad == d ? 4 : 0);
              if (m > high) seeds.push_back(idx);
            }
          } else {
            int s = d > 0 ? (ad < border2 ? 1 : -1) : (ad < border2 ? -1 : 1);
            if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
              pdmap[idx] = (d > 0 ? (ad < border2 ? 3 : 5) : (ad < border2 ? 1 : 7));
              if (m > high) seeds.push_back(idx);
            }
          }
        }
      }
    }
    return mag_max;
  }
};

template <class GT, class MT, class DT = float>
struct FastNMS4 {
  static constexpr int NUM_DIR = 4;
  // compute non maxima supression by given derivative maps and magnitude map
  static MT process(const cv::Mat& gx,
                    const cv::Mat& gy,
                    const cv::Mat& mag,
                    MT low,
                    MT high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    static const int TG22 = (int)(0.4142135623730950488016887242097 * (1 << 15) + 0.5);

    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8SC1);
    setBorder<char>(dmap, border, -1);

    if (low > high) std::swap(high, low);

    const MT* pmag = mag.ptr<MT>();
    const GT* pgx = gx.ptr<GT>();
    const GT* pgy = gy.ptr<GT>();
    char* pdmap = dmap.template ptr<char>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > low) {
          MT xs = static_cast<MT>(pgx[idx]);
          MT ys = static_cast<MT>(pgy[idx]);
          MT axs = std::abs(xs);
          // MT ays = std::abs(ys) << 15;
          MT ays = std::abs(ys) * 32768;

          int tg22x = axs * TG22;

          if (ays < tg22x)  // |
          {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else {
            // int tg67x = tg22x + (axs << 16);
            MT tg67x = tg22x + (axs * 65536);
            if (ays > tg67x)  // -
            {
              if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
                pdmap[idx] = 0;
                if (m > high) seeds.push_back(idx);
              }
            } else {  // \ or /
                      // int s = (xs ^ ys) < 0 ? -1 : 1;
              int s = neg_sign(xs, ys) ? -1 : 1;
              if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
                pdmap[idx] = (s > 0 ? 3 : 1);
                if (m > high) seeds.push_back(idx);
              }
            }
          }
        }
      }
    }
    return mag_max;
  }

  // compute non maxima supression by given derivative maps and magnitude map
  static MT process(const cv::Mat& gx,
                    const cv::Mat& gy,
                    const cv::Mat& mag,
                    const cv::Mat& low,
                    const cv::Mat& high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    static const int TG22 = (int)(0.4142135623730950488016887242097 * (1 << 15) + 0.5);

    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8SC1);
    setBorder<char>(dmap, border, -1);

    const MT* pmag = mag.ptr<MT>();
    const MT* plow = low.ptr<MT>();
    const MT* phigh = high.ptr<MT>();
    const GT* pgx = gx.ptr<GT>();
    const GT* pgy = gy.ptr<GT>();
    char* pdmap = dmap.template ptr<char>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > plow[idx]) {
          MT high = phigh[idx];
          MT xs = static_cast<MT>(pgx[idx]);
          MT ys = static_cast<MT>(pgy[idx]);
          MT axs = std::abs(xs);
          // MT ays = std::abs(ys) << 15;
          MT ays = std::abs(ys) * 32768;

          int tg22x = axs * TG22;

          if (ays < tg22x)  // |
          {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else {
            // int tg67x = tg22x + (axs << 16);
            MT tg67x = tg22x + (axs * 65536);
            if (ays > tg67x)  // -
            {
              if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
                pdmap[idx] = 0;
                if (m > high) seeds.push_back(idx);
              }
            } else {  // \ or /
                      // int s = (xs ^ ys) < 0 ? -1 : 1;
              int s = neg_sign(xs, ys) ? -1 : 1;
              if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
                pdmap[idx] = (s > 0 ? 3 : 1);
                if (m > high) seeds.push_back(idx);
              }
            }
          }
        }
      }
    }
    return mag_max;
  }

  // compute non maxima supression by given direction map and magnitude map
  static MT process(const cv::Mat& dir,
                    const cv::Mat& mag,
                    MT low,
                    MT high,
                    DT r_low,
                    DT r_high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8SC1);
    setBorder<char>(dmap, border, -1);

    if (low > high) std::swap(high, low);

    if (r_low > r_high) std::swap(r_high, r_low);

    DT r_size = r_high - r_low;
    if (r_low < 0)
      return processRotHalf(dir, mag, low, high, r_size, dmap.template ptr<char>(), seeds, border);
    else
      return processRotFull(dir, mag, low, high, r_size, dmap.template ptr<char>(), seeds, border);
  }

  // compute non maxima supression by given direction map and magnitude map
  static MT process(const cv::Mat& dir,
                    const cv::Mat& mag,
                    const cv::Mat& low,
                    const cv::Mat& high,
                    DT r_low,
                    DT r_high,
                    IndexVector& seeds,
                    cv::Mat& dmap,
                    int border = 1) {
    seeds.clear();
    seeds.reserve(mag.rows * mag.cols / 3);
    dmap.create(mag.rows, mag.cols, CV_8SC1);
    setBorder<char>(dmap, border, -1);

    if (r_low > r_high) std::swap(r_high, r_low);

    DT r_size = r_high - r_low;
    if (r_low < 0)
      return processRotHalf(dir, mag, low, high, r_size, dmap.template ptr<char>(), seeds, border);
    else
      return processRotFull(dir, mag, low, high, r_size, dmap.template ptr<char>(), seeds, border);
  }

  static inline const char* name() { return "FastNMS4"; }

 private:
  static MT processRotFull(
      const cv::Mat& dir, const cv::Mat& mag, MT low, MT high, DT r_size, char* pdmap, IndexVector& seeds, int border) {
    const DT step = r_size / 8;
    const DT border1 = step / 2;
    const DT border2 = step / 2 + step;
    const DT border3 = step / 2 + 2 * step;
    const DT border4 = step / 2 + 3 * step;
    const DT r2 = r_size / 2;

    const MT* pmag = mag.ptr<MT>();
    const DT* pdir = dir.ptr<DT>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > low) {
          DT d = pdir[idx];
          if (d > r2) d = d - r_size;
          DT ad = std::abs(d);

          if (ad < border1) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad >= border4) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad < border3 && ad >= border2) {
            if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
              pdmap[idx] = 0;
              if (m > high) seeds.push_back(idx);
            }
          } else {
            int s = d > 0 ? (ad < border2 ? 1 : -1) : (ad < border2 ? -1 : 1);
            if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
              pdmap[idx] = (d > 0 ? (ad < border2 ? 3 : 1) : (ad < border2 ? 1 : 3));
              if (m > high) seeds.push_back(idx);
            }
          }
        }
      }
    }
    return mag_max;
  }

  static MT processRotFull(const cv::Mat& dir,
                           const cv::Mat& mag,
                           const cv::Mat& low,
                           const cv::Mat& high,
                           DT r_size,
                           char* pdmap,
                           IndexVector& seeds,
                           int border) {
    const DT step = r_size / 8;
    const DT border1 = step / 2;
    const DT border2 = step / 2 + step;
    const DT border3 = step / 2 + 2 * step;
    const DT border4 = step / 2 + 3 * step;
    const DT r2 = r_size / 2;

    const MT* pmag = mag.ptr<MT>();
    const MT* plow = low.ptr<MT>();
    const MT* phigh = high.ptr<MT>();
    const DT* pdir = dir.ptr<DT>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > plow[idx]) {
          MT high = phigh[idx];
          DT d = pdir[idx];
          if (d > r2) d = d - r_size;
          DT ad = std::abs(d);

          if (ad < border1) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad >= border4) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad < border3 && ad >= border2) {
            if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
              pdmap[idx] = 0;
              if (m > high) seeds.push_back(idx);
            }
          } else {
            int s = d > 0 ? (ad < border2 ? 1 : -1) : (ad < border2 ? -1 : 1);
            if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
              pdmap[idx] = (d > 0 ? (ad < border2 ? 3 : 1) : (ad < border2 ? 1 : 3));
              if (m > high) seeds.push_back(idx);
            }
          }
        }
      }
    }
    return mag_max;
  }

  static MT processRotHalf(
      const cv::Mat& dir, const cv::Mat& mag, MT low, MT high, DT r_size, char* pdmap, IndexVector& seeds, int border) {
    const DT step = r_size / 8;
    const DT border1 = step / 2;
    const DT border2 = step / 2 + step;
    const DT border3 = step / 2 + 2 * step;
    const DT border4 = step / 2 + 3 * step;

    const MT* pmag = mag.ptr<MT>();
    const DT* pdir = dir.ptr<DT>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > low) {
          DT d = pdir[idx];
          DT ad = std::abs(d);

          if (ad < border1) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad >= border4) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad < border3 && ad >= border2) {
            if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
              pdmap[idx] = 0;
              if (m > high) seeds.push_back(idx);
            }
          } else {
            int s = d > 0 ? (ad < border2 ? 1 : -1) : (ad < border2 ? -1 : 1);
            if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
              pdmap[idx] = (d > 0 ? (ad < border2 ? 3 : 1) : (ad < border2 ? 1 : 3));
              if (m > high) seeds.push_back(idx);
            }
          }
        }
      }
    }
    return mag_max;
  }

  static MT processRotHalf(const cv::Mat& dir,
                           const cv::Mat& mag,
                           const cv::Mat& low,
                           const cv::Mat& high,
                           DT r_size,
                           char* pdmap,
                           IndexVector& seeds,
                           int border) {
    const DT step = r_size / 8;
    const DT border1 = step / 2;
    const DT border2 = step / 2 + step;
    const DT border3 = step / 2 + 2 * step;
    const DT border4 = step / 2 + 3 * step;

    const MT* pmag = mag.ptr<MT>();
    const MT* plow = low.ptr<MT>();
    const MT* phigh = high.ptr<MT>();
    const DT* pdir = dir.ptr<DT>();
    MT mag_max = 0;

    int last_row = mag.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * mag.cols;
      r_end = idx + (mag.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        MT m = pmag[idx];
        mag_max = std::max(m, mag_max);
        pdmap[idx] = -1;

        if (m > plow[idx]) {
          MT high = phigh[idx];
          DT d = pdir[idx];
          DT ad = std::abs(d);

          if (ad < border1) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad >= border4) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) {
              pdmap[idx] = 2;
              if (m > high) seeds.push_back(idx);
            }
          } else if (ad < border3 && ad >= border2) {
            if (m > pmag[idx - mag.cols] && m >= pmag[idx + mag.cols]) {
              pdmap[idx] = 0;
              if (m > high) seeds.push_back(idx);
            }
          } else {
            int s = d > 0 ? (ad < border2 ? 1 : -1) : (ad < border2 ? -1 : 1);
            if (m > pmag[idx - mag.cols - s] && m > pmag[idx + mag.cols + s]) {
              pdmap[idx] = (d > 0 ? (ad < border2 ? 3 : 1) : (ad < border2 ? 1 : 3));
              if (m > high) seeds.push_back(idx);
            }
          }
        }
      }
    }
    return mag_max;
  }
};

template <class GT, class MT, class DT = float, class NMS = FastNMS8<GT, MT, DT>>
class NonMaximaSuppression : public ValueManager {
  cv::Mat dmap_;       // direction map, storing direction 0-X for each value that is above lower
                       // threshold, else -1 (border values are set all to -1); be 4, 8
  IndexVector seeds_;  // indexes of values above higher treshold (border values not included)

  double th_low_, th_high_;
  MT mag_max_;
  int border_;

 public:
  typedef GT grad_type;
  typedef MT mag_type;
  typedef DT dir_type;
  static constexpr int NUM_DIR = NMS::NUM_DIR;

  NonMaximaSuppression(double low = 0.004, double high = 0.012, int border = 1)
      : th_low_(low), th_high_(high), mag_max_(0), border_(border) {
    this->add("nms_th_low",
              std::bind(&NonMaximaSuppression<GT, MT, DT, NMS>::valueThresholdLow, this, std::placeholders::_1),
              "Lower threshold.");
    this->add("nms_th_high",
              std::bind(&NonMaximaSuppression<GT, MT, DT, NMS>::valueThresholdHigh, this, std::placeholders::_1),
              "Upper threshold.");
    this->add("nms_border", std::bind(&NonMaximaSuppression<GT, MT, DT, NMS>::valueBorder, this, std::placeholders::_1),
              "Border that is skipped for NMS.");
  }

  Value valueThresholdLow(const Value& t = Value::NAV()) {
    if (t.type()) thresholdLow(t.getDouble());
    return th_low_;
  }

  double thresholdLow() const { return th_low_; }

  void thresholdLow(double t) { th_low_ = t; }

  Value valueThresholdHigh(const Value& t = Value::NAV()) {
    if (t.type()) thresholdHigh(t.getDouble());
    return th_high_;
  }

  double thresholdHigh() const { return th_high_; }

  void thresholdHigh(double t) { th_high_ = t; }

  void threshold(double low, double high) {
    th_low_ = low;
    th_high_ = high;
  }

  Value valueBorder(const Value& b = Value::NAV()) {
    if (b.type()) border(b.getInt());
    return border_;
  }

  int border() const { return border_; }

  void border(int b) { border_ = b; }

  MT magMax() const { return mag_max_; }


  inline const cv::Mat& directionMap() const { return dmap_; }

  inline const IndexVector& seeds() const { return seeds_; }

  template <class GRAD>
  inline MT process(const cv::Mat& img, bool force_dir) {
    GRAD grad;
    grad.process(img);
    return process(grad, th_low_, th_high_, force_dir);
  }

  template <class GRAD>
  inline MT process(GRAD& grad, bool force_dir) {
    MT low = grad.magnitudeThreshold(th_low_);
    MT high = grad.magnitudeThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    if (force_dir) {
      typename GRAD::DirectionRange dr = grad.directionRange();
      return process(grad.direction(), grad.magnitude(), low, high, dr.lower, dr.upper);
    }
    return process(grad.gx(), grad.gy(), grad.magnitude(), low, high);
  }

  template <class GRAD>
  inline MT process(const cv::Mat& img) {
    GRAD grad;
    grad.process(img);
    return process(grad, th_low_, th_high_);
  }

  template <class GRAD>
  inline MT process(GRAD& grad) {
    MT low = grad.magnitudeThreshold(th_low_);
    MT high = grad.magnitudeThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    return process(grad.gx(), grad.gy(), grad.magnitude(), low, high);
  }

  template <class GRAD>
  inline MT process(GRAD& grad, double th_low, double th_high) {
    MT low = grad.magnitudeThreshold(th_low);
    MT high = grad.magnitudeThreshold(th_high);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    return process(grad.gx(), grad.gy(), grad.magnitude(), low, high);
  }

  template <class QUAD>
  inline MT processQ(QUAD& grad, double th_low, double th_high) {
    MT low = grad.evenThreshold(th_low);
    MT high = grad.evenThreshold(th_high);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    return process(grad.oddx(), grad.oddy(), grad.even(), low, high);
  }

  template <class ENERGY>
  inline MT processE(ENERGY& grad) {
    MT low = grad.energyThreshold(th_low_);
    MT high = grad.energyThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    return process(grad.oddx(), grad.oddy(), grad.energy(), low, high);
  }

  template <class PC>
  inline MT processPC(PC& grad) {
    MT low = th_low_;
    MT high = th_high_;

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    return process(grad.oddx(), grad.oddy(), grad.phaseCongruency(), low, high);
  }


  template <class GRAD>
  inline MT processD(const cv::Mat& img) {
    GRAD grad;
    grad.process(img);
    return process(grad, th_low_, th_high_);
  }

  template <class GRAD>
  inline MT processD(GRAD& grad) {
    MT low = grad.magnitudeThreshold(th_low_);
    MT high = grad.magnitudeThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    typename GRAD::DirectionRange dr = grad.directionRange();
    return process(grad.direction(), grad.magnitude(), low, high, dr.lower, dr.upper);
  }

  // compute non maxima supression by given derivative maps and magnitude map
  inline MT process(const cv::Mat& gx, const cv::Mat& gy, const cv::Mat& mag, MT low, MT high) {
    mag_max_ = NMS::process(gx.isContinuous() ? gx : gx.clone(), gy.isContinuous() ? gy : gy.clone(),
                            mag.isContinuous() ? mag : mag.clone(), low, high, seeds_, dmap_, border_);
    return mag_max_;
  }

  // compute non maxima supression by given direction map and magnitude map
  inline MT process(const cv::Mat& dir, const cv::Mat& mag, MT low, MT high, DT r_low, DT r_high) {
    mag_max_ = NMS::process(dir.isContinuous() ? dir : dir.clone(), mag.isContinuous() ? mag : mag.clone(), low, high,
                            r_low, r_high, seeds_, dmap_, border_);
    return mag_max_;
  }

  // compute non maxima supression by given derivative maps and magnitude map
  inline MT process(const cv::Mat& gx, const cv::Mat& gy, const cv::Mat& mag, const cv::Mat& low, const cv::Mat& high) {
    mag_max_ = NMS::process(gx.isContinuous() ? gx : gx.clone(), gy.isContinuous() ? gy : gy.clone(),
                            mag.isContinuous() ? mag : mag.clone(), low.isContinuous() ? low : low.clone(),
                            high.isContinuous() ? high : high.clone(), seeds_, dmap_, border_);
    return mag_max_;
  }

  // compute non maxima supression by given direction map and magnitude map
  inline MT process(
      const cv::Mat& dir, const cv::Mat& mag, const cv::Mat& low, const cv::Mat& high, DT r_low, DT r_high) {
    mag_max_ = NMS::process(dir.isContinuous() ? dir : dir.clone(), mag.isContinuous() ? mag : mag.clone(),
                            low.isContinuous() ? low : low.clone(), high.isContinuous() ? high : high.clone(), r_low,
                            r_high, seeds_, dmap_, border_);
    return mag_max_;
  }

  //! compute hysteresis of seeds and dmap_
  cv::Mat hysteresis() const { return lsfm::hysteresis(dmap_, seeds_); }

  //! compute hysteresis of seeds and dmap_
  cv::Mat hysteresisBinary(uchar val = 255) const { return lsfm::hysteresis_binary(dmap_, seeds_, val); }

  //! compute hysteresis of seeds and dmap_
  IndexVector hysteresis_edgels() const {
    IndexVector edgels = seeds_;
    lsfm::hysteresis_edgels(dmap_, edgels);
    return edgels;
  }

  //! get name of direction operator
  inline std::string name() const { return NMS::name(); }
};
}  // namespace lsfm
#endif
#endif
