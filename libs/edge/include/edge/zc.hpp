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

#ifndef _ZC_HPP_
#define _ZC_HPP_
#ifdef __cplusplus

#  include <edge/nms.hpp>

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


typedef size_t index_type;
typedef std::vector<index_type> IndexVector;


template <class T>
inline bool eps_zero(T v) {
  // return std::abs(v) < 2;
  return v == 0;
}

inline bool eps_zero(float v) { return std::abs(v) < 0.001f; }

inline bool eps_zero(double v) { return std::abs(v) < 0.001; }

template <class LT>
struct NCC_NONE {
  static inline void test(size_t& idx1, size_t& idx2, LT l1, LT l2) {}
};


// get index closer to zero
template <class LT>
struct NCC_BASIC {
  static inline void test(size_t& idx1, size_t& idx2, LT l1, LT l2) {
    if (std::abs(l1) > std::abs(l2)) std::swap(idx1, idx2);
  }
};


template <class LT, class DT>
struct EZCMap8 {
  static constexpr int NUM_DIR = 8;
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

  static inline void mapDiff(char& dmap, LT xs, LT ys) {
    if (xs == 0) {
      dmap = (ys > 0 ? 4 : 0);
      return;
    }

    if (ys == 0) {
      dmap = (xs > 0 ? 2 : 6);
      return;
    }

    DT d = std::abs(static_cast<DT>(ys) / xs);

    if (d < 0.414) {
      dmap = (xs > 0 ? 2 : 6);
    } else if (d > 2.414) {
      dmap = (ys > 0 ? 4 : 0);
    } else {
      if (xs > 0)
        dmap = (ys > 0 ? 3 : 1);
      else
        dmap = (ys > 0 ? 5 : 7);
    }
  }

  /*static inline void process(const cv::Mat& l, LT low, LT high, IndexVector& seeds, cv::Mat &dmap) {
      FastZC8<LT,DT>::process(l,low,high,seeds,dmap);
  }*/

  static inline const char type() { return '8'; }
};

template <class LT, class DT>
struct EZCMap4 {
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

  static inline void mapDiff(char& dmap, LT xs, LT ys) {
    if (xs == 0) {
      dmap = 0;
      return;
    }

    if (ys == 0) {
      dmap = 2;
      return;
    }

    DT d = std::abs(static_cast<DT>(ys) / xs);

    if (d < 0.414) {
      dmap = 2;
    } else if (d > 2.414) {
      dmap = 0;
    } else {
      dmap = (xs * ys > 0 ? 3 : 1);
    }
  }

  /*static inline void process(const cv::Mat& l, LT low, LT high, IndexVector& seeds, cv::Mat &dmap) {
      FastZC4<LT,DT>::process(l,low,high,seeds,dmap);
  }*/

  static inline const char type() { return '4'; }
};

template <class LT, class FT, template <class> class NCC, template <class, class> class EM>
void zc_base(const cv::Mat& l, LT low, LT high, IndexVector& seeds, cv::Mat& dmap, int border = 2) {
  seeds.clear();
  seeds.reserve(l.rows * l.cols / 3);
  dmap.create(l.rows, l.cols, CV_8S);
  dmap.setTo(-1);

  if (low > high) std::swap(high, low);

  const LT* pl = l.ptr<LT>();
  char* pdmap = dmap.template ptr<char>();

  size_t idx2, idxm;
  LT lp, lx, ly;
  LT xdiff, ydiff, mdiff;

  int last_row = l.rows - border, idx, r_end;
  for (int r = border; r != last_row; ++r) {
    idx = r * l.cols;
    r_end = idx + (l.cols - border);
    idx += border;

    for (; idx < r_end; ++idx) {
      if (pdmap[idx] != -1) continue;

      lp = pl[idx];
      lx = pl[idx + 1];
      ly = pl[idx + l.cols];


      xdiff =
          (neg_sign(lp, lx) && (!eps_zero(lp) || !eps_zero(pl[idx - 1])) && (!eps_zero(lx) || !eps_zero(pl[idx + 1])))
              ? lp - lx
              : 0;
      ydiff = (neg_sign(lp, ly) && (!eps_zero(lp) || !eps_zero(pl[idx - l.cols])) &&
               (!eps_zero(ly) || !eps_zero(pl[idx + 2 * l.cols])))
                  ? lp - ly
                  : 0;

      idxm = idx + 1;
      mdiff = std::abs(xdiff);
      if (std::abs(ydiff) > mdiff) {
        mdiff = std::abs(ydiff);
        idxm = idx + l.cols;
        lx = ly;
      }

      if (mdiff > low) {
        idx2 = idx;
        NCC<LT>::test(idx2, idxm, lp, lx);
        EM<LT, FT>::mapDiff(pdmap[idx2], xdiff, ydiff);
        pdmap[idxm] = -2;
        if (mdiff > high) seeds.push_back(idx2);
      }
    }
  }
}

template <class LT, class FT, template <class> class NCC, template <class, class> class EM>
void zc_base(
    const cv::Mat& l, const cv::Mat& low, const cv::Mat& high, IndexVector& seeds, cv::Mat& dmap, int border = 2) {
  seeds.clear();
  seeds.reserve(l.rows * l.cols / 3);
  dmap.create(l.rows, l.cols, CV_8S);
  dmap.setTo(-1);

  const LT* pl = l.ptr<LT>();
  const LT* plow = low.ptr<LT>();
  const LT* phigh = high.ptr<LT>();
  char* pdmap = dmap.template ptr<char>();

  size_t idx2, idxm;
  LT lp, lx, ly;
  LT xdiff, ydiff, mdiff;

  int last_row = l.rows - border, idx, r_end;
  for (int r = border; r != last_row; ++r) {
    idx = r * l.cols;
    r_end = idx + (l.cols - border);
    idx += border;

    for (; idx < r_end; ++idx) {
      if (pdmap[idx] != -1) continue;

      lp = pl[idx];
      lx = pl[idx + 1];
      ly = pl[idx + l.cols];


      xdiff =
          (neg_sign(lp, lx) && (!eps_zero(lp) || !eps_zero(pl[idx - 1])) && (!eps_zero(lx) || !eps_zero(pl[idx + 1])))
              ? lp - lx
              : 0;
      ydiff = (neg_sign(lp, ly) && (!eps_zero(lp) || !eps_zero(pl[idx - l.cols])) &&
               (!eps_zero(ly) || !eps_zero(pl[idx + 2 * l.cols])))
                  ? lp - ly
                  : 0;

      idxm = idx + 1;
      mdiff = std::abs(xdiff);
      if (std::abs(ydiff) > mdiff) {
        mdiff = std::abs(ydiff);
        idxm = idx + l.cols;
        lx = ly;
      }

      if (mdiff > plow[idx]) {
        idx2 = idx;
        NCC<LT>::test(idx2, idxm, lp, lx);
        EM<LT, FT>::mapDiff(pdmap[idx2], xdiff, ydiff);
        pdmap[idxm] = -2;
        if (mdiff > phigh[idx]) seeds.push_back(idx2);
      }
    }
  }
}

template <class GT,
          class LT,
          class FT = float,
          template <class> class NCC = NCC_BASIC,
          template <class, class> class EM = EZCMap8,
          template <class, class> class Interpolate = LinearInterpolator,
          template <class, class> class P = Polar>
struct PreciseZC {
  static constexpr int NUM_DIR = EM<LT, FT>::NUM_DIR;

  // compute zero crossing by given derivative maps and laplacian map
  static void process(const cv::Mat& gx,
                      const cv::Mat& gy,
                      const cv::Mat& l,
                      LT low,
                      LT high,
                      IndexVector& seeds,
                      cv::Mat& dmap,
                      int border = 2) {
    seeds.clear();
    seeds.reserve(l.rows * l.cols / 3);
    dmap.create(l.rows, l.cols, CV_8S);
    int borderStart = std::max(Interpolate<FT, LT>::BorderStart + 1, border),
        borderEnd = std::max(Interpolate<FT, LT>::BorderEnd + 1, border);
    setBorder<char>(dmap, borderStart, borderEnd, -1);

    if (low > high) std::swap(high, low);

    const LT* pl = l.ptr<LT>();
    const GT* pgx = gx.ptr<GT>();
    const GT* pgy = gy.ptr<GT>();
    char* pdmap = dmap.template ptr<char>();

    int last_row = l.rows - borderEnd, idx, r_end;
    for (int r = borderStart; r != last_row; ++r) {
      idx = r * l.cols;
      r_end = idx + (l.cols - borderEnd);
      idx += borderStart;

      for (; idx < r_end; ++idx) {
        FT xs = pgx[idx];
        FT ys = pgy[idx];
        FT n = std::hypot(xs, ys);
        if (std::abs(n) < std::numeric_limits<FT>::epsilon()) continue;
        xs /= n;
        ys /= n;

        FT l1 = pl[idx];
        FT l2 = Interpolate<FT, LT>::getNB(l, idx % l.cols + xs, idx / l.cols + ys);
        pdmap[idx] = -1;

        if (neg_sign(l1, l2) && !eps_zero(l1) && !eps_zero(l2)) {
          FT diff = std::abs(l1 - l2);
          if (diff > low) {
            EM<LT, FT>::map(pdmap[idx], xs, ys);
            if (diff > high) seeds.push_back(idx);
          }
        }
      }
    }
  }

  // compute zero crossing by given derivative maps and laplacian map
  static void process(const cv::Mat& gx,
                      const cv::Mat& gy,
                      const cv::Mat& l,
                      const cv::Mat& low,
                      const cv::Mat& high,
                      IndexVector& seeds,
                      cv::Mat& dmap,
                      int border = 2) {
    seeds.clear();
    seeds.reserve(l.rows * l.cols / 3);
    dmap.create(l.rows, l.cols, CV_8S);
    int borderStart = std::max(Interpolate<FT, LT>::BorderStart + 1, border),
        borderEnd = std::max(Interpolate<FT, LT>::BorderEnd + 1, border);
    setBorder<char>(dmap, borderStart, borderEnd, -1);


    const LT* pl = l.ptr<LT>();
    const LT* plow = low.ptr<LT>();
    const LT* phigh = high.ptr<LT>();
    const GT* pgx = gx.ptr<GT>();
    const GT* pgy = gy.ptr<GT>();
    char* pdmap = dmap.template ptr<char>();

    int last_row = l.rows - borderEnd, idx, r_end;
    for (int r = borderStart; r != last_row; ++r) {
      idx = r * l.cols;
      r_end = idx + (l.cols - borderEnd);
      idx += borderStart;

      for (; idx < r_end; ++idx) {
        FT xs = pgx[idx];
        FT ys = pgy[idx];
        FT n = std::hypot(xs, ys);
        if (std::abs(n) < std::numeric_limits<FT>::epsilon()) continue;
        xs /= n;
        ys /= n;

        FT l1 = pl[idx];
        FT l2 = Interpolate<FT, LT>::getNB(l, idx % l.cols + xs, idx / l.cols + ys);
        pdmap[idx] = -1;

        if (neg_sign(l1, l2) && !eps_zero(l1) && !eps_zero(l2)) {
          FT diff = std::abs(l1 - l2);
          if (diff > plow[idx]) {
            EM<LT, FT>::map(pdmap[idx], xs, ys);
            if (diff > phigh[idx]) seeds.push_back(idx);
          }
        }
      }
    }
  }

  // compute zero crossing by given derivative maps and laplacian map (normalized gradient)
  static void processN(const cv::Mat& gx,
                       const cv::Mat& gy,
                       const cv::Mat& l,
                       LT low,
                       LT high,
                       IndexVector& seeds,
                       cv::Mat& dmap,
                       int border = 2) {
    seeds.clear();
    seeds.reserve(l.rows * l.cols / 3);
    dmap.create(l.rows, l.cols, CV_8S);
    int borderStart = std::max(Interpolate<FT, LT>::BorderStart + 1, border),
        borderEnd = std::max(Interpolate<FT, LT>::BorderEnd + 1, border);
    setBorder<char>(dmap, borderStart, borderEnd, -1);

    if (low > high) std::swap(high, low);

    const LT* pl = l.ptr<LT>();
    const FT* pgx = gx.ptr<FT>();
    const FT* pgy = gy.ptr<FT>();
    char* pdmap = dmap.template ptr<char>();

    int last_row = l.rows - borderEnd, idx, r_end;
    for (int r = borderStart; r != last_row; ++r) {
      idx = r * l.cols;
      r_end = idx + (l.cols - borderEnd);
      idx += borderStart;

      for (; idx < r_end; ++idx) {
        FT l1 = pl[idx];
        FT xs = pgx[idx];
        FT ys = pgy[idx];
        FT l2 = Interpolate<FT, LT>::getNB(l, idx % l.cols + xs, idx / l.cols + ys);
        pdmap[idx] = -1;

        if (neg_sign(l1, l2) && !eps_zero(l1) && !eps_zero(l2)) {
          FT diff = std::abs(l1 - l2);
          if (diff > low) {
            EM<LT, FT>::map(pdmap[idx], xs, ys);
            if (diff > high) seeds.push_back(idx);
          }
        }
      }
    }
  }

  // compute zero crossing by given derivative maps and laplacian map (normalized gradient)
  static void processN(const cv::Mat& gx,
                       const cv::Mat& gy,
                       const cv::Mat& l,
                       const cv::Mat& low,
                       const cv::Mat& high,
                       IndexVector& seeds,
                       cv::Mat& dmap,
                       int border = 2) {
    seeds.clear();
    seeds.reserve(l.rows * l.cols / 3);
    dmap.create(l.rows, l.cols, CV_8S);
    int borderStart = std::max(Interpolate<FT, LT>::BorderStart + 1, border),
        borderEnd = std::max(Interpolate<FT, LT>::BorderEnd + 1, border);
    setBorder<char>(dmap, borderStart, borderEnd, -1);

    const LT* pl = l.ptr<LT>();
    const LT* plow = low.ptr<LT>();
    const LT* phigh = high.ptr<LT>();
    const FT* pgx = gx.ptr<FT>();
    const FT* pgy = gy.ptr<FT>();
    char* pdmap = dmap.template ptr<char>();

    int last_row = l.rows - borderEnd, idx, r_end;
    for (int r = borderStart; r != last_row; ++r) {
      idx = r * l.cols;
      r_end = idx + (l.cols - borderEnd);
      idx += borderStart;

      for (; idx < r_end; ++idx) {
        FT l1 = pl[idx];
        FT xs = pgx[idx];
        FT ys = pgy[idx];
        FT l2 = Interpolate<FT, LT>::getNB(l, idx % l.cols + xs, idx / l.cols + ys);
        pdmap[idx] = -1;

        if (neg_sign(l1, l2) && !eps_zero(l1) && !eps_zero(l2)) {
          FT diff = std::abs(l1 - l2);
          if (diff > plow[idx]) {
            EM<LT, FT>::map(pdmap[idx], xs, ys);
            if (diff > phigh[idx]) seeds.push_back(idx);
          }
        }
      }
    }
  }

  // compute zero crossing by given direction map and laplacian map
  static void process(const cv::Mat& dir,
                      const cv::Mat& l,
                      LT low,
                      LT high,
                      FT r_low,
                      FT r_high,
                      IndexVector& seeds,
                      cv::Mat& dmap,
                      int border = 2) {
    static FT pi2 = static_cast<FT>(2 * CV_PI), epsilon = static_cast<FT>(0.01);
    FT r_size = r_high - r_low;
    cv::Mat dirf = dir, gxf, gyf;
    if (r_size > pi2 + epsilon || r_size < pi2 - epsilon) {
      dirf = dir * (pi2 / r_size);
    }

    P<FT, FT>::polar2Cart(dirf, gxf, gyf);
    processN(gxf, gyf, l, low, high, seeds, dmap, border);
  }

  // compute zero crossing by given direction map and laplacian map
  static void process(const cv::Mat& dir,
                      const cv::Mat& l,
                      const cv::Mat& low,
                      const cv::Mat& high,
                      FT r_low,
                      FT r_high,
                      IndexVector& seeds,
                      cv::Mat& dmap,
                      int border = 2) {
    static FT pi2 = static_cast<FT>(2 * CV_PI), epsilon = static_cast<FT>(0.01);
    FT r_size = r_high - r_low;
    cv::Mat dirf = dir, gxf, gyf;
    if (r_size > pi2 + epsilon || r_size < pi2 - epsilon) {
      dirf = dir * (pi2 / r_size);
    }

    P<FT, FT>::polar2Cart(dirf, gxf, gyf);
    processN(gxf, gyf, l, low, high, seeds, dmap, border);
  }

  static void process(const cv::Mat& l, LT low, LT high, IndexVector& seeds, cv::Mat& dmap, int border = 2) {
    zc_base<LT, FT, NCC, EM>(l, low, high, seeds, dmap, border);
  }

  static void process(
      const cv::Mat& l, const cv::Mat& low, const cv::Mat& high, IndexVector& seeds, cv::Mat& dmap, int border = 2) {
    zc_base<LT, FT, NCC, EM>(l, low, high, seeds, dmap, border);
  }

  static inline std::string name() { return std::string("PreciseZC") + EM<LT, FT>::type(); }
};

template <class GT,
          class LT,
          class FT = float,
          template <class> class NCC = NCC_BASIC,
          template <class, class> class EM = EZCMap8,
          template <class, class> class P = Polar>
struct FastZC {
  static constexpr int NUM_DIR = EM<LT, FT>::NUM_DIR;

  // compute zero crossing by given derivative maps and laplacian map
  static void process(const cv::Mat& gx,
                      const cv::Mat& gy,
                      const cv::Mat& l,
                      LT low,
                      LT high,
                      IndexVector& seeds,
                      cv::Mat& dmap,
                      int border = 2) {
    seeds.clear();
    seeds.reserve(l.rows * l.cols / 3);
    dmap.create(l.rows, l.cols, CV_8S);
    setBorder<char>(dmap, border, -1);

    if (low > high) std::swap(high, low);

    const LT* pl = l.ptr<LT>();
    const GT* pgx = gx.ptr<GT>();
    const GT* pgy = gy.ptr<GT>();
    char* pdmap = dmap.template ptr<char>();

    int last_row = l.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * l.cols;
      r_end = idx + (l.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        pdmap[idx] = -1;

        FT xs = pgx[idx];
        FT ys = pgy[idx];
        FT n = std::hypot(xs, ys);
        if (std::abs(n) < std::numeric_limits<FT>::epsilon()) continue;
        xs /= n;
        ys /= n;

        LT l1 = pl[idx];
        LT l2 = pl[static_cast<index_type>(idx + static_cast<int>(std::round(xs)) +
                                           static_cast<int>(std::round(ys) * l.cols))];

        if (neg_sign(l1, l2) && !eps_zero(l1) && !eps_zero(l2)) {
          LT diff = std::abs(l1 - l2);
          if (diff > low) {
            EM<LT, FT>::map(pdmap[idx], xs, ys);
            if (diff > high) seeds.push_back(idx);
          }
        }
      }
    }
  }

  // compute zero crossing by given derivative maps and laplacian map
  static void process(const cv::Mat& gx,
                      const cv::Mat& gy,
                      const cv::Mat& l,
                      const cv::Mat& low,
                      const cv::Mat& high,
                      IndexVector& seeds,
                      cv::Mat& dmap,
                      int border = 2) {
    seeds.clear();
    seeds.reserve(l.rows * l.cols / 3);
    dmap.create(l.rows, l.cols, CV_8S);
    setBorder<char>(dmap, border, -1);

    const LT* pl = l.ptr<LT>();
    const LT* plow = low.ptr<LT>();
    const LT* phigh = high.ptr<LT>();
    const GT* pgx = gx.ptr<GT>();
    const GT* pgy = gy.ptr<GT>();
    char* pdmap = dmap.template ptr<char>();

    int last_row = l.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * l.cols;
      r_end = idx + (l.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        pdmap[idx] = -1;

        FT xs = pgx[idx];
        FT ys = pgy[idx];
        FT n = std::hypot(xs, ys);
        if (std::abs(n) < std::numeric_limits<FT>::epsilon()) continue;
        xs /= n;
        ys /= n;

        LT l1 = pl[idx];
        LT l2 = pl[static_cast<index_type>(idx + static_cast<int>(std::round(xs)) +
                                           static_cast<int>(std::round(ys) * l.cols))];

        if (neg_sign(l1, l2) && !eps_zero(l1) && !eps_zero(l2)) {
          LT diff = std::abs(l1 - l2);
          if (diff > plow[idx]) {
            EM<LT, FT>::map(pdmap[idx], xs, ys);
            if (diff > phigh[idx]) seeds.push_back(idx);
          }
        }
      }
    }
  }

  // compute zero crossing by given derivative maps and laplacian map (normalized gradient)
  static void processN(const cv::Mat& gx,
                       const cv::Mat& gy,
                       const cv::Mat& l,
                       LT low,
                       LT high,
                       IndexVector& seeds,
                       cv::Mat& dmap,
                       int border = 2) {
    seeds.clear();
    seeds.reserve(l.rows * l.cols / 3);
    dmap.create(l.rows, l.cols, CV_8S);
    setBorder<char>(dmap, border, -1);

    if (low > high) std::swap(high, low);

    const LT* pl = l.ptr<LT>();
    const FT* pgx = gx.ptr<FT>();
    const FT* pgy = gy.ptr<FT>();
    char* pdmap = dmap.template ptr<char>();

    int last_row = l.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * l.cols;
      r_end = idx + (l.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        pdmap[idx] = -1;

        FT xs = pgx[idx];
        FT ys = pgy[idx];

        LT l1 = pl[idx];
        LT l2 = pl[static_cast<index_type>(idx + static_cast<int>(std::round(xs)) +
                                           static_cast<int>(std::round(ys) * l.cols))];

        if (neg_sign(l1, l2) && !eps_zero(l1) && !eps_zero(l2)) {
          LT diff = std::abs(l1 - l2);
          if (diff > low) {
            EM<LT, FT>::map(pdmap[idx], xs, ys);
            if (diff > high) seeds.push_back(idx);
          }
        }
      }
    }
  }

  // compute zero crossing by given derivative maps and laplacian map (normalized gradient)
  static void processN(const cv::Mat& gx,
                       const cv::Mat& gy,
                       const cv::Mat& l,
                       const cv::Mat& low,
                       const cv::Mat& high,
                       IndexVector& seeds,
                       cv::Mat& dmap,
                       int border = 2) {
    seeds.clear();
    seeds.reserve(l.rows * l.cols / 3);
    dmap.create(l.rows, l.cols, CV_8S);
    setBorder<char>(dmap, border, -1);

    const LT* pl = l.ptr<LT>();
    const LT* plow = low.ptr<LT>();
    const LT* phigh = high.ptr<LT>();
    const FT* pgx = gx.ptr<FT>();
    const FT* pgy = gy.ptr<FT>();
    char* pdmap = dmap.template ptr<char>();

    int last_row = l.rows - border, idx, r_end;
    for (int r = border; r != last_row; ++r) {
      idx = r * l.cols;
      r_end = idx + (l.cols - border);
      idx += border;

      for (; idx < r_end; ++idx) {
        pdmap[idx] = -1;

        FT xs = pgx[idx];
        FT ys = pgy[idx];

        LT l1 = pl[idx];
        LT l2 = pl[static_cast<index_type>(idx + static_cast<int>(std::round(xs)) +
                                           static_cast<int>(std::round(ys) * l.cols))];

        if (neg_sign(l1, l2) && !eps_zero(l1) && !eps_zero(l2)) {
          LT diff = std::abs(l1 - l2);
          if (diff > plow[idx]) {
            EM<LT, FT>::map(pdmap[idx], xs, ys);
            if (diff > phigh[idx]) seeds.push_back(idx);
          }
        }
      }
    }
  }

  // compute zero crossing by given direction map and laplacian map
  static void process(const cv::Mat& dir,
                      const cv::Mat& l,
                      LT low,
                      LT high,
                      FT r_low,
                      FT r_high,
                      IndexVector& seeds,
                      cv::Mat& dmap,
                      int border = 2) {
    static FT pi2 = static_cast<FT>(2 * CV_PI), epsilon = static_cast<FT>(0.01);
    FT r_size = r_high - r_low;
    cv::Mat dirf = dir, gxf, gyf;
    if (r_size > pi2 + epsilon || r_size < pi2 - epsilon) {
      dirf = dir * (pi2 / r_size);
    }

    P<FT, FT>::polar2Cart(dirf, gxf, gyf);
    processN(gxf, gyf, l, low, high, seeds, dmap, border);
  }

  // compute zero crossing by given direction map and laplacian map
  static void process(const cv::Mat& dir,
                      const cv::Mat& l,
                      const cv::Mat& low,
                      const cv::Mat& high,
                      FT r_low,
                      FT r_high,
                      IndexVector& seeds,
                      cv::Mat& dmap,
                      int border = 2) {
    static FT pi2 = static_cast<FT>(2 * CV_PI), epsilon = static_cast<FT>(0.01);
    FT r_size = r_high - r_low;
    cv::Mat dirf = dir, gxf, gyf;
    if (r_size > pi2 + epsilon || r_size < pi2 - epsilon) {
      dirf = dir * (pi2 / r_size);
    }

    P<FT, FT>::polar2Cart(dirf, gxf, gyf);
    processN(gxf, gyf, l, low, high, seeds, dmap, border);
  }

  static void process(const cv::Mat& l, LT low, LT high, IndexVector& seeds, cv::Mat& dmap, int border = 2) {
    zc_base<LT, FT, NCC, EM>(l, low, high, seeds, dmap, border);
  }

  static void process(
      const cv::Mat& l, const cv::Mat& low, const cv::Mat& high, IndexVector& seeds, cv::Mat& dmap, int border = 2) {
    zc_base<LT, FT, NCC, EM>(l, low, high, seeds, dmap, border);
  }

  static inline std::string name() { return std::string("FastZC") + EM<LT, FT>::type(); }
};


template <class GT, class LT, class DT = float, class ZC = FastZC<GT, LT, DT>>
class ZeroCrossing : public ValueManager {
  cv::Mat dmap_;       // direction map, storing direction 0-X for each value that is above lower
                       // threshold, else -1 (border values are set all to -1); be 4, 8
  IndexVector seeds_;  // indexes of values above higher treshold (border values not included)


  double th_low_, th_high_;
  int border_;

 public:
  typedef GT grad_type;
  typedef LT lapalce_type;
  typedef DT dir_type;
  static constexpr int NUM_DIR = ZC::NUM_DIR;

  ZeroCrossing(double low = 0.004, double high = 0.012, int border = 2)
      : th_low_(low), th_high_(high), border_(border) {
    this->add("nms_th_low", std::bind(&ZeroCrossing<GT, LT, DT, ZC>::valueThresholdLow, this, std::placeholders::_1),
              "Lower threshold.");
    this->add("nms_th_high", std::bind(&ZeroCrossing<GT, LT, DT, ZC>::valueThresholdHigh, this, std::placeholders::_1),
              "Upper threshold.");
    this->add("nms_border", std::bind(&ZeroCrossing<GT, LT, DT, ZC>::valueBorder, this, std::placeholders::_1),
              "Border that is skipped for ZC.");
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

  inline const cv::Mat& directionMap() const { return dmap_; }

  inline const IndexVector& seeds() const { return seeds_; }

  template <class LAPLACE, class QUAD>
  inline void processQ(LAPLACE& laplace, QUAD& quad) {
    LT low = laplace.laplaceThreshold(th_low_);
    LT high = laplace.laplaceThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    process(quad.oddx(), quad.oddy(), laplace.laplace(), low, high);
  }

  template <class QUAD>
  inline void processQ(QUAD& quad) {
    LT low = quad.evenThreshold(th_low_);
    LT high = quad.evenThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    process(quad.oddx(), quad.oddy(), quad.even(), low, high);
  }

  template <class QUAD>
  inline void processQD(QUAD& quad) {
    LT low = quad.evenThreshold(th_low_);
    LT high = quad.evenThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    typename QUAD::DirectionRange dr = quad.directionRange();
    process(quad.direction(), quad.even(), low, high, dr.lower, dr.upper);
  }

  template <class LAPLACE, class GRAD>
  inline void processG(LAPLACE& laplace, GRAD& grad) {
    LT low = laplace.laplaceThreshold(th_low_);
    LT high = laplace.laplaceThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    process(grad.gx(), grad.gy(), laplace.laplace(), low, high);
  }

  template <class LAPLACE, class GRAD>
  inline void processD(LAPLACE& laplace, GRAD& grad) {
    LT low = laplace.laplaceThreshold(th_low_);
    LT high = laplace.laplaceThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    typename GRAD::DirectionRange dr = grad.directionRange();
    process(grad.direction(), laplace.laplace(), low, high, dr.lower, dr.upper);
  }

  template <class LAPLACE>
  inline void process(const cv::Mat& img) {
    LAPLACE laplace;
    laplace.process(img);
    process(laplace, th_low_, th_high_);
  }

  template <class LAPLACE>
  inline void process(LAPLACE& laplace) {
    LT low = laplace.laplaceThreshold(th_low_);
    LT high = laplace.laplaceThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);


    process(laplace.laplace(), low, high);
  }

  template <class LAPLACE>
  inline void process(LAPLACE& laplace, const cv::Mat data) {
    LT low = laplace.laplaceThreshold(th_low_);
    LT high = laplace.laplaceThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);


    process(data, low, high);
  }

  // compute non zero crossing by given derivative maps and laplacian map
  inline void process(const cv::Mat& gx, const cv::Mat& gy, const cv::Mat& l, LT low, LT high) {
    ZC::process(gx.isContinuous() ? gx : gx.clone(), gy.isContinuous() ? gy : gy.clone(),
                l.isContinuous() ? l : l.clone(), low, high, seeds_, dmap_, border_);
  }

  // compute non zero crossing by given direction map and laplacian map
  inline void process(const cv::Mat& dir, const cv::Mat& l, LT low, LT high, DT r_low, DT r_high) {
    ZC::process(dir.isContinuous() ? dir : dir.clone(), l.isContinuous() ? l : l.clone(), low, high, r_low, r_high,
                seeds_, dmap_, border_);
  }

  // compute non zero crossing laplacian map
  inline void process(const cv::Mat& l, LT low, LT high) {
    ZC::process(l.isContinuous() ? l : l.clone(), low, high, seeds_, dmap_, border_);
  }

  // compute non zero crossing by given derivative maps and laplacian map
  inline void process(const cv::Mat& gx, const cv::Mat& gy, const cv::Mat& l, const cv::Mat& low, const cv::Mat& high) {
    ZC::process(gx.isContinuous() ? gx : gx.clone(), gy.isContinuous() ? gy : gy.clone(),
                l.isContinuous() ? l : l.clone(), low.isContinuous() ? low : low.clone(),
                high.isContinuous() ? high : high.clone(), seeds_, dmap_, border_);
  }

  // compute non zero crossing by given direction map and laplacian map
  inline void process(
      const cv::Mat& dir, const cv::Mat& l, const cv::Mat& low, const cv::Mat& high, DT r_low, DT r_high) {
    ZC::process(dir.isContinuous() ? dir : dir.clone(), l.isContinuous() ? l : l.clone(),
                low.isContinuous() ? low : low.clone(), high.isContinuous() ? high : high.clone(), r_low, r_high,
                seeds_, dmap_, border_);
  }

  // compute non zero crossing laplacian map
  inline void process(const cv::Mat& l, const cv::Mat& low, const cv::Mat& high) {
    ZC::process(l.isContinuous() ? l : l.clone(), low.isContinuous() ? low : low.clone(),
                high.isContinuous() ? high : high.clone(), seeds_, dmap_, border_);
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

  //! get name
  inline std::string name() const { return ZC::name(); }
};
}  // namespace lsfm
#endif
#endif
