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

#include <geometry/point.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace lsfm {

// helpers to read data from ptr or mat

//! read value at pos x from src, if x is out of bounds, uses border type to determine value
//! (see http://docs.opencv.org/modules/imgproc/doc/filtering.html)
template <class MT>
inline MT readValX(const MT* src, int cols, int x, int border_type, MT border_val = 0) {
  if (x >= 0 && x < cols) return src[x];
  return border_type == cv::BORDER_CONSTANT ? border_val : src[cv::borderInterpolate(x, cols, border_type)];
}

//! read value at pos x from src, if x is out of bounds, uses border replicate
template <class MT>
inline MT readValX(const MT* src, int cols, int x) {
  return src[x < 0 ? 0 : x >= cols ? cols - 1 : x];
}

//! read value at pos y from src, if y is out of bounds, uses border type to determine value
template <class MT>
inline MT readValY(const MT* src, int rows, int cols, int y, int border_type, MT border_val = 0) {
  if (y >= 0 && y < rows) return src[y * cols];
  return border_type == cv::BORDER_CONSTANT ? border_val : src[cv::borderInterpolate(y, rows, border_type) * cols];
}

//! read value at pos x from src, if x is out of bounds, uses border replicate
template <class MT>
inline MT readValY(const MT* src, int rows, int cols, int y) {
  return src[y < 0 ? 0 : y >= rows ? (rows - 1) * cols : y * cols];
}

//! read value at pos x,y from src, if x or y is out of bounds, uses border type to determine value
template <class MT>
inline MT readVal(const cv::Mat& src, int y, int x, int border_type, MT border_val = 0) {
  if (y >= 0 && y < src.rows && x >= 0 && x < src.cols) return src.at<MT>(y, x);
  return border_type == cv::BORDER_CONSTANT ? border_val
                                            : src.at<MT>(cv::borderInterpolate(y, src.rows, border_type),
                                                         cv::borderInterpolate(x, src.cols, border_type));
}

//! read value at pos x,y from src, if x or y is out of bounds, uses border replicate
template <class MT>
inline MT readVal(const cv::Mat& src, int y, int x) {
  return src.at<MT>(y < 0 ? 0 : y >= src.rows ? src.rows - 1 : y, x < 0 ? 0 : x >= src.cols ? src.cols - 1 : x);
}

//! read value at pos x,y from src, if x or y is out of bounds, uses border type to determine value
template <class MT>
inline MT readVal(
    const cv::Mat& src, int y, int x, int border_type_x, int border_type_y, MT border_val_x = 0, MT border_val_y = 0) {
  if (y >= 0 && y < src.rows && x >= 0 && x < src.cols) return src.at<MT>(y, x);

  if (border_type_x == cv::BORDER_CONSTANT && (x < 0 || x >= src.cols)) return border_val_x;
  if (border_type_y == cv::BORDER_CONSTANT && (y < 0 || y >= src.rows)) return border_val_y;

  return src.at<MT>(cv::borderInterpolate(y, src.rows, border_type_y),
                    cv::borderInterpolate(x, src.cols, border_type_x));
}

//! template class for nearest neighbhor interpolation
template <class FT, class MT, class RT = FT>
struct NearestInterpolator {
  typedef FT float_type;
  typedef MT mat_type;
  static constexpr int BorderStart = 0;
  static constexpr int BorderEnd = 0;
  ////////// 1D interpolation

  //! get 1D interpolation at pos fx in src (no boundary check)
  static inline RT getXNB(const MT* src, FT fx) { return src[static_cast<int>(fx)]; }

  //! get 1D interpolation at pos fy in src (no boundary check)
  static inline RT getYNB(const MT* src, int cols, FT fy) { return src[static_cast<int>(fy) * cols]; }

  //! get 1D interpolation at pos fx in src
  static inline RT getX(const MT* src, int cols, FT fx) { return readValX<MT>(src, cols, static_cast<int>(fx)); }

  //! get 1D interpolation at pos fy in src
  static inline RT getY(const MT* src, int cols, int rows, FT fy) {
    return readValY<MT>(src, rows, cols, static_cast<int>(fy));
  }

  //! get 1D interpolation at pos fx in src using border type if out of bounds
  static inline RT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    return readValX<MT>(src, cols, static_cast<int>(fx), border_type, border_val);
  }

  //! get 1D interpolation at pos fy in src using border type if out of bounds
  static inline RT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    return readValY<MT>(src, rows, cols, static_cast<int>(fy), border_type, border_val);
  }

  //! get 2D interpolation at src at pos fx, fy (no bound check)
  static inline RT getNB(const cv::Mat& src, FT fx, FT fy) {
    return src.at<MT>(static_cast<int>(fy), static_cast<int>(fx));
  }

  //! get 2D interpolation at src at pos p (x,y) (no bound check)
  static inline RT getNB(const cv::Mat& src, const FT* p) {
    return NearestInterpolator<FT, MT>::getNB(src, p[0], p[1]);
  }

  //! get 2D interpolation at src at pos p (x,y) (no bound check)
  template <class PT>
  static inline RT getNB(const cv::Mat& src, const PT& p) {
    return NearestInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }

  //! get 2D interpolation at src at pos fx, fy
  static inline RT get(const cv::Mat& src, FT fx, FT fy) {
    return readVal<MT>(src, static_cast<int>(fy), static_cast<int>(fx));
  }

  //! get 2D interpolation at src at pos p (x,y)
  static inline RT get(const cv::Mat& src, const FT* p) { return NearestInterpolator<FT, MT>::get(src, p[0], p[1]); }

  //! get 2D interpolation at src at pos p (x,y)
  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p) {
    return NearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }

  //! get 2D interpolation at pos fx, fy in src using border type if out of bounds
  static inline RT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    return static_cast<FT>(readVal<MT>(src, static_cast<int>(fy), static_cast<int>(fx), border_type, border_val));
  }

  //! get 2D interpolation at pos p in src using border type if out of bounds
  static inline RT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return NearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  //! get 2D interpolation at pos p in src using border type if out of bounds
  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return NearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  //! get 2D interpolation at pos fx, fy in src using border type if out of bounds
  static inline RT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    return static_cast<FT>(readVal<MT>(src, static_cast<int>(fy), static_cast<int>(fx), border_type_x, border_type_y,
                                       border_val_x, border_val_y));
  }

  //! get 2D interpolation at pos p in src using border type if out of bounds
  static inline RT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return NearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x, border_val_y);
  }

  //! get 2D interpolation at pos p in src using border type if out of bounds
  template <class PT>
  static inline RT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return NearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y,
                                            border_val_x, border_val_y);
  }
};

//! template class for rounded nearest neighbhor interpolation (from pixel center)
template <class FT, class MT, class RT = FT>
struct RoundNearestInterpolator {
  typedef FT float_type;
  typedef MT mat_type;

  static constexpr int BorderStart = 0;
  static constexpr int BorderEnd = 0;

  static inline RT getXNB(const MT* src, FT fx) { return src[static_cast<int>(round(fx))]; }

  static inline RT getYNB(const MT* src, int cols, FT fy) { return src[static_cast<int>(round(fy)) * cols]; }

  static inline RT getX(const MT* src, int cols, FT fx) { return readValX<MT>(src, cols, static_cast<int>(round(fx))); }

  static inline RT getY(const MT* src, int cols, int rows, FT fy) {
    return readValY<MT>(src, rows, cols, static_cast<int>(round(fy)));
  }

  static inline RT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    return readValX<MT>(src, cols, static_cast<int>(round(fx)), border_type, border_val);
  }

  static inline RT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    return readValY<MT>(src, rows, cols, static_cast<int>(round(fy)), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src, FT fx, FT fy) {
    return readVal<MT>(src, static_cast<int>(round(fy)), static_cast<int>(round(fx)));
  }

  static inline RT get(const cv::Mat& src, const FT* p) {
    return RoundNearestInterpolator<FT, MT>::get(src, p[0], p[1]);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p) {
    return RoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }


  static inline RT getNB(const cv::Mat& src, FT fx, FT fy) {
    return src.at<MT>(static_cast<int>(round(fy)), static_cast<int>(round(fx)));
  }

  static inline RT getNB(const cv::Mat& src, const FT* p) {
    return RoundNearestInterpolator<FT, MT>::getNB(src, p[0], p[1]);
  }

  template <class PT>
  static inline RT getNB(const cv::Mat& src, const PT& p) {
    return RoundNearestInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }


  static inline RT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    return readVal<MT>(src, static_cast<int>(round(fy)), static_cast<int>(round(fx)), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return RoundNearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return RoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    return static_cast<FT>(readVal<MT>(src, static_cast<int>(round(fy)), static_cast<int>(round(fx)), border_type_x,
                                       border_type_y, border_val_x, border_val_y));
  }

  static inline RT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return RoundNearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x,
                                                 border_val_y);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return RoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y,
                                                 border_val_x, border_val_y);
  }
};

//! template class for rounded nearest neighbhor interpolation (from pixel center)
template <class FT, class MT, class RT = FT>
struct FastRoundNearestInterpolator {
  typedef FT float_type;
  typedef MT mat_type;

  static constexpr int BorderStart = 0;
  static constexpr int BorderEnd = 0;

  static inline RT getXNB(const MT* src, FT fx) { return src[static_cast<int>(fx + 0.5)]; }

  static inline RT getYNB(const MT* src, int cols, FT fy) { return src[static_cast<int>(fy + 0.5) * cols]; }

  static inline RT getX(const MT* src, int cols, FT fx) { return readValX<MT>(src, cols, static_cast<int>(fx + 0.5)); }

  static inline RT getY(const MT* src, int cols, int rows, FT fy) {
    return readValY<MT>(src, rows, cols, static_cast<int>(fy + 0.5));
  }

  static inline RT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    return readValX<MT>(src, cols, static_cast<int>(fx + 0.5), border_type, border_val);
  }

  static inline RT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    return readValY<MT>(src, rows, cols, static_cast<int>(fy + 0.5), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src, FT fx, FT fy) {
    return readVal<MT>(src, static_cast<int>(fy + 0.5), static_cast<int>(fx + 0.5));
  }

  static inline RT get(const cv::Mat& src, const FT* p) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, p[0], p[1]);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline RT getNB(const cv::Mat& src, FT fx, FT fy) {
    return src.at<MT>(static_cast<int>(fy + 0.5), static_cast<int>(fx + 0.5));
  }

  static inline RT getNB(const cv::Mat& src, const FT* p) {
    return FastRoundNearestInterpolator<FT, MT>::getNB(src, p[0], p[1]);
  }

  template <class PT>
  static inline RT getNB(const cv::Mat& src, const PT& p) {
    return FastRoundNearestInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline RT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    return readVal<MT>(src, static_cast<int>(fy + 0.5), static_cast<int>(fx + 0.5), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    return readVal<MT>(src, static_cast<int>(fy + 0.5), static_cast<int>(fx + 0.5), border_type_x, border_type_y,
                       border_val_x, border_val_y);
  }

  static inline RT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x,
                                                     border_val_y);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y,
                                                     border_val_x, border_val_y);
  }
};


//! 1D linear interpolation
//! data is pointer to array of 2 values to be interpolated
//! x is position between value 0 and 1
template <class FT, class MT>
inline FT interpolate_linear(const MT* data, FT x) {
  return (data[1] - data[0]) * x + data[0];
}

template <class FT, class MT>
inline FT interpolate_linear(const Vec<MT, 2>& data, FT x) {
  return interpolate_linear<FT, MT>(&getX(data), x);
}

template <class FT, class MT>
inline FT interpolate_linear(MT a, MT b, FT x) {
  return (b - a) * x + a;
}

template <class FT, class MT>
struct LinearInterpolator {
  typedef FT float_type;
  typedef MT mat_type;

  static constexpr int BorderStart = 0;
  static constexpr int BorderEnd = 1;

  static inline FT getXNB(const MT* src, FT fx) {
    int xi = static_cast<int>(fx);
    return interpolate_linear<FT, MT>(src + xi, fx - xi);
  }

  static inline FT getYNB(const MT* src, int cols, FT fy) {
    int yi = static_cast<int>(fy) * cols;
    return interpolate_linear<FT, MT>(src[yi], src[yi + cols], fy - static_cast<int>(fy));
  }

  static inline FT getX(const MT* src, int cols, FT fx) {
    int xi = static_cast<int>(fx);
    return interpolate_linear<FT, MT>(readValX(src, cols, xi), readValX(src, cols, xi + 1), fx - xi);
  }

  static inline FT getY(const MT* src, int cols, int rows, FT fy) {
    int yi = static_cast<int>(fy);
    return interpolate_linear<FT, MT>(readValY(src, rows, cols, yi), readValY(src, rows, cols, yi + 1), fy - yi);
  }

  static inline FT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    int xi = static_cast<int>(fx);
    return interpolate_linear<FT, MT>(readValX(src, cols, xi, border_type, border_val),
                                      readVal(src, cols, xi + 1, border_type, border_val), fx - xi);
  }

  static inline FT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    int yi = static_cast<int>(fy);
    return interpolate_linear<FT, MT>(readValY(src, rows, cols, yi, border_type, border_val),
                                      readVal(src, rows, cols, yi + 1, border_type, border_val), fy - yi);
  }

  static inline FT get(const cv::Mat& src, FT fx, FT fy) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= xi;
    fy -= yi;

    if (fx == 0 || fy == 0) {
      // interpolate with 2 pixels along y
      if (fy != 0) return interpolate_linear<FT, MT>({readVal<MT>(src, yi, xi), readVal<MT>(src, yi + 1, xi)}, fy);

      // interpolate with 2 pixels along x
      if (fx != 0) {
        if (xi >= 0 && xi < src.cols - 1) return interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx);
        return interpolate_linear<FT, MT>({readVal<MT>(src, yi, xi), readVal<MT>(src, yi, xi + 1)}, fx);
      }

      // no interpolation required, get entry from src
      static_cast<FT>(readVal<MT>(src, yi, xi));
    }

    // interpolate with 4 pixels
    if (xi >= 0 && xi < src.cols - 1 && yi >= 0 && yi < src.rows - 1)
      return interpolate_linear<FT, FT>({interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx),
                                         interpolate_linear<FT, MT>(&src.at<MT>(yi + 1, xi), fx)},
                                        fy);
    return interpolate_linear<FT, FT>(
        {interpolate_linear<FT, MT>({readVal<MT>(src, yi, xi), readVal<MT>(src, yi, xi + 1)}, fx),
         interpolate_linear<FT, MT>({readVal<MT>(src, yi + 1, xi), readVal<MT>(src, yi + 1, xi + 1)}, fx)},
        fy);
  }

  static inline FT get(const cv::Mat& src, const FT* p) { return LinearInterpolator<FT, MT>::get(src, p[0], p[1]); }

  template <class PT>
  static inline FT get(const cv::Mat& src, const PT& p) {
    return LinearInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline FT getNB(const cv::Mat& src, FT fx, FT fy) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);
    fx -= xi;
    fy -= yi;

    /*if (fx == 0 || fy == 0) {
        // interpolate with 2 pixels along y
        if (fy != 0)
            return interpolate_linear<FT, MT>(src.at<MT>(yi, xi), src.at<MT>(yi + 1, xi), fy);

        // interpolate with 2 pixels along x
        if (fx != 0)
            return interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx);

        // no interpolation required, get entry from src
        static_cast<FT>(readVal<MT>(src, yi, xi));
    }*/
    return interpolate_linear<FT, FT>(interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx),
                                      interpolate_linear<FT, MT>(&src.at<MT>(yi + 1, xi), fx), fy);
  }

  static inline FT getNB(const cv::Mat& src, const FT* p) { return LinearInterpolator<FT, MT>::getNB(src, p[0], p[1]); }

  template <class PT>
  static inline FT getNB(const cv::Mat& src, const PT& p) {
    return LinearInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline FT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= xi;
    fy -= yi;

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 2 pixels along y
      if (fy != 0)
        return interpolate_linear<FT, MT>(
            {readVal<MT>(src, yi, xi, border_type, border_val), readVal<MT>(src, yi + 1, xi, border_type, border_val)},
            fy);

      // interpolate with 2 pixels along x
      if (fx != 0) {
        if (xi >= 0 && xi < src.cols - 1) return interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx);
        return interpolate_linear<FT, MT>(
            {readVal<MT>(src, yi, xi, border_type, border_val), readVal<MT>(src, yi, xi + 1, border_type, border_val)},
            fx);
      }

      static_cast<FT>(readVal<MT>(src, yi, xi));
    }


    // interpolate with 4 pixels
    if (xi >= 0 && xi < src.cols - 1 && yi >= 0 && yi < src.rows - 1)
      return interpolate_linear<FT, FT>({interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx),
                                         interpolate_linear<FT, MT>(&src.at<MT>(yi + 1, xi), fx)},
                                        fy);
    return interpolate_linear<FT, FT>(
        {interpolate_linear<FT, MT>(
             {readVal<MT>(src, yi, xi, border_type, border_val), readVal<MT>(src, yi, xi + 1, border_type, border_val)},
             fx),
         interpolate_linear<FT, MT>({readVal<MT>(src, yi + 1, xi, border_type, border_val),
                                     readVal<MT>(src, yi + 1, xi + 1, border_type, border_val)},
                                    fx)},
        fy);
  }

  static inline FT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return LinearInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  template <class PT>
  static inline FT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return LinearInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  static inline FT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= xi;
    fy -= yi;

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 2 pixels along y
      if (fy != 0)
        return interpolate_linear<FT, MT>(
            {readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
             readVal<MT>(src, yi + 1, xi, border_type_x, border_type_y, border_val_x, border_val_y)},
            fy);

      // interpolate with 2 pixels along x
      if (fx != 0) {
        if (xi >= 0 && xi < src.cols - 1) return interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx);
        return interpolate_linear<FT, MT>(
            {readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
             readVal<MT>(src, yi, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y)},
            fx);
      }

      static_cast<FT>(readVal<MT>(src, yi, xi));
    }

    // interpolate with 4 pixels
    if (xi >= 0 && xi < src.cols - 1 && yi >= 0 && yi < src.rows - 1)
      return interpolate_linear<FT, FT>({interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx),
                                         interpolate_linear<FT, MT>(&src.at<MT>(yi + 1, xi), fx)},
                                        fy);
    return interpolate_linear<FT, FT>(
        {interpolate_linear<FT, MT>(
             {readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y)},
             fx),
         interpolate_linear<FT, MT>(
             {readVal<MT>(src, yi + 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 1, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y)},
             fx)},
        fy);
  }

  static inline FT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return LinearInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x, border_val_y);
  }

  template <class PT>
  static inline FT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return LinearInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y,
                                           border_val_x, border_val_y);
  }
};


//! 1D cubic interpolation (Catmull-Rom spline)
//! data is pointer to array of 4 values to be interpolated
//! x is position between value 1 and 2
//! see http://www.paulinternet.nl/?page=bicubic
template <class FT, class MT>
inline FT interpolate_cubic(const MT* data, FT x) {
  return data[1] + 0.5 * x *
                       (data[2] - data[0] +
                        x * (2.0 * data[0] - 5.0 * data[1] + 4.0 * data[2] - data[3] +
                             x * (3.0 * (data[1] - data[2]) + data[3] - data[0])));
}

template <class FT, class MT>
inline FT interpolate_cubic(const Vec<MT, 4>& data, FT x) {
  return interpolate_cubic(data.val, x);
}

template <class FT, class MT>
inline FT interpolate_cubic(MT a, MT b, MT c, MT d, FT x) {
  return b + 0.5 * x * (c - a + x * (2.0 * a - 5.0 * b + 4.0 * c - d + x * (3.0 * (b - c) + d - a)));
}

template <class FT, class MT>
struct CubicInterpolator {
  typedef FT float_type;
  typedef MT mat_type;

  static constexpr int BorderStart = 1;
  static constexpr int BorderEnd = 2;

  static inline FT getXNB(const MT* src, FT fx) {
    int xi = static_cast<int>(fx);
    return interpolate_cubic<FT, MT>(src + xi - 1, fx - xi);
  }

  static inline FT getYNB(const MT* src, int cols, FT fy) {
    int yi = static_cast<int>(fy);
    src += yi * cols;
    return interpolate_cubic<FT, MT>(src[-cols], src[0], src[cols], src[2 * cols], fy - yi);
  }

  static inline FT getX(const MT* src, int cols, FT fx) {
    int xi = static_cast<int>(fx);

    // border case
    if (xi < 1 || xi > src.cols - 3)
      return interpolate_cubic<FT, MT>(readValX<MT>(src, cols, xi - 1), readValX<MT>(src, cols, xi),
                                       readValX<MT>(src, cols, xi + 1), readValX<MT>(src, cols, xi + 2), fx - xi);

    return interpolate_cubic<FT, MT>(src + xi - 1, fx - xi);
  }

  static inline FT getY(const MT* src, int cols, int rows, FT fy) {
    int yi = static_cast<int>(fy);
    fy -= yi;

    // border case
    if (yi < 1 || yi > src.rows - 3)
      return interpolate_cubic<FT, MT>(readValY<MT>(src, rows, cols, yi - 1), readValY<MT>(src, rows, cols, yi),
                                       readValY<MT>(src, rows, cols, yi + 1), readValY<MT>(src, rows, cols, yi + 2),
                                       fy);
    src += yi * cols;
    return interpolate_cubic<FT, MT>(src[-cols], src[0], src[cols], src[2 * cols], fy);
  }


  static inline FT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    int xi = static_cast<int>(fx);
    fx -= xi;

    // if (fx == 0)
    //     static_cast<FT>(readValX(src, cols, xi), border_type, border_val);

    // border case
    if (xi < 1 || xi > src.cols - 3)
      return interpolate_cubic<FT, MT>(readValX<MT>(src, cols, xi - 1, border_type, border_val),
                                       readValX<MT>(src, cols, xi, border_type, border_val),
                                       readValX<MT>(src, cols, xi + 1, border_type, border_val),
                                       readValX<MT>(src, cols, xi + 2, border_type, border_val), fx);

    return interpolate_cubic<FT, MT>(src + xi - 1, fx);
  }

  static inline FT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    int yi = static_cast<int>(fy);
    fy -= yi;

    // if (fy == 0)
    //     static_cast<FT>(readValY(src, rows, cols, yi), border_type, border_val);

    // border case
    if (yi < 1 || yi > rows - 3)
      return interpolate_cubic<FT, MT>(readValY<MT>(src, rows, cols, yi - 1, border_type, border_val),
                                       readValY<MT>(src, rows, cols, yi, border_type, border_val),
                                       readValY<MT>(src, rows, cols, yi + 1, border_type, border_val),
                                       readValY<MT>(src, rows, cols, yi + 2, border_type, border_val), fy);

    src += yi * cols;
    return interpolate_cubic<FT, MT>(src[-cols], src[0], src[cols], src[2 * cols], fy);
  }

  static inline FT get(const cv::Mat& src, FT fx, FT fy) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= xi;
    fy -= yi;

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 4 pixels along y
      if (fy != 0)
        return interpolate_cubic<FT, MT>(readVal<MT>(src, yi - 1, xi), readVal<MT>(src, yi, xi),
                                         readVal<MT>(src, yi + 1, xi), readVal<MT>(src, yi + 2, xi), fy);


      // interpolate with 4 pixels along x
      if (fx != 0) {
        if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
          // interpolate with 4 pixels along x
          return interpolate_cubic<FT, MT>(readVal<MT>(src, yi, xi - 1), readVal<MT>(src, yi, xi),
                                           readVal<MT>(src, yi, xi + 1), readVal<MT>(src, yi, xi + 2), fx);

        return interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx);
      }

      return static_cast<FT>(readVal<MT>(src, yi, xi));
    }

    // border case
    if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
      return interpolate_cubic<FT, FT>(
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi - 1, xi - 1), readVal<MT>(src, yi - 1, xi),
                                    readVal<MT>(src, yi - 1, xi + 1), readVal<MT>(src, yi - 1, xi + 2), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi, xi - 1), readVal<MT>(src, yi, xi),
                                    readVal<MT>(src, yi, xi + 1), readVal<MT>(src, yi, xi + 2), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi + 1, xi - 1), readVal<MT>(src, yi + 1, xi),
                                    readVal<MT>(src, yi + 1, xi + 1), readVal<MT>(src, yi + 1, xi + 2), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi + 2, xi - 1), readVal<MT>(src, yi + 2, xi),
                                    readVal<MT>(src, yi + 2, xi + 1), readVal<MT>(src, yi + 2, xi + 2), fx),
          fy);

    // interpolate with 16 pixels
    return interpolate_cubic<FT, FT>(interpolate_cubic<FT, MT>(&src.at<MT>(yi - 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 2, xi - 1), fx), fy);
  }

  static inline FT get(const cv::Mat& src, const FT* p) { return CubicInterpolator<FT, MT>::get(src, p[0], p[1]); }

  template <class PT>
  static inline FT get(const cv::Mat& src, const PT& p) {
    return CubicInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline FT getNB(const cv::Mat& src, FT fx, FT fy) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= xi;
    fy -= yi;

    if (fx == 0 || fy == 0) {
      if (fy != 0)
        return interpolate_cubic<FT, MT>(src.at<MT>(yi - 1, xi), src.at<MT>(yi, xi), src.at<MT>(yi + 1, xi),
                                         src.at<MT>(yi + 2, xi), fy);

      if (fx != 0) return interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx);

      return static_cast<FT>(src.at<MT>(yi, xi));
    }

    // interpolate with 16 pixels
    return interpolate_cubic<FT, FT>(interpolate_cubic<FT, MT>(&src.at<MT>(yi - 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 2, xi - 1), fx), fy);
  }

  static inline FT getNB(const cv::Mat& src, const FT* p) { return CubicInterpolator<FT, MT>::getNB(src, p[0], p[1]); }

  template <class PT>
  static inline FT getNB(const cv::Mat& src, const PT& p) {
    return CubicInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline FT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= xi;
    fy -= yi;

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 4 pixels along y
      if (fy != 0)
        return interpolate_cubic<FT, MT>(readVal<MT>(src, yi - 1, xi, border_type, border_val),
                                         readVal<MT>(src, yi, xi, border_type, border_val),
                                         readVal<MT>(src, yi + 1, xi, border_type, border_val),
                                         readVal<MT>(src, yi + 2, xi, border_type, border_val), fy);


      // interpolate with 4 pixels along x
      if (fx != 0) {
        // border case
        if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
          return interpolate_cubic<FT, MT>(readVal<MT>(src, yi, xi - 1, border_type, border_val),
                                           readVal<MT>(src, yi, xi, border_type, border_val),
                                           readVal<MT>(src, yi, xi + 1, border_type, border_val),
                                           readVal<MT>(src, yi, xi + 2, border_type, border_val), fx);

        return interpolate_cubic<FT, MT>(src.ptr<MT>(yi) + xi - 1, fx);
      }

      return static_cast<FT>(readVal<MT>(src, yi, xi));
    }

    // border case
    if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
      return interpolate_cubic<FT, FT>(
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi - 1, xi - 1, border_type, border_val),
                                    readVal<MT>(src, yi - 1, xi, border_type, border_val),
                                    readVal<MT>(src, yi - 1, xi + 1, border_type, border_val),
                                    readVal<MT>(src, yi - 1, xi + 2, border_type, border_val), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi, xi - 1, border_type, border_val),
                                    readVal<MT>(src, yi, xi, border_type, border_val),
                                    readVal<MT>(src, yi, xi + 1, border_type, border_val),
                                    readVal<MT>(src, yi, xi + 2, border_type, border_val), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi + 1, xi - 1, border_type, border_val),
                                    readVal<MT>(src, yi + 1, xi, border_type, border_val),
                                    readVal<MT>(src, yi + 1, xi + 1, border_type, border_val),
                                    readVal<MT>(src, yi + 1, xi + 2, border_type, border_val), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi + 2, xi - 1, border_type, border_val),
                                    readVal<MT>(src, yi + 2, xi, border_type, border_val),
                                    readVal<MT>(src, yi + 2, xi + 1, border_type, border_val),
                                    readVal<MT>(src, yi + 2, xi + 2, border_type, border_val), fx),
          fy);

    // interpolate with 16 pixels
    return interpolate_cubic<FT, FT>(interpolate_cubic<FT, MT>(&src.at<MT>(yi - 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 2, xi - 1), fx), fy);
  }

  static inline FT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return CubicInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  template <class PT>
  static inline FT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return CubicInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  static inline FT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= xi;
    fy -= yi;

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 4 pixels along y
      if (fy != 0)
        return interpolate_cubic<FT, MT>(
            readVal<MT>(src, yi - 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
            readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
            readVal<MT>(src, yi + 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
            readVal<MT>(src, yi + 2, xi, border_type_x, border_type_y, border_val_x, border_val_y), fy);
      // interpolate with 4 pixels along x
      if (fx != 0) {
        if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
          return interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx);

        return interpolate_cubic<FT, MT>(src.ptr<MT>(yi) + xi - 1, fx);
      }

      return static_cast<FT>(readVal<MT>(src, yi, xi));
    }


    // border case
    if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
      return interpolate_cubic<FT, FT>(
          interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi - 1, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi - 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi - 1, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi - 1, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx),
          interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx),
          interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi + 1, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 1, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 1, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx),
          interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi + 2, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 2, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 2, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 2, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx),
          fy);

    // interpolate with 16 pixels
    return interpolate_cubic<FT, FT>(interpolate_cubic<FT, MT>(&src.at<MT>(yi - 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 2, xi - 1), fx), fy);
  }

  static inline FT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return CubicInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x, border_val_y);
  }

  template <class PT>
  static inline FT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return CubicInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y, border_val_x,
                                          border_val_y);
  }
};

//! template interpolator helper class
template <class Interpolator, int BorderType = cv::BORDER_DEFAULT, int BorderValue = 0>
struct FixedBorderInterpolator {
  typedef typename Interpolator::float_type float_type;
  typedef typename Interpolator::mat_type mat_type;


  static inline float_type getX(const mat_type* src, int cols, float_type fx) {
    return Interpolator::getX(src, cols, fx, BorderType, BorderValue);
  }

  static inline float_type getY(const mat_type* src, int rows, float_type fy) {
    return Interpolator::getY(src, rows, fy, BorderType, BorderValue);
  }

  static inline float_type get(const cv::Mat& src, float_type fx, float_type fy) {
    return Interpolator::get(src, fx, fy, BorderType, BorderValue);
  }

  static inline float_type get(const cv::Mat& src, const float_type* p) {
    return Interpolator::get(src, p, BorderType, BorderValue);
  }

  template <template <class> class PT>
  static inline float_type get(const cv::Mat& src, const PT<float_type>& p) {
    return Interpolator::get(src, p, BorderType, BorderValue);
  }
};

//! template interpolator helper class
template <class Interpolator,
          int BorderTypeX = cv::BORDER_DEFAULT,
          int BorderTypeY = cv::BORDER_DEFAULT,
          int BorderValueX = 0,
          int BorderValueY = 0>
struct Fixed2BorderInterpolator {
  typedef typename Interpolator::float_type float_type;
  typedef typename Interpolator::mat_type mat_type;

  static inline float_type getX(const mat_type* src, int cols, float_type fx) {
    return Interpolator::getX(src, cols, fx, BorderTypeX, BorderValueX);
  }

  static inline float_type getY(const mat_type* src, int rows, float_type fy) {
    return Interpolator::getY(src, rows, fy, BorderTypeX, BorderValueX);
  }

  static inline float_type get(const cv::Mat& src, float_type fx, float_type fy) {
    return Interpolator::get(src, fx, fy, BorderTypeX, BorderTypeY, BorderValueX, BorderValueY);
  }

  static inline float_type get(const cv::Mat& src, const float_type* p) {
    return Interpolator::get(src, p, BorderTypeX, BorderTypeY, BorderValueX, BorderValueY);
  }

  template <template <class> class PT>
  static inline float_type get(const cv::Mat& src, const PT<float_type>& p) {
    return Interpolator::get(src, p, BorderTypeX, BorderTypeY, BorderValueX, BorderValueY);
  }
};


//! helper to access templated interpolate function pointer
template <class FT, template <class> class PT>
struct InterpolationHelper {
  typedef FT (*func_type)(const cv::Mat&, FT x, FT y);
  typedef FT (*func_type_ptr)(const cv::Mat&, const FT*);
  typedef FT (*func_type_point)(const cv::Mat&, const PT<FT>&);
};

}  // namespace lsfm
