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

#include <opencv2/core/core.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace lsfm {
using std::acos;
using std::asin;
using std::atan;
using std::atan2;
using std::cos;
using std::cosh;
using std::sin;
using std::sinh;
using std::tan;
using std::tanh;


constexpr int IMG_NORM_FALSE = 0;
constexpr int IMG_NORM_TRUE = 1;
constexpr int IMG_NORM_AUTO = 2;

inline cv::Mat normalizeMat(const cv::Mat& in) {
  cv::Mat cpy;
  if (in.type() != CV_32F || in.type() != CV_64F)
    in.convertTo(cpy, CV_32F);
  else
    in.copyTo(cpy);

  double vmin, vmax;
  cv::minMaxIdx(cpy, &vmin, &vmax);
  cpy -= vmin;
  cpy /= vmax - vmin;
  return cpy;
}

inline void showMat(const std::string& name, const cv::Mat& out, int normalize = IMG_NORM_AUTO, bool refresh = true) {
  if (normalize == IMG_NORM_AUTO)
    normalize = (out.type() == CV_8U || out.channels() > 1) ? IMG_NORM_FALSE : IMG_NORM_TRUE;

  cv::Mat cpy;
  if (normalize == IMG_NORM_TRUE) {
    cpy = normalizeMat(out);
  } else
    cpy = out;

  cv::imshow(name, cpy);
  if (refresh) cv::waitKey(1);
}

inline void meshgrid(const cv::Mat& xgv, const cv::Mat& ygv, cv::Mat& X, cv::Mat& Y) {
  cv::repeat(xgv.reshape(1, 1), static_cast<int>(ygv.total()), 1, X);
  cv::repeat(ygv.reshape(1, 1).t(), 1, static_cast<int>(xgv.total()), Y);
}

template <class T>
inline void meshgrid(const cv::Range& xgv, const cv::Range& ygv, cv::Mat& X, cv::Mat& Y) {
  std::vector<T> t_x, t_y;
  t_x.reserve(xgv.size());
  t_y.reserve(ygv.size());
  for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);
  for (int i = ygv.start; i <= ygv.end; i++) t_y.push_back(i);
  meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}

template <class T>
inline void meshgrid(const cv::Range& xgv, const cv::Range& ygv, cv::Mat_<T>& X, cv::Mat_<T>& Y) {
  meshgrid<T>(xgv, ygv, cv::Mat(X), cv::Mat(Y));
}

//! end inclusive!
template <class T>
inline cv::Mat_<T> range(T start, T end) {
  cv::Mat_<T> ret(1, static_cast<int>(end - start) + 1);
  int idx = 0;
  while (start <= end) {
    ret(0, idx++) = start;
    start += 1;
  }
  return ret;
}

//! end inclusive!
template <class T>
inline cv::Mat_<T> range(T start, T end, T inc) {
  cv::Mat_<T> ret(1, static_cast<int>((end - start) / inc) + 1);
  int idx = 0;
  while (start <= end) {
    ret(0, idx++) = start;
    start += inc;
  }
  return ret;
}

template <class FT>
inline void split(const cv::Mat& src, cv::Mat& R, cv::Mat& I) {
  cv::Mat planes[2];
  cv::split(src, planes);
  R = planes[0];
  I = planes[1];
}

template <class FT>
inline void splitT(const cv::Mat_<std::complex<FT>>& src, cv::Mat_<FT>& R, cv::Mat_<FT>& I) {
  split<FT>(src, R, I);
}

template <class FT>
inline cv::Mat_<std::complex<FT>> merge(const cv::Mat& R, const cv::Mat& I) {
  cv::Mat_<std::complex<FT>> ret;
  cv::Mat planes[] = {R, I};
  cv::merge(planes, 2, ret);
  return ret;
}

template <class FT>
inline cv::Mat_<std::complex<FT>> mergeT(const cv::Mat_<FT>& R, const cv::Mat_<FT>& I) {
  return merge<FT>(R, I);
}

template <class FT>
inline cv::Mat_<FT> real(const cv::Mat& src) {
  cv::Mat_<FT> planes[2];
  cv::split(src, planes);
  return planes[0];
}

template <class FT>
inline cv::Mat_<FT> real(const cv::Mat_<std::complex<FT>>& src) {
  return real<FT>(cv::Mat(src));
}

template <class FT>
inline cv::Mat_<FT> imag(const cv::Mat& src) {
  cv::Mat_<FT> planes[2];
  cv::split(src, planes);
  return planes[1];
}

template <class FT>
inline cv::Mat_<FT> imag(const cv::Mat_<std::complex<FT>>& src) {
  return imag<FT>(cv::Mat(src));
}

//! fast fourier transform -> autopads if not already padded
template <class FT>
cv::Mat_<std::complex<FT>> fft2(const cv::Mat& src, bool inverse = false) {
  cv::Mat data, ret;

  CV_Assert(src.channels() < 3);

  // need to convert to two channel
  if (src.channels() == 1) {
    if (src.type() != cv::DataType<FT>::type)
      src.convertTo(data, cv::DataType<FT>::type);
    else
      data = src;
    cv::Mat planes[] = {data, cv::Mat::zeros(data.size(), cv::DataType<FT>::type)};
    cv::merge(planes, 2, data);
  } else {
    if (src.type() != cv::DataType<std::complex<FT>>::type)
      src.convertTo(data, cv::DataType<std::complex<FT>>::type);
    else
      data = src;
  }

  // get optimal size for fft;
  int r = cv::getOptimalDFTSize(data.rows);
  int c = cv::getOptimalDFTSize(data.cols);

  // do we have to adapt size
  if (r != data.rows || c != data.cols) {
    // test if src is already padded
    cv::Size ws;
    cv::Point ofs;
    data.locateROI(ws, ofs);
    if (ofs.x == 0 && ofs.y == 0 && ws.width == c && ws.height == r)
      data.adjustROI(0, r - data.rows, 0, c - data.cols);
    else
      cv::copyMakeBorder(data, data, 0, r - src.rows, 0, c - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    cv::dft(data, ret, cv::DFT_COMPLEX_OUTPUT | (inverse ? cv::DFT_INVERSE | cv::DFT_SCALE : 0));
    ret.adjustROI(0, src.rows - r, 0, src.cols - c);

  } else
    cv::dft(data, ret, cv::DFT_COMPLEX_OUTPUT | (inverse ? cv::DFT_INVERSE | cv::DFT_SCALE : 0));

  return cv::Mat_<std::complex<FT>>(ret);
}

template <class FT>
inline cv::Mat_<std::complex<FT>> fft2(const cv::Mat_<FT>& src) {
  return fft2<FT>(cv::Mat(src));
}

template <class FT>
inline cv::Mat_<std::complex<FT>> fft2(const cv::Mat_<std::complex<FT>>& src) {
  return fft2<FT>(cv::Mat(src));
}

template <class FT>
inline void fft2(const cv::Mat& src, cv::Mat& R, cv::Mat& I) {
  split<FT>(fft2<FT>(src), R, I);
}

template <class FT>
inline void fft2T(const cv::Mat_<FT>& src, cv::Mat_<FT>& R, cv::Mat_<FT>& I) {
  split<FT>(fft2<FT>(src), R, I);
}

template <class FT>
inline void fft2T(const cv::Mat_<std::complex<FT>>& src, cv::Mat_<FT>& R, cv::Mat_<FT>& I) {
  split<FT>(fft2<FT>(src), R, I);
}

template <class FT>
inline cv::Mat_<std::complex<FT>> ifft2(const cv::Mat& src) {
  return fft2<FT>(src, true);
}

template <class FT>
inline cv::Mat_<std::complex<FT>> ifft2(const cv::Mat_<FT>& src) {
  return ifft2<FT>(cv::Mat(src));
}

template <class FT>
inline cv::Mat_<std::complex<FT>> ifft2(const cv::Mat_<std::complex<FT>>& src) {
  return ifft2<FT>(cv::Mat(src));
}

template <class FT>
inline void ifft2(const cv::Mat& src, cv::Mat& R, cv::Mat& I) {
  split<FT>(ifft2<FT>(src), R, I);
}

template <class FT>
inline void ifft2T(const cv::Mat_<FT>& src, cv::Mat_<FT>& R, cv::Mat_<FT>& I) {
  split<FT>(ifft2<FT>(src), R, I);
}

template <class FT>
inline void ifft2T(const cv::Mat_<std::complex<FT>>& src, cv::Mat_<FT>& R, cv::Mat_<FT>& I) {
  split<FT>(ifft2<FT>(src), R, I);
}


// include iota (in our own namespace) so it works with old and new compilers
template <class ForwardIterator, class T>
inline void iota(ForwardIterator first, ForwardIterator last, T value) {
  while (first != last) {
    *first++ = value;
    ++value;
  }
}

template <class T>
cv::Mat_<T> ifftshift(const cv::Mat& src) {
  cv::Mat_<T> tmp = src.clone();
  const int cx = src.cols / 2;
  const int cy = src.rows / 2;  // i.e., floor(x/2)
  const std::size_t cols = static_cast<std::size_t>(src.cols);
  const std::size_t rows = static_cast<std::size_t>(src.rows);
  std::vector<int> xvals(cols, 0), yvals(rows, 0);

  using diff_t = std::vector<int>::difference_type;
  const diff_t row_offset = static_cast<diff_t>(src.rows - cy);
  const diff_t col_offset = static_cast<diff_t>(src.cols - cx);
  lsfm::iota(yvals.begin(), yvals.begin() + row_offset, cy);
  lsfm::iota(yvals.begin() + row_offset, yvals.end(), 0);
  lsfm::iota(xvals.begin(), xvals.begin() + col_offset, cx);
  lsfm::iota(xvals.begin() + col_offset, xvals.end(), 0);

  for (int y = 0; y < src.rows; ++y) {
    const auto yi = static_cast<std::size_t>(y);
    for (int x = 0; x < src.cols; ++x) {
      const auto xi = static_cast<std::size_t>(x);
      tmp(y, x) = src.at<T>(yvals[yi], xvals[xi]);
    }
  }
  return tmp;
}

template <class T>
inline cv::Mat_<T> ifftshift(const cv::Mat_<T>& src) {
  return ifftshift<T>(cv::Mat(src));
}

template <class T>
inline void ifftshift(const cv::Mat& src, cv::Mat& dst) {
  dst = ifftshift<T>(src);
}

template <class T>
inline void ifftshiftT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  dst = ifftshift<T>(src);
}

template <class T>
cv::Mat_<T> fftshift(const cv::Mat& src) {
  cv::Mat_<T> tmp = src.clone();
  const int cx = (src.cols + 1) / 2;
  const int cy = (src.rows + 1) / 2;  // i.e., ceil(x/2)
  const std::size_t cols = static_cast<std::size_t>(src.cols);
  const std::size_t rows = static_cast<std::size_t>(src.rows);
  std::vector<int> xvals(cols), yvals(rows);

  using diff_t = std::vector<int>::difference_type;
  const diff_t row_offset = static_cast<diff_t>(src.rows - cy);
  const diff_t col_offset = static_cast<diff_t>(src.cols - cx);
  lsfm::iota(yvals.begin(), yvals.begin() + row_offset, cy);
  lsfm::iota(yvals.begin() + row_offset, yvals.end(), 0);
  lsfm::iota(xvals.begin(), xvals.begin() + col_offset, cx);
  lsfm::iota(xvals.begin() + col_offset, xvals.end(), 0);

  for (int y = 0; y < src.rows; ++y) {
    const auto yi = static_cast<std::size_t>(y);
    for (int x = 0; x < src.cols; ++x) {
      const auto xi = static_cast<std::size_t>(x);
      tmp(y, x) = src.at<T>(yvals[yi], xvals[xi]);
    }
  }
  return tmp;
}

template <class T>
inline cv::Mat_<T> fftshift(const cv::Mat_<T>& src) {
  return fftshift<T>(cv::Mat(src));
}

template <class T>
inline void fftshift(const cv::Mat& src, cv::Mat& dst) {
  dst = fftshift<T>(src);
}

template <class T>
inline void fftshiftT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  dst = fftshift<T>(src);
}

template <class FT>
void multiplyComplex(const cv::Mat& a, const cv::Mat& b, cv::Mat& dst) {
  cv::Size s = a.size();
  if (a.type() != dst.type() || a.rows != dst.rows || a.cols != dst.cols)
    dst.create(s, cv::DataType<std::complex<FT>>::type);

  if (a.isContinuous() && b.isContinuous() && dst.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  if (b.channels() == 2) {
    for (int i = 0; i < s.height; ++i) {
      const std::complex<FT>* pa = a.ptr<std::complex<FT>>(i);
      const std::complex<FT>* pb = b.ptr<std::complex<FT>>(i);
      std::complex<FT>* pdst = dst.ptr<std::complex<FT>>(i);
      for (int j = 0; j < s.width; ++j) pdst[j] = pa[j] * pb[j];
    }
  } else {
    // what is faster?
    // loop
    for (int i = 0; i < s.height; ++i) {
      const std::complex<FT>* pa = a.ptr<std::complex<FT>>(i);
      const FT* pb = b.ptr<FT>(i);
      std::complex<FT>* pdst = dst.ptr<std::complex<FT>>(i);
      for (int j = 0; j < s.width; ++j) pdst[j] = pa[j] * pb[j];
    }
    // or cv
    // cv::multiply(a, merge(b, b), dst);
  }
}

template <class FT>
void multiply(const cv::Mat_<std::complex<FT>>& a,
              const cv::Mat_<std::complex<FT>>& b,
              cv::Mat_<std::complex<FT>>& dst) {
  multiplyComplex<FT>(a, b, dst);
}

template <class FT>
cv::Mat_<std::complex<FT>> multiply(const cv::Mat_<std::complex<FT>>& a, const cv::Mat_<std::complex<FT>>& b) {
  cv::Mat_<std::complex<FT>> dst;
  multiplyComplex<FT>(a, b, dst);
  return dst;
}

template <class FT>
void multiply(const cv::Mat_<std::complex<FT>>& a, const cv::Mat_<FT>& b, cv::Mat_<std::complex<FT>>& dst) {
  multiplyComplex<FT>(a, b, dst);
}

template <class FT>
cv::Mat_<std::complex<FT>> multiply(const cv::Mat_<std::complex<FT>>& a, const cv::Mat_<FT>& b) {
  cv::Mat_<std::complex<FT>> dst;
  multiplyComplex<FT>(a, b, dst);
  return dst;
}

template <class FT>
void divideComplex(const cv::Mat& a, const cv::Mat& b, cv::Mat& dst) {
  cv::Size s = a.size();
  if (a.type() != dst.type() || a.rows != dst.rows || a.cols != dst.cols)
    dst.create(s, cv::DataType<std::complex<FT>>::type);

  if (a.isContinuous() && b.isContinuous() && dst.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  if (b.channels() == 2) {
    for (int i = 0; i < s.height; ++i) {
      const std::complex<FT>* pa = a.ptr<std::complex<FT>>(i);
      const std::complex<FT>* pb = b.ptr<std::complex<FT>>(i);
      std::complex<FT>* pdst = dst.ptr<std::complex<FT>>(i);
      for (int j = 0; j < s.width; ++j) pdst[j] = pa[j] / pb[j];
    }
  } else {
    // what is faster?
    // loop
    for (int i = 0; i < s.height; ++i) {
      const std::complex<FT>* pa = a.ptr<std::complex<FT>>(i);
      const FT* pb = b.ptr<FT>(i);
      std::complex<FT>* pdst = dst.ptr<std::complex<FT>>(i);
      for (int j = 0; j < s.width; ++j) pdst[j] = pa[j] / pb[j];
    }
    // or cv
    // cv::divide(a, merge(b, b), dst);
  }
}

template <class FT>
void divide(const cv::Mat_<std::complex<FT>>& a, const cv::Mat_<std::complex<FT>>& b, cv::Mat_<std::complex<FT>>& dst) {
  divideComplex<FT>(a, b, dst);
}

template <class FT>
cv::Mat_<std::complex<FT>> divide(const cv::Mat_<std::complex<FT>>& a, const cv::Mat_<std::complex<FT>>& b) {
  cv::Mat_<std::complex<FT>> dst;
  divideComplex<FT>(a, b, dst);
  return dst;
}

template <class FT>
void divide(const cv::Mat_<std::complex<FT>>& a, const cv::Mat_<FT>& b, cv::Mat_<std::complex<FT>>& dst) {
  divideComplex<FT>(a, b, dst);
}

template <class FT>
cv::Mat_<std::complex<FT>> divide(const cv::Mat_<std::complex<FT>>& a, const cv::Mat_<FT>& b) {
  cv::Mat_<std::complex<FT>> dst;
  divideComplex<FT>(a, b, dst);
  return dst;
}

template <class T, int N>
inline void sin(const cv::Vec<T, N>& src, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::sin(src[i]);
}

template <class T, int N>
inline cv::Vec<T, N> sin(const cv::Vec<T, N>& src) {
  cv::Vec<T, N> ret;
  sin(src, ret);
  return ret;
}

template <class T, int N>
inline void cos(const cv::Vec<T, N>& src, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::cos(src[i]);
}

template <class T, int N>
inline cv::Vec<T, N> cos(const cv::Vec<T, N>& src) {
  cv::Vec<T, N> ret;
  cos(src, ret);
  return ret;
}

template <class T, int N>
inline void tan(const cv::Vec<T, N>& src, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::tan(src[i]);
}

template <class T, int N>
inline cv::Vec<T, N> tan(const cv::Vec<T, N>& src) {
  cv::Vec<T, N> ret;
  tan(src, ret);
  return ret;
}

template <class T, int N>
inline void asin(const cv::Vec<T, N>& src, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::asin(src[i]);
}

template <class T, int N>
inline cv::Vec<T, N> asin(const cv::Vec<T, N>& src) {
  cv::Vec<T, N> ret;
  asin(src, ret);
  return ret;
}

template <class T, int N>
inline void acos(const cv::Vec<T, N>& src, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::acos(src[i]);
}

template <class T, int N>
inline cv::Vec<T, N> acos(const cv::Vec<T, N>& src) {
  cv::Vec<T, N> ret;
  acos(src, ret);
  return ret;
}

template <class T, int N>
inline void atan(const cv::Vec<T, N>& src, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::atan(src[i]);
}

template <class T, int N>
inline cv::Vec<T, N> atan(const cv::Vec<T, N>& src) {
  cv::Vec<T, N> ret;
  atan(src, ret);
  return ret;
}

template <class T, int N>
inline void atan2(const cv::Vec<T, N>& Y, const cv::Vec<T, N>& X, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::atan2(Y[i], X[i]);
}

template <class T, int N>
inline cv::Vec<T, N> atan2(const cv::Vec<T, N>& Y, const cv::Vec<T, N>& X) {
  cv::Vec<T, N> ret;
  atan2(Y, X, ret);
  return ret;
}

template <class T, int N>
inline void sinh(const cv::Vec<T, N>& src, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::sinh(src[i]);
}

template <class T, int N>
inline cv::Vec<T, N> sinh(const cv::Vec<T, N>& src) {
  cv::Vec<T, N> ret;
  sinh(src, ret);
  return ret;
}

template <class T, int N>
inline void cosh(const cv::Vec<T, N>& src, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::cosh(src[i]);
}

template <class T, int N>
inline cv::Vec<T, N> cosh(const cv::Vec<T, N>& src) {
  cv::Vec<T, N> ret;
  cosh(src, ret);
  return ret;
}

template <class T, int N>
inline void tanh(const cv::Vec<T, N>& src, cv::Vec<T, N>& dst) {
  for (int i = 0; i != N; ++i) dst[i] = std::tanh(src[i]);
}

template <class T, int N>
inline cv::Vec<T, N> tanh(const cv::Vec<T, N>& src) {
  cv::Vec<T, N> ret;
  tanh(src, ret);
  return ret;
}

template <class T>
void sin(const cv::Mat& src, cv::Mat& dst) {
  cv::Size s = src.size();
  dst.create(s, cv::DataType<T>::type);

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = sin(psrc[j]);
    }
  }
}

template <class T>
inline void sinT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  sin<T>(src, dst);
}


template <class T>
inline cv::Mat_<T> sin(const cv::Mat& src) {
  cv::Mat_<T> ret;
  sin<T>(src, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> sin(const cv::Mat_<T>& src) {
  return sin<T>(cv::Mat(src));
}

template <class T>
void cos(const cv::Mat& src, cv::Mat& dst) {
  cv::Size s = src.size();
  dst.create(s, cv::DataType<T>::type);

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = cos(psrc[j]);
    }
  }
}

template <class T>
inline void cosT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  cos<T>(src, dst);
}


template <class T>
inline cv::Mat_<T> cos(const cv::Mat& src) {
  cv::Mat_<T> ret;
  cos<T>(src, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> cos(const cv::Mat_<T>& src) {
  return cos<T>(cv::Mat(src));
}

template <class T>
void tan(const cv::Mat& src, cv::Mat& dst) {
  cv::Size s = src.size();
  dst.create(s, cv::DataType<T>::type);

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = tan(psrc[j]);
    }
  }
}

template <class T>
inline void tanT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  tan<T>(src, dst);
}


template <class T>
inline cv::Mat_<T> tan(const cv::Mat& src) {
  cv::Mat_<T> ret;
  tan<T>(src, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> tan(const cv::Mat_<T>& src) {
  return tan<T>(cv::Mat(src));
}

template <class T>
void asin(const cv::Mat& src, cv::Mat& dst) {
  cv::Size s = src.size();
  dst.create(s, cv::DataType<T>::type);

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = asin(psrc[j]);
    }
  }
}

template <class T>
inline void asinT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  asin<T>(src, dst);
}

template <class T>
inline cv::Mat_<T> asin(const cv::Mat& src) {
  cv::Mat_<T> ret;
  asin<T>(src, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> asin(const cv::Mat_<T>& src) {
  return asin<T>(cv::Mat(src));
}

template <class T>
void acos(const cv::Mat& src, cv::Mat& dst) {
  cv::Size s = src.size();
  dst.create(s, cv::DataType<T>::type);

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = acos(psrc[j]);
    }
  }
}

template <class T>
inline void acosT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  acos<T>(src, dst);
}

template <class T>
inline cv::Mat_<T> acos(const cv::Mat& src) {
  cv::Mat_<T> ret;
  acos<T>(src, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> acos(const cv::Mat_<T>& src) {
  return acos<T>(cv::Mat(src));
}

template <class T>
void atan(const cv::Mat& src, cv::Mat& dst) {
  cv::Size s = src.size();
  dst.create(s, cv::DataType<T>::type);

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = atan(psrc[j]);
    }
  }
}

template <class T>
inline void atanT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  atan<T>(src, dst);
}

template <class T>
inline cv::Mat_<T> atan(const cv::Mat& src) {
  cv::Mat_<T> ret;
  atan<T>(src, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> atan(const cv::Mat_<T>& src) {
  return atan<T>(cv::Mat(src));
}

template <class T>
void atan2(const cv::Mat& Y, const cv::Mat& X, cv::Mat& dst) {
  cv::Size s = Y.size();
  dst.create(s, cv::DataType<T>::type);

  if (Y.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* pY = Y.ptr<T>(i);
    const T* pX = X.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = atan2(pY[j], pX[j]);
    }
  }
}

template <class T>
inline void atan2T(const cv::Mat_<T>& Y, const cv::Mat_<T>& X, cv::Mat_<T>& dst) {
  atan2<T>(Y, X, dst);
}

template <class T>
inline cv::Mat_<T> atan2(const cv::Mat& Y, const cv::Mat& X) {
  cv::Mat_<T> ret;
  atan2<T>(Y, X, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> atan2(const cv::Mat_<T>& Y, const cv::Mat_<T>& X) {
  return atan2<T>(cv::Mat(Y), cv::Mat(X));
}

template <class T>
void sinh(const cv::Mat& src, cv::Mat& dst) {
  cv::Size s = src.size();
  dst.create(s, cv::DataType<T>::type);

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = sinh(psrc[j]);
    }
  }
}

template <class T>
inline void sinhT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  sinh<T>(src, dst);
}

template <class T>
inline cv::Mat_<T> sinh(const cv::Mat& src) {
  cv::Mat_<T> ret;
  sinh<T>(src, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> sinh(const cv::Mat_<T>& src) {
  return sinh<T>(cv::Mat(src));
}

template <class T>
void cosh(const cv::Mat& src, cv::Mat& dst) {
  cv::Size s = src.size();
  dst.create(s, cv::DataType<T>::type);

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = cosh(psrc[j]);
    }
  }
}

template <class T>
inline void coshT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  cosh<T>(src, dst);
}

template <class T>
inline cv::Mat_<T> cosh(const cv::Mat& src) {
  cv::Mat_<T> ret;
  cosh<T>(src, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> cosh(const cv::Mat_<T>& src) {
  return cosh<T>(cv::Mat(src));
}

template <class T>
void tanh(const cv::Mat& src, cv::Mat& dst) {
  cv::Size s = src.size();
  dst.create(s, cv::DataType<T>::type);

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    T* pdst = dst.ptr<T>(i);
    for (int j = 0; j < s.width; ++j) {
      pdst[j] = tanh(psrc[j]);
    }
  }
}

template <class T>
inline void tanhT(const cv::Mat_<T>& src, cv::Mat_<T>& dst) {
  tanh<T>(src, dst);
}

template <class T>
inline cv::Mat_<T> tanh(const cv::Mat& src) {
  cv::Mat_<T> ret;
  tanh<T>(src, ret);
  return ret;
}

template <class T>
inline cv::Mat_<T> tanh(const cv::Mat_<T>& src) {
  return tanh<T>(cv::Mat(src));
}

//! periodic fft -> see http://www.mi.parisdescartes.fr/~moisan/p+s -> requires one additional fft
template <class FT>
void perfft2(const cv::Mat& src, cv::Mat& P, cv::Mat& S) {
  cv::Mat data;
  if (src.type() != cv::DataType<FT>::type)
    src.convertTo(data, cv::DataType<FT>::type);
  else
    data = src;

  // get optimal size for fft;
  int r = cv::getOptimalDFTSize(data.rows);
  int c = cv::getOptimalDFTSize(data.cols);

  if (r != data.rows || c != data.cols) {
    // test if src is already padded
    cv::Size ws;
    cv::Point ofs;
    data.locateROI(ws, ofs);
    if (ofs.x == 0 && ofs.y == 0 && ws.width == c && ws.height == r)
      data.adjustROI(0, r - data.rows, 0, c - data.cols);
    else
      cv::copyMakeBorder(data, data, 0, r - src.rows, 0, c - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
  }

  cv::Mat_<FT> s(data.size(), FT{}), cx, cy, tmp;
  s.row(0) = data.row(0) - data.row(data.rows - 1);
  s.row(data.rows - 1) = -s.row(0);

  s.col(0) += data.col(0) - data.col(data.cols - 1);
  s.col(data.cols - 1) += data.col(data.cols - 1) - data.col(0);

  meshgrid((2 * CV_PI / data.cols) * range(static_cast<FT>(0), static_cast<FT>(data.cols - 1)),
           (2 * CV_PI / data.rows) * range(static_cast<FT>(0), static_cast<FT>(data.rows - 1)), cx, cy);

  tmp = 2 * (2 - cos(cx) - cos(cy));
  S = fft2(s);
  divideComplex<FT>(S, tmp, S);
  S.at<std::complex<FT>>(0, 0) = std::complex<FT>();
  P = fft2<FT>(data) - S;
}

template <class FT>
inline void perfft2T(const cv::Mat_<FT>& src, cv::Mat_<std::complex<FT>>& P, cv::Mat_<std::complex<FT>>& S) {
  perfft2<FT>(src, P, S);
}

template <class FT>
inline void perfft2(const cv::Mat& src, cv::Mat& P) {
  cv::Mat tmp;
  perfft2<FT>(src, P, tmp);
}

template <class FT>
inline void perfft2T(const cv::Mat_<FT>& src, cv::Mat_<std::complex<FT>>& P) {
  cv::Mat tmp;
  perfft2<FT>(src, P, tmp);
}

template <class FT>
inline cv::Mat_<std::complex<FT>> perfft2(const cv::Mat& src) {
  cv::Mat_<std::complex<FT>> ret;
  perfft2<FT>(src, ret);
  return ret;
}

template <class FT>
inline cv::Mat_<std::complex<FT>> perfft2(const cv::Mat_<FT>& src) {
  return perfft2<FT>(cv::Mat(src));
}

template <class T>
T median(const cv::Mat& src) {
  cv::Mat tmp;
  src.copyTo(tmp);

  int size = tmp.rows * tmp.cols;
  T* first = tmp.ptr<T>();
  T* last = first + size;
  T* middle = first + size / 2;
  std::nth_element(first, middle, last);
  return *middle;
}

template <class T>
inline T median(const cv::Mat_<T>& src) {
  return median<T>(cv::Mat(src));
}

template <class T>
cv::Mat_<int> hist(const cv::Mat& src, const cv::Mat& edges, bool accum = false) {
  int bins = edges.rows * edges.cols - 1;
  cv::Mat_<int> ret(1, bins, 0);
  cv::Size s = src.size();

  if (src.isContinuous()) {
    s.width *= s.height;
    s.height = 1;
  }

  int* pret = ret.ptr<int>();
  for (int i = 0; i < s.height; ++i) {
    const T* psrc = src.ptr<T>(i);
    const T* pedges = edges.ptr<T>();

    for (int j = 0; j < s.width; ++j) {
      for (int k = 0; k != bins; ++k) {
        if (psrc[j] > pedges[k] && psrc[j] <= pedges[k + 1]) ++pret[k];
      }
    }
  }

  if (accum) {
    for (int i = 1; i != bins; ++i) pret[i] += pret[i - 1];
  }

  return ret;
}

template <class T>
inline cv::Mat_<int> histT(const cv::Mat_<T>& src, const cv::Mat_<T>& edges, bool accum = false) {
  return hist<T>(src, edges, accum);
}

template <class T>
inline cv::Mat_<int> hist(const cv::Mat& src, int nbins = 50, bool accum = false) {
  double vmin, vmax;
  cv::minMaxIdx(src, &vmin, &vmax);
  return hist<T>(src, range<T>(0, vmax / nbins, vmax), accum);
}

template <class T>
inline cv::Mat_<int> hist(const cv::Mat_<T>& src, int bins = 50, bool accum = false) {
  return hist<T>(cv::Mat(src), bins, accum);
}

template <class T>
T rayleighMode(const cv::Mat& src, int nbins = 50) {
  double vmin, vmax;
  cv::minMaxIdx(src, &vmin, &vmax);
  cv::Mat_<T> edges = range<T>(0, static_cast<T>(vmax), static_cast<T>(vmax / nbins));
  cv::Mat_<int> h = hist<T>(src, edges);
  cv::Point midx;
  cv::minMaxLoc(h, 0, 0, 0, &midx);
  return (edges(midx) + edges(0, midx.x + 1)) / 2;
}

template <class T>
inline T rayleighMode(const cv::Mat_<T>& src, int nbins = 50) {
  return rayleighMode<T>(cv::Mat(src), nbins);
}


template <class FT>
void filterGrid(int rows, int cols, cv::Mat& radius, cv::Mat& u1, cv::Mat& u2, bool shift = true) {
  meshgrid(
      (cols % 2 ? range<FT>(-static_cast<FT>(cols - 1) / 2, static_cast<FT>(cols - 1) / 2) / static_cast<FT>(cols - 1)
                : range<FT>(static_cast<FT>(-cols) / 2, static_cast<FT>(cols) / 2 - 1) / static_cast<FT>(cols)),
      (rows % 2 ? range<FT>(-static_cast<FT>(rows - 1) / 2, static_cast<FT>(rows - 1) / 2) / static_cast<FT>(rows - 1)
                : range<FT>(static_cast<FT>(-rows) / 2, static_cast<FT>(rows) / 2 - 1) / static_cast<FT>(rows)),
      u1, u2);
  if (shift) {
    u1 = ifftshift<FT>(u1);
    u2 = ifftshift<FT>(u2);
  }
  cv::sqrt(u1.mul(u1) + u2.mul(u2), radius);
  // std::cout << u1 << std::endl << u2 << std::endl << radius << std::endl;
}

template <class FT>
inline void filtergridT(
    int rows, int cols, cv::Mat_<FT>& radius, cv::Mat_<FT>& u1, cv::Mat_<FT>& u2, bool shift = true) {
  filterGrid<FT>(rows, cols, radius, u1, u2, shift);
}

template <class FT>
inline cv::Mat_<FT> lowpassFilter(int rows, int cols, FT cutoff, int n) {
  cv::Mat x, y, r;
  filterGrid<FT>(rows, cols, r, x, y, false);
  cv::pow(r / cutoff, 2 * n, x);
  x += 1;
  // std::cout << "lowpass:" << std::endl << ifftshift<FT>(cv::Mat(static_cast<FT>(1) / x)) << std::endl;
  return ifftshift<FT>(cv::Mat(static_cast<FT>(1) / x));
}

}  // namespace lsfm
