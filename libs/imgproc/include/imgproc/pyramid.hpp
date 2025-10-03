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


#ifndef _PYRAMID_H
#define _PYRAMID_H


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

namespace lsfm {

template <class MT>
class Pyramid {
  std::vector<cv::Mat> scales_;

  void create(const cv::Mat data, int scale_num) {
    scales_.push_back(data);
    cv::Mat sc = data, level;
    bool convert = false;
    if (type() != CV_32F || type() != CV_64F) {
      data.convertTo(sc, CV_32F);
      convert = true;
    }

    if (scale_num == 0) {
      while (scales_.back().size().area() >= 2) {
        cv::pyrDown(sc, level);
        if (convert) {
          level.convertTo(sc, type());
          scales_.push_back(sc);
        } else
          scales_.push_back(level);
        sc = level;
      }
    } else if (scale_num > 0) {
      while (scales_.back().size().area() >= 2 && scales_.size() < static_cast<size_t>(scale_num)) {
        cv::pyrDown(sc, level);
        if (convert) {
          level.convertTo(sc, type());
          scales_.push_back(sc);
        } else
          scales_.push_back(level);
        sc = level;
      }
    } else {
      int min_cols = -scale_num;
      while (scales_.back().cols / 2 >= min_cols) {
        cv::pyrDown(sc, level);
        if (convert) {
          level.convertTo(sc, type());
          scales_.push_back(sc);
        } else
          scales_.push_back(level);
        sc = level;
      }
    }
  }

 public:
  typedef MT mat_type;

  Pyramid() {}

  Pyramid(int rows, int cols, int scale_num = 0) { create(cv::Mat(rows, cols, type()), scale_num); }

  Pyramid(const cv::Size& size, int scale_num = 0) { create(cv::Mat(size, type()), scale_num); }

  Pyramid(int rows, int cols, cv::Scalar& fill, int scale_num = 0) {
    create(cv::Mat(rows, cols, type(), fill), scale_num);
  }

  Pyramid(const cv::Size& size, cv::Scalar& fill, int scale_num = 0) { create(cv::Mat(size, type(), fill), scale_num); }

  Pyramid(const cv::Mat& data, int scale_num = 0) {
    cv::Mat tmp = data;
    if (type() != data.type()) data.convertTo(tmp, type());
    create(data, scale_num);
  }

  Pyramid(const Pyramid& p) {
    for (int i = 0; i < p.size(); ++i) scales_.push_back(p[i].clone());
  }

  template <class C>
  Pyramid(const Pyramid<C>& p) {
    cv::Mat tmp;
    for (int i = 0; i < p.size(); ++i) {
      p[i].convertTo(tmp, type());
      scales_.push_back(tmp);
    }
  }

  ~Pyramid() {}

  Pyramid clone() const {
    Pyramid pyramid;
    for (int i = 0; i < scales_.size(); ++i) pyramid.scales_.push_back(scales_[i].clone());
    return pyramid;
  }

  template <class C>
  Pyramid<C> convert() const {
    if (cv::DataType<C>::type == type()) return clone();

    Pyramid<C> pyramid;
    cv::Mat tmp;
    for (int i = 0; i < scales_.size(); ++i) {
      scales_[i].convertTo(tmp, cv::DataType<C>::type);
      pyramid.scales_.push_back(tmp);
    }
    return pyramid;
  }

  static inline int type() { return cv::DataType<MT>::type; }

  inline const cv::Mat& operator[](int i) const { return scales_[i]; }

  inline cv::Mat& operator[](int i) { return scales_[i]; }

  inline size_t size() const { return scales_.size(); }

  inline const std::vector<cv::Mat>& scales() const { return scales_; }

  inline std::vector<cv::Mat>& scales() { return scales_; }

  inline void resize(size_t size) { scales_.resize(size); }

  inline Pyramid& mul(const Pyramid& rhs) {
    for (int i = 0; i < scales_.size(); ++i) scales_[i] = scales_[i].mul(rhs);
    return *this;
  }

  inline Pyramid& div(const Pyramid& rhs) {
    for (int i = 0; i < scales_.size(); ++i) cv::divide(scales_[i], rhs, scales_[i]);
    return *this;
  }

  inline Pyramid& operator*=(double value) {
    for (int i = 0; i < scales_.size(); ++i) scales_[i] *= value;
    return *this;
  }

  inline Pyramid& operator+=(double value) {
    for (int i = 0; i < scales_.size(); ++i) scales_[i] += value;
    return *this;
  }


  inline Pyramid& operator+=(const Pyramid& rhs) {
    for (int i = 0; i < scales_.size(); ++i) scales_[i] += rhs;
    return *this;
  }

  inline Pyramid& operator-=(double value) {
    for (int i = 0; i < scales_.size(); ++i) scales_[i] -= value;
    return *this;
  }

  inline Pyramid& operator-=(const Pyramid& rhs) {
    for (int i = 0; i < scales_.size(); ++i) scales_[i] += rhs;
    return *this;
  }
};

template <class MT>
inline Pyramid<MT> operator*(double value, const Pyramid<MT>& pyramid) {
  Pyramid<MT> tmp = pyramid.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] = (value * tmp[i]);
  return tmp;
}

template <class MT>
inline Pyramid<MT> operator*(const Pyramid<MT>& pyramid, double value) {
  Pyramid<MT> tmp = pyramid.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] = (value * tmp[i]);
  return tmp;
}

template <class MT>
inline Pyramid<MT> operator/(double value, const Pyramid<MT>& pyramid) {
  Pyramid<MT> tmp = pyramid.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] = (value / tmp[i]);
  return tmp;
}

template <class MT>
inline Pyramid<MT> operator/(const Pyramid<MT>& pyramid, double value) {
  Pyramid<MT> tmp = pyramid.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] = (value / tmp[i]);
  return tmp;
}

template <class MT>
inline Pyramid<MT> operator+(double value, const Pyramid<MT>& pyramid) {
  Pyramid<MT> tmp = pyramid.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] = (value + tmp[i]);
  return tmp;
}

template <class MT>
inline Pyramid<MT> operator+(const Pyramid<MT>& pyramid, double value) {
  Pyramid<MT> tmp = pyramid.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] = (value + tmp[i]);
  return tmp;
}

template <class MT>
inline Pyramid<MT> operator+(const Pyramid<MT>& a, const Pyramid<MT>& b) {
  Pyramid<MT> tmp = a.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] += b[i];
  return tmp;
}

template <class MT>
inline Pyramid<MT> operator-(double value, const Pyramid<MT>& pyramid) {
  Pyramid<MT> tmp = pyramid.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] = (value - tmp[i]);
  return tmp;
}

template <class MT>
inline Pyramid<MT> operator-(const Pyramid<MT>& pyramid, double value) {
  Pyramid<MT> tmp = pyramid.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] = (value - tmp[i]);
  return tmp;
}


template <class MT>
inline Pyramid<MT> operator-(const Pyramid<MT>& a, const Pyramid<MT>& b) {
  Pyramid<MT> tmp = a.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] -= b[i];
  return tmp;
}

template <class MT>
inline Pyramid<MT> abs(const Pyramid<MT>& a) {
  Pyramid<MT> tmp = a.clone();
  for (int i = 0; i < tmp.size(); ++i) tmp[i] = cv::abs(tmp[i]);
  return tmp;
}

template <class MT>
inline Pyramid<MT> mul(const Pyramid<MT>& a, const Pyramid<MT>& b) {
  Pyramid<MT> tmp = a.clone();
  return tmp.mul(b);
}

template <class MT>
inline Pyramid<MT> div(const Pyramid<MT>& a, const Pyramid<MT>& b) {
  Pyramid<MT> tmp = a.clone();
  return tmp.div(b);
}

template <class MT>
cv::Mat draw(const Pyramid<MT>& pyramid) {
  cv::Point tl(0, 0);
  cv::Mat canvas(pyramid[0].rows, pyramid[0].cols + ((pyramid[0].cols + 2) >> 1), pyramid.type(), cv::Scalar::all(0));
  for (int i = 0; i < pyramid.size() - 1; ++i) {
    pyramid[i].copyTo(canvas(cv::Rect(tl, pyramid[i].size())));
    if (!(i % 2)) {
      tl.x += pyramid[i].cols;
    } else {
      tl.y += pyramid[i].rows;
    }
  }
  return canvas;
}

}  // namespace lsfm


#endif
