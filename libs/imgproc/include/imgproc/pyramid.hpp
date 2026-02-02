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
#include <opencv2/imgproc.hpp>

#include <vector>

namespace lsfm {

/// @brief Gaussian image pyramid for multi-scale analysis.
///
/// Creates and manages a sequence of progressively downsampled images using
/// Gaussian filtering (cv::pyrDown). Used for scale-space analysis, feature
/// detection at multiple scales, and coarse-to-fine processing.
///
/// @tparam MT Matrix element type (uchar, float, double, etc.)
template <class MT>
class Pyramid {
  std::vector<cv::Mat> scales_;  ///< Vector of pyramid levels

  /// @brief Create pyramid levels from base image.
  /// @param data Base image (level 0)
  /// @param scale_num Number of scales (0=auto, >0=exact count, <0=min width)
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
  typedef MT mat_type;  ///< Matrix element type

  /// @brief Default constructor.
  Pyramid() : scales_() {}

  /// @brief Construct pyramid from dimensions.
  /// @param rows Image height
  /// @param cols Image width
  /// @param scale_num Number of scales (0=auto, >0=exact, <0=min width)
  Pyramid(int rows, int cols, int scale_num = 0) : scales_() { create(cv::Mat(rows, cols, type()), scale_num); }

  /// @brief Construct pyramid from size.
  /// @param size Image size
  /// @param scale_num Number of scales
  Pyramid(const cv::Size& size, int scale_num = 0) : scales_() { create(cv::Mat(size, type()), scale_num); }

  /// @brief Construct pyramid with fill value.
  /// @param rows Image height
  /// @param cols Image width
  /// @param fill Initial fill value
  /// @param scale_num Number of scales
  Pyramid(int rows, int cols, cv::Scalar& fill, int scale_num = 0) : scales_() {
    create(cv::Mat(rows, cols, type(), fill), scale_num);
  }

  /// @brief Construct pyramid with fill value from size.
  /// @param size Image size
  /// @param fill Initial fill value
  /// @param scale_num Number of scales
  Pyramid(const cv::Size& size, cv::Scalar& fill, int scale_num = 0) : scales_() {
    create(cv::Mat(size, type(), fill), scale_num);
  }

  /// @brief Construct pyramid from existing image.
  /// @param data Source image (becomes level 0)
  /// @param scale_num Number of scales
  Pyramid(const cv::Mat& data, int scale_num = 0) : scales_() {
    cv::Mat tmp = data;
    if (type() != data.type()) data.convertTo(tmp, type());
    create(data, scale_num);
  }

  /// @brief Copy constructor.
  /// @param p Source pyramid to copy
  Pyramid(const Pyramid& p) : scales_() {
    for (std::size_t i = 0; i < p.size(); ++i) scales_.push_back(p[static_cast<int>(i)].clone());
  }

  /// @brief Converting copy constructor from different element type.
  /// @tparam C Source element type
  /// @param p Source pyramid
  template <class C>
  Pyramid(const Pyramid<C>& p) {
    cv::Mat tmp;
    for (std::size_t i = 0; i < p.size(); ++i) {
      p[i].convertTo(tmp, type());
      scales_.push_back(tmp);
    }
  }

  ~Pyramid() {}

  /// @brief Create deep copy of pyramid.
  /// @return New pyramid with cloned data
  Pyramid clone() const {
    Pyramid pyramid;
    for (std::size_t i = 0; i < scales_.size(); ++i) pyramid.scales_.push_back(scales_[i].clone());
    return pyramid;
  }

  /// @brief Convert pyramid to different element type.
  /// @tparam C Target element type
  /// @return New pyramid with converted data
  template <class C>
  Pyramid<C> convert() const {
    if (cv::DataType<C>::type == type()) return clone();

    Pyramid<C> pyramid;
    cv::Mat tmp;
    for (std::size_t i = 0; i < scales_.size(); ++i) {
      scales_[i].convertTo(tmp, cv::DataType<C>::type);
      pyramid.scales_.push_back(tmp);
    }
    return pyramid;
  }

  /// @brief Get OpenCV type code.
  /// @return CV_* type constant
  static inline int type() { return cv::DataType<MT>::type; }

  /// @brief Access pyramid level (const).
  /// @param i Level index (0 = base)
  /// @return Matrix at level i
  inline const cv::Mat& operator[](int i) const { return scales_[static_cast<size_t>(i)]; }

  /// @brief Access pyramid level.
  /// @param i Level index (0 = base)
  /// @return Matrix at level i
  inline cv::Mat& operator[](int i) { return scales_[static_cast<size_t>(i)]; }

  /// @brief Get number of pyramid levels.
  /// @return Number of scales
  inline size_t size() const { return scales_.size(); }

  /// @brief Get all scales (const).
  /// @return Reference to scales vector
  inline const std::vector<cv::Mat>& scales() const { return scales_; }

  /// @brief Get all scales.
  /// @return Reference to scales vector
  inline std::vector<cv::Mat>& scales() { return scales_; }

  /// @brief Resize number of levels.
  /// @param size New number of levels
  inline void resize(size_t size) { scales_.resize(size); }

  /// @brief Element-wise multiply with another pyramid.
  /// @param rhs Right-hand side pyramid
  /// @return Reference to this
  inline Pyramid& mul(const Pyramid& rhs) {
    for (std::size_t i = 0; i < scales_.size(); ++i) scales_[i] = scales_[i].mul(rhs);
    return *this;
  }

  /// @brief Element-wise divide by another pyramid.
  /// @param rhs Right-hand side pyramid
  /// @return Reference to this
  inline Pyramid& div(const Pyramid& rhs) {
    for (std::size_t i = 0; i < scales_.size(); ++i) cv::divide(scales_[i], rhs, scales_[i]);
    return *this;
  }

  /// @brief Multiply all levels by scalar.
  /// @param value Scalar multiplier
  /// @return Reference to this
  inline Pyramid& operator*=(double value) {
    for (std::size_t i = 0; i < scales_.size(); ++i) scales_[i] *= value;
    return *this;
  }

  /// @brief Add scalar to all levels.
  /// @param value Scalar to add
  /// @return Reference to this
  inline Pyramid& operator+=(double value) {
    for (std::size_t i = 0; i < scales_.size(); ++i) scales_[i] += value;
    return *this;
  }

  /// @brief Add another pyramid element-wise.
  /// @param rhs Right-hand side pyramid
  /// @return Reference to this
  inline Pyramid& operator+=(const Pyramid& rhs) {
    for (std::size_t i = 0; i < scales_.size(); ++i) scales_[i] += rhs;
    return *this;
  }

  /// @brief Subtract scalar from all levels.
  /// @param value Scalar to subtract
  /// @return Reference to this
  inline Pyramid& operator-=(double value) {
    for (std::size_t i = 0; i < scales_.size(); ++i) scales_[i] -= value;
    return *this;
  }

  /// @brief Subtract another pyramid element-wise.
  /// @param rhs Right-hand side pyramid
  /// @return Reference to this
  inline Pyramid& operator-=(const Pyramid& rhs) {
    for (std::size_t i = 0; i < scales_.size(); ++i) scales_[i] += rhs;
    return *this;
  }
};

/// @brief Multiply scalar by pyramid.
/// @tparam MT Matrix element type
/// @param value Scalar multiplier
/// @param pyramid Source pyramid
/// @return New pyramid with scaled values
template <class MT>
inline Pyramid<MT> operator*(double value, const Pyramid<MT>& pyramid) {
  Pyramid<MT> tmp = pyramid.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[static_cast<int>(i)] = (value * tmp[static_cast<int>(i)]);
  return tmp;
}

/// @brief Multiply pyramid by scalar.
/// @tparam MT Matrix element type
/// @param pyramid Source pyramid
/// @param value Scalar multiplier
/// @return New pyramid with scaled values
template <class MT>
inline Pyramid<MT> operator*(const Pyramid<MT>& pyramid, double value) {
  Pyramid<MT> tmp = pyramid.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[static_cast<int>(i)] = (value * tmp[static_cast<int>(i)]);
  return tmp;
}

/// @brief Divide scalar by pyramid element-wise.
/// @tparam MT Matrix element type
/// @param value Numerator scalar
/// @param pyramid Denominator pyramid
/// @return New pyramid
template <class MT>
inline Pyramid<MT> operator/(double value, const Pyramid<MT>& pyramid) {
  Pyramid<MT> tmp = pyramid.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[i] = (value / tmp[i]);
  return tmp;
}

/// @brief Divide pyramid by scalar.
/// @tparam MT Matrix element type
/// @param pyramid Numerator pyramid
/// @param value Denominator scalar
/// @return New pyramid
template <class MT>
inline Pyramid<MT> operator/(const Pyramid<MT>& pyramid, double value) {
  Pyramid<MT> tmp = pyramid.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[i] = (value / tmp[i]);
  return tmp;
}

/// @brief Add scalar to pyramid.
/// @tparam MT Matrix element type
/// @param value Scalar to add
/// @param pyramid Source pyramid
/// @return New pyramid
template <class MT>
inline Pyramid<MT> operator+(double value, const Pyramid<MT>& pyramid) {
  Pyramid<MT> tmp = pyramid.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[i] = (value + tmp[i]);
  return tmp;
}

/// @brief Add scalar to pyramid.
/// @tparam MT Matrix element type
/// @param pyramid Source pyramid
/// @param value Scalar to add
/// @return New pyramid
template <class MT>
inline Pyramid<MT> operator+(const Pyramid<MT>& pyramid, double value) {
  Pyramid<MT> tmp = pyramid.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[i] = (value + tmp[i]);
  return tmp;
}

/// @brief Add two pyramids element-wise.
/// @tparam MT Matrix element type
/// @param a First pyramid
/// @param b Second pyramid
/// @return New pyramid with sum
template <class MT>
inline Pyramid<MT> operator+(const Pyramid<MT>& a, const Pyramid<MT>& b) {
  Pyramid<MT> tmp = a.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[static_cast<int>(i)] += b[static_cast<int>(i)];
  return tmp;
}

/// @brief Subtract pyramid from scalar.
/// @tparam MT Matrix element type
/// @param value Scalar minuend
/// @param pyramid Subtrahend pyramid
/// @return New pyramid
template <class MT>
inline Pyramid<MT> operator-(double value, const Pyramid<MT>& pyramid) {
  Pyramid<MT> tmp = pyramid.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[i] = (value - tmp[i]);
  return tmp;
}

/// @brief Subtract scalar from pyramid.
/// @tparam MT Matrix element type
/// @param pyramid Minuend pyramid
/// @param value Scalar subtrahend
/// @return New pyramid
template <class MT>
inline Pyramid<MT> operator-(const Pyramid<MT>& pyramid, double value) {
  Pyramid<MT> tmp = pyramid.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[i] = (value - tmp[i]);
  return tmp;
}

/// @brief Subtract two pyramids element-wise.
/// @tparam MT Matrix element type
/// @param a First pyramid (minuend)
/// @param b Second pyramid (subtrahend)
/// @return New pyramid with difference
template <class MT>
inline Pyramid<MT> operator-(const Pyramid<MT>& a, const Pyramid<MT>& b) {
  Pyramid<MT> tmp = a.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[i] -= b[i];
  return tmp;
}

/// @brief Compute absolute value of pyramid.
/// @tparam MT Matrix element type
/// @param a Source pyramid
/// @return New pyramid with absolute values
template <class MT>
inline Pyramid<MT> abs(const Pyramid<MT>& a) {
  Pyramid<MT> tmp = a.clone();
  for (std::size_t i = 0; i < tmp.size(); ++i) tmp[i] = cv::abs(tmp[i]);
  return tmp;
}

/// @brief Element-wise multiply two pyramids.
/// @tparam MT Matrix element type
/// @param a First pyramid
/// @param b Second pyramid
/// @return New pyramid with product
template <class MT>
inline Pyramid<MT> mul(const Pyramid<MT>& a, const Pyramid<MT>& b) {
  Pyramid<MT> tmp = a.clone();
  return tmp.mul(b);
}

/// @brief Element-wise divide two pyramids.
/// @tparam MT Matrix element type
/// @param a Numerator pyramid
/// @param b Denominator pyramid
/// @return New pyramid with quotient
template <class MT>
inline Pyramid<MT> div(const Pyramid<MT>& a, const Pyramid<MT>& b) {
  Pyramid<MT> tmp = a.clone();
  return tmp.div(b);
}

/// @brief Visualize pyramid as single image.
///
/// Draws all pyramid levels arranged in a single canvas for visualization.
///
/// @tparam MT Matrix element type
/// @param pyramid Pyramid to visualize
/// @return Canvas image showing all levels
template <class MT>
cv::Mat draw(const Pyramid<MT>& pyramid) {
  cv::Point tl(0, 0);
  cv::Mat canvas(pyramid[0].rows, pyramid[0].cols + ((pyramid[0].cols + 2) >> 1), pyramid.type(), cv::Scalar::all(0));
  for (std::size_t i = 0; i < pyramid.size() - 1; ++i) {
    pyramid[static_cast<int>(i)].copyTo(canvas(cv::Rect(tl, pyramid[static_cast<int>(i)].size())));
    if (!(i % 2)) {
      tl.x += static_cast<int>(pyramid[static_cast<int>(i)].cols);
    } else {
      tl.y += static_cast<int>(pyramid[static_cast<int>(i)].rows);
    }
  }
  return canvas;
}

}  // namespace lsfm
