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

#ifndef _GRADIENT_HPP_
#define _GRADIENT_HPP_
#ifdef __cplusplus

#  include "derivative.hpp"
#  include "direction.hpp"
#  include "filter.hpp"
#  include "magnitude.hpp"


namespace lsfm {

//! Gradient interface class
//! Use IT to define Image Type (8Bit, 16Bit, 32Bit, or floating type float or double)
//! Use GT to define directional Derivative or Separable orientation Type (short, float or double)
//! Use MT to define Magnitude Type (int, float or double)
//! Use DT to define Direction Type (float or double)
//! For 8 Bit images use GT = short, for 16 Bit images float.
//! For images with floating values, use IT and GT = image floating type (float or double)
template <class IT, class GT, class MT, class DT>
class GradientI : public FilterI<IT> {
  GradientI(const GradientI&);

 protected:
  GradientI() {}

 public:
  typedef IT img_type;
  typedef MT mag_type;
  typedef GT grad_type;
  typedef DT dir_type;
  typedef Range<IT> IntensityRange;
  typedef Range<GT> GradientRange;
  typedef Range<MT> MagnitudeRange;
  typedef Range<DT> DirectionRange;

  virtual ~GradientI() {}

  //! get magnitude
  virtual cv::Mat magnitude() const = 0;

  //! get magnitude range for intensity range
  virtual MagnitudeRange magnitudeRange() const = 0;

  //! convert threshold between 0-1 to magnitude threshold
  virtual MT magnitudeThreshold(double val) const { return static_cast<MT>(magnitudeRange().upper * val); }

  //! get information about magnitude norm type
  //! default: NONE -> norm does not correspond gx, gy
  virtual NormType normType() const { return NormType::NONE; }

  //! get direction
  virtual cv::Mat direction() const = 0;

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  virtual DirectionRange directionRange() const = 0;

  //! get x,y derivatives or directional data
  virtual void directionals(cv::Mat& gx, cv::Mat& gy) const {
    switch (normType()) {
      case NormType::NORM_L2:
        Polar<GT, DT>::polar2Cart(magnitude(), direction(), gx, gy);
        break;
      case NormType::NORM_L2SQR: {
        cv::Mat tmp;
        cv::sqrt(magnitude(), tmp);
        Polar<GT, DT>::polar2Cart(tmp, direction(), gx, gy);
      } break;
      default:
        if (cv::DataType<GT>::type == CV_32F || cv::DataType<GT>::type == CV_64F)
          Polar<GT, DT>::polar2Cart(direction(), gx, gy);
        else
          Polar<GT, DT>::polar2Cart(direction(), 127, gx, gy);
        break;
    }
  }

  //! get x derivative
  virtual cv::Mat gx() const {
    cv::Mat gx, gy;
    directionals(gx, gy);
    return gx;
  }

  //! get y derivative
  virtual cv::Mat gy() const {
    cv::Mat gx, gy;
    directionals(gx, gy);
    return gy;
  }

  //! get gradient range
  virtual GradientRange gradientRange() const {
    GT val = 0;
    switch (normType()) {
      case NormType::NORM_L2:
        val = static_cast<GT>(magnitudeRange().upper * 0.707106781);
        break;
      case NormType::NORM_L2SQR: {
        val = static_cast<GT>(std::sqrt(static_cast<DT>(magnitudeRange().upper)) * 0.707106781);
      } break;
      default:
        if (cv::DataType<GT>::type == CV_32F || cv::DataType<GT>::type == CV_64F)
          val = 1;
        else
          val = 127;
        break;
    }
    return GradientRange(-val, val);
  }
};

//! Gradient base class
//! Use IT to define Image Type (8Bit, 16Bit, 32Bit, or floating type float or double)
//! Use GT to define directional Derivative or Separable orientation Type (short, float or double)
//! Use MT to define Magnitude Type (int, float or double)
//! Use DT to define Direction Type (float or double)
//! For 8 Bit images use GT = short, for 16 Bit images float.
//! For images with floating values, use IT and GT = image floating type (float or double)
template <class IT, class GT, class MT, class DT>
class Gradient : public GradientI<IT, GT, MT, DT> {
  Gradient();
  Gradient(const Gradient&);

 protected:
  Range<IT> intRange_;

  Gradient(IT int_lower, IT int_upper) : intRange_(int_lower, int_upper) {
    if (intRange_.lower > intRange_.upper) intRange_.swap();
  }

 public:
  typedef IT img_type;
  typedef MT mag_type;
  typedef GT grad_type;
  typedef DT dir_type;
  typedef Range<IT> IntensityRange;
  typedef Range<GT> GradientRange;
  typedef Range<MT> MagnitudeRange;
  typedef Range<DT> DirectionRange;


  //! get image intensity range (for single channel)
  IntensityRange intensityRange() const { return intRange_; }

  //! generic interface to get processed data
  virtual FilterResults results() const {
    FilterResults ret;
    ret["gx"] = FilterData(this->gx(), this->gradientRange());
    ret["gy"] = FilterData(this->gy(), this->gradientRange());
    ret["mag"] = FilterData(this->magnitude(), this->magnitudeRange());
    ret["dir"] = FilterData(this->direction(), this->directionRange());
    return ret;
  }
};

}  // namespace lsfm
#endif
#endif
