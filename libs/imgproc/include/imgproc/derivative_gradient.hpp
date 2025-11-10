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

#include <imgproc/gradient.hpp>

namespace lsfm {

//! Derivative Gradient
//! Use IT to define Image Type (8Bit, 16Bit, 32Bit, or floating type float or double)
//! Use GT to define Derivative/Gradient Type (short, float or double)
//! Use MT to define Magnitude Type (int, float or double)
//! Use DT to define Direction Type (float or double)
//! For 8 Bit images use GT = short, for 16 Bit images float.
//! For images with floating values, use IT and GT = image floating type (float or double)
template <class IT = uchar,
          class GT = short,
          class MT = int,
          class DT = float,
          template <class, class> class GO = SobelDerivative,
          template <class, class> class MO = Magnitude,
          template <class, class> class DO = Direction>
class DerivativeGradient : public Gradient<IT, GT, MT, DT> {
  GO<IT, GT> derivative_;

  mutable cv::Mat_<GT> gx_, gy_;
  mutable cv::Mat_<MT> mag_;
  mutable cv::Mat_<DT> dir_;

  mutable bool mag_done_, dir_done_;

 public:
  typedef IT img_type;
  typedef GT grad_type;
  typedef MT mag_type;
  typedef DT dir_type;
  typedef Range<IT> IntensityRange;
  typedef Range<GT> GradientRange;
  typedef Range<MT> MagnitudeRange;
  typedef Range<DT> DirectionRange;

  DerivativeGradient(IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
      : Gradient<IT, GT, MT, DT>(int_lower, int_upper),
        derivative_(),
        gx_(),
        gy_(),
        mag_(),
        dir_(),
        mag_done_(false),
        dir_done_(false) {
    this->addManager(derivative_);
  }

  DerivativeGradient(const ValueManager::NameValueVector& options,
                     IT int_lower = std::numeric_limits<IT>::lowest(),
                     IT int_upper = std::numeric_limits<IT>::max())
      : Gradient<IT, GT, MT, DT>(int_lower, int_upper),
        derivative_(),
        gx_(),
        gy_(),
        mag_(),
        dir_(),
        mag_done_(false),
        dir_done_(false) {
    this->addManager(derivative_);
    this->value(options);
  }

  DerivativeGradient(ValueManager::InitializerList options,
                     IT int_lower = std::numeric_limits<IT>::lowest(),
                     IT int_upper = std::numeric_limits<IT>::max())
      : Gradient<IT, GT, MT, DT>(int_lower, int_upper),
        derivative_(),
        gx_(),
        gy_(),
        mag_(),
        dir_(),
        mag_done_(false),
        dir_done_(false) {
    this->addManager(derivative_);
    this->value(options);
  }

  //! process gradient
  void process(const cv::Mat& img) {
    mag_done_ = false;
    dir_done_ = false;

    derivative_.process(img, this->gx_, this->gy_);
  }

  //! process gradient and get results
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) {
    process(img);
    directionals(gx, gy);
  }

  //! process gradient and get results
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
  }

  //! process gradient and get results
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag, cv::Mat& dir) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
    dir = direction();
  }

  //! get x,y derivatives
  void directionals(cv::Mat& gx, cv::Mat& gy) const {
    gx = gx_;
    gy = gy_;
  }

  //! get x derivative
  cv::Mat gx() const { return gx_; }

  //! get y derivative
  cv::Mat gy() const { return gy_; }

  GradientRange gradientRange() const {
    GT val = this->intRange_.upper * derivative_.max().max_1st;
    return GradientRange(-val, val);
  }

  //! test if magnitude is computed
  inline bool isMagnitudeDone() const { return mag_done_; }

  //! get magnitude
  cv::Mat magnitude() const {
    if (!mag_done_) {
      MO<GT, MT>::process(gx_, gy_, mag_);
      mag_done_ = true;
    }
    return mag_;
  }

  //! test if direction is computed
  inline bool isDirectionDone() const { return dir_done_; }

  //! get direction
  cv::Mat direction() const {
    if (!dir_done_) {
      DO<GT, DT>::process(gx_, gy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  DirectionRange directionRange() const { return DO<GT, DT>::range(); }

  MagnitudeRange magnitudeRange() const { return Range<MT>(0, magnitudeMaxStep(this->intRange_.upper)); }

  MT magnitudeThreshold(double val) const { return static_cast<MT>(magnitudeRange().upper * MO<GT, MT>::singled(val)); }

  //! get maximum magnitude step for intensity step
  MT magnitudeMaxStep(IT intensity = 1) const { return MO<GT, MT>::max(derivativeMax(), intensity); }

  //! compute single value magnitude
  MT magnitudeSingle(MT val) const { return static_cast<MT>(MO<GT, MT>::singled(val)); }

  DerivativeMax<GT> derivativeMax() const { return derivative_.max(); }

  //! get name of gradient operator
  inline std::string name() const { return "derivative_" + derivative_.name(); }

  NormType normType() const { return MO<GT, MT>::normType(); }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using Gradient<IT, GT, MT, DT>::intensityRange;
  using Gradient<IT, GT, MT, DT>::results;
};

}  // namespace lsfm
