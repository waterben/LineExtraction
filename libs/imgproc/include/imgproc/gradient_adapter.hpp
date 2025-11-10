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

//! Gradient odd adapter class
template <class QUAD>
class GradientOdd : public GradientI<typename QUAD::img_type,
                                     typename QUAD::grad_type,
                                     typename QUAD::mag_type,
                                     typename QUAD::dir_type> {
  typedef GradientI<typename QUAD::img_type, typename QUAD::grad_type, typename QUAD::mag_type, typename QUAD::dir_type>
      GRAD;

 protected:
  QUAD quad_;

 public:
  typedef typename QUAD::img_type img_type;
  typedef typename QUAD::mag_type mag_type;
  typedef typename QUAD::grad_type grad_type;
  typedef typename QUAD::dir_type dir_type;
  typedef typename QUAD::IntensityRange IntensityRange;
  typedef typename QUAD::GradientRange GradientRange;
  typedef typename QUAD::MagnitudeRange MagnitudeRange;
  typedef typename QUAD::DirectionRange DirectionRange;

  GradientOdd(img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : quad_({}, int_lower, int_upper) {
    this->addManager(quad_);
  }

  GradientOdd(const ValueManager::NameValueVector& options,
              img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : quad_(options, int_lower, int_upper) {
    this->addManager(quad_);
  }

  GradientOdd(ValueManager::InitializerList options,
              img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : quad_(options, int_lower, int_upper) {
    this->addManager(quad_);
  }

  //! get magnitude
  virtual cv::Mat magnitude() const { return quad_.odd(); }

  //! get magnitude range for intensity range
  virtual MagnitudeRange magnitudeRange() const { return quad_.oddRange(); }

  //! convert threshold between 0-1 to magnitude threshold
  virtual mag_type magnitudeThreshold(double val) const { return quad_.oddThreshold(val); }

  //! get information about magnitude norm type
  //! default: NONE -> norm does not correspond gx, gy
  virtual NormType normType() const { return quad_.normType(); }

  //! get direction
  virtual cv::Mat direction() const { return quad_.direction(); }

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  virtual DirectionRange directionRange() const { return quad_.directionRange(); }

  //! get x,y derivatives or directional data
  virtual void directionals(cv::Mat& grad_x, cv::Mat& grad_y) const { quad_.odd(grad_x, grad_y); }

  //! get gradient range
  virtual GradientRange gradientRange() const { return quad_.oddGradRange(); }

  //! get image intensity range (for single channel)
  IntensityRange intensityRange() const { return quad_.intensityRange(); }

  //! process filter
  virtual void process(const cv::Mat& img) { quad_.process(img); }

  //! generic interface to get processed data
  virtual FilterResults results() const {
    FilterResults ret = quad_.results();
    ret["gx"] = this->gx();
    ret["gy"] = this->gy();
    ret["mag"] = this->magnitude();
    ret["dir"] = this->direction();
    return ret;
  }

  //! get name of direction operator
  std::string name() const { return quad_.name(); }

  using GRAD::gx;
  using GRAD::gy;
  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
};

//! Gradient energy adapter class
template <class QUAD>
class GradientEnergy : public GradientOdd<QUAD> {
  typedef GradientOdd<QUAD> GRAD;

 public:
  typedef typename QUAD::img_type img_type;
  typedef typename QUAD::mag_type mag_type;
  typedef typename QUAD::grad_type grad_type;
  typedef typename QUAD::dir_type dir_type;
  typedef typename QUAD::IntensityRange IntensityRange;
  typedef typename QUAD::GradientRange GradientRange;
  typedef typename QUAD::MagnitudeRange MagnitudeRange;
  typedef typename QUAD::DirectionRange DirectionRange;

  GradientEnergy(img_type int_lower = std::numeric_limits<img_type>::lowest(),
                 img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(int_lower, int_upper) {}

  GradientEnergy(const ValueManager::NameValueVector& options,
                 img_type int_lower = std::numeric_limits<img_type>::lowest(),
                 img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(options, int_lower, int_upper) {}

  GradientEnergy(ValueManager::InitializerList options,
                 img_type int_lower = std::numeric_limits<img_type>::lowest(),
                 img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(options, int_lower, int_upper) {}

  //! get magnitude
  virtual cv::Mat magnitude() const { return this->quad_.energy(); }

  //! get magnitude range for intensity range
  virtual MagnitudeRange magnitudeRange() const { return this->quad_.energyRange(); }

  //! convert threshold between 0-1 to magnitude threshold
  virtual mag_type magnitudeThreshold(double val) const { return this->quad_.energyThreshold(val); }

  using GRAD::direction;
  using GRAD::directionals;
  using GRAD::directionRange;
  using GRAD::gradientRange;
  using GRAD::gx;
  using GRAD::gy;
  using GRAD::intensityRange;
  using GRAD::name;
  using GRAD::normType;
  using GRAD::process;
  using GRAD::results;
  using GRAD::value;
  using GRAD::valuePair;
  using GRAD::values;
};


//! Gradient phase congruency adapter class
template <class QUAD>
class GradientPC : public GradientOdd<QUAD> {
  typedef GradientOdd<QUAD> GRAD;

 public:
  typedef typename QUAD::img_type img_type;
  typedef typename QUAD::mag_type mag_type;
  typedef typename QUAD::grad_type grad_type;
  typedef typename QUAD::dir_type dir_type;
  typedef typename QUAD::IntensityRange IntensityRange;
  typedef typename QUAD::GradientRange GradientRange;
  typedef typename QUAD::MagnitudeRange MagnitudeRange;
  typedef typename QUAD::DirectionRange DirectionRange;

  GradientPC(img_type int_lower = std::numeric_limits<img_type>::lowest(),
             img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(int_lower, int_upper) {}

  GradientPC(const ValueManager::NameValueVector& options,
             img_type int_lower = std::numeric_limits<img_type>::lowest(),
             img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(options, int_lower, int_upper) {}

  GradientPC(ValueManager::InitializerList options,
             img_type int_lower = std::numeric_limits<img_type>::lowest(),
             img_type int_upper = std::numeric_limits<img_type>::max())
      : GradientOdd<QUAD>(options, int_lower, int_upper) {}

  //! get magnitude
  virtual cv::Mat magnitude() const { return this->quad_.phaseCongruency(); }

  //! get magnitude range for intensity range
  virtual MagnitudeRange magnitudeRange() const { return this->quad_.phaseCongruencyRange(); }

  //! convert threshold between 0-1 to magnitude threshold
  virtual mag_type magnitudeThreshold(double val) const { return static_cast<mag_type>(val); }

  using GRAD::direction;
  using GRAD::directionals;
  using GRAD::directionRange;
  using GRAD::gradientRange;
  using GRAD::gx;
  using GRAD::gy;
  using GRAD::intensityRange;
  using GRAD::name;
  using GRAD::normType;
  using GRAD::process;
  using GRAD::results;
  using GRAD::value;
  using GRAD::valuePair;
  using GRAD::values;
};

}  // namespace lsfm
