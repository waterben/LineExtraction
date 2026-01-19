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
#include <imgproc/laplace.hpp>

namespace lsfm {

//! Quadrature interface class (local energy)
//! Use IT to define Image type (8Bit, 16Bit, 32Bit, or floating type float or double)
//! Use GT to define result types for even and odd operations (short, float or double)
//! Use MT to define magnitude of odd operators (int, float or double)
//! Use ET to define energy type (int, float or double)
//! Use DT to define phase or direction type (float or double)
//! For images with floating values, use IT = image floating type (float or double)
template <class IT, class GT, class MT, class ET, class DT>
class QuadratureI : public LaplaceI<IT, GT> {
  QuadratureI(const QuadratureI&);

 protected:
  QuadratureI() {}

 public:
  typedef IT img_type;
  typedef GT grad_type;
  typedef MT mag_type;
  typedef ET energy_type;
  typedef DT dir_type;
  typedef DT phase_type;

  typedef Range<IT> IntensityRange;
  typedef Range<GT> GradientRange;
  typedef Range<MT> MagnitudeRange;
  typedef Range<ET> EnergyRange;
  typedef Range<DT> DirectionRange;
  typedef Range<DT> PhaseRange;

  virtual ~QuadratureI() {}

  //! get even response
  virtual cv::Mat laplace() const override { return even(); }

  //! get even response range
  virtual GradientRange laplaceRange() const override { return evenRange(); }

  //! convert threshold between 0-1 to even threshold
  virtual GT laplaceThreshold(double val) const override { return evenThreshold(val); }

  //! get even response
  virtual cv::Mat even() const = 0;

  //! get even response range
  virtual GradientRange evenRange() const = 0;

  //! convert threshold between 0-1 to even threshold
  virtual GT evenThreshold(double val) const { return static_cast<GT>(evenRange().size() * val); }

  //! get odd response
  virtual cv::Mat odd() const = 0;

  //! get odd response range (combined)
  virtual MagnitudeRange oddRange() const = 0;

  //! convert threshold between 0-1 to odd magnitude threshold
  virtual MT oddThreshold(double val) const { return static_cast<MT>(oddRange().upper * val); }

  //! get single odd responses
  virtual void odd(cv::Mat& ox, cv::Mat& oy) const = 0;

  //! get x response of odd filter
  virtual cv::Mat oddx() const {
    cv::Mat ox, oy;
    odd(ox, oy);
    return ox;
  }

  //! get y response of odd filter
  virtual cv::Mat oddy() const {
    cv::Mat ox, oy;
    odd(ox, oy);
    return oy;
  }

  //! get odd response range (single direction)
  virtual GradientRange oddGradRange() const {
    double val = oddRange().upper * 0.707106781;
    return GradientRange(static_cast<GT>(-val), static_cast<GT>(val));
  }


  //! get information about odd magnitude norm type
  virtual NormType normType() const { return NormType::NORM_L2; }

  //! get direction off odd filter
  virtual cv::Mat direction() const = 0;

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  virtual DirectionRange directionRange() const = 0;

  //! get energy
  virtual cv::Mat energy() const = 0;

  //! get magnitude range for intensity range
  virtual EnergyRange energyRange() const = 0;

  //! convert threshold between 0-1 to energy threshold
  virtual ET energyThreshold(double val) const { return static_cast<ET>(energyRange().upper * val); }

  //! get phase
  virtual cv::Mat phase() const = 0;

  //! get phase range ([-PI,PI], [0,2PI] or [0,360])
  virtual PhaseRange phaseRange() const = 0;
};

//! Quadrature base class
template <class IT, class GT, class MT, class ET, class DT>
class Quadrature : public QuadratureI<IT, GT, MT, ET, DT> {
  Quadrature();
  Quadrature(const Quadrature&);

 protected:
  Range<IT> intRange_;

  Quadrature(IT int_lower, IT int_upper) : intRange_(int_lower, int_upper) {
    if (intRange_.lower > intRange_.upper) intRange_.swap();
  }

 public:
  typedef IT img_type;
  typedef GT grad_type;
  typedef MT mag_type;
  typedef ET energy_type;
  typedef DT dir_type;
  typedef DT phase_type;

  typedef Range<IT> IntensityRange;
  typedef Range<GT> GradientRange;
  typedef Range<MT> MagnitudeRange;
  typedef Range<ET> EnergyRange;
  typedef Range<DT> DirectionRange;
  typedef Range<DT> PhaseRange;


  //! get image intensity range (for single channel)
  IntensityRange intensityRange() const { return intRange_; }

  //! generic interface to get processed data
  virtual FilterResults results() const {
    FilterResults ret;
    ret["even"] = FilterData(this->even(), this->evenRange());
    ret["oddx"] = FilterData(this->oddx(), this->oddGradRange());
    ret["oddy"] = FilterData(this->oddy(), this->oddGradRange());
    ret["odd"] = FilterData(this->odd(), this->oddRange());
    ret["dir"] = FilterData(this->direction(), this->directionRange());
    ret["energy"] = FilterData(this->energy(), this->energyRange());
    ret["phase"] = FilterData(this->phase(), this->phaseRange());
    return ret;
  }
};

//! Quadrature filter based on gradient and laplace filter (no real qudarature pair
//! since signal don't have a hilbert constrain -> invalid phase)
template <class IT, class GT, class FT, class GRAD, class LAPLACE, template <typename, typename> class P = Polar>
class QuadratureSimple : public Quadrature<IT, GT, FT, FT, FT> {
  QuadratureSimple();
  QuadratureSimple(const QuadratureSimple&);

  GRAD grad_;
  LAPLACE laplace_;

  mutable bool phase_done_, energy_done_;

  mutable cv::Mat_<FT> energy_, phase_;

  Range<FT> energyRange_;

  void max_response() {
    GradientRange lr = laplace_.laplaceRange();
    GradientRange gr = grad_.gradientRange();
    FT lmax = std::max(lr.lower * lr.lower, lr.upper * lr.upper);
    energyRange_.lower = 0;
    energyRange_.upper = std::sqrt(lmax + static_cast<FT>(gr.upper) * gr.upper);
  }

  inline void init() {
    phase_done_ = false;
    energy_done_ = false;
    this->addManager(grad_);
    this->addManager(laplace_);

    max_response();
  }

 public:
  typedef IT img_type;
  typedef GT grad_type;
  typedef FT mag_type;
  typedef FT energy_type;
  typedef FT dir_type;
  typedef FT phase_type;

  typedef Range<IT> IntensityRange;
  typedef Range<GT> GradientRange;
  typedef Range<FT> MagnitudeRange;
  typedef Range<FT> EnergyRange;
  typedef Range<FT> DirectionRange;
  typedef Range<FT> PhaseRange;

  QuadratureSimple(IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        grad_(int_lower, int_upper),
        laplace_(int_lower, int_upper) {
    init();
  }

  QuadratureSimple(const ValueManager::NameValueVector& options,
                   IT int_lower = std::numeric_limits<IT>::lowest(),
                   IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        grad_(int_lower, int_upper),
        laplace_(int_lower, int_upper) {
    init();
    this->value(options);
  }

  QuadratureSimple(ValueManager::InitializerList options,
                   IT int_lower = std::numeric_limits<IT>::lowest(),
                   IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        grad_(int_lower, int_upper),
        laplace_(int_lower, int_upper) {
    init();
    this->value(options);
  }


  using Quadrature<IT, GT, FT, FT, FT>::intensityRange;
  using Quadrature<IT, GT, FT, FT, FT>::results;

  void process(const cv::Mat& img) {
    energy_done_ = false;
    phase_done_ = false;
    grad_.process(img);
    laplace_.process(img);
  }

  cv::Mat even() const { return laplace_.laplace(); }

  GradientRange evenRange() const { return laplace_.laplaceRange(); }

  GT evenThreshold(double val) const { return laplace_.laplaceThreshold(val); }

  cv::Mat odd() const { return grad_.magnitude(); }

  MagnitudeRange oddRange() const { return grad_.magnitudeRange(); }

  FT oddThreshold(double val) { return grad_.magnitudeThreshold(val); }

  void odd(cv::Mat& ox, cv::Mat& oy) const { return grad_.directionals(ox, oy); }

  cv::Mat oddx() const { return grad_.gx(); }

  cv::Mat oddy() const { return grad_.gy(); }

  GradientRange oddGradRange() const { return grad_.gradientRange(); }

  NormType normType() const { return grad_.normType(); }

  cv::Mat direction() const { return grad_.direction(); }

  DirectionRange directionRange() const { return grad_.directionRange(); }

  cv::Mat energy() const {
    if (!energy_done_) {
      P<FT, FT>::magnitude(odd(), even(), energy_);
      energy_done_ = true;
    }
    return energy_;
  }

  EnergyRange energyRange() const { return energyRange_; }

  cv::Mat phase() const {
    if (!phase_done_) {
      P<FT, FT>::phase(odd(), even(), phase_);
      phase_done_ = true;
    }
    return phase_;
  }

  PhaseRange phaseRange() const { return P<FT, FT>::range(); }

  std::string name() const { return "quad_" + grad_.name() + "_" + laplace_.name(); }
};

}  // namespace lsfm
