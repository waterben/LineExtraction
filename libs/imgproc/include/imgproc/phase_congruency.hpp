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

#include <imgproc/quadrature.hpp>

namespace lsfm {

/// @brief Abstract interface for phase congruency computation.
///
/// Phase congruency is a measure of feature significance based on the principle
/// that features occur at points where Fourier components are maximally in phase.
/// It provides illumination and contrast invariant edge/feature detection.
///
/// @tparam ET Energy type (float or double)
template <class ET>
class PhaseCongruencyI {
  PhaseCongruencyI(const PhaseCongruencyI&);

 protected:
  PhaseCongruencyI() {}

 public:
  typedef ET energy_type;         ///< Energy/phase congruency value type
  typedef Range<ET> EnergyRange;  ///< Range for energy values

  virtual ~PhaseCongruencyI() {}

  /// @brief Get phase congruency image.
  /// @return Phase congruency values in range [0, 1]
  virtual cv::Mat phaseCongruency() const = 0;

  /// @brief Get phase congruency value range.
  /// @return Range [0, 1] by default
  virtual EnergyRange phaseCongruencyRange() const { return EnergyRange(0, 1); }
};

/// @brief Base implementation for phase congruency with quadrature filter support.
///
/// Combines phase congruency interface with quadrature filter functionality.
///
/// @tparam IT Input image pixel type
/// @tparam GT Gradient type
/// @tparam MT Magnitude type
/// @tparam ET Energy type
/// @tparam DT Direction/phase type
template <class IT, class GT, class MT, class ET, class DT>
class PhaseCongruency : public PhaseCongruencyI<ET>, public QuadratureI<IT, GT, MT, ET, DT> {
  PhaseCongruency();
  PhaseCongruency(const PhaseCongruency&);

 protected:
  Range<IT> intRange_;  ///< Input intensity range

  /// @brief Construct with intensity range.
  /// @param int_lower Lower bound of input intensity
  /// @param int_upper Upper bound of input intensity
  PhaseCongruency(IT int_lower, IT int_upper) : intRange_(int_lower, int_upper) {
    if (intRange_.lower > intRange_.upper) intRange_.swap();
  }

 public:
  typedef IT img_type;     ///< Input image pixel type
  typedef GT grad_type;    ///< Gradient type
  typedef MT mag_type;     ///< Magnitude type
  typedef ET energy_type;  ///< Energy type
  typedef DT dir_type;     ///< Direction type
  typedef DT phase_type;   ///< Phase type

  typedef Range<IT> IntensityRange;  ///< Range for intensity values
  typedef Range<GT> GradientRange;   ///< Range for gradient values
  typedef Range<MT> MagnitudeRange;  ///< Range for magnitude values
  typedef Range<ET> EnergyRange;     ///< Range for energy values
  typedef Range<DT> DirectionRange;  ///< Range for direction values
  typedef Range<DT> PhaseRange;      ///< Range for phase values

  /// @brief Get the expected input image intensity range.
  /// @return The intensity range for single-channel input images
  IntensityRange intensityRange() const { return intRange_; }

  /// @brief Get all filter outputs as named results.
  /// @return Map containing all outputs including phase congruency
  virtual FilterResults results() const {
    FilterResults ret;
    ret["even"] = FilterData(this->even(), this->evenRange());
    ret["oddx"] = FilterData(this->oddx(), this->oddGradRange());
    ret["oddy"] = FilterData(this->oddy(), this->oddGradRange());
    ret["odd"] = FilterData(this->odd(), this->oddRange());
    ret["dir"] = FilterData(this->direction(), this->directionRange());
    ret["energy"] = FilterData(this->energy(), this->energyRange());
    ret["phase"] = FilterData(this->phase(), this->phaseRange());
    ret["pc"] = FilterData(this->phaseCongruency(), this->phaseCongruencyRange());
    return ret;
  }
};


/// @brief Abstract interface for phase congruency Laplacian responses.
///
/// Extends phase congruency with directional Laplacian responses for
/// more detailed feature characterization.
///
/// @tparam ET Energy type (float or double)
template <class ET>
class PhaseCongruencyLaplaceI {
  PhaseCongruencyLaplaceI(const PhaseCongruencyLaplaceI&);

 protected:
  PhaseCongruencyLaplaceI() {}

 public:
  typedef ET energy_type;         ///< Energy value type
  typedef Range<ET> EnergyRange;  ///< Range for energy values

  virtual ~PhaseCongruencyLaplaceI() {}

  /// @brief Get separate phase congruency Laplacian responses.
  /// @param[out] lx X-direction Laplacian response
  /// @param[out] ly Y-direction Laplacian response
  virtual void pcLaplace(cv::Mat& lx, cv::Mat& ly) const = 0;

  /// @brief Get x-direction phase congruency Laplacian.
  /// @return X-direction Laplacian response
  virtual cv::Mat pclx() const {
    cv::Mat lx, ly;
    pcLaplace(lx, ly);
    return lx;
  }

  /// @brief Get y-direction phase congruency Laplacian.
  /// @return Y-direction Laplacian response
  virtual cv::Mat pcly() const {
    cv::Mat lx, ly;
    pcLaplace(lx, ly);
    return ly;
  }

  /// @brief Get phase congruency Laplacian value range.
  /// @return Range for Laplacian values
  virtual EnergyRange pcLaplaceRange() const = 0;
};

/// @brief Base implementation for phase congruency with Laplacian support.
///
/// Combines phase congruency Laplacian interface with quadrature filter functionality.
///
/// @tparam IT Input image pixel type
/// @tparam GT Gradient type
/// @tparam MT Magnitude type
/// @tparam ET Energy type
/// @tparam DT Direction/phase type
template <class IT, class GT, class MT, class ET, class DT>
class PhaseCongruencyLaplace : public PhaseCongruencyLaplaceI<ET>, public QuadratureI<IT, GT, MT, ET, DT> {
  PhaseCongruencyLaplace();
  PhaseCongruencyLaplace(const PhaseCongruencyLaplace&);

 protected:
  Range<IT> intRange_;  ///< Input intensity range

  /// @brief Construct with intensity range.
  /// @param int_lower Lower bound of input intensity
  /// @param int_upper Upper bound of input intensity
  PhaseCongruencyLaplace(IT int_lower, IT int_upper) : intRange_(int_lower, int_upper) {
    if (intRange_.lower > intRange_.upper) intRange_.swap();
  }

 public:
  typedef IT img_type;     ///< Input image pixel type
  typedef GT grad_type;    ///< Gradient type
  typedef MT mag_type;     ///< Magnitude type
  typedef ET energy_type;  ///< Energy type
  typedef DT dir_type;     ///< Direction type
  typedef DT phase_type;   ///< Phase type

  typedef Range<IT> IntensityRange;  ///< Range for intensity values
  typedef Range<GT> GradientRange;   ///< Range for gradient values
  typedef Range<MT> MagnitudeRange;  ///< Range for magnitude values
  typedef Range<ET> EnergyRange;     ///< Range for energy values
  typedef Range<DT> DirectionRange;  ///< Range for direction values
  typedef Range<DT> PhaseRange;      ///< Range for phase values

  /// @brief Get the expected input image intensity range.
  /// @return The intensity range for single-channel input images
  IntensityRange intensityRange() const { return intRange_; }

  /// @brief Get all filter outputs as named results.
  /// @return Map containing all outputs including PC Laplacian
  virtual FilterResults results() const {
    FilterResults ret;
    ret["even"] = FilterData(this->even(), this->evenRange());
    ret["oddx"] = FilterData(this->oddx(), this->oddGradRange());
    ret["oddy"] = FilterData(this->oddy(), this->oddGradRange());
    ret["odd"] = FilterData(this->odd(), this->oddRange());
    ret["dir"] = FilterData(this->direction(), this->directionRange());
    ret["energy"] = FilterData(this->energy(), this->energyRange());
    ret["phase"] = FilterData(this->phase(), this->phaseRange());
    ret["pclx"] = FilterData(this->pclx(), this->pcLaplaceRange());
    ret["pcly"] = FilterData(this->pcly(), this->pcLaplaceRange());
    return ret;
  }
};

}  // namespace lsfm
