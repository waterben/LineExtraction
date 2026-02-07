//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file gradient.hpp
/// @brief Abstract interface for image gradient computation filters.

#pragma once

#include <imgproc/derivative.hpp>
#include <imgproc/direction.hpp>
#include <imgproc/filter.hpp>
#include <imgproc/magnitude.hpp>


namespace lsfm {

/// @brief Abstract interface for gradient computation filters.
///
/// Provides a unified interface for computing image gradients, including
/// magnitude, direction, and directional derivatives (gx, gy).
/// Inherits from FilterI for the basic filter interface.
/// @tparam IT Input image pixel type (uchar, short, float, double)
/// @tparam GT Gradient/derivative type (short for 8-bit, float for 16-bit)
/// @tparam MT Magnitude type (int, float, or double)
/// @tparam DT Direction type (float or double)
template <class IT, class GT, class MT, class DT>
class GradientI : public FilterI<IT> {
  GradientI(const GradientI&);

 protected:
  GradientI() {}

 public:
  typedef IT img_type;               ///< Input image pixel type
  typedef MT mag_type;               ///< Magnitude value type
  typedef GT grad_type;              ///< Gradient/derivative value type
  typedef DT dir_type;               ///< Direction value type
  typedef Range<IT> IntensityRange;  ///< Range type for intensity values
  typedef Range<GT> GradientRange;   ///< Range type for gradient values
  typedef Range<MT> MagnitudeRange;  ///< Range type for magnitude values
  typedef Range<DT> DirectionRange;  ///< Range type for direction values

  virtual ~GradientI() {}

  /// @brief Get the gradient magnitude image.
  /// @return Matrix containing gradient magnitude at each pixel
  virtual cv::Mat magnitude() const = 0;

  /// @brief Get the expected magnitude value range.
  /// @return Range of possible magnitude values based on intensity range
  virtual MagnitudeRange magnitudeRange() const = 0;

  /// @brief Convert normalized threshold [0,1] to magnitude threshold.
  /// @param val Normalized threshold value in range [0,1]
  /// @return Corresponding magnitude threshold value
  virtual MT magnitudeThreshold(double val) const { return static_cast<MT>(magnitudeRange().upper * val); }

  /// @brief Get the magnitude norm type used.
  ///
  /// Indicates how magnitude relates to gx, gy derivatives.
  /// @return NormType::NONE if norm doesn't correspond to gx, gy
  virtual NormType normType() const { return NormType::NONE; }

  /// @brief Get the gradient direction image.
  /// @return Matrix containing gradient direction at each pixel
  virtual cv::Mat direction() const = 0;

  /// @brief Get the direction value range.
  /// @return Range of direction values (e.g., [-PI,PI] or [0,360])
  virtual DirectionRange directionRange() const = 0;

  /// @brief Get directional derivatives (gx, gy).
  ///
  /// Default implementation reconstructs from polar coordinates.
  /// @param[out] gx X-direction gradient output
  /// @param[out] gy Y-direction gradient output
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

  /// @brief Get X-direction gradient.
  /// @return Matrix containing gradient in X direction
  virtual cv::Mat gx() const {
    cv::Mat gx, gy;
    directionals(gx, gy);
    return gx;
  }

  /// @brief Get Y-direction gradient.
  /// @return Matrix containing gradient in Y direction
  virtual cv::Mat gy() const {
    cv::Mat gx, gy;
    directionals(gx, gy);
    return gy;
  }

  /// @brief Get the gradient value range for single direction.
  /// @return Range of possible gradient values
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

/// @brief Base implementation class for gradient computation.
///
/// Provides common functionality for gradient filters including
/// intensity range management and default results() implementation.
/// @tparam IT Input image pixel type (uchar, short, float, double)
/// @tparam GT Gradient/derivative type (short for 8-bit, float for 16-bit)
/// @tparam MT Magnitude type (int, float, or double)
/// @tparam DT Direction type (float or double)
template <class IT, class GT, class MT, class DT>
class Gradient : public GradientI<IT, GT, MT, DT> {
  Gradient();
  Gradient(const Gradient&);

 protected:
  Range<IT> intRange_;  ///< Input intensity range

  /// @brief Construct with intensity range.
  /// @param int_lower Lower bound of input intensity range
  /// @param int_upper Upper bound of input intensity range
  Gradient(IT int_lower, IT int_upper) : intRange_(int_lower, int_upper) {
    if (intRange_.lower > intRange_.upper) intRange_.swap();
  }

 public:
  typedef IT img_type;               ///< Input image pixel type
  typedef MT mag_type;               ///< Magnitude value type
  typedef GT grad_type;              ///< Gradient/derivative value type
  typedef DT dir_type;               ///< Direction value type
  typedef Range<IT> IntensityRange;  ///< Range type for intensity values
  typedef Range<GT> GradientRange;   ///< Range type for gradient values
  typedef Range<MT> MagnitudeRange;  ///< Range type for magnitude values
  typedef Range<DT> DirectionRange;  ///< Range type for direction values

  /// @brief Get the expected input image intensity range.
  /// @return The intensity range for single-channel input images
  IntensityRange intensityRange() const { return intRange_; }

  /// @brief Get all gradient outputs as named results.
  ///
  /// Returns gx, gy, magnitude, and direction as FilterData.
  /// @return Map of output name to FilterData
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
