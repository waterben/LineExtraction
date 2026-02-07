//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file derivative_gradient.hpp
/// @brief Gradient computation using derivative operators.
///
/// This file provides a flexible gradient operator that combines configurable
/// derivative operators (Sobel, Scharr, etc.) with magnitude and direction
/// computation policies.

#pragma once

#include <imgproc/gradient.hpp>

namespace lsfm {

/// @brief Gradient operator using configurable derivative, magnitude, and direction policies.
///
/// This class provides a flexible gradient computation framework by combining
/// three configurable policies:
/// - GO: Derivative operator (e.g., SobelDerivative, ScharrDerivative)
/// - MO: Magnitude operator (e.g., Magnitude, MagnitudeL1)
/// - DO: Direction operator (e.g., Direction, Direction360)
/// Magnitude and direction are computed lazily on first access.
/// @tparam IT Input image type (uchar for 8-bit, ushort for 16-bit, float/double).
/// @tparam GT Gradient component type (short for 8-bit images, float for 16-bit).
/// @tparam MT Magnitude type (int, float, or double).
/// @tparam DT Direction type (float or double).
/// @tparam GO Derivative operator template (default: SobelDerivative).
/// @tparam MO Magnitude operator template (default: Magnitude, i.e., L2 norm).
/// @tparam DO Direction operator template (default: Direction, i.e., atan2).
template <class IT = uchar,
          class GT = short,
          class MT = int,
          class DT = float,
          template <class, class> class GO = SobelDerivative,
          template <class, class> class MO = Magnitude,
          template <class, class> class DO = Direction>
class DerivativeGradient : public Gradient<IT, GT, MT, DT> {
  GO<IT, GT> derivative_;  ///< Derivative operator instance.

  mutable cv::Mat_<GT> gx_;   ///< Cached X-gradient.
  mutable cv::Mat_<GT> gy_;   ///< Cached Y-gradient.
  mutable cv::Mat_<MT> mag_;  ///< Cached magnitude (lazy).
  mutable cv::Mat_<DT> dir_;  ///< Cached direction (lazy).

  mutable bool mag_done_;  ///< True if magnitude has been computed.
  mutable bool dir_done_;  ///< True if direction has been computed.

 public:
  typedef IT img_type;               ///< Input image pixel type.
  typedef GT grad_type;              ///< Gradient component type.
  typedef MT mag_type;               ///< Magnitude type.
  typedef DT dir_type;               ///< Direction type.
  typedef Range<IT> IntensityRange;  ///< Intensity value range type.
  typedef Range<GT> GradientRange;   ///< Gradient value range type.
  typedef Range<MT> MagnitudeRange;  ///< Magnitude value range type.
  typedef Range<DT> DirectionRange;  ///< Direction value range type.

  /// @brief Construct with default derivative parameters.
  /// @param[in] int_lower Lower bound of input intensity range.
  /// @param[in] int_upper Upper bound of input intensity range.
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

  /// @brief Construct with named options.
  /// @param[in] options Configuration options for the derivative operator.
  /// @param[in] int_lower Lower bound of input intensity range.
  /// @param[in] int_upper Upper bound of input intensity range.
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

  /// @brief Construct with initializer list options.
  /// @param[in] options Configuration options for the derivative operator.
  /// @param[in] int_lower Lower bound of input intensity range.
  /// @param[in] int_upper Upper bound of input intensity range.
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

  /// @brief Process an image to compute gradients.
  ///
  /// Computes X and Y derivatives. Magnitude and direction are computed
  /// lazily when accessed via magnitude() or direction().
  /// @param[in] img Input image.
  void process(const cv::Mat& img) {
    mag_done_ = false;
    dir_done_ = false;

    derivative_.process(img, this->gx_, this->gy_);
  }

  /// @brief Process image and retrieve gradient components.
  /// @param[in] img Input image.
  /// @param[out] gx Output X-gradient.
  /// @param[out] gy Output Y-gradient.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) {
    process(img);
    directionals(gx, gy);
  }

  /// @brief Process image and retrieve gradients and magnitude.
  /// @param[in] img Input image.
  /// @param[out] gx Output X-gradient.
  /// @param[out] gy Output Y-gradient.
  /// @param[out] mag Output magnitude.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
  }

  /// @brief Process image and retrieve all gradient outputs.
  /// @param[in] img Input image.
  /// @param[out] gx Output X-gradient.
  /// @param[out] gy Output Y-gradient.
  /// @param[out] mag Output magnitude.
  /// @param[out] dir Output direction.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag, cv::Mat& dir) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
    dir = direction();
  }

  /// @brief Get both gradient components.
  /// @param[out] gx X-gradient image reference.
  /// @param[out] gy Y-gradient image reference.
  void directionals(cv::Mat& gx, cv::Mat& gy) const {
    gx = gx_;
    gy = gy_;
  }

  /// @brief Get X-gradient image.
  /// @return X-gradient component.
  cv::Mat gx() const { return gx_; }

  /// @brief Get Y-gradient image.
  /// @return Y-gradient component.
  cv::Mat gy() const { return gy_; }

  /// @brief Get gradient value range.
  /// @return Range of possible gradient component values.
  GradientRange gradientRange() const {
    GT val = this->intRange_.upper * derivative_.max().max_1st;
    return GradientRange(-val, val);
  }

  /// @brief Check if magnitude has been computed.
  /// @return True if magnitude() has been called, false otherwise.
  inline bool isMagnitudeDone() const { return mag_done_; }

  /// @brief Get gradient magnitude image.
  ///
  /// Magnitude is computed lazily on first access using the MO policy.
  /// @return Magnitude image.
  cv::Mat magnitude() const {
    if (!mag_done_) {
      MO<GT, MT>::process(gx_, gy_, mag_);
      mag_done_ = true;
    }
    return mag_;
  }

  /// @brief Check if direction has been computed.
  /// @return True if direction() has been called, false otherwise.
  inline bool isDirectionDone() const { return dir_done_; }

  /// @brief Get gradient direction image.
  ///
  /// Direction is computed lazily on first access using the DO policy.
  /// @return Direction image.
  cv::Mat direction() const {
    if (!dir_done_) {
      DO<GT, DT>::process(gx_, gy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /// @brief Get direction value range.
  /// @return Range of direction values.
  DirectionRange directionRange() const { return DO<GT, DT>::range(); }

  /// @brief Get magnitude value range.
  /// @return Range of possible magnitude values.
  MagnitudeRange magnitudeRange() const { return Range<MT>(0, magnitudeMaxStep(this->intRange_.upper)); }

  /// @brief Convert normalized threshold to magnitude threshold.
  /// @param[in] val Normalized threshold value in [0, 1].
  /// @return Absolute magnitude threshold.
  MT magnitudeThreshold(double val) const { return static_cast<MT>(magnitudeRange().upper * MO<GT, MT>::singled(val)); }

  /// @brief Get maximum magnitude for a given intensity step.
  /// @param[in] intensity Intensity step value (default: 1).
  /// @return Maximum magnitude value.
  MT magnitudeMaxStep(IT intensity = 1) const { return MO<GT, MT>::max(derivativeMax(), intensity); }

  /// @brief Compute single-value magnitude normalization factor.
  /// @param[in] val Input value.
  /// @return Normalized magnitude value.
  MT magnitudeSingle(MT val) const { return static_cast<MT>(MO<GT, MT>::singled(val)); }

  /// @brief Get derivative operator maximum response information.
  /// @return DerivativeMax structure with max response values.
  DerivativeMax<GT> derivativeMax() const { return derivative_.max(); }

  /// @brief Get the name of this gradient operator.
  /// @return Name string combining "derivative_" with the derivative operator name.
  inline std::string name() const { return "derivative_" + derivative_.name(); }

  /// @brief Get the norm type used for magnitude computation.
  /// @return NormType enum value.
  NormType normType() const { return MO<GT, MT>::normType(); }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using Gradient<IT, GT, MT, DT>::intensityRange;
  using Gradient<IT, GT, MT, DT>::results;
};

}  // namespace lsfm
