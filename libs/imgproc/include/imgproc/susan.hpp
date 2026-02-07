//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file susan.hpp
/// @brief SUSAN edge detection algorithm.
///
/// Wrapper around the SUSAN core algorithm (susan_core.hpp) providing the
/// Gradient<> interface, option management, and lazy direction computation.
/// The core algorithm implementation is by Stephen Smith (FMRIB, Oxford)
/// under Crown Copyright (1995-1999), and resides in
/// third-party/contrib/imgproc_external under its original copyright.
///
/// @see Smith, S.M. and Brady, J.M., "SUSAN - A New Approach to Low Level
///      Image Processing", Int. Journal of Computer Vision, 23(1),
///      pp. 45-78, May 1997.

#pragma once

#include <imgproc/gradient.hpp>

#include <susan_core.hpp>

namespace lsfm {

/// @brief Safe type cast with compile-time type checking.
///
/// Returns the input value unchanged if InT and OutT are the same type,
/// otherwise performs a static_cast.
/// @tparam OutT Target output type.
/// @tparam InT Input type (deduced).
/// @param[in] in Input value to cast.
/// @return Value cast to OutT.
template <typename OutT, typename InT>
constexpr inline OutT auto_cast(const InT& in) {
  if constexpr (std::is_same<InT, OutT>::value) {
    return in;
  } else {
    return static_cast<OutT>(in);
  }
}

/// @brief SUSAN edge detector and gradient operator.
///
/// Implements the SUSAN (Smallest Univalue Segment Assimilating Nucleus)
/// edge detection algorithm by Smith and Brady. SUSAN detects edges by
/// examining the local brightness consistency within a circular mask
/// around each pixel, making it robust to noise while preserving edge
/// localization.
/// The algorithm uses a brightness look-up table (LUT) to efficiently
/// compute the exponential similarity function between pixel intensities.
/// @note This implementation is based on SUSAN Version 2l by Stephen Smith.
///
/// A UK patent applies to commercial use.
/// @see Smith, S.M. and Brady, J.M., "SUSAN - A New Approach to Low Level
/// Image Processing", Int. Journal of Computer Vision, 23(1), 1997.
/// @tparam GT Gradient component type (default: short).
/// @tparam MT Magnitude type (default: short).
/// @tparam DT Direction type (default: float).
/// @tparam DO Direction operator type (default: Direction<GT, DT>).
template <class GT = short, class MT = short, class DT = float, class DO = Direction<GT, DT>>
class SusanGradient : public Gradient<uchar, GT, MT, DT> {
  // Thresholds for brightness and distance
  MT bt_, max_no_;

  bool small_kernel_;
  mutable bool dir_done_;

  // LUT
  uchar bp_[516];  ///< Brightness look-up table for exponential function.

  cv::Mat_<MT> mag_;          ///< Computed magnitude image.
  cv::Mat_<GT> dx_;           ///< Computed X-gradient image.
  cv::Mat_<GT> dy_;           ///< Computed Y-gradient image.
  mutable cv::Mat_<DT> dir_;  ///< Computed direction image (lazy evaluation).


 public:
  typedef uchar img_type;  ///< Input image pixel type (grayscale only).
  typedef MT mag_type;     ///< Magnitude component type.
  typedef GT grad_type;    ///< Gradient component type.
  typedef DT dir_type;     ///< Direction component type.

  typedef Range<GT> GradientRange;   ///< Range type for gradient values.
  typedef Range<MT> MagnitudeRange;  ///< Range type for magnitude values.
  typedef Range<DT> DirectionRange;  ///< Range type for direction values.


  /// @brief Construct a SUSAN gradient operator.
  /// @param[in] bt Brightness threshold [0-256], controls edge sensitivity.
  /// Lower values give more edges (default: 20).
  /// @param[in] small_kernel Use 3x3 kernel instead of 37-pixel circular mask.
  /// Gives finer detail but may be noisier (default: false).
  /// @param[in] max_no Maximum response value for normalization (default: 2650).
  /// @param[in] int_lower Lower bound of input intensity range.
  /// @param[in] int_upper Upper bound of input intensity range.
  SusanGradient(MT bt = 20,
                bool small_kernel = false,
                MT max_no = 2650,
                uchar int_lower = std::numeric_limits<uchar>::lowest(),
                uchar int_upper = std::numeric_limits<uchar>::max())
      : Gradient<uchar, GT, MT, dir_type>(int_lower, int_upper),
        bt_(bt),
        max_no_(max_no),
        small_kernel_(small_kernel),
        dir_done_(false),
        mag_(),
        dx_(),
        dy_(),
        dir_() {
    this->add("grad_brightness_th",
              std::bind(&SusanGradient<GT, MT, DT, DO>::brightnessTh, this, std::placeholders::_1),
              "Brightness threshold [0-256].");
    // this->add("distance_th", std::bind(&SusanGradient<GT,MT,DT,DO>::distanceTh,this,std::placeholders::_1),"Distance
    // threshold [0-256]."); this->add("form",
    // std::bind(&SusanGradient<GT,MT,DT,DO>::form,this,std::placeholders::_1),"form of the LUT [2,6].");
    this->add("grad_small_kernel", std::bind(&SusanGradient<GT, MT, DT, DO>::smallKernel, this, std::placeholders::_1),
              "Use small kernel size.");
    this->add("grad_max_no", std::bind(&SusanGradient<GT, MT, DT, DO>::maxNo, this, std::placeholders::_1),
              "Maximum number for edges (2650).");

    susan_impl::SusanCore<GT, MT>::init_lut(bp_, bt_);
  }

  /// @brief Get or set the brightness threshold.
  /// @param bt Optional new brightness threshold [0-256].
  /// @return Current brightness threshold.
  Value brightnessTh(const Value& bt = Value::NAV()) {
    if (bt.type()) {
      bt_ = bt;
      susan_impl::SusanCore<GT, MT>::init_lut(bp_, bt_);
    }
    return bt_;
  }

  /// @brief Get or set whether to use the small 3x3 kernel.
  /// @param sk Optional new setting (true = 3x3, false = 37-pixel circular).
  /// @return Current small kernel setting.
  Value smallKernel(const Value& sk = Value::NAV()) {
    if (sk.type()) small_kernel_ = sk;
    return small_kernel_;
  }

  /// @brief Get or set the maximum response number.
  /// @param mn Optional new max number for edge response normalization.
  /// @return Current maximum number.
  Value maxNo(const Value& mn = Value::NAV()) {
    if (mn.type()) max_no_ = mn;
    return max_no_;
  }

  /// @brief Process an image to compute SUSAN edge gradients.
  ///
  /// Computes gradient magnitude and direction using the SUSAN algorithm.
  /// Results can be retrieved using magnitude(), direction(), gx(), gy().
  /// @param[in] img Input grayscale image (must be CV_8UC1).
  inline void process(const cv::Mat& img) {
    dx_.create(img.rows, img.cols);
    dx_.setTo(0);
    dy_.create(img.rows, img.cols);
    dy_.setTo(0);
    mag_.create(img.rows, img.cols);
    mag_.setTo(0);

    dir_done_ = false;

    if (small_kernel_)
      susan_impl::SusanCore<GT, MT>::edges_small(img, mag_, dx_, dy_, bp_, max_no_);
    else
      susan_impl::SusanCore<GT, MT>::edges(img, mag_, dx_, dy_, bp_, max_no_);
  }

  /// @brief Process image and retrieve gradient components.
  /// @param[in] img Input grayscale image.
  /// @param[out] gx Output X-gradient image.
  /// @param[out] gy Output Y-gradient image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) {
    process(img);
    directionals(gx, gy);
  }

  /// @brief Process image and retrieve gradients and magnitude.
  /// @param[in] img Input grayscale image.
  /// @param[out] gx Output X-gradient image.
  /// @param[out] gy Output Y-gradient image.
  /// @param[out] mag Output magnitude image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
  }

  /// @brief Process image and retrieve all gradient outputs.
  /// @param[in] img Input grayscale image.
  /// @param[out] gx Output X-gradient image.
  /// @param[out] gy Output Y-gradient image.
  /// @param[out] mag Output magnitude image.
  /// @param[out] dir Output direction image.
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
    gx = dx_;
    gy = dy_;
  }

  /// @brief Get X-gradient image.
  /// @return X-gradient component image.
  cv::Mat gx() const { return dx_; }

  /// @brief Get Y-gradient image.
  /// @return Y-gradient component image.
  cv::Mat gy() const { return dy_; }

  /// @brief Get gradient magnitude image.
  /// @return Magnitude image computed from SUSAN response.
  cv::Mat magnitude() const { return mag_; }

  /// @brief Check if direction has been computed.
  /// @return True if direction() has been called, false otherwise.
  inline bool isDirectionDone() const { return dir_done_; }

  /// @brief Get gradient direction image.
  ///
  /// Direction is computed lazily on first access from the gradient components.
  /// @return Direction image in range determined by DO::range().
  cv::Mat direction() const {
    if (!dir_done_) {
      DO::process(dx_, dy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /// @brief Get direction value range.
  /// @return Range of direction values (e.g., [-π, π], [0, 2π], or [0, 360]).
  DirectionRange directionRange() const { return DO::range(); }

  /// @brief Get magnitude value range.
  /// @return Range of possible magnitude values.
  MagnitudeRange magnitudeRange() const {
    return MagnitudeRange(0, small_kernel_ ? static_cast<MT>(static_cast<int>(max_no_ * 0.277)) : max_no_);
  }

  /// @brief Get gradient value range.
  /// @return Range of possible gradient component values.
  GradientRange gradientRange() const {
    GT val = small_kernel_ ? 6 * this->intensityRange().upper : 26 * this->intensityRange().upper;
    return GradientRange(-val, val);
  }


  /// @brief Get the name of this gradient operator.
  /// @return String "susan".
  inline std::string name() const { return "susan"; }
};
}  // namespace lsfm
