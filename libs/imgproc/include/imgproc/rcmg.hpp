//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file rcmg.hpp
/// @brief Robust Colour Morphological Gradient (RCMG) edge detector.
///
/// Wrapper around the RCMG core algorithm (rcmg_core.hpp) providing the
/// Gradient<> interface, option management, and lazy direction computation.
/// The core algorithm implementation is by Adrian N. Evans, University of
/// Bath (2005-2007) and resides in third-party/contrib/imgproc_external
/// under its original copyright.
/// @see A.N. Evans and X.U. Liu, "A Morphological Gradient Approach to Color
///      Edge Detection", IEEE Trans. Image Processing, 15(6), pp. 1454-1463, 2006.

#pragma once

#include <imgproc/gradient.hpp>

#include <rcmg_core.hpp>

namespace lsfm {

/// @brief Robust Colour Morphological Gradient (RCMG) edge detector.
///
/// Computes edges in multi-channel images using the morphological gradient
/// approach. The algorithm finds the median-centered difference within a
/// sliding window, rejecting s pairs of pixels that are furthest apart in
/// color space to achieve noise robustness.
/// The gradient direction is derived from the spatial positions of the
/// maximum-distance pixel pair within the window.
///
/// This class provides the Gradient<> framework interface; the core algorithm
/// is implemented in rcmg_impl::RCMGCore (see rcmg_core.hpp).
/// @tparam IT Input image type (e.g., uchar).
/// @tparam channels Number of color channels (default: 3 for RGB/BGR).
/// @tparam GT Gradient component type (default: short).
/// @tparam MT Magnitude type (default: int for storing squared norms).
/// @tparam DT Direction type (default: float).
/// @tparam DO Direction operator type (default: Direction<GT, DT>).
template <class IT = uchar,
          int channels = 3,
          class GT = short,
          class MT = int,
          class DT = float,
          class DO = Direction<GT, DT>>
class RCMGradient : public Gradient<IT, GT, MT, DT> {
  int mask_;  ///< Window size (e.g., 3 for 3x3).
  int s_;     ///< Number of vector pairs to reject.
  int norm_;  ///< Norm type (cv::NORM_L1, cv::NORM_L2, cv::NORM_L2SQR).

  cv::Mat_<MT> mag_;  ///< Computed magnitude image.
  cv::Mat_<GT> gx_;   ///< Computed X-gradient image.
  cv::Mat_<GT> gy_;   ///< Computed Y-gradient image.

  mutable cv::Mat_<DT> dir_;  ///< Computed direction image (lazy).
  mutable bool dir_done_;     ///< Direction computation flag.

  rcmg_impl::RCMGCore<IT, channels, GT, MT> core_;  ///< Core algorithm implementation.

 public:
  typedef IT img_type;   ///< Input image pixel type.
  typedef GT grad_type;  ///< Gradient component type.
  typedef MT mag_type;   ///< Magnitude type.
  typedef DT dir_type;   ///< Direction type.

  typedef Range<GT> GradientRange;   ///< Range type for gradient values.
  typedef Range<MT> MagnitudeRange;  ///< Range type for magnitude values.
  typedef Range<DT> DirectionRange;  ///< Range type for direction values.

  /// @brief Construct an RCMG gradient operator.
  /// @param mask Window size for computing differences (default: 3, meaning 3x3).
  /// @param s Number of vector pairs to reject for noise reduction (default: 0).
  /// @param cn Norm type: cv::NORM_L1, cv::NORM_L2, cv::NORM_L2SQR (default: L2SQR).
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  RCMGradient(int mask = 3,
              int s = 0,
              int cn = cv::NORM_L2SQR,
              IT int_lower = std::numeric_limits<IT>::lowest(),
              IT int_upper = std::numeric_limits<IT>::max())
      : Gradient<IT, GT, MT, DT>(int_lower, int_upper),
        mask_(mask < 3 ? 3 : mask),
        s_(s),
        norm_(cn),
        mag_(),
        gx_(),
        gy_(),
        dir_(),
        dir_done_(false),
        core_() {
    this->add("grad_mask_size",
              std::bind(&RCMGradient<IT, channels, GT, MT, DT, DO>::maskSize, this, std::placeholders::_1),
              "Window size for differences.");
    this->add("grad_rejection",
              std::bind(&RCMGradient<IT, channels, GT, MT, DT, DO>::rejection, this, std::placeholders::_1),
              "Vector rejection for noise compensation.");
    this->add("grad_color_norm",
              std::bind(&RCMGradient<IT, channels, GT, MT, DT, DO>::colorNorm, this, std::placeholders::_1),
              "Norm type: L1 or L2.");
  }

  /// @brief Get or set the window (mask) size.
  /// @param ms Optional new mask size (must be >= 3).
  /// @return Current mask size.
  Value maskSize(const Value& ms = Value::NAV()) {
    if (ms.type()) {
      int m = ms;
      mask_ = (m < 3) ? 3 : m;
    }
    return mask_;
  }

  /// @brief Get or set the number of rejected vector pairs.
  /// @param r Optional new rejection count.
  /// @return Current rejection count.
  Value rejection(const Value& r = Value::NAV()) {
    if (r.type()) s_ = r;
    return s_;
  }

  /// @brief Get or set the color norm type.
  /// @param nt Optional new norm type (cv::NORM_L1, cv::NORM_L2, cv::NORM_L2SQR).
  /// @return Current norm type.
  Value colorNorm(const Value& nt = Value::NAV()) {
    if (nt.type()) norm_ = nt;
    return norm_;
  }


  /// @brief Process an image to compute RCMG edge gradients.
  ///
  /// Computes gradient magnitude and components using the morphological
  /// gradient algorithm. Results can be retrieved using magnitude(), gx(), gy().
  /// @param img Input multi-channel image.
  void process(const cv::Mat& img) {
    dir_done_ = false;
    core_.compute(img, mag_, gx_, gy_, mask_, s_, norm_);
  }

  /// @brief Process image and retrieve gradient components and magnitude.
  /// @param img Input multi-channel image.
  /// @param gx Output X-gradient image.
  /// @param gy Output Y-gradient image.
  /// @param mag Output magnitude image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
  }

  /// @brief Process image and retrieve all gradient outputs.
  /// @param img Input multi-channel image.
  /// @param gx Output X-gradient image.
  /// @param gy Output Y-gradient image.
  /// @param mag Output magnitude image.
  /// @param dir Output direction image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag, cv::Mat& dir) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
    dir = direction();
  }

  /// @brief Get both gradient components.
  /// @param gx X-gradient image reference.
  /// @param gy Y-gradient image reference.
  void directionals(cv::Mat& gx, cv::Mat& gy) const {
    gx = gx_;
    gy = gy_;
  }

  /// @brief Get X-gradient image.
  /// @return X-gradient component image.
  cv::Mat gx() const { return gx_; }

  /// @brief Get Y-gradient image.
  /// @return Y-gradient component image.
  cv::Mat gy() const { return gy_; }

  /// @brief Get gradient value range.
  /// @return Range of possible gradient component values.
  GradientRange gradientRange() const { return GradientRange(static_cast<GT>(-mask_), static_cast<GT>(mask_)); }

  /// @brief Get gradient magnitude image.
  /// @return Magnitude image.
  cv::Mat magnitude() const { return mag_; }

  /// @brief Check if direction has been computed.
  /// @return True if direction() has been called, false otherwise.
  inline bool isDirectionDone() const { return dir_done_; }

  /// @brief Get gradient direction image.
  ///
  /// Direction is computed lazily on first access.
  /// @return Direction image in range determined by DO::range().
  cv::Mat direction() const {
    if (!dir_done_) {
      DO::process(gx_, gy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /// @brief Get direction value range.
  /// @return Range of direction values.
  DirectionRange directionRange() const { return DO::range(); }

  /// @brief Get magnitude value range.
  /// @return Range of possible magnitude values.
  MagnitudeRange magnitudeRange() const {
    return MagnitudeRange(0, static_cast<MT>(norm(cv::Vec<IT, channels>::all(this->intRange_.upper), norm_)));
  }


  /// @brief Get the name of this gradient operator.
  /// @return String "rcmg".
  inline std::string name() const { return "rcmg"; }

 private:
  /// @brief Compute norm of a cv::Matx for magnitude range computation.
  template <typename T, int m, int n>
  static inline double norm(const cv::Matx<T, m, n>& M, int normType) {
    return normType == cv::NORM_INF
               ? static_cast<double>(cv::normInf<T, typename cv::DataType<T>::work_type>(M.val, m * n))
           : normType == cv::NORM_L1
               ? static_cast<double>(cv::normL1<T, typename cv::DataType<T>::work_type>(M.val, m * n))
           : normType == cv::NORM_L2SQR
               ? static_cast<double>(cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(M.val, m * n))
               : std::sqrt(static_cast<double>(cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(M.val, m * n)));
  }
};

/// @brief Single-channel specialization of RCMG gradient operator.
///
/// Specialized implementation of the RCMG algorithm for grayscale images.
/// Uses scalar intensity differences instead of vector color differences.
///
/// This class provides the Gradient<> framework interface; the core algorithm
/// is implemented in rcmg_impl::RCMGCore (see rcmg_core.hpp).
/// @tparam IT Input image type (e.g., uchar).
/// @tparam GT Gradient component type (default: short).
/// @tparam MT Magnitude type (default: int).
/// @tparam DT Direction type (default: float).
/// @tparam DO Direction operator type.
template <class IT, class GT, class MT, class DT, class DO>
class RCMGradient<IT, 1, GT, MT, DT, DO> : public Gradient<IT, GT, MT, DT> {
  int mask_;  ///< Window size.
  int s_;     ///< Number of pairs to reject.
  int norm_;  ///< Norm type.

  cv::Mat_<MT> mag_;  ///< Computed magnitude image.
  cv::Mat_<GT> gx_;   ///< Computed X-gradient image.
  cv::Mat_<GT> gy_;   ///< Computed Y-gradient image.

  mutable cv::Mat_<DT> dir_;  ///< Computed direction image (lazy).
  mutable bool dir_done_;     ///< Direction computation flag.

  rcmg_impl::RCMGCore<IT, 1, GT, MT> core_;  ///< Core algorithm implementation.

 public:
  typedef IT img_type;   ///< Input image pixel type.
  typedef GT grad_type;  ///< Gradient component type.
  typedef MT mag_type;   ///< Magnitude type.
  typedef DT dir_type;   ///< Direction type.

  typedef Range<GT> GradientRange;   ///< Range type for gradient values.
  typedef Range<MT> MagnitudeRange;  ///< Range type for magnitude values.
  typedef Range<DT> DirectionRange;  ///< Range type for direction values.

  /// @brief Construct a single-channel RCMG gradient operator.
  /// @param mask Window size for computing differences (default: 3).
  /// @param s Number of pairs to reject for noise reduction (default: 0).
  /// @param nt Norm type (default: cv::NORM_L2SQR).
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  RCMGradient(int mask = 3,
              int s = 0,
              int nt = cv::NORM_L2SQR,
              IT int_lower = std::numeric_limits<IT>::lowest(),
              IT int_upper = std::numeric_limits<IT>::max())
      : Gradient<IT, GT, MT, DT>(int_lower, int_upper),
        mask_(mask < 3 ? 3 : mask),
        s_(s),
        norm_(nt),
        mag_(),
        gx_(),
        gy_(),
        dir_(),
        dir_done_(false),
        core_() {
    this->add("grad_mask_size", std::bind(&RCMGradient<IT, 1, GT, MT, DT, DO>::maskSize, this, std::placeholders::_1),
              "Window size for differences.");
    this->add("grad_rejection", std::bind(&RCMGradient<IT, 1, GT, MT, DT, DO>::rejection, this, std::placeholders::_1),
              "Vector rejection for noise compensation.");
    this->add("grad_norm_type",
              std::bind(&RCMGradient<IT, 1, GT, MT, DT, DO>::normTypeOption, this, std::placeholders::_1),
              "Norm type: L1 or L2.");
  }

  /// @brief Get or set the window (mask) size.
  /// @param ms Optional new mask size (must be >= 3).
  /// @return Current mask size.
  Value maskSize(const Value& ms = Value::NAV()) {
    if (ms.type()) {
      int m = ms;
      mask_ = (m < 3) ? 3 : m;
    }
    return mask_;
  }

  /// @brief Get or set the number of rejected vector pairs.
  /// @param r Optional new rejection count.
  /// @return Current rejection count.
  Value rejection(const Value& r = Value::NAV()) {
    if (r.type()) s_ = r;
    return s_;
  }

  /// @brief Get or set the norm type.
  /// @param nt Optional new norm type (cv::NORM_L1, cv::NORM_L2, cv::NORM_L2SQR).
  /// @return Current norm type.
  Value normTypeOption(const Value& nt = Value::NAV()) {
    if (nt.type()) norm_ = nt;
    return norm_;
  }


  /// @brief Process an image to compute RCMG edge gradients.
  /// @param img Input single-channel image.
  void process(const cv::Mat& img) {
    dir_done_ = false;
    core_.compute(img, mag_, gx_, gy_, mask_, s_, norm_);
  }

  /// @brief Process image and retrieve gradient components and magnitude.
  /// @param img Input single-channel image.
  /// @param gx Output X-gradient image.
  /// @param gy Output Y-gradient image.
  /// @param mag Output magnitude image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
  }

  /// @brief Process image and retrieve all gradient outputs.
  /// @param img Input single-channel image.
  /// @param gx Output X-gradient image.
  /// @param gy Output Y-gradient image.
  /// @param mag Output magnitude image.
  /// @param dir Output direction image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag, cv::Mat& dir) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
    dir = direction();
  }

  /// @brief Get both gradient components.
  /// @param gx X-gradient image reference.
  /// @param gy Y-gradient image reference.
  void directionals(cv::Mat& gx, cv::Mat& gy) const {
    gx = gx_;
    gy = gy_;
  }

  /// @brief Get X-gradient image.
  /// @return X-gradient component image.
  cv::Mat gx() const { return gx_; }

  /// @brief Get Y-gradient image.
  /// @return Y-gradient component image.
  cv::Mat gy() const { return gy_; }

  /// @brief Get gradient value range.
  /// @return Range of possible gradient component values.
  GradientRange gradientRange() const { return GradientRange(static_cast<GT>(-mask_), static_cast<GT>(mask_)); }

  /// @brief Get gradient magnitude image.
  /// @return Magnitude image.
  cv::Mat magnitude() const { return mag_; }

  /// @brief Check if direction has been computed.
  /// @return True if direction() has been called, false otherwise.
  inline bool isDirectionDone() const { return dir_done_; }

  /// @brief Get gradient direction image.
  /// @return Direction image in range determined by DO::range().
  cv::Mat direction() const {
    if (!dir_done_) {
      DO::process(gx_, gy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /// @brief Get direction value range.
  /// @return Range of direction values.
  DirectionRange directionRange() const { return DO::range(); }

  /// @brief Get magnitude value range.
  /// @return Range of possible magnitude values.
  MagnitudeRange magnitudeRange() const { return MagnitudeRange(0, this->intRange_.upper); }


  /// @brief Get the name of this gradient operator.
  /// @return String "rcmg".
  inline std::string name() const { return "rcmg"; }
};
}  // namespace lsfm
