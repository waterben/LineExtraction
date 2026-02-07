//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file laplace.hpp
/// @brief Abstract interface for Laplacian filters.

#pragma once

#include "opencv2/imgproc/imgproc.hpp"
#include <imgproc/filter.hpp>


namespace lsfm {

/// @brief Abstract interface for Laplacian filters.
///
/// Provides interface for computing second-order derivatives (Laplacian)
/// used for blob detection and edge enhancement.
/// @tparam IT Input image pixel type (uchar, short, float, double)
/// @tparam LT Laplacian output type (int, float, or double)
template <class IT, class LT>
class LaplaceI : public FilterI<IT> {
  LaplaceI(const LaplaceI&);

 public:
  typedef IT img_type;               ///< Input image pixel type
  typedef Range<IT> IntensityRange;  ///< Range type for intensity values
  typedef LT laplace_type;           ///< Laplacian output type
  typedef Range<LT> LaplaceRange;    ///< Range type for Laplacian values

  LaplaceI() {}
  virtual ~LaplaceI() {}

  /// @brief Get the Laplacian response image.
  /// @return Matrix containing Laplacian values at each pixel
  virtual cv::Mat laplace() const = 0;

  /// @brief Get the expected Laplacian value range.
  /// @return Range of possible Laplacian values
  virtual LaplaceRange laplaceRange() const = 0;

  /// @brief Convert normalized threshold [0,1] to Laplacian threshold.
  /// @param val Normalized threshold value in range [0,1]
  /// @return Corresponding Laplacian threshold value
  virtual LT laplaceThreshold(double val) const { return static_cast<LT>(laplaceRange().size() * val); }
};

/// @brief Base implementation class for Laplacian filters.
///
/// Provides common functionality including intensity range management.
/// @tparam IT Input image pixel type
/// @tparam LT Laplacian output type
template <class IT, class LT>
class Laplace : public LaplaceI<IT, LT> {
 protected:
  Range<IT> intRange_;  ///< Input intensity range

 public:
  typedef IT img_type;               ///< Input image pixel type
  typedef Range<IT> IntensityRange;  ///< Range type for intensity values
  typedef LT laplace_type;           ///< Laplacian output type
  typedef Range<LT> LaplaceRange;    ///< Range type for Laplacian values

  /// @brief Construct with intensity range.
  /// @param int_lower Lower bound of input intensity (default: type minimum)
  /// @param int_upper Upper bound of input intensity (default: type maximum)
  Laplace(IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
      : intRange_(int_lower, int_upper) {}

  /// @brief Get the expected input image intensity range.
  /// @return The intensity range for single-channel input images
  IntensityRange intensityRange() const { return intRange_; }

  /// @brief Get all filter outputs as named results.
  /// @return Map containing "laplace" output with range
  virtual FilterResults results() const {
    FilterResults ret;
    ret["laplace"] = FilterData(this->laplace(), this->laplaceRange());
    return ret;
  }
};

/// @brief Simple 3x3 Laplacian filter.
///
/// Uses standard 3x3 kernel with all neighbors weighted 1 and center -8.
/// Fast but sensitive to noise.
/// @tparam IT Input image pixel type
/// @tparam LT Laplacian output type
template <class IT, class LT>
class LaplaceSimple : public Laplace<IT, LT> {
 protected:
  cv::Mat laplace_;  ///< Cached Laplacian result
  cv::Mat_<LT> k_;   ///< Convolution kernel
  cv::Point anchor;  ///< Kernel anchor point
  using Laplace<IT, LT>::intRange_;

 public:
  typedef IT img_type;               ///< Input image pixel type
  typedef Range<IT> IntensityRange;  ///< Range type for intensity values
  typedef LT laplace_type;           ///< Laplacian output type
  typedef Range<LT> LaplaceRange;    ///< Range type for Laplacian values

  /// @brief Construct simple Laplacian filter.
  /// @param int_lower Lower bound of input intensity (default: type minimum)
  /// @param int_upper Upper bound of input intensity (default: type maximum)
  LaplaceSimple(IT int_lower = std::numeric_limits<IT>::lowest(), IT int_upper = std::numeric_limits<IT>::max())
      : Laplace<IT, LT>(int_lower, int_upper), laplace_(), k_(cv::Mat_<LT>::ones(3, 3)), anchor(-1, -1) {
    k_(1, 1) = -8;
  }

  /// @brief Process image through Laplacian filter.
  /// @param img Input grayscale image
  void process(const cv::Mat& img) {
    cv::filter2D(img, laplace_, cv::DataType<LT>::type, k_, anchor, 0, cv::BORDER_REFLECT_101);
  }

  /// @brief Get the Laplacian response image.
  /// @return Cached Laplacian result
  cv::Mat laplace() const { return laplace_; }

  /// @brief Get the Laplacian value range.
  /// @return Range based on intensity range * 8
  virtual LaplaceRange laplaceRange() const {
    LT val = intRange_.upper * 8;
    return LaplaceRange(-val, val);
  }

  /// @brief Get filter name.
  /// @return "laplace"
  std::string name() const { return "laplace"; }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using Laplace<IT, LT>::intensityRange;
  using Laplace<IT, LT>::results;
};

/// @brief Laplacian of Gaussian (LoG) filter.
///
/// Combines Gaussian smoothing with Laplacian for noise-robust blob detection.
/// Also known as "Mexican hat" filter due to its 2D shape.
/// @tparam IT Input image pixel type
/// @tparam LT Laplacian output type
template <class IT, class LT>
class LoG : public LaplaceSimple<IT, LT> {
  int ksize_;
  double kspace_, kscale_;
  using LaplaceSimple<IT, LT>::intRange_;
  Range<LT> laplaceRange_;

 public:
  /// @brief Compute LoG kernel value at position (x, y).
  /// @param x X coordinate relative to center
  /// @param y Y coordinate relative to center
  /// @param s Scale factor
  /// @return LoG value at (x, y)
  static inline double exp_d2(double x, double y, double s) {
    double xy2 = x * x + y * y;
    return s * (xy2 - 1) * std::exp(-xy2);
  }

  /// @brief Create LoG filter kernel.
  /// @param width Kernel width (will be expanded to full size)
  /// @param spacing Spacing between sample points
  /// @param scale Scale factor for kernel values
  /// @return NxN LoG kernel matrix
  static inline cv::Mat_<LT> createFilter(int width, double spacing, double scale) {
    width = width / 2;
    cv::Mat_<LT> kernel(width * 2 + 1, width * 2 + 1);
    for (int i = -width; i <= width; ++i)
      for (int j = -width; j <= width; ++j)
        kernel(j + width, i + width) = static_cast<LT>(exp_d2(i * spacing, j * spacing, scale));
        // zero dc
#ifndef DISABLE_DC_ZERO_FIX
    kernel -= cv::sum(kernel)[0] / (kernel.size().area());
#endif

    return kernel;
  }

 private:
  void create_kernel() {
    this->k_ = createFilter(ksize_, kspace_, kscale_);
    cv::Mat_<LT> tmp;
    this->k_.copyTo(tmp);
    tmp.setTo(0, tmp < 0);
    laplaceRange_.upper = static_cast<LT>(cv::sum(tmp)[0] * intRange_.upper);
    this->k_.copyTo(tmp);
    tmp.setTo(0, tmp > 0);
    laplaceRange_.lower = static_cast<LT>(cv::sum(tmp)[0] * intRange_.upper);
    // std::cout << this->k_ << std::endl << laplaceRange_.upper << std::endl << laplaceRange_.lower << std::endl;
  }


 public:
  typedef IT img_type;               ///< Input image pixel type
  typedef Range<IT> IntensityRange;  ///< Range type for intensity values
  typedef LT laplace_type;           ///< Laplacian output type
  typedef Range<LT> LaplaceRange;    ///< Range type for Laplacian values

  /// @brief Construct Laplacian of Gaussian filter.
  /// @param kernel_size Kernel size (default: 5)
  /// @param kernel_spacing Spacing between samples (default: 1.008)
  /// @param kernel_scale Scale factor for kernel values (default: 1)
  /// @param int_lower Lower bound of input intensity (default: type minimum)
  /// @param int_upper Upper bound of input intensity (default: type maximum)
  LoG(int kernel_size = 5,
      double kernel_spacing = 1.008,
      double kernel_scale = 1,
      IT int_lower = std::numeric_limits<IT>::lowest(),
      IT int_upper = std::numeric_limits<IT>::max())
      : LaplaceSimple<IT, LT>(int_lower, int_upper),
        ksize_(kernel_size),
        kspace_(kernel_spacing),
        kscale_(kernel_scale),
        laplaceRange_() {
    this->add("grad_kernel_size", std::bind(&LoG<IT, LT>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for LoG-Operator.");
    this->add("grad_kernel_spacing", std::bind(&LoG<IT, LT>::valueKernelSpacing, this, std::placeholders::_1),
              "Spacing for a single step for LoG-Operator.");
    this->add("grad_kernel_scale", std::bind(&LoG<IT, LT>::valueKernelScale, this, std::placeholders::_1),
              "Upscale for LoG-Operator (e.g. for converting to short).");

    create_kernel();
  }

  /// @brief Value accessor for kernel size option.
  /// @param ks New kernel size value (optional)
  /// @return Current kernel size
  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks);
    return ksize_;
  }

  /// @brief Get kernel size.
  /// @return Current kernel size
  int kernelSize() const { return ksize_; }

  /// @brief Set kernel size.
  ///
  /// Range 3-99, must be odd (even values corrected to ksize+1).
  /// @note Large kernels need larger GT type like int or long long int
  /// @param ks New kernel size
  void kernelSize(int ks) {
    if (ks == ksize_) return;

    if (ks < 3) ks = 3;
    if (ks % 2 == 0) ++ks;
    if (ks > 99) ks = 99;
    ksize_ = ks;
    create_kernel();
  }

  /// @brief Value accessor for kernel spacing option.
  /// @param ks New kernel spacing value (optional)
  /// @return Current kernel spacing
  Value valueKernelSpacing(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSpacing(ks);
    return kspace_;
  }

  /// @brief Get kernel spacing.
  /// @return Current kernel spacing between sample points
  double kernelSpacing() const { return kspace_; }

  /// @brief Set kernel spacing between sample points.
  /// @param ks New kernel spacing (must be > 0)
  void kernelSpacing(double ks) {
    if (ks == kspace_ || ks <= 0) return;
    kspace_ = ks;
    create_kernel();
  }

  /// @brief Value accessor for kernel scale option.
  /// @param ks New kernel scale value (optional)
  /// @return Current kernel scale
  Value valueKernelScale(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelScale(ks);
    return kscale_;
  }

  /// @brief Get kernel scale factor.
  /// @return Current kernel scale
  double kernelScale() const { return kscale_; }

  /// @brief Set kernel scale factor.
  /// @param ks New kernel scale (must be > 0)
  void kernelScale(double ks) {
    if (ks == kscale_ || ks <= 0) return;
    kscale_ = ks;
    create_kernel();
  }

  /// @brief Get the LoG convolution kernel.
  /// @return The current kernel matrix
  cv::Mat kernel() const { return this->k_; }

  /// @brief Get even filter response (alias for laplace).
  /// @return Laplacian response image
  cv::Mat even() const { return this->laplace(); }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using LaplaceSimple<IT, LT>::process;
  using LaplaceSimple<IT, LT>::laplace;
  using LaplaceSimple<IT, LT>::intensityRange;
  using LaplaceSimple<IT, LT>::results;

  /// @brief Get the Laplacian value range.
  /// @return Computed range based on kernel and intensity
  virtual LaplaceRange laplaceRange() const { return laplaceRange_; }

  /// @brief Get filter name.
  /// @return "LoG"
  std::string name() const { return "LoG"; }
};

/// @brief OpenCV Laplacian filter wrapper.
///
/// Uses cv::Laplacian internally with configurable kernel size.
/// @tparam IT Input image pixel type
/// @tparam LT Laplacian output type
template <class IT, class LT>
class LaplaceCV : public Laplace<IT, LT> {
  Range<LT> laplaceRange_;  ///< Computed Laplacian range
  cv::Mat laplace_;         ///< Cached Laplacian result
  int ksize_;               ///< Kernel size

  /// @brief Recalculate Laplacian range based on kernel size.
  void calc_range() {
    // TODO
    laplaceRange_.lower = static_cast<LT>(-this->intRange_.upper * 2 * ksize_);
    laplaceRange_.upper = static_cast<LT>(this->intRange_.upper * 2 * ksize_);
  }

 public:
  typedef IT img_type;               ///< Input image pixel type
  typedef Range<IT> IntensityRange;  ///< Range type for intensity values
  typedef LT laplace_type;           ///< Laplacian output type
  typedef Range<LT> LaplaceRange;    ///< Range type for Laplacian values

  /// @brief Construct OpenCV Laplacian filter.
  /// @param ksize Kernel size (default: 5)
  /// @param int_lower Lower bound of input intensity (default: type minimum)
  /// @param int_upper Upper bound of input intensity (default: type maximum)
  LaplaceCV(int ksize = 5,
            IT int_lower = std::numeric_limits<IT>::lowest(),
            IT int_upper = std::numeric_limits<IT>::max())
      : Laplace<IT, LT>(int_lower, int_upper), laplaceRange_(), laplace_(), ksize_(ksize) {
    this->add("grad_kernel_size", std::bind(&LaplaceCV<IT, LT>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for Laplace-Operator.");
    calc_range();
  }

  /// @brief Value accessor for kernel size option.
  /// @param ks New kernel size value (optional)
  /// @return Current kernel size
  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks.getInt());
    return ksize_;
  }

  /// @brief Get kernel size.
  /// @return Current kernel size
  int kernelSize() const { return ksize_; }

  /// @brief Set kernel size.
  ///
  /// Range 1-31, must be odd (even values corrected to ksize+1).
  /// @note Large kernels need larger GT type like float or double
  /// @param ks New kernel size
  void kernelSize(int ks) {
    if (ksize_ == ks) return;
    if (ks < 1) ks = 1;
    if (ks % 2 == 0) ++ks;
    if (ks > 31) ks = 31;
    ksize_ = ks;
    calc_range();
  }

  /// @brief Process image through Laplacian filter.
  /// @param img Input grayscale image
  void process(const cv::Mat& img) {
    cv::Laplacian(img, laplace_, cv::DataType<LT>::type, ksize_, 1, 0, cv::BORDER_REFLECT_101);
  }

  /// @brief Get the Laplacian response image.
  /// @return Cached Laplacian result
  cv::Mat laplace() const { return laplace_; }

  /// @brief Get the Laplacian value range.
  /// @return Computed range based on kernel size and intensity
  LaplaceRange laplaceRange() const { return laplaceRange_; }

  /// @brief Get filter name.
  /// @return "laplaceCV"
  std::string name() const { return "laplaceCV"; }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using Laplace<IT, LT>::intensityRange;
  using Laplace<IT, LT>::results;
};

}  // namespace lsfm
