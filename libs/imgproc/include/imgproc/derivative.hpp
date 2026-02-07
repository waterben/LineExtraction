//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file derivative.hpp
/// @brief Derivative operators for image gradient computation.

#pragma once

#include <imgproc/gaussian.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/value_manager.hpp>


namespace lsfm {

/// @brief Maximum derivative values for unit intensity change.
///
/// Stores the maximum possible derivative response values for a given operator
/// when the intensity changes by 1. Used for normalization and thresholding.
/// @tparam GT Gradient/derivative type (short, float, or double)
template <class GT = short>
struct DerivativeMax {
  typedef GT grad_type;  ///< Gradient value type

  /// @brief Construct with maximum values.
  /// @param mf Maximum when derivative is in primary direction only
  /// @param ms Maximum when both directions contribute equally
  /// @param mt Maximum for secondary direction when primary is at max
  DerivativeMax(grad_type mf = 0, grad_type ms = 0, grad_type mt = 0) : max_1st(mf), max_2nd(ms), max_3rd(mt) {}

  grad_type max_1st;  ///< Max value for primary direction only
  grad_type max_2nd;  ///< Max value when both directions equal
  grad_type max_3rd;  ///< Max for secondary when primary is max
};

/// @brief Abstract base class for derivative operators.
///
/// Provides interface for computing image derivatives (gradients) using
/// various convolution kernels (Sobel, Scharr, Prewitt, etc.).
/// @tparam IT Input image pixel type (uchar for 8-bit, short for 16-bit)
/// @tparam GT Gradient output type (short for 8-bit input, float for 16-bit)
template <class IT = uchar, class GT = short>
struct Derivative {
  /// @brief Compute X and Y derivatives from input image.
  /// @param img Input grayscale image
  /// @param[out] gx Output X-direction derivative
  /// @param[out] gy Output Y-direction derivative
  /// @note 8-bit images produce short output, 16/32-bit produce float output
  virtual void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const = 0;

  /// @brief Get maximum derivative values for unit intensity change.
  /// @return DerivativeMax containing max values for normalization
  virtual DerivativeMax<GT> max() const = 0;

  /// @brief Get the name of this derivative operator.
  /// @return String identifier (e.g., "sobel", "scharr", "prewitt")
  virtual std::string name() const = 0;

  virtual ~Derivative() = default;
};


/// @brief Roberts cross derivative operator.
///
/// Computes diagonal derivatives using 2x2 cross-difference kernels.
/// Fast but sensitive to noise due to small kernel size.
/// @tparam IT Input image pixel type
/// @tparam GT Gradient output type (default: short)
template <class IT, class GT = short>
class RobertsDerivative : public Derivative<IT, GT> {
  cv::Mat kx, ky;
  mutable cv::Mat da, bc;
  cv::Point anchor;

 public:
  typedef IT img_type;   ///< Input image pixel type
  typedef GT grad_type;  ///< Gradient output type

  /// @brief Construct Roberts derivative operator.
  ///
  /// Initializes 2x2 diagonal difference kernels.
  RobertsDerivative() : kx(), ky(), da(), bc(), anchor(0, 0) {
    kx.create(2, 2, cv::DataType<GT>::type);
    kx.at<GT>(0, 0) = -1;
    kx.at<GT>(1, 1) = 1;
    kx.at<GT>(0, 1) = 0;
    kx.at<GT>(1, 0) = 0;

    ky.create(2, 2, cv::DataType<GT>::type);
    ky.at<GT>(0, 0) = 0;
    ky.at<GT>(1, 1) = 0;
    ky.at<GT>(0, 1) = 1;
    ky.at<GT>(1, 0) = -1;
  }

  /// @brief Compute Roberts cross derivatives.
  /// @param img Input grayscale image
  /// @param[out] gx Output X-direction derivative
  /// @param[out] gy Output Y-direction derivative
  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    cv::filter2D(img, da, cv::DataType<GT>::type, kx, anchor, 0, cv::BORDER_REFLECT_101);
    cv::filter2D(img, bc, cv::DataType<GT>::type, ky, anchor, 0, cv::BORDER_REFLECT_101);

    gx = da + bc;
    gy = da - bc;
  }


  /// @brief Get maximum derivative values.
  /// @return DerivativeMax with values (2, 1, 0)
  DerivativeMax<GT> max() const { return DerivativeMax<GT>(2, 1, 0); }

  /// @brief Get operator name.
  /// @return "roberts"
  std::string name() const { return "roberts"; }
};

/// @brief Prewitt derivative operator.
///
/// Uses 3x3 separable kernel with uniform smoothing perpendicular to
/// the derivative direction. Simple and fast with moderate noise reduction.
/// @tparam IT Input image pixel type
/// @tparam GT Gradient output type (default: short)
template <class IT, class GT = short>
class PrewittDerivative : public Derivative<IT, GT> {
  cv::Mat kx, ky;
  cv::Point anchor;

 public:
  typedef IT img_type;   ///< Input image pixel type
  typedef GT grad_type;  ///< Gradient output type

  /// @brief Construct Prewitt derivative operator.
  ///
  /// Initializes 3x3 separable kernels with uniform weights.
  PrewittDerivative() : kx(), ky(), anchor(-1, -1) {
    kx.create(3, 1, cv::DataType<GT>::type);
    kx.at<GT>(0) = -1;
    kx.at<GT>(1) = 0;
    kx.at<GT>(2) = 1;
    ky.create(3, 1, cv::DataType<GT>::type);
    ky.at<GT>(0) = 1;
    ky.at<GT>(1) = 1;
    ky.at<GT>(2) = 1;
  }

  /// @brief Compute Prewitt derivatives.
  /// @param img Input grayscale image
  /// @param[out] gx Output X-direction derivative
  /// @param[out] gy Output Y-direction derivative
  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    // std::cout << "prewitt: " << kx << "; " << ky << std::endl;
    cv::sepFilter2D(img, gx, cv::DataType<GT>::type, kx, ky, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, gy, cv::DataType<GT>::type, ky, kx, anchor, 0, cv::BORDER_REFLECT_101);
  }

  /// @brief Get maximum derivative values.
  /// @return DerivativeMax with values (3, 2, 1)
  DerivativeMax<GT> max() const { return DerivativeMax<GT>(3, 2, 1); }

  /// @brief Get operator name.
  /// @return "prewitt"
  std::string name() const { return "prewitt"; }
};


/// @brief Sobel derivative operator with configurable kernel size.
///
/// Uses separable Gaussian-weighted kernels for noise-robust gradient estimation.
/// The most commonly used derivative operator with good balance of accuracy and
/// noise reduction. Supports kernel sizes from 3 to 31.
/// @tparam IT Input image pixel type (default: uchar)
/// @tparam GT Gradient output type (default: short)
template <class IT = uchar, class GT = short>
class SobelDerivative : public Derivative<IT, GT>, public ValueManager {
  cv::Mat kx, ky;
  DerivativeMax<GT> gm;
  cv::Point anchor;
  int ksize_;

 public:
  typedef IT img_type;   ///< Input image pixel type
  typedef GT grad_type;  ///< Gradient output type

  /// @brief Construct Sobel derivative operator.
  /// @param kernel_size Kernel size (3-31, must be odd, default: 3)
  SobelDerivative(int kernel_size = 3) : kx(), ky(), gm(4, 3, 2), anchor(-1, -1), ksize_(0) {
    this->add("grad_kernel_size", std::bind(&SobelDerivative<IT, GT>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for Sobel-Operator.");
    kernelSize(kernel_size);
  }

  /// @brief Value accessor for kernel size (for ValueManager).
  /// @param ks New kernel size value (optional)
  /// @return Current kernel size
  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks.getInt());
    return ksize_;
  }

  /// @brief Get current kernel size.
  /// @return Kernel size (3, 5, 7, ..., 31)
  int kernelSize() const { return ksize_; }


  /// @brief Set kernel size.
  ///
  /// Larger kernels provide more noise reduction but less localization.
  /// @param ks New kernel size (3-31, odd values only, even values rounded up)
  /// @note Large kernels require larger GT type (float or double)
  void kernelSize(int ks) {
    if (ksize_ == ks) return;
    if (ks < 3) ks = 3;
    if (ks % 2 == 0) ++ks;
    if (ks > 31) ks = 31;

    cv::getDerivKernels(kx, ky, 1, 0, ks, false, std::max<int>(CV_32F, cv::DataType<GT>::type));
    kx.convertTo(kx, cv::DataType<GT>::type);
    ky.convertTo(ky, cv::DataType<GT>::type);
    GT s = static_cast<GT>(sum(ky)[0]);
    GT c = ky.at<GT>(ks / 2);
    gm.max_1st = gm.max_2nd = gm.max_3rd = 0;
    for (int i = ks / 2 + 1; i != ks; ++i) {
      GT v = kx.at<GT>(i);
      gm.max_1st += static_cast<typename DerivativeMax<GT>::grad_type>(s * v);
      gm.max_3rd += static_cast<typename DerivativeMax<GT>::grad_type>(c * v);
    }
    for (int i = 0; i != ks / 2; ++i) {
      GT sc = static_cast<GT>(std::abs(kx.at<GT>(i)));
      for (int j = i; j < ks - i - 1; ++j)
        gm.max_2nd += static_cast<typename DerivativeMax<GT>::grad_type>(ky.at<GT>(j) * sc);
    }
    ksize_ = ks;
  }

  /// @brief Compute Sobel derivatives.
  /// @param img Input grayscale image
  /// @param[out] gx Output X-direction derivative
  /// @param[out] gy Output Y-direction derivative
  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    // std::cout << "sobel: " << kx << "; " << ky << std::endl;
    cv::sepFilter2D(img, gx, cv::DataType<GT>::type, kx, ky, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, gy, cv::DataType<GT>::type, ky, kx, anchor, 0, cv::BORDER_REFLECT_101);
  }

  /// @brief Get maximum derivative values.
  /// @return DerivativeMax computed for current kernel size
  DerivativeMax<GT> max() const { return gm; }

  /// @brief Get operator name.
  /// @return "sobel"
  std::string name() const { return "sobel"; }
};

/// @brief Scharr derivative operator.
///
/// Optimized 3x3 kernel providing more rotational symmetry than Sobel.
/// Better approximation to true image gradient at the cost of fixed kernel size.
/// @tparam IT Input image pixel type (default: uchar)
/// @tparam GT Gradient output type (default: short)
template <class IT = uchar, class GT = short>
class ScharrDerivative : public Derivative<IT, GT> {
  cv::Mat kx, ky;
  cv::Point anchor;

 public:
  typedef IT img_type;   ///< Input image pixel type
  typedef GT grad_type;  ///< Gradient output type

  /// @brief Construct Scharr derivative operator.
  ///
  /// Uses optimized 3x3 kernel with weights [-3, 0, 3] and [3, 10, 3].
  ScharrDerivative() : kx(), ky(), anchor(-1, -1) {
    cv::getDerivKernels(kx, ky, 1, 0, -1, false, std::max<int>(CV_32F, cv::DataType<GT>::type));
    kx.convertTo(kx, cv::DataType<GT>::type);
    ky.convertTo(ky, cv::DataType<GT>::type);
  }

  /// @brief Compute Scharr derivatives.
  /// @param img Input grayscale image
  /// @param[out] gx Output X-direction derivative
  /// @param[out] gy Output Y-direction derivative
  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    // std::cout << "scharr: " << kx << "; " << ky << std::endl;
    cv::sepFilter2D(img, gx, cv::DataType<GT>::type, kx, ky, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, gy, cv::DataType<GT>::type, ky, kx, anchor, 0, cv::BORDER_REFLECT_101);
  }

  /// @brief Get maximum derivative values.
  /// @return DerivativeMax with values (16, 13, 10)
  DerivativeMax<GT> max() const { return DerivativeMax<GT>(16, 13, 10); }

  /// @brief Get operator name.
  /// @return "scharr"
  std::string name() const { return "scharr"; }
};


/// @brief Gaussian derivative operator with scale-space support.
///
/// Computes first derivative of Gaussian for scale-space edge detection.
/// Configurable kernel size, range (sigma-equivalent), and scale factor.
/// @tparam IT Input image pixel type (default: uchar)
/// @tparam GT Gradient output type (default: short)
template <class IT = uchar, class GT = short>
class GaussianDerivative : public Derivative<IT, GT>, public ValueManager {
  cv::Mat kx, ky;
  DerivativeMax<GT> gm;
  cv::Point anchor;
  int ksize_;
  double range_, scale_;

  /// @brief Create Gaussian derivative kernels.
  /// @param ksize Kernel size
  /// @param range Value range (inversely related to sigma)
  /// @param scale Scaling factor for kernel values
  void create_kernel(int ksize, double range, double scale) {
    kx = gaussianD1<double>(ksize, range);
    kx *= std::sqrt(scale);
    cv::Mat kx2 = kx.t();
    ky = gaussian<double>(ksize, range).t() * sqrt(scale);
    // std::cout << kx * ky.t() << std::endl;

    kx.convertTo(kx, cv::DataType<GT>::type);
    ky.convertTo(ky, cv::DataType<GT>::type);
    // std::cout << "gauss: " << kx << "; " << ky << std::endl;

    GT s = static_cast<GT>(sum(ky)[0]);
    GT c = ky.at<GT>(ksize / 2);
    gm.max_1st = gm.max_2nd = gm.max_3rd = 0;
    for (int i = ksize / 2 + 1; i != ksize; ++i) {
      GT v = kx.at<GT>(i);
      gm.max_1st += static_cast<typename DerivativeMax<GT>::grad_type>(s * v);
      gm.max_3rd += static_cast<typename DerivativeMax<GT>::grad_type>(c * v);
    }
    for (int i = 0; i != ksize / 2; ++i) {
      GT sc = static_cast<GT>(std::abs(kx.at<GT>(i)));
      for (int j = i; j < ksize - i - 1; ++j)
        gm.max_2nd += static_cast<typename DerivativeMax<GT>::grad_type>(ky.at<GT>(j) * sc);
    }

    // std::cout << gm.max_1st << " " << gm.max_2nd << " " << gm.max_3rd << std::endl;
  }

 public:
  typedef IT img_type;   ///< Input image pixel type
  typedef GT grad_type;  ///< Gradient output type

  /// @brief Construct Gaussian derivative operator.
  /// @param kernel_size Kernel size (3-99, must be odd, default: 5)
  /// @param range Value range parameter (larger = narrower Gaussian, default: 3)
  /// @param scale Scaling factor for output (default: 1)
  GaussianDerivative(int kernel_size = 5, double range = 3, double scale = 1)
      : kx(), ky(), gm(0, 0, 0), anchor(-1, -1), ksize_(0), range_(range), scale_(scale) {
    CV_Assert(kernel_size > 2 && range > 0 && scale > 0);

    this->add("grad_kernel_size", std::bind(&GaussianDerivative<IT, GT>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for Gaussian-Operator.");
    this->add("grad_range", std::bind(&GaussianDerivative<IT, GT>::valueRange, this, std::placeholders::_1),
              "Parameter range for Gaussian-Operator.");
    this->add("grad_scale", std::bind(&GaussianDerivative<IT, GT>::valueScale, this, std::placeholders::_1),
              "Upscale for Gaussian-Operator (max value in operator == upscale).");
    kernelSize(kernel_size);
  }

  /// @brief Value accessor for kernel size (for ValueManager).
  /// @param ks New kernel size value (optional)
  /// @return Current kernel size
  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks);
    return ksize_;
  }

  /// @brief Get current kernel size.
  /// @return Kernel size (3, 5, 7, ..., 99)
  int kernelSize() const { return ksize_; }

  /// @brief Set kernel size.
  /// @param ks New kernel size (3-99, odd values only)
  /// @note Large kernels may require larger GT type (int or long long)
  void kernelSize(int ks) {
    if (ks == ksize_) return;

    if (ks < 3) ks = 3;
    if (ks % 2 == 0) ++ks;
    if (ks > 99) ks = 99;
    create_kernel(ks, range(), scale());
    ksize_ = ks;
  }


  /// @brief Value accessor for range (for ValueManager).
  /// @param r New range value (optional)
  /// @return Current range value
  Value valueRange(const Value& r = Value::NAV()) {
    if (r.type()) range(r);
    return range_;
  }

  /// @brief Get current range parameter.
  /// @return Range value (inversely related to sigma)
  double range() const { return range_; }

  /// @brief Set range parameter.
  ///
  /// Larger range values produce narrower Gaussians (smaller sigma equivalent).
  /// @param r New range value (must be > 0)
  void range(double r) {
    if (r <= 0 || r == range_) return;
    create_kernel(ksize_, r, scale_);
    range_ = r;
  }

  /// @brief Value accessor for scale (for ValueManager).
  /// @param s New scale value (optional)
  /// @return Current scale value
  Value valueScale(const Value& s = Value::NAV()) {
    if (s.type()) scale(s);
    return scale_;
  }

  /// @brief Get current scale factor.
  /// @return Scale multiplier for kernel values
  double scale() const { return scale_; }

  /// @brief Set scale factor.
  ///
  /// Scales the kernel values to control output magnitude range.
  /// @param sc New scale value (must be > 0)
  void scale(double sc) {
    if (sc <= 0 || sc == scale_) return;
    create_kernel(ksize_, range_, sc);
    scale_ = sc;
  }

  /// @brief Compute Gaussian derivatives.
  /// @param img Input grayscale image
  /// @param[out] gx Output X-direction derivative
  /// @param[out] gy Output Y-direction derivative
  void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy) const {
    cv::sepFilter2D(img, gx, cv::DataType<GT>::type, kx, ky, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, gy, cv::DataType<GT>::type, ky, kx, anchor, 0, cv::BORDER_REFLECT_101);
  }

  /// @brief Get maximum derivative values.
  /// @return DerivativeMax computed for current parameters
  DerivativeMax<GT> max() const { return gm; }

  /// @brief Get operator name.
  /// @return "gauss"
  std::string name() const { return "gauss"; }
};
}  // namespace lsfm
