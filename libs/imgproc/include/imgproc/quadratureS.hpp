/// @file quadratureS.hpp
/// @brief Spherical quadrature filter using difference of Poisson kernels.
/// This file implements a quadrature filter based on the difference of
/// Poisson (DoP) kernels. The Poisson kernel and its derivatives form
/// a quadrature pair with good localization properties.

#pragma once

#include <imgproc/derivative.hpp>
#include <imgproc/polar.hpp>
#include <imgproc/quadrature.hpp>
#include <opencv2/core/core.hpp>

namespace lsfm {

/// @brief Spherical quadrature filter using difference of Poisson kernels.
/// QuadratureS implements a quadrature filter pair using the difference of
/// Poisson (DoP) kernels. The even (symmetric) filter uses dop() and the
/// odd (antisymmetric) filter uses docp().
/// Key features:
/// - 2D non-separable filter kernels
/// - Configurable scale and bandwidth via muls parameter
/// - Supports scale-space analysis with multiple scales
/// @tparam IT Input image type (default: uchar).
/// @tparam GT Gradient type (default: float).
/// @tparam FT Floating-point type for computations (default: float).
/// @tparam P Polar coordinate class template (default: Polar).
/// @see Quadrature Base class providing interface.
template <class IT = uchar, class GT = float, class FT = float, template <typename, typename> class P = Polar>
class QuadratureS : public Quadrature<IT, GT, FT, FT, FT> {
 public:
  /// @brief Difference of Poisson kernel (center-antisymmetric, odd).
  /// Computes the odd quadrature filter kernel value at position (x,y).
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @param s Scale parameter.
  /// @param m Scale multiplier for second kernel.
  /// @return Kernel value at (x,y).
  static inline FT docp(FT x, FT y, FT s, FT m) {
    FT a = (s * s + x * x + y * y), b = (m * m * s * s + x * x + y * y);
    return x / (2 * static_cast<FT>(CV_PI) * std::sqrt(a * a * a)) -
           x / (2 * static_cast<FT>(CV_PI) * std::sqrt(b * b * b));
  }

  /// @brief Difference of Poisson kernel (center-symmetric, even).
  /// Computes the even quadrature filter kernel value at position (x,y).
  /// @param x X coordinate.
  /// @param y Y coordinate.
  /// @param s Scale parameter.
  /// @param m Scale multiplier for second kernel.
  /// @return Kernel value at (x,y).
  static inline FT dop(FT x, FT y, FT s, FT m) {
    FT a = (s * s + x * x + y * y), b = (m * m * s * s + x * x + y * y);
    return s / (2 * static_cast<FT>(CV_PI) * std::sqrt(a * a * a)) -
           (m * s) / (2 * static_cast<FT>(CV_PI) * std::sqrt(b * b * b));
  }

  /// Function pointer type for kernel generation.
  typedef FT (*KernelType)(FT x, FT y, FT s, FT m);

  /// @brief Create a 2D filter kernel from a kernel function.
  /// @param width Kernel width (will be centered).
  /// @param spacing Sample spacing between kernel elements.
  /// @param s Scale parameter.
  /// @param m Scale multiplier.
  /// @param f Kernel function to evaluate.
  /// @return 2D kernel matrix.
  static cv::Mat_<FT> createFilter(int width, FT spacing, FT s, FT m, KernelType f) {
    width = width / 2;
    cv::Mat_<FT> kernel(width * 2 + 1, width * 2 + 1);
    for (int i = -width; i <= width; ++i)
      for (int j = -width; j <= width; ++j)
        kernel(j + width, i + width) = f(static_cast<FT>(i) * spacing, static_cast<FT>(j) * spacing, s, m);

    return kernel;
  }

 protected:
  cv::Point anchor;  ///< Filter anchor point.

  cv::Mat kox_;  ///< Odd X kernel.
  cv::Mat koy_;  ///< Odd Y kernel.
  cv::Mat ke_;   ///< Even kernel.
  cv::Mat img_;  ///< Cached input image.

  mutable cv::Mat o_;       ///< Cached odd magnitude.
  mutable cv::Mat phase_;   ///< Cached phase image.
  mutable cv::Mat dir_;     ///< Cached direction image.
  mutable cv::Mat ox_;      ///< Cached odd X response.
  mutable cv::Mat oy_;      ///< Cached odd Y response.
  mutable cv::Mat e_;       ///< Cached even response.
  mutable cv::Mat energy_;  ///< Cached energy image.

  Range<GT> evenRange_;    ///< Range of even filter values.
  Range<FT> energyRange_;  ///< Range of energy values.
  DerivativeMax<GT> gm_;   ///< Derivative max response info.

  int ksize_;    ///< Kernel size.
  FT kspacing_;  ///< Kernel spacing.
  FT kscale_;    ///< Kernel scale factor.
  FT scale_;     ///< Filter scale.
  FT muls_;      ///< Scale multiplier.

  mutable bool odd_done_;     ///< Flag: odd magnitude computed.
  mutable bool energy_done_;  ///< Flag: energy computed.
  mutable bool dir_done_;     ///< Flag: direction computed.
  mutable bool phase_done_;   ///< Flag: phase computed.
  mutable bool even_done_;    ///< Flag: even computed.

  /// @brief Compute maximum filter response ranges.
  void max_response() {
    cv::Mat_<IT> tmp(2 * ksize_, 2 * ksize_);
    tmp.setTo(this->intRange_.lower);
    tmp.colRange(0, ksize_).setTo(this->intRange_.upper);
    process(tmp);
    energyRange_.lower = 0;
    double vmin, vmax;
    cv::minMaxIdx(odd(), &vmin, &vmax);
    energyRange_.upper = static_cast<FT>(vmax);
    cv::minMaxIdx(cv::abs(even()), &vmin, &vmax);
    evenRange_.upper = static_cast<GT>(vmax);
    evenRange_.lower = -evenRange_.upper;
  }

  /// @brief Create all filter kernels.
  /// Creates the odd X, odd Y, and even kernels using the configured
  /// scale and spacing parameters.
  virtual void create_kernels() {
    kox_ = createFilter(ksize_, kspacing_, scale_, muls_, docp) * kscale_;
    ke_ = createFilter(ksize_, kspacing_, scale_, muls_, dop) * kscale_;

    if (cv::DataType<GT>::type != cv::DataType<FT>::type) {
      kox_.convertTo(kox_, cv::DataType<GT>::type);
      ke_.convertTo(ke_, cv::DataType<GT>::type);
    }

    // zero dc
#ifndef DISABLE_DC_ZERO_FIX
    // std::cout << "dc fix (" << ksize_ << "): " << cv::sum(ke_)[0] / (ksize_*ksize_) << std::endl;
    ke_ -= cv::sum(ke_)[0] / (ksize_ * ksize_);
#endif

    koy_ = kox_.t();


    gm_.max_1st = std::abs(static_cast<GT>(cv::sum(kox_.colRange(0, ksize_ / 2))[0]));
    gm_.max_2nd = gm_.max_3rd = 0;
    for (int i = 0; i != ksize_ / 2; ++i) {
      for (int j = i; j < ksize_ - i - 1; ++j) gm_.max_2nd += std::abs(kox_.at<GT>(j, i));
    }
    for (int i = 0; i != ksize_ / 2; ++i) gm_.max_3rd += std::abs(kox_.at<GT>(ksize_ / 2 + 1, i));

    max_response();
  }

  /// @brief Initialize option manager entries.
  void init() {
    this->add("grad_kernel_size", std::bind(&QuadratureS<IT, GT, FT, P>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for Quadrature-Operators.");
    this->add("grad_kernel_spacing",
              std::bind(&QuadratureS<IT, GT, FT, P>::valueKernelSpacing, this, std::placeholders::_1),
              "Spacing for a single step for Quadrature-Operators.");
    this->add("grad_kernel_scale",
              std::bind(&QuadratureS<IT, GT, FT, P>::valueKernelScale, this, std::placeholders::_1),
              "Upscale for Quadrature-Operators (e.g. for converting to short).");
    this->add("grad_scale", std::bind(&QuadratureS<IT, GT, FT, P>::valueScale, this, std::placeholders::_1),
              "Coarsest scale for filter.");
    this->add("grad_muls", std::bind(&QuadratureS<IT, GT, FT, P>::valueMuls, this, std::placeholders::_1),
              "Multiply for coarsed scale.");

    this->create_kernels();
  }

 public:
  typedef IT img_type;     ///< Input image element type.
  typedef GT grad_type;    ///< Gradient element type.
  typedef FT mag_type;     ///< Magnitude element type.
  typedef FT energy_type;  ///< Energy element type.
  typedef FT dir_type;     ///< Direction element type.
  typedef FT phase_type;   ///< Phase element type.

  typedef Range<IT> IntensityRange;  ///< Intensity value range type.
  typedef Range<GT> GradientRange;   ///< Gradient value range type.
  typedef Range<FT> MagnitudeRange;  ///< Magnitude value range type.
  typedef Range<FT> EnergyRange;     ///< Energy value range type.
  typedef Range<FT> DirectionRange;  ///< Direction value range type.
  typedef Range<FT> PhaseRange;      ///< Phase value range type.

  /// @brief Construct a spherical quadrature filter.
  /// @param scale Filter scale (default: 1).
  /// @param muls Scale multiplier for bandwidth (default: 3).
  /// @param kernel_size Kernel size (default: 5, must be odd).
  /// @param kernel_spacing Spacing between samples (default: 1).
  /// @param kernel_scale Overall kernel scale factor (default: 1).
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureS(FT scale = 1,
              FT muls = 3,
              int kernel_size = 5,
              FT kernel_spacing = 1,
              FT kernel_scale = 1,
              IT int_lower = std::numeric_limits<IT>::lowest(),
              IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        anchor(-1, -1),
        kox_(),
        koy_(),
        ke_(),
        img_(),
        o_(),
        phase_(),
        dir_(),
        ox_(),
        oy_(),
        e_(),
        energy_(),
        evenRange_(),
        energyRange_(),
        gm_(),
        ksize_(kernel_size),
        kspacing_(kernel_spacing),
        kscale_(kernel_scale),
        scale_(scale),
        muls_(muls),
        odd_done_(false),
        energy_done_(false),
        dir_done_(false),
        phase_done_(false),
        even_done_(false) {
    init();
  }

  /// @brief Construct from scale-space parameters.
  /// Computes scale and muls from a base scale and scale ratio.
  /// @param scale Base scale.
  /// @param l Scale ratio (lambda).
  /// @param k Scale index.
  /// @param kernel_size Kernel size.
  /// @param kernel_spacing Spacing between samples.
  /// @param kernel_scale Overall kernel scale factor.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureS(FT scale,
              FT l,
              int k,
              int kernel_size,
              FT kernel_spacing,
              FT kernel_scale,
              IT int_lower = std::numeric_limits<IT>::lowest(),
              IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        anchor(-1, -1),
        kox_(),
        koy_(),
        ke_(),
        img_(),
        o_(),
        phase_(),
        dir_(),
        ox_(),
        oy_(),
        e_(),
        energy_(),
        evenRange_(),
        energyRange_(),
        gm_(),
        ksize_(kernel_size),
        kspacing_(kernel_spacing),
        kscale_(kernel_scale),
        scale_(scale * pow(l, k)),
        muls_(pow(l, k - 1) / pow(l, k)),
        odd_done_(false),
        energy_done_(false),
        dir_done_(false),
        phase_done_(false),
        even_done_(false) {
    init();
  }

  /// @brief Construct from a name-value option vector.
  /// @param options Configuration options.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureS(const ValueManager::NameValueVector& options,
              img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        anchor(-1, -1),
        kox_(),
        koy_(),
        ke_(),
        img_(),
        o_(),
        phase_(),
        dir_(),
        ox_(),
        oy_(),
        e_(),
        energy_(),
        evenRange_(),
        energyRange_(),
        gm_(),
        ksize_(5),
        kspacing_(1),
        kscale_(1),
        scale_(1),
        muls_(2),
        odd_done_(false),
        energy_done_(false),
        dir_done_(false),
        phase_done_(false),
        even_done_(false) {
    init();
    this->value(options);
  }

  /// @brief Construct from an initializer list of options.
  /// @param options Configuration options as initializer list.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureS(ValueManager::InitializerList options,
              img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        anchor(-1, -1),
        kox_(),
        koy_(),
        ke_(),
        img_(),
        o_(),
        phase_(),
        dir_(),
        ox_(),
        oy_(),
        e_(),
        energy_(),
        evenRange_(),
        energyRange_(),
        gm_(),
        ksize_(5),
        kspacing_(1),
        kscale_(1),
        scale_(1),
        muls_(2),
        odd_done_(false),
        energy_done_(false),
        dir_done_(false),
        phase_done_(false),
        even_done_(false) {
    init();
    this->value(options);
  }

  /// @brief Get or set kernel size through ValueManager interface.
  /// @param ks Optional new kernel size value.
  /// @return Current kernel size.
  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks);
    return ksize_;
  }

  /// @brief Get the kernel size.
  /// @return Current kernel size.
  int kernelSize() const { return ksize_; }

  /// @brief Set the kernel size.
  /// The kernel size must be odd and in range [3, 99].
  /// Even values are automatically incremented.
  /// @param ks New kernel size.
  /// @note Large kernels may require larger gradient types.
  void kernelSize(int ks) {
    if (ks == ksize_) return;

    if (ks < 3) ks = 3;
    if (ks % 2 == 0) ++ks;
    if (ks > 99) ks = 99;
    ksize_ = ks;
    create_kernels();
  }

  /// @brief Get or set kernel spacing through ValueManager interface.
  /// @param ks Optional new kernel spacing value.
  /// @return Current kernel spacing.
  Value valueKernelSpacing(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSpacing(ks);
    return kspacing_;
  }

  /// @brief Get the kernel spacing.
  /// @return Current kernel spacing.
  FT kernelSpacing() const { return kspacing_; }

  /// @brief Set the kernel spacing.
  /// @param ks New kernel spacing (must be positive).
  void kernelSpacing(FT ks) {
    if (ks == kspacing_ || ks <= 0) return;
    kspacing_ = ks;
    create_kernels();
  }

  /// @brief Get or set kernel scale through ValueManager interface.
  /// @param ks Optional new kernel scale value.
  /// @return Current kernel scale.
  Value valueKernelScale(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelScale(ks);
    return kscale_;
  }

  /// @brief Get the kernel scale factor.
  /// @return Current kernel scale.
  FT kernelScale() const { return kscale_; }

  /// @brief Set the kernel scale factor.
  /// @param ks New kernel scale (must be positive).
  void kernelScale(FT ks) {
    if (ks == kscale_ || ks <= 0) return;
    kscale_ = ks;
    create_kernels();
  }

  /// @brief Get or set filter scale through ValueManager interface.
  /// @param s Optional new scale value.
  /// @return Current scale.
  Value valueScale(const Value& s = Value::NAV()) {
    if (s.type()) scale(s);
    return scale_;
  }

  /// @brief Get the filter scale.
  /// @return Current scale.
  FT scale() const { return scale_; }

  /// @brief Set the filter scale.
  /// @param s New scale (must be positive).
  void scale(FT s) {
    if (s == scale_ || s <= 0) return;
    scale_ = s;
    create_kernels();
  }

  /// @brief Get or set scale multiplier through ValueManager interface.
  /// @param m Optional new muls value.
  /// @return Current muls.
  Value valueMuls(const Value& m = Value::NAV()) {
    if (m.type()) muls(m);
    return muls_;
  }

  /// @brief Get the scale multiplier.
  /// @return Current muls value.
  FT muls() const { return muls_; }

  /// @brief Set the scale multiplier.
  /// @param m New muls value (must be positive).
  void muls(FT m) {
    if (m == muls_ || m <= 0) return;
    muls_ = m;
    create_kernels();
  }

  /// @brief Set scale and muls from scale-space parameters.
  /// @param s Base scale.
  /// @param l Scale ratio (lambda).
  /// @param k Scale index.
  void muls(FT s, FT l, int k) {
    if (k <= 0 || l <= 0 || s <= 0) return;
    scale_ = s * std::pow(l, k);
    muls_ = std::pow(l, k - 1) / std::pow(l, k);
    create_kernels();
  }

  /// @brief Get the even filter kernel.
  /// @return Even kernel matrix.
  cv::Mat kernel() const { return ke_; }

  /// @brief Process an image to compute quadrature filter responses.
  /// Applies the odd X and Y filters to the image. Even response is
  /// computed lazily when accessed via even().
  /// @param[in] img Input image.
  void process(const cv::Mat& img) {
    odd_done_ = false;
    dir_done_ = false;
    even_done_ = false;
    phase_done_ = false;
    energy_done_ = false;

    img_ = img;

    // compute the outputs
    cv::filter2D(img, ox_, cv::DataType<GT>::type, kox_, anchor, 0, cv::BORDER_REFLECT_101);
    cv::filter2D(img, oy_, cv::DataType<GT>::type, koy_, anchor, 0, cv::BORDER_REFLECT_101);
  }

  /// @brief Get X and Y components of odd filter response.
  /// @param[out] gx X component (odd X).
  /// @param[out] gy Y component (odd Y).
  void odd(cv::Mat& gx, cv::Mat& gy) const {
    gx = ox_;
    gy = oy_;
  }

  /// @brief Check if even response has been computed.
  /// @return True if even() has been called, false otherwise.
  inline bool isEvenDone() const { return even_done_; }

  /// @brief Get the even filter response.
  /// Computed lazily from the cached input image.
  /// @return Even filter response image.
  inline cv::Mat even() const {
    if (!even_done_) {
      cv::filter2D(img_, e_, cv::DataType<GT>::type, ke_, anchor, 0, cv::BORDER_REFLECT_101);
      even_done_ = true;
    }
    return e_;
  }

  /// @brief Get the even filter response range.
  /// @return Range of even filter values.
  GradientRange evenRange() const { return evenRange_; }

  /// @brief Check if odd magnitude has been computed.
  /// @return True if odd() has been called, false otherwise.
  inline bool isOddDone() const { return odd_done_; }

  /// @brief Get the odd filter response magnitude.
  /// Computes magnitude from X/Y components if not already computed.
  /// @return Odd filter response magnitude image.
  inline cv::Mat odd() const {
    if (!odd_done_) {
      P<GT, FT>::magnitude(ox_, oy_, o_);
      odd_done_ = true;
    }
    return o_;
  }

  /// @brief Get the odd filter response range.
  /// @return Range of odd filter magnitude values.
  MagnitudeRange oddRange() const { return Range<FT>(0, magnitudeMaxStep(this->intRange_.upper)); }

  /// @brief Convert normalized threshold to odd magnitude threshold.
  /// @param[in] val Normalized threshold value in [0, 1].
  /// @return Absolute odd magnitude threshold.
  FT oddThreshold(double val) const { return oddRange().upper * Magnitude<FT, FT>::single(static_cast<FT>(val)); }

  /// @brief Check if energy has been computed.
  /// @return True if energy() has been called, false otherwise.
  inline bool isEnergyDone() const { return energy_done_; }

  /// @brief Get the local energy.
  /// Energy is computed as the magnitude of the complex
  /// quadrature response (even + i*odd).
  /// @return Energy image.
  inline cv::Mat energy() const {
    if (!energy_done_) {
      P<GT, FT>::magnitude(odd(), even(), energy_);
      energy_done_ = true;
    }
    return energy_;
  }

  /// @brief Get the energy value range.
  /// @return Range of energy values.
  inline EnergyRange energyRange() const { return energyRange_; }

  /// @brief Check if direction has been computed.
  /// @return True if direction() has been called, false otherwise.
  inline bool isDirectionDone() const { return dir_done_; }

  /// @brief Get the direction image.
  /// Direction is computed from the odd X/Y components.
  /// @return Direction image.
  cv::Mat direction() const {
    if (!dir_done_) {
      P<GT, FT>::phase(ox_, oy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /// @brief Get the direction value range.
  /// @return Range of direction values from Polar class.
  DirectionRange directionRange() const { return P<GT, FT>::range(); }

  /// @brief Check if phase has been computed.
  /// @return True if phase() has been called, false otherwise.
  inline bool isPhaseDone() const { return phase_done_; }

  /// @brief Get the local phase.
  /// Phase is the angle of the complex quadrature response.
  /// It indicates the type of feature (edge, line, etc.).
  /// @return Phase image.
  cv::Mat phase() const {
    if (!phase_done_) {
      P<FT, FT>::phase(odd(), even(), phase_);
      phase_done_ = true;
    }
    return phase_;
  }

  /// @brief Get filtered phase with near-zero values set to zero.
  /// Useful for avoiding phase noise in low-energy regions.
  /// @return Filtered phase image.
  cv::Mat phaseF() const {
    cv::Mat o = odd(), e = even(), ret;
    o.setTo(0, o < 0.001);
    e.setTo(0, e<0.001 & e> - 0.001);
    P<FT, FT>::phase(o, e, ret);
    return ret;
  }

  /// @brief Get the phase value range.
  /// @return Range of phase values from Polar class.
  PhaseRange phaseRange() const { return P<FT, FT>::range(); }

  /// @brief Get maximum magnitude for a given intensity step.
  /// @param[in] intensity Intensity step value (default: 1).
  /// @return Maximum magnitude value.
  FT magnitudeMaxStep(IT intensity = 1) const { return Magnitude<GT, FT>::max(derivativeMax(), intensity); }

  /// @brief Compute single-value magnitude normalization factor.
  /// @param[in] val Input value.
  /// @return Normalized magnitude value.
  FT magnitudeSingle(FT val) const { return Magnitude<FT, FT>::single(val); }

  /// @brief Get derivative operator maximum response information.
  /// @return DerivativeMax structure with max response values.
  DerivativeMax<GT> derivativeMax() const { return gm_; }

  /// @brief Get the name of this gradient operator.
  /// @return "quadratureS".
  inline std::string name() const { return "quadratureS"; }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using Quadrature<IT, GT, FT, FT, FT>::intensityRange;
  using Quadrature<IT, GT, FT, FT, FT>::results;
  using Quadrature<IT, GT, FT, FT, FT>::evenThreshold;
  using Quadrature<IT, GT, FT, FT, FT>::oddThreshold;
  using Quadrature<IT, GT, FT, FT, FT>::oddx;
  using Quadrature<IT, GT, FT, FT, FT>::oddy;
  using Quadrature<IT, GT, FT, FT, FT>::oddGradRange;
  using Quadrature<IT, GT, FT, FT, FT>::normType;
  using Quadrature<IT, GT, FT, FT, FT>::energyThreshold;
};

}  // namespace lsfm
