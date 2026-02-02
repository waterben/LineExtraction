/// @file quadratureSF.hpp
/// @brief Spherical Poisson quadrature filter operating in frequency domain.
/// This file implements a quadrature filter based on the Poisson kernel
/// operating in the frequency domain using FFT. This is the frequency-domain
/// version of QuadratureS, providing the same functionality with FFT-based
/// computation.

#pragma once

#include <imgproc/polar.hpp>
#include <imgproc/quadrature.hpp>
#include <opencv2/core/core.hpp>
#include <utility/matlab_helpers.hpp>

namespace lsfm {

/// @brief Spherical Poisson quadrature filter operating in frequency domain.
/// QuadratureSF implements a quadrature filter pair using the difference of
/// Poisson kernels in the frequency domain. This is computationally efficient
/// for larger images and provides exact Poisson filter responses.
/// The odd filter uses docpf() (complex, antisymmetric) and the even filter
/// uses dopf() (real, symmetric).
/// Key features:
/// - Frequency domain computation using FFT
/// - Automatic optimal DFT size selection
/// - Configurable scale and bandwidth via muls parameter
/// @tparam IT Input image type (default: uchar).
/// @tparam FT Floating-point type for computations (default: float).
/// @tparam P Polar coordinate class template (default: Polar).
/// @see Quadrature Base class providing interface.
/// @see QuadratureS Spatial domain implementation.
template <class IT = uchar, class FT = float, template <typename, typename> class P = Polar>
class QuadratureSF : public Quadrature<IT, FT, FT, FT, FT> {
 public:
  /// Constant 2*pi.
  static constexpr FT PI2 = 2 * static_cast<FT>(CV_PI);

  /// @brief Difference of center-antisymmetric Poisson kernel in frequency domain.
  /// Computes the odd quadrature filter value at frequency (u,v).
  /// @param u Horizontal frequency.
  /// @param v Vertical frequency.
  /// @param s Scale parameter.
  /// @param m Scale multiplier for second kernel.
  /// @return Complex kernel value at (u,v).
  static inline std::complex<FT> docpf(FT u, FT v, FT s, FT m) {
    FT a = std::sqrt(u * u + v * v), b = -PI2 * a * s, c = std::exp(b) - std::exp(b * m);
    return std::complex<FT>(-u / a * c, v / a * c);
  }

  /// @brief Difference of Poisson kernel in frequency domain.
  /// Computes the even quadrature filter value at frequency (u,v).
  /// @param u Horizontal frequency.
  /// @param v Vertical frequency.
  /// @param s Scale parameter.
  /// @param m Scale multiplier for second kernel.
  /// @return Kernel value at (u,v).
  static inline FT dopf(FT u, FT v, FT s, FT m) {
    FT a = -PI2 * std::sqrt(u * u + v * v) * s;
    return std::exp(a) - std::exp(a * m);
  }

  /// Function pointer type for real kernel generation.
  typedef FT (*KernelType)(FT u, FT v, FT s, FT m);
  /// Function pointer type for complex kernel generation.
  typedef std::complex<FT> (*KernelTypeC)(FT u, FT v, FT s, FT m);

  /// @brief Create a real-valued filter in frequency domain.
  /// @param rows Number of rows.
  /// @param cols Number of columns.
  /// @param spacing Frequency spacing.
  /// @param s Scale parameter.
  /// @param m Scale multiplier.
  /// @param f Kernel function to evaluate.
  /// @return Real filter matrix.
  static cv::Mat_<FT> createFilter(int rows, int cols, FT spacing, FT s, FT m, KernelType f) {
    cv::Mat_<FT> kernel(rows, cols);
    FT step = spacing * static_cast<FT>(cols - 1);
    FT startx = static_cast<FT>(rows) / static_cast<FT>(2) - static_cast<FT>(0.5),
       starty = static_cast<FT>(cols) / static_cast<FT>(2) - static_cast<FT>(0.5);
    for (int i = 0; i != rows; ++i)
      for (int j = 0; j != cols; ++j)
        kernel(i, j) = f((static_cast<FT>(j) - starty) / step, (static_cast<FT>(i) - startx) / step, s, m);

    return kernel;
  }

  /// @brief Create a complex-valued filter in frequency domain.
  /// @param rows Number of rows.
  /// @param cols Number of columns.
  /// @param spacing Frequency spacing.
  /// @param s Scale parameter.
  /// @param m Scale multiplier.
  /// @param f Complex kernel function to evaluate.
  /// @return Complex filter matrix.
  static cv::Mat_<std::complex<FT>> createFilterC(int rows, int cols, FT spacing, FT s, FT m, KernelTypeC f) {
    cv::Mat_<std::complex<FT>> kernel(rows, cols);
    FT step = spacing * static_cast<FT>(cols - 1);
    FT startx = static_cast<FT>(rows) / 2 - static_cast<FT>(0.5),
       starty = static_cast<FT>(cols) / 2 - static_cast<FT>(0.5);
    for (int i = 0; i != rows; ++i)
      for (int j = 0; j != cols; ++j)
        kernel(i, j) = f((static_cast<FT>(j) - starty) / step, (static_cast<FT>(i) - startx) / step, s, m);
    return kernel;
  }

 protected:
  cv::Mat_<FT> fe_;                  ///< Even filter in frequency domain.
  cv::Mat_<std::complex<FT>> fo_;    ///< Odd filter in frequency domain.
  cv::Mat_<std::complex<FT>> imgf_;  ///< FFT of input image.

  mutable cv::Mat_<FT> o_;       ///< Cached odd magnitude.
  mutable cv::Mat_<FT> phase_;   ///< Cached phase image.
  mutable cv::Mat_<FT> dir_;     ///< Cached direction image.
  mutable cv::Mat_<FT> ox_;      ///< Cached odd X response.
  mutable cv::Mat_<FT> oy_;      ///< Cached odd Y response.
  mutable cv::Mat_<FT> e_;       ///< Cached even response.
  mutable cv::Mat_<FT> energy_;  ///< Cached energy image.

  int rows_;      ///< Original image rows.
  int cols_;      ///< Original image columns.
  int rows_ext_;  ///< Extended rows for optimal DFT size.
  int cols_ext_;  ///< Extended columns for optimal DFT size.

  Range<FT> energyRange_;  ///< Range of energy values.
  Range<FT> oddRange_;     ///< Range of odd filter values.
  Range<FT> evenRange_;    ///< Range of even filter values.

  using Quadrature<IT, FT, FT, FT, FT>::intRange_;

  FT kspacing_;  ///< Kernel spacing.
  FT scale_;     ///< Filter scale.
  FT muls_;      ///< Scale multiplier.

  mutable bool odd_done_;     ///< Flag: odd magnitude computed.
  mutable bool energy_done_;  ///< Flag: energy computed.
  mutable bool dir_done_;     ///< Flag: direction computed.
  mutable bool phase_done_;   ///< Flag: phase computed.
  mutable bool even_done_;    ///< Flag: even computed.
  mutable bool oddxy_done_;   ///< Flag: odd X/Y computed.

  /// @brief Compute maximum filter response ranges.
  void max_response() {
    cv::Mat_<IT> tmp(128, 128);
    tmp.setTo(this->intRange_.lower);
    tmp.colRange(0, 64).setTo(this->intRange_.upper);
    // force update
    updateFilter(tmp);
    process(tmp);
    double vmin, vmax;
    cv::minMaxIdx(energy(), &vmin, &vmax);
    energyRange_.upper = static_cast<FT>(vmax);
    cv::minMaxIdx(odd(), &vmin, &vmax);
    oddRange_.upper = static_cast<FT>(vmax);
    cv::minMaxIdx(cv::abs(even()), &vmin, &vmax);
    evenRange_.upper = static_cast<FT>(vmax);
    evenRange_.lower = -evenRange_.upper;
  }

  /// @brief Update filter kernels for new image size.
  /// @param img Image to match filter size to.
  /// @param force If true, always regenerate. If false, only if size changed.
  /// @return True if filter was updated, false otherwise.
  virtual bool updateFilter(const cv::Mat img, bool force = true) {
    // if force is of, only renew if image size has changed
    if (!force && img.rows == rows_ && img.cols == cols_) return false;

    rows_ = img.rows;
    cols_ = img.cols;
    rows_ext_ = cv::getOptimalDFTSize(img.rows);
    cols_ext_ = cv::getOptimalDFTSize(img.cols);

    fe_ = ifftshift(createFilter(rows_ext_, cols_ext_, kspacing_, scale_, muls_, dopf));
    // fe_.at<FT>(0, 0) = 0;
    fo_ = ifftshift(createFilterC(rows_ext_, cols_ext_, kspacing_, scale_, muls_, docpf));
    return true;
  }

  /// @brief Initialize option manager entries.
  void init() {
    this->add("grad_kernel_spacing",
              std::bind(&QuadratureSF<IT, FT, P>::valueKernelSpacing, this, std::placeholders::_1),
              "Spacing for a single step for Quadrature-Operators.");
    this->add("grad_scale", std::bind(&QuadratureSF<IT, FT, P>::valueScale, this, std::placeholders::_1),
              "Coarsest scale for filter.");
    this->add("grad_muls", std::bind(&QuadratureSF<IT, FT, P>::valueMuls, this, std::placeholders::_1),
              "Multiply for coarsed scale.");

    max_response();
  }


 public:
  typedef IT img_type;     ///< Input image element type.
  typedef FT grad_type;    ///< Gradient element type.
  typedef FT mag_type;     ///< Magnitude element type.
  typedef FT energy_type;  ///< Energy element type.
  typedef FT dir_type;     ///< Direction element type.
  typedef FT phase_type;   ///< Phase element type.

  typedef Range<IT> IntensityRange;  ///< Intensity value range type.
  typedef Range<FT> GradientRange;   ///< Gradient value range type.
  typedef Range<FT> MagnitudeRange;  ///< Magnitude value range type.
  typedef Range<FT> EnergyRange;     ///< Energy value range type.
  typedef Range<FT> DirectionRange;  ///< Direction value range type.
  typedef Range<FT> PhaseRange;      ///< Phase value range type.

  /// @brief Construct a frequency-domain spherical quadrature filter.
  /// @param scale Filter scale (default: 1).
  /// @param muls Scale multiplier for bandwidth (default: 2).
  /// @param kernel_spacing Frequency spacing (default: 1).
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureSF(FT scale = 1,
               FT muls = 2,
               FT kernel_spacing = 1,
               IT int_lower = std::numeric_limits<IT>::lowest(),
               IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        fe_(),
        fo_(),
        imgf_(),
        o_(),
        phase_(),
        dir_(),
        ox_(),
        oy_(),
        e_(),
        energy_(),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        energyRange_(),
        oddRange_(),
        evenRange_(),
        kspacing_(kernel_spacing),
        scale_(scale),
        muls_(muls),
        odd_done_(false),
        energy_done_(false),
        dir_done_(false),
        phase_done_(false),
        even_done_(false),
        oddxy_done_(false) {
    init();
  }

  /// @brief Construct from scale-space parameters.
  /// Computes scale and muls from a base scale and scale ratio.
  /// @param scale Base scale.
  /// @param l Scale ratio (lambda).
  /// @param k Scale index.
  /// @param kernel_spacing Frequency spacing.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureSF(FT scale,
               FT l,
               int k,
               FT kernel_spacing,
               IT int_lower = std::numeric_limits<IT>::lowest(),
               IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        fe_(),
        fo_(),
        imgf_(),
        o_(),
        phase_(),
        dir_(),
        ox_(),
        oy_(),
        e_(),
        energy_(),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        energyRange_(),
        oddRange_(),
        evenRange_(),
        kspacing_(kernel_spacing),
        scale_(scale * pow(l, k)),
        muls_(pow(l, k - 1) / pow(l, k)),
        odd_done_(false),
        energy_done_(false),
        dir_done_(false),
        phase_done_(false),
        even_done_(false),
        oddxy_done_(false) {
    init();
  }

  /// @brief Construct from a name-value option vector.
  /// @param options Configuration options.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureSF(const ValueManager::NameValueVector& options,
               img_type int_lower = std::numeric_limits<img_type>::lowest(),
               img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        fe_(),
        fo_(),
        imgf_(),
        o_(),
        phase_(),
        dir_(),
        ox_(),
        oy_(),
        e_(),
        energy_(),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        energyRange_(),
        oddRange_(),
        evenRange_(),
        kspacing_(1),
        scale_(1),
        muls_(2),
        odd_done_(false),
        energy_done_(false),
        dir_done_(false),
        phase_done_(false),
        even_done_(false),
        oddxy_done_(false) {
    init();
    value(options);
  }

  /// @brief Construct from an initializer list of options.
  /// @param options Configuration options as initializer list.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureSF(ValueManager::InitializerList options,
               img_type int_lower = std::numeric_limits<img_type>::lowest(),
               img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        fe_(),
        fo_(),
        imgf_(),
        o_(),
        phase_(),
        dir_(),
        ox_(),
        oy_(),
        e_(),
        energy_(),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        energyRange_(),
        oddRange_(),
        evenRange_(),
        kspacing_(1),
        scale_(1),
        muls_(2),
        odd_done_(false),
        energy_done_(false),
        dir_done_(false),
        phase_done_(false),
        even_done_(false),
        oddxy_done_(false) {
    init();
    value(options);
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
    max_response();
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
    max_response();
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
    max_response();
  }

  /// @brief Set scale and muls from scale-space parameters.
  /// @param s Base scale.
  /// @param l Scale ratio (lambda).
  /// @param k Scale index.
  void muls(FT s, FT l, int k) {
    if (k <= 0 || l <= 0 || s <= 0) return;
    scale_ = s * std::pow(l, k);
    muls_ = std::pow(l, k - 1) / std::pow(l, k);
    max_response();
  }

  /// @brief Process an image to compute quadrature filter responses.
  /// Transforms the image to frequency domain using FFT. Filter
  /// responses are computed lazily when accessed.
  /// @param[in] img Input image.
  virtual void process(const cv::Mat& img) {
    cv::Mat_<std::complex<FT>> tmpc;

    oddxy_done_ = false;
    odd_done_ = false;
    dir_done_ = false;
    even_done_ = false;
    phase_done_ = false;
    energy_done_ = false;

    // update filter, but only if image size has changed
    updateFilter(img, false);

    cv::Mat_<FT> src;
    if (img.type() != cv::DataType<FT>::type)
      img.convertTo(src, cv::DataType<FT>::type);
    else
      src = img;

    if (cols_ < cols_ext_ || rows_ < rows_ext_)
      cv::copyMakeBorder(src, src, 0, rows_ext_ - rows_, 0, cols_ext_ - cols_, cv::BORDER_REPLICATE);

#ifdef USE_PERIODIC_FFT  // slower, but removes artifacts
    imgf_ = perfft2(src);
#else
    imgf_ = fft2(src);
#endif
  }

  /// @brief Check if odd X/Y components have been computed.
  /// @return True if odd(gx, gy) has been called, false otherwise.
  inline bool isOddxyDone() const { return oddxy_done_; }

  /// @brief Get X and Y components of odd filter response.
  /// Computes response in frequency domain and transforms back.
  /// @param[out] gx X component.
  /// @param[out] gy Y component.
  void odd(cv::Mat& gx, cv::Mat& gy) const {
    if (!oddxy_done_) {
      cv::Mat_<std::complex<FT>> tmpc = ifft2(multiply(imgf_, fo_));

      if (cols_ < cols_ext_ || rows_ < rows_ext_) {
        tmpc.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
      }
      splitT(tmpc, oy_, ox_);
      oddxy_done_ = true;
    }
    gx = ox_;
    gy = oy_;
  }

  /// @brief Check if even filter response has been computed.
  /// @return True if even() has been called, false otherwise.
  inline bool isEvenDone() const { return even_done_; }

  /// @brief Get the even (symmetric) filter response.
  /// Computed lazily via FFT inverse transform.
  /// @return Even filter response image.
  inline cv::Mat even() const {
    if (!even_done_) {
      e_ = real(ifft2(multiply(imgf_, fe_)));
      if (cols_ < cols_ext_ || rows_ < rows_ext_) e_.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
      even_done_ = true;
    }
    return e_;
  }

  /// @brief Get the even response value range.
  /// @return Valid range for even response values.
  GradientRange evenRange() const { return evenRange_; }

  /// @brief Check if odd magnitude has been computed.
  /// @return True if odd() has been called, false otherwise.
  inline bool isOddDone() const { return odd_done_; }

  /// @brief Get the odd (antisymmetric) filter response magnitude.
  /// Computed as magnitude of X and Y odd components.
  /// @return Odd filter response magnitude image.
  inline cv::Mat odd() const {
    if (!odd_done_) {
      cv::Mat ox, oy;
      odd(ox, oy);
      P<FT, FT>::magnitude(ox, oy, o_);
      odd_done_ = true;
    }
    return o_;
  }

  /// @brief Get the odd response magnitude range.
  /// @return Valid range for odd response values.
  MagnitudeRange oddRange() const { return oddRange_; }

  /// @brief Check if energy has been computed.
  /// @return True if energy() has been called, false otherwise.
  inline bool isEnergyDone() const { return energy_done_; }

  /// @brief Get the local energy (magnitude of even and odd).
  /// Energy = sqrt(even^2 + odd^2)
  /// @return Local energy image.
  inline cv::Mat energy() const {
    if (!energy_done_) {
      P<FT, FT>::magnitude(odd(), even(), energy_);
      energy_done_ = true;
    }

    return energy_;
  }

  /// @brief Get the energy value range.
  /// @return Valid range for energy values.
  inline EnergyRange energyRange() const { return energyRange_; }

  /// @brief Check if direction has been computed.
  /// @return True if direction() has been called, false otherwise.
  inline bool isDirectionDone() const { return dir_done_; }

  /// @brief Get the local orientation/direction.
  /// Computed from odd X and Y components using phase operator.
  /// @return Direction image.
  cv::Mat direction() const {
    if (!dir_done_) {
      cv::Mat ox, oy;
      odd(ox, oy);
      P<FT, FT>::phase(ox, oy, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /// @brief Get direction value range.
  /// @return Range depending on phase operator ([-PI,PI], [0,2PI], or [0,360]).
  DirectionRange directionRange() const { return P<FT, FT>::range(); }

  /// @brief Check if phase has been computed.
  /// @return True if phase() has been called, false otherwise.
  inline bool isPhaseDone() const { return phase_done_; }

  /// @brief Get the local phase.
  /// Phase = atan2(odd, even) representing feature type.
  /// @return Phase image.
  cv::Mat phase() const {
    if (!phase_done_) {
      P<FT, FT>::phase(odd(), even(), phase_);
      phase_done_ = true;
    }
    return phase_;
  }

  /// @brief Get filtered/thresholded phase.
  /// Phase with small odd and even values zeroed to reduce noise.
  /// @return Filtered phase image.
  cv::Mat phaseF() const {
    cv::Mat o = odd(), e = even(), ret;
    o.setTo(0, o < 0.001);
    e.setTo(0, e<0.001 & e> - 0.001);
    P<FT, FT>::phase(o, e, ret);
    return ret;
  }

  /// @brief Get phase value range.
  /// @return Range depending on phase operator ([-PI,PI], [0,2PI], or [0,360]).
  PhaseRange phaseRange() const { return P<FT, FT>::range(); }

  /// @brief Get the name of the filter.
  /// @return "quadratureSF".
  virtual std::string name() const { return "quadratureSF"; }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using Quadrature<IT, FT, FT, FT, FT>::intensityRange;
  using Quadrature<IT, FT, FT, FT, FT>::results;
  using Quadrature<IT, FT, FT, FT, FT>::evenThreshold;
  using Quadrature<IT, FT, FT, FT, FT>::oddThreshold;
  using Quadrature<IT, FT, FT, FT, FT>::oddx;
  using Quadrature<IT, FT, FT, FT, FT>::oddy;
  using Quadrature<IT, FT, FT, FT, FT>::oddGradRange;
  using Quadrature<IT, FT, FT, FT, FT>::normType;
  using Quadrature<IT, FT, FT, FT, FT>::energyThreshold;
};

}  // namespace lsfm
