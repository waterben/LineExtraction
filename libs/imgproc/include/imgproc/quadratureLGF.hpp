/**
 * @file quadratureLGF.hpp
 * @brief Log-Gabor Filter (LGF) quadrature filter operating in frequency domain.
 *
 * This file implements a spherical Log-Gabor quadrature filter that operates
 * in the frequency domain using FFT. Log-Gabor filters have the advantage of
 * having no DC component and extended high-frequency coverage compared to
 * standard Gabor filters.
 *
 * @see "Fingerprint Enhancement using a Spherical Log Gabor Filter", Wedgwood et al.
 */

#pragma once

#include <imgproc/polar.hpp>
#include <imgproc/quadrature.hpp>
#include <opencv2/core/core.hpp>
#include <utility/matlab_helpers.hpp>

namespace lsfm {

/**
 * @brief Spherical Log-Gabor quadrature filter operating in frequency domain.
 *
 * QuadratureLGF implements a Log-Gabor filter in the frequency domain using FFT.
 * Log-Gabor filters are defined by their transfer function, which has no DC
 * component and covers the high-frequency end of the spectrum better than
 * standard Gabor filters.
 *
 * The filter is characterized by two parameters:
 * - wavelength: The wavelength of the filter (inversely related to frequency)
 * - sigmaOnf: Ratio of standard deviation to center frequency (bandwidth control)
 *
 * Key features:
 * - Frequency domain processing using FFT for efficiency
 * - Automatic optimal DFT size selection
 * - Zero DC response (important for edge detection)
 * - Configurable wavelength and bandwidth
 *
 * @tparam IT Input image type.
 * @tparam FT Floating-point type for computations.
 * @tparam P Polar coordinate class template (default: Polar).
 *
 * @see Quadrature Base class providing interface.
 */
template <class IT, class FT, template <typename, typename> class P = Polar>
class QuadratureLGF : public Quadrature<IT, FT, FT, FT, FT> {
 public:
  /**
   * @brief Create Log-Gabor filter kernels in frequency domain.
   *
   * Creates both the even (real) and odd (imaginary) filter components
   * in the frequency domain. A low-pass filter is applied to prevent
   * aliasing artifacts.
   *
   * @param[out] e Even filter kernel (real-valued).
   * @param[out] o Odd filter kernel (complex-valued).
   * @param[in] rows Number of rows in filter.
   * @param[in] cols Number of columns in filter.
   * @param[in] wavelength Filter wavelength.
   * @param[in] sigmaOnf Bandwidth parameter (std dev / center freq).
   * @param[in] cutoff Low-pass filter cutoff frequency (default: 0.45).
   * @param[in] n Low-pass filter order (default: 15).
   */
  static void createFilter(cv::Mat_<FT>& e,
                           cv::Mat_<std::complex<FT>>& o,
                           int rows,
                           int cols,
                           FT wavelength,
                           FT sigmaOnf,
                           FT cutoff = static_cast<FT>(0.45),
                           int n = 15) {
    cv::Mat u1, u2, r;
    filterGrid<FT>(rows, cols, r, u1, u2);
    r.at<FT>(0, 0) = 1;
    divideComplex<FT>(merge<FT>(-u2, u1), r, o);

    FT lsOnf = std::log(sigmaOnf);
    lsOnf = -1 / (2 * lsOnf * lsOnf);

    cv::log(r * wavelength, u1);
    cv::exp(u1.mul(u1) * lsOnf, e);
    cv::multiply(e, lowpassFilter<FT>(rows, cols, cutoff, n), e);
    e.template at<FT>(0, 0) = 0;
  }

 protected:
  int rows_;      ///< Original image rows.
  int cols_;      ///< Original image columns.
  int rows_ext_;  ///< Extended rows for optimal DFT size.
  int cols_ext_;  ///< Extended columns for optimal DFT size.
  FT waveL_;      ///< Filter wavelength.
  FT sigmaOnf_;   ///< Bandwidth parameter.

  mutable cv::Mat_<FT> dir_;     ///< Cached direction image.
  mutable cv::Mat_<FT> phase_;   ///< Cached phase image.
  mutable cv::Mat_<FT> energy_;  ///< Cached energy image.
  mutable cv::Mat_<FT> o_;       ///< Cached odd magnitude image.
  mutable cv::Mat_<FT> ox_;      ///< Cached odd X component.
  mutable cv::Mat_<FT> oy_;      ///< Cached odd Y component.
  mutable cv::Mat_<FT> e_;       ///< Cached even response.

  mutable bool dir_done_;     ///< Flag: direction computed.
  mutable bool phase_done_;   ///< Flag: phase computed.
  mutable bool odd_done_;     ///< Flag: odd magnitude computed.
  mutable bool energy_done_;  ///< Flag: energy computed.

  Range<FT> energyRange_;  ///< Range of energy values.
  Range<FT> oddRange_;     ///< Range of odd filter values.
  Range<FT> evenRange_;    ///< Range of even filter values.

  cv::Mat_<std::complex<FT>> H_;  ///< Hilbert transform filter (odd).
  cv::Mat_<FT> lgf_;              ///< Log-Gabor filter (even).

  /**
   * @brief Compute maximum filter response ranges.
   *
   * Creates a step edge test pattern and processes it to determine
   * the maximum energy, odd, and even response values.
   */
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

  /**
   * @brief Update filter kernels for new image size.
   *
   * @param img Image to match filter size to.
   * @param force If true, always regenerate. If false, only if size changed.
   */
  void updateFilter(const cv::Mat img, bool force = true) {
    // if force is of, only renew if image size has changed
    if (!force && img.rows == rows_ && img.cols == cols_) return;

    rows_ = img.rows;
    cols_ = img.cols;
    rows_ext_ = cv::getOptimalDFTSize(img.rows);
    cols_ext_ = cv::getOptimalDFTSize(img.cols);

    createFilter(lgf_, H_, rows_ext_, cols_ext_, waveL_, sigmaOnf_);
  }

  /**
   * @brief Initialize option manager entries.
   */
  void init() {
    this->add("grad_waveLength", std::bind(&QuadratureLGF<IT, FT, P>::valueWaveLength, this, std::placeholders::_1),
              "Wave length of scale filter.");
    this->add("grad_sigmaOnf", std::bind(&QuadratureLGF<IT, FT, P>::valueSigmaOnf, this, std::placeholders::_1),
              "Ratio of the standard deviation of the Gaussian describing the log Gabor filter's transfer function in "
              "the frequency domain to the filter center frequency.");

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

  /**
   * @brief Construct a Log-Gabor quadrature filter.
   *
   * @param waveLength Filter wavelength (default: 5).
   * @param sigmaOnf Bandwidth parameter (default: 0.55).
   * @param int_lower Lower bound of input intensity range.
   * @param int_upper Upper bound of input intensity range.
   */
  QuadratureLGF(FT waveLength = 5,
                FT sigmaOnf = static_cast<FT>(0.55),
                IT int_lower = std::numeric_limits<IT>::lowest(),
                IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        waveL_(waveLength),
        sigmaOnf_(sigmaOnf),
        dir_(),
        phase_(),
        energy_(),
        o_(),
        ox_(),
        oy_(),
        e_(),
        dir_done_(false),
        phase_done_(false),
        odd_done_(false),
        energy_done_(false),
        energyRange_(),
        oddRange_(),
        evenRange_(),
        H_(),
        lgf_() {
    init();
  }

  /**
   * @brief Construct from a name-value option vector.
   *
   * @param options Configuration options (grad_waveLength, grad_sigmaOnf).
   * @param int_lower Lower bound of input intensity range.
   * @param int_upper Upper bound of input intensity range.
   */
  QuadratureLGF(const ValueManager::NameValueVector& options,
                img_type int_lower = std::numeric_limits<img_type>::lowest(),
                img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        waveL_(3),
        sigmaOnf_(static_cast<FT>(0.55)),
        dir_(),
        phase_(),
        energy_(),
        o_(),
        ox_(),
        oy_(),
        e_(),
        dir_done_(false),
        phase_done_(false),
        odd_done_(false),
        energy_done_(false),
        energyRange_(),
        oddRange_(),
        evenRange_(),
        H_(),
        lgf_() {
    init();
    value(options);
  }

  /**
   * @brief Construct from an initializer list of options.
   *
   * @param options Configuration options as initializer list.
   * @param int_lower Lower bound of input intensity range.
   * @param int_upper Upper bound of input intensity range.
   */
  QuadratureLGF(ValueManager::InitializerList options,
                img_type int_lower = std::numeric_limits<img_type>::lowest(),
                img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        waveL_(3),
        sigmaOnf_(static_cast<FT>(0.55)),
        dir_(),
        phase_(),
        energy_(),
        o_(),
        ox_(),
        oy_(),
        e_(),
        dir_done_(false),
        phase_done_(false),
        odd_done_(false),
        energy_done_(false),
        energyRange_(),
        oddRange_(),
        evenRange_(),
        H_(),
        lgf_() {
    init();
    value(options);
  }

  /**
   * @brief Get or set wavelength through ValueManager interface.
   * @param w Optional new wavelength value.
   * @return Current wavelength.
   */
  Value valueWaveLength(const Value& w = Value::NAV()) {
    if (w.type()) waveLength(w);
    return waveL_;
  }

  /**
   * @brief Get the filter wavelength.
   * @return Current wavelength.
   */
  FT waveLength() const { return waveL_; }

  /**
   * @brief Set the filter wavelength.
   *
   * The wavelength controls the scale of features detected.
   * Larger wavelengths detect larger-scale features.
   *
   * @param w New wavelength (must be positive).
   */
  void waveLength(FT w) {
    if (w == waveL_ || w <= 0) return;
    waveL_ = w;
    max_response();
  }

  /**
   * @brief Get or set sigmaOnf through ValueManager interface.
   * @param so Optional new sigmaOnf value.
   * @return Current sigmaOnf.
   */
  Value valueSigmaOnf(const Value& so = Value::NAV()) {
    if (so.type()) sigmaOnf(so);
    return sigmaOnf_;
  }

  /**
   * @brief Get the bandwidth parameter.
   * @return Current sigmaOnf value.
   */
  FT sigmaOnf() const { return sigmaOnf_; }

  /**
   * @brief Set the bandwidth parameter.
   *
   * sigmaOnf is the ratio of std deviation to center frequency.
   * Smaller values give narrower bandwidth (more selective).
   *
   * @param s New sigmaOnf value (must be positive).
   */
  void sigmaOnf(FT s) {
    if (s == sigmaOnf_ || s <= 0) return;
    sigmaOnf_ = s;
    max_response();
  }

  /**
   * @brief Process an image to compute quadrature filter responses.
   *
   * Applies the Log-Gabor filter in the frequency domain using FFT.
   * The image is padded to optimal DFT size if necessary.
   *
   * @param[in] img Input image.
   */
  void process(const cv::Mat& img) {
    cv::Mat_<std::complex<FT>> IM, tmpv;

    dir_done_ = false;
    odd_done_ = false;
    phase_done_ = false;
    energy_done_ = false;

    updateFilter(img, false);

    cv::Mat_<FT> src;
    if (img.type() != cv::DataType<FT>::type)
      img.convertTo(src, cv::DataType<FT>::type);
    else
      src = img;

    if (cols_ < cols_ext_ || rows_ < rows_ext_)
      cv::copyMakeBorder(src, src, 0, rows_ext_ - rows_, 0, cols_ext_ - cols_, cv::BORDER_REPLICATE);

#ifdef USE_PERIODIC_FFT  // slower, but removes artifacts
    IM = perfft2<FT>(src);
#else
    IM = fft2<FT>(src);
#endif

    tmpv = multiply(IM, lgf_);
    e_ = real(ifft2<FT>(tmpv));
    tmpv = ifft2(multiply(tmpv, H_));

    if (cols_ < cols_ext_ || rows_ < rows_ext_) {
      tmpv.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
      e_.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
    }

    splitT(tmpv, ox_, oy_);
  }

  /**
   * @brief Get the even filter response.
   * @return Even (symmetric) filter response image.
   */
  cv::Mat even() const { return e_; }

  /**
   * @brief Get the even filter response range.
   * @return Range of even filter values.
   */
  GradientRange evenRange() const { return evenRange_; }

  /**
   * @brief Get X and Y components of odd filter response.
   * @param[out] ox X component.
   * @param[out] oy Y component.
   */
  void odd(cv::Mat& ox, cv::Mat& oy) const {
    ox = ox_;
    oy = oy_;
  }

  /**
   * @brief Check if odd magnitude has been computed.
   * @return True if odd() has been called, false otherwise.
   */
  inline bool isOddDone() const { return odd_done_; }

  /**
   * @brief Get the odd filter response magnitude.
   *
   * Computes magnitude from X/Y components if not already computed.
   *
   * @return Odd filter response magnitude image.
   */
  cv::Mat odd() const {
    if (!odd_done_) {
      Polar<FT, FT>::magnitude(ox_, oy_, o_);
      odd_done_ = true;
    }
    return o_;
  }

  /**
   * @brief Get the odd filter response range.
   * @return Range of odd filter magnitude values.
   */
  MagnitudeRange oddRange() const { return oddRange_; }

  /**
   * @brief Check if direction has been computed.
   * @return True if direction() has been called, false otherwise.
   */
  inline bool isDirectionDone() const { return dir_done_; }

  /**
   * @brief Get the direction image.
   *
   * Direction is computed from the odd X/Y components.
   *
   * @return Direction image.
   */
  cv::Mat direction() const {
    if (!dir_done_) {
      Polar<FT, FT>::phase(ox_, oy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /**
   * @brief Get the direction value range.
   * @return Range of direction values from Polar class.
   */
  DirectionRange directionRange() const { return Polar<FT, FT>::range(); }

  /**
   * @brief Check if energy has been computed.
   * @return True if energy() has been called, false otherwise.
   */
  inline bool isEnergyDone() const { return energy_done_; }

  /**
   * @brief Get the local energy.
   *
   * Energy is computed as the magnitude of the complex
   * quadrature response (even + i*odd).
   *
   * @return Energy image.
   */
  inline cv::Mat energy() const {
    if (!energy_done_) {
      P<FT, FT>::magnitude(odd(), e_, energy_);
      energy_done_ = true;
    }
    return energy_;
  }

  /**
   * @brief Get the energy value range.
   * @return Range of energy values.
   */
  EnergyRange energyRange() const { return energyRange_; }

  /**
   * @brief Check if phase has been computed.
   * @return True if phase() has been called, false otherwise.
   */
  inline bool isPhaseDone() const { return phase_done_; }

  /**
   * @brief Get the local phase.
   *
   * Phase is the angle of the complex quadrature response.
   * It indicates the type of feature (edge, line, etc.).
   *
   * @return Phase image.
   */
  cv::Mat phase() const {
    if (!phase_done_) {
      Polar<FT, FT>::phase(odd(), e_, phase_);
      phase_done_ = true;
    }
    return phase_;
  }

  /**
   * @brief Get the phase value range.
   * @return Range of phase values from Polar class.
   */
  PhaseRange phaseRange() const { return Polar<FT, FT>::range(); }

  /**
   * @brief Get the name of this gradient operator.
   * @return "quadratureLGF".
   */
  std::string name() const { return "quadratureLGF"; }

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
