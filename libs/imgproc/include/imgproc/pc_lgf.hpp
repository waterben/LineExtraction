/// @file pc_lgf.hpp
/// @brief Phase Congruency using Log-Gabor Filters in frequency domain.
///
/// Implements the phase congruency model using Log-Gabor filters as described
/// by Kovesi. Log-Gabor filters have Gaussian transfer functions when viewed
/// on a logarithmic frequency scale, providing zero DC component in the even
/// symmetric filter and constant bandwidth on a log scale.
/// The implementation uses FFT for efficient frequency domain processing
/// across multiple scales.

#pragma once

#include <imgproc/phase_congruency.hpp>
#include <utility/limit.hpp>
#include <utility/matlab_helpers.hpp>

namespace lsfm {

/// @brief Phase Congruency detector using Log-Gabor Filters.
///
/// This class implements phase congruency computation using Log-Gabor filters
/// in the frequency domain. Log-Gabor filters are particularly well-suited
/// for phase congruency because:
/// - They have zero DC component in the even symmetric filter
/// - Gaussian transfer function on logarithmic frequency scale
/// - Constant shape ratio (bandwidth on log scale)
/// - Extended tail at high frequencies (better detection of fine features)
/// The filter bank consists of multiple scales with wavelengths increasing
/// by a multiplicative factor. Noise estimation can use various methods
/// including median-based or Rayleigh distribution fitting.
/// @tparam IT Input image pixel type.
/// @tparam FT Floating-point type for computations (float or double).
/// @tparam P Phase computation policy (default: Polar).
/// @see PhaseCongruency Base class interface
/// @see QuadratureLGF Log-Gabor quadrature filter
template <class IT, class FT, template <typename, typename> class P = Polar>
class PCLgf : public PhaseCongruency<IT, FT, FT, FT, FT> {
 public:
  /// @brief Create Log-Gabor filter bank for phase congruency.
  ///
  /// Generates a bank of Log-Gabor filters across multiple scales in the
  /// frequency domain. Each filter has Gaussian transfer function on
  /// logarithmic frequency scale.
  /// @param[out] lgf Array of filter responses (size nscale).
  /// @param[out] H Complex transfer function for odd filter computation.
  /// @param[in] rows Filter height.
  /// @param[in] cols Filter width.
  /// @param[in] minW Minimum wavelength (scale 0).
  /// @param[in] mult Wavelength multiplier between scales.
  /// @param[in] sigmaOnf Ratio of standard deviation to center frequency.
  /// @param[in] nscale Number of scales.
  /// @param[in] cutoff Low-pass filter cutoff (default 0.45).
  /// @param[in] n Low-pass filter order (default 15).
  static void logGaborFilter(cv::Mat* lgf,
                             cv::Mat& H,
                             int rows,
                             int cols,
                             FT minW,
                             FT mult,
                             FT sigmaOnf,
                             int nscale,
                             FT cutoff = static_cast<FT>(0.45),
                             int n = 15) {
    cv::Mat u1, u2, r;
    filterGrid<FT>(rows, cols, r, u1, u2);
    r.at<FT>(0, 0) = 1;
    divideComplex<FT>(merge<FT>(-u2, u1), r, H);
    // std::cout << "H:" << std::endl << H << std::endl;
    cv::Mat lp = lowpassFilter<FT>(rows, cols, cutoff, n);
    FT lsOnf = std::log(sigmaOnf);
    lsOnf = -1 / (2 * lsOnf * lsOnf);
    for (int s = 0; s != nscale; ++s) {
      FT wavelength = minW * std::pow(mult, static_cast<FT>(s));
      FT fo = 1 / wavelength;
      cv::log(r / fo, u1);
      cv::exp(u1.mul(u1) * lsOnf, lgf[s]);
      cv::multiply(lgf[s], lp, lgf[s]);
      lgf[s].at<FT>(0, 0) = 0;
      // std::cout << "lgf[" << s << "]:" << std::endl << lgf[s] << std::endl;
    }
  }

 private:
  int rows_;      ///< Image rows.
  int cols_;      ///< Image columns.
  int rows_ext_;  ///< Extended rows for FFT.
  int cols_ext_;  ///< Extended columns for FFT.
  int nscale_;    ///< Number of filter scales.

  FT minW_;             ///< Minimum wavelength.
  FT mult_;             ///< Wavelength multiplier.
  FT sigmaOnf_;         ///< Bandwidth parameter.
  FT k_;                ///< Noise threshold multiplier.
  FT cutOff_;           ///< Weighting function cutoff.
  FT g_;                ///< Weighting function gain.
  FT deviationGain_;    ///< Frequency spread penalty.
  FT T_;                ///< Estimated noise threshold.
  FT eps_;              ///< Small value to avoid division by zero.
  double noiseMethod_;  ///< Noise estimation method.

  cv::Mat_<FT> weight;  ///< Frequency spread weighting.
  cv::Mat_<FT> sumAn;   ///< Sum of amplitudes across scales.
  cv::Mat_<FT> ox_;     ///< Odd response X component.
  cv::Mat_<FT> oy_;     ///< Odd response Y component.
  cv::Mat_<FT> e_;      ///< Even response.
  cv::Mat_<FT> zeros;   ///< Zero matrix for max operations.

  mutable cv::Mat_<FT> odd_;     ///< Cached odd magnitude.
  mutable cv::Mat_<FT> dir_;     ///< Cached direction.
  mutable cv::Mat_<FT> phase_;   ///< Cached phase.
  mutable cv::Mat_<FT> energy_;  ///< Cached energy.
  mutable cv::Mat_<FT> oddSqr_;  ///< Cached odd squared.
  mutable cv::Mat_<FT> pc_;      ///< Cached phase congruency.

  mutable bool dir_done_;     ///< Direction computed flag.
  mutable bool phase_done_;   ///< Phase computed flag.
  mutable bool odd_done_;     ///< Odd magnitude computed flag.
  mutable bool energy_done_;  ///< Energy computed flag.
  mutable bool oddSqr_done_;  ///< Odd squared computed flag.
  mutable bool pc_done_;      ///< Phase congruency computed flag.

  cv::Mat_<std::complex<FT>> H_;                 ///< Transfer function.
  std::vector<cv::Mat_<std::complex<FT>>> lgf_;  ///< Log-Gabor filters.

  Range<FT> energyRange_;  ///< Valid energy range.
  Range<FT> oddRange_;     ///< Valid odd response range.
  Range<FT> evenRange_;    ///< Valid even response range.

  /// @brief Update filter bank for new image size.
  void updateFilter() {
    lgf_.resize(static_cast<size_t>(nscale_));
    std::vector<cv::Mat_<FT>> lgfTmp;
    lgfTmp.resize(static_cast<size_t>(nscale_));
    logGaborFilter(&lgfTmp[0], H_, rows_ext_, cols_ext_, minW_, mult_, sigmaOnf_, nscale_);
    for (int i = 0; i != nscale_; ++i) {
      lgf_[static_cast<size_t>(i)] = merge<FT>(lgfTmp[static_cast<size_t>(i)], lgfTmp[static_cast<size_t>(i)]);
    }

    sumAn = cv::Mat_<FT>::zeros(rows_, cols_);
    ox_ = cv::Mat_<FT>::zeros(rows_, cols_);
    oy_ = cv::Mat_<FT>::zeros(rows_, cols_);
    e_ = cv::Mat_<FT>::zeros(rows_, cols_);
    zeros = cv::Mat_<FT>::zeros(rows_, cols_);
  }

  /// @brief Calculate maximum response ranges for normalization.
  void max_response() {
    cv::Mat_<IT> tmp(128, 128);
    tmp.setTo(this->intRange_.lower);
    tmp.colRange(0, 64).setTo(this->intRange_.upper);
    // force update
    rows_ = 0;
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

  /// @brief Initialize ValueManager parameters.
  void init() {
    this->add("grad_numScales", std::bind(&PCLgf::numScales, this, std::placeholders::_1), "Number of scales.");
    this->add("grad_minWaveLength", std::bind(&PCLgf::minWaveLength, this, std::placeholders::_1),
              "Minimal wave length.");
    this->add("grad_mult", std::bind(&PCLgf::mult, this, std::placeholders::_1), "mult.");
    this->add("grad_sigmaOnf", std::bind(&PCLgf::sigmaOnf, this, std::placeholders::_1), "sigmaOnf.");
    this->add("grad_k", std::bind(&PCLgf::k, this, std::placeholders::_1), "k.");
    this->add("grad_cutOff", std::bind(&PCLgf::cutOff, this, std::placeholders::_1), "cutOff.");
    this->add("grad_g", std::bind(&PCLgf::g, this, std::placeholders::_1), "g.");
    this->add("grad_deviationGain", std::bind(&PCLgf::deviationGain, this, std::placeholders::_1), "deviationGain.");
    this->add("grad_noiseMethod", std::bind(&PCLgf::noiseMethod, this, std::placeholders::_1),
              "Noise method: 0 = none, -1 = dsdf, -2 = sdf, > 0 = threshold.");
    max_response();
  }


 public:
  typedef IT img_type;     ///< Input image type.
  typedef FT grad_type;    ///< Gradient type.
  typedef FT mag_type;     ///< Magnitude type.
  typedef FT energy_type;  ///< Energy type.
  typedef FT dir_type;     ///< Direction type.
  typedef FT phase_type;   ///< Phase type.

  typedef Range<IT> IntensityRange;  ///< Intensity value range.
  typedef Range<FT> GradientRange;   ///< Gradient value range.
  typedef Range<FT> MagnitudeRange;  ///< Magnitude value range.
  typedef Range<FT> EnergyRange;     ///< Energy value range.
  typedef Range<FT> DirectionRange;  ///< Direction value range.
  typedef Range<FT> PhaseRange;      ///< Phase value range.

  /// @brief Construct Phase Congruency detector with Log-Gabor filters.
  /// @param nscale Number of filter scales (default: 4).
  /// @param minWaveLength Minimum wavelength in pixels (default: 3).
  /// @param mult Wavelength multiplier between scales (default: 2.1).
  /// @param sigmaOnf Bandwidth parameter sigma/center freq (default: 0.55).
  /// @param k Noise threshold multiplier (default: 3).
  /// @param cutOff Frequency spread weighting cutoff (default: 0.5).
  /// @param g Frequency spread weighting gain (default: 10).
  /// @param deviationGain Frequency deviation penalty (default: 1.5).
  /// @param noiseMethod Noise estimation: -1=median, -2=Rayleigh, >=0=fixed.
  /// @param int_lower Minimum input intensity.
  /// @param int_upper Maximum input intensity.
  PCLgf(int nscale = 4,
        FT minWaveLength = 3,
        FT mult = static_cast<FT>(2.1),
        FT sigmaOnf = static_cast<FT>(0.55),
        FT k = 3,
        FT cutOff = static_cast<FT>(0.5),
        FT g = 10,
        FT deviationGain = static_cast<FT>(1.5),
        double noiseMethod = -1,
        img_type int_lower = std::numeric_limits<img_type>::lowest(),
        img_type int_upper = std::numeric_limits<img_type>::max())
      : PhaseCongruency<IT, FT, FT, FT, FT>(int_lower, int_upper),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        nscale_(nscale),
        minW_(minWaveLength),
        mult_(mult),
        sigmaOnf_(sigmaOnf),
        k_(k),
        cutOff_(cutOff),
        g_(g),
        deviationGain_(deviationGain),
        T_(0),
        eps_(static_cast<FT>(0.0001)),
        noiseMethod_(noiseMethod),
        weight(),
        sumAn(),
        ox_(),
        oy_(),
        e_(),
        zeros(),
        odd_(),
        dir_(),
        phase_(),
        energy_(),
        oddSqr_(),
        pc_(),
        dir_done_(false),
        phase_done_(false),
        odd_done_(false),
        energy_done_(false),
        oddSqr_done_(false),
        pc_done_(false),
        H_(),
        lgf_(),
        energyRange_(),
        oddRange_(),
        evenRange_() {
    init();
  }

  /// @brief Construct from name-value parameter list.
  /// @param options Vector of parameter name-value pairs.
  /// @param int_lower Minimum input intensity.
  /// @param int_upper Maximum input intensity.
  PCLgf(const ValueManager::NameValueVector& options,
        img_type int_lower = std::numeric_limits<img_type>::lowest(),
        img_type int_upper = std::numeric_limits<img_type>::max())
      : PhaseCongruency<IT, FT, FT, FT, FT>(int_lower, int_upper),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        nscale_(4),
        minW_(3),
        mult_(static_cast<FT>(2.1)),
        sigmaOnf_(static_cast<FT>(0.55)),
        k_(3),
        cutOff_(static_cast<FT>(0.5)),
        g_(10),
        deviationGain_(1.5),
        T_(0),
        eps_(static_cast<FT>(0.0001)),
        noiseMethod_(-1) {
    init();
    value(options);
  }

  /// @brief Construct from initializer list of parameters.
  /// @param options Initializer list of parameter name-value pairs.
  /// @param int_lower Minimum input intensity.
  /// @param int_upper Maximum input intensity.
  PCLgf(ValueManager::InitializerList options,
        img_type int_lower = std::numeric_limits<img_type>::lowest(),
        img_type int_upper = std::numeric_limits<img_type>::max())
      : PhaseCongruency<IT, FT, FT, FT, FT>(int_lower, int_upper),
        rows_(0),
        cols_(0),
        rows_ext_(0),
        cols_ext_(0),
        nscale_(4),
        minW_(3),
        mult_(static_cast<FT>(2.1)),
        sigmaOnf_(static_cast<FT>(0.55)),
        k_(3),
        cutOff_(static_cast<FT>(0.5)),
        g_(10),
        deviationGain_(1.5),
        T_(0),
        eps_(static_cast<FT>(0.0001)),
        noiseMethod_(-1),
        weight(),
        sumAn(),
        ox_(),
        oy_(),
        e_(),
        zeros(),
        odd_(),
        dir_(),
        phase_(),
        energy_(),
        oddSqr_(),
        pc_(),
        dir_done_(false),
        phase_done_(false),
        odd_done_(false),
        energy_done_(false),
        oddSqr_done_(false),
        pc_done_(false),
        H_(),
        lgf_(),
        energyRange_(),
        oddRange_(),
        evenRange_() {
    init();
    value(options);
  }

  /// @brief Get or set number of filter scales.
  /// @param ns Optional new value.
  /// @return Current number of scales.
  Value numScales(const Value& ns = Value::NAV()) {
    if (ns.type()) {
      nscale_ = ns;
      max_response();
    }
    return nscale_;
  }

  /// @brief Get or set minimum wavelength.
  /// @param mw Optional new value.
  /// @return Current minimum wavelength.
  Value minWaveLength(const Value& mw = Value::NAV()) {
    if (mw.type()) {
      minW_ = mw;
      max_response();
    }
    return minW_;
  }

  /// @brief Get or set wavelength multiplier.
  /// @param m Optional new value.
  /// @return Current multiplier.
  Value mult(const Value& m = Value::NAV()) {
    if (m.type()) {
      mult_ = m;
      max_response();
    }
    return mult_;
  }

  /// @brief Get or set bandwidth parameter.
  /// @param so Optional new value.
  /// @return Current sigmaOnf.
  Value sigmaOnf(const Value& so = Value::NAV()) {
    if (so.type()) {
      sigmaOnf_ = so;
      max_response();
    }
    return sigmaOnf_;
  }

  /// @brief Get or set noise threshold multiplier.
  /// @param kv Optional new value.
  /// @return Current k value.
  Value k(const Value& kv = Value::NAV()) {
    if (kv.type()) k_ = kv;
    return k_;
  }

  /// @brief Get or set frequency spread weighting cutoff.
  /// @param cutOff Optional new value.
  /// @return Current cutoff.
  Value cutOff(const Value& cutOff = Value::NAV()) {
    if (cutOff.type()) cutOff_ = cutOff;
    return cutOff_;
  }

  /// @brief Get or set frequency spread weighting gain.
  /// @param gv Optional new value.
  /// @return Current g value.
  Value g(const Value& gv = Value::NAV()) {
    if (gv.type()) g_ = gv;
    return g_;
  }

  /// @brief Get or set frequency deviation penalty.
  /// @param g Optional new value.
  /// @return Current deviation gain.
  Value deviationGain(const Value& g = Value::NAV()) {
    if (g.type()) deviationGain_ = g;
    return deviationGain_;
  }

  /// @brief Get or set noise estimation method.
  ///
  /// - -1: Median-based estimation
  /// - -2: Rayleigh distribution mode
  /// - >= 0: Fixed threshold value
  /// @param n Optional new value.
  /// @return Current noise method.
  Value noiseMethod(const Value& n = Value::NAV()) {
    if (n.type()) noiseMethod_ = n;
    return noiseMethod_;
  }

  /// @brief Process an image to compute phase congruency.
  ///
  /// Applies Log-Gabor filter bank across multiple scales,
  /// accumulates responses, estimates noise, and computes
  /// frequency spread weighting. Results are cached for lazy access.
  /// @param[in] img Input image.
  void process(const cv::Mat& img) {
    cv::Mat_<std::complex<FT>> IM, tmpv;
    cv::Mat_<FT> tmp1, tmp2, tmp3, An, maxAn;

    if (img.rows != rows_ || img.cols != cols_) {
      rows_ = img.rows;
      cols_ = img.cols;
      rows_ext_ = cv::getOptimalDFTSize(img.rows);
      cols_ext_ = cv::getOptimalDFTSize(img.cols);
      updateFilter();
    } else {
      sumAn.setTo(0);
      ox_.setTo(0);
      oy_.setTo(0);
      e_.setTo(0);
    }

    dir_done_ = false;
    odd_done_ = false;
    oddSqr_done_ = false;
    phase_done_ = false;
    energy_done_ = false;
    pc_done_ = false;


    FT tau = 1;

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

    for (int s = 0; s != nscale_; ++s) {
      // lgf already has two same channels, so cv::multiply is fastest
      cv::multiply(IM, lgf_[static_cast<size_t>(s)], tmpv);
      tmp1 = real<FT>(ifft2<FT>(tmpv));
      multiply(tmpv, H_, tmpv);
      tmpv = ifft2<FT>(tmpv);

      if (cols_ < cols_ext_ || rows_ < rows_ext_) {
        tmpv.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
        tmp1.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
      }

      split<FT>(tmpv, tmp2, tmp3);
      cv::sqrt(tmp1.mul(tmp1) + tmp2.mul(tmp2) + tmp3.mul(tmp3), An);
      sumAn += An;
      e_ += tmp1;
      ox_ += tmp2;
      oy_ += tmp3;

      if (s == 0) {
        if (noiseMethod_ == -1)
          tau = median(sumAn) / std::sqrt(std::log(static_cast<FT>(4)));
        else if (noiseMethod_ == -2)
          tau = rayleighMode(sumAn);
        An.copyTo(maxAn);
      } else
        maxAn = cv::max(maxAn, An);
    }

    cv::divide(sumAn, (maxAn + eps_), tmp1);
    tmp1 -= 1;
    tmp1 /= (nscale_ - 1);
    cv::exp((cutOff_ - tmp1) * g_, weight);
    weight += 1;
    weight = 1 / weight;

    if (noiseMethod_ >= 0)
      T_ = static_cast<FT>(noiseMethod_);
    else {
      FT totalTau = tau * (1 - std::pow(static_cast<FT>(1 / mult_), static_cast<FT>(nscale_))) / (1 - (1 / mult_));
      FT EstNoiseEnergyMean = totalTau * std::sqrt(static_cast<FT>(CV_PI) / 2);
      FT EstNoiseEnergySigma = totalTau * std::sqrt((4 - static_cast<FT>(CV_PI)) / 2);

      T_ = EstNoiseEnergyMean + k_ * EstNoiseEnergySigma;
    }
  }

  /// @brief Get X and Y odd filter response components.
  /// @param[out] ox X component.
  /// @param[out] oy Y component.
  void odd(cv::Mat& ox, cv::Mat& oy) const {
    ox = ox_;
    oy = oy_;
  }

  /// @brief Get squared odd response magnitude.
  /// @return Squared odd response (ox^2 + oy^2).
  cv::Mat oddSqr() const {
    if (!oddSqr_done_) {
      oddSqr_ = ox_.mul(ox_) + oy_.mul(oy_);
      oddSqr_done_ = true;
    }
    return oddSqr_;
  }

  /// @brief Get odd filter response magnitude.
  /// @return sqrt(ox^2 + oy^2).
  cv::Mat odd() const {
    if (odd_done_) return odd_;
    cv::sqrt(oddSqr(), odd_);
    odd_done_ = true;
    return odd_;
  }

  /// @brief Get odd response value range.
  /// @return Valid range for odd values.
  MagnitudeRange oddRange() const { return oddRange_; }

  /// @brief Check if odd magnitude has been computed.
  /// @return True if odd() has been called, false otherwise.
  inline bool isOddDone() const { return odd_done_; }

  /// @brief Get even filter response.
  /// @return Even (symmetric) response.
  cv::Mat even() const { return e_; }

  /// @brief Get even response value range.
  /// @return Valid range for even values.
  GradientRange evenRange() const { return evenRange_; }

  /// @brief Get local energy.
  /// @return sqrt(even^2 + odd^2).
  cv::Mat energy() const {
    if (energy_done_) return energy_;
    cv::sqrt(e_.mul(e_) + oddSqr(), energy_);
    energy_done_ = true;
    return energy_;
  }

  /// @brief Get energy value range.
  /// @return Valid range for energy values.
  EnergyRange energyRange() const { return energyRange_; }

  /// @brief Get local orientation/direction.
  /// @return Direction computed from odd X/Y components.
  cv::Mat direction() const {
    if (dir_done_) return dir_;
    P<FT, FT>::phase(ox_, oy_, dir_);
    dir_done_ = true;
    return dir_;
  }

  /// @brief Get direction value range.
  /// @return Range depending on phase operator.
  DirectionRange directionRange() const { return P<FT, FT>::range(); }

  /// @brief Get local phase.
  /// @return Phase = atan2(odd, even).
  cv::Mat phase() const {
    if (phase_done_) return phase_;
    P<FT, FT>::phase(odd(), e_, phase_);
    phase_done_ = true;
    return phase_;
  }

  /// @brief Get phase value range.
  /// @return Range depending on phase operator.
  PhaseRange phaseRange() const { return P<FT, FT>::range(); }

  /// @brief Get phase congruency measure.
  ///
  /// Computes weighted, noise-compensated phase congruency:
  /// PC = weight * (1 - deviation) * (energy - T) / energy
  /// @return Phase congruency image [0, 1].
  cv::Mat phaseCongruency() const {
    if (pc_done_) return pc_;
    energy();
    cv::Mat_<FT> tmp1, tmp2;
    cv::divide(energy_, sumAn + eps_, tmp1);
    cv::max(1 - deviationGain_ * acos(tmp1), zeros, tmp1);
    cv::max(energy_ - T_, zeros, tmp2);
    cv::divide(tmp2, energy_ + eps_, tmp2);
    pc_ = weight.mul(tmp1.mul(tmp2));
    pc_done_ = true;
    return pc_;
  }

  /// @brief Get the name of the detector.
  /// @return "pc_lgf".
  std::string name() const { return "pc_lgf"; }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using PhaseCongruency<IT, FT, FT, FT, FT>::intensityRange;
  using PhaseCongruency<IT, FT, FT, FT, FT>::results;
  using PhaseCongruency<IT, FT, FT, FT, FT>::evenThreshold;
  using PhaseCongruency<IT, FT, FT, FT, FT>::oddThreshold;
  using PhaseCongruency<IT, FT, FT, FT, FT>::oddx;
  using PhaseCongruency<IT, FT, FT, FT, FT>::oddy;
  using PhaseCongruency<IT, FT, FT, FT, FT>::oddGradRange;
  using PhaseCongruency<IT, FT, FT, FT, FT>::normType;
  using PhaseCongruency<IT, FT, FT, FT, FT>::energyThreshold;
  using PhaseCongruency<IT, FT, FT, FT, FT>::phaseCongruencyRange;
};

}  // namespace lsfm
