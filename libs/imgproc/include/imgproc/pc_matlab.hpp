/// @file pc_matlab.hpp
/// @brief MATLAB-compatible Phase Congruency implementation.
///
/// Computes the PC_2 measure of phase congruency following the original
/// MATLAB implementation by Kovesi. For maximum speed the input image
/// should be square and have a size that is a power of 2, but the code
/// will operate on images of arbitrary size.
/// This implementation uses the PhaseCong helper class which wraps
/// MATLAB-style computation routines for compatibility with existing
/// workflows and comparison with reference implementations.
/// @see http://www.csse.uwa.edu.au/~pk/research/matlabfns/

#pragma once

#include <imgproc/impl/PhaseCong.hpp>
#include <imgproc/phase_congruency.hpp>

namespace lsfm {

/// @brief MATLAB-compatible Phase Congruency detector.
///
/// This class implements phase congruency using the algorithm from Kovesi's
/// MATLAB toolbox. It uses double precision throughout for compatibility
/// with the original MATLAB implementation.
/// The implementation inherits from both PhaseCongruency (for the interface)
/// and PhaseCong (for the MATLAB-style computation backend).
/// Features:
/// - Compatible with Kovesi's MATLAB phasecong2 function
/// - Multi-scale Log-Gabor filter bank
/// - Automatic noise estimation
/// - Frequency spread weighting
/// @tparam IT Input image pixel type.
/// @see PhaseCongruency Base interface
/// @see PhaseCong MATLAB computation backend
/// @see PCLgf Alternative Log-Gabor implementation
template <class IT>
class PCMatlab : public PhaseCongruency<IT, double, double, double, double>, public PhaseCong {
  double k_;              ///< Noise threshold multiplier.
  double cutOff_;         ///< Frequency spread weighting cutoff.
  double g_;              ///< Frequency spread weighting gain.
  double deviationGain_;  ///< Frequency deviation penalty.
  double noiseMethod_;    ///< Noise estimation method.

  cv::Mat e_;       ///< Even filter response.
  cv::Mat ox_;      ///< Odd X component.
  cv::Mat oy_;      ///< Odd Y component.
  cv::Mat pc_;      ///< Phase congruency result.
  cv::Mat energy_;  ///< Local energy.

  mutable cv::Mat_<double> odd_;    ///< Cached odd magnitude.
  mutable cv::Mat_<double> dir_;    ///< Cached direction.
  mutable cv::Mat_<double> phase_;  ///< Cached phase.

  mutable bool dir_done_;    ///< Direction computed flag.
  mutable bool phase_done_;  ///< Phase computed flag.
  mutable bool odd_done_;    ///< Odd magnitude computed flag.

  Range<double> energyRange_;  ///< Valid energy range.
  Range<double> oddRange_;     ///< Valid odd response range.
  Range<double> evenRange_;    ///< Valid even response range.

  /// @brief Calculate maximum response ranges for normalization.
  void max_response() {
    cv::Mat_<IT> tmp(128, 128);
    tmp.setTo(this->intRange_.lower);
    tmp.colRange(0, 64).setTo(this->intRange_.upper);
    process(tmp);
    double vmin, vmax;
    cv::minMaxIdx(energy(), &vmin, &vmax);
    energyRange_.upper = vmax;
    cv::minMaxIdx(odd(), &vmin, &vmax);
    oddRange_.upper = vmax;
    cv::minMaxIdx(cv::abs(even()), &vmin, &vmax);
    evenRange_.upper = vmax;
    evenRange_.lower = -evenRange_.upper;
  }

  /// @brief Force filter update and recalculate ranges.
  void update() {
    rows_ = 0;
    max_response();
  }

  /// @brief Initialize ValueManager parameters.
  void init() {
    this->add("grad_numScales", std::bind(&PCMatlab<IT>::numScales, this, std::placeholders::_1), "Number of scales.");
    this->add("grad_minWaveLength", std::bind(&PCMatlab<IT>::minWaveLength, this, std::placeholders::_1),
              "Minimal wave length.");
    this->add("grad_mult", std::bind(&PCMatlab<IT>::mult, this, std::placeholders::_1), "mult.");
    this->add("grad_sigmaOnf", std::bind(&PCMatlab<IT>::sigmaOnf, this, std::placeholders::_1), "sigmaOnf.");
    this->add("grad_k", std::bind(&PCMatlab<IT>::k, this, std::placeholders::_1), "k.");
    this->add("grad_cutOff", std::bind(&PCMatlab<IT>::cutOff, this, std::placeholders::_1), "cutOff.");
    this->add("grad_g", std::bind(&PCMatlab<IT>::g, this, std::placeholders::_1), "g.");
    this->add("grad_deviationGain", std::bind(&PCMatlab<IT>::deviationGain, this, std::placeholders::_1),
              "deviationGain.");
    this->add("grad_noiseMethod", std::bind(&PCMatlab<IT>::noiseMethod, this, std::placeholders::_1),
              "Noise method: 0 = none, -1 = dsdf, -2 = sdf, > 0 = threshold.");
    update();
  }

 public:
  typedef IT img_type;         ///< Input image type.
  typedef double grad_type;    ///< Gradient type (always double).
  typedef double mag_type;     ///< Magnitude type.
  typedef double energy_type;  ///< Energy type.
  typedef double dir_type;     ///< Direction type.
  typedef double phase_type;   ///< Phase type.

  typedef Range<IT> IntensityRange;      ///< Intensity value range.
  typedef Range<double> GradientRange;   ///< Gradient value range.
  typedef Range<double> MagnitudeRange;  ///< Magnitude value range.
  typedef Range<double> EnergyRange;     ///< Energy value range.
  typedef Range<double> DirectionRange;  ///< Direction value range.
  typedef Range<double> PhaseRange;      ///< Phase value range.

  /// @brief Construct MATLAB-compatible Phase Congruency detector.
  /// @param nscale Number of filter scales (default: 4).
  /// @param minWaveLength Minimum wavelength in pixels (default: 3).
  /// @param mult Wavelength multiplier between scales (default: 2.1).
  /// @param sigmaOnf Bandwidth parameter (default: 0.55).
  /// @param k Noise threshold multiplier (default: 3).
  /// @param cutOff Frequency spread weighting cutoff (default: 0.5).
  /// @param g Frequency spread weighting gain (default: 10).
  /// @param deviationGain Frequency deviation penalty (default: 1.5).
  /// @param noiseMethod Noise estimation: -1=median, -2=Rayleigh, >=0=fixed.
  /// @param it_low Minimum input intensity.
  /// @param it_high Maximum input intensity.
  PCMatlab(int nscale = 4,
           int minWaveLength = 3,
           double mult = 2.1,
           double sigmaOnf = 0.55,
           double k = 3,
           double cutOff = 0.5,
           int g = 10,
           double deviationGain = 1.5,
           int noiseMethod = -1,
           IT it_low = std::numeric_limits<IT>::lowest(),
           IT it_high = std::numeric_limits<IT>::max())
      : PhaseCongruency<IT, double, double, double, double>(it_low, it_high),
        PhaseCong(nscale, minWaveLength, mult, sigmaOnf),
        k_(k),
        cutOff_(cutOff),
        g_(g),
        deviationGain_(deviationGain),
        noiseMethod_(noiseMethod),
        e_(),
        ox_(),
        oy_(),
        pc_(),
        energy_(),
        odd_(),
        dir_(),
        phase_(),
        dir_done_(false),
        phase_done_(false),
        odd_done_(false),
        energyRange_(),
        oddRange_(),
        evenRange_() {
    init();
  }

  /// @brief Construct from name-value parameter list.
  /// @param options Vector of parameter name-value pairs.
  /// @param int_lower Minimum input intensity.
  /// @param int_upper Maximum input intensity.
  PCMatlab(const ValueManager::NameValueVector& options,
           img_type int_lower = std::numeric_limits<img_type>::lowest(),
           img_type int_upper = std::numeric_limits<img_type>::max())
      : PhaseCongruency<IT, double, double, double, double>(int_lower, int_upper),
        PhaseCong(4, 3, 2.1, 0.55),
        k_(3),
        cutOff_(0.5),
        g_(10),
        deviationGain_(1.5),
        noiseMethod_(-1),
        e_(),
        ox_(),
        oy_(),
        pc_(),
        energy_(),
        odd_(),
        dir_(),
        phase_(),
        dir_done_(false),
        phase_done_(false),
        odd_done_(false),
        energyRange_(),
        oddRange_(),
        evenRange_() {
    init();
    value(options);
  }

  /// @brief Construct from initializer list of parameters.
  /// @param options Initializer list of parameter name-value pairs.
  /// @param int_lower Minimum input intensity.
  /// @param int_upper Maximum input intensity.
  PCMatlab(ValueManager::InitializerList options,
           img_type int_lower = std::numeric_limits<img_type>::lowest(),
           img_type int_upper = std::numeric_limits<img_type>::max())
      : PhaseCongruency<IT, double, double, double, double>(int_lower, int_upper),
        PhaseCong(4, 3, 2.1, 0.55),
        k_(3),
        cutOff_(0.5),
        g_(10),
        deviationGain_(1.5),
        noiseMethod_(-1),
        e_(),
        ox_(),
        oy_(),
        pc_(),
        energy_(),
        odd_(),
        dir_(),
        phase_(),
        dir_done_(false),
        phase_done_(false),
        odd_done_(false),
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
      update();
    }
    return nscale_;
  }

  /// @brief Get or set minimum wavelength.
  /// @param mw Optional new value.
  /// @return Current minimum wavelength.
  Value minWaveLength(const Value& mw = Value::NAV()) {
    if (mw.type()) {
      minW_ = mw;
      update();
    }
    return minW_;
  }

  /// @brief Get or set wavelength multiplier.
  /// @param m Optional new value.
  /// @return Current multiplier.
  Value mult(const Value& m = Value::NAV()) {
    if (m.type()) {
      mult_ = m;
      update();
    }
    return mult_;
  }

  /// @brief Get or set bandwidth parameter.
  /// @param so Optional new value.
  /// @return Current sigmaOnf.
  Value sigmaOnf(const Value& so = Value::NAV()) {
    if (so.type()) {
      sigmaOnf_ = so;
      update();
    }
    return sigmaOnf_;
  }

  /// @brief Get or set noise threshold multiplier.
  /// @param kv Optional new value.
  /// @return Current k value.
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
  /// Uses the PhaseCong backend to compute all filter responses
  /// and phase congruency in one call. Results are cached.
  /// @param[in] img Input image.
  void process(const cv::Mat& img) {
    odd_done_ = false;
    dir_done_ = false;
    phase_done_ = false;
    this->run(img, e_, ox_, oy_, energy_, pc_, k_, cutOff_, g_, deviationGain_, noiseMethod_);
  }

  /// @brief Get X and Y odd filter response components.
  /// @param[out] ox X component.
  /// @param[out] oy Y component.
  void odd(cv::Mat& ox, cv::Mat& oy) const {
    ox = ox_;
    oy = oy_;
  }

  /// @brief Get local energy.
  /// @return Energy image.
  inline cv::Mat energy() const { return energy_; }

  /// @brief Get energy value range.
  /// @return Valid range for energy values.
  inline EnergyRange energyRange() const { return energyRange_; }

  /// @brief Check if direction has been computed.
  /// @return True if direction() has been called, false otherwise.
  inline bool isDirectionDone() const { return dir_done_; }

  /// @brief Get local orientation/direction.
  /// @return Direction computed from odd X/Y components.
  cv::Mat direction() const {
    if (!dir_done_) {
      Polar<double, double>::phase(ox_, oy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /// @brief Get direction value range.
  /// @return Range depending on Polar phase operator.
  DirectionRange directionRange() const { return Polar<double, double>::range(); }

  /// @brief Get odd filter response magnitude.
  /// @return Magnitude of odd X/Y components.
  cv::Mat odd() const {
    if (!odd_done_) {
      Polar<double, double>::magnitude(ox_, oy_, odd_);
      odd_done_ = true;
    }
    return odd_;
  }

  /// @brief Get odd response magnitude range.
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

  /// @brief Check if phase has been computed.
  /// @return True if phase() has been called, false otherwise.
  inline bool isPhaseDone() const { return phase_done_; }

  /// @brief Get local phase.
  /// @return Phase = atan2(odd, even).
  cv::Mat phase() const {
    if (!phase_done_) {
      Polar<double, double>::phase(odd(), e_, phase_);
      phase_done_ = true;
    }
    return phase_;
  }

  /// @brief Get phase value range.
  /// @return Range depending on Polar phase operator.
  PhaseRange phaseRange() const { return Polar<double, double>::range(); }

  /// @brief Get phase congruency measure.
  /// @return Phase congruency image [0, 1].
  cv::Mat phaseCongruency() const { return pc_; }

  /// @brief Get the name of the detector.
  /// @return "pc_matlab".
  std::string name() const { return "pc_matlab"; }

  using ValueManager::value;
  using ValueManager::valuePair;
  using ValueManager::values;
  using PhaseCongruency<IT, double, double, double, double>::intensityRange;
  using PhaseCongruency<IT, double, double, double, double>::results;
  using PhaseCongruency<IT, double, double, double, double>::evenThreshold;
  using PhaseCongruency<IT, double, double, double, double>::oddThreshold;
  using PhaseCongruency<IT, double, double, double, double>::oddx;
  using PhaseCongruency<IT, double, double, double, double>::oddy;
  using PhaseCongruency<IT, double, double, double, double>::oddGradRange;
  using PhaseCongruency<IT, double, double, double, double>::normType;
  using PhaseCongruency<IT, double, double, double, double>::energyThreshold;
  using PhaseCongruency<IT, double, double, double, double>::phaseCongruencyRange;
};

}  // namespace lsfm
