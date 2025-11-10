/**********************************************************************\

PHASECONG - Computes phase congruency on an image.

This function calculates the PC_2 measure of phase congruency.
For maximum speed the input image should be square and have a
size that is a power of 2, but the code will operate on images
of arbitrary size.


\**********************************************************************/

#pragma once

#include <imgproc/impl/PhaseCong.hpp>
#include <imgproc/phase_congruency.hpp>

namespace lsfm {
template <class IT>
class PCMatlab : public PhaseCongruency<IT, double, double, double, double>, public PhaseCong {
  double k_, cutOff_, g_, deviationGain_, noiseMethod_;
  cv::Mat e_, ox_, oy_, pc_, energy_;

  mutable cv::Mat_<double> odd_, dir_, phase_;
  mutable bool dir_done_, phase_done_, odd_done_;

  Range<double> energyRange_, oddRange_, evenRange_;

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

  void update() {
    rows_ = 0;
    max_response();
  }

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
  typedef IT img_type;
  typedef double grad_type;
  typedef double mag_type;
  typedef double energy_type;
  typedef double dir_type;
  typedef double phase_type;

  typedef Range<IT> IntensityRange;
  typedef Range<double> GradientRange;
  typedef Range<double> MagnitudeRange;
  typedef Range<double> EnergyRange;
  typedef Range<double> DirectionRange;
  typedef Range<double> PhaseRange;

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

  Value numScales(const Value& ns = Value::NAV()) {
    if (ns.type()) {
      nscale_ = ns;
      update();
    }
    return nscale_;
  }
  Value minWaveLength(const Value& mw = Value::NAV()) {
    if (mw.type()) {
      minW_ = mw;
      update();
    }
    return minW_;
  }
  Value mult(const Value& m = Value::NAV()) {
    if (m.type()) {
      mult_ = m;
      update();
    }
    return mult_;
  }
  Value sigmaOnf(const Value& so = Value::NAV()) {
    if (so.type()) {
      sigmaOnf_ = so;
      update();
    }
    return sigmaOnf_;
  }

  Value k(const Value& kv = Value::NAV()) {
    if (kv.type()) k_ = kv;
    return k_;
  }
  Value cutOff(const Value& cutOff = Value::NAV()) {
    if (cutOff.type()) cutOff_ = cutOff;
    return cutOff_;
  }
  Value g(const Value& gv = Value::NAV()) {
    if (gv.type()) g_ = gv;
    return g_;
  }
  Value deviationGain(const Value& g = Value::NAV()) {
    if (g.type()) deviationGain_ = g;
    return deviationGain_;
  }
  Value noiseMethod(const Value& n = Value::NAV()) {
    if (n.type()) noiseMethod_ = n;
    return noiseMethod_;
  }

  //! process gradient
  void process(const cv::Mat& img) {
    odd_done_ = false;
    dir_done_ = false;
    phase_done_ = false;
    this->run(img, e_, ox_, oy_, energy_, pc_, k_, cutOff_, g_, deviationGain_, noiseMethod_);
  }

  //! get x,y odd repsonses
  void odd(cv::Mat& ox, cv::Mat& oy) const {
    ox = ox_;
    oy = oy_;
  }

  //! get energy
  inline cv::Mat energy() const { return energy_; }

  inline EnergyRange energyRange() const { return energyRange_; }

  //! test if direction is computed
  inline bool isDirectionDone() const { return dir_done_; }

  //! get direction
  cv::Mat direction() const {
    if (!dir_done_) {
      Polar<double, double>::phase(ox_, oy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  DirectionRange directionRange() const { return Polar<double, double>::range(); }

  //! get energy
  cv::Mat odd() const {
    if (!odd_done_) {
      Polar<double, double>::magnitude(ox_, oy_, odd_);
      odd_done_ = true;
    }
    return odd_;
  }

  MagnitudeRange oddRange() const { return oddRange_; }

  //! test if mag is computed
  inline bool isOddDone() const { return odd_done_; }

  cv::Mat even() const { return e_; }

  GradientRange evenRange() const { return evenRange_; }

  //! test if phase is computed
  inline bool isPhaseDone() const { return phase_done_; }

  //! get phase
  cv::Mat phase() const {
    if (!phase_done_) {
      Polar<double, double>::phase(odd(), e_, phase_);
      phase_done_ = true;
    }
    return phase_;
  }

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  PhaseRange phaseRange() const { return Polar<double, double>::range(); }

  //! get phase congruency
  cv::Mat phaseCongruency() const { return pc_; }

  //! get name of direction operator
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
