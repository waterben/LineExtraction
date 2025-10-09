#pragma once

#include <imgproc/phase_congruency.hpp>
#include <utility/limit.hpp>
#include <utility/matlab_helpers.hpp>

namespace lsfm {
template <class IT, class FT, template <typename, typename> class P = Polar>
class PCLgf : public PhaseCongruency<IT, FT, FT, FT, FT> {
 public:
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
  int rows_, cols_, rows_ext_, cols_ext_, nscale_;
  FT minW_, mult_, sigmaOnf_, k_, cutOff_, g_, deviationGain_, T_, eps_;
  double noiseMethod_;
  cv::Mat_<FT> weight, sumAn, ox_, oy_, e_, zeros;

  mutable cv::Mat_<FT> odd_, dir_, phase_, energy_, oddSqr_, pc_;
  mutable bool dir_done_, phase_done_, odd_done_, energy_done_, oddSqr_done_, pc_done_;

  cv::Mat_<std::complex<FT>> H_;
  std::vector<cv::Mat_<std::complex<FT>>> lgf_;

  Range<FT> energyRange_, oddRange_, evenRange_;


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
  typedef IT img_type;
  typedef FT grad_type;
  typedef FT mag_type;
  typedef FT energy_type;
  typedef FT dir_type;
  typedef FT phase_type;

  typedef Range<IT> IntensityRange;
  typedef Range<FT> GradientRange;
  typedef Range<FT> MagnitudeRange;
  typedef Range<FT> EnergyRange;
  typedef Range<FT> DirectionRange;
  typedef Range<FT> PhaseRange;

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

  Value numScales(const Value& ns = Value::NAV()) {
    if (ns.type()) {
      nscale_ = ns;
      max_response();
    }
    return nscale_;
  }
  Value minWaveLength(const Value& mw = Value::NAV()) {
    if (mw.type()) {
      minW_ = mw;
      max_response();
    }
    return minW_;
  }
  Value mult(const Value& m = Value::NAV()) {
    if (m.type()) {
      mult_ = m;
      max_response();
    }
    return mult_;
  }
  Value sigmaOnf(const Value& so = Value::NAV()) {
    if (so.type()) {
      sigmaOnf_ = so;
      max_response();
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

  //! get x,y odd repsonses
  void odd(cv::Mat& ox, cv::Mat& oy) const {
    ox = ox_;
    oy = oy_;
  }

  cv::Mat oddSqr() const {
    if (!oddSqr_done_) {
      oddSqr_ = ox_.mul(ox_) + oy_.mul(oy_);
      oddSqr_done_ = true;
    }
    return oddSqr_;
  }

  //! get energy
  cv::Mat odd() const {
    if (odd_done_) return odd_;
    cv::sqrt(oddSqr(), odd_);
    odd_done_ = true;
    return odd_;
  }

  MagnitudeRange oddRange() const { return oddRange_; }

  //! test if mag is computed
  inline bool isOddDone() const { return odd_done_; }

  cv::Mat even() const { return e_; }

  GradientRange evenRange() const { return evenRange_; }

  cv::Mat energy() const {
    if (energy_done_) return energy_;
    cv::sqrt(e_.mul(e_) + oddSqr(), energy_);
    energy_done_ = true;
    return energy_;
  }

  EnergyRange energyRange() const { return energyRange_; }

  //! get direction
  cv::Mat direction() const {
    if (dir_done_) return dir_;
    P<FT, FT>::phase(ox_, oy_, dir_);
    dir_done_ = true;
    return dir_;
  }


  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  DirectionRange directionRange() const { return P<FT, FT>::range(); }

  //! get phase (optinal, since not all gradient support phase)
  cv::Mat phase() const {
    if (phase_done_) return phase_;
    P<FT, FT>::phase(odd(), e_, phase_);
    phase_done_ = true;
    return phase_;
  }

  //! get phase range ([-PI,PI], [0,2PI] or [0,360])
  PhaseRange phaseRange() const { return P<FT, FT>::range(); }

  //! get magnitude
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

  //! get name of direction operator
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
