
#pragma once

#include <imgproc/polar.hpp>
#include <imgproc/quadrature.hpp>
#include <opencv2/core/core.hpp>
#include <utility/matlab_helpers.hpp>

namespace lsfm {
//! Spherical Possion qudarture filter operating in freqency space
template <class IT = uchar, class FT = float, template <typename, typename> class P = Polar>
class QuadratureSF : public Quadrature<IT, FT, FT, FT, FT> {
 public:
  static constexpr FT PI2 = 2 * static_cast<FT>(CV_PI);

  static inline std::complex<FT> docpf(FT u, FT v, FT s, FT m) {
    FT a = std::sqrt(u * u + v * v), b = -PI2 * a * s, c = std::exp(b) - std::exp(b * m);
    return std::complex<FT>(-u / a * c, v / a * c);
  }
  static inline FT dopf(FT u, FT v, FT s, FT m) {
    FT a = -PI2 * std::sqrt(u * u + v * v) * s;
    return std::exp(a) - std::exp(a * m);
  }

  typedef FT (*KernelType)(FT u, FT v, FT s, FT m);
  typedef std::complex<FT> (*KernelTypeC)(FT u, FT v, FT s, FT m);

  static cv::Mat_<FT> createFilter(int rows, int cols, FT spacing, FT s, FT m, KernelType f) {
    cv::Mat_<FT> kernel(rows, cols);
    FT step = spacing * (cols - 1);
    FT startx = rows / 2 - static_cast<FT>(0.5), starty = cols / 2 - static_cast<FT>(0.5);
    for (int i = 0; i != rows; ++i)
      for (int j = 0; j != cols; ++j) kernel(i, j) = f((j - starty) / step, (i - startx) / step, s, m);

    return kernel;
  }

  static cv::Mat_<std::complex<FT>> createFilterC(int rows, int cols, FT spacing, FT s, FT m, KernelTypeC f) {
    cv::Mat_<std::complex<FT>> kernel(rows, cols);
    FT step = spacing * (cols - 1);
    FT startx = rows / 2 - static_cast<FT>(0.5), starty = cols / 2 - static_cast<FT>(0.5);
    for (int i = 0; i != rows; ++i)
      for (int j = 0; j != cols; ++j) kernel(i, j) = f((j - starty) / step, (i - startx) / step, s, m);
    return kernel;
  }

 protected:
  // filter
  cv::Mat_<FT> fe_;
  cv::Mat_<std::complex<FT>> fo_, imgf_;

  mutable cv::Mat_<FT> o_, phase_, dir_, ox_, oy_, e_, energy_;
  int rows_, cols_, rows_ext_, cols_ext_;

  Range<FT> energyRange_, oddRange_, evenRange_;
  using Quadrature<IT, FT, FT, FT, FT>::intRange_;

  FT kspacing_, scale_, muls_;

  mutable bool odd_done_, energy_done_, dir_done_, phase_done_, even_done_, oddxy_done_;

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


  QuadratureSF(FT scale = 1,
               FT muls = 2,
               FT kernel_spacing = 1,
               IT int_lower = std::numeric_limits<IT>::lowest(),
               IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        kspacing_(kernel_spacing),
        scale_(scale),
        muls_(muls),
        cols_(0),
        cols_ext_(0),
        rows_(0),
        rows_ext_(0) {
    init();
  }

  QuadratureSF(FT scale,
               FT l,
               int k,
               FT kernel_spacing,
               IT int_lower = std::numeric_limits<IT>::lowest(),
               IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        kspacing_(kernel_spacing),
        scale_(scale * pow(l, k)),
        muls_(pow(l, k - 1) / pow(l, k)),
        cols_(0),
        cols_ext_(0),
        rows_(0),
        rows_ext_(0) {
    init();
  }

  QuadratureSF(const ValueManager::NameValueVector& options,
               img_type int_lower = std::numeric_limits<img_type>::lowest(),
               img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        kspacing_(1),
        scale_(1),
        muls_(2),
        cols_(0),
        cols_ext_(0),
        rows_(0),
        rows_ext_(0) {
    init();
    value(options);
  }

  QuadratureSF(ValueManager::InitializerList options,
               img_type int_lower = std::numeric_limits<img_type>::lowest(),
               img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        kspacing_(1),
        scale_(1),
        muls_(2),
        cols_(0),
        cols_ext_(0),
        rows_(0),
        rows_ext_(0) {
    init();
    value(options);
  }

  Value valueKernelSpacing(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSpacing(ks);
    return kspacing_;
  }

  FT kernelSpacing() const { return kspacing_; }

  void kernelSpacing(FT ks) {
    if (ks == kspacing_ || ks <= 0) return;
    kspacing_ = ks;
    max_response();
  }

  Value valueScale(const Value& s = Value::NAV()) {
    if (s.type()) scale(s);
    return scale_;
  }

  FT scale() const { return scale_; }

  void scale(FT s) {
    if (s == scale_ || s <= 0) return;
    scale_ = s;
    max_response();
  }

  Value valueMuls(const Value& m = Value::NAV()) {
    if (m.type()) muls(m);
    return muls_;
  }

  FT muls() const { return muls_; }

  void muls(FT m) {
    if (m == muls_ || m <= 0) return;
    muls_ = m;
    max_response();
  }

  void muls(FT s, FT l, int k) {
    if (k <= 0 || l <= 0 || s <= 0) return;
    scale_ = s * std::pow(l, k);
    muls_ = std::pow(l, k - 1) / std::pow(l, k);
    max_response();
  }

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

  //! test if energy is computed
  inline bool isOddxyDone() const { return oddxy_done_; }

  //! get x,y odd responses
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

  //! test if energy is computed
  inline bool isEvenDone() const { return even_done_; }

  //! get even filter result
  inline cv::Mat even() const {
    if (!even_done_) {
      e_ = real(ifft2(multiply(imgf_, fe_)));
      if (cols_ < cols_ext_ || rows_ < rows_ext_) e_.adjustROI(0, rows_ - rows_ext_, 0, cols_ - cols_ext_);
      even_done_ = true;
    }
    return e_;
  }

  GradientRange evenRange() const { return evenRange_; }

  //! test if energy is computed
  inline bool isOddDone() const { return odd_done_; }

  //! get odd filter result
  inline cv::Mat odd() const {
    if (!odd_done_) {
      cv::Mat ox, oy;
      odd(ox, oy);
      P<FT, FT>::magnitude(ox, oy, o_);
      odd_done_ = true;
    }
    return o_;
  }

  MagnitudeRange oddRange() const { return oddRange_; }

  //! test if energy is computed
  inline bool isEnergyDone() const { return energy_done_; }

  //! get energy
  inline cv::Mat energy() const {
    if (!energy_done_) {
      P<FT, FT>::magnitude(odd(), even(), energy_);
      energy_done_ = true;
    }

    return energy_;
  }

  inline EnergyRange energyRange() const { return energyRange_; }

  //! test if direction is computed
  inline bool isDirectionDone() const { return dir_done_; }

  //! get direction
  cv::Mat direction() const {
    if (!dir_done_) {
      cv::Mat ox, oy;
      odd(ox, oy);
      P<FT, FT>::phase(ox, oy, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  DirectionRange directionRange() const { return P<FT, FT>::range(); }

  //! test if phase is computed
  inline bool isPhaseDone() const { return phase_done_; }

  //! get phase
  cv::Mat phase() const {
    if (!phase_done_) {
      P<FT, FT>::phase(odd(), even(), phase_);
      phase_done_ = true;
    }
    return phase_;
  }

  cv::Mat phaseF() const {
    cv::Mat o = odd(), e = even(), ret;
    o.setTo(0, o < 0.001);
    e.setTo(0, e<0.001 & e> - 0.001);
    P<FT, FT>::phase(o, e, ret);
    return ret;
  }

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  PhaseRange phaseRange() const { return P<FT, FT>::range(); }

  //! get name of gradient operator
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
