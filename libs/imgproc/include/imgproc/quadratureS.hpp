
#pragma once

#include <imgproc/derivative.hpp>
#include <imgproc/polar.hpp>
#include <imgproc/quadrature.hpp>
#include <opencv2/core/core.hpp>

namespace lsfm {

template <class IT = uchar, class GT = float, class FT = float, template <typename, typename> class P = Polar>
class QuadratureS : public Quadrature<IT, GT, FT, FT, FT> {
 public:
  static inline FT docp(FT x, FT y, FT s, FT m) {
    FT a = (s * s + x * x + y * y), b = (m * m * s * s + x * x + y * y);
    return x / (2 * static_cast<FT>(CV_PI) * std::sqrt(a * a * a)) -
           x / (2 * static_cast<FT>(CV_PI) * std::sqrt(b * b * b));
  }
  static inline FT dop(FT x, FT y, FT s, FT m) {
    FT a = (s * s + x * x + y * y), b = (m * m * s * s + x * x + y * y);
    return s / (2 * static_cast<FT>(CV_PI) * std::sqrt(a * a * a)) -
           (m * s) / (2 * static_cast<FT>(CV_PI) * std::sqrt(b * b * b));
  }


  typedef FT (*KernelType)(FT x, FT y, FT s, FT m);

  static cv::Mat_<FT> createFilter(int width, FT spacing, FT s, FT m, KernelType f) {
    width = width / 2;
    cv::Mat_<FT> kernel(width * 2 + 1, width * 2 + 1);
    for (int i = -width; i <= width; ++i)
      for (int j = -width; j <= width; ++j) kernel(j + width, i + width) = f(i * spacing, j * spacing, s, m);

    return kernel;
  }

 protected:
  cv::Point anchor;

  // the kernels
  cv::Mat kox_, koy_, ke_, img_;

  mutable cv::Mat o_, phase_, dir_, ox_, oy_, e_, energy_;

  Range<GT> evenRange_;
  Range<FT> energyRange_;
  DerivativeMax<GT> gm_;

  int ksize_;
  FT kspacing_, kscale_, scale_, muls_;

  mutable bool odd_done_, energy_done_, dir_done_, phase_done_, even_done_;

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
  typedef IT img_type;
  typedef GT grad_type;
  typedef FT mag_type;
  typedef FT energy_type;
  typedef FT dir_type;
  typedef FT phase_type;

  typedef Range<IT> IntensityRange;
  typedef Range<GT> GradientRange;
  typedef Range<FT> MagnitudeRange;
  typedef Range<FT> EnergyRange;
  typedef Range<FT> DirectionRange;
  typedef Range<FT> PhaseRange;

  QuadratureS(FT scale = 1,
              FT muls = 3,
              int kernel_size = 5,
              FT kernel_spacing = 1,
              FT kernel_scale = 1,
              IT int_lower = std::numeric_limits<IT>::lowest(),
              IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        anchor(-1, -1),
        ksize_(kernel_size),
        kspacing_(kernel_spacing),
        kscale_(kernel_scale),
        scale_(scale),
        muls_(muls) {
    init();
  }

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
        ksize_(kernel_size),
        kspacing_(kernel_spacing),
        kscale_(kernel_scale),
        scale_(scale * pow(l, k)),
        muls_(pow(l, k - 1) / pow(l, k)) {
    init();
  }

  QuadratureS(const ValueManager::NameValueVector& options,
              img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        anchor(-1, -1),
        ksize_(5),
        kspacing_(1),
        kscale_(1),
        scale_(1),
        muls_(2) {
    init();
    this->value(options);
  }

  QuadratureS(ValueManager::InitializerList options,
              img_type int_lower = std::numeric_limits<img_type>::lowest(),
              img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, GT, FT, FT, FT>(int_lower, int_upper),
        anchor(-1, -1),
        ksize_(5),
        kspacing_(1),
        kscale_(1),
        scale_(1),
        muls_(2) {
    init();
    this->value(options);
  }

  Value valueKernelSize(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSize(ks);
    return ksize_;
  }

  //! get kernel size
  int kernelSize() const { return ksize_; }

  //! set kernel size (range 3-99, has to be odd, even will be corrected to ksize+1)
  //! Note: large kernels needs larger GT type like int or long long int
  void kernelSize(int ks) {
    if (ks == ksize_) return;

    if (ks < 3) ks = 3;
    if (ks % 2 == 0) ++ks;
    if (ks > 99) ks = 99;
    ksize_ = ks;
    create_kernels();
  }

  Value valueKernelSpacing(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelSpacing(ks);
    return kspacing_;
  }

  FT kernelSpacing() const { return kspacing_; }

  void kernelSpacing(FT ks) {
    if (ks == kspacing_ || ks <= 0) return;
    kspacing_ = ks;
    create_kernels();
  }

  Value valueKernelScale(const Value& ks = Value::NAV()) {
    if (ks.type()) kernelScale(ks);
    return kscale_;
  }

  FT kernelScale() const { return kscale_; }

  void kernelScale(FT ks) {
    if (ks == kscale_ || ks <= 0) return;
    kscale_ = ks;
    create_kernels();
  }

  Value valueScale(const Value& s = Value::NAV()) {
    if (s.type()) scale(s);
    return scale_;
  }

  FT scale() const { return scale_; }

  void scale(FT s) {
    if (s == scale_ || s <= 0) return;
    scale_ = s;
    create_kernels();
  }

  Value valueMuls(const Value& m = Value::NAV()) {
    if (m.type()) muls(m);
    return muls_;
  }

  FT muls() const { return muls_; }

  void muls(FT m) {
    if (m == muls_ || m <= 0) return;
    muls_ = m;
    create_kernels();
  }

  void muls(FT s, FT l, int k) {
    if (k <= 0 || l <= 0 || s <= 0) return;
    scale_ = s * std::pow(l, k);
    muls_ = std::pow(l, k - 1) / std::pow(l, k);
    create_kernels();
  }

  // return even kernel
  cv::Mat kernel() const { return ke_; }

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

  //! get x,y odd responses
  void odd(cv::Mat& gx, cv::Mat& gy) const {
    gx = ox_;
    gy = oy_;
  }

  //! test if energy is computed
  inline bool isEvenDone() const { return even_done_; }

  //! get even filter result
  inline cv::Mat even() const {
    if (!even_done_) {
      cv::filter2D(img_, e_, cv::DataType<GT>::type, ke_, anchor, 0, cv::BORDER_REFLECT_101);
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
      P<GT, FT>::magnitude(ox_, oy_, o_);
      odd_done_ = true;
    }
    return o_;
  }

  MagnitudeRange oddRange() const { return Range<FT>(0, magnitudeMaxStep(this->intRange_.upper)); }

  FT oddThreshold(double val) const { return oddRange().upper * Magnitude<FT, FT>::single(static_cast<FT>(val)); }

  //! test if energy is computed
  inline bool isEnergyDone() const { return energy_done_; }

  //! get energy
  inline cv::Mat energy() const {
    if (!energy_done_) {
      P<GT, FT>::magnitude(odd(), even(), energy_);
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
      P<GT, FT>::phase(ox_, oy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  //! get direction range ([-PI,PI], [0,2PI] or [0,360])
  DirectionRange directionRange() const { return P<GT, FT>::range(); }

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


  //! get maximum magnitude step for intensity step
  FT magnitudeMaxStep(IT intensity = 1) const { return Magnitude<GT, FT>::max(derivativeMax(), intensity); }

  //! compute single value magnitude
  FT magnitudeSingle(FT val) const { return Magnitude<FT, FT>::single(val); }

  DerivativeMax<GT> derivativeMax() const { return gm_; }

  //! get name of gradient operator
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
