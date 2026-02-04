/// @file quadratureG2.hpp
/// @brief Second-order Gaussian quadrature filter (G2/H2) for edge detection.
///
/// This file implements a steerable quadrature filter pair using second-order
/// Gaussian derivatives (G2) and their Hilbert transform pairs (H2). The filters
/// can be steered to any orientation to detect oriented features like edges.
/// The implementation is based on Freeman & Adelson's steerable filters framework.
/// @see "The Design and Use of Steerable Filters", Freeman & Adelson, PAMI 1991

#pragma once

#include <imgproc/polar.hpp>
#include <imgproc/quadrature.hpp>
#include <opencv2/core/core.hpp>


// #define ENABLE_G2_FULL_STEER

namespace lsfm {

/// @brief Second-order Gaussian quadrature filter for steerable edge detection.
///
/// QuadratureG2 implements a quadrature filter pair using second-order Gaussian
/// derivatives (G2 - even symmetric) and their Hilbert transforms (H2 - odd symmetric).
/// These filters can be steered to any orientation using steering coefficients,
/// enabling detection of edges at arbitrary angles.
/// The G2 filter is defined by three basis functions:
/// - G21(x) = 0.9213 * (2x² - 1) * exp(-x²)
/// - G22(x) = exp(-x²)
/// - G23(x) = √1.843 * x * exp(-x²)
/// The H2 filter is defined by four basis functions:
/// - H21(x) = 0.9780 * (-2.254x + x³) * exp(-x²)
/// - H22(x) = exp(-x²)
/// - H23(x) = x * exp(-x²)
/// - H24(x) = 0.9780 * (-0.7515 + x²) * exp(-x²)
/// Key features:
/// - Separable filter implementation for efficiency
/// - Automatic dominant orientation computation
/// - Lazy computation of energy and phase
/// - Configurable kernel size and spacing
/// @tparam IT Input image type (default: uchar).
/// @tparam FT Floating-point type for computations (default: float).
/// @tparam P Polar coordinate class template (default: Polar).
/// @see Quadrature Base class providing interface.
template <class IT = uchar, class FT = float, template <typename, typename> class P = Polar>
class QuadratureG2 : public Quadrature<IT, FT, FT, FT, FT> {
 public:
  /// @brief G2 basis function 1.
  /// @param x Input coordinate.
  /// @return Function value: 0.9213 * (2x² - 1) * exp(-x²).
  static FT G21(FT x) { return static_cast<FT>(0.9213 * (2.0 * x * x - 1.0) * exp(-x * x)); }

  /// @brief G2 basis function 2 (Gaussian).
  /// @param x Input coordinate.
  /// @return Function value: exp(-x²).
  static FT G22(FT x) { return exp(-x * x); }

  /// @brief G2 basis function 3.
  /// @param x Input coordinate.
  /// @return Function value: √1.843 * x * exp(-x²).
  static FT G23(FT x) { return sqrt(static_cast<FT>(1.8430)) * x * exp(-x * x); }

  /// @brief H2 basis function 1.
  /// @param x Input coordinate.
  /// @return Function value: 0.9780 * (-2.254x + x³) * exp(-x²).
  static FT H21(FT x) { return static_cast<FT>(0.9780 * (-2.254 * x + x * x * x)) * exp(-x * x); }

  /// @brief H2 basis function 2 (Gaussian).
  /// @param x Input coordinate.
  /// @return Function value: exp(-x²).
  static FT H22(FT x) { return exp(-x * x); }

  /// @brief H2 basis function 3.
  /// @param x Input coordinate.
  /// @return Function value: x * exp(-x²).
  static FT H23(FT x) { return x * exp(-x * x); }

  /// @brief H2 basis function 4.
  /// @param x Input coordinate.
  /// @return Function value: 0.9780 * (-0.7515 + x²) * exp(-x²).
  static FT H24(FT x) { return static_cast<FT>(0.9780 * (-0.7515 + x * x)) * exp(-x * x); }

  /// Function pointer type for kernel generation.
  typedef FT (*KernelType)(FT x);

  /// @brief Create a 1D filter kernel from a basis function.
  /// @param width Kernel width (will be centered).
  /// @param spacing Sample spacing between kernel elements.
  /// @param f Basis function to evaluate.
  /// @return 1D kernel matrix.
  static cv::Mat_<FT> createFilter(int width, FT spacing, KernelType f) {
    width = width / 2;
    cv::Mat_<FT> kernel(1, width * 2 + 1);
    for (int i = -width; i <= width; ++i) kernel(i + width) = f(static_cast<FT>(i) * spacing);

    return kernel;
  }

 private:
  /// X-derivative kernel (unused in current implementation).
  cv::Mat_<FT> m_dx;
  /// Y-derivative kernel (unused in current implementation).
  cv::Mat_<FT> m_dy;

  /// G2 basis filter 1 (second derivative).
  cv::Mat_<FT> m_g1;
  /// G2 basis filter 2 (Gaussian envelope).
  cv::Mat_<FT> m_g2;
  /// G2 basis filter 3 (first derivative scaled).
  cv::Mat_<FT> m_g3;
  /// H2 basis filter 1.
  cv::Mat_<FT> m_h1;
  /// H2 basis filter 2.
  cv::Mat_<FT> m_h2;
  /// H2 basis filter 3.
  cv::Mat_<FT> m_h3;
  /// H2 basis filter 4.
  cv::Mat_<FT> m_h4;

  /// G2 filter response a (g1 * g2^T).
  cv::Mat_<FT> m_g2a;
  /// G2 filter response b (g3 * g3^T).
  cv::Mat_<FT> m_g2b;
  /// G2 filter response c (g2 * g1^T).
  cv::Mat_<FT> m_g2c;
  /// H2 filter response a.
  cv::Mat_<FT> m_h2a;
  /// H2 filter response b.
  cv::Mat_<FT> m_h2b;
  /// H2 filter response c.
  cv::Mat_<FT> m_h2c;
  /// H2 filter response d.
  cv::Mat_<FT> m_h2d;

  /// Steering coefficient 1 for full steering mode.
  cv::Mat_<FT> m_c1;
  /// Steering coefficient 2 for orientation computation.
  cv::Mat_<FT> m_c2;
  /// Steering coefficient 3 for orientation computation.
  cv::Mat_<FT> m_c3;
  /// Dominant orientation (theta).
  cv::Mat_<FT> m_theta;

#ifdef ENABLE_G2_FULL_STEER
  /// Orientation strength for full steering mode.
  cv::Mat_<FT> m_s;
#endif

  /// Cached energy image.
  mutable cv::Mat energy_;
  /// Cached phase image.
  mutable cv::Mat phase_;
  /// Cached odd response X component.
  mutable cv::Mat ox_;
  /// Cached odd response Y component.
  mutable cv::Mat oy_;
  /// Cached even filter response.
  mutable cv::Mat e_;
  /// Cached odd filter response.
  mutable cv::Mat o_;

  /// Range of odd filter responses.
  Range<FT> oddRange_;
  /// Range of even filter responses.
  Range<FT> evenRange_;
  /// Range of energy values.
  Range<FT> energyRange_;

  using Quadrature<IT, FT, FT, FT, FT>::intRange_;

  /// Kernel size.
  int ksize_;
  /// Kernel spacing.
  FT kspacing_;
  /// Filter anchor point.
  cv::Point anchor;

  /// Flag indicating energy has been computed.
  mutable bool energy_done_;
  /// Flag indicating phase has been computed.
  mutable bool phase_done_;
  /// Flag indicating odd X/Y have been computed.
  mutable bool oxy_done_;
  /// Flag indicating even/odd have been computed.
  mutable bool eo_done_;

  /// @brief Create all filter kernels.
  ///
  /// Creates separable G2 and H2 basis filters using the configured
  /// kernel size and spacing. Also applies DC offset correction.
  void create_kernels() {
    // Create separable filters for G2
    m_g1 = createFilter(ksize_, kspacing_, G21);
    m_g2 = createFilter(ksize_, kspacing_, G22);
    m_g3 = createFilter(ksize_, kspacing_, G23);

    // Create separable filters for H2
    m_h1 = createFilter(ksize_, kspacing_, H21);
    m_h2 = createFilter(ksize_, kspacing_, H22);
    m_h3 = createFilter(ksize_, kspacing_, H23);
    m_h4 = createFilter(ksize_, kspacing_, H24);

    // zero dc
#ifndef DISABLE_DC_ZERO_FIX
    m_g1 -= cv::sum(m_g1)[0] / ksize_;
    m_h4 -= cv::sum(m_h4)[0] / ksize_;
#endif
    max_response();
  }

  /// @brief Compute maximum filter responses.
  ///
  /// Creates a step edge test pattern and processes it to determine
  /// the maximum possible energy, odd, and even response values.
  void max_response() {
    cv::Mat_<IT> tmp(2 * ksize_, 2 * ksize_);
    tmp.setTo(this->intRange_.lower);
    tmp.colRange(0, ksize_).setTo(this->intRange_.upper);
    process(tmp);
    double vmin, vmax;
    cv::minMaxIdx(energy(), &vmin, &vmax);
    energyRange_.upper = static_cast<FT>(vmax);
    cv::minMaxIdx(odd(), &vmin, &vmax);
    oddRange_.upper = static_cast<FT>(vmax);
    cv::minMaxIdx(cv::abs(even()), &vmin, &vmax);
    evenRange_.upper = static_cast<FT>(vmax);
    ;
    evenRange_.lower = -evenRange_.upper;
  }

  /// @brief Initialize option manager entries.
  ///
  /// Registers kernel_size and kernel_spacing as configurable options.
  void init() {
    this->add("grad_kernel_size", std::bind(&QuadratureG2<IT, FT, P>::valueKernelSize, this, std::placeholders::_1),
              "Kernel size for Quadrature-Operators.");
    this->add("grad_kernel_spacing",
              std::bind(&QuadratureG2<IT, FT, P>::valueKernelSpacing, this, std::placeholders::_1),
              "Spacing for a single step for Quadrature-Operators.");

    create_kernels();
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

  /// @brief Construct a G2 quadrature filter with specified parameters.
  /// @param kernel_size Kernel size (default: 9, must be odd, range 3-99).
  /// @param kernel_spacing Sample spacing for kernel generation (default: 0.782).
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureG2(int kernel_size = 9,
               FT kernel_spacing = static_cast<FT>(0.782),
               IT int_lower = std::numeric_limits<IT>::lowest(),
               IT int_upper = std::numeric_limits<IT>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        m_dx(),
        m_dy(),
        m_g1(),
        m_g2(),
        m_g3(),
        m_h1(),
        m_h2(),
        m_h3(),
        m_h4(),
        m_g2a(),
        m_g2b(),
        m_g2c(),
        m_h2a(),
        m_h2b(),
        m_h2c(),
        m_h2d(),
        m_c1(),
        m_c2(),
        m_c3(),
        m_theta(),
        energy_(),
        phase_(),
        ox_(),
        oy_(),
        e_(),
        o_(),
        oddRange_(),
        evenRange_(),
        energyRange_(),
        ksize_(kernel_size),
        kspacing_(kernel_spacing),
        anchor(-1, -1),
        energy_done_(false),
        phase_done_(false),
        oxy_done_(false),
        eo_done_(false) {
    init();
  }

  /// @brief Construct from a name-value option vector.
  /// @param options Configuration options (grad_kernel_size, grad_kernel_spacing).
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureG2(const ValueManager::NameValueVector& options,
               img_type int_lower = std::numeric_limits<img_type>::lowest(),
               img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        m_dx(),
        m_dy(),
        m_g1(),
        m_g2(),
        m_g3(),
        m_h1(),
        m_h2(),
        m_h3(),
        m_h4(),
        m_g2a(),
        m_g2b(),
        m_g2c(),
        m_h2a(),
        m_h2b(),
        m_h2c(),
        m_h2d(),
        m_c1(),
        m_c2(),
        m_c3(),
        m_theta(),
        energy_(),
        phase_(),
        ox_(),
        oy_(),
        e_(),
        o_(),
        oddRange_(),
        evenRange_(),
        energyRange_(),
        ksize_(9),
        kspacing_(static_cast<FT>(0.782)),
        anchor(-1, -1),
        energy_done_(false),
        phase_done_(false),
        oxy_done_(false),
        eo_done_(false) {
    init();
    this->value(options);
  }

  /// @brief Construct from an initializer list of options.
  /// @param options Configuration options as initializer list.
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  QuadratureG2(ValueManager::InitializerList options,
               img_type int_lower = std::numeric_limits<img_type>::lowest(),
               img_type int_upper = std::numeric_limits<img_type>::max())
      : Quadrature<IT, FT, FT, FT, FT>(int_lower, int_upper),
        m_dx(),
        m_dy(),
        m_g1(),
        m_g2(),
        m_g3(),
        m_h1(),
        m_h2(),
        m_h3(),
        m_h4(),
        m_g2a(),
        m_g2b(),
        m_g2c(),
        m_h2a(),
        m_h2b(),
        m_h2c(),
        m_h2d(),
        m_c1(),
        m_c2(),
        m_c3(),
        m_theta(),
        energy_(),
        phase_(),
        ox_(),
        oy_(),
        e_(),
        o_(),
        oddRange_(),
        evenRange_(),
        energyRange_(),
        ksize_(9),
        kspacing_(static_cast<FT>(0.782)),
        anchor(-1, -1),
        energy_done_(false),
        phase_done_(false),
        oxy_done_(false),
        eo_done_(false) {
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
  ///
  /// The kernel size must be odd and in range [3, 99].
  /// Even values are automatically incremented.
  /// @param ks New kernel size.
  /// @note Large kernels may require larger gradient types (int, long long).
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
  ///
  /// The spacing controls the sampling density of the filter kernel.
  /// Smaller values give higher resolution.
  /// @param ks New kernel spacing (must be positive).
  void kernelSpacing(FT ks) {
    if (ks == kspacing_ || ks <= 0) return;
    kspacing_ = ks;
    create_kernels();
  }

  /// @brief Process an image to compute quadrature filter responses.
  ///
  /// Applies all G2 and H2 basis filters to the image and computes
  /// the dominant orientation at each pixel. Energy, phase, and
  /// steered responses are computed lazily when accessed.
  /// @param[in] img Input image.
  void process(const cv::Mat& img) {
    energy_done_ = false;
    oxy_done_ = false;
    eo_done_ = false;
    phase_done_ = false;

    // compute the outputs
    cv::sepFilter2D(img, m_g2a, cv::DataType<FT>::type, m_g1, m_g2, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, m_g2b, cv::DataType<FT>::type, m_g3, m_g3, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, m_g2c, cv::DataType<FT>::type, m_g2, m_g1, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, m_h2a, cv::DataType<FT>::type, m_h1, m_h2, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, m_h2b, cv::DataType<FT>::type, m_h4, m_h3, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, m_h2c, cv::DataType<FT>::type, m_h3, m_h4, anchor, 0, cv::BORDER_REFLECT_101);
    cv::sepFilter2D(img, m_h2d, cv::DataType<FT>::type, m_h2, m_h1, anchor, 0, cv::BORDER_REFLECT_101);

    cv::Mat g2aa = m_g2a.mul(m_g2a);  // g2a*
    cv::Mat g2ab = m_g2a.mul(m_g2b);
#ifdef ENABLE_G2_FULL_STEER
    cv::Mat g2ac = m_g2a.mul(m_g2c);
    cv::Mat g2bb = m_g2b.mul(m_g2b);  // g2b*
#endif
    cv::Mat g2bc = m_g2b.mul(m_g2c);
    cv::Mat g2cc = m_g2c.mul(m_g2c);  // g2c*
    cv::Mat h2aa = m_h2a.mul(m_h2a);  // h2a*
    cv::Mat h2ab = m_h2a.mul(m_h2b);
    cv::Mat h2ac = m_h2a.mul(m_h2c);
    cv::Mat h2ad = m_h2a.mul(m_h2d);
    cv::Mat h2bb = m_h2b.mul(m_h2b);  // h2b*
    cv::Mat h2bc = m_h2b.mul(m_h2c);
    cv::Mat h2bd = m_h2b.mul(m_h2d);
    cv::Mat h2cc = m_h2c.mul(m_h2c);  // h2c*
    cv::Mat h2cd = m_h2c.mul(m_h2d);
    cv::Mat h2dd = m_h2d.mul(m_h2d);  // h2d*

#ifdef ENABLE_G2_FULL_STEER
    m_c1 = 0.5 * (g2bb) + 0.25 * (g2ac) + 0.375 * (g2aa + g2cc) + 0.3125 * (h2aa + h2dd) + 0.5625 * (h2bb + h2cc) +
           0.375 * (h2ac + h2bd);
#endif
    m_c2 = 0.5 * (g2aa - g2cc) + 0.46875 * (h2aa - h2dd) + 0.28125 * (h2bb - h2cc) + 0.1875 * (h2ac - h2bd);
    m_c3 = (-g2ab) - g2bc - (0.9375 * (h2cd + h2ab)) - (1.6875 * (h2bc)) - (0.1875 * (h2ad));

#ifdef ENABLE_G2_FULL_STEER
    P<FT, FT>::cart2Polar(m_c2, m_c3, m_s, m_theta);
#else
    P<FT, FT>::phase(m_c2, m_c3, m_theta);
#endif
    m_theta *= -0.5;
  }

  /// @brief Get the 2D kernel used for G2 filtering.
  /// @return 2D kernel matrix (m_g1 * m_g2^T).
  cv::Mat kernel() const { return m_g1 * m_g2.t(); }

  /// @brief Get the dominant orientation at each pixel.
  /// @return Direction image with angles.
  cv::Mat direction() const { return m_theta; }

  /// @brief Get the direction value range.
  /// @return Range of direction values from Polar class.
  DirectionRange directionRange() const { return P<FT, FT>::range(); }

#ifdef ENABLE_G2_FULL_STEER
  /// @brief Get the orientation strength at each pixel.
  /// @return Strength image (only in full steer mode).
  cv::Mat strength() const { return m_s; }
#endif

  /// @brief Get the odd filter response as X and Y components.
  ///
  /// Converts the dominant orientation to unit vector components.
  /// @param[out] ox X component of odd response direction.
  /// @param[out] oy Y component of odd response direction.
  void odd(cv::Mat& ox, cv::Mat& oy) const {
    if (!oxy_done_) {
      P<FT, FT>::polar2Cart(m_theta, ox_, oy_);
      oxy_done_ = true;
    }
    ox = ox_;
    oy = oy_;
  }

  /// @brief Steer the filters to a given orientation image.
  ///
  /// Computes the G2 and H2 responses at the orientations specified
  /// in the theta image using the steering equations.
  /// @param[in] theta Orientation image (angles).
  /// @param[out] g2 G2 (even) filter response at theta.
  /// @param[out] h2 H2 (odd) filter response at theta.
  void steer(const cv::Mat& theta, cv::Mat& g2, cv::Mat& h2) const {
    // Create the steering coefficients, then compute G2 and H2 at orientation theta:
    cv::Mat ct, ct2, ct3, st, st2, st3;
    P<FT, FT>::polarToCart(-theta, ct, st);
    ct2 = ct.mul(ct), ct3 = ct2.mul(ct), st2 = st.mul(st), st3 = st2.mul(st);
    g2 = ct2.mul(m_g2a) + (-2.0 * ct.mul(st).mul(m_g2b)) + (st2.mul(m_g2c));
    h2 = ct3.mul(m_h2a) + (-3.0 * ct2.mul(st).mul(m_h2b)) + (3.0 * ct.mul(st2).mul(m_h2c)) + (-st3.mul(m_h2d));
  }

  /// @brief Steer filters using pre-computed gradient components.
  ///
  /// Computes the G2 and H2 responses using gradient X/Y components
  /// as steering direction (avoids trigonometric functions).
  /// @param[in] gx X-gradient component (cos(theta)).
  /// @param[in] gy Y-gradient component (sin(theta)).
  /// @param[out] g2 G2 (even) filter response.
  /// @param[out] h2 H2 (odd) filter response.
  void steer(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& g2, cv::Mat& h2) const {
    // Create the steering coefficients, then compute G2 and H2 at orientation theta:
    cv::Mat ct2, ct3, st2, st3, mgy = -gy;
    ct2 = gx.mul(gx), ct3 = ct2.mul(gx), st2 = mgy.mul(mgy), st3 = st2.mul(mgy);
    g2 = ct2.mul(m_g2a) + (-2.0 * gx.mul(mgy).mul(m_g2b)) + (st2.mul(m_g2c));
    h2 = ct3.mul(m_h2a) + (-3.0 * ct2.mul(mgy).mul(m_h2b)) + (3.0 * gx.mul(st2).mul(m_h2c)) + (-st3.mul(m_h2d));
  }

  /// @brief Check if even/odd responses have been computed.
  /// @return True if even() or odd() has been called, false otherwise.
  inline bool isEvenDone() const { return eo_done_; }

  /// @brief Get the even filter response at dominant orientation.
  ///
  /// Returns the G2 (even symmetric) response steered to the
  /// dominant orientation at each pixel.
  /// @return Even filter response image.
  cv::Mat even() const {
    if (!eo_done_) {
      cv::Mat t1, t2;
      evenOdd(t1, t2);
    }
    return e_;
  }

  /// @brief Get the even filter response range.
  /// @return Range of even filter values.
  GradientRange evenRange() const { return evenRange_; }

  /// @brief Check if odd responses have been computed.
  /// @return True if even/odd responses have been computed.
  inline bool isOddDone() const { return eo_done_; }

  /// @brief Get the odd filter response at dominant orientation.
  ///
  /// Returns the H2 (odd symmetric) response steered to the
  /// dominant orientation at each pixel. The absolute value is returned.
  /// @return Odd filter response image (absolute values).
  cv::Mat odd() const {
    if (!eo_done_) {
      cv::Mat t1, t2;
      evenOdd(t1, t2);
    }
    return o_;
  }

  /// @brief Get both even and odd filter responses at dominant orientation.
  ///
  /// Steers both G2 and H2 filters to the dominant orientation and
  /// returns the responses.
  /// @param[out] e Even (G2) filter response.
  /// @param[out] o Odd (H2) filter response (absolute value).
  void evenOdd(cv::Mat& e, cv::Mat& o) const {
    if (!eo_done_) {
      steer(this->oddx(), oy_, e_, o_);
      o_ = abs(o_);
      eo_done_ = true;
    }
    e = e_;
    o = o_;
  }

  /// @brief Get the odd filter response range.
  /// @return Range of odd filter magnitude values.
  MagnitudeRange oddRange() const { return oddRange_; }

  /// @brief Check if energy has been computed.
  /// @return True if energy() has been called, false otherwise.
  inline bool isEnergyDone() const { return energy_done_; }

  /// @brief Get the local energy.
  ///
  /// Local energy is computed as the magnitude of the complex
  /// quadrature response (even + i*odd).
  /// @return Energy image.
  cv::Mat energy() const {
    if (!energy_done_) {
      P<FT, FT>::magnitude(odd(), e_, energy_);
      energy_done_ = true;
    }
    return energy_;
  }

  /// @brief Get the energy value range.
  /// @return Range of energy values.
  EnergyRange energyRange() const { return energyRange_; }

  /// @brief Check if phase has been computed.
  /// @return True if phase() has been called, false otherwise.
  inline bool isPhaseDone() const { return phase_done_; }

  /// @brief Get the local phase.
  ///
  /// Local phase is the angle of the complex quadrature response.
  /// It indicates the type of feature (edge, line, etc.).
  /// @return Phase image.
  cv::Mat phase() const {
    if (!phase_done_) {
      P<FT, FT>::phase(odd(), e_, phase_);
      phase_done_ = true;
    }
    return phase_;
  }

  /// @brief Get the phase value range.
  /// @return Range of phase values from Polar class.
  PhaseRange phaseRange() const { return P<FT, FT>::range(); }

  /// @brief Get both energy and phase.
  ///
  /// Efficiently computes both energy and phase together using
  /// a single cart2Polar call if neither has been computed yet.
  /// @param[out] en Energy image.
  /// @param[out] ph Phase image.
  void energyPhase(cv::Mat& en, cv::Mat& ph) const {
    if (!phase_done_ && !energy_done_) {
      P<FT, FT>::cart2Polar(odd(), e_, energy_, phase_);
      phase_done_ = true;
      energy_done_ = true;
    } else if (!phase_done_) {
      phase();
    } else if (!energy_done_) {
      energy();
    }
    en = energy_;
    ph = phase_;
  }


  /// @brief Get the name of this gradient operator.
  /// @return "quadratureG2".
  inline std::string name() const { return "quadratureG2"; }

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

#ifdef ENABLE_G2_FULL_STEER
  /// @brief Compute energy and phase from G2/H2 responses.
  /// @param[in] g2 G2 (even) filter response.
  /// @param[in] h2 H2 (odd) filter response.
  /// @param[out] energy Energy (magnitude).
  /// @param[out] phase Phase (angle).
  void computeEnergyAndPhase(const cv::Mat& g2, const cv::Mat& h2, cv::Mat& energy, cv::Mat& phase) const {
    P<FT, FT>::cart2Polar(g2, h2, energy, phase);
    // cv::cartToPolar(g2, h2, energy, phase); // [0..2*piI]
    // wrap(phase, phase); // [-pi/2 pi/2]
    // cv::patchNaNs(phase);
    // phase.setTo(10000, energy < 1e-2f);
  }

  /// @brief Steer filters at a single pixel.
  /// @param[in] p Pixel location.
  /// @param[in] theta Orientation angle.
  /// @param[out] g2 G2 response at p steered to theta.
  /// @param[out] h2 H2 response at p steered to theta.
  void steer(const cv::Point& p, FT theta, FT& g2, FT& h2) const {
    // Create the steering coefficients, then compute G2 and H2 at orientation theta:
    FT ct(std::cos(theta)), ct2(ct * ct), ct3(ct2 * ct), st(std::sin(theta)), st2(st * st), st3(st2 * st);
    FT ga(ct2), gb(-2.0 * ct * st), gc(st2);
    FT ha(ct3), hb(-3.0 * ct2 * st), hc(3.0 * ct * st2), hd(-st3);
    g2 = ga * m_g2a(p) + gb * m_g2b(p) + gc * m_g2c(p);
    h2 = ha * m_h2a(p) + hb * m_h2b(p) + hc * m_h2c(p) + hd * m_h2d(p);
  }

  /// @brief Steer filters at a single pixel with full output.
  /// @param[in] p Pixel location.
  /// @param[in] theta Orientation angle.
  /// @param[out] g2 G2 response.
  /// @param[out] h2 H2 response.
  /// @param[out] e Oriented energy.
  /// @param[out] energy Quadrature energy.
  /// @param[out] phase Quadrature phase.
  void steer(const cv::Point& p, FT theta, FT& g2, FT& h2, FT& e, FT& energy, FT& phase) const {
    steer(p, theta, g2, h2);
    phase = std::atan2(h2, g2);
    energy = std::sqrt(h2 * h2 + g2 * g2);

    // Compute oriented energy as a function of angle theta
    FT c2t(std::cos(theta * 2.0)), s2t(std::sin(theta * 2.0));
    e = m_c1(p) + (c2t * m_c2(p)) + (s2t * m_c3(p));
  }

  /// @brief Steer filters across entire image to a single angle.
  /// @param[in] theta Orientation angle.
  /// @param[out] g2 G2 response image.
  /// @param[out] h2 H2 response image.
  void steer(FT theta, cv::Mat& g2, cv::Mat& h2) const {
    // Create the steering coefficients, then compute G2 and H2 at orientation theta:
    FT ct(std::cos(theta)), ct2(ct * ct), ct3(ct2 * ct), st(std::sin(theta)), st2(st * st), st3(st2 * st);
    FT ga(ct2), gb(-2.0 * ct * st), gc(st2);
    FT ha(ct3), hb(-3.0 * ct2 * st), hc(3.0 * ct * st2), hd(-st3);
    g2 = ga * m_g2a + gb * m_g2b + gc * m_g2c;
    h2 = ha * m_h2a + hb * m_h2b + hc * m_h2c + hd * m_h2d;
  }

  /// @brief Steer filters to single angle with full output.
  /// @param[in] theta Orientation angle.
  /// @param[out] g2 G2 response image.
  /// @param[out] h2 H2 response image.
  /// @param[out] e Oriented energy image.
  /// @param[out] energy Quadrature energy image.
  /// @param[out] phase Quadrature phase image.
  void steer(FT theta, cv::Mat& g2, cv::Mat& h2, cv::Mat& e, cv::Mat& energy, cv::Mat& phase) const {
    steer(theta, g2, h2);
    computeEnergyAndPhase(g2, h2, energy, phase);

    // Compute oriented energy as a function of angle theta
    FT c2t(std::cos(theta * 2.0)), s2t(std::sin(theta * 2.0));
    e = m_c1 + (c2t * m_c2) + (s2t * m_c3);
  }

  /// @brief Steer filters to orientation image with full output.
  /// @param[in] theta Orientation image (angle at each pixel).
  /// @param[out] g2 G2 response image.
  /// @param[out] h2 H2 response image.
  /// @param[out] e Oriented energy image.
  /// @param[out] energy Quadrature energy image.
  /// @param[out] phase Quadrature phase image.
  void steer(const cv::Mat& theta, cv::Mat& g2, cv::Mat& h2, cv::Mat& e, cv::Mat& energy, cv::Mat& phase) const {
    // Create the steering coefficients, then compute G2 and H2 at orientation theta:
    steer(theta, g2, h2);
    computeEnergyAndPhase(g2, h2, energy, phase);

    // Compute oriented energy as a function of angle theta
    cv::Mat c2t, s2t;
    P<FT, FT>::polar2Cart(theta * 2.0, c2t, s2t);
    e = m_c1 + m_c2.mul(c2t) + m_c3.mul(s2t);
  }
#endif
};

}  // namespace lsfm
