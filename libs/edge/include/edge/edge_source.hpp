//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file edge_source.hpp
/// @brief Abstract interface and implementations for edge response computation.
/// Provides a flexible framework for combining different edge response filters
/// (Sobel, Laplace, Quadrature) with edge pixel extraction methods (NMS, Zero-Crossing).

#pragma once

#include <edge/zc.hpp>
#include <imgproc/gradient.hpp>
#include <imgproc/laplace.hpp>
#include <imgproc/phase_congruency.hpp>

namespace lsfm {

/// @brief Abstract interface for edge source computation.
/// Provides a unified interface for combining arbitrary edge response methods
/// (Sobel, Laplace, quadrature, etc.) with different edgel extraction methods
/// (NMS, zero-crossing) and more complex combinations.
class EdgeSourceI : public ValueManager {
  EdgeSourceI(const EdgeSourceI&);

 protected:
  EdgeSourceI() {}

 public:
  virtual ~EdgeSourceI() {}

  /// @brief Process an input image to compute edge responses.
  /// @param img Input grayscale image
  virtual void process(const cv::Mat& img) = 0;

  /// @brief Get all processed filter results.
  /// @return Map of named filter result matrices
  virtual FilterResults results() const = 0;

  /// @brief Get the gradient in the X direction.
  /// @return Gradient matrix in X direction
  virtual cv::Mat gx() const = 0;

  /// @brief Get the gradient in the Y direction.
  /// @return Gradient matrix in Y direction
  virtual cv::Mat gy() const = 0;

  /// @brief Get the edge magnitude map.
  /// @return Magnitude matrix
  virtual cv::Mat magnitude() const = 0;

  /// @brief Convert a relative threshold value to an absolute magnitude threshold.
  /// @param val Relative threshold value (0-1 range or domain-specific)
  /// @return Absolute magnitude threshold
  virtual double magnitudeThreshold(double val) const = 0;

  /// @brief Get the maximum magnitude value.
  /// @return Maximum magnitude found in the processed image
  virtual double magnitudeMax() const = 0;

  /// @brief Get the edge direction map.
  /// @return Direction matrix (continuous angles)
  virtual cv::Mat direction() const = 0;

  /// @brief Get the valid range of direction values.
  /// @return Range with lower and upper bounds for direction values
  virtual Range<double> directionRange() const = 0;

  /// @brief Get the quantized direction map for edge processing.
  /// @return Direction map (CV_8S) with discrete direction indices
  virtual cv::Mat directionMap() const = 0;

  /// @brief Get seed pixel indices for edge processing.
  /// @return Const reference to vector of seed edgel indices
  virtual const IndexVector& seeds() const = 0;

  /// @brief Get the name of this edge source.
  /// @return Descriptive string identifier
  virtual std::string name() const = 0;

  /// @brief Compute hysteresis-linked direction map from seeds and direction map.
  /// @return Direction map with only hysteresis-linked edgels retaining their values
  cv::Mat hysteresis() const { return lsfm::hysteresis(directionMap(), seeds()); }

  /// @brief Compute binary hysteresis map from seeds and direction map.
  /// @return Binary edge map marking all hysteresis-linked edgels
  cv::Mat hysteresis_binary() const { return lsfm::hysteresis_binary(directionMap(), seeds()); }

  /// @brief Compute hysteresis-linked edgel indices from seeds and direction map.
  /// @return Vector of all edgel indices after hysteresis linking
  IndexVector hysteresis_edgels() const {
    IndexVector edgels = seeds();
    lsfm::hysteresis_edgels(directionMap(), edgels);
    return edgels;
  }
};

/// @brief Templated edge source combining an edge response filter with a pixel extractor.
/// Connects an edge response filter (ERF) producing gradients with a pixel extraction
/// method (EPE) such as NMS or zero-crossing detection.
/// @tparam ERF Edge response filter type (e.g., Gradient, Quadrature)
/// @tparam EPE Edge pixel extractor type (e.g., NMS, ZC)
template <class ERF, class EPE>
class EdgeSourceT : public EdgeSourceI {
  EdgeSourceT(const EdgeSourceT&);

  inline void init() {
    this->addManager(erf_);
    this->addManager(epe_);
  }

 protected:
  EdgeSourceT() : erf_(), epe_() { init(); }

  EdgeSourceT(const ValueManager::NameValueVector& options) {
    init();
    this->value(options);
  }

  EdgeSourceT(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  ERF erf_;
  EPE epe_;

 public:
  /// @typedef EdgeResponseFilter
  /// @brief Type alias for the edge response filter
  typedef ERF EdgeResponseFilter;

  /// @typedef EdgePixelExtractor
  /// @brief Type alias for the edge pixel extractor
  typedef EPE EdgePixelExtractor;

  /// @brief Number of quantized directions supported by the pixel extractor.
  static constexpr int NUM_DIR = EPE::NUM_DIR;

  using EdgeSourceI::hysteresis;
  using EdgeSourceI::hysteresis_edgels;

  virtual ~EdgeSourceT() {}

  /// @brief Get const reference to the edge response filter.
  /// @return Const reference to the filter
  const ERF& responseFilter() const { return erf_; }

  /// @brief Get mutable reference to the edge response filter.
  /// @return Mutable reference to the filter
  ERF& responseFilter() { return erf_; }

  /// @brief Get const reference to the pixel extractor.
  /// @return Const reference to the extractor
  const EPE& pixelExtractor() const { return epe_; }

  /// @brief Get mutable reference to the pixel extractor.
  /// @return Mutable reference to the extractor
  EPE& pixelExtractor() { return epe_; }

  virtual FilterResults results() const override { return erf_.results(); }

  virtual cv::Mat direction() const override { return erf_.direction(); }

  virtual Range<double> directionRange() const override {
    auto tmp = erf_.directionRange();
    return Range<double>(tmp.lower, tmp.upper);
  }

  virtual std::string name() const override { return "es_" + erf_.name() + "_" + epe_.name(); }
};

/// @brief Edge source base for NMS and ZC pixel extraction.
/// Intermediate class providing direction map and seed accessors from the pixel extractor.
/// @tparam ERF Edge response filter type
/// @tparam NMS_ZC NMS or zero-crossing pixel extractor type
template <class ERF, class NMS_ZC>
class EdgeSourceNMS_ZC : public EdgeSourceT<ERF, NMS_ZC> {
  EdgeSourceNMS_ZC(const EdgeSourceNMS_ZC&);

 protected:
  using EdgeSourceT<ERF, NMS_ZC>::erf_;
  using EdgeSourceT<ERF, NMS_ZC>::epe_;

  EdgeSourceNMS_ZC() {}
  EdgeSourceNMS_ZC(const ValueManager::NameValueVector& options) : EdgeSourceT<ERF, NMS_ZC>(options) {}
  EdgeSourceNMS_ZC(ValueManager::InitializerList& options) : EdgeSourceT<ERF, NMS_ZC>(options) {}

 public:
  /// @brief Get the quantized direction map from the pixel extractor.
  /// @return Direction map (CV_8S)
  virtual cv::Mat directionMap() const override { return epe_.directionMap(); }

  /// @brief Get seed indices from the pixel extractor.
  /// @return Const reference to seed index vector
  virtual const IndexVector& seeds() const override { return epe_.seeds(); }
};

/// @brief Edge source base for NMS pixel extraction.
/// Provides magnitude max accessor from the NMS extractor.
/// @tparam ERF Edge response filter type
/// @tparam NMS Non-maximum suppression pixel extractor type
template <class ERF, class NMS>
class EdgeSourceNMS : public EdgeSourceNMS_ZC<ERF, NMS> {
  EdgeSourceNMS(const EdgeSourceNMS&);

 protected:
  using EdgeSourceT<ERF, NMS>::erf_;
  using EdgeSourceT<ERF, NMS>::epe_;

  EdgeSourceNMS() {}
  EdgeSourceNMS(const ValueManager::NameValueVector& options) : EdgeSourceNMS_ZC<ERF, NMS>(options) {}
  EdgeSourceNMS(ValueManager::InitializerList& options) : EdgeSourceNMS_ZC<ERF, NMS>(options) {}

 public:
  /// @brief Get the maximum magnitude value from the NMS extractor.
  /// @return Maximum magnitude as double
  virtual double magnitudeMax() const override { return static_cast<double>(epe_.magMax()); }
};

/// @brief Edge source base for zero-crossing pixel extraction.
/// Provides lazy-computed magnitude max via cv::minMaxLoc.
/// @tparam ERF Edge response filter type
/// @tparam ZC Zero-crossing pixel extractor type
template <class ERF, class ZC>
class EdgeSourceZC : public EdgeSourceNMS_ZC<ERF, ZC> {
  EdgeSourceZC(const EdgeSourceZC&);

 protected:
  using EdgeSourceT<ERF, ZC>::erf_;
  using EdgeSourceT<ERF, ZC>::epe_;
  mutable double magMax_;  ///< Cached maximum magnitude (-1 = not yet computed)

  EdgeSourceZC() : magMax_(-1) {}
  EdgeSourceZC(const ValueManager::NameValueVector& options) : EdgeSourceNMS_ZC<ERF, ZC>(options), magMax_(-1) {}
  EdgeSourceZC(ValueManager::InitializerList& options) : EdgeSourceNMS_ZC<ERF, ZC>(options), magMax_(-1) {}

 public:
  /// @brief Get the maximum magnitude value, computing it lazily if needed.
  /// @return Maximum magnitude as double
  virtual double magnitudeMax() const override {
    if (magMax_ < 0) {
      double vmin;
      cv::minMaxLoc(this->magnitude(), &vmin, &magMax_);
    }
    return magMax_;
  }
};

/// @brief Gradient-based edge source using NMS for edgel extraction.
/// Combines a gradient filter (Sobel, Scharr, etc.) with non-maximum suppression.
/// Optionally uses direction instead of gx/gy for NMS computation.
/// @tparam GRAD Gradient filter type
/// @tparam NMS Non-maximum suppression type
template <class GRAD, class NMS>
class EdgeSourceGRAD : public EdgeSourceNMS<GRAD, NMS> {
  EdgeSourceGRAD(const EdgeSourceGRAD&);
  using EdgeSourceT<GRAD, NMS>::erf_;
  using EdgeSourceT<GRAD, NMS>::epe_;

  bool useDir_;

  inline void init() {
    this->add("es_use_dir", std::bind(&EdgeSourceGRAD<GRAD, NMS>::valueUseDir, this, std::placeholders::_1),
              "If true, uses direction for non maxima suppression instead of gx and gy.");
  }

 public:
  EdgeSourceGRAD(bool useDir = false) : useDir_(useDir) { init(); }
  EdgeSourceGRAD(const ValueManager::NameValueVector& options) : useDir_(false) {
    init();
    this->value(options);
  }
  EdgeSourceGRAD(ValueManager::InitializerList& options) : useDir_(false) {
    init();
    this->value(options);
  }

  Value valueUseDir(const Value& v = Value::NAV()) {
    if (v.type()) useDir_ = (v.getInt() != 0);
    return (useDir_ ? 1 : 0);
  }

  using EdgeSourceT<GRAD, NMS>::responseFilter;
  using EdgeSourceT<GRAD, NMS>::pixelExtractor;
  using EdgeSourceI::hysteresis;
  using EdgeSourceI::hysteresis_edgels;

  /// @brief Process edge source from input image.
  /// Computes gradient response and applies NMS to extract edge pixels.
  /// @param img Input grayscale image
  virtual void process(const cv::Mat& img) override {
    erf_.process(img);
    epe_.process(erf_, useDir_);
  }

  /// @brief Get gradient in X direction.
  /// @return Gradient X matrix
  virtual cv::Mat gx() const override { return erf_.gx(); }

  /// @brief Get gradient in Y direction.
  /// @return Gradient Y matrix
  virtual cv::Mat gy() const override { return erf_.gy(); }

  /// @brief Get edge magnitude map.
  /// @return Magnitude matrix
  virtual cv::Mat magnitude() const override { return erf_.magnitude(); }

  /// @brief Convert relative threshold to absolute magnitude threshold.
  /// @param val Relative threshold value
  /// @return Absolute magnitude threshold
  virtual double magnitudeThreshold(double val) const override {
    return static_cast<double>(erf_.magnitudeThreshold(val));
  }
};

/// @brief Direction options for edge sources supporting multiple direction modes.
enum ESDirectionOptions {
  ESDO_NONE = 0,  ///< No direction enrichment
  ESDO_GXGY,      ///< Use gradient components (gx, gy) for direction
  ESDO_DIR        ///< Use pre-computed direction map
};

/// @brief Laplace-based edge source using zero-crossing for edgel extraction.
/// Combines a Laplacian filter with gradient computation and zero-crossing detection.
/// @tparam LAPLACE Laplacian filter type
/// @tparam GRAD Gradient filter type
/// @tparam ZC Zero-crossing detector type
template <class LAPLACE, class GRAD, class ZC>
class EdgeSourceLAPLACE : public EdgeSourceZC<GRAD, ZC> {
  EdgeSourceLAPLACE(const EdgeSourceLAPLACE&);

  inline void init() {
    this->addManager(laplace_);
    this->add("es_dir_ops", std::bind(&EdgeSourceLAPLACE<LAPLACE, GRAD, ZC>::valueDirOps, this, std::placeholders::_1),
              "Set direction options: None - 0, gxgy - 1 or dir - 2.");
  }

 protected:
  using EdgeSourceT<GRAD, ZC>::erf_;
  using EdgeSourceT<GRAD, ZC>::epe_;
  using EdgeSourceZC<GRAD, ZC>::magMax_;

  LAPLACE laplace_;
  mutable FilterResults fresults_;
  ESDirectionOptions dirOps_;

 public:
  EdgeSourceLAPLACE(ESDirectionOptions dirOp = ESDO_NONE) : dirOps_(dirOp) {}
  EdgeSourceLAPLACE(const ValueManager::NameValueVector& options) : dirOps_(ESDO_NONE) {
    init();
    this->value(options);
  }
  EdgeSourceLAPLACE(ValueManager::InitializerList& options) : dirOps_(ESDO_NONE) {
    init();
    this->value(options);
  }

  Value valueDirOps(const Value& v = Value::NAV()) {
    if (v.type()) dirOps_ = static_cast<ESDirectionOptions>(v.getInt());
    return static_cast<int>(dirOps_);
  }

  using EdgeSourceT<GRAD, ZC>::responseFilter;
  using EdgeSourceT<GRAD, ZC>::pixelExtractor;
  using EdgeSourceI::hysteresis;
  using EdgeSourceI::hysteresis_edgels;

  virtual FilterResults results() const override {
    if (fresults_.empty()) {
      fresults_ = laplace_.results();
      FilterResults tmp = erf_.results();
      fresults_.insert(tmp.begin(), tmp.end());
    }
    return fresults_;
  }

  /// @brief Process edge source from input image.
  /// Computes quadrature filter and Laplace response, then extracts edges.
  /// @param img Input grayscale image
  virtual void process(const cv::Mat& img) override {
    fresults_.clear();
    magMax_ = -1;
    erf_.process(img);
    laplace_.process(img);
    switch (dirOps_) {
      case ESDO_GXGY:
        epe_.processG(laplace_, erf_);
        break;
      case ESDO_DIR:
        epe_.processD(laplace_, erf_);
        break;
      default:
        epe_.process(laplace_);
        break;
    }
  }

  /// @brief Get gradient in X direction.
  /// @return Gradient X matrix
  virtual cv::Mat gx() const override { return erf_.gx(); }

  /// @brief Get gradient in Y direction.
  /// @return Gradient Y matrix
  virtual cv::Mat gy() const override { return erf_.gy(); }

  /// @brief Get edge magnitude map.
  /// @return Magnitude matrix from gradient filter
  virtual cv::Mat magnitude() const override { return erf_.magnitude(); }

  /// @brief Convert relative threshold to absolute magnitude threshold.
  /// @param val Relative threshold value
  /// @return Absolute magnitude threshold
  virtual double magnitudeThreshold(double val) const override {
    return static_cast<double>(erf_.magnitudeThreshold(val));
  }
};

/// @brief Quadrature magnitude options for quadrature-based edge sources.
enum ESQuadratureOptions {
  ESQO_MAG = 0,  ///< Use odd response magnitude
  ESQO_ENERGY,   ///< Use energy response
  ESQO_PC,       ///< Use phase congruency
};

/// @brief Quadrature-based edge source using odd response and NMS.
/// Combines quadrature filter with non-maximum suppression for edge detection.
/// Supports multiple magnitude modes (odd, energy, phase congruency).
/// @tparam QUAD Quadrature filter type
/// @tparam NMS Non-maximum suppression type
template <class QUAD, class NMS>
class EdgeSourceQUAD : public EdgeSourceNMS<QUAD, NMS> {
  EdgeSourceQUAD(const EdgeSourceQUAD&);

  inline void init() {
    this->add("es_use_dir", std::bind(&EdgeSourceQUAD<QUAD, NMS>::valueUseDir, this, std::placeholders::_1),
              "If true, uses direction for non maxima suppression instead of gx and gy.");
    this->add("es_quad_ops", std::bind(&EdgeSourceQUAD<QUAD, NMS>::valueQuadOps, this, std::placeholders::_1),
              "Select quadrature magnitude: odd magnitude - 0, energy - 1, pc - 2 (if available).");
  }

 protected:
  using EdgeSourceT<QUAD, NMS>::erf_;
  using EdgeSourceT<QUAD, NMS>::epe_;

  bool useDir_;
  ESQuadratureOptions quadOps_;

 public:
  EdgeSourceQUAD(ESQuadratureOptions quadOps = ESQO_MAG, bool useDir = false) : useDir_(useDir), quadOps_(quadOps) {
    init();
  }
  EdgeSourceQUAD(const ValueManager::NameValueVector& options) : useDir_(false), quadOps_(ESQO_MAG) {
    init();
    this->value(options);
  }
  EdgeSourceQUAD(ValueManager::InitializerList& options) : useDir_(false), quadOps_(ESQO_MAG) {
    init();
    this->value(options);
  }

  Value valueUseDir(const Value& v = Value::NAV()) {
    if (v.type()) useDir_ = (v.getInt() != 0);
    return useDir_ ? 1 : 0;
  }
  Value valueQuadOps(const Value& v = Value::NAV()) {
    if (v.type()) quadOps_ = static_cast<ESQuadratureOptions>(v.getInt());
    return static_cast<int>(quadOps_);
  }

  using EdgeSourceT<QUAD, NMS>::responseFilter;
  using EdgeSourceT<QUAD, NMS>::pixelExtractor;
  using EdgeSourceI::hysteresis;
  using EdgeSourceI::hysteresis_edgels;

  /// @brief Process edge source from input image.
  /// Computes quadrature filter response and applies NMS with appropriate thresholds.
  /// @param img Input grayscale image
  virtual void process(const cv::Mat& img) override {
    high = magnitudeThreshold(high);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    if (useDir_) {
      typename QUAD::DirectionRange dr = erf_.directionRange();
      epe_.process(erf_.direction(), magnitude(), low, high, dr.lower, dr.upper);
    } else {
      epe_.process(erf_.oddx(), erf_.oddy(), magnitude(), low, high);
    }
  }

  /// @brief Get gradient in X direction (odd X component).
  /// @return Odd X response matrix
  virtual cv::Mat gx() const override { return erf_.oddx(); }

  /// @brief Get gradient in Y direction (odd Y component).
  /// @return Odd Y response matrix
  virtual cv::Mat gy() const override { return erf_.oddy(); }

  /// @brief Get edge magnitude map based on quadrature options.
  /// Returns energy response if ESQO_ENERGY, otherwise odd response magnitude.
  /// @return Magnitude matrix
  virtual cv::Mat magnitude() const override {
    if (quadOps_ == ESQO_ENERGY) return erf_.energy();
    return erf_.odd();
  }

  /// @brief Convert relative threshold to absolute magnitude threshold.
  /// @param val Relative threshold value
  /// @return Absolute magnitude threshold
  virtual double magnitudeThreshold(double val) const override {
    if (quadOps_ == ESQO_ENERGY) return static_cast<double>(erf_.energyThreshold(val));
    return static_cast<double>(erf_.oddThreshold(val));
  }
};

/// @brief Phase congruency edge source using odd response and NMS.
/// Extends EdgeSourceQUAD with phase congruency magnitude support.
/// @tparam PC Phase congruency filter type
/// @tparam NMS Non-maximum suppression type
template <class PC, class NMS>
class EdgeSourcePC : public EdgeSourceQUAD<PC, NMS> {
  EdgeSourcePC(const EdgeSourcePC&);

 protected:
  using EdgeSourceT<PC, NMS>::erf_;
  using EdgeSourceT<PC, NMS>::epe_;
  using EdgeSourceQUAD<PC, NMS>::quadOps_;

 public:
  EdgeSourcePC(ESQuadratureOptions quadOps = ESQO_PC, bool useDir = false) : EdgeSourceQUAD<PC, NMS>(quadOps, useDir) {}
  EdgeSourcePC(const ValueManager::NameValueVector& options) {
    quadOps_ = ESQO_PC;
    this->value(options);
  }
  EdgeSourcePC(ValueManager::InitializerList& options) {
    quadOps_ = ESQO_PC;
    this->value(options);
  }

  using EdgeSourceT<PC, NMS>::responseFilter;
  using EdgeSourceT<PC, NMS>::pixelExtractor;
  using EdgeSourceI::hysteresis;
  using EdgeSourceI::hysteresis_edgels;


  /// @brief Get edge magnitude map based on quadrature options.
  /// Returns odd magnitude, energy, or phase congruency depending on quadOps_.
  /// @return Magnitude matrix
  virtual cv::Mat magnitude() const override {
    switch (quadOps_) {
      case ESQO_MAG:
        return erf_.odd();
      case ESQO_ENERGY:
        return erf_.energy();
      case ESQO_PC:
        return erf_.phaseCongruency();
    }
    return erf_.phaseCongruency();
  }

  /// @brief Convert relative threshold to absolute magnitude threshold.
  /// @param val Relative threshold value
  /// @return Absolute magnitude threshold based on selected mode
  virtual double magnitudeThreshold(double val) const override {
    switch (quadOps_) {
      case ESQO_MAG:
        return erf_.oddThreshold(val);
      case ESQO_ENERGY:
        return erf_.energyThreshold(val);
      case ESQO_PC:
        return val;
    }
    return val;
  }
};

/// @brief Quadrature-based edge source using even response and zero-crossing.
/// Combines quadrature filter with zero-crossing detection for edge extraction.
/// @tparam QUAD Quadrature filter type
/// @tparam ZC Zero-crossing detector type
template <class QUAD, class ZC>
class EdgeSourceQUADZC : public EdgeSourceZC<QUAD, ZC> {
  EdgeSourceQUADZC(const EdgeSourceQUADZC&);

  inline void init() {
    this->add("es_dir_ops", std::bind(&EdgeSourceQUADZC<QUAD, ZC>::valueDirOps, this, std::placeholders::_1),
              "Set direction options: None - 0, gxgy - 1 or dir - 2.");
    this->add("es_quad_ops", std::bind(&EdgeSourceQUADZC<QUAD, ZC>::valueQuadOps, this, std::placeholders::_1),
              "Select quadrature magnitude: odd magnitude - 0, energy - 1, pc - 2 (if available).");
  }

 protected:
  using EdgeSourceT<QUAD, ZC>::erf_;
  using EdgeSourceT<QUAD, ZC>::epe_;
  using EdgeSourceZC<QUAD, ZC>::magMax_;
  ESDirectionOptions dirOps_;
  ESQuadratureOptions quadOps_;

 public:
  EdgeSourceQUADZC(ESQuadratureOptions quadOps = ESQO_MAG, ESDirectionOptions dirOps = ESDO_NONE)
      : dirOps_(dirOps), quadOps_(quadOps) {
    init();
  }
  EdgeSourceQUADZC(const ValueManager::NameValueVector& options) : dirOps_(ESDO_NONE), quadOps_(ESQO_MAG) {
    init();
    this->value(options);
  }
  EdgeSourceQUADZC(ValueManager::InitializerList& options) : dirOps_(ESDO_NONE), quadOps_(ESQO_MAG) {
    init();
    this->value(options);
  }

  Value valueDirOps(const Value& v = Value::NAV()) {
    if (v.type()) dirOps_ = static_cast<ESDirectionOptions>(v.getInt());
    return static_cast<int>(dirOps_);
  }
  Value valueQuadOps(const Value& v = Value::NAV()) {
    if (v.type()) quadOps_ = static_cast<ESQuadratureOptions>(v.getInt());
    return static_cast<int>(quadOps_);
  }

  using EdgeSourceT<QUAD, ZC>::responseFilter;
  using EdgeSourceT<QUAD, ZC>::pixelExtractor;
  using EdgeSourceI::hysteresis;
  using EdgeSourceI::hysteresis_edgels;

  /// @brief Process edge source from input image.
  /// Computes quadrature filter response and applies zero-crossing detection.
  /// @param img Input grayscale image
  virtual void process(const cv::Mat& img) override {
    magMax_ = -1;
    erf_.process(img);
    switch (dirOps_) {
      case ESDO_GXGY:
        epe_.processQ(erf_);
        break;
      case ESDO_DIR:
        epe_.processQD(erf_);
        break;
      default:
        epe_.process(erf_);
        break;
    }
  }

  /// @brief Get gradient in X direction (odd X component).
  /// @return Odd X response matrix
  virtual cv::Mat gx() const override { return erf_.oddx(); }

  /// @brief Get gradient in Y direction (odd Y component).
  /// @return Odd Y response matrix
  virtual cv::Mat gy() const override { return erf_.oddy(); }

  /// @brief Get edge magnitude map based on quadrature options.
  /// @return Magnitude matrix (energy or odd response)
  virtual cv::Mat magnitude() const override {
    if (quadOps_ == ESQO_ENERGY) return erf_.energy();
    return erf_.odd();
  }

  /// @brief Convert relative threshold to absolute magnitude threshold.
  /// @param val Relative threshold value
  /// @return Absolute magnitude threshold
  virtual double magnitudeThreshold(double val) const override {
    if (quadOps_ == ESQO_ENERGY) return static_cast<double>(erf_.energyThreshold(val));
    return static_cast<double>(erf_.oddThreshold(val));
  }
};

/// @brief Phase congruency edge source using even response and zero-crossing.
/// Extends EdgeSourceQUADZC with phase congruency magnitude support.
/// @tparam PC Phase congruency filter type
/// @tparam ZC Zero-crossing detector type
template <class PC, class ZC>
class EdgeSourcePCZC : public EdgeSourceQUADZC<PC, ZC> {
  EdgeSourcePCZC(const EdgeSourcePCZC&);

 protected:
  using EdgeSourceT<PC, ZC>::erf_;
  using EdgeSourceT<PC, ZC>::epe_;
  using EdgeSourceQUADZC<PC, ZC>::quadOps_;

 public:
  EdgeSourcePCZC(ESQuadratureOptions quadOps = ESQO_MAG, ESDirectionOptions dirOps = ESDO_NONE)
      : EdgeSourceQUADZC<PC, ZC>(quadOps, dirOps) {}
  EdgeSourcePCZC(const ValueManager::NameValueVector& options) : EdgeSourceQUADZC<PC, ZC>(options) {}
  EdgeSourcePCZC(ValueManager::InitializerList& options) : EdgeSourceQUADZC<PC, ZC>(options) {}

  using EdgeSourceT<PC, ZC>::responseFilter;
  using EdgeSourceT<PC, ZC>::pixelExtractor;
  using EdgeSourceI::hysteresis;
  using EdgeSourceI::hysteresis_edgels;

  /// @brief Get edge magnitude based on quadrature options.
  /// @return Magnitude matrix (odd, energy, or phase congruency)
  virtual cv::Mat magnitude() const override {
    switch (quadOps_) {
      case ESQO_MAG:
        return erf_.odd();
      case ESQO_ENERGY:
        return erf_.energy();
      case ESQO_PC:
        return erf_.phaseCongruency();
    }
    return erf_.phaseCongruency();
  }

  /// @brief Convert relative threshold to absolute magnitude threshold.
  /// @param val Relative threshold value
  /// @return Absolute threshold based on selected mode
  virtual double magnitudeThreshold(double val) const override {
    switch (quadOps_) {
      case ESQO_MAG:
        return erf_.oddThreshold(val);
      case ESQO_ENERGY:
        return erf_.energyThreshold(val);
      case ESQO_PC:
        return val;
    }
    return val;
  }
};


/// @brief Phase congruency Laplace edge source using even response and zero-crossing.
/// Combines phase congruency Laplacian with zero-crossing detection,
/// supporting direction-based filtering for X and Y components.
/// @tparam PCL Phase congruency Laplace filter type
/// @tparam ZC Zero-crossing detector type
template <class PCL, class ZC>
class EdgeSourcePCLZC : public EdgeSourceQUADZC<PCL, ZC> {
  EdgeSourcePCLZC(const EdgeSourcePCLZC&);

 protected:
  using EdgeSourceT<PCL, ZC>::erf_;
  using EdgeSourceT<PCL, ZC>::epe_;
  using EdgeSourceZC<PCL, ZC>::magMax_;
  using EdgeSourceQUADZC<PCL, ZC>::dirOps_;

  cv::Mat dirMap_;
  IndexVector seeds_;

 public:
  typedef typename PCL::dir_type dir_type;

  EdgeSourcePCLZC(ESQuadratureOptions quadOps = ESQO_MAG, ESDirectionOptions dirOps = ESDO_NONE)
      : EdgeSourceQUADZC<PCL, ZC>(quadOps, dirOps), dirMap_(), seeds_() {}
  EdgeSourcePCLZC(const ValueManager::NameValueVector& options,
                  ESQuadratureOptions quadOps = ESQO_MAG,
                  ESDirectionOptions dirOps = ESDO_NONE)
      : EdgeSourceQUADZC<PCL, ZC>(options), dirMap_(), seeds_() {
    this->quadOps_ = quadOps;
    this->dirOps_ = dirOps;
  }
  EdgeSourcePCLZC(ValueManager::InitializerList& options,
                  ESQuadratureOptions quadOps = ESQO_MAG,
                  ESDirectionOptions dirOps = ESDO_NONE)
      : EdgeSourceQUADZC<PCL, ZC>(options), dirMap_(), seeds_() {
    this->quadOps_ = quadOps;
    this->dirOps_ = dirOps;
  }

  using EdgeSourceT<PCL, ZC>::responseFilter;
  using EdgeSourceT<PCL, ZC>::pixelExtractor;
  using EdgeSourceI::hysteresis;
  using EdgeSourceI::hysteresis_edgels;

  template <class FT>
  static inline void filterDirX(const IndexVector& in,
                                const cv::Mat& dir,
                                const typename PCL::DirectionRange& dirRange,
                                IndexVector& out) {
    if (dirRange.lower < 0) {
      FT rangeHalf4 = dirRange.upper / 4;
      for_each(in.begin(), in.end(), [&](index_type idx) {
        FT d = std::abs(dir.ptr<FT>()[idx]);
        if (d <= rangeHalf4 || d > rangeHalf4 * 3) out.push_back(idx);
      });
    } else {
      FT rangeHalf = dirRange.upper / 2;
      FT rangeHalf4 = rangeHalf / 4;
      for_each(in.begin(), in.end(), [&](index_type idx) {
        FT d = dir.ptr<FT>()[idx];
        if (d > rangeHalf) d = dirRange.upper - d;
        if (d <= rangeHalf4 / 4 || d > rangeHalf4 / 4 * 3) out.push_back(idx);
      });
    }
  }

  template <class FT>
  static inline void filterDirY(const IndexVector& in,
                                const cv::Mat& dir,
                                const typename PCL::DirectionRange& dirRange,
                                IndexVector& out) {
    if (dirRange.lower < 0) {
      FT rangeHalf4 = dirRange.upper / 4;
      for_each(in.begin(), in.end(), [&](index_type idx) {
        FT d = std::abs(dir.ptr<FT>()[idx]);
        if (d > rangeHalf4 || d <= rangeHalf4 * 3) out.push_back(idx);
      });
    } else {
      FT rangeHalf = dirRange.upper / 2;
      FT rangeHalf4 = rangeHalf / 4;
      for_each(in.begin(), in.end(), [&](index_type idx) {
        FT d = dir.ptr<FT>()[idx];
        if (d > rangeHalf) d = dirRange.upper - d;
        if (d > rangeHalf4 || d <= rangeHalf4 * 3) out.push_back(idx);
      });
    }
  }

  template <class FT>
  static inline void filterDirCheckY(
      const char* inY, const FT* dir, const typename PCL::DirectionRange& dirRange, char* inXOut, int size) {
    if (dirRange.lower < 0) {
      FT rangeHalf4 = dirRange.upper / 4;
      for (int i = 0; i != size; ++i) {
        FT d = std::abs(dir[i]);
        if (d > rangeHalf4 && d <= rangeHalf4 * 3) inXOut[i] = inY[i];
      }
    } else {
      FT rangeHalf = dirRange.upper / 2;
      FT rangeHalf4 = rangeHalf / 4;
      for (int i = 0; i != size; ++i) {
        FT d = dir[i];
        if (d > rangeHalf) d = dirRange.upper - d;
        if (d > rangeHalf4 || d <= rangeHalf4 * 3) inXOut[i] = inY[i];
      }
    }
  }

  template <class FT>
  static inline void filterDirCheckY(const cv::Mat& inY,
                                     const cv::Mat& dir,
                                     const typename PCL::DirectionRange& dirRange,
                                     cv::Mat& inXOut) {
    if (inY.isContinuous() && dir.isContinuous() && inXOut.isContinuous()) {
      filterDirCheckY(inY.ptr<char>(), dir.ptr<FT>(), dirRange, inXOut.ptr<char>(), dir.rows * dir.cols);
    } else {
      for (int row = 0; row != dir.rows; ++row)
        filterDirCheckY(inY.ptr<char>(row), dir.ptr<FT>(row), dirRange, inXOut.ptr<char>(row), dir.cols);
    }
  }

  /// @brief Process edge source from input image.
  /// Computes phase congruency Laplacian and applies direction-filtered ZC detection.
  /// @param img Input grayscale image
  virtual void process(const cv::Mat& img) override {
    epe_.threshold(low, high);

    low = erf_.pcLaplaceThreshold(low);
    high = erf_.pcLaplaceThreshold(high);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    seeds_.clear();
    dirMap_.setTo(-1);
    cv::Mat lx, ly, dir = erf_.direction();
    erf_.pcLaplace(lx, ly);
    typename PCL::DirectionRange dirRange = erf_.directionRange();

    switch (dirOps_) {
      case ESDO_GXGY:
      case ESDO_DIR:
        epe_.process(dir, lx, low, high, dirRange.lower, dirRange.upper);
        filterDirX<dir_type>(epe_.seeds(), dir, dirRange, seeds_);
        dirMap_ = epe_.directionMap().clone();
        epe_.process(dir, ly, low, high, dirRange.lower, dirRange.upper);
        filterDirY<dir_type>(epe_.seeds(), dir, dirRange, seeds_);
        filterDirCheckY<dir_type>(epe_.directionMap(), dir, dirRange, dirMap_);
        break;
      default:
        epe_.process(lx, low, high);
        filterDirX<dir_type>(epe_.seeds(), dir, dirRange, seeds_);
        dirMap_ = epe_.directionMap().clone();
        epe_.process(ly, low, high);
        filterDirY<dir_type>(epe_.seeds(), dir, dirRange, seeds_);
        filterDirCheckY<dir_type>(epe_.directionMap(), dir, dirRange, dirMap_);
        break;
    }
  }

  /// @brief Get the quantized direction map.
  /// @return Direction map (CV_8S)
  virtual cv::Mat directionMap() const override { return dirMap_; }

  /// @brief Get seed indices.
  /// @return Const reference to seed index vector
  virtual const IndexVector& seeds() const override { return seeds_; }
};
}  // namespace lsfm
