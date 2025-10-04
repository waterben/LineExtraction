/*M///////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/*
 *  (C) by Benjamin Wassermann
 */

#pragma once

#include <edge/zc.hpp>
#include <imgproc/gradient.hpp>
#include <imgproc/laplace.hpp>
#include <imgproc/phase_congruency.hpp>

namespace lsfm {

//! Edge source interface
//! Edge source can be used to combine arbitrary edge response
//! methods (sobel, laplace, qudarature, etc.) with different edgel
//! extraction methods (nms, zc) or even more complex combinations.
class EdgeSourceI : public ValueManager {
  EdgeSourceI(const EdgeSourceI&);

 protected:
  EdgeSourceI() {}

 public:
  virtual ~EdgeSourceI() {}

  //! process edge source from image
  virtual void process(const cv::Mat& img) = 0;

  //! generic interface to get processed data
  virtual FilterResults results() const = 0;

  //! get gradient in x dir for edge processing
  virtual cv::Mat gx() const = 0;

  //! get gradient in y dir for edge processing
  virtual cv::Mat gy() const = 0;

  //! get magnitude for edge processing
  virtual cv::Mat magnitude() const = 0;

  //! get magnitude threshold for edge processing
  virtual double magnitudeThreshold(double val) const = 0;

  //! get magnitude max for edge processing
  virtual double magnitudeMax() const = 0;

  //! get direction
  virtual cv::Mat direction() const = 0;

  //! get direction range
  virtual Range<double> directionRange() const = 0;

  //! get direction map for edge processing
  virtual cv::Mat directionMap() const = 0;

  //! get seeds for edge processing
  virtual const IndexVector& seeds() const = 0;

  //! get edge source name
  virtual std::string name() const = 0;

  //! compute hysteresis map with directions of seeds and direction map
  cv::Mat hysteresis() const { return lsfm::hysteresis(directionMap(), seeds()); }

  //! compute binary hysteresis map of seeds and direction map
  cv::Mat hysteresis_binary() const { return lsfm::hysteresis_binary(directionMap(), seeds()); }

  //! compute hysteresis indexes of seeds and direction map
  IndexVector hysteresis_edgels() const {
    IndexVector edgels = seeds();
    lsfm::hysteresis_edgels(directionMap(), edgels);
    return edgels;
  }
};

//! Edge source interface
template <class ERF, class EPE>
class EdgeSourceT : public EdgeSourceI {
  EdgeSourceT(const EdgeSourceT&);

  inline void init() {
    this->addManager(erf_);
    this->addManager(epe_);
  }

 protected:
  EdgeSourceT() { init(); }

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
  typedef ERF EdgeResponseFilter;
  typedef EPE EdgePixelExtractor;

  static constexpr int NUM_DIR = EPE::NUM_DIR;

  using EdgeSourceI::hysteresis;
  using EdgeSourceI::hysteresis_edgels;

  virtual ~EdgeSourceT() {}

  const ERF& responseFilter() const { return erf_; }

  ERF& responseFilter() { return erf_; }

  const EPE& pixelExtractor() const { return epe_; }

  EPE& pixelExtractor() { return epe_; }

  virtual FilterResults results() const override { return erf_.results(); }

  virtual cv::Mat direction() const override { return erf_.direction(); }

  virtual Range<double> directionRange() const override {
    auto tmp = erf_.directionRange();
    return Range<double>(tmp.lower, tmp.upper);
  }

  virtual std::string name() const override { return "es_" + erf_.name() + "_" + epe_.name(); }
};

//! Baisc edge source expecting NMS/ZC object as template arg
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
  //! get direction map for edge processing
  virtual cv::Mat directionMap() const override { return epe_.directionMap(); }

  //! get seeds for edge processing
  virtual const IndexVector& seeds() const override { return epe_.seeds(); }
};

//! Baisc edge source expecting NMS object as template arg
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
  //! get magnitude max for edge processing
  virtual double magnitudeMax() const override { return static_cast<double>(epe_.magMax()); }
};

//! Baisc edge source expecting ZC object as template arg
template <class ERF, class ZC>
class EdgeSourceZC : public EdgeSourceNMS_ZC<ERF, ZC> {
  EdgeSourceZC(const EdgeSourceZC&);

 protected:
  using EdgeSourceT<ERF, ZC>::erf_;
  using EdgeSourceT<ERF, ZC>::epe_;
  mutable double magMax_;

  EdgeSourceZC() {}
  EdgeSourceZC(const ValueManager::NameValueVector& options) : EdgeSourceNMS_ZC<ERF, ZC>(options), magMax_(-1) {}
  EdgeSourceZC(ValueManager::InitializerList& options) : EdgeSourceNMS_ZC<ERF, ZC>(options), magMax_(-1) {}

 public:
  //! get magnitude max for edge processing
  virtual double magnitudeMax() const override {
    if (magMax_ < 0) {
      double vmin;
      cv::minMaxLoc(this->magnitude(), &vmin, &magMax_);
    }
    return magMax_;
  }
};

//! Gradient edge source
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

  //! process edge source from image
  virtual void process(const cv::Mat& img) override {
    erf_.process(img);
    epe_.process(erf_, useDir_);
  }

  virtual cv::Mat gx() const override { return erf_.gx(); }

  virtual cv::Mat gy() const override { return erf_.gy(); }

  //! get magnitude for edge processing
  virtual cv::Mat magnitude() const override { return erf_.magnitude(); }

  //! get magnitude threshold for edge processing
  virtual double magnitudeThreshold(double val) const override {
    return static_cast<double>(erf_.magnitudeThreshold(val));
  }
};

enum ESDirectionOptions { ESDO_NONE = 0, ESDO_GXGY, ESDO_DIR };

//! Laplace edge source
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

  //! process edge source from image
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

  virtual cv::Mat gx() const override { return erf_.gx(); }

  virtual cv::Mat gy() const override { return erf_.gy(); }

  //! get magnitude for edge processing
  virtual cv::Mat magnitude() const override { return erf_.magnitude(); }

  //! get magnitude threshold for edge processing
  virtual double magnitudeThreshold(double val) const override {
    return static_cast<double>(erf_.magnitudeThreshold(val));
  }
};

enum ESQuadratureOptions {
  ESQO_MAG = 0,
  ESQO_ENERGY,
  ESQO_PC,
};

//! Quadrature edge source (odd response)
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
  EdgeSourceQUAD(ESQuadratureOptions quadOps = ESQO_MAG, bool useDir = false) : quadOps_(quadOps), useDir_(useDir) {
    init();
  }
  EdgeSourceQUAD(const ValueManager::NameValueVector& options) : quadOps_(ESQO_MAG), useDir_(false) {
    init();
    this->value(options);
  }
  EdgeSourceQUAD(ValueManager::InitializerList& options) : quadOps_(ESQO_MAG), useDir_(false) {
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

  //! process edge source from image
  virtual void process(const cv::Mat& img) override {
    erf_.process(img);
    double low = 0, high = 0;
    epe_.threshold(low, high);
    low = magnitudeThreshold(low);
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

  virtual cv::Mat gx() const override { return erf_.oddx(); }

  virtual cv::Mat gy() const override { return erf_.oddy(); }

  //! get magnitude for edge processing
  virtual cv::Mat magnitude() const override {
    if (quadOps_ == ESQO_ENERGY) return erf_.energy();
    return erf_.odd();
  }

  //! get magnitude threshold for edge processing
  virtual double magnitudeThreshold(double val) const override {
    if (quadOps_ == ESQO_ENERGY) return static_cast<double>(erf_.energyThreshold(val));
    return static_cast<double>(erf_.oddThreshold(val));
  }
};

//! Phase congruency edge source (odd response)
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


  //! get magnitude for edge processing
  virtual cv::Mat magnitude() const override {
    switch (quadOps_) {
      case ESQO_MAG:
        return erf_.odd();
      case ESQO_ENERGY:
        return erf_.energy();
    }
    return erf_.phaseCongruency();
  }

  //! get magnitude threshold for edge processing
  virtual double magnitudeThreshold(double val) const override {
    switch (quadOps_) {
      case ESQO_MAG:
        return erf_.oddThreshold(val);
      case ESQO_ENERGY:
        return erf_.energyThreshold(val);
    }
    return val;
  }
};

//! Quadrature edge source (even response)
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
      : quadOps_(quadOps), dirOps_(dirOps) {
    init();
  }
  EdgeSourceQUADZC(const ValueManager::NameValueVector& options) : quadOps_(ESQO_MAG), dirOps_(ESDO_NONE) {
    init();
    this->value(options);
  }
  EdgeSourceQUADZC(ValueManager::InitializerList& options) : quadOps_(ESQO_MAG), dirOps_(ESDO_NONE) {
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

  //! process edge source from image
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

  virtual cv::Mat gx() const override { return erf_.oddx(); }

  virtual cv::Mat gy() const override { return erf_.oddy(); }

  //! get magnitude for edge processing
  virtual cv::Mat magnitude() const override {
    if (quadOps_ == ESQO_ENERGY) return erf_.energy();
    return erf_.odd();
  }

  //! get magnitude threshold for edge processing
  virtual double magnitudeThreshold(double val) const override {
    if (quadOps_ == ESQO_ENERGY) return static_cast<double>(erf_.energyThreshold(val));
    return static_cast<double>(erf_.oddThreshold(val));
  }
};

//! Phase congruency edge source (even response)
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

  //! get magnitude for edge processing
  virtual cv::Mat magnitude() const override {
    switch (quadOps_) {
      case ESQO_MAG:
        return erf_.odd();
      case ESQO_ENERGY:
        return erf_.energy();
    }
    return erf_.phaseCongruency();
  }

  //! get magnitude threshold for edge processing
  virtual double magnitudeThreshold(double val) const override {
    switch (quadOps_) {
      case ESQO_MAG:
        return erf_.oddThreshold(val);
      case ESQO_ENERGY:
        return erf_.energyThreshold(val);
    }
    return val;
  }
};


//! Phase congruency laplace edge source (even response)
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
      : EdgeSourceQUADZC<PCL, ZC>(quadOps, dirOps) {}
  EdgeSourcePCLZC(const ValueManager::NameValueVector& options,
                  ESQuadratureOptions quadOps = ESQO_MAG,
                  ESDirectionOptions dirOps = ESDO_NONE)
      : EdgeSourceQUADZC<PCL, ZC>(options) {}
  EdgeSourcePCLZC(ValueManager::InitializerList& options,
                  ESQuadratureOptions quadOps = ESQO_MAG,
                  ESDirectionOptions dirOps = ESDO_NONE)
      : EdgeSourceQUADZC<PCL, ZC>(options) {}

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

  //! process edge source from image
  virtual void process(const cv::Mat& img) override {
    magMax_ = -1;
    erf_.process(img);

    double low = 0, high = 0;
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

  //! get direction map for edge processing
  virtual cv::Mat directionMap() const override { return dirMap_; }

  //! get seeds for edge processing
  virtual const IndexVector& seeds() const override { return seeds_; }
};
}  // namespace lsfm
