///////////////////////////////////////////////////////////////////////////////////////
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
// C by Benjamin Wassermann
// M*/

#ifndef _HOUGH_CV_LSD_HPP_
#define _HOUGH_CV_LSD_HPP_
#ifdef __cplusplus

#  include <edge/edge_source.hpp>
#  include <lsd/lsd_ld.hpp>
#  include <opencv2/opencv.hpp>

namespace lsfm {

template <class FT,
          template <class> class LPT = Vec2,
          class ESOURCE = EdgeSourceGRAD<DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
                                         NonMaximaSuppression<short, int, FT>>>
class LdHough : public LdBase<FT, LPT> {
  using LdBase<FT, LPT>::lines_;

  ESOURCE esource_;
  std::vector<cv::Vec2f> cvLines_;

  cv::Mat binaryEdgeMap_;
  mutable typename LsdBase<FT, LPT>::ImageData imageData_;

  double rho_, theta_, rho_msdiv_, theta_msdiv_;
  int voteThreshold_;

  void init() {
    this->addManager(esource_);

    this->add("hough_rho", std::bind(&LdHough<FT, LPT, ESOURCE>::valueRho, this, std::placeholders::_1),
              "Distance resolution of the accumulator in pixels.");
    this->add("hough_theta", std::bind(&LdHough<FT, LPT, ESOURCE>::valueTheta, this, std::placeholders::_1),
              "Angle resolution of the accumulator in radians.");
    this->add("hough_vote_th", std::bind(&LdHough<FT, LPT, ESOURCE>::valueVoteTh, this, std::placeholders::_1),
              "Accumulator threshold parameter. Only those lines are returned that get enough votes.");
    this->add(
        "hough_rho_msdiv", std::bind(&LdHough<FT, LPT, ESOURCE>::valueRhoMsDiv, this, std::placeholders::_1),
        "For the multi-scale Hough transform, it is a divisor for the distance resolution rho (0 - no multi scale)");
    this->add(
        "hough_theta_msdiv", std::bind(&LdHough<FT, LPT, ESOURCE>::valueThetaMsDiv, this, std::placeholders::_1),
        "For the multi-scale Hough transform, it is a divisor for the distance resolution theta (0 - no multi scale)");
  }

  virtual void clearData() final {
    lines_.clear();
    cvLines_.clear();
    imageData_.clear();
  }

 public:
  typedef FT float_type;
  typedef LPT<FT> line_point;

  typedef typename LdBase<FT, LPT>::Line Line;
  typedef typename LdBase<FT, LPT>::LineVector LineVector;
  typedef typename LdBase<FT, LPT>::ImageData ImageData;
  typedef ESOURCE EdgeSource;

  // CV Hough Transform
  LdHough(FT th_low = static_cast<FT>(0.004),
          FT th_high = static_cast<FT>(0.012),
          double rho = 1.5,
          double theta = CV_PI / 180,
          int voteThreshold = 150,
          double rho_msdiv = 0,
          double theta_msdiv = 0)
      : esource_({NV("nms_th_low", th_low), NV("nms_th_high", th_high)}),
        rho_(rho),
        theta_(theta),
        voteThreshold_(voteThreshold),
        rho_msdiv_(rho_msdiv),
        theta_msdiv_(theta_msdiv) {
    init();
  }

  LdHough(ValueManager::InitializerList options)
      : rho_(1.5), theta_(CV_PI / 180), voteThreshold_(150), rho_msdiv_(0), theta_msdiv_(0) {
    init();
    this->value(options);
  }

  LdHough(ValueManager::NameValueVector options)
      : rho_(1.5), theta_(CV_PI / 180), voteThreshold_(150), rho_msdiv_(0), theta_msdiv_(0) {
    init();
    this->value(options);
  }

  Value valueRho(const Value& v = Value::NAV()) {
    if (v.type()) rho_ = v.getDouble();
    return rho_;
  }
  Value valueTheta(const Value& v = Value::NAV()) {
    if (v.type()) theta_ = v.getDouble();
    return theta_;
  }
  Value valueVoteTh(const Value& v = Value::NAV()) {
    if (v.type()) voteThreshold_ = v.getInt();
    return voteThreshold_;
  }
  Value valueRhoMsDiv(const Value& v = Value::NAV()) {
    if (v.type()) rho_msdiv_ = v.getDouble();
    return rho_msdiv_;
  }
  Value valueThetaMsDiv(const Value& v = Value::NAV()) {
    if (v.type()) theta_msdiv_ = v.getDouble();
    return theta_msdiv_;
  }

  using LdBase<FT, LPT>::detect;
  using LdBase<FT, LPT>::lines;
  using LdBase<FT, LPT>::imageData;

  virtual void detect(const cv::Mat& image) final {
    clearData();
    esource_.process(image);
    binaryEdgeMap_ = esource_.hysteresis_binary();

    cv::HoughLines(binaryEdgeMap_, cvLines_, rho_, theta_, voteThreshold_);

    for_each(cvLines_.begin(), cvLines_.end(), [this](const cv::Vec2f& line) {
      this->lines_.push_back(Line(static_cast<FT>(line[1]), static_cast<FT>(line[0])));
    });
  }

  virtual const DataDescriptor& imageDataDescriptor() const final {
    static DataDescriptor dsc;
    if (dsc.empty()) {
      dsc.push_back(DataDescriptorEntry("gx", "Gradient in x direction"));
      dsc.push_back(DataDescriptorEntry("gy", "Gradient in y direction"));
      dsc.push_back(DataDescriptorEntry("dir", "Gradient direction"));
      dsc.push_back(DataDescriptorEntry("mag", "Gradient magnitude"));
      dsc.push_back(DataDescriptorEntry("edge_map",
                                        "Dir map, indicating if pixel is on edge or not (also giving direction 0-7)"));
      dsc.push_back(
          DataDescriptorEntry("canny_map", "Binary edge map, indicating if pixel is on edge or not after hysteresis"));
    }
    return dsc;
  }

  virtual const ImageData& imageData() const final {
    if (imageData_.empty()) {
      imageData_.push_back(esource_.gx());
      imageData_.push_back(esource_.gy());
      imageData_.push_back(esource_.direction());
      imageData_.push_back(esource_.magnitude());
      imageData_.push_back(esource_.directionMap());
      imageData_.push_back(binaryEdgeMap_);
    }
    return imageData_;
  }

  EdgeSource& edgeSource() { return esource_; }

  const EdgeSource& edgeSource() const { return esource_; }
};


template <class FT,
          template <class> class LPT = Vec2,
          class PT = Vec2i,
          class TRACER = LineTracer<FT, LPT, PT, 4>,
          class SPE = PixelEstimator<FT, PT>,
          class FIT = FitLine<EigenFit<FT, PT>>,
          class ESOURCE = EdgeSourceGRAD<DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
                                         NonMaximaSuppression<short, int, FT, FastNMS4<short, int, FT>>>>
using LsdHough = LsdLd<LdHough<FT, LPT, ESOURCE>, LPT, PT, TRACER, SPE, FIT>;


template <class FT,
          template <class> class LPT = Vec2,
          class ESOURCE = EdgeSourceGRAD<DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
                                         NonMaximaSuppression<short, int, FT>>>
class LsdHoughP : public LsdBase<FT, LPT> {
  using LsdBase<FT, LPT>::lineSegments_;

  ESOURCE esource_;
  std::vector<cv::Vec4i> cvLines_;

  cv::Mat binaryEdgeMap_;
  mutable typename LsdBase<FT, LPT>::ImageData imageData_;

  double rho_, theta_, minLength_, maxGap_;
  int voteThreshold_;

  void init() {
    this->addManager(esource_);

    this->add("hough_rho", std::bind(&LsdHoughP<FT, LPT, ESOURCE>::valueRho, this, std::placeholders::_1),
              "Distance resolution of the accumulator in pixels.");
    this->add("hough_theta", std::bind(&LsdHoughP<FT, LPT, ESOURCE>::valueTheta, this, std::placeholders::_1),
              "Angle resolution of the accumulator in radians.");
    this->add("hough_vote_th", std::bind(&LsdHoughP<FT, LPT, ESOURCE>::valueVoteTh, this, std::placeholders::_1),
              "Accumulator threshold parameter. Only those lines are returned that get enough votes.");
    this->add("edge_min_len", std::bind(&LsdHoughP<FT, LPT, ESOURCE>::valueMinLen, this, std::placeholders::_1),
              "Minimum line length. Line segments shorter than that are rejected.");
    this->add("edge_max_gap", std::bind(&LsdHoughP<FT, LPT, ESOURCE>::valueMaxGap, this, std::placeholders::_1),
              "Maximum allowed gap between points on the same line to link them.");
  }

  virtual void clearData() final {
    LsdBase<FT, LPT>::clearData();
    imageData_.clear();
  }

 public:
  typedef FT float_type;
  typedef LPT<FT> line_point;

  typedef typename LsdBase<FT, LPT>::Line Line;
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;

  LsdHoughP(FT th_low = static_cast<FT>(0.004),
            FT th_high = static_cast<FT>(0.012),
            double rho = 1.5,
            double theta = CV_PI / 180,
            int voteThreshold = 150,
            double minLength = 10,
            double maxGap = 3)
      : esource_({NV("nms_th_low", th_low), NV("nms_th_high", th_high)}),
        rho_(rho),
        theta_(theta),
        voteThreshold_(voteThreshold),
        minLength_(minLength),
        maxGap_(maxGap) {
    init();
  }

  LsdHoughP(ValueManager::InitializerList options)
      : rho_(1.5), theta_(CV_PI / 180), voteThreshold_(150), minLength_(10), maxGap_(3) {
    init();
    this->value(options);
  }

  LsdHoughP(ValueManager::NameValueVector options)
      : rho_(1.5), theta_(CV_PI / 180), voteThreshold_(150), minLength_(10), maxGap_(2) {
    init();
    this->value(options);
  }

  Value valueRho(const Value& v = Value::NAV()) {
    if (v.type()) rho_ = v.getDouble();
    return rho_;
  }
  Value valueTheta(const Value& v = Value::NAV()) {
    if (v.type()) theta_ = v.getDouble();
    return theta_;
  }
  Value valueVoteTh(const Value& v = Value::NAV()) {
    if (v.type()) voteThreshold_ = v.getInt();
    return voteThreshold_;
  }
  Value valueMinLen(const Value& v = Value::NAV()) {
    if (v.type()) minLength_ = v.getDouble();
    return minLength_;
  }
  Value valueMaxGap(const Value& v = Value::NAV()) {
    if (v.type()) maxGap_ = v.getDouble();
    return maxGap_;
  }

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;
  using LsdBase<FT, LPT>::imageData;

  virtual void detect(const cv::Mat& image) final {
    clearData();
    esource_.process(image);
    binaryEdgeMap_ = esource_.hysteresis_binary();

    cv::HoughLinesP(binaryEdgeMap_, cvLines_, rho_, theta_, voteThreshold_, minLength_, maxGap_);

    for_each(cvLines_.begin(), cvLines_.end(), [this](const cv::Vec4i& line) {
      this->lineSegments_.push_back(
          LineSegment(typename LineSegment::point_type(static_cast<FT>(line[0]), static_cast<FT>(line[1])),
                      typename LineSegment::point_type(static_cast<FT>(line[2]), static_cast<FT>(line[3]))));
    });
  }

  virtual const DataDescriptor& imageDataDescriptor() const final {
    static DataDescriptor dsc;
    if (dsc.empty()) {
      dsc.push_back(DataDescriptorEntry("gx", "Gradient in x direction"));
      dsc.push_back(DataDescriptorEntry("gy", "Gradient in y direction"));
      dsc.push_back(DataDescriptorEntry("dir", "Gradient direction"));
      dsc.push_back(DataDescriptorEntry("mag", "Gradient magnitude"));
      dsc.push_back(DataDescriptorEntry("edge_map",
                                        "Dir map, indicating if pixel is on edge or not (also giving direction 0-7)"));
      dsc.push_back(
          DataDescriptorEntry("canny_map", "Binary edge map, indicating if pixel is on edge or not after hysteresis"));
    }
    return dsc;
  }

  virtual const ImageData& imageData() const final {
    if (imageData_.empty()) {
      imageData_.push_back(esource_.gx());
      imageData_.push_back(esource_.gy());
      imageData_.push_back(esource_.direction());
      imageData_.push_back(esource_.magnitude());
      imageData_.push_back(esource_.directionMap());
      imageData_.push_back(binaryEdgeMap_);
    }
    return imageData_;
  }

  ESOURCE& edgeSource() { return esource_; }

  const ESOURCE& edgeSource() const { return esource_; }
};
}  // namespace lsfm

#endif
#endif
