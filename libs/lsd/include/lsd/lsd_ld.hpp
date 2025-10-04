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

#pragma once

#include <edge/edge_segment.hpp>
#include <edge/edge_source.hpp>
#include <edge/fit.hpp>
#include <edge/line_tracer.hpp>
#include <edge/spe.hpp>
#include <geometry/draw.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <lsd/lsd_base.hpp>

namespace lsfm {

//! No ld flags set
static const int LD_NONE = 0;
//! Enable to use tracer with direction map
static const int LD_TRACER_DIRMAP = 1;
//! Use precise direction map for sub pixel esitmation
static const int LD_USE_PRECISE_SPE = 2;

//! line detector to line segment detector adapter class
template <class LD,
          template <class> class LPT = Vec2,
          class PT = Vec2i,
          class TRACER = LineTracer<typename LD::float_type, LPT, PT>,
          class SPE = PixelEstimator<typename LD::float_type, PT>,
          class FIT = FitLine<EigenFit<typename LD::float_type, PT>>>
class LsdLd : public LsdExt<typename LD::float_type, LPT, PT> {
  typedef LsdExt<typename LD::float_type, LPT, PT> MyLsdBase;

 public:
  typedef typename MyLsdBase::float_type float_type;
  typedef typename MyLsdBase::line_point line_point;
  typedef typename MyLsdBase::point_type point_type;
  typedef typename MyLsdBase::PointVector PointVector;
  typedef typename MyLsdBase::Line Line;
  typedef typename MyLsdBase::LineVector LineVector;
  typedef typename MyLsdBase::LineSegment LineSegment;
  typedef typename MyLsdBase::LineSegmentVector LineSegmentVector;
  typedef typename MyLsdBase::ImageData ImageData;
  typedef LD LineDetector;
  typedef typename LD::EdgeSource EdgeSource;

 protected:
  using MyLsdBase::lineSegments_;

  LD lineDetector_;
  FIT fit_;
  TRACER tracer_;

  int flags_;

  void init() {
    this->addManager(lineDetector_);
    this->addManager(fit_);
    this->addManager(tracer_);

    this->add("line_flags", std::bind(&LsdLd<LD, LPT, PT, TRACER, SPE, FIT>::valueFlags, this, std::placeholders::_1),
              "Flags for line detector: 0 - none, 1 - use tracer with dirmap, 2 - use precise sub pixel estimation.");
  }

  PointVector points_;

  virtual void clearData() override {
    MyLsdBase::clearData();
    points_.clear();
  }

 public:
  LsdLd(float_type th_low = static_cast<float_type>(0.004),
        float_type th_high = static_cast<float_type>(0.012),
        int minPix = 10,
        int maxGap = 3,
        int flags = 0)
      : lineDetector_(th_low, th_high), tracer_(minPix, maxGap), flags_(flags) {
    init();
  }

  LsdLd(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  LsdLd(ValueManager::NameValueVector options) {
    init();
    this->value(options);
  }

  using MyLsdBase::detect;
  using MyLsdBase::endPoints;
  using MyLsdBase::imageData;
  using MyLsdBase::lines;
  using MyLsdBase::lineSegments;

  Value valueFlags(const Value& f = Value::NAV()) {
    if (f.type()) flags(f.getInt());
    return flags_;
  }

  int flags() const { return flags_; }

  void flags(int f) { flags_ = f; }

  virtual void detect(const cv::Mat& image) override {
    clearData();
    lineDetector_.detect(image);
    cv::Mat binary = lineDetector_.imageData("canny_map");

    if (flags_ & LD_TRACER_DIRMAP) {
      if (!binary.empty())
        tracer_.traceDirmap(lineDetector_.lines(), binary, edgeSource().directionMap());
      else
        tracer_.traceDirmap(lineDetector_.lines(), edgeSource().hysteresis());
    } else {
      if (binary.empty()) binary = edgeSource().hysteresis_binary();
      tracer_.traceBinary(lineDetector_.lines(), binary);
    }


    if (flags_ & LD_USE_PRECISE_SPE)
      SPE::convertDir(indexes(), points_, edgeSource().magnitude(), edgeSource().direction());
    else
      SPE::convert(indexes(), points_, edgeSource().magnitude(), edgeSource().directionMap());
    // pointConvert(tracer_.points(), points_);

    lineSegments_.reserve(tracer_.segments().size());
    for_each(tracer_.segments().begin(), tracer_.segments().end(), [this](const EdgeSegment& seg) {
      Line l;
      fit_.apply(this->points_.data() + seg.begin(), this->points_.data() + seg.end(), l);

      // std::cout << "reverse: " << (seg.reverse() ? "true" : "false") << std::endl;
      const point_type& first = this->points_[seg.first()];
      const point_type& last = this->points_[seg.last()];

      // lastx - firstx = dx = -ny = -dx = firstx - lastx
      // lasty - firsty = dy = nx
      //  correct direction of line
      /*int epnx = getY(first) - getY(last);
      int epny = getX(last) - getX(first);
      if (epnx * l.normalX() + epny * l.normalY() < 0)
      l.normalFlip();*/

      this->lineSegments_.push_back(LineSegment(l, first, last));
    });
  }

  virtual const DataDescriptor& imageDataDescriptor() const final { return lineDetector_.imageDataDescriptor(); }

  virtual const ImageData& imageData() const final { return lineDetector_.imageData(); }

  virtual const EdgeSegmentVector& lineSupportSegments() const final { return tracer_.segments(); }

  virtual const PointVector& points() const final { return points_; }

  virtual const IndexVector& indexes() const final { return tracer_.indexes(); }

  LineDetector& lineDetector() { return lineDetector_; }

  const LineDetector& lineDetector() const { return lineDetector_; }

  EdgeSource& edgeSource() { return lineDetector_.edgeSource(); }

  const EdgeSource& edgeSource() const { return lineDetector_.edgeSource(); }
};

//! line detector to line segment detector adapter class
template <class LD,
          template <class> class LPT = Vec2,
          class PT = Vec2i,
          class TRACER = LineTracer<typename LD::float_type, LPT, PT>,
          class SPE = PixelEstimator<typename LD::float_type, PT>,
          class FIT = FitLine<EigenFit<typename LD::float_type, PT>>,
          class ESOURCE = EdgeSourceGRAD<
              DerivativeGradient<uchar, short, int, typename LD::float_type, SobelDerivative, QuadraticMagnitude>,
              NonMaximaSuppression<short, int, typename LD::float_type>>>
class LsdLdES : public LsdExt<typename LD::float_type, LPT, PT> {
  typedef LsdExt<typename LD::float_type, LPT, PT> MyLsdBase;

 public:
  typedef typename MyLsdBase::float_type float_type;
  typedef typename MyLsdBase::line_point line_point;
  typedef typename MyLsdBase::point_type point_type;
  typedef typename MyLsdBase::PointVector PointVector;
  typedef typename MyLsdBase::Line Line;
  typedef typename MyLsdBase::LineVector LineVector;
  typedef typename MyLsdBase::LineSegment LineSegment;
  typedef typename MyLsdBase::LineSegmentVector LineSegmentVector;
  typedef typename MyLsdBase::ImageData ImageData;
  typedef LD LineDetector;
  typedef ESOURCE EdgeSource;


  using MyLsdBase::lineSegments_;

  ESOURCE esource_;
  LD lineDetector_;
  FIT fit_;
  TRACER tracer_;

  cv::Mat binaryEdgeMap_;

  int flags_;

  void init() {
    this->addManager(lineDetector_);
    this->addManager(esource_);
    this->addManager(fit_);
    this->addManager(tracer_);

    this->add("line_flags", std::bind(&LsdLdES<LD, LPT, PT, TRACER, SPE, FIT>::valueFlags, this, std::placeholders::_1),
              "Flags for line detector: 0 - none, 1 - use tracer with dirmap, 2 - use precise sub pixel estimation.");
  }

  typename MyLsdBase::ImageData imageData_;

  PointVector points_;


  virtual void clearData() override {
    MyLsdBase::clearData();
    imageData_.clear();
    points_.clear();
  }

 public:
  LsdLdES(float_type th_low = static_cast<float_type>(0.004),
          float_type th_high = static_cast<float_type>(0.012),
          int minPix = 10,
          int maxGap = 3,
          int flags = 0)
      : lineDetector_(th_low, th_high), tracer_(minPix, maxGap) {
    init();
  }

  LsdLdES(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  LsdLdES(ValueManager::NameValueVector options) {
    init();
    this->value(options);
  }

  using MyLsdBase::detect;
  using MyLsdBase::endPoints;
  using MyLsdBase::imageData;
  using MyLsdBase::lines;
  using MyLsdBase::lineSegments;

  Value valueFlags(const Value& f = Value::NAV()) {
    if (f.type()) flags(f.getInt());
    return flags_;
  }

  int flags() const { return flags_; }

  void flags(int f) { flags_ = f; }

  virtual void detect(const cv::Mat& image) override {
    clearData();
    lineDetector_.process(image);
    esource_.process(image);
    binaryEdgeMap_ = esource_.hysteresis_binary();

    if (flags_ & LD_TRACER_DIRMAP)
      tracer_.traceDirmap(lineDetector_.lines(), binaryEdgeMap_, esource_.directionMap());
    else
      tracer_.traceBinary(lineDetector_.lines(), binaryEdgeMap_);

    if (flags_ & LD_USE_PRECISE_SPE)
      SPE::convertDir(indexes(), points_, esource_.magnitude(), esource_.direction());
    else
      pointConvert(tracer_.points(), points_);

    lineSegments_.reserve(tracer_.segments().size());
    for_each(tracer_.segments().begin(), tracer_.segments().end(), [this](const EdgeSegment& seg) {
      Line l;
      fit_.apply(this->points_.data() + seg.begin(), this->points_.data() + seg.end(), l);

      // std::cout << "reverse: " << (seg.reverse() ? "true" : "false") << std::endl;
      const point_type& first = this->points_[seg.first()];
      const point_type& last = this->points_[seg.last()];

      // lastx - firstx = dx = -ny = -dx = firstx - lastx
      // lasty - firsty = dy = nx
      //  correct direction of line
      /*int epnx = getY(first) - getY(last);
      int epny = getX(last) - getX(first);
      if (epnx * l.normalX() + epny * l.normalY() < 0)
      l.normalFlip();*/

      this->lineSegments_.push_back(LineSegment(l, first, last));
    });
  }

  virtual const DataDescriptor& imageDataDescriptor() const final {
    static DataDescriptor dsc;
    if (dsc.empty()) {
      dsc = lineDetector_.imageDataDescriptor();
      typename DataDescriptor::const_iterator f =
          find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "gx"); });
      if (f == dsc.end()) dsc.push_back(DataDescriptorEntry("gx", "Gradient in x direction"));

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "gy"); });
      if (f == dsc.end()) dsc.push_back(DataDescriptorEntry("gy", "Gradient in y direction"));

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "dir"); });
      if (f == dsc.end()) dsc.push_back(DataDescriptorEntry("dir", "Gradient direction"));

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "mag"); });
      if (f == dsc.end()) dsc.push_back(DataDescriptorEntry("mag", "Gradient magnitude"));

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "edge_map"); });
      if (f == dsc.end())
        dsc.push_back(DataDescriptorEntry(
            "edge_map", "Dir map, indicating if pixel is on edge or not (also giving direction 0-7)"));

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "canny_map"); });
      if (f == dsc.end())
        dsc.push_back(DataDescriptorEntry("canny_map",
                                          "Binary edge map, indicating if pixel is on edge or not after hysteresis"));
    }
    return dsc;
  }

  virtual const ImageData& imageData() const final {
    if (imageData_.empty()) {
      imageData_ = lineDetector_.imageData();

      DataDescriptor dsc = lineDetector_.imageDataDescriptor();
      typename DataDescriptor::const_iterator f =
          find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "gx"); });
      if (f == dsc.end()) imageData_.push_back(esource_.gx());

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "gy"); });
      if (f == dsc.end()) imageData_.push_back(esource_.gy());

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "dir"); });
      if (f == dsc.end()) imageData_.push_back(esource_.direction());

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "mag"); });
      if (f == dsc.end()) imageData_.push_back(esource_.magnitude());

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "edge_map"); });
      if (f == dsc.end()) imageData_.push_back(esource_.directionMap());

      f = find_if(dsc.begin(), dsc.end(), [](const DataDescriptorEntry& e) { return (e.name == "canny_map"); });
      if (f == dsc.end()) imageData_.push_back(binaryEdgeMap_);
    }
    return imageData_;
  }

  virtual const EdgeSegmentVector& lineSupportSegments() const final { return tracer_.segments(); }

  virtual const PointVector& points() const final { return points_; }

  virtual const IndexVector& indexes() const final { return tracer_.indexes(); }

  LineDetector& lineDetector() { return lineDetector_; }

  const LineDetector& lineDetector() const { return lineDetector_; }

  EdgeSource& edgeSource() { return esource_; }

  const EdgeSource& edgeSource() const { return esource_; }
};

}  // namespace lsfm
