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

#include <lsd/impl/lsd_edlz.hpp>
#include <lsd/lsd_base.hpp>

class EDLineDetector;

namespace lsfm {

template <class FT, template <class> class LPT = Vec2>
class LsdEDLZ : public LsdBase<FT, LPT> {
  EDLineDetector* edl;
  typename LsdBase<FT, LPT>::ImageData imageData_;

  // LsdEDLZ& operator= (const LsdEDLZ&); // to quiet MSVC
  void init() {
    edl = new EDLineDetector;

    this->add("grad_th", std::bind(&LsdEDLZ<FT, LPT>::valueGradientThreshold, this, std::placeholders::_1),
              "Gradient threshold.");
    this->add("anchor_th", std::bind(&LsdEDLZ<FT, LPT>::valueAnchorThreshold, this, std::placeholders::_1),
              "Anchor threshold.");
    this->add("scan_int", std::bind(&LsdEDLZ<FT, LPT>::valueScanIntervals, this, std::placeholders::_1),
              "Scan intervals.");
    this->add("min_len", std::bind(&LsdEDLZ<FT, LPT>::valueMinLength, this, std::placeholders::_1),
              "Minimal line length.");
    this->add("fit_error", std::bind(&LsdEDLZ<FT, LPT>::valueFitError, this, std::placeholders::_1),
              "Line fit error in pixel.");
    this->add("validate", std::bind(&LsdEDLZ<FT, LPT>::valueValidate, this, std::placeholders::_1),
              "Validate line using nfa.");
  }

  using LsdBase<FT, LPT>::endPoints_;
  using LsdBase<FT, LPT>::lineSegments_;

 public:
  typedef FT float_type;
  typedef LPT<FT> line_point;
  typedef typename LsdBase<FT, LPT>::Line Line;
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;

  //! Create a LsdEDLZ object
  LsdEDLZ(FT gradientThreshold = 10,
          FT anchorThreshold = 2,
          int scanIntervals = 2,
          int minLineLen = 15,
          FT lineFitErrThreshold = 2,
          bool validate = false) {
    init();
    EDLineParam p;
    p.gradientThreshold = static_cast<float>(gradientThreshold);
    p.anchorThreshold = static_cast<float>(anchorThreshold);
    p.scanIntervals = scanIntervals;
    p.minLineLen = minLineLen;
    p.lineFitErrThreshold = static_cast<double>(lineFitErrThreshold);
    p.validate = validate;
    edl->setParams(p);
  }

  LsdEDLZ(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  LsdEDLZ(const ValueManager::NameValueVector& options) {
    init();
    this->value(options);
  }

  ~LsdEDLZ() { delete edl; }

  Value valueGradientThreshold(const Value& t = Value::NAV()) {
    if (t.type()) gradientThreshold(t.get<FT>());
    return gradientThreshold();
  }

  FT gradientThreshold() const { return static_cast<FT>(edl->getParams().gradientThreshold); }

  void gradientThreshold(FT t) {
    EDLineParam p = edl->getParams();
    p.gradientThreshold = static_cast<float>(t);
    edl->setParams(p);
  }

  Value valueAnchorThreshold(const Value& t = Value::NAV()) {
    if (t.type()) anchorThreshold(t.get<FT>());
    return anchorThreshold();
  }

  FT anchorThreshold() const { return static_cast<FT>(edl->getParams().anchorThreshold); }

  void anchorThreshold(FT t) {
    EDLineParam p = edl->getParams();
    p.anchorThreshold = static_cast<float>(t);
    edl->setParams(p);
  }

  Value valueFitError(const Value& e = Value::NAV()) {
    if (e.type()) fitError(e.get<FT>());
    return fitError();
  }

  FT fitError() const { return static_cast<FT>(edl->getParams().lineFitErrThreshold); }

  void fitError(FT e) {
    EDLineParam p = edl->getParams();
    p.lineFitErrThreshold = static_cast<float>(e);
    edl->setParams(p);
  }

  Value valueScanIntervals(const Value& si = Value::NAV()) {
    if (si.type()) scanIntervals(si.getInt());
    return scanIntervals();
  }

  int scanIntervals() const { return edl->getParams().scanIntervals; }

  void scanIntervals(int si) {
    EDLineParam p = edl->getParams();
    p.scanIntervals = si;
    edl->setParams(p);
  }

  Value valueMinLength(const Value& ml = Value::NAV()) {
    if (ml.type()) minLength(ml.getInt());
    return minLength();
  }

  int minLength() const { return edl->getParams().minLineLen; }

  void minLength(int ml) {
    EDLineParam p = edl->getParams();
    p.minLineLen = ml;
    edl->setParams(p);
  }

  Value valueValidate(const Value& v = Value::NAV()) {
    if (v.type()) validate(v.getInt() != 0);
    return validate() ? 1 : 0;
  }

  bool validate() const { return edl->getParams().validate; }

  void validate(bool v) {
    EDLineParam p = edl->getParams();
    p.validate = v;
    edl->setParams(p);
  }

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::endPoints;
  using LsdBase<FT, LPT>::imageDataDescriptor;
  using LsdBase<FT, LPT>::imageData;

  virtual void detect(const cv::Mat& image) final {
    cv::Mat img = image;
    CV_Assert(!img.empty());

    if (img.channels() != 1) cvtColor(img, img, cv::COLOR_BGR2GRAY);

    this->clearData();
    imageData_.clear();

    if (edl->EDline(img)) {
      lineSegments_.reserve(edl->lineEndpoints_.size());
      for_each(edl->lineEndpoints_.begin(), edl->lineEndpoints_.end(), [&](const std::array<float, 4>& p) {
        lineSegments_.push_back(LineSegment(LPT<FT>(static_cast<FT>(p[0]), static_cast<FT>(p[1])),
                                            LPT<FT>(static_cast<FT>(p[2]), static_cast<FT>(p[3]))));
      });

      imageData_.push_back(edl->dxImg_);
      imageData_.push_back(edl->dyImg_);
      imageData_.push_back(edl->gImgWO_);
      imageData_.push_back(edl->gImg_);
      imageData_.push_back(edl->dirImg_);
      imageData_.push_back(edl->edgeImage_);
    }
  }

  virtual const DataDescriptor& imageDataDescriptor() const final {
    static DataDescriptor dsc;
    if (dsc.empty()) {
      dsc.push_back(DataDescriptorEntry("gx", "Gradient in x direction"));
      dsc.push_back(DataDescriptorEntry("gy", "Gradient in y direction"));
      dsc.push_back(DataDescriptorEntry("mag", "Gradient magnitude"));
      dsc.push_back(DataDescriptorEntry("th_mag", "Gradient magnitude with threshold"));
      dsc.push_back(DataDescriptorEntry("dir_map", "Direction map"));
      dsc.push_back(DataDescriptorEntry("edge_map", "Edge map"));
    }
    return dsc;
  }

  virtual const ImageData& imageData() const final { return imageData_; }
};
}  // namespace lsfm
