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

#include <lsd/impl/lsd_fgioi.hpp>
#include <lsd/lsd_base.hpp>


namespace lsfm {

template <class FT, template <class> class LPT = Vec2>
class LsdFGioi : public LsdBase<FT, LPT> {
  FT quant_{}, angle_th_{}, log_eps_{}, density_th_{};
  int num_bins_{};


  // LsdFGioi& operator= (const LsdFGioi&); // to quiet MSVC

  void init() {
    this->add("quant_error", std::bind(&LsdFGioi<FT, LPT>::valueQuantError, this, std::placeholders::_1),
              "Quantization error (Bound to the gradient norm).");
    this->add("angle_th", std::bind(&LsdFGioi<FT, LPT>::valueAngleThreshold, this, std::placeholders::_1),
              "Gradient angle tolerance.");
    this->add("log_eps", std::bind(&LsdFGioi<FT, LPT>::valueLogEps, this, std::placeholders::_1),
              "NFA Epsilon (logarithmic).");
    this->add("density_th", std::bind(&LsdFGioi<FT, LPT>::valueDensityThreshold, this, std::placeholders::_1),
              "Minimal density of aligned region points in rectangle.");
    this->add("bins", std::bind(&LsdFGioi<FT, LPT>::valueBins, this, std::placeholders::_1),
              "Number of bins in pseudo-ordering of gradient modulus.");
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

  struct LineData {
    LineData(double w = 0, double p = 0, double n = 0) : width(w), prec(p), nfa(n) {}

    double width{}, prec{}, nfa{};
  };

  typedef std::vector<LineData> LineDataVector;

 private:
  LineDataVector lineData_{};

 public:
  //! Create a PrimalLineSegmentDetector object. Specifying scale, number of subdivisions for the image, should the
  //! lines be refined and other constants as follows:
  //! @param quant        Bound to the quantization error on the gradient norm.
  //! @param ang_th       Gradient angle tolerance in degrees.
  //! @param log_eps      Detection threshold: -log10(NFA) > _log_eps
  //! @param density_th   Minimal density of aligned region points in rectangle.
  //! @param n_bins       Number of bins in pseudo-ordering of gradient modulus.
  LsdFGioi(FT quant = 2,
           FT ang_th = static_cast<FT>(22.5),
           FT log_eps = 0,
           FT density_th = static_cast<FT>(0.7),
           int n_bins = 1024)
      : quant_(quant), angle_th_(ang_th), log_eps_(log_eps), density_th_(density_th), num_bins_(n_bins) {
    CV_Assert(quant >= 0 && ang_th > 0 && ang_th <= 180 && log_eps >= 0 && density_th < 1 && density_th >= 0 &&
              n_bins > 0);
    init();
  }

  LsdFGioi(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  LsdFGioi(const ValueManager::NameValueVector& options) {
    init();
    this->value(options);
  }

  Value valueQuantError(const Value& q = Value::NAV()) {
    if (q.type()) quantError(q.get<FT>());
    return quant_;
  }

  FT quantError() const { return quant_; }

  void quantError(FT q) { quant_ = q; }

  Value valueAngleThreshold(const Value& t = Value::NAV()) {
    if (t.type()) angleThreshold(t.get<FT>());
    return angle_th_;
  }

  FT angleThreshold() const { return angle_th_; }

  void angleThreshold(FT t) { angle_th_ = t; }

  Value valueLogEps(const Value& e = Value::NAV()) {
    if (e.type()) logEps(e.get<FT>());
    return log_eps_;
  }

  FT logEps() const { return log_eps_; }

  void logEps(FT e) { log_eps_ = e; }

  Value valueDensityThreshold(const Value& t = Value::NAV()) {
    if (t.type()) densityThreshold(t.get<FT>());
    return density_th_;
  }

  FT densityThreshold() const { return density_th_; }

  void densityThreshold(FT t) { density_th_ = t; }

  Value valueBins(const Value& b = Value::NAV()) {
    if (b.type()) bins(b.getInt());
    return num_bins_;
  }

  int bins() const { return num_bins_; }

  void bins(int b) { num_bins_ = b; }

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;
  using LsdBase<FT, LPT>::imageDataDescriptor;
  using LsdBase<FT, LPT>::imageData;

  virtual void detect(const cv::Mat& image) final {
    cv::Mat img = image;
    CV_Assert(!img.empty());

    if (img.channels() != 1) cvtColor(img, img, cv::COLOR_BGR2GRAY);

    // Convert image to double
    img.convertTo(img, CV_64FC1);

    this->clearData();
    lineData_.clear();

    int seg_num;
    double* segs = LineSegmentDetection(&seg_num, img.ptr<double>(), img.cols, img.rows, 1.0, 0.1, quant_, angle_th_,
                                        log_eps_, density_th_, num_bins_, NULL, NULL, NULL);

    lineSegments_.reserve(static_cast<size_t>(seg_num));
    lineData_.reserve(static_cast<size_t>(seg_num));
    for (int i = 0; i != seg_num; ++i, segs += 7) {
      lineSegments_.push_back(LineSegment(LPT<FT>(static_cast<FT>(segs[0]), static_cast<FT>(segs[1])),
                                          LPT<FT>(static_cast<FT>(segs[2]), static_cast<FT>(segs[3]))));
      lineData_.push_back(LineData(segs[4], segs[5], segs[6]));
    }
  }

  const LineDataVector& lineData() const { return lineData_; }
};
}  // namespace lsfm
