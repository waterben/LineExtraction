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

#include <edge/edge_linking.hpp>
#include <edge/edge_source.hpp>
#include <edge/fit.hpp>
#include <edge/nfa.hpp>
#include <edge/spe.hpp>
#include <edge/split.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <lsd/lsd_base.hpp>

namespace lsfm {

/// @brief Flag to enable NFA-based validation in EL detector.
static const int EL_USE_NFA = 1;

/// @brief Flag to use precise sub-pixel estimation with direction map.
static const int EL_USE_PRECISE_SPE = 2;

/// @brief Advanced line segment detector combining edge linking with NFA validation.
/// This detector provides maximum flexibility through template parameters,
/// allowing customization of edge detection, linking, validation, and fitting strategies.
///
/// **Key Features:**
/// - Sophisticated edge linking algorithm
/// - NFA-based statistical validation
/// - Sub-pixel position estimation
/// - Configurable line fitting strategies
/// - Ramer-Douglas-Peucker line simplification
///
/// **Template Parameters:**
/// - FT: Floating-point type (float, double)
/// - LPT: Line point template
/// - PT: Point type for internal calculations
/// - ESOURCE: Edge source and NMS strategy
/// - EDGE: Edge linking algorithm
/// - NFA: NFA computation strategy
/// - SPE: Sub-pixel estimation
/// - SPLIT: Line splitting algorithm
/// - FIT: Line fitting method
///
/// @tparam FT Floating-point type
/// @tparam LPT Line point template
/// @tparam PT Point type
/// @tparam ESOURCE Edge source with NMS
/// @tparam EDGE Edge linking algorithm
/// @tparam NFA NFA validation
/// @tparam SPE Sub-pixel estimator
/// @tparam SPLIT Line splitting strategy
/// @tparam FIT Line fitting method
template <class FT,
          template <class> class LPT = Vec2,
          class PT = LPT<int>,
          class ESOURCE = EdgeSourceGRAD<DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
                                         NonMaximaSuppression<short, int, FT>>,
          class EDGE = EsdLinking<int, ESOURCE::NUM_DIR>,
          class NFA = NfaBinom<short, FT, index_type>,
          class SPE = PixelEstimator<FT, PT>,
          class SPLIT = RamerSplit<FT, PT>,
          class FIT = FitLine<EigenFit<FT, PT>>>
class LsdEL : public LsdExt<FT, LPT, PT> {
  ESOURCE esource_{};
  EDGE edge_{};
  SPLIT split_{};
  FIT fit_{};
  NFA nfa_{};
  int flags_{};

  typename LsdExt<FT, LPT, PT>::PointVector points_{};
  EdgeSegmentVector segments_{};

  mutable typename LsdBase<FT, LPT>::ImageData imageData_{};

  void init() {
    this->addManager(esource_);
    this->addManager(edge_);
    this->addManager(split_);
    this->addManager(fit_);
    this->addManager(nfa_);

    this->add(
        "line_flags",
        std::bind(&LsdEL<FT, LPT, PT, ESOURCE, EDGE, NFA, SPE, SPLIT, FIT>::valueFlags, this, std::placeholders::_1),
        "Flags for line detector: 0 - none, 1 - use nfa, 2 - use precise sub pixel estimation.");
  }

  virtual void clearData() final {
    LsdBase<FT, LPT>::clearData();
    imageData_.clear();
  }

  using LsdBase<FT, LPT>::endPoints_;
  using LsdBase<FT, LPT>::lineSegments_;

 public:
  typedef FT float_type;
  typedef LPT<FT> line_point;
  typedef PT point_type;

  typedef typename LsdBase<FT, LPT>::Line Line;
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;
  typedef typename LsdExt<FT, LPT, PT>::PointVector PointVector;

  /// @brief Create an EL (Edge Linking) detector with specified parameters.
  /// @param th_low Lower gradient threshold (normalized, 0-1)
  /// @param th_high Upper gradient threshold (normalized, 0-1)
  /// @param min_pix Minimum supporting pixels
  /// @param dist Line splitting distance threshold
  /// @param min_len Minimum line segment length
  /// @param log_eps NFA detection threshold (logarithmic)
  /// @param flags Detection flags (EL_USE_NFA, EL_USE_PRECISE_SPE)
  LsdEL(FT th_low = static_cast<FT>(0.004),
        FT th_high = static_cast<FT>(0.012),
        int min_pix = 10,
        FT dist = 2,
        int min_len = 5,
        FT log_eps = 0,
        int flags = 0)
      : esource_({NV("nms_th_low", th_low), NV("nms_th_high", th_high)}),
        edge_(min_pix),
        split_(dist, min_len),
        nfa_(log_eps),
        flags_(flags) {
    init();
  }

  /// @brief Create detector from initializer list.
  /// @param options Initializer list with parameter name/value pairs
  LsdEL(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  /// @brief Create detector from parameter vector.
  /// @param options Vector with parameter name/value pairs
  LsdEL(ValueManager::NameValueVector options) {
    init();
    this->value(options);
  }

  /// @brief Get/set flags via ValueManager interface.
  /// @param f New flags value (optional)
  /// @return Current or updated flags
  Value valueFlags(const Value& f = Value::NAV()) {
    if (f.type()) flags(f.getInt());
    return flags_;
  }

  /// @brief Get current detection flags.
  /// @return Flags bitmask (EL_USE_NFA, EL_USE_PRECISE_SPE)
  int flags() const { return flags_; }

  /// @brief Set detection flags.
  /// @param f Flags bitmask (0 = none, EL_USE_NFA = enable NFA, EL_USE_PRECISE_SPE = use precise SPE)
  void flags(int f) { flags_ = f; }

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;
  using LsdBase<FT, LPT>::imageDataDescriptor;
  using LsdBase<FT, LPT>::imageData;

  /// @brief Detect line segments using edge linking algorithm.
  /// @param image Input image (8-bit single-channel or will be converted)
  virtual void detect(const cv::Mat& image) final {
    clearData();

    esource_.process(image);
    edge_.detect(esource_);
    if (flags_ & EL_USE_PRECISE_SPE)
      SPE::convertDir(edge_.points(), points_, esource_.magnitude(), esource_.direction());
    else
      SPE::convert(edge_.points(), points_, esource_.magnitude(), esource_.directionMap());

    split_.setup(esource_);
    if (flags_ & EL_USE_NFA) {
      nfa_.update(esource_);
      EdgeSegmentVector tmpSeg;
      std::vector<FT> tmpNfa;
      nfa_.eval(edge_, tmpSeg, tmpNfa);
      split_.apply(tmpSeg, points_, segments_);
    } else {
      split_.apply(edge_.segments(), points_, segments_);
    }

    lineSegments_.reserve(segments_.size());
    for_each(segments_.begin(), segments_.end(), [this](const EdgeSegment& seg) {
      Line l;
      fit_.apply(this->points_.data() + seg.begin(), this->points_.data() + seg.end(), l);

      const PT& first = this->points_[seg.first()];
      const PT& last = this->points_[seg.last()];

      // lastx - firstx = dx = -ny = -dx = firstx - lastx
      // lasty - firsty = dy = nx
      //  correct direction of line
      /*int epnx = getY(first) - getY(last);
      int epny = getX(last) - getX(first);
      if (epnx * l.normalX() + epny * l.normalY() < 0)
          l.normalFlip();*/

      lineSegments_.push_back(LineSegment(l, first, last));
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
                                        "Edge map, indicating if pixel is on edge or not (also giving direction 0-7)"));
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
    }
    return imageData_;
  }

  /// @brief Get edge segments from edge linking.
  /// @return Const reference to edge segment vector
  const EdgeSegmentVector& segments() const { return edge_.segments(); }

  /// @brief Get line support segments (edge segment list).
  /// @return Const reference to line support edge segments
  virtual const EdgeSegmentVector& lineSupportSegments() const final { return segments_; }

  /// @brief Get all support points used in line detection.
  /// @return Const reference to support point vector
  virtual const PointVector& points() const final { return points_; }

  /// @brief Get point indices for edge segments.
  /// @return Const reference to index vector
  virtual const IndexVector& indexes() const final { return edge_.points(); }

  /// @brief Get mutable reference to edge source (gradient computation).
  /// @return Mutable reference to edge source object
  ESOURCE& edgeSource() { return esource_; }

  /// @brief Get const reference to edge source.
  /// @return Const reference to edge source object
  const ESOURCE& edgeSource() const { return esource_; }

  /// @brief Get mutable reference to edge linking algorithm.
  /// @return Mutable reference to edge linking object
  EDGE& edge() { return edge_; }

  /// @brief Get const reference to edge linking algorithm.
  /// @return Const reference to edge linking object
  const EDGE& edge() const { return edge_; }

  /// @brief Get mutable reference to line splitting algorithm.
  /// @return Mutable reference to split object
  SPLIT& split() { return split_; }

  /// @brief Get const reference to line splitting algorithm.
  /// @return Const reference to split object
  const SPLIT& split() const { return split_; }

  /// @brief Get mutable reference to line fitting algorithm.
  /// @return Mutable reference to fit object
  FIT& fit() { return fit_; }

  /// @brief Get const reference to line fitting algorithm.
  /// @return Const reference to fit object
  const FIT& fit() const { return fit_; }

  /// @brief Get mutable reference to NFA validation algorithm.
  /// @return Mutable reference to NFA object
  NFA& nfa() { return nfa_; }

  /// @brief Get const reference to NFA validation algorithm.
  /// @return Const reference to NFA object
  const NFA& nfa() const { return nfa_; }
};

}  // namespace lsfm
