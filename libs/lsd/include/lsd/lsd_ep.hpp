//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file lsd_ep.hpp
/// @brief Line segment detector using edge patterns and sub-pixel estimation.

#pragma once

#include <edge/edge_pattern.hpp>
#include <edge/edge_source.hpp>
#include <edge/fit.hpp>
#include <edge/spe.hpp>
#include <edge/split.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <lsd/lsd_base.hpp>

namespace lsfm {

// TODO adapt nfa for patterns and add to lsdep
/// @brief Flag: Use precise direction map for sub-pixel estimation.
static const int EP_USE_PRECISE_SPE = 2;

/// @brief Line segment detector using edge patterns and sub-pixel estimation.
/// Combines edge pattern detection with configurable splitting and fitting strategies.
/// Detects edge segments using directional patterns, then splits them at curvature
/// changes and fits line segments.
///
/// **Key Features:**
/// - Pattern-based edge detection with configurable corner rules
/// - Sub-pixel position estimation (standard or direction-map based)
/// - Ramer-Douglas-Peucker style line splitting
/// - Eigenvalue-based line fitting
///
/// @tparam FT Floating-point type (float, double)
/// @tparam LPT Line point template (default Vec2)
/// @tparam CORNER_RULE Whether to enable corner detection during pattern matching
/// @tparam PT Point type for support points (default LPT<int>)
/// @tparam ESOURCE Edge source with gradient and NMS
/// @tparam SPE Sub-pixel position estimator
/// @tparam SPLIT Line splitting algorithm
/// @tparam FIT Line fitting method
///
/// @code{cpp}
/// lsfm::LsdEP<float> detector(0.004f, 0.012f, 10, 2.0f, 5, 2, 3, 3.0f, 5.0f, 0);
/// detector.detect(image);
/// const auto& segments = detector.lineSegments();
/// @endcode
template <class FT,
          template <class> class LPT = Vec2,
          bool CORNER_RULE = true,
          class PT = LPT<int>,
          class ESOURCE = EdgeSourceGRAD<DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
                                         NonMaximaSuppression<short, int, FT>>,
          class SPE = PixelEstimator<FT, PT>,
          class SPLIT =
              RamerSplit<FT, PT>,  // RamerSplit<FT, PT>, ExtRamerSplit<SimpleMerge<ExtSplitCheck<FT,int,PT>>>,
                                   // LeastSquareSplit<FT,PT>,
          class FIT = FitLine<EigenFit<FT, PT>>  // MEstimatorFitLine<FT,PT>
          >
class LsdEP : public LsdExt<FT, LPT, PT> {
  ESOURCE esource_{};
  EsdPattern<typename ESOURCE::EdgeResponseFilter::mag_type, ESOURCE::NUM_DIR, CORNER_RULE> edge_{};
  SPLIT split_{};
  FIT fit_{};
  int flags_{};

  typename LsdExt<FT, LPT, PT>::PointVector points_{};
  EdgeSegmentVector segments_{};

  mutable typename LsdBase<FT, LPT>::ImageData imageData_{};

  void init() {
    this->addManager(esource_);
    this->addManager(edge_);
    this->addManager(split_);
    this->addManager(fit_);

    this->add(
        "line_flags",
        std::bind(&LsdEP<FT, LPT, CORNER_RULE, PT, ESOURCE, SPE, SPLIT, FIT>::valueFlags, this, std::placeholders::_1),
        "Flags for line detector: 0 - none, 2 - use precise sub pixel estimation");
  }

  using LsdBase<FT, LPT>::endPoints_;
  using LsdBase<FT, LPT>::lineSegments_;

  virtual void clearData() final {
    LsdBase<FT, LPT>::clearData();
    imageData_.clear();
  }

 public:
  typedef FT float_type;                                                                 ///< Floating-point type
  typedef LPT<FT> line_point;                                                            ///< Line point type
  typedef PT point_type;                                                                 ///< Support point type
  typedef EsdPattern<typename ESOURCE::EdgeResponseFilter::mag_type, CORNER_RULE> Edge;  ///< Edge pattern type

  typedef typename LsdBase<FT, LPT>::Line Line;                            ///< Line type
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;                ///< Vector of lines
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;              ///< Line segment type
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;  ///< Vector of segments
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;                  ///< Image data type
  typedef typename LsdExt<FT, LPT, PT>::PointVector PointVector;           ///< Vector of points

  /// @brief Create an edge pattern line segment detector.
  /// @param th_low Lower gradient threshold (normalized, 0-1)
  /// @param th_high Upper gradient threshold (normalized, 0-1)
  /// @param min_pix Minimum supporting pixels per edge segment
  /// @param dist Line splitting distance threshold in pixels
  /// @param min_len Minimum resulting line segment length
  /// @param pat_tol Pattern tolerance for primitive matching
  /// @param maxGap Maximum gap between consecutive edge pixels
  /// @param magMul Magnitude multiplier for pattern detection
  /// @param magTh Magnitude threshold for pattern detection
  /// @param flags Detection flags (EP_USE_PRECISE_SPE)
  LsdEP(FT th_low = static_cast<FT>(0.004),
        FT th_high = static_cast<FT>(0.012),
        int min_pix = 10,
        FT dist = 2,
        int min_len = 5,
        int pat_tol = 2,
        int maxGap = 3,
        float magMul = 3,
        float magTh = 5,
        int flags = 0)
      : esource_({NV("nms_th_low", th_low), NV("nms_th_high", th_high)}),
        edge_(min_pix, maxGap, magMul, magTh, pat_tol),
        split_(dist, min_len),
        flags_(flags) {
    init();
  }

  /// @brief Create detector from an initializer list of parameter name/value pairs.
  /// @param options Initializer list with parameter name/value pairs
  LsdEP(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  /// @brief Create detector from a vector of parameter name/value pairs.
  /// @param options Vector with parameter name/value pairs
  LsdEP(ValueManager::NameValueVector options) {
    init();
    this->value(options);
  }

  //! @brief Get or set the detection flags controlling algorithm behavior.
  //! @param f The new flags value, or Value::NAV() to only query.
  //! @return The current flags value.
  //! Supported flags:
  //! - EP_USE_PRECISE_SPE: Use precise sub-pixel estimation for endpoint localization
  Value valueFlags(const Value& f = Value::NAV()) {
    if (f.type()) flags(f.getInt());
    return flags_;
  }

  //! @brief Get the detection flags controlling algorithm behavior.
  //! @return The current flags value. See valueFlags() documentation for flag meanings.
  int flags() const { return flags_; }

  //! @brief Set the detection flags controlling algorithm behavior.
  //! @param f The new flags value. See valueFlags() documentation for flag meanings.
  void flags(int f) { flags_ = f; }

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;
  using LsdBase<FT, LPT>::imageDataDescriptor;
  using LsdBase<FT, LPT>::imageData;

  /// @brief Detect line segments using edge pattern analysis.
  /// Processes the image through edge source, pattern detection, sub-pixel estimation,
  /// splitting, and line fitting.
  /// @param image Input image (8-bit single-channel or color)
  virtual void detect(const cv::Mat& image) final {
    clearData();

    esource_.process(image);
    edge_.detect(esource_);
    if (flags_ & EL_USE_PRECISE_SPE)
      SPE::convertDir(edge_.points(), points_, esource_.magnitude(), esource_.direction());
    else
      SPE::convert(edge_.points(), points_, esource_.magnitude(), esource_.directionMap());

    split_.setup(esource_);
    split_.apply(edge_.patternSegments(), edge_.patterns(), points_, segments_);

    lineSegments_.reserve(segments_.size());
    for_each(segments_.begin(), segments_.end(), [this](const EdgeSegment& seg) {
      Line l;
      fit_.apply(this->points_.data() + seg.begin(), this->points_.data() + seg.end(), l);

      // std::cout << "reverse: " << (seg.reverse() ? "true" : "false") << std::endl;
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

  /// @brief Get descriptor information for auxiliary image data layers.
  /// @return Data descriptor with name and description for each layer
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

  /// @brief Get auxiliary image data computed during detection.
  /// @return Vector of image data layers (gx, gy, dir, mag, edge_map)
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

  //! @brief Get the patterns detected during edge pattern detection.
  //! @return Constant reference to vector of edge segments representing patterns.
  const EdgeSegmentVector& patterns() const { return edge_.patterns(); }

  //! @brief Get the edge segments detected in the image.
  //! @return Constant reference to vector of edge segments.
  const EdgeSegmentVector& segments() const { return edge_.segments(); }

  /// @brief Get line support segments (edge segments supporting each detected line).
  /// @return Const reference to line support edge segments
  virtual const EdgeSegmentVector& lineSupportSegments() const final { return segments_; }

  /// @brief Get all support points used in line detection.
  /// @return Const reference to support point vector
  virtual const PointVector& points() const final { return points_; }

  /// @brief Get point indices for edge segments.
  /// @return Const reference to index vector
  virtual const IndexVector& indexes() const final { return edge_.points(); }

  //! @brief Get access to the edge source containing gradient information.
  //! @return Reference to the edge source providing gradient and edge maps.
  ESOURCE& edgeSource() { return esource_; }

  //! @brief Get const access to the edge source containing gradient information.
  //! @return Constant reference to the edge source providing gradient and edge maps.
  const ESOURCE& edgeSource() const { return esource_; }

  //! @brief Get access to the edge detector using edge patterns.
  //! @return Reference to the pattern-based edge detector.
  Edge& edge() { return edge_; }

  //! @brief Get const access to the edge detector using edge patterns.
  //! @return Constant reference to the pattern-based edge detector.
  const Edge& edge() const { return edge_; }

  //! @brief Get access to the line segment splitter for refining detections.
  //! @return Reference to the split algorithm object.
  SPLIT& split() { return split_; }

  //! @brief Get const access to the line segment splitter for refining detections.
  //! @return Constant reference to the split algorithm object.
  const SPLIT& split() const { return split_; }

  //! @brief Get access to the line fitting algorithm.
  //! @return Reference to the line fitting object.
  FIT& fit() { return fit_; }

  //! @brief Get const access to the line fitting algorithm.
  //! @return Constant reference to the line fitting object.
  const FIT& fit() const { return fit_; }
};

}  // namespace lsfm
