//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file lsd_ld.hpp
/// @brief Line segment detector adapter using a line detector.

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

/// @brief No flags set for Ld-based detection.
static const int LD_NONE = 0;
/// @brief Enable line tracer to use direction map instead of binary edge map.
static const int LD_TRACER_DIRMAP = 1;
/// @brief Use precise direction-based sub-pixel estimation for endpoints.
static const int LD_USE_PRECISE_SPE = 2;

/// @brief Line detector to line segment detector adapter.
/// Wraps an infinite-line detector (LD) and converts its output into finite
/// line segments by tracing edges along each detected line and fitting
/// segments to the traced pixel support.
///
/// @tparam LD Underlying line detector type (must provide lines() and edgeSource())
/// @tparam LPT Line point template (default Vec2)
/// @tparam PT Point type for pixel coordinates (default Vec2i)
/// @tparam TRACER Edge-tracing algorithm type
/// @tparam SPE Sub-pixel estimator for endpoint refinement
/// @tparam FIT Line fitting algorithm type
///
/// @code{cpp}
/// lsfm::LsdLd<lsfm::LdHough<float>> detector(0.004f, 0.012f, 10, 3);
/// detector.detect(image);
/// const auto& segments = detector.lineSegments();
/// @endcode
template <class LD,
          template <class> class LPT = Vec2,
          class PT = Vec2i,
          class TRACER = LineTracer<typename LD::float_type, LPT, PT>,
          class SPE = PixelEstimator<typename LD::float_type, PT>,
          class FIT = FitLine<EigenFit<typename LD::float_type, PT>>>
class LsdLd : public LsdExt<typename LD::float_type, LPT, PT> {
  typedef LsdExt<typename LD::float_type, LPT, PT> MyLsdBase;

 public:
  typedef typename MyLsdBase::float_type float_type;                ///< Floating-point type
  typedef typename MyLsdBase::line_point line_point;                ///< Line point type
  typedef typename MyLsdBase::point_type point_type;                ///< Point type for pixel coordinates
  typedef typename MyLsdBase::PointVector PointVector;              ///< Vector of sub-pixel points
  typedef typename MyLsdBase::Line Line;                            ///< Line type
  typedef typename MyLsdBase::LineVector LineVector;                ///< Vector of lines
  typedef typename MyLsdBase::LineSegment LineSegment;              ///< Line segment type
  typedef typename MyLsdBase::LineSegmentVector LineSegmentVector;  ///< Vector of line segments
  typedef typename MyLsdBase::ImageData ImageData;                  ///< Image data type
  typedef LD LineDetector;                                          ///< Underlying line detector type
  typedef typename LD::EdgeSource EdgeSource;                       ///< Edge source type from line detector

 protected:
  using MyLsdBase::lineSegments_;

  LD lineDetector_{};  ///< Underlying line detector instance
  FIT fit_{};          ///< Line fitting algorithm
  TRACER tracer_{};    ///< Edge pixel tracer

  int flags_{};  ///< Detection flags (LD_NONE, LD_TRACER_DIRMAP, LD_USE_PRECISE_SPE)

  void init() {
    this->addManager(lineDetector_);
    this->addManager(fit_);
    this->addManager(tracer_);

    this->add("line_flags", std::bind(&LsdLd<LD, LPT, PT, TRACER, SPE, FIT>::valueFlags, this, std::placeholders::_1),
              "Flags for line detector: 0 - none, 1 - use tracer with dirmap, 2 - use precise sub pixel estimation.");
  }

  PointVector points_{};

  virtual void clearData() override {
    MyLsdBase::clearData();
    points_.clear();
  }

 public:
  /// @brief Create an Ld-based line segment detector with explicit parameters.
  /// @param th_low Lower gradient threshold for edge detection (normalized)
  /// @param th_high Upper gradient threshold for edge detection (normalized)
  /// @param minPix Minimum number of edge pixels to form a segment
  /// @param maxGap Maximum gap allowed between edge pixels when tracing
  /// @param flags Detection flags (LD_NONE, LD_TRACER_DIRMAP, LD_USE_PRECISE_SPE)
  LsdLd(float_type th_low = static_cast<float_type>(0.004),
        float_type th_high = static_cast<float_type>(0.012),
        int minPix = 10,
        int maxGap = 3,
        int flags = 0)
      : lineDetector_(th_low, th_high), tracer_(minPix, maxGap), flags_(flags) {
    init();
  }

  /// @brief Create detector from an initializer list of parameter name/value pairs.
  /// @param options Initializer list with parameter name/value pairs
  LsdLd(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  /// @brief Create detector from a vector of parameter name/value pairs.
  /// @param options Vector with parameter name/value pairs
  LsdLd(ValueManager::NameValueVector options) {
    init();
    this->value(options);
  }

  using MyLsdBase::detect;
  using MyLsdBase::endPoints;
  using MyLsdBase::imageData;
  using MyLsdBase::lines;
  using MyLsdBase::lineSegments;

  //! @brief Get or set the detection flags controlling algorithm behavior.
  //! @param f The new flags value, or Value::NAV() to only query.
  //! @return The current flags value.
  //! Supported flags:
  //! - LD_TRACER_DIRMAP: Use line tracer with direction map for enhanced tracing
  //! - LD_USE_PRECISE_SPE: Use precise sub-pixel estimation for endpoint localization
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

  /// @brief Detect line segments by running the line detector, tracing edges,
  /// and fitting line segments to the traced pixel support.
  /// @param image Input image (8-bit single-channel or color)
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

  /// @brief Get descriptor information for auxiliary image data.
  /// @return Data descriptor from the underlying line detector
  virtual const DataDescriptor& imageDataDescriptor() const final { return lineDetector_.imageDataDescriptor(); }

  /// @brief Get auxiliary image data computed during detection.
  /// @return Image data layers from the underlying line detector
  virtual const ImageData& imageData() const final { return lineDetector_.imageData(); }

  /// @brief Get the edge support segments traced along detected lines.
  /// @return Vector of edge segments defining pixel support for each line
  virtual const EdgeSegmentVector& lineSupportSegments() const final { return tracer_.segments(); }

  /// @brief Get the sub-pixel edge points used for line fitting.
  /// @return Vector of sub-pixel edge points
  virtual const PointVector& points() const final { return points_; }

  /// @brief Get the pixel-level edge indexes from the tracer.
  /// @return Vector of integer pixel indexes
  virtual const IndexVector& indexes() const final { return tracer_.indexes(); }

  //! @brief Get access to the underlying line detector object.
  //! @return Reference to the line detector used for initial line detection.
  LineDetector& lineDetector() { return lineDetector_; }

  //! @brief Get const access to the underlying line detector object.
  //! @return Constant reference to the line detector used for initial line detection.
  const LineDetector& lineDetector() const { return lineDetector_; }

  //! @brief Get access to the edge source containing gradient information.
  //! @return Reference to the edge source providing gradient and edge maps.
  EdgeSource& edgeSource() { return lineDetector_.edgeSource(); }

  //! @brief Get const access to the edge source containing gradient information.
  //! @return Constant reference to the edge source providing gradient and edge maps.
  const EdgeSource& edgeSource() const { return lineDetector_.edgeSource(); }
};

/// @brief Line detector to line segment detector adapter with separate edge source.
/// Like LsdLd, but uses its own edge source (ESOURCE) for gradient computation
/// rather than relying on the line detector's built-in edge source.
///
/// @tparam LD Underlying line detector type (must provide lines() and process())
/// @tparam LPT Line point template (default Vec2)
/// @tparam PT Point type for pixel coordinates (default Vec2i)
/// @tparam TRACER Edge-tracing algorithm type
/// @tparam SPE Sub-pixel estimator for endpoint refinement
/// @tparam FIT Line fitting algorithm type
/// @tparam ESOURCE Edge source providing independent gradient computation and NMS
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
  typedef typename MyLsdBase::float_type float_type;                ///< Floating-point type
  typedef typename MyLsdBase::line_point line_point;                ///< Line point type
  typedef typename MyLsdBase::point_type point_type;                ///< Point type for pixel coordinates
  typedef typename MyLsdBase::PointVector PointVector;              ///< Vector of sub-pixel points
  typedef typename MyLsdBase::Line Line;                            ///< Line type
  typedef typename MyLsdBase::LineVector LineVector;                ///< Vector of lines
  typedef typename MyLsdBase::LineSegment LineSegment;              ///< Line segment type
  typedef typename MyLsdBase::LineSegmentVector LineSegmentVector;  ///< Vector of line segments
  typedef typename MyLsdBase::ImageData ImageData;                  ///< Image data type
  typedef LD LineDetector;                                          ///< Underlying line detector type
  typedef ESOURCE EdgeSource;                                       ///< Edge source type


  using MyLsdBase::lineSegments_;

  ESOURCE esource_{};  ///< Edge source for gradient and NMS
  LD lineDetector_{};  ///< Underlying line detector instance
  FIT fit_{};          ///< Line fitting algorithm
  TRACER tracer_{};    ///< Edge pixel tracer

  cv::Mat binaryEdgeMap_{};  ///< Binary edge map from hysteresis thresholding

  int flags_{};  ///< Detection flags (LD_NONE, LD_TRACER_DIRMAP, LD_USE_PRECISE_SPE)

  void init() {
    this->addManager(lineDetector_);
    this->addManager(esource_);
    this->addManager(fit_);
    this->addManager(tracer_);

    this->add("line_flags", std::bind(&LsdLdES<LD, LPT, PT, TRACER, SPE, FIT>::valueFlags, this, std::placeholders::_1),
              "Flags for line detector: 0 - none, 1 - use tracer with dirmap, 2 - use precise sub pixel estimation.");
  }

  typename MyLsdBase::ImageData imageData_{};  ///< Cached auxiliary image data

  PointVector points_{};  ///< Sub-pixel edge points used for fitting


  virtual void clearData() override {
    MyLsdBase::clearData();
    imageData_.clear();
    points_.clear();
  }

 public:
  /// @brief Create an LdES-based line segment detector with explicit parameters.
  /// @param th_low Lower gradient threshold for edge detection (normalized)
  /// @param th_high Upper gradient threshold for edge detection (normalized)
  /// @param minPix Minimum number of edge pixels to form a segment
  /// @param maxGap Maximum gap allowed between edge pixels when tracing
  /// @param flags Detection flags (LD_NONE, LD_TRACER_DIRMAP, LD_USE_PRECISE_SPE)
  LsdLdES(float_type th_low = static_cast<float_type>(0.004),
          float_type th_high = static_cast<float_type>(0.012),
          int minPix = 10,
          int maxGap = 3,
          int flags = 0)
      : lineDetector_(th_low, th_high), tracer_(minPix, maxGap) {
    init();
  }

  /// @brief Create detector from an initializer list of parameter name/value pairs.
  /// @param options Initializer list with parameter name/value pairs
  LsdLdES(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  /// @brief Create detector from a vector of parameter name/value pairs.
  /// @param options Vector with parameter name/value pairs
  LsdLdES(ValueManager::NameValueVector options) {
    init();
    this->value(options);
  }

  using MyLsdBase::detect;
  using MyLsdBase::endPoints;
  using MyLsdBase::imageData;
  using MyLsdBase::lines;
  using MyLsdBase::lineSegments;

  //! @brief Get or set the detection flags controlling algorithm behavior.
  //! @param f The new flags value, or Value::NAV() to only query.
  //! @return The current flags value.
  //! Supported flags:
  //! - LD_TRACER_DIRMAP: Use line tracer with direction map for enhanced tracing
  //! - LD_USE_PRECISE_SPE: Use precise sub-pixel estimation for endpoint localization
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

  /// @brief Detect line segments using a separate edge source.
  /// Runs the line detector and edge source independently, then traces
  /// edges and fits line segments to the traced pixel support.
  /// @param image Input image (8-bit single-channel or color)
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

  /// @brief Get descriptor information for auxiliary image data.
  /// Merges descriptors from the line detector with gradient data from the
  /// edge source, adding any missing layers.
  /// @return Data descriptor with name and description for each layer
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

  /// @brief Get auxiliary image data computed during detection.
  /// Combines image data from the line detector with gradient layers from
  /// the edge source, adding any missing layers.
  /// @return Vector of image data layers
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

  /// @brief Get the edge support segments traced along detected lines.
  /// @return Vector of edge segments defining pixel support for each line
  virtual const EdgeSegmentVector& lineSupportSegments() const final { return tracer_.segments(); }

  /// @brief Get the sub-pixel edge points used for line fitting.
  /// @return Vector of sub-pixel edge points
  virtual const PointVector& points() const final { return points_; }

  /// @brief Get the pixel-level edge indexes from the tracer.
  /// @return Vector of integer pixel indexes
  virtual const IndexVector& indexes() const final { return tracer_.indexes(); }

  //! @brief Get access to the underlying line detector object.
  //! @return Reference to the line detector used for initial line detection.
  LineDetector& lineDetector() { return lineDetector_; }

  //! @brief Get const access to the underlying line detector object.
  //! @return Constant reference to the line detector used for initial line detection.
  const LineDetector& lineDetector() const { return lineDetector_; }

  //! @brief Get access to the edge source containing gradient information.
  //! @return Reference to the edge source providing gradient and edge maps.
  EdgeSource& edgeSource() { return esource_; }

  //! @brief Get const access to the edge source containing gradient information.
  //! @return Constant reference to the edge source providing gradient and edge maps.
  const EdgeSource& edgeSource() const { return esource_; }
};

}  // namespace lsfm
