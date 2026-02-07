//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file lsd_hcv.hpp
/// @brief Hough Transform based line detector.

#pragma once

#include <edge/edge_source.hpp>
#include <lsd/lsd_ld.hpp>
#include <opencv2/opencv.hpp>

namespace lsfm {

/// @brief Standard Hough Transform based line detector.
/// Wraps OpenCV's HoughLines with integrated gradient-based edge detection.
/// Detects infinite lines (not segments) in the Hough (rho, theta) parameter space.
///
/// @tparam FT Floating-point type (float, double)
/// @tparam LPT Line point template (default Vec2)
/// @tparam ESOURCE Edge source providing gradient computation and NMS
///
/// @code{cpp}
/// lsfm::LdHough<float> detector(0.004f, 0.012f, 1.5, CV_PI/180, 150);
/// detector.detect(image);
/// const auto& lines = detector.lines();
/// @endcode
template <class FT,
          template <class> class LPT = Vec2,
          class ESOURCE = EdgeSourceGRAD<DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
                                         NonMaximaSuppression<short, int, FT>>>
class LdHough : public LdBase<FT, LPT> {
  using LdBase<FT, LPT>::lines_;

  ESOURCE esource_{};                 ///< Edge source for gradient and NMS
  std::vector<cv::Vec2f> cvLines_{};  ///< Raw OpenCV Hough line output (rho, theta)

  cv::Mat binaryEdgeMap_{};                                   ///< Binary edge map from hysteresis thresholding
  mutable typename LsdBase<FT, LPT>::ImageData imageData_{};  ///< Cached auxiliary image data

  double rho_{};          ///< Distance resolution of the accumulator in pixels
  double theta_{};        ///< Angle resolution of the accumulator in radians
  double rho_msdiv_{};    ///< Multi-scale rho divisor (0 = disabled)
  double theta_msdiv_{};  ///< Multi-scale theta divisor (0 = disabled)
  int voteThreshold_{};   ///< Minimum votes to accept a line

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
  typedef FT float_type;       ///< Floating-point type
  typedef LPT<FT> line_point;  ///< Line point type

  typedef typename LdBase<FT, LPT>::Line Line;              ///< Line type
  typedef typename LdBase<FT, LPT>::LineVector LineVector;  ///< Vector of lines
  typedef typename LdBase<FT, LPT>::ImageData ImageData;    ///< Image data type
  typedef ESOURCE EdgeSource;                               ///< Edge source type

  /// @brief Create a Standard Hough Transform line detector.
  /// @param th_low Lower gradient threshold for edge detection (normalized)
  /// @param th_high Upper gradient threshold for edge detection (normalized)
  /// @param rho Distance resolution of the Hough accumulator in pixels
  /// @param theta Angle resolution of the Hough accumulator in radians
  /// @param voteThreshold Minimum accumulator votes to accept a line
  /// @param rho_msdiv Multi-scale rho divisor (0 = no multi-scale)
  /// @param theta_msdiv Multi-scale theta divisor (0 = no multi-scale)
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
        rho_msdiv_(rho_msdiv),
        theta_msdiv_(theta_msdiv),
        voteThreshold_(voteThreshold) {
    init();
  }

  /// @brief Create detector from an initializer list of parameter name/value pairs.
  /// @param options Initializer list with parameter name/value pairs
  LdHough(ValueManager::InitializerList options)
      : rho_(1.5), theta_(CV_PI / 180), rho_msdiv_(0), theta_msdiv_(0), voteThreshold_(150) {
    init();
    this->value(options);
  }

  /// @brief Create detector from a vector of parameter name/value pairs.
  /// @param options Vector with parameter name/value pairs
  LdHough(ValueManager::NameValueVector options)
      : rho_(1.5), theta_(CV_PI / 180), rho_msdiv_(0), theta_msdiv_(0), voteThreshold_(150) {
    init();
    this->value(options);
  }

  /// @brief Get/set distance resolution of the Hough accumulator.
  /// @param v New rho value (optional)
  /// @return Current or updated rho
  Value valueRho(const Value& v = Value::NAV()) {
    if (v.type()) rho_ = v.getDouble();
    return rho_;
  }

  /// @brief Get/set angle resolution of the Hough accumulator.
  /// @param v New theta value in radians (optional)
  /// @return Current or updated theta
  Value valueTheta(const Value& v = Value::NAV()) {
    if (v.type()) theta_ = v.getDouble();
    return theta_;
  }

  /// @brief Get/set the minimum vote threshold for line acceptance.
  /// @param v New vote threshold (optional)
  /// @return Current or updated vote threshold
  Value valueVoteTh(const Value& v = Value::NAV()) {
    if (v.type()) voteThreshold_ = v.getInt();
    return voteThreshold_;
  }

  /// @brief Get/set multi-scale rho divisor.
  /// @param v New rho divisor (optional, 0 = no multi-scale)
  /// @return Current or updated rho divisor
  Value valueRhoMsDiv(const Value& v = Value::NAV()) {
    if (v.type()) rho_msdiv_ = v.getDouble();
    return rho_msdiv_;
  }

  /// @brief Get/set multi-scale theta divisor.
  /// @param v New theta divisor (optional, 0 = no multi-scale)
  /// @return Current or updated theta divisor
  Value valueThetaMsDiv(const Value& v = Value::NAV()) {
    if (v.type()) theta_msdiv_ = v.getDouble();
    return theta_msdiv_;
  }

  using LdBase<FT, LPT>::detect;
  using LdBase<FT, LPT>::lines;
  using LdBase<FT, LPT>::imageData;

  /// @brief Detect lines using Standard Hough Transform.
  /// Computes gradients and NMS on the input image, then runs
  /// OpenCV HoughLines on the resulting binary edge map.
  /// @param image Input image (8-bit single-channel or color)
  virtual void detect(const cv::Mat& image) final {
    clearData();
    esource_.process(image);
    binaryEdgeMap_ = esource_.hysteresis_binary();

    cv::HoughLines(binaryEdgeMap_, cvLines_, rho_, theta_, voteThreshold_);

    for_each(cvLines_.begin(), cvLines_.end(), [this](const cv::Vec2f& line) {
      this->lines_.push_back(Line(static_cast<FT>(line[1]), static_cast<FT>(line[0])));
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
                                        "Dir map, indicating if pixel is on edge or not (also giving direction 0-7)"));
      dsc.push_back(
          DataDescriptorEntry("canny_map", "Binary edge map, indicating if pixel is on edge or not after hysteresis"));
    }
    return dsc;
  }

  /// @brief Get auxiliary image data computed during detection.
  /// @return Vector of image data layers (gx, gy, dir, mag, edge_map, canny_map)
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

  /// @brief Get mutable reference to the edge source.
  /// @return Mutable reference to edge source object
  EdgeSource& edgeSource() { return esource_; }

  /// @brief Get const reference to the edge source.
  /// @return Const reference to edge source object
  const EdgeSource& edgeSource() const { return esource_; }
};

/// @brief Type alias: Standard Hough Transform with line segment output.
/// Combines LdHough (infinite line detection) with LsdLd (line-to-segment
/// conversion) using line tracing to produce finite line segments.
///
/// @tparam FT Floating-point type (float, double)
/// @tparam LPT Line point template (default Vec2)
/// @tparam PT Point type for pixel coordinates (default Vec2i)
/// @tparam TRACER Line tracer type for pixel-level line tracing
/// @tparam SPE Sub-pixel estimator type
/// @tparam FIT Line fitting algorithm type
/// @tparam ESOURCE Edge source providing gradient computation and NMS
template <class FT,
          template <class> class LPT = Vec2,
          class PT = Vec2i,
          class TRACER = LineTracer<FT, LPT, PT, 4>,
          class SPE = PixelEstimator<FT, PT>,
          class FIT = FitLine<EigenFit<FT, PT>>,
          class ESOURCE = EdgeSourceGRAD<DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
                                         NonMaximaSuppression<short, int, FT, FastNMS4<short, int, FT>>>>
using LsdHough = LsdLd<LdHough<FT, LPT, ESOURCE>, LPT, PT, TRACER, SPE, FIT>;

/// @brief Probabilistic Hough Transform line segment detector.
/// Wraps OpenCV's HoughLinesP with integrated gradient-based edge detection.
/// Directly produces line segments (not infinite lines).
///
/// @tparam FT Floating-point type (float, double)
/// @tparam LPT Line point template (default Vec2)
/// @tparam ESOURCE Edge source providing gradient computation and NMS
///
/// @code{cpp}
/// lsfm::LsdHoughP<float> detector(0.004f, 0.012f, 1.5, CV_PI/180, 150, 10, 3);
/// detector.detect(image);
/// const auto& segments = detector.lineSegments();
/// @endcode
template <class FT,
          template <class> class LPT = Vec2,
          class ESOURCE = EdgeSourceGRAD<DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
                                         NonMaximaSuppression<short, int, FT>>>
class LsdHoughP : public LsdBase<FT, LPT> {
  using LsdBase<FT, LPT>::lineSegments_;

  ESOURCE esource_;                 ///< Edge source for gradient and NMS
  std::vector<cv::Vec4i> cvLines_;  ///< Raw OpenCV HoughLinesP output (x1,y1,x2,y2)

  cv::Mat binaryEdgeMap_;                                   ///< Binary edge map from hysteresis thresholding
  mutable typename LsdBase<FT, LPT>::ImageData imageData_;  ///< Cached auxiliary image data

  double rho_;         ///< Distance resolution of the accumulator in pixels
  double theta_;       ///< Angle resolution of the accumulator in radians
  double minLength_;   ///< Minimum segment length to accept
  double maxGap_;      ///< Maximum gap to merge collinear segments
  int voteThreshold_;  ///< Minimum votes to accept a line

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
  typedef FT float_type;       ///< Floating-point type
  typedef LPT<FT> line_point;  ///< Line point type

  typedef typename LsdBase<FT, LPT>::Line Line;                            ///< Line type
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;                ///< Vector of lines
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;              ///< Line segment type
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;  ///< Vector of line segments
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;                  ///< Image data type

  /// @brief Create a Probabilistic Hough Transform detector.
  /// @param th_low Lower gradient threshold for edge detection (normalized)
  /// @param th_high Upper gradient threshold for edge detection (normalized)
  /// @param rho Distance resolution of the Hough accumulator in pixels
  /// @param theta Angle resolution of the Hough accumulator in radians
  /// @param voteThreshold Minimum accumulator votes to accept a line
  /// @param minLength Minimum line segment length to accept
  /// @param maxGap Maximum gap between collinear points to link them
  LsdHoughP(FT th_low = static_cast<FT>(0.004),
            FT th_high = static_cast<FT>(0.012),
            double rho = 1.5,
            double theta = CV_PI / 180,
            int voteThreshold = 150,
            double minLength = 10,
            double maxGap = 3)
      : esource_({NV("nms_th_low", th_low), NV("nms_th_high", th_high)}),
        cvLines_(),
        binaryEdgeMap_(),
        imageData_(),
        rho_(rho),
        theta_(theta),
        minLength_(minLength),
        maxGap_(maxGap),
        voteThreshold_(voteThreshold) {
    init();
  }

  /// @brief Create detector from an initializer list of parameter name/value pairs.
  /// @param options Initializer list with parameter name/value pairs
  LsdHoughP(ValueManager::InitializerList options)
      : esource_(),
        cvLines_(),
        binaryEdgeMap_(),
        imageData_(),
        rho_(1.5),
        theta_(CV_PI / 180),
        minLength_(10),
        maxGap_(3),
        voteThreshold_(150) {
    init();
    this->value(options);
  }

  /// @brief Create detector from a vector of parameter name/value pairs.
  /// @param options Vector with parameter name/value pairs
  LsdHoughP(ValueManager::NameValueVector options)
      : esource_(),
        cvLines_(),
        binaryEdgeMap_(),
        imageData_(),
        rho_(1.5),
        theta_(CV_PI / 180),
        minLength_(10),
        maxGap_(2),
        voteThreshold_(150) {
    init();
    this->value(options);
  }

  /// @brief Get/set distance resolution of the Hough accumulator.
  /// @param v New rho value (optional)
  /// @return Current or updated rho
  Value valueRho(const Value& v = Value::NAV()) {
    if (v.type()) rho_ = v.getDouble();
    return rho_;
  }

  /// @brief Get/set angle resolution of the Hough accumulator.
  /// @param v New theta value in radians (optional)
  /// @return Current or updated theta
  Value valueTheta(const Value& v = Value::NAV()) {
    if (v.type()) theta_ = v.getDouble();
    return theta_;
  }

  /// @brief Get/set the minimum vote threshold for line acceptance.
  /// @param v New vote threshold (optional)
  /// @return Current or updated vote threshold
  Value valueVoteTh(const Value& v = Value::NAV()) {
    if (v.type()) voteThreshold_ = v.getInt();
    return voteThreshold_;
  }

  /// @brief Get/set the minimum line segment length.
  /// Segments shorter than this value are rejected.
  /// @param v New minimum length (optional)
  /// @return Current or updated minimum length
  Value valueMinLen(const Value& v = Value::NAV()) {
    if (v.type()) minLength_ = v.getDouble();
    return minLength_;
  }

  /// @brief Get/set the maximum gap allowed when linking collinear points.
  /// @param v New maximum gap (optional)
  /// @return Current or updated maximum gap
  Value valueMaxGap(const Value& v = Value::NAV()) {
    if (v.type()) maxGap_ = v.getDouble();
    return maxGap_;
  }

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;
  using LsdBase<FT, LPT>::imageData;

  /// @brief Detect line segments using Probabilistic Hough Transform.
  /// Computes gradients and NMS on the input image, then runs
  /// OpenCV HoughLinesP on the resulting binary edge map.
  /// @param image Input image (8-bit single-channel or color)
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
                                        "Dir map, indicating if pixel is on edge or not (also giving direction 0-7)"));
      dsc.push_back(
          DataDescriptorEntry("canny_map", "Binary edge map, indicating if pixel is on edge or not after hysteresis"));
    }
    return dsc;
  }

  /// @brief Get auxiliary image data computed during detection.
  /// @return Vector of image data layers (gx, gy, dir, mag, edge_map, canny_map)
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

  /// @brief Get mutable reference to the edge source.
  /// @return Mutable reference to edge source object
  ESOURCE& edgeSource() { return esource_; }

  /// @brief Get const reference to the edge source.
  /// @return Const reference to edge source object
  const ESOURCE& edgeSource() const { return esource_; }
};
}  // namespace lsfm
