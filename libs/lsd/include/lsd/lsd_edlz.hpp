//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file lsd_edlz.hpp
/// @brief Line segment detector using EDLines (Edge Drawing Lines) algorithm.

#pragma once

#include <lsd_edlz.hpp>

// External implementation header (third-party code)
#include <lsd/lsd_base.hpp>
class EDLineDetector;

namespace lsfm {

/// @brief Line segment detector using EDLines (Edge Drawing Lines) algorithm.
/// The EDLines algorithm detects line segments from image edges with high precision.
/// Based on the Edge Drawing algorithm by Cihan Topal and Cuneyt Akinlar.
///
/// **Key Features:**
/// - Gradient-based edge detection with threshold
/// - Anchor-based line detection
/// - Configurable scan intervals for line validation
/// - Automatic line fitting and validation using Number of False Alarms (NFA)
///
/// @tparam FT Floating-point type for line coordinates (float, double)
/// @tparam LPT Line point template class (default Vec2)
///
/// **Example:**
/// @example examples/lsd/lsd.cpp
///
/// @code{cpp}
/// #include <lsd/lsd_edlz.hpp>
/// #include <opencv2/imgcodecs.hpp>
///
/// cv::Mat image = cv::imread("input.png", cv::IMREAD_GRAYSCALE);
///
/// // Create detector with default parameters
/// lsfm::LsdEDLZ<float> detector(10.0f, 2.0f, 2, 15, 2.0f, false);
///
/// // Detect line segments
/// detector.detect(image);
/// const auto& lines = detector.lineSegments();
///
/// // Access auxiliary image data (gradients, edges)
/// const auto& imageData = detector.imageData();
/// const auto& dataDesc = detector.imageDataDescriptor();
/// for (size_t i = 0; i < imageData.size(); ++i) {
///   std::cout << dataDesc[i].name << ": " << dataDesc[i].description << std::endl;
/// }
/// @endcode
template <class FT, template <class> class LPT = Vec2>
class LsdEDLZ : public LsdBase<FT, LPT> {
  EDLineDetector* edl{nullptr};                       ///< Internal EDLineDetector instance
  typename LsdBase<FT, LPT>::ImageData imageData_{};  ///< Auxiliary image data (gradients, edges)

  /// @brief Deleted copy constructor (non-copyable due to pointer member).
  LsdEDLZ(const LsdEDLZ&) = delete;

  /// @brief Deleted assignment operator (non-copyable due to pointer member).
  LsdEDLZ& operator=(const LsdEDLZ&) = delete;

  /// @brief Initialize the detector and register all parameters.
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
  typedef FT float_type;                                                   ///< Floating-point type
  typedef LPT<FT> line_point;                                              ///< Line point type
  typedef typename LsdBase<FT, LPT>::Line Line;                            ///< Line type
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;                ///< Vector of lines
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;              ///< Line segment type
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;  ///< Vector of segments
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;                  ///< Image data type

  /// @brief Create an EDLines line detector with specified parameters.
  /// @param gradientThreshold Threshold for gradient magnitude (typically 5-20). Lower values detect weaker edges.
  /// @param anchorThreshold Threshold for anchor points (typically 1-5). Anchors are reliable edge points.
  /// @param scanIntervals Number of scan intervals for line continuity (typically 1-3).
  /// @param minLineLen Minimum line length in pixels (typically 10-30).
  /// @param lineFitErrThreshold Maximum line-to-segment fit error in pixels (typically 1-3).
  /// @param validate Whether to validate lines using NFA (Number of False Alarms) criterion.
  /// @code{cpp}
  /// // Sensitive detector for weak edges
  /// lsfm::LsdEDLZ<float> sensitive(5.0f, 1.0f, 1, 10, 1.0f, true);
  ///
  /// // Conservative detector for strong edges only
  /// lsfm::LsdEDLZ<float> conservative(20.0f, 5.0f, 2, 30, 3.0f, true);
  /// @endcode
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

  /// @brief Create detector from an initializer list of parameter names and values.
  /// @param options Initializer list with parameter name/value pairs
  /// @code{cpp}
  /// lsfm::LsdEDLZ<float> detector({{"grad_th", 15}, {"min_len", 20}});
  /// @endcode
  LsdEDLZ(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  /// @brief Create detector from a vector of parameter name/value pairs.
  /// @param options Vector with parameter name/value pairs
  LsdEDLZ(const ValueManager::NameValueVector& options) {
    init();
    this->value(options);
  }

  /// @brief Destructor: Clean up internal EDLineDetector instance.
  ~LsdEDLZ() { delete edl; }

  /// @brief Get/set gradient threshold via ValueManager interface.
  /// @param t New gradient threshold value (optional)
  /// @return Current or updated gradient threshold
  Value valueGradientThreshold(const Value& t = Value::NAV()) {
    if (t.type()) gradientThreshold(t.get<FT>());
    return gradientThreshold();
  }

  /// @brief Get current gradient threshold for edge detection.
  /// @return Gradient magnitude threshold
  FT gradientThreshold() const { return static_cast<FT>(edl->getParams().gradientThreshold); }

  /// @brief Set gradient threshold for edge detection.
  /// @param t New threshold value
  void gradientThreshold(FT t) {
    EDLineParam p = edl->getParams();
    p.gradientThreshold = static_cast<float>(t);
    edl->setParams(p);
  }

  /// @brief Get/set anchor threshold via ValueManager interface.
  /// @param t New anchor threshold value (optional)
  /// @return Current or updated anchor threshold
  Value valueAnchorThreshold(const Value& t = Value::NAV()) {
    if (t.type()) anchorThreshold(t.get<FT>());
    return anchorThreshold();
  }

  /// @brief Get current anchor threshold for reliable edge points.
  /// @return Anchor threshold value
  FT anchorThreshold() const { return static_cast<FT>(edl->getParams().anchorThreshold); }

  /// @brief Set anchor threshold for reliable edge point detection.
  /// @param t New threshold value
  void anchorThreshold(FT t) {
    EDLineParam p = edl->getParams();
    p.anchorThreshold = static_cast<float>(t);
    edl->setParams(p);
  }

  /// @brief Get/set line fit error via ValueManager interface.
  /// @param e New fit error value in pixels (optional)
  /// @return Current or updated fit error
  Value valueFitError(const Value& e = Value::NAV()) {
    if (e.type()) fitError(e.get<FT>());
    return fitError();
  }

  /// @brief Get current maximum line fit error threshold.
  /// @return Line fit error in pixels
  FT fitError() const { return static_cast<FT>(edl->getParams().lineFitErrThreshold); }

  /// @brief Set maximum line fit error threshold.
  /// Lower values enforce stricter line-to-segment fitting.
  /// @param e New fit error in pixels
  void fitError(FT e) {
    EDLineParam p = edl->getParams();
    p.lineFitErrThreshold = static_cast<float>(e);
    edl->setParams(p);
  }

  /// @brief Get/set scan intervals via ValueManager interface.
  /// @param si New scan intervals value (optional)
  /// @return Current or updated scan intervals
  Value valueScanIntervals(const Value& si = Value::NAV()) {
    if (si.type()) scanIntervals(si.getInt());
    return scanIntervals();
  }

  /// @brief Get current number of scan intervals for line validation.
  /// @return Number of scan intervals
  int scanIntervals() const { return edl->getParams().scanIntervals; }

  /// @brief Set number of scan intervals for line continuity validation.
  /// Higher values perform more thorough validation.
  /// @param si New number of intervals
  void scanIntervals(int si) {
    EDLineParam p = edl->getParams();
    p.scanIntervals = si;
    edl->setParams(p);
  }

  /// @brief Get/set minimum line length via ValueManager interface.
  /// @param ml New minimum length in pixels (optional)
  /// @return Current or updated minimum length
  Value valueMinLength(const Value& ml = Value::NAV()) {
    if (ml.type()) minLength(ml.getInt());
    return minLength();
  }

  /// @brief Get current minimum line segment length threshold.
  /// @return Minimum length in pixels
  int minLength() const { return edl->getParams().minLineLen; }

  /// @brief Set minimum line segment length threshold.
  /// Segments shorter than this are discarded.
  /// @param ml Minimum length in pixels
  void minLength(int ml) {
    EDLineParam p = edl->getParams();
    p.minLineLen = ml;
    edl->setParams(p);
  }

  /// @brief Get/set line validation flag via ValueManager interface.
  /// @param v Validation flag (0=disable, 1=enable)
  /// @return 1 if validation is enabled, 0 otherwise
  Value valueValidate(const Value& v = Value::NAV()) {
    if (v.type()) validate(v.getInt() != 0);
    return validate() ? 1 : 0;
  }

  /// @brief Check if NFA-based line validation is enabled.
  /// @return true if validation is enabled, false otherwise
  bool validate() const { return edl->getParams().validate; }

  /// @brief Enable or disable NFA-based line validation.
  /// When enabled, only statistically significant lines (low NFA) are retained.
  /// @param v true to enable, false to disable
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

  /// @brief Detect line segments in the input image.
  /// Applies EDLines algorithm to find line segments with high precision.
  /// @param image Input image (preferably 8-bit single-channel grayscale)
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

  /// @brief Get descriptor information for auxiliary image data.
  /// @return Data descriptor with name and description for each layer
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

  /// @brief Get auxiliary image data computed during detection.
  /// @return Vector of image data layers (gx, gy, mag, th_mag, dir_map, edge_map)
  virtual const ImageData& imageData() const final { return imageData_; }
};
}  // namespace lsfm
