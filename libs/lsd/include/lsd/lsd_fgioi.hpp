//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file lsd_fgioi.hpp
/// @brief Line Segment Detector using the FGioi (Fusion of Global and Local Information)

#pragma once

// External implementation header (third-party code - AGPL)
#include <lsd/lsd_base.hpp>

#include <cstdlib>
#include <lsd_fgioi.hpp>

namespace lsfm {

/// @brief Line Segment Detector using the FGioi (Fusion of Global and Local Information) algorithm.
/// Also known as the Probabilistic Line Segment Detector (PLSD).
/// This detector uses the Number of False Alarms (NFA) principle for statistical validation.
///
/// **Key Features:**
/// - Gradient quantization with configurable error bounds
/// - Angle-based gradient matching with tolerance
/// - NFA-based statistical validation
/// - Density-based validation of aligned regions
/// - Configurable gradient magnitude bins
///
/// @tparam FT Floating-point type for line coordinates (float, double)
/// @tparam LPT Line point template class (default Vec2)
///
/// **Example:**
/// @example examples/lsd/lsd.cpp
///
/// @code{cpp}
/// #include <lsd/lsd_fgioi.hpp>
/// #include <opencv2/imgcodecs.hpp>
///
/// cv::Mat image = cv::imread("input.png", cv::IMREAD_GRAYSCALE);
///
/// // Create detector with default parameters
/// lsfm::LsdFGioi<float> detector(2.0f, 22.5f, 0.0f, 0.7f, 1024);
///
/// // Detect line segments
/// detector.detect(image);
/// const auto& segments = detector.lineSegments();
/// @endcode
template <class FT, template <class> class LPT = Vec2>
class LsdFGioi : public LsdBase<FT, LPT> {
  FT quant_{}, angle_th_{}, log_eps_{}, density_th_{};  ///< Detection parameters
  int num_bins_{};                                      ///< Number of gradient magnitude bins

  /// @brief Initialize the detector and register all parameters.
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
  typedef FT float_type;                                                   ///< Floating-point type
  typedef LPT<FT> line_point;                                              ///< Line point type
  typedef typename LsdBase<FT, LPT>::Line Line;                            ///< Line type
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;                ///< Vector of lines
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;              ///< Line segment type
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;  ///< Vector of segments
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;                  ///< Image data type

  /// @brief Additional data associated with each detected line.
  /// Includes geometric and statistical information.
  struct LineData {
    /// @brief Create a LineData structure.
    /// @param w Line width or support region property
    /// @param p Precision/confidence of the detection
    /// @param n Number of False Alarms (NFA) value
    LineData(double w = 0, double p = 0, double n = 0) : width(w), prec(p), nfa(n) {}

    double width{};  ///< Width of line support region
    double prec{};   ///< Precision/confidence value
    double nfa{};    ///< Number of False Alarms (statistical measure)
  };

  typedef std::vector<LineData> LineDataVector;  ///< Vector of line data structures

 private:
  LineDataVector lineData_{};  ///< Storage for line-associated data

 public:
  /// @brief Create a FGioi line detector with specified parameters.
  /// This detector implements the Fusion of Global and Local Information algorithm,
  /// which combines global NFA principles with local alignment validation.
  /// @param quant Quantization error bound relative to gradient norm (typically 0-5)
  /// @param ang_th Gradient angle tolerance in degrees for alignment (typically 10-45)
  /// @param log_eps NFA detection threshold: -log10(NFA) > log_eps (typically 0-5)
  /// @param density_th Minimum density of aligned points in line rectangle (0-1, typically 0.5-0.9)
  /// @param n_bins Number of gradient magnitude histogram bins (typically 1024-4096)
  /// @code{cpp}
  /// // Sensitive to weak structures
  /// lsfm::LsdFGioi<float> sensitive(0.5f, 45.0f, 0.0f, 0.5f, 1024);
  ///
  /// // Conservative, only strong clear lines
  /// lsfm::LsdFGioi<float> conservative(5.0f, 10.0f, 5.0f, 0.9f, 4096);
  /// @endcode
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

  /// @brief Create detector from an initializer list.
  /// @param options Initializer list with parameter name/value pairs
  LsdFGioi(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  /// @brief Create detector from parameter vector.
  /// @param options Vector with parameter name/value pairs
  LsdFGioi(const ValueManager::NameValueVector& options) {
    init();
    this->value(options);
  }

  /// @brief Get/set quantization error via ValueManager interface.
  /// @param q New quantization error value (optional)
  /// @return Current or updated quantization error
  Value valueQuantError(const Value& q = Value::NAV()) {
    if (q.type()) quantError(q.get<FT>());
    return quant_;
  }

  /// @brief Get current quantization error bound.
  /// @return Quantization error relative to gradient norm
  FT quantError() const { return quant_; }

  /// @brief Set quantization error bound relative to gradient norm.
  /// @param q Quantization error (typically 0-5)
  void quantError(FT q) { quant_ = q; }

  /// @brief Get/set angle threshold via ValueManager interface.
  /// @param t New angle threshold value in degrees (optional)
  /// @return Current or updated angle threshold
  Value valueAngleThreshold(const Value& t = Value::NAV()) {
    if (t.type()) angleThreshold(t.get<FT>());
    return angle_th_;
  }

  /// @brief Get current gradient angle tolerance threshold.
  /// @return Angle tolerance in degrees
  FT angleThreshold() const { return angle_th_; }

  /// @brief Set gradient angle tolerance threshold.
  /// Controls how strictly gradient angles must align for line detection.
  /// @param t Angle tolerance in degrees (typically 10-45)
  void angleThreshold(FT t) { angle_th_ = t; }

  /// @brief Get/set log epsilon via ValueManager interface.
  /// @param e New log epsilon value (optional)
  /// @return Current or updated log epsilon
  Value valueLogEps(const Value& e = Value::NAV()) {
    if (e.type()) logEps(e.get<FT>());
    return log_eps_;
  }

  /// @brief Get current NFA detection threshold (logarithmic scale).
  /// @return Log epsilon value (-log10(NFA) threshold)
  FT logEps() const { return log_eps_; }

  /// @brief Set NFA detection threshold in logarithmic scale.
  /// Higher values result in fewer, more statistically significant detections.
  /// @param e Log epsilon value (typically 0-5, -log10(NFA) > log_eps)
  void logEps(FT e) { log_eps_ = e; }

  /// @brief Get/set density threshold via ValueManager interface.
  /// @param t New density threshold value (optional)
  /// @return Current or updated density threshold
  Value valueDensityThreshold(const Value& t = Value::NAV()) {
    if (t.type()) densityThreshold(t.get<FT>());
    return density_th_;
  }

  /// @brief Get current minimum density of aligned points.
  /// @return Density threshold (0-1)
  FT densityThreshold() const { return density_th_; }

  /// @brief Set minimum density of aligned region points in rectangle.
  /// @param t Density threshold (0-1, typically 0.5-0.9)
  void densityThreshold(FT t) { density_th_ = t; }

  /// @brief Get/set bins via ValueManager interface.
  /// @param b New bin count (optional)
  /// @return Current or updated bin count
  Value valueBins(const Value& b = Value::NAV()) {
    if (b.type()) bins(b.getInt());
    return num_bins_;
  }

  /// @brief Get current number of gradient magnitude histogram bins.
  /// @return Number of bins
  int bins() const { return num_bins_; }

  /// @brief Set number of gradient magnitude histogram bins.
  /// More bins provide finer magnitude quantization.
  /// @param b Number of bins (typically 1024-4096)
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

    double* segs_orig = segs;  // Keep original pointer for freeing
    lineSegments_.reserve(static_cast<size_t>(seg_num));
    lineData_.reserve(static_cast<size_t>(seg_num));
    for (int i = 0; i != seg_num; ++i, segs += 7) {
      lineSegments_.push_back(LineSegment(LPT<FT>(static_cast<FT>(segs[0]), static_cast<FT>(segs[1])),
                                          LPT<FT>(static_cast<FT>(segs[2]), static_cast<FT>(segs[3]))));
      lineData_.push_back(LineData(segs[4], segs[5], segs[6]));
    }

    // Free memory allocated by LineSegmentDetection
    std::free(segs_orig);
  }

  const LineDataVector& lineData() const { return lineData_; }
};
}  // namespace lsfm
