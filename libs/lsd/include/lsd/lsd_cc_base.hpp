//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file lsd_cc_base.hpp
/// @brief Base class for line segment detectors based on connected components.

#pragma once

#include <edge/fit.hpp>
#include <edge/index.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <lsd/lsd_base.hpp>

#include <functional>
#include <memory>
#include <vector>

namespace lsfm {

/// @brief Base class for line segment detectors using connected component analysis.
/// Provides gradient computation, Non-Maxima Suppression (NMS), line fitting,
/// and edge/segment map generation as shared infrastructure for derived CC detectors.
///
/// @tparam FT Floating-point type (float, double)
/// @tparam LPT Line point template (default Vec2)
/// @tparam PT Point type for support points (default LPT<int>)
/// @tparam GRAD Gradient computation strategy
/// @tparam FIT Line fitting strategy
template <class FT,
          template <class> class LPT = Vec2,
          class PT = LPT<int>,
          class GRAD = DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
          class FIT = FitLine<EigenFit<FT, PT>>>
class LsdCCBase : public LsdBase<FT, LPT> {
 public:
  typedef FT value_type;                                                   ///< Floating-point value type
  typedef LPT<FT> line_point;                                              ///< Line point type
  typedef PT point_type;                                                   ///< Support point type
  typedef typename GRAD::mag_type mag_type;                                ///< Gradient magnitude type
  typedef typename GRAD::grad_type grad_type;                              ///< Gradient vector component type
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;                  ///< Image data type
  typedef typename LsdBase<FT, LPT>::Line Line;                            ///< Line type
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;                ///< Vector of lines
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;              ///< Line segment type
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;  ///< Vector of line segments
  typedef std::vector<PT> PointVector;                                     ///< Vector of support points

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;

  //! @brief Get the points vector containing all detected line points.
  //! Individual line segments reference ranges in this vector.
  //! @return Constant reference to vector of point coordinates.
  const PointVector& points() const { return points_; }

  //! @brief Convert point vector to linear indexes for image dimensions.
  //! @return Vector of linear indexes computed from point coordinates.
  IndexVector indexes() const {
    IndexVector ret;
    if (!points_.empty()) IndexConvert<PT>::toIndexV(points_, ret, grad_.gx().cols);
    return ret;
  }

  //! @brief Get descriptor for available internal image data layers.
  //! Describes the gradient components, magnitude, edge map, and segment map.
  //! @return Constant reference to DataDescriptor listing all available data layers.
  virtual const DataDescriptor& imageDataDescriptor() const final {
    static DataDescriptor dsc;
    if (dsc.empty()) {
      dsc.push_back(DataDescriptorEntry("gx", "Gradient in x direction"));
      dsc.push_back(DataDescriptorEntry("gy", "Gradient in y direction"));
      dsc.push_back(DataDescriptorEntry("mag", "Gradient magnitude"));
      dsc.push_back(DataDescriptorEntry("edge_map",
                                        "Edge map, indicating if pixel is on edge or not (also giving direction 0-7)"));
      dsc.push_back(
          DataDescriptorEntry("segment_map", "LineData map, indicating which pixel corresponds to which segment"));
    }
    return dsc;
  }

  //! @brief Get the internal image data layers (gradient, magnitude, edge maps).
  //! Lazily constructs the data if not yet created.
  //! @return Constant reference to vector of internal image data matrices.
  virtual const ImageData& imageData() const final {
    if (imageData_.empty()) {
      imageData_.push_back(grad_.gx());
      imageData_.push_back(grad_.gy());
      imageData_.push_back(grad_.magnitude());
      imageData_.push_back(emap_);
      imageData_.push_back(lsmap_);
    }
    return imageData_;
  }

  using LsdBase<FT, LPT>::imageData;

 protected:
  cv::Mat img_{};    ///< Input image data
  cv::Mat emap_{};   ///< Edge map (direction 0-7, -1 = no edge)
  cv::Mat lsmap_{};  ///< Line segment map (pixel-to-segment assignment)

  int rows_{};  ///< Number of image rows
  int cols_{};  ///< Number of image columns
  int size_{};  ///< Total image size (cols * rows)

  mutable ImageData imageData_{};  ///< Cached auxiliary image data

  FT th_low_{};   ///< Lower NMS threshold (normalized, 0-1)
  FT th_high_{};  ///< Upper NMS threshold (normalized, 0-1)

  /// @brief Construct the base CC detector with NMS thresholds.
  /// @param th_low Lower gradient threshold (normalized, range 0-1, e.g., 0.004 ~ 1/255)
  /// @param th_high Upper gradient threshold (normalized, range 0-1, e.g., 0.012 ~ 3/255)
  LsdCCBase(FT th_low = 0.004, FT th_high = 0.012) : th_low_(th_low), th_high_(th_high) {
    this->addManager(grad_);
    this->addManager(fit_);

    this->add("nms_th_low",
              std::bind(&LsdCCBase<FT, LPT, PT, GRAD, FIT>::valueThresholdLow, this, std::placeholders::_1),
              "Lower threshold.");
    this->add("nms_th_high",
              std::bind(&LsdCCBase<FT, LPT, PT, GRAD, FIT>::valueThresholdHigh, this, std::placeholders::_1),
              "Upper threshold.");
  }

  //! @brief Get or set the lower threshold for Non-Maxima Suppression.
  //! @param t The new threshold value, or Value::NAV() to only query.
  //! @return The current lower threshold value.
  Value valueThresholdLow(const Value& t = Value::NAV()) {
    if (t.type()) thresholdLow(t.get<FT>());
    return th_low_;
  }

  //! @brief Get the lower threshold for Non-Maxima Suppression.
  //! @return The current lower threshold value. Range [0..1] normalized (0.004 ~ 1/255).
  FT thresholdLow() const { return th_low_; }

  //! @brief Set the lower threshold for Non-Maxima Suppression.
  //! @param t The new lower threshold value. Range [0..1] normalized (0.004 ~ 1/255).
  void thresholdLow(FT t) { th_low_ = t; }

  //! @brief Get or set the upper threshold for Non-Maxima Suppression.
  //! @param t The new threshold value, or Value::NAV() to only query.
  //! @return The current upper threshold value.
  Value valueThresholdHigh(const Value& t = Value::NAV()) {
    if (t.type()) thresholdHigh(t.get<FT>());
    return th_high_;
  }

  //! @brief Get the upper threshold for Non-Maxima Suppression.
  //! @return The current upper threshold value. Range [0..1] normalized (0.004 ~ 1/255).
  FT thresholdHigh() const { return th_high_; }

  //! @brief Set the upper threshold for Non-Maxima Suppression.
  //! @param t The new upper threshold value. Range [0..1] normalized (0.004 ~ 1/255).
  void thresholdHigh(FT t) { th_high_ = t; }

  //! @brief Set both lower and upper thresholds for Non-Maxima Suppression.
  //! @param low The new lower threshold value.
  //! @param high The new upper threshold value.
  void threshold(FT low, FT high) {
    th_low_ = low;
    th_high_ = high;
  }


  std::vector<PT> seeds_{};  ///< Seed edge pixels from Non-Maxima Suppression

  PointVector points_{};  ///< All line support points sorted by segment

  GRAD grad_{};  ///< Gradient computation object
  FIT fit_{};    ///< Line fitting object

  /// @brief Preprocess the image by computing gradient, magnitude, and NMS.
  void preprocess() {
    grad_.process(img_);
    computeNMS();
  }

  /// @brief Clear all internal detection data.
  virtual void clearData() final {
    LsdBase<FT, LPT>::clearData();
    imageData_.clear();
  }

 private:
  /// @brief Compute Non-Maxima Suppression on gradient magnitude.
  /// Produces an edge map with direction labels and collects seed pixels
  /// that exceed the upper threshold.
  inline void computeNMS() {
    static const int TG22 = static_cast<int>(0.4142135623730950488016887242097 * (1 << 15) + 0.5);

    seeds_.clear();
    seeds_.reserve(static_cast<size_t>(size_ / 3));
    emap_.create(rows_, cols_, CV_8SC1);
    emap_.row(0).setTo(-1);
    emap_.row(rows_ - 1).setTo(-1);

    mag_type low = grad_.magnitudeThreshold(th_low_);
    mag_type high = grad_.magnitudeThreshold(th_high_);

    if (low < 0) low = 0;
    if (high < 0) high = 0;

    if (low > high) std::swap(low, high);

    mag_type* pmag = grad_.magnitude().template ptr<mag_type>() + cols_;
    grad_type* pgx = grad_.gx().template ptr<grad_type>() + cols_;
    grad_type* pgy = grad_.gy().template ptr<grad_type>() + cols_;
    char* pemap = emap_.ptr<char>() + cols_;

    int r = rows_ - 1, c = cols_ - 1;
    for (int y = 1; y != r; ++y, pgx += cols_, pgy += cols_, pemap += cols_, pmag += cols_) {
      pemap[0] = pemap[c] = -1;
      for (int x = 1; x != c; ++x) {
        /* region numbers
        (Top-Left Origin)

        1   2   3
         \  |  /
          \ | /
        0 --+-- 4
          / | \
         /  |  \
        7   6   5
        */

        /* region numbers
        (Top-Left Origin)
        */
        mag_type m = pmag[x];
        pemap[x] = -1;

        if (m > low) {
          mag_type xs = static_cast<int>(pgx[x]);
          mag_type ys = static_cast<int>(pgy[x]);
          mag_type axs = std::abs(xs);
          mag_type ays = std::abs(ys) * 32768;

          int tg22x = axs * TG22;

          // normal -22 < m < 22 deg -> edge at region 3 -> look left, look right
          if (ays < tg22x) {
            if (m > pmag[x - 1] && m >= pmag[x + 1]) {
              pemap[x] = (xs < 0) ? 2 : 6;
              if (m > high) seeds_.push_back(PT(x, y));
            }
          } else {
            int tg67x = tg22x + axs * 65536;
            // normal 67 < m < 112 -> edge at region 1 -> look up, look down
            if (ays > tg67x) {
              if (m > pmag[x - cols_] && m >= pmag[x + cols_]) {
                pemap[x] = (ys < 0) ? 4 : 0;
                if (m > high) seeds_.push_back(PT(x, y));
              }
            } else {
              // normal 22 <= m <= 67 || 112 <= m <= 157 -> region 2 or 4 -> look diagonal
              int s = neg_sign(xs, ys) ? -1 : 1;
              if (m > pmag[x - cols_ - s] && m > pmag[x + cols_ + s]) {
                pemap[x] = (s > 0 ? (xs < 0 ? 3 : 7) : (xs < 0 ? 1 : 5));
                if (m > high) seeds_.push_back(PT(x, y));
              }
            }
          }
        }
      }
    }
  }
};
}  // namespace lsfm
