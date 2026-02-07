//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file threshold_estimator.hpp
/// @brief Threshold estimation methods for edge magnitude maps.
/// Implements automatic threshold estimation including Otsu's method and other
/// statistical approaches for determining edge thresholds from magnitude histograms.

#pragma once

#include <opencv2/imgproc/imgproc.hpp>
#include <utility/range.hpp>

#include <string>


namespace lsfm {

/// @brief User-defined fixed threshold estimator.
/// Returns a constant threshold value regardless of input magnitude data.
/// @tparam IT Image element type
template <class IT>
class ThresholdUser {
  IT th_{};
  IT max_{std::numeric_limits<IT>::max()};

 public:
  /// @typedef img_type
  /// @brief Image element type
  typedef IT img_type;

  /// @brief Construct a user-defined threshold.
  /// @param r_max Maximum possible magnitude value
  /// @param th Fixed threshold value to use
  explicit ThresholdUser(IT r_max = std::numeric_limits<IT>::max(), IT th = 0) : th_(th), max_(r_max) {}

  /// @brief Return the fixed threshold value.
  /// @param mag Input magnitude map (unused)
  /// @return The user-defined threshold
  inline IT process(const cv::Mat& mag) const {
    CV_Assert(mag.type() == cv::DataType<IT>::type);
    return th_;
  }

  /// @brief Get the maximum magnitude value.
  /// @return Maximum value
  IT max() const { return max_; }


  /// @brief Get name of this threshold method.
  /// @return "user"
  static std::string name() { return "user"; }
};

/// @brief Automatic threshold estimation using Otsu's method.
/// Computes the optimal threshold by maximizing inter-class variance
/// of the histogram of magnitude values.
/// @tparam IT Image element type
/// @tparam N Number of histogram bins (default: 256)
/// @tparam FT Floating-point type for internal computation (default: float)
template <class IT, int N = 256, class FT = float>
class ThresholdOtsu {
  Range<IT> intensity_range_;
  FT scale_;

 public:
  /// @typedef img_type
  /// @brief Image element type
  typedef IT img_type;

  /// @brief Construct an Otsu threshold estimator with min/max values.
  /// @param r_max Maximum magnitude value
  /// @param r_min Minimum magnitude value
  explicit ThresholdOtsu(IT r_max = std::numeric_limits<IT>::max(), IT r_min = 0)
      : intensity_range_(r_min, r_max), scale_(static_cast<FT>(N) / static_cast<FT>(intensity_range_.size())) {}

  /// @brief Construct an Otsu threshold estimator from a range.
  /// @param r Intensity range for histogram computation
  explicit ThresholdOtsu(const Range<IT>& r)
      : intensity_range_(r), scale_(static_cast<FT>(N) / static_cast<FT>(intensity_range_.size())) {}

  /// @brief Compute the Otsu threshold from a magnitude map.
  /// Builds a histogram and finds the threshold that maximizes inter-class variance.
  /// @param mag Input magnitude map
  /// @return Computed threshold value
  inline IT process(const cv::Mat& mag) const {
    CV_Assert(mag.type() == cv::DataType<IT>::type);

    cv::Size size = mag.size();
    if (mag.isContinuous()) {
      size.width *= size.height;
      size.height = 1;
    }

    int i, j;
    size_t idx;
    size_t h[static_cast<size_t>(N)] = {0};
    for (i = 0; i < size.height; ++i) {
      const IT* src = mag.ptr<IT>(i);
      for (j = 0; j < size.width; ++j) {
        idx = static_cast<size_t>(src[j] * scale_);
        if (idx < N) h[idx]++;
      }
    }

    FT mu = 0, scale = static_cast<FT>(1.) / static_cast<FT>(size.width * size.height);
    for (i = 0; i < N; ++i) mu += static_cast<FT>(i) * static_cast<FT>(h[i]);

    mu *= scale;
    FT mu1 = 0, q1 = 0;
    FT max_sigma = 0, max_val = 0;

    for (i = 0; i < N; ++i) {
      FT p_i, q2, mu2, sigma;

      p_i = static_cast<FT>(h[i]) * scale;
      mu1 *= q1;
      q1 += p_i;
      q2 = static_cast<FT>(1.) - q1;

      if (std::min(q1, q2) < std::numeric_limits<FT>::epsilon() ||
          std::max(q1, q2) > static_cast<FT>(1.) - std::numeric_limits<FT>::epsilon())
        continue;

      mu1 = (mu1 + static_cast<FT>(i) * p_i) / q1;
      mu2 = (mu - q1 * mu1) / q2;
      sigma = q1 * q2 * (mu1 - mu2) * (mu1 - mu2);
      if (sigma > max_sigma) {
        max_sigma = sigma;
        max_val = static_cast<FT>(i);
      }
    }

    return static_cast<IT>(max_val / scale_);
  }

  /// @brief Get the maximum magnitude value.
  /// @return Maximum value from the intensity range
  IT max() const { return intensity_range_.upper; }


  /// @brief Get the name of this threshold method.
  /// @return "otsu"
  static std::string name() { return "otsu"; }
};

/// @brief Commented-out specialization for Otsu method with uchar type.
// template <class FT>
// class ThresholdOtsu<uchar, 256, FT> {
//   FT scale_;

//  public:
//   typedef uchar img_type;

//   explicit ThresholdOtsu(uchar rMax = std::numeric_limits<uchar>::max()) {
//     scale_ = static_cast<FT>(256) / static_cast<FT>(rMax);
//   }

//   explicit ThresholdOtsu(const Range<uchar>& r) { scale_ = static_cast<FT>(256) / static_cast<FT>(r.size()); }

//   //! Compute threshold
//   inline uchar process(const cv::Mat& mag) const {
//     CV_Assert(mag.type() == cv::DataType<uchar>::type);

//     cv::Size size = mag.size();
//     if (mag.isContinuous()) {
//       size.width *= size.height;
//       size.height = 1;
//     }

//     const int N = 256;
//     int i, j, h[N] = {0};
//     for (i = 0; i < size.height; ++i) {
//       const uchar* src = mag.ptr(i);
//       for (j = 0; j < size.width; ++j) h[src[j]]++;
//     }

//     FT mu = 0, scale = static_cast<FT>(1.) / (size.width * size.height);
//     for (i = 0; i < N; ++i) mu += i * static_cast<FT>(h[i]);

//     mu *= scale;
//     FT mu1 = 0, q1 = 0;
//     FT max_sigma = 0, max_val = 0;

//     for (i = 0; i < N; ++i) {
//       FT p_i, q2, mu2, sigma;

//       p_i = h[i] * scale;
//       mu1 *= q1;
//       q1 += p_i;
//       q2 = static_cast<FT>(1) - q1;

//       if (std::min(q1, q2) < std::numeric_limits<FT>::epsilon() ||
//           std::max(q1, q2) > static_cast<FT>(1.) - std::numeric_limits<FT>::epsilon())
//         continue;

//       mu1 = (mu1 + i * p_i) / q1;
//       mu2 = (mu - q1 * mu1) / q2;
//       sigma = q1 * q2 * (mu1 - mu2) * (mu1 - mu2);
//       if (sigma > max_sigma) {
//         max_sigma = sigma;
//         max_val = static_cast<FT>(i);
//       }
//     }

//     return static_cast<uchar>(max_val * scale_);
//   }


//! Get name of threshold method
// static std::string name() { return "otsu"; }
// };

}  // namespace lsfm
