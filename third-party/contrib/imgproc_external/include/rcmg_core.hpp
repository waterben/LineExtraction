/***************************************************************************
 * The Robust Colour Morphological Gradient (RCMG) for edge detection      *
 * in multichannel images                                                  *
 *                                                                         *
 * Adrian N. Evans                                                         *
 * University of Bath                                                      *
 * July 2007                                                               *
 * Copyright 2005-2007.  All rights reserved.                              *
 *                                                                         *
 * The RCMG find the median centered difference for a given window size    *
 * by rejecting the s pairs of pixels that are furthest apart. For full    *
 * details of its operation see A.N. Evans and X.U. Liu, A Morphological   *
 * Gradient Approach to Color Edge Detection, IEEE Transactions on Image   *
 * Processing, 15(6), pp. 1454-1463, June 2006.                           *
 *                                                                         *
 * This implementation uses the L2 norm.                                   *
 *                                                                         *
 * Modified 2016-2026 by Benjamin Wassermann:                              *
 *   - Ported from C to C++17 templates                                    *
 *   - Added single-channel specialization                                 *
 *   - Integrated with OpenCV cv::Mat                                      *
 *   - Extracted as standalone core algorithm                               *
 ***************************************************************************/

/// @file rcmg_core.hpp
/// @brief Core RCMG algorithm implementation.
///
/// Standalone Robust Colour Morphological Gradient computation without
/// framework dependencies. Only requires OpenCV for cv::Mat and norm
/// utilities.
/// @see A.N. Evans and X.U. Liu, "A Morphological Gradient Approach to Color
///      Edge Detection", IEEE Trans. Image Processing, 15(6), pp. 1454-1463, 2006.

#pragma once

#include <opencv2/core/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rcmg_impl {

/// @brief Core RCMG algorithm for multi-channel images.
///
/// Computes the morphological gradient with vector rejection for edge
/// detection. This class manages internal state (window buffer, distance
/// matrix) for efficient incremental computation across sliding window
/// positions.
/// @tparam IT Input pixel type (e.g., uchar).
/// @tparam channels Number of image channels (e.g., 3 for BGR).
/// @tparam GT Gradient component output type (e.g., short).
/// @tparam MT Magnitude output type (e.g., int).
template <class IT, int channels, class GT, class MT>
class RCMGCore {
 public:
  /// @brief Compute RCMG gradient on the given image.
  ///
  /// Computes magnitude and directional gradients (gx, gy) for the input
  /// image using the RCMG sliding window algorithm.
  /// @param[in] img Input multi-channel image (channels must match template).
  /// @param[out] mag Output magnitude image.
  /// @param[out] gx Output X-gradient component image.
  /// @param[out] gy Output Y-gradient component image.
  /// @param[in] mask Window size (e.g., 3 for 3x3). Minimum is 3.
  /// @param[in] s Number of vector pairs to reject for noise robustness.
  /// @param[in] norm_type OpenCV norm type (cv::NORM_L1, cv::NORM_L2, cv::NORM_L2SQR).
  inline void compute(
      const cv::Mat& img, cv::Mat_<MT>& mag, cv::Mat_<GT>& gx, cv::Mat_<GT>& gy, int mask, int s, int norm_type) {
    update_mask(mask);
    do_process(img, mag, gx, gy, mask, s, norm_type);
  }

 private:
  std::vector<int> locations_;                 ///< Location indices for max distances.
  std::vector<char> not_removed_;              ///< Rejection status flags.
  std::vector<cv::Vec<IT, channels>> window_;  ///< Current window pixel values.
  MT max_dist_{0};                             ///< Current maximum distance.
  MT max_dist2_{0};                            ///< Secondary maximum distance.
  std::vector<MT> max_dists_;                  ///< Per-pixel max distances.
  std::vector<MT> copyof_max_dists_;           ///< Copy for rejection iteration.
  cv::Mat_<MT> dists_;                         ///< Pairwise distance matrix.

  template <typename T, int m, int n>
  static inline double norm(const cv::Matx<T, m, n>& M, int normType) {
    return normType == cv::NORM_INF
               ? static_cast<double>(cv::normInf<T, typename cv::DataType<T>::work_type>(M.val, m * n))
           : normType == cv::NORM_L1
               ? static_cast<double>(cv::normL1<T, typename cv::DataType<T>::work_type>(M.val, m * n))
           : normType == cv::NORM_L2SQR
               ? static_cast<double>(cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(M.val, m * n))
               : std::sqrt(static_cast<double>(cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(M.val, m * n)));
  }

  template <typename T, int m, int n>
  static inline double norm(const cv::Matx<T, m, n>& A, const cv::Matx<T, m, n>& B, int normType) {
    return normType == cv::NORM_INF
               ? static_cast<double>(cv::normInf<T, typename cv::DataType<T>::work_type>(A.val, B.val, m * n))
           : normType == cv::NORM_L1
               ? static_cast<double>(cv::normL1<T, typename cv::DataType<T>::work_type>(A.val, B.val, m * n))
           : normType == cv::NORM_L2SQR
               ? static_cast<double>(cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(A.val, B.val, m * n))
               : std::sqrt(
                     static_cast<double>(cv::normL2Sqr<T, typename cv::DataType<T>::work_type>(A.val, B.val, m * n)));
  }

  void update_mask(int m) {
    if (m < 3) m = 3;
    const int m2 = m * m;
    const auto window_size = static_cast<std::size_t>(m2);

    max_dists_.resize(window_size);
    locations_.resize(window_size);
    not_removed_.resize(window_size);
    window_.resize(window_size);
    dists_.create(m2, m2);
  }

  void norms(int norm_type) {
    int i, j;
    MT n, max_norm;

    for (i = 0; i < dists_.rows - 1; ++i) {
      max_norm = static_cast<MT>(-1);
      int best = i + 1;
      const size_t idx = static_cast<size_t>(i);
      for (j = i + 1; j < dists_.rows; ++j) {
        const size_t jdx = static_cast<size_t>(j);
        n = static_cast<MT>(norm(window_[idx], window_[jdx], norm_type));
        if (n > max_norm) {
          max_norm = n;
          best = j;
        }
        dists_(i, j) = n;
      }
      max_dists_[idx] = max_norm;
      locations_[idx] = best;
    }
  }

  void update_norms(int mask, int norm_type) {
    int i, j;
    MT n;

    for (i = 0; i < dists_.rows - mask - 1; ++i) {
      const size_t idx = static_cast<size_t>(i);
      const size_t idxMask = static_cast<size_t>(i + mask);
      max_dists_[idx] = max_dists_[idxMask];
      locations_[idx] = locations_[idxMask] - mask;
      for (j = i + 1; j < dists_.rows - mask; ++j) {
        dists_(i, j) = dists_(i + mask, j + mask);
      }

      for (j = i + 1; j < dists_.rows - mask; ++j) {
        dists_(i, j) = n = dists_(i + mask, j + mask);
        if (n > max_dists_[idx]) {
          max_dists_[idx] = n;
          locations_[idx] = j;
        }
      }
    }

    for (i = dists_.rows - mask - 1; i < dists_.rows - 1; ++i) max_dists_[static_cast<size_t>(i)] = static_cast<MT>(-1);

    for (i = 0; i < dists_.rows - 1; ++i) {
      const size_t idx = static_cast<size_t>(i);
      for (j = cv::max(i + 1, mask); j < dists_.rows; ++j) {
        const size_t jdx = static_cast<size_t>(j);
        dists_(i, j) = n = static_cast<MT>(norm(window_[idx], window_[jdx], norm_type));
        if (n > max_dists_[idx]) {
          max_dists_[idx] = n;
          locations_[idx] = j;
        }
      }
    }
  }

  void do_process(
      const cv::Mat& img, cv::Mat_<MT>& mag, cv::Mat_<GT>& gx, cv::Mat_<GT>& gy, int mask, int s, int norm_type) {
    CV_Assert(img.channels() == channels);

    int imask, jmask, k, j, index, loop;
    int hs = (mask - 1) / 2;
    int m2 = mask * mask;
    int v1{0}, v2{0}, location{0}, pos{0};

    mag.create(img.size());
    gx.create(img.size());
    gy.create(img.size());

    MT* pmag = mag.template ptr<MT>();
    GT* pgx = gx.template ptr<GT>();
    GT* pgy = gy.template ptr<GT>();

    dists_.setTo(-1);

    for (int i = hs; i < img.rows - hs - 1; ++i) {
      for (j = hs; j < img.cols - hs; ++j) {
        index = 0;
        for (jmask = j - hs; jmask <= j + hs; ++jmask) {
          for (imask = i - hs; imask <= i + hs; ++imask) {
            window_[static_cast<size_t>(index)] = img.at<cv::Vec<IT, channels>>(imask, jmask);
            index++;
          }
        }

        if (j == hs)
          norms(norm_type);
        else
          update_norms(mask, norm_type);

        max_dist_ = static_cast<MT>(-1);
        for (k = 0; k < m2 - 1; ++k) {
          if (max_dists_[static_cast<size_t>(k)] > max_dist_) {
            max_dist_ = max_dists_[static_cast<size_t>(k)];
            v1 = k;
            v2 = locations_[static_cast<size_t>(k)];
          }
        }

        if (s > 0) {
          std::fill(not_removed_.begin(), not_removed_.end(), 1);
          copyof_max_dists_ = max_dists_;
          for (index = 1; index <= s; ++index) {
            not_removed_[static_cast<size_t>(v1)] = 0;
            not_removed_[static_cast<size_t>(v2)] = 0;
            max_dist_ = static_cast<MT>(-1);
            for (k = 0; k < m2 - 1; ++k) {
              if (not_removed_[static_cast<size_t>(k)]) {
                if (copyof_max_dists_[static_cast<size_t>(k)] > max_dist_) {
                  if (not_removed_[static_cast<size_t>(locations_[static_cast<size_t>(k)])]) {
                    max_dist_ = copyof_max_dists_[static_cast<size_t>(k)];
                    v1 = k;
                    v2 = locations_[static_cast<size_t>(k)];
                  } else {
                    max_dist2_ = static_cast<MT>(-1);
                    for (loop = k + 1; loop < m2; ++loop) {
                      if (dists_(k, loop) > max_dist2_) {
                        if (not_removed_[static_cast<size_t>(loop)]) {
                          max_dist2_ = dists_(k, loop);
                          location = loop;
                        }
                      }
                      copyof_max_dists_[static_cast<size_t>(k)] = max_dist2_;
                    }
                    if (copyof_max_dists_[static_cast<size_t>(k)] > max_dist_) {
                      max_dist_ = copyof_max_dists_[static_cast<size_t>(k)];
                      v1 = k;
                      v2 = location;
                    }
                  }
                }
              }
            }
          }
        }

        pos = i * img.cols + j;
        pmag[pos] = max_dist_;
        pgx[pos] = static_cast<GT>(v2 / mask - v1 / mask);
        pgy[pos] = static_cast<GT>((v2 % mask) - (v1 % mask));
      }
    }
  }
};

/// @brief Single-channel specialization of RCMGCore.
///
/// Uses scalar intensity differences instead of vector color distances.
/// @tparam IT Input pixel type (e.g., uchar).
/// @tparam GT Gradient component output type.
/// @tparam MT Magnitude output type.
template <class IT, class GT, class MT>
class RCMGCore<IT, 1, GT, MT> {
 public:
  /// @brief Compute single-channel RCMG gradient.
  /// @param[in] img Input single-channel image.
  /// @param[out] mag Output magnitude image.
  /// @param[out] gx Output X-gradient component image.
  /// @param[out] gy Output Y-gradient component image.
  /// @param[in] mask Window size (minimum 3).
  /// @param[in] s Number of pairs to reject.
  /// @param[in] norm_type Unused for single-channel (kept for API consistency).
  inline void compute(
      const cv::Mat& img, cv::Mat_<MT>& mag, cv::Mat_<GT>& gx, cv::Mat_<GT>& gy, int mask, int s, int /*norm_type*/) {
    update_mask(mask);
    do_process(img, mag, gx, gy, mask, s);
  }

 private:
  std::vector<int> locations_;
  std::vector<char> not_removed_;
  std::vector<IT> window_;
  MT max_dist_{0};
  MT max_dist2_{0};
  std::vector<MT> max_dists_;
  std::vector<MT> copyof_max_dists_;
  cv::Mat_<MT> dists_;

  void update_mask(int m) {
    if (m < 3) m = 3;
    int m2 = m * m;

    max_dists_.resize(static_cast<size_t>(m2));
    locations_.resize(static_cast<size_t>(m2));
    not_removed_.resize(static_cast<size_t>(m2));
    window_.resize(static_cast<size_t>(m2));
    dists_.create(m2, m2);
  }

  void norms() {
    int i, j;
    MT n, max_norm;

    for (i = 0; i < dists_.rows - 1; ++i) {
      max_norm = -1;
      int best = i + 1;
      const size_t idx = static_cast<size_t>(i);
      for (j = i + 1; j < dists_.rows; ++j) {
        const size_t jdx = static_cast<size_t>(j);
        n = std::abs(static_cast<MT>(window_[idx]) - window_[jdx]);
        if (n > max_norm) {
          max_norm = n;
          best = j;
        }
        dists_(i, j) = n;
      }
      max_dists_[idx] = max_norm;
      locations_[idx] = best;
    }
  }

  void update_norms(int mask) {
    int i, j;
    MT n;

    for (i = 0; i < dists_.rows - mask - 1; ++i) {
      const size_t idx = static_cast<size_t>(i);
      const size_t idxMask = static_cast<size_t>(i + mask);
      max_dists_[idx] = max_dists_[idxMask];
      locations_[idx] = locations_[idxMask] - mask;
      for (j = i + 1; j < dists_.rows - mask; ++j) {
        dists_(i, j) = dists_(i + mask, j + mask);
      }

      for (j = i + 1; j < dists_.rows - mask; ++j) {
        dists_(i, j) = n = dists_(i + mask, j + mask);
        if (n > max_dists_[idx]) {
          max_dists_[idx] = n;
          locations_[idx] = j;
        }
      }
    }

    for (i = dists_.rows - mask - 1; i < dists_.rows - 1; ++i) max_dists_[static_cast<size_t>(i)] = -1;

    for (i = 0; i < dists_.rows - 1; ++i) {
      const size_t idx = static_cast<size_t>(i);
      for (j = cv::max(i + 1, mask); j < dists_.rows; ++j) {
        const size_t jdx = static_cast<size_t>(j);
        dists_(i, j) = n = std::abs(static_cast<MT>(window_[idx]) - window_[jdx]);
        if (n > max_dists_[idx]) {
          max_dists_[idx] = n;
          locations_[idx] = j;
        }
      }
    }
  }

  void do_process(const cv::Mat& img, cv::Mat_<MT>& mag, cv::Mat_<GT>& gx, cv::Mat_<GT>& gy, int mask, int s) {
    CV_Assert(img.channels() == 1);

    int imask, jmask, k, j, index, loop;
    int hs = (mask - 1) / 2;
    int m2 = mask * mask;
    int v1{0}, v2{0}, location{0}, pos{0};

    mag.create(img.size());
    gx.create(img.size());
    gy.create(img.size());

    MT* pmag = mag.template ptr<MT>();
    GT* pgx = gx.template ptr<GT>();
    GT* pgy = gy.template ptr<GT>();

    dists_.setTo(-1);

    for (int i = hs; i < img.rows - hs - 1; ++i) {
      for (j = hs; j < img.cols - hs; ++j) {
        index = 0;
        for (jmask = j - hs; jmask <= j + hs; ++jmask) {
          for (imask = i - hs; imask <= i + hs; ++imask) {
            window_[static_cast<size_t>(index)] = img.at<IT>(imask, jmask);
            index++;
          }
        }

        if (j == hs)
          norms();
        else
          update_norms(mask);

        max_dist_ = -1;
        for (k = 0; k < m2 - 1; ++k) {
          if (max_dists_[static_cast<size_t>(k)] > max_dist_) {
            max_dist_ = max_dists_[static_cast<size_t>(k)];
            v1 = k;
            v2 = locations_[static_cast<size_t>(k)];
          }
        }

        if (s > 0) {
          std::fill(not_removed_.begin(), not_removed_.end(), 1);
          copyof_max_dists_ = max_dists_;
          for (index = 1; index <= s; ++index) {
            not_removed_[static_cast<size_t>(v1)] = 0;
            not_removed_[static_cast<size_t>(v2)] = 0;
            max_dist_ = -1;
            for (k = 0; k < m2 - 1; ++k) {
              if (not_removed_[static_cast<size_t>(k)]) {
                if (copyof_max_dists_[static_cast<size_t>(k)] > max_dist_) {
                  if (not_removed_[static_cast<size_t>(locations_[static_cast<size_t>(k)])]) {
                    max_dist_ = copyof_max_dists_[static_cast<size_t>(k)];
                    v1 = k;
                    v2 = locations_[static_cast<size_t>(k)];
                  } else {
                    max_dist2_ = -1;
                    for (loop = k + 1; loop < m2; ++loop) {
                      if (dists_(k, loop) > max_dist2_) {
                        if (not_removed_[static_cast<size_t>(loop)]) {
                          max_dist2_ = dists_(k, loop);
                          location = loop;
                        }
                      }
                      copyof_max_dists_[static_cast<size_t>(k)] = max_dist2_;
                    }
                    if (copyof_max_dists_[static_cast<size_t>(k)] > max_dist_) {
                      max_dist_ = copyof_max_dists_[static_cast<size_t>(k)];
                      v1 = k;
                      v2 = location;
                    }
                  }
                }
              }
            }
          }
        }

        pos = i * img.cols + j;
        pmag[pos] = max_dist_;
        pgx[pos] = static_cast<GT>(v2 / mask - v1 / mask);
        pgy[pos] = static_cast<GT>((v2 % mask) - (v1 % mask));
      }
    }
  }
};

}  // namespace rcmg_impl
