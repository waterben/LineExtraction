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
 * Processing, 15(6), pp. 1454-1463, June 2006.                            *
 *																		   *
 * This implementation uses the L2 norm                                    *
 *																		   *
 ***************************************************************************/

/**
 * @file rcmg.hpp
 * @brief Robust Colour Morphological Gradient edge detector.
 *
 * Implementation of the RCMG algorithm for edge detection in multichannel
 * (color) images. The algorithm finds edges by computing the morphological
 * gradient with vector rejection for noise robustness.
 *
 * @see A.N. Evans and X.U. Liu, "A Morphological Gradient Approach to Color
 *      Edge Detection", IEEE Trans. Image Processing, 15(6), pp. 1454-1463, 2006.
 */

#pragma once

#include <imgproc/gradient.hpp>
#include <opencv2/core/core.hpp>

#include <cstddef>
#include <vector>

namespace lsfm {

/// @brief Robust Colour Morphological Gradient (RCMG) edge detector.
///
/// Computes edges in multi-channel images using the morphological gradient
/// approach. The algorithm finds the median-centered difference within a
/// sliding window, rejecting s pairs of pixels that are furthest apart in
/// color space to achieve noise robustness.
/// The gradient direction is derived from the spatial positions of the
/// maximum-distance pixel pair within the window.
/// @tparam IT Input image type (e.g., uchar).
/// @tparam channels Number of color channels (default: 3 for RGB/BGR).
/// @tparam GT Gradient component type (default: short).
/// @tparam MT Magnitude type (default: int for storing squared norms).
/// @tparam DT Direction type (default: float).
/// @tparam DO Direction operator type (default: Direction<GT, DT>).
template <class IT = uchar,
          int channels = 3,
          class GT = short,
          class MT = int,
          class DT = float,
          class DO = Direction<GT, DT>>
class RCMGradient : public Gradient<IT, GT, MT, DT> {
  int mask_;  ///< Window size (e.g., 3 for 3x3).
  int s_;     ///< Number of vector pairs to reject.
  int m2_;    ///< Squared mask size.
  int norm_;  ///< Norm type (cv::NORM_L1, cv::NORM_L2, cv::NORM_L2SQR).

  cv::Mat_<MT> mag_;  ///< Computed magnitude image.
  cv::Mat_<GT> gx_;   ///< Computed X-gradient image.
  cv::Mat_<GT> gy_;   ///< Computed Y-gradient image.

  mutable cv::Mat_<DT> dir_;  ///< Computed direction image (lazy).
  mutable bool dir_done_;     ///< Direction computation flag.


  // vectors
  std::vector<int> locations;                   ///< Location indices for max distances.
  std::vector<char> not_removed;                ///< Rejection status flags.
  std::vector<cv::Vec<IT, channels>> window;    ///< Current window pixel values.
  MT max_dist, max_dist2;                       ///< Maximum distance values.
  std::vector<MT> max_dists, copyof_max_dists;  ///< Per-pixel max distances.
  cv::Mat_<MT> dists;                           ///< Pairwise distance matrix.

 public:
  typedef IT img_type;   ///< Input image pixel type.
  typedef GT grad_type;  ///< Gradient component type.
  typedef MT mag_type;   ///< Magnitude type.
  typedef DT dir_type;   ///< Direction type.

  typedef Range<GT> GradientRange;   ///< Range type for gradient values.
  typedef Range<MT> MagnitudeRange;  ///< Range type for magnitude values.
  typedef Range<DT> DirectionRange;  ///< Range type for direction values.

  /// @brief Construct an RCMG gradient operator.
  /// @param mask Window size for computing differences (default: 3, meaning 3x3).
  /// @param s Number of vector pairs to reject for noise reduction (default: 0).
  /// @param cn Norm type: cv::NORM_L1, cv::NORM_L2, cv::NORM_L2SQR (default: L2SQR).
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  RCMGradient(int mask = 3,
              int s = 0,
              int cn = cv::NORM_L2SQR,
              IT int_lower = std::numeric_limits<IT>::lowest(),
              IT int_upper = std::numeric_limits<IT>::max())
      : Gradient<IT, GT, MT, DT>(int_lower, int_upper),
        mask_(mask),
        s_(s),
        m2_(0),
        norm_(cn),
        mag_(),
        gx_(),
        gy_(),
        dir_(),
        dir_done_(false),
        locations(),
        not_removed(),
        window(),
        max_dist(0),
        max_dist2(0),
        max_dists(),
        copyof_max_dists(),
        dists() {
    this->add("grad_mask_size",
              std::bind(&RCMGradient<IT, channels, GT, MT, DT, DO>::maskSize, this, std::placeholders::_1),
              "Window size for differences.");
    this->add("grad_rejection",
              std::bind(&RCMGradient<IT, channels, GT, MT, DT, DO>::rejection, this, std::placeholders::_1),
              "Vector rejection for noise compensation.");
    this->add("grad_color_norm",
              std::bind(&RCMGradient<IT, channels, GT, MT, DT, DO>::colorNorm, this, std::placeholders::_1),
              "Norm type: L1 or L2.");
    updateMask(mask);
  }

  Value maskSize(const Value& ms = Value::NAV()) {
    if (ms.type()) {
      updateMask(ms);
    }
    return mask_;
  }
  Value rejection(const Value& r = Value::NAV()) {
    if (r.type()) s_ = r;
    return s_;
  }
  Value colorNorm(const Value& nt = Value::NAV()) {
    if (nt.type()) norm_ = nt;
    return norm_;
  }


  /// @brief Process an image to compute RCMG edge gradients.
  ///
  /// Computes gradient magnitude and components using the morphological
  /// gradient algorithm. Results can be retrieved using magnitude(), gx(), gy().
  /// @param img Input multi-channel image.
  void process(const cv::Mat& img) {
    dir_done_ = false;
    do_process(img);
  }

  /// @brief Process image and retrieve gradient components and magnitude.
  /// @param img Input multi-channel image.
  /// @param gx Output X-gradient image.
  /// @param gy Output Y-gradient image.
  /// @param mag Output magnitude image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
  }

  /// @brief Process image and retrieve all gradient outputs.
  /// @param img Input multi-channel image.
  /// @param gx Output X-gradient image.
  /// @param gy Output Y-gradient image.
  /// @param mag Output magnitude image.
  /// @param dir Output direction image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag, cv::Mat& dir) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
    dir = direction();
  }

  /// @brief Get both gradient components.
  /// @param gx X-gradient image reference.
  /// @param gy Y-gradient image reference.
  void directionals(cv::Mat& gx, cv::Mat& gy) const {
    gx = gx_;
    gy = gy_;
  }

  /// @brief Get X-gradient image.
  /// @return X-gradient component image.
  cv::Mat gx() const { return gx_; }

  /// @brief Get Y-gradient image.
  /// @return Y-gradient component image.
  cv::Mat gy() const { return gy_; }

  /// @brief Get gradient value range.
  /// @return Range of possible gradient component values.
  GradientRange gradientRange() const { return GradientRange(static_cast<GT>(-mask_), static_cast<GT>(mask_)); }

  /// @brief Get gradient magnitude image.
  /// @return Magnitude image.
  cv::Mat magnitude() const { return mag_; }

  /// @brief Check if direction has been computed.
  /// @return True if direction() has been called, false otherwise.
  inline bool isDirectionDone() const { return dir_done_; }

  /// @brief Get gradient direction image.
  ///
  /// Direction is computed lazily on first access.
  /// @return Direction image in range determined by DO::range().
  cv::Mat direction() const {
    if (!dir_done_) {
      DO::process(gx_, gy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /// @brief Get direction value range.
  /// @return Range of direction values.
  DirectionRange directionRange() const { return DO::range(); }

  /// @brief Get magnitude value range.
  /// @return Range of possible magnitude values.
  MagnitudeRange magnitudeRange() const {
    return MagnitudeRange(0, static_cast<MT>(norm(cv::Vec<IT, channels>::all(this->intRange_.upper), norm_)));
  }


  /// @brief Get the name of this gradient operator.
  /// @return String "rcmg".
  inline std::string name() const { return "rcmg"; }

 private:
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

  void updateMask(int m) {
    if (m < 3) m = 3;

    mask_ = m;
    const int m2 = m * m;
    const auto window_size = static_cast<std::size_t>(m2);

    max_dists.resize(window_size);
    locations.resize(window_size);
    not_removed.resize(window_size);
    window.resize(window_size);
    dists.create(m2, m2);
  }

  void norms() {
    int i, j;
    MT n, max_norm;

    for (i = 0; i < dists.rows - 1; ++i) {
      max_norm = static_cast<MT>(-1);
      int best = i + 1;
      const size_t idx = static_cast<size_t>(i);
      for (j = i + 1; j < dists.rows; ++j) {
        const size_t jdx = static_cast<size_t>(j);
        n = static_cast<MT>(norm(window[idx], window[jdx], norm_));
        if (n > max_norm) {
          max_norm = n;
          best = j;
        }
        dists(i, j) = n;
      }
      max_dists[idx] = max_norm;
      locations[idx] = best;
    }
  }

  void updateNorms() {
    int i, j;
    MT n;

    for (i = 0; i < dists.rows - mask_ - 1; ++i) {  // copy existing values into new positions
      const size_t idx = static_cast<size_t>(i);
      const size_t idxMask = static_cast<size_t>(i + mask_);
      max_dists[idx] = max_dists[idxMask];            // copy max dists and location and
      locations[idx] = locations[idxMask] - mask_;    // adjust by mask_ for move
      for (j = i + 1; j < dists.rows - mask_; ++j) {  // copy dists
        dists(i, j) = dists(i + mask_, j + mask_);
      }

      for (j = i + 1; j < dists.rows - mask_; ++j) {  // copy dists
        dists(i, j) = n = dists(i + mask_, j + mask_);
        if (n > max_dists[idx]) {
          max_dists[idx] = n;
          locations[idx] = j;
        }
      }
    }

    for (i = dists.rows - mask_ - 1; i < dists.rows - 1; ++i)  // clear remaining max_dists
      max_dists[static_cast<size_t>(i)] = static_cast<MT>(-1);

    for (i = 0; i < dists.rows - 1; ++i) {  // add new values and update max_dists
      const size_t idx = static_cast<size_t>(i);
      for (j = cv::max(i + 1, mask_); j < dists.rows; ++j) {
        const size_t jdx = static_cast<size_t>(j);
        dists(i, j) = n = static_cast<MT>(norm(window[idx], window[jdx], norm_));
        if (n > max_dists[idx]) {
          max_dists[idx] = n;
          locations[idx] = j;
        }
      }
    }
  }


  void do_process(const cv::Mat& img) {
    CV_Assert(img.channels() == channels);
    this->dir_done_ = false;

    // loops
    int imask, jmask, k, j, index, loop;

    int hs = (mask_ - 1) / 2;
    int m2 = mask_ * mask_;
    //
    int v1{0}, v2{0}, location{0}, pos{0};


    // magnitude, gx, gy
    // verwendung von mag gx gy aus gradient
    mag_.create(img.size());
    gx_.create(img.size());
    gy_.create(img.size());

    MT* pmag = this->mag_.template ptr<MT>();
    GT* pgx = this->gx_.template ptr<GT>();
    GT* pgy = this->gy_.template ptr<GT>();

    dists.setTo(-1);

    // iterate the image
    for (int i = hs; i < img.rows - hs - 1; ++i) {
      for (j = hs; j < img.cols - hs; ++j) {
        // grab the values for the window
        index = 0;
        // iterate the window
        for (jmask = j - hs; jmask <= j + hs; ++jmask) {  // fill col-1 first
          for (imask = i - hs; imask <= i + hs; ++imask) {
            window[static_cast<size_t>(index)] = img.at<cv::Vec<IT, channels>>(imask, jmask);
            // window[index] /= std::numeric_limits<IT>::max();
            index++;
          }
        }

        // compute the norms
        if (j == hs)  // for first col
          norms();
        else
          updateNorms();


        max_dist = static_cast<MT>(-1);
        for (k = 0; k < m2 - 1; ++k) {  // *m2-1 as last row only -1s
          if (max_dists[static_cast<size_t>(k)] > max_dist) {
            max_dist = max_dists[static_cast<size_t>(k)];
            v1 = k;  // Note: v1 and v2 also needed for direction
            v2 = locations[static_cast<size_t>(k)];
          }
        }

        if (s_ > 0) {  // true if any pairs of vector going to be rejected
          std::fill(not_removed.begin(), not_removed.end(), 1);
          copyof_max_dists = max_dists;
          for (index = 1; index <= s_; ++index) {
            not_removed[static_cast<size_t>(v1)] = 0;
            not_removed[static_cast<size_t>(v2)] = 0;
            max_dist = static_cast<MT>(-1);
            for (k = 0; k < m2 - 1; ++k) {
              if (not_removed[static_cast<size_t>(k)]) {
                if (copyof_max_dists[static_cast<size_t>(k)] > max_dist) {
                  if (not_removed[static_cast<size_t>(
                          locations[static_cast<size_t>(k)])]) {  // vector locations[k] not removed
                    max_dist = copyof_max_dists[static_cast<size_t>(k)];
                    v1 = k;
                    v2 = locations[static_cast<size_t>(k)];
                  } else {  // update max_dists
                    max_dist2 = static_cast<MT>(-1);
                    for (loop = k + 1; loop < m2; ++loop) {
                      if (dists(k, loop) > max_dist2) {
                        if (not_removed[static_cast<size_t>(loop)]) {
                          max_dist2 = dists(k, loop);
                          location = loop;
                        }
                      }
                      copyof_max_dists[static_cast<size_t>(k)] = max_dist2;
                    }
                    // repeat check for > max_dist2 with updated max_dists[k]
                    if (copyof_max_dists[static_cast<size_t>(k)] > max_dist) {
                      max_dist = copyof_max_dists[static_cast<size_t>(k)];
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
        pmag[pos] = max_dist;
        pgx[pos] = static_cast<GT>(v2 / mask_ - v1 / mask_);
        pgy[pos] = static_cast<GT>((v2 % mask_) - (v1 % mask_));
      }
    }
  }
};

/// @brief Single-channel specialization of RCMG gradient operator.
///
/// Specialized implementation of the RCMG algorithm for grayscale images.
/// Uses scalar intensity differences instead of vector color differences.
/// @tparam IT Input image type (e.g., uchar).
/// @tparam GT Gradient component type (default: short).
/// @tparam MT Magnitude type (default: int).
/// @tparam DT Direction type (default: float).
/// @tparam DO Direction operator type.
template <class IT, class GT, class MT, class DT, class DO>
class RCMGradient<IT, 1, GT, MT, DT, DO> : public Gradient<IT, GT, MT, DT> {
  int mask_;  ///< Window size.
  int s_;     ///< Number of pairs to reject.
  int m2_;    ///< Squared mask size.
  int norm_;  ///< Norm type.

  cv::Mat_<MT> mag_;  ///< Computed magnitude image.
  cv::Mat_<GT> gx_;   ///< Computed X-gradient image.
  cv::Mat_<GT> gy_;   ///< Computed Y-gradient image.

  mutable cv::Mat_<DT> dir_;  ///< Computed direction image (lazy).
  mutable bool dir_done_;     ///< Direction computation flag.


  // vectors
  std::vector<int> locations;                   ///< Location indices for max distances.
  std::vector<char> not_removed;                ///< Rejection status flags.
  std::vector<IT> window;                       ///< Current window pixel values.
  MT max_dist, max_dist2;                       ///< Maximum distance values.
  std::vector<MT> max_dists, copyof_max_dists;  ///< Per-pixel max distances.
  cv::Mat_<MT> dists;                           ///< Pairwise distance matrix.

 public:
  typedef IT img_type;   ///< Input image pixel type.
  typedef GT grad_type;  ///< Gradient component type.
  typedef MT mag_type;   ///< Magnitude type.
  typedef DT dir_type;   ///< Direction type.

  typedef Range<GT> GradientRange;   ///< Range type for gradient values.
  typedef Range<MT> MagnitudeRange;  ///< Range type for magnitude values.
  typedef Range<DT> DirectionRange;  ///< Range type for direction values.

  /// @brief Construct a single-channel RCMG gradient operator.
  /// @param mask Window size for computing differences (default: 3).
  /// @param s Number of pairs to reject for noise reduction (default: 0).
  /// @param nt Norm type (default: cv::NORM_L2SQR).
  /// @param int_lower Lower bound of input intensity range.
  /// @param int_upper Upper bound of input intensity range.
  RCMGradient(int mask = 3,
              int s = 0,
              int nt = cv::NORM_L2SQR,
              IT int_lower = std::numeric_limits<IT>::lowest(),
              IT int_upper = std::numeric_limits<IT>::max())
      : Gradient<IT, GT, MT, DT>(int_lower, int_upper),
        mask_(mask),
        s_(s),
        m2_(0),
        norm_(nt),
        mag_(),
        gx_(),
        gy_(),
        dir_(),
        dir_done_(false),
        locations(),
        not_removed(),
        window(),
        max_dist(0),
        max_dist2(0),
        max_dists(),
        copyof_max_dists(),
        dists() {
    this->add("grad_mask_size", std::bind(&RCMGradient<IT, 1, GT, MT, DT, DO>::maskSize, this, std::placeholders::_1),
              "Window size for differences.");
    this->add("grad_rejection", std::bind(&RCMGradient<IT, 1, GT, MT, DT, DO>::rejection, this, std::placeholders::_1),
              "Vector rejection for noise compensation.");
    this->add("grad_norm_type",
              std::bind(&RCMGradient<IT, 1, GT, MT, DT, DO>::normTypeOption, this, std::placeholders::_1),
              "Norm type: L1 or L2.");
    updateMask(mask);
  }

  Value maskSize(const Value& ms = Value::NAV()) {
    if (ms.type()) {
      updateMask(ms);
    }
    return mask_;
  }
  Value rejection(const Value& r = Value::NAV()) {
    if (r.type()) s_ = r;
    return s_;
  }
  Value normTypeOption(const Value& nt = Value::NAV()) {
    if (nt.type()) norm_ = nt;
    return norm_;
  }


  /// @brief Process an image to compute RCMG edge gradients.
  /// @param img Input single-channel image.
  void process(const cv::Mat& img) {
    dir_done_ = false;
    do_process(img);
  }

  /// @brief Process image and retrieve gradient components and magnitude.
  /// @param img Input single-channel image.
  /// @param gx Output X-gradient image.
  /// @param gy Output Y-gradient image.
  /// @param mag Output magnitude image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
  }

  /// @brief Process image and retrieve all gradient outputs.
  /// @param img Input single-channel image.
  /// @param gx Output X-gradient image.
  /// @param gy Output Y-gradient image.
  /// @param mag Output magnitude image.
  /// @param dir Output direction image.
  inline void process(const cv::Mat& img, cv::Mat& gx, cv::Mat& gy, cv::Mat& mag, cv::Mat& dir) {
    process(img);
    directionals(gx, gy);
    mag = magnitude();
    dir = direction();
  }

  /// @brief Get both gradient components.
  /// @param gx X-gradient image reference.
  /// @param gy Y-gradient image reference.
  void directionals(cv::Mat& gx, cv::Mat& gy) const {
    gx = gx_;
    gy = gy_;
  }

  /// @brief Get X-gradient image.
  /// @return X-gradient component image.
  cv::Mat gx() const { return gx_; }

  /// @brief Get Y-gradient image.
  /// @return Y-gradient component image.
  cv::Mat gy() const { return gy_; }

  /// @brief Get gradient value range.
  /// @return Range of possible gradient component values.
  GradientRange gradientRange() const { return GradientRange(static_cast<GT>(-mask_), static_cast<GT>(mask_)); }

  /// @brief Get gradient magnitude image.
  /// @return Magnitude image.
  cv::Mat magnitude() const { return mag_; }

  /// @brief Check if direction has been computed.
  /// @return True if direction() has been called, false otherwise.
  inline bool isDirectionDone() const { return dir_done_; }

  /// @brief Get gradient direction image.
  /// @return Direction image in range determined by DO::range().
  cv::Mat direction() const {
    if (!dir_done_) {
      DO::process(gx_, gy_, dir_);
      dir_done_ = true;
    }
    return dir_;
  }

  /// @brief Get direction value range.
  /// @return Range of direction values.
  DirectionRange directionRange() const { return DO::range(); }

  /// @brief Get magnitude value range.
  /// @return Range of possible magnitude values.
  MagnitudeRange magnitudeRange() const { return MagnitudeRange(0, this->intRange_.upper); }


  /// @brief Get the name of this gradient operator.
  /// @return String "rcmg".
  inline std::string name() const { return "rcmg"; }

 private:
  void updateMask(int m) {
    if (m < 3) m = 3;

    mask_ = m;
    int m2 = m * m;

    max_dists.resize(static_cast<size_t>(m2));
    locations.resize(static_cast<size_t>(m2));
    not_removed.resize(static_cast<size_t>(m2));
    window.resize(static_cast<size_t>(m2));
    dists.create(m2, m2);
  }

  void norms() {
    int i, j;
    MT norm, max_norm;

    for (i = 0; i < dists.rows - 1; ++i) {
      max_norm = -1;
      int best = i + 1;
      const size_t idx = static_cast<size_t>(i);
      for (j = i + 1; j < dists.rows; ++j) {
        const size_t jdx = static_cast<size_t>(j);
        norm = std::abs(static_cast<MT>(window[idx]) - window[jdx]);
        if (norm > max_norm) {
          max_norm = norm;
          best = j;
        }
        dists(i, j) = norm;
      }
      max_dists[idx] = max_norm;
      locations[idx] = best;
    }
  }

  void updateNorms() {
    int i, j;
    MT n;

    for (i = 0; i < dists.rows - mask_ - 1; ++i) {  // copy existing values into new positions
      const size_t idx = static_cast<size_t>(i);
      const size_t idxMask = static_cast<size_t>(i + mask_);
      max_dists[idx] = max_dists[idxMask];            // copy max dists and location and
      locations[idx] = locations[idxMask] - mask_;    // adjust by mask_ for move
      for (j = i + 1; j < dists.rows - mask_; ++j) {  // copy dists
        dists(i, j) = dists(i + mask_, j + mask_);
      }

      for (j = i + 1; j < dists.rows - mask_; ++j) {  // copy dists
        dists(i, j) = n = dists(i + mask_, j + mask_);
        if (n > max_dists[idx]) {
          max_dists[idx] = n;
          locations[idx] = j;
        }
      }
    }

    for (i = dists.rows - mask_ - 1; i < dists.rows - 1; ++i)  // clear remaining max_dists
      max_dists[static_cast<size_t>(i)] = -1;

    for (i = 0; i < dists.rows - 1; ++i) {  // add new values and update max_dists
      const size_t idx = static_cast<size_t>(i);
      for (j = cv::max(i + 1, mask_); j < dists.rows; ++j) {
        const size_t jdx = static_cast<size_t>(j);
        dists(i, j) = n = std::abs(static_cast<MT>(window[idx]) - window[jdx]);
        if (n > max_dists[idx]) {
          max_dists[idx] = n;
          locations[idx] = j;
        }
      }
    }
  }


  void do_process(const cv::Mat& img) {
    CV_Assert(img.channels() == 1);
    this->dir_done_ = false;

    // loops
    int imask, jmask, k, j, index, loop;

    int hs = (mask_ - 1) / 2;
    int m2 = mask_ * mask_;
    //
    int v1{0}, v2{0}, location{0}, pos{0};


    // magnitude, gx, gy
    // verwendung von mag gx gy aus gradient
    mag_.create(img.size());
    gx_.create(img.size());
    gy_.create(img.size());

    MT* pmag = this->mag_.template ptr<MT>();
    GT* pgx = this->gx_.template ptr<GT>();
    GT* pgy = this->gy_.template ptr<GT>();

    dists.setTo(-1);

    // iterate the image
    for (int i = hs; i < img.rows - hs - 1; ++i) {
      for (j = hs; j < img.cols - hs; ++j) {
        // grab the values for the window
        index = 0;
        // iterate the window
        for (jmask = j - hs; jmask <= j + hs; ++jmask) {  // fill col-1 first
          for (imask = i - hs; imask <= i + hs; ++imask) {
            window[static_cast<size_t>(index)] = img.at<IT>(imask, jmask);
            index++;
          }
        }

        // compute the norms
        if (j == hs)  // for first col
          norms();
        else
          updateNorms();


        max_dist = -1;
        for (k = 0; k < m2 - 1; ++k) {  // m2-1 as last row only -1s
          if (max_dists[static_cast<size_t>(k)] > max_dist) {
            max_dist = max_dists[static_cast<size_t>(k)];
            v1 = k;  // Note: v1 and v2 also needed for direction
            v2 = locations[static_cast<size_t>(k)];
          }
        }

        if (s_ > 0) {  // true if any pairs of vector going to be rejected
          std::fill(not_removed.begin(), not_removed.end(), 1);
          copyof_max_dists = max_dists;
          for (index = 1; index <= s_; ++index) {
            not_removed[static_cast<size_t>(v1)] = 0;
            not_removed[static_cast<size_t>(v2)] = 0;
            max_dist = -1;
            for (k = 0; k < m2 - 1; ++k) {
              if (not_removed[static_cast<size_t>(k)]) {
                if (copyof_max_dists[static_cast<size_t>(k)] > max_dist) {
                  if (not_removed[static_cast<size_t>(
                          locations[static_cast<size_t>(k)])]) {  // vector locations[k] not removed
                    max_dist = copyof_max_dists[static_cast<size_t>(k)];
                    v1 = k;
                    v2 = locations[static_cast<size_t>(k)];
                  } else {  // update max_dists
                    max_dist2 = -1;
                    for (loop = k + 1; loop < m2; ++loop) {
                      if (dists(k, loop) > max_dist2) {
                        if (not_removed[static_cast<size_t>(loop)]) {
                          max_dist2 = dists(k, loop);
                          location = loop;
                        }
                      }
                      copyof_max_dists[static_cast<size_t>(k)] = max_dist2;
                    }
                    // repeat check for > max_dist2 with updated max_dists[k]
                    if (copyof_max_dists[static_cast<size_t>(k)] > max_dist) {
                      max_dist = copyof_max_dists[static_cast<size_t>(k)];
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
        pmag[pos] = max_dist;
        pgx[pos] = static_cast<GT>(v2 / mask_ - v1 / mask_);
        pgy[pos] = static_cast<GT>((v2 % mask_) - (v1 % mask_));
      }
    }
  }
};
}  // namespace lsfm
