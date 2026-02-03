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

//////////////////////////////////////////////////////////////////////////////////////
// Implementation of J. Burns: "Extracting Straight Lines"
// Implemented by Fabian Gorschl?er and Benjamin Wassermann
//////////////////////////////////////////////////////////////////////////////////////


#pragma once

#include <edge/fit.hpp>
#include <edge/index.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <lsd/lsd_base.hpp>

namespace lsfm {

/// @brief Flag to enable Non-Maxima Suppression in Burns detector.
static const int BURNS_NMS = 1;

/// @brief Line Segment Detector using the Burns algorithm.
/// Implements the classic "Extracting Straight Lines" algorithm by James Burns.
/// This detector uses gradient-based edge segmentation and connected component analysis.
///
/// **Key Features:**
/// - Gradient magnitude thresholding (hysteresis)
/// - Directional partitioning of edges
/// - Connected component labeling
/// - Line fitting using eigenvalue decomposition
/// - Optional Non-Maxima Suppression
///
/// @tparam FT Floating-point type for line coordinates (float, double)
/// @tparam LPT Line point template (default Vec2)
/// @tparam PT Point type (default Vec2i)
/// @tparam GRAD Gradient computation class
/// @tparam FIT Line fitting class
///
/// @example
/// @code{cpp}
/// #include <lsd/lsd_burns.hpp>
/// #include <opencv2/imgcodecs.hpp>
///
/// cv::Mat image = cv::imread("input.png", cv::IMREAD_GRAYSCALE);
///
/// // Create detector with NMS enabled
/// lsfm::LsdBurns<float> detector(0.004f, 0.012f, 5, 12, lsfm::BURNS_NMS);
///
/// // Detect line segments
/// detector.detect(image);
/// const auto& segments = detector.lineSegments();
/// @endcode
template <class FT,
          template <class> class LPT = Vec2,
          class PT = LPT<int>,
          class GRAD = DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
          class FIT = EigenFit<FT, PT, typename GRAD::mag_type>>
class LsdBurns : public LsdBase<FT, LPT> {
  using LsdBase<FT, LPT>::endPoints_;
  using LsdBase<FT, LPT>::lineSegments_;

  GRAD grad_{};  ///< Gradient computation object

  // Segmentation components
  cv::Mat partitionMap_{};                    ///< CV_8U, partition indices for pixels
  cv::Mat partitionMapShifted_{};             ///< CV_8U, shifted partition indices
  std::vector<IndexVector> seeds_{};          ///< Edge points sorted by partition
  std::vector<IndexVector> seeds_shifted_{};  ///< Edge points sorted by shifted partition

  /// @brief Connected component data structure.
  struct CcData {
    /// @brief Create connected component data.
    /// @param part Partition index
    /// @param sh Whether this is a shifted partition
    CcData(uchar part = 0, bool sh = false) : partition(part), accepted(false), shifted(sh){};

    IndexVector data{};  ///< Point indices in this component
    uchar partition{};   ///< Partition index
    bool accepted{};     ///< Whether component was accepted as valid
    bool shifted{};      ///< Whether this is a shifted partition
  };
  typedef std::vector<CcData> CcDataVector;  ///< Vector of connected components

  CcDataVector ccLists_{}, ccListsShifted_{};  ///< Connected component lists

  cv::Mat voteMap_{};  ///< Voting map for point extraction

  int pixel_count_{}, rows_{}, cols_{}, min_pix_{}, part_num_{}, flags_{};  ///< Configuration parameters
  FT th_low_{}, th_high_{};                                                 ///< Threshold values

  mutable typename LsdBase<FT, LPT>::ImageData imageData_{};  ///< Auxiliary image data

  /// @brief Initialize the detector and register all parameters.
  void init() {
    this->addManager(grad_);

    this->add("nms_th_low",
              std::bind(&LsdBurns<FT, LPT, PT, GRAD, FIT>::valueThresholdLow, this, std::placeholders::_1),
              "Normalized lower threshold for nms.");
    this->add("nms_th_high",
              std::bind(&LsdBurns<FT, LPT, PT, GRAD, FIT>::valueThresholdHigh, this, std::placeholders::_1),
              "Normalized upper threshold for nms.");
    this->add("edge_min_pixels",
              std::bind(&LsdBurns<FT, LPT, PT, GRAD, FIT>::valueMinPixel, this, std::placeholders::_1),
              "Minimal number of support pixels for line segment.");
    this->add("edge_partitions",
              std::bind(&LsdBurns<FT, LPT, PT, GRAD, FIT>::valuePartitions, this, std::placeholders::_1),
              "Number of partitions for gradient directions.");
    this->add("line_flags", std::bind(&LsdBurns<FT, LPT, PT, GRAD, FIT>::valueFlags, this, std::placeholders::_1),
              "Flags for line detector: 0 - none, 1 - enable nms.");
  }

  virtual void clearData() final {
    LsdBase<FT, LPT>::clearData();
    imageData_.clear();
  }

 public:
  typedef FT float_type;                                                   ///< Floating-point type
  typedef LPT<FT> line_point;                                              ///< Line point type
  typedef PT point_type;                                                   ///< Point type
  typedef typename GRAD::mag_type mag_type;                                ///< Gradient magnitude type
  typedef typename GRAD::grad_type grad_type;                              ///< Gradient vector type
  typedef typename LsdBase<FT, LPT>::Line Line;                            ///< Line type
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;                ///< Vector of lines
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;              ///< Line segment type
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;  ///< Vector of segments
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;                  ///< Image data type
  typedef std::vector<PT> PointVector;                                     ///< Vector of points

  /// @brief Create a Burns line detector with specified parameters.
  /// The Burns algorithm uses gradient thresholding and directional partitioning
  /// to segment edges, then fits lines to connected components.
  /// @param th_low Lower threshold for gradient magnitude (normalized, range 0-1, e.g., 0.004~1/255)
  /// @param th_high Higher threshold for gradient magnitude (normalized, range 0-1, e.g., 0.012~3/255)
  /// @param min_pix Minimum number of pixels to form a valid line segment (typically 5-20)
  /// @param part_num Number of directional partitions (4-255, typically 12-16)
  /// @param flags Detection flags: 0 = standard, BURNS_NMS = enable Non-Maxima Suppression
  /// @example
  /// @code{cpp}
  /// // Standard Burns detector
  /// lsfm::LsdBurns<float> detector(0.004f, 0.012f, 5, 12, 0);
  ///
  /// // With Non-Maxima Suppression for cleaner results
  /// lsfm::LsdBurns<float> detector_nms(0.004f, 0.012f, 5, 12, lsfm::BURNS_NMS);
  /// @endcode
  LsdBurns(FT th_low = 0.004, FT th_high = 0.012, int min_pix = 5, int part_num = 12, int flags = 0)
      : min_pix_(min_pix), part_num_(part_num), flags_(flags), th_low_(th_low), th_high_(th_high) {
    CV_Assert(part_num_ < 256 && part_num_ > 3 && th_low <= 1 && th_low > 0 && th_high <= 1 && th_high > 0 &&
              th_high >= th_low && min_pix > 1);
    init();
  }

  /// @brief Create detector from an initializer list of parameter names and values.
  /// @param options Initializer list with parameter name/value pairs
  LsdBurns(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  /// @brief Create detector from a vector of parameter name/value pairs.
  /// @param options Vector with parameter name/value pairs
  LsdBurns(const ValueManager::NameValueVector& options) {
    init();
    this->value(options);
  }

  /// @brief Get/set lower threshold via ValueManager interface.
  /// @param t New lower threshold value (optional)
  /// @return Current or updated lower threshold
  Value valueThresholdLow(const Value& t = Value::NAV()) {
    if (t.type()) thresholdLow(t.get<FT>());
    return th_low_;
  }

  /// @brief Get current lower threshold for gradient magnitude.
  /// @return Lower threshold (normalized, 0-1)
  FT thresholdLow() const { return th_low_; }

  /// @brief Set lower threshold for gradient magnitude.
  /// @param t New lower threshold value (normalized, 0-1)
  void thresholdLow(FT t) { th_low_ = t; }

  /// @brief Get/set upper threshold via ValueManager interface.
  /// @param t New upper threshold value (optional)
  /// @return Current or updated upper threshold
  Value valueThresholdHigh(const Value& t = Value::NAV()) {
    if (t.type()) thresholdHigh(t.get<FT>());
    return th_high_;
  }

  /// @brief Get current upper threshold for gradient magnitude.
  /// @return Upper threshold (normalized, 0-1)
  FT thresholdHigh() const { return th_high_; }

  /// @brief Set upper threshold for gradient magnitude.
  /// @param t New upper threshold value (normalized, 0-1)
  void thresholdHigh(FT t) { th_high_ = t; }

  /// @brief Set both lower and upper thresholds at once.
  /// @param low New lower threshold (normalized, 0-1)
  /// @param high New upper threshold (normalized, 0-1)
  void threshold(FT low, FT high) {
    th_low_ = low;
    th_high_ = high;
  }

  /// @brief Get/set minimum pixels via ValueManager interface.
  /// @param mp New minimum pixels value (optional)
  /// @return Current or updated minimum pixels
  Value valueMinPixel(const Value& mp = Value::NAV()) {
    if (mp.type()) minPixels(mp.getInt());
    return min_pix_;
  }

  /// @brief Get current minimum number of supporting pixels for line segments.
  /// @return Minimum pixel count
  int minPixels() const { return min_pix_; }

  /// @brief Set minimum number of supporting pixels for line segments.
  /// @param mp Minimum pixel count (typically 5-20)
  void minPixels(int mp) { min_pix_ = mp; }

  /// @brief Get/set partitions via ValueManager interface.
  /// @param p New partition count (optional)
  /// @return Current or updated partition count
  Value valuePartitions(const Value& p = Value::NAV()) {
    if (p.type()) partitions(p.getInt());
    return part_num_;
  }

  /// @brief Get current number of directional partitions.
  /// @return Number of partitions (4-255)
  int partitions() const { return part_num_; }

  /// @brief Set number of directional partitions for gradient discretization.
  /// Higher values provide finer directional resolution.
  /// @param p Number of partitions (4-255, typically 12-16)
  void partitions(int p) { part_num_ = p; }

  /// @brief Get/set flags via ValueManager interface.
  /// @param f New flags value (optional)
  /// @return Current or updated flags
  Value valueFlags(const Value& f = Value::NAV()) {
    if (f.type()) flags(f.getInt());
    return flags_;
  }

  /// @brief Get current detection flags.
  /// @return Flags bitmask (BURNS_NMS, etc.)
  int flags() const { return flags_; }

  /// @brief Set detection flags.
  /// @param f Flags bitmask (0 = none, BURNS_NMS = enable Non-Maxima Suppression)
  void flags(int f) { flags_ = f; }

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;

  //! Detect lines in the input image.
  virtual void detect(const cv::Mat& src) final {
    // the image
    CV_Assert(!src.empty());

    // clear data
    clearData();

    // init vars
    rows_ = src.rows;
    cols_ = src.cols;
    pixel_count_ = cols_ * rows_;

    grad_.process(src);

    // partition maps
    partitionMap_.create(rows_, cols_, CV_8U);
    partitionMap_.setTo(0);
    partitionMapShifted_.create(rows_, cols_, CV_8U);
    partitionMapShifted_.setTo(0);

    // reserve for the point vectors
    seeds_.resize(static_cast<size_t>(part_num_));
    seeds_shifted_.resize(static_cast<size_t>(part_num_));
    int size = static_cast<int>(pixel_count_ / part_num_ / 100 * (1.0 - sqrt(th_high_))) + 100;
    // cap capacy to 10000, even for large images
    if (size > 10000) size = 10000;
    for (int seg = 0; seg < part_num_; ++seg) {
      seeds_[static_cast<size_t>(seg)].clear();
      seeds_[static_cast<size_t>(seg)].reserve(static_cast<size_t>(size));
      seeds_shifted_[static_cast<size_t>(seg)].clear();
      seeds_shifted_[static_cast<size_t>(seg)].reserve(static_cast<size_t>(size));
    }

    // the algorithm
    flags_& BURNS_NMS ? segmentationNMS() : segmentation();
    ccComputation();
    areaVoting();
    computeLines();
  }

  /// @brief Get const reference to the gradient computation object.
  /// @return Const reference to gradient object
  inline const GRAD& grad() const { return grad_; }

  /// @brief Get mutable reference to the gradient computation object.
  /// Allows advanced configuration of gradient computation parameters.
  /// @return Mutable reference to gradient object
  inline GRAD& grad() { return grad_; }

  /// @brief Get descriptor information for auxiliary image data.
  /// @return Data descriptor with name and description for each layer
  virtual const DataDescriptor& imageDataDescriptor() const final {
    static DataDescriptor dsc;
    if (dsc.empty()) {
      dsc.push_back(DataDescriptorEntry("gx", "Gradient in x direction"));
      dsc.push_back(DataDescriptorEntry("gy", "Gradient in y direction"));
      dsc.push_back(DataDescriptorEntry("mag", "Gradient magnitude"));
      dsc.push_back(DataDescriptorEntry("pmap", "Parition map"));
      dsc.push_back(DataDescriptorEntry("spmap", "Shifted parition map"));
    }
    return dsc;
  }

  /// @brief Get auxiliary image data computed during detection.
  /// @return Vector of image data layers
  virtual const ImageData& imageData() const final {
    if (imageData_.empty()) {
      imageData_.push_back(grad_.gx());
      imageData_.push_back(grad_.gy());
      imageData_.push_back(grad_.magnitude());
      imageData_.push_back(partitionMap_);
      imageData_.push_back(partitionMapShifted_);
    }
    return imageData_;
  }

 private:
  void segmentationNMS() {
    const int TG22 = static_cast<int>(0.4142135623730950488016887242097 * (1 << 15) + 0.5);


    // pointer to the magnitude
    const mag_type* pmag = grad_.magnitude().template ptr<mag_type>();

    // pointer to the partition maps
    uchar* ppmap = partitionMap_.template ptr<uchar>();
    uchar* ppmaps = partitionMapShifted_.template ptr<uchar>();


    mag_type low = grad_.magnitudeThreshold(th_low_), high = grad_.magnitudeThreshold(th_high_);
    if (low > high) std::swap(high, low);

    // pointer to the gradient values
    const grad_type* pgx = grad_.gx().template ptr<grad_type>();
    const grad_type* pgy = grad_.gy().template ptr<grad_type>();

    // iterate over whole image, line by line and magintude one line ahead
    size_t idx = static_cast<size_t>(cols_ - 1), end = static_cast<size_t>(cols_ * (rows_ - 1) - 1), r_end;
    while (idx < end) {
      idx += 2;
      r_end = idx + static_cast<size_t>(cols_ - 2);

      for (; idx < r_end; ++idx) {
        mag_type m = pmag[idx];

        if (m > low) {
          // do NMS
          mag_type xs = pgx[idx];
          mag_type ys = pgy[idx];
          mag_type x = std::abs(xs);
          mag_type y = std::abs(ys) * 32768;

          int tg22x = x * TG22;

          if (y < tg22x) {
            if (m > pmag[idx - 1] && m >= pmag[idx + 1]) goto maxima;
          } else {
            mag_type tg67x = tg22x + x * 65536;
            if (y > tg67x) {
              if (m > pmag[idx - static_cast<size_t>(cols_)] && m >= pmag[idx + static_cast<size_t>(cols_)])
                goto maxima;
            } else {
              int s = neg_sign(xs, ys) ? -1 : 1;
              if (m > pmag[idx - static_cast<size_t>(cols_) - static_cast<size_t>(s)] &&
                  m > pmag[idx + static_cast<size_t>(cols_) + static_cast<size_t>(s)])
                goto maxima;
            }
          }
          continue;
          // only get here if we have a maxima, else continue will jump to next iteration
        maxima:
          // compute the partition of the pixel
          float part =
              cv::fastAtan2(static_cast<float>(xs), static_cast<float>(-ys)) / 360.0f * static_cast<float>(part_num_);

          // get partition number and shifted partition number
          int part_num = static_cast<int>(part);
          int part_num_shift = static_cast<int>(round(part));
          if (part_num_shift >= part_num_) part_num_shift = 0;


          // set pixel in the segment map and shifted segment map
          // increment by 1 for map (0 is reserved for no data)
          ppmap[idx] = static_cast<uchar>(part_num + 1);
          ppmaps[idx] = static_cast<uchar>(part_num_shift + 1);

          if (m > high) {
            // add index to according partition lists as seeds
            seeds_[static_cast<size_t>(part_num)].push_back(idx);
            seeds_shifted_[static_cast<size_t>(part_num_shift)].push_back(idx);
          }
        }
      }
    }
  }

  void segmentation() {
    // pointer to the magnitude
    const mag_type* pmag = grad_.magnitude().template ptr<mag_type>();

    // pointer to the partition maps
    uchar* ppmap = partitionMap_.template ptr<uchar>();
    uchar* ppmaps = partitionMapShifted_.template ptr<uchar>();


    mag_type low = grad_.magnitudeThreshold(th_low_), high = grad_.magnitudeThreshold(th_high_);
    if (low > high) std::swap(high, low);

    // pointer to the gradient values
    const grad_type* pgx = grad_.gx().template ptr<grad_type>();
    const grad_type* pgy = grad_.gy().template ptr<grad_type>();

    // iterate over whole image, line by line and magintude one line ahead
    size_t idx = static_cast<size_t>(cols_ - 1), end = static_cast<size_t>(cols_ * (rows_ - 1) - 1), r_end;
    while (idx < end) {
      idx += 2;
      r_end = idx + static_cast<size_t>(cols_ - 2);

      for (; idx < r_end; ++idx) {
        // the magnitude in the pixel
        mag_type mag = pmag[idx];

        // threshold by 16 corresponds to 4 colorsteps change
        if (mag > low) {
          // the partition of the pixel
          float part = cv::fastAtan2(static_cast<float>(pgx[idx]), static_cast<float>(-pgy[idx])) / 360.0f *
                       static_cast<float>(part_num_);

          // get partition number and shifted partition number
          int part_num = static_cast<int>(part);
          int part_num_shift = static_cast<int>(round(part));
          if (part_num_shift >= part_num_) part_num_shift = 0;

          // set pixel in the segment map and shifted segment map
          // increment by 1 for map (1-part_num, 0 is reserved for no data)
          ppmap[idx] = static_cast<uchar>(part_num + 1);
          ppmaps[idx] = static_cast<uchar>(part_num_shift + 1);

          if (mag > high) {
            // add index to according partition list as seeds
            seeds_[static_cast<size_t>(part_num)].push_back(idx);
            seeds_shifted_[static_cast<size_t>(part_num_shift)].push_back(idx);
          }
        }
      }
    }
  }

  void ccComputation() {
    // cc vectors
    ccLists_.clear();
    ccListsShifted_.clear();

    int size = pixel_count_ / (min_pix_ * min_pix_) + 100;
    if (size > 10000) size = 10000;
    ccLists_.reserve(static_cast<size_t>(size));
    ccListsShifted_.reserve(static_cast<size_t>(size));


    uchar* pmap = nullptr;
    IndexVector* ccdata = nullptr;
    int part = 0;

    // get the connected components area for partition
    auto getArea = [&](index_type idx) {
      pmap[idx] = 0;
      ccdata->push_back(idx);

      for (size_t i = 0; i != ccdata->size(); ++i) {
        idx = (*ccdata)[i];

        // test left pixel
        if (pmap[idx - 1] == part) {
          ccdata->push_back(idx - 1);
          pmap[idx - 1] = 0;
        }
        // test right pixel
        if (pmap[idx + 1] == part) {
          ccdata->push_back(idx + 1);
          pmap[idx + 1] = 0;
        }

        int upper = static_cast<int>(idx) - this->cols_ - 1;
        int lower = static_cast<int>(idx) + this->cols_ - 1;

        // test the remaining 6 pixels above and under the current pixel
        for (int m = 0; m < 3; ++m) {
          // test the three pixels in y-1
          if (pmap[upper + m] == part) {
            ccdata->push_back(static_cast<size_t>(upper + m));
            pmap[upper + m] = 0;
          }
          // test the three pixels in y+1
          if (pmap[lower + m] == part) {
            ccdata->push_back(static_cast<size_t>(lower + m));
            pmap[lower + m] = 0;
          }
        }
      }
    };

    double sscale = th_high_ / th_low_ / 100;
    pmap = partitionMap_.ptr<uchar>();
    for (int i = 0; i != part_num_; ++i) {
      part = i + 1;
      size_t cc_size = static_cast<size_t>(static_cast<double>(seeds_[static_cast<size_t>(i)].size()) * sscale) + 100;
      if (cc_size > 10000) cc_size = 10000;
      ccLists_.push_back(CcData(static_cast<uchar>(static_cast<int>(i))));
      // use the CcData struct
      ccdata = &ccLists_.back().data;
      ccdata->reserve(cc_size);

      // iterate the seed edges of the partition
      for_each(seeds_[static_cast<size_t>(i)].begin(), seeds_[static_cast<size_t>(i)].end(), [&](index_type idx) {
        if (pmap[idx] == part) {
          // get all elements of the area and add them to the vectors

          getArea(idx);
          if (ccdata->size() < static_cast<size_t>(this->min_pix_))
            ccdata->clear();
          else {
            ccLists_.push_back(CcData(static_cast<uchar>(i)));
            ccdata = &ccLists_.back().data;
            ccdata->reserve(cc_size);
          }
        }
      });

      // raise the area count
      if (ccdata->size() < static_cast<size_t>(this->min_pix_)) ccLists_.pop_back();
    }

    pmap = partitionMapShifted_.ptr<uchar>();
    for (int i = 0; i != part_num_; ++i) {
      part = i + 1;
      size_t cc_size_shifted =
          static_cast<size_t>(static_cast<double>(seeds_shifted_[static_cast<size_t>(i)].size()) * sscale) + 100;
      if (cc_size_shifted > 10000) cc_size_shifted = 10000;
      ccListsShifted_.push_back(CcData(static_cast<uchar>(i), true));
      // use the CcData struct
      ccdata = &ccListsShifted_.back().data;
      ccdata->reserve(cc_size_shifted);

      // iterate the seed edges of the partition
      for_each(seeds_shifted_[static_cast<size_t>(i)].begin(), seeds_shifted_[static_cast<size_t>(i)].end(),
               [&](index_type idx) {
                 if (pmap[idx] == part) {
                   // get all elements of the area and add them to the vectors
                   getArea(idx);
                   // raise the area count
                   if (ccdata->size() < static_cast<size_t>(this->min_pix_))
                     ccdata->clear();
                   else {
                     ccListsShifted_.push_back(CcData(static_cast<uchar>(i), true));
                     ccdata = &ccListsShifted_.back().data;
                     ccdata->reserve(cc_size_shifted);
                   }
                 }
               });
      // raise the area count
      if (ccdata->size() < static_cast<size_t>(this->min_pix_)) ccListsShifted_.pop_back();
    }
  }

  void areaVoting() {
    // init
    voteMap_.create(rows_, cols_, CV_32S);
    voteMap_.setTo(0);
    int* pvoteMap = voteMap_.ptr<int>();

    // iterate the normal areas and set size of the areas
    for_each(ccLists_.begin(), ccLists_.end(), [&](const CcData& ccV) {
      for_each(ccV.data.begin(), ccV.data.end(),
               [&](index_type idx) { pvoteMap[idx] = static_cast<int>(ccV.data.size()); });
    });


    // iterate the shifted areas and set size to sizemap, also set accepted flag
    for_each(ccListsShifted_.begin(), ccListsShifted_.end(), [&](CcData& ccV) {
      // the size of the shifted area
      int size = static_cast<int>(ccV.data.size());
      int voteCount = 0;

      for_each(ccV.data.begin(), ccV.data.end(), [&](index_type idx) {
        int& at = pvoteMap[idx];
        // count pixels that vote for the area
        if (size > at) {
          ++voteCount;
          at = size;
        }
      });

      // is the area accepted, set bool in voteArea
      if (voteCount >= size / 2) ccV.accepted = true;
    });

    // iterate the normal areas and set size of the areas
    for_each(ccLists_.begin(), ccLists_.end(), [&](CcData& ccV) {
      // the size of the area
      int size = static_cast<int>(ccV.data.size());
      int voteCount = 0;

      for_each(ccV.data.begin(), ccV.data.end(), [&](index_type idx) {
        // count pixels that vote for the area
        if (size >= pvoteMap[idx]) ++voteCount;
      });

      // is the area accepted, set bool in voteArea
      if (voteCount >= size / 2) ccV.accepted = true;
    });
  }

  void computeLines() {
    lineSegments_.reserve(ccLists_.size());

    // apply the linear regression either on the shifted elements or the normal
    auto lineRegressionHelper = [&](CcData& ccd) {
      Line l;
      std::vector<PT> tmp;
      IndexConvert<PT>::toPointV(ccd.data, tmp, this->cols_);

      FIT::fit(&tmp.front(), &tmp.back() + 1, l, this->grad_.magnitude());

      // first and last point of the vector
      const PT* first = &tmp[0];
      const PT* last = &tmp[0];
      FT bx = -l.normalY(), by = l.normalX();
      FT dNegMax = static_cast<FT>(getX(*first)) * bx + static_cast<FT>(getY(*first)) * by;
      FT dPosMax = dNegMax;

      // compute the distance to the centroid for each point
      for_each(tmp.begin(), tmp.end(), [&](const PT& p) {
        FT dist = static_cast<FT>(getX(p)) * bx + static_cast<FT>(getY(p)) * by;

        if (dist < dNegMax) {
          dNegMax = dist;
          first = &p;
        }

        if (dist > dPosMax) {
          dPosMax = dist;
          last = &p;
        }
      });


      // correct direction of line
      FT a = static_cast<FT>(ccd.partition * 2 * CV_PI / this->part_num_ + (ccd.shifted ? 0 : CV_PI / this->part_num_));
      FT epnx = sin(a);
      FT epny = -cos(a);
      if (epnx * l.normalX() + epny * l.normalY() < 0) std::swap(first, last);

      this->lineSegments_.push_back(LineSegment(l, *first, *last));
    };


    // iterate the normal edges and apply line regression on accepted areas
    for_each(ccLists_.begin(), ccLists_.end(), [&](CcData& ccV) {
      if (ccV.accepted) lineRegressionHelper(ccV);
    });

    // iterate the shifted edges and apply line regression on accepted areas
    for_each(ccListsShifted_.begin(), ccListsShifted_.end(), [&](CcData& sccV) {
      if (sccV.accepted) lineRegressionHelper(sccV);
    });
  }
};
}  // namespace lsfm
