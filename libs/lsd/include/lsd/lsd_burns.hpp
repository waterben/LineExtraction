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

//! Use Non Maxima Supression for burns detector
static const int BURNS_NMS = 1;

template <class FT,
          template <class> class LPT = Vec2,
          class PT = LPT<int>,
          class GRAD = DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
          class FIT = EigenFit<FT, PT, typename GRAD::mag_type>>
class LsdBurns : public LsdBase<FT, LPT> {
  using LsdBase<FT, LPT>::endPoints_;
  using LsdBase<FT, LPT>::lineSegments_;

  GRAD grad_{};  // gradient object

  // segmentation components
  cv::Mat partitionMap_{};                    // CV_8U, 0, pixels with partition indicies
  cv::Mat partitionMapShifted_{};             // CV_8U, 0, pixels with shifted partition indicies
  std::vector<IndexVector> seeds_{};          // edge points sorted by partition
  std::vector<IndexVector> seeds_shifted_{};  // edge points sorted by shifted partition

  // connected components
  struct CcData {
    CcData(uchar part = 0, bool sh = false) : partition(part), accepted(false), shifted(sh){};

    IndexVector data{};
    uchar partition{};
    bool accepted{};
    bool shifted{};
  };
  typedef std::vector<CcData> CcDataVector;  // vector of connected components

  CcDataVector ccLists_{}, ccListsShifted_{};

  cv::Mat voteMap_{};

  int pixel_count_{}, rows_{}, cols_{}, min_pix_{}, part_num_{}, flags_{};
  FT th_low_{}, th_high_{};

  mutable typename LsdBase<FT, LPT>::ImageData imageData_{};

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
  typedef FT float_type;
  typedef LPT<FT> line_point;
  typedef PT point_type;
  typedef typename GRAD::mag_type mag_type;
  typedef typename GRAD::grad_type grad_type;
  typedef typename LsdBase<FT, LPT>::Line Line;
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;
  typedef std::vector<PT> PointVector;


  //! Create a BurnsDetector object.
  //!
  //! @param th_high    Higher intensity threshold for magnitude. Range [0..1] (0.004 ~ 1/255).
  //! @param th_low     Lower intensity threshold for magnitude. Range [0..1] (0.004 ~ 1/255).
  //! @param min_pix        Minimum number of supporting pixels for line segment. Range [2..X]
  //! @param part_num       Number of partitions the gradient directions are assigned to. Range [4...255]
  //! @param flags Flags for line direction estimation
  //!                   BURNS_NMS - Use Non Maxima Supression for burns detector
  LsdBurns(FT th_low = 0.004, FT th_high = 0.012, int min_pix = 5, int part_num = 12, int flags = 0)
      : min_pix_(min_pix), part_num_(part_num), flags_(flags), th_low_(th_low), th_high_(th_high) {
    CV_Assert(part_num_ < 256 && part_num_ > 3 && th_low <= 1 && th_low > 0 && th_high <= 1 && th_high > 0 &&
              th_high >= th_low && min_pix > 1);
    init();
  }

  LsdBurns(ValueManager::InitializerList options) {
    init();
    this->value(options);
  }

  LsdBurns(const ValueManager::NameValueVector& options) {
    init();
    this->value(options);
  }

  Value valueThresholdLow(const Value& t = Value::NAV()) {
    if (t.type()) thresholdLow(t.get<FT>());
    return th_low_;
  }

  FT thresholdLow() const { return th_low_; }

  void thresholdLow(FT t) { th_low_ = t; }

  Value valueThresholdHigh(const Value& t = Value::NAV()) {
    if (t.type()) thresholdHigh(t.get<FT>());
    return th_high_;
  }

  FT thresholdHigh() const { return th_high_; }

  void thresholdHigh(FT t) { th_high_ = t; }

  void threshold(FT low, FT high) {
    th_low_ = low;
    th_high_ = high;
  }

  Value valueMinPixel(const Value& mp = Value::NAV()) {
    if (mp.type()) minPixels(mp.getInt());
    return min_pix_;
  }

  int minPixels() const { return min_pix_; }

  void minPixels(int mp) { min_pix_ = mp; }

  Value valuePartitions(const Value& p = Value::NAV()) {
    if (p.type()) partitions(p.getInt());
    return part_num_;
  }

  int partitions() const { return part_num_; }

  void partitions(int p) { part_num_ = p; }

  Value valueFlags(const Value& f = Value::NAV()) {
    if (f.type()) flags(f.getInt());
    return flags_;
  }

  int flags() const { return flags_; }

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

  inline const GRAD& grad() const { return grad_; }
  inline GRAD& grad() { return grad_; }

  //! Get image data description. For every layer in image data, a DataDescriptorEntry is defined, giving the name
  //! and a description for the layer
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

  //! Get image data.
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


    uchar* pmap = 0;
    IndexVector* ccdata = 0;
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
      size_t size = static_cast<size_t>(static_cast<double>(seeds_[static_cast<size_t>(i)].size()) * sscale) + 100;
      if (size > 10000) size = 10000;
      ccLists_.push_back(CcData(static_cast<uchar>(static_cast<int>(i))));
      // use the CcData struct
      ccdata = &ccLists_.back().data;
      ccdata->reserve(size);

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
            ccdata->reserve(size);
          }
        }
      });

      // raise the area count
      if (ccdata->size() < static_cast<size_t>(this->min_pix_)) ccLists_.pop_back();
    }

    pmap = partitionMapShifted_.ptr<uchar>();
    for (int i = 0; i != part_num_; ++i) {
      part = i + 1;
      size_t size =
          static_cast<size_t>(static_cast<double>(seeds_shifted_[static_cast<size_t>(i)].size()) * sscale) + 100;
      if (size > 10000) size = 10000;
      ccListsShifted_.push_back(CcData(static_cast<uchar>(i), true));
      // use the CcData struct
      ccdata = &ccListsShifted_.back().data;
      ccdata->reserve(size);

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
                     ccdata->reserve(size);
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
