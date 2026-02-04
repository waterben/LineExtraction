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
//
// C by Benjamin Wassermann
//M*/


#pragma once

#include <edge/fit.hpp>
#include <edge/index.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <lsd/lsd_base.hpp>

#include <limits>


namespace lsfm {


//! Use Non Maxima Supression for fast lsd detector
static const int FBW_NMS = 1;
static const int FBW_PATAN = 2;

static const uchar FBW_NOTUSED = 0;
static const uchar FBW_USED = 1;

template <class FT,
          template <class> class LPT = Vec2,
          class PT = LPT<int>,
          class GRAD = DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude>,
          class FIT = EigenFit<FT, PT, typename GRAD::mag_type>>
class LsdFBW : public LsdBase<FT, LPT> {
  using LsdBase<FT, LPT>::endPoints_;
  using LsdBase<FT, LPT>::lineSegments_;

  GRAD grad_{};

  // the images
  cv::Mat img_{};  // input image

  cv::Mat used_{};  // indicates used pixels

  // segmentation components
  cv::Mat th_dir_{};     // FT,  Threshold filtered gradient direction
  IndexVector seeds_{};  // seed indexes (above higer thres)
  IndexVector areas_{};

  mutable typename LsdBase<FT, LPT>::ImageData imageData_{};

  // image parameters
  int cols_{};         // image width
  int rows_{};         // image height
  int pixel_count_{};  // image pixel count

  int flags_{};
  FT th_low_{};    // gives the lower threshold for the gradient
  FT th_high_{};   // gives the higher threshold for the gradient
  FT angle_th_{};  // Gradient angle tolerance in degrees.
  int min_pix_{};  // minimum amount of supporting pixels
  FT quant_{};

  FT prec_{};
  FT nang_{};

  void init() {
    this->addManager(grad_);

    this->add("nms_th_low", std::bind(&LsdFBW<FT, LPT, PT, GRAD, FIT>::valueThresholdLow, this, std::placeholders::_1),
              "Lower threshold for nms.");
    this->add("nms_th_high",
              std::bind(&LsdFBW<FT, LPT, PT, GRAD, FIT>::valueThresholdHigh, this, std::placeholders::_1),
              "Upper threshold for nms.");
    this->add("edge_min_pixels", std::bind(&LsdFBW<FT, LPT, PT, GRAD, FIT>::valueMinPixel, this, std::placeholders::_1),
              "Minimal number of support pixels for line segment.");
    this->add("angle_th", std::bind(&LsdFBW<FT, LPT, PT, GRAD, FIT>::valueAngleThreshold, this, std::placeholders::_1),
              "Gradient angle tolerance.");
    this->add("line_flags", std::bind(&LsdFBW<FT, LPT, PT, GRAD, FIT>::valueFlags, this, std::placeholders::_1),
              "Flags for line detector: 0 - none, 1 - enable nms, 2 - use precise atan2.");
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

  static const FT FBW_NOTDEF;


  struct LineData {
    //! @brief Construct line data from point positions.
    //! @param b Starting position in the point vector.
    //! @param e Ending position (past-the-end) in the point vector.
    LineData(size_t b = 0, size_t e = 0) : p_beg(b), p_end(e), wbeg(0), wend(0), prec(0), prob(0) {}

    // start / end position in point list (supporting points)
    size_t p_beg, p_end;

    FT wbeg, wend;  // width begin and end
    FT prec, prob;  // precision and probability

    //! @brief Get the width of the line (difference between end and begin width).
    //! @return The absolute width value |wend - wbeg|.
    inline FT width() const { return std::abs(wend - wbeg); }

    //! @brief Get the number of supporting points in this line data.
    //! @return The size of the point range (p_end - p_beg).
    inline size_t size() const { return p_end - p_beg; }

    //! @brief Get the starting position in the global point vector.
    //! @return The index of the first point in this line data.
    inline size_t begpos() const { return p_beg; }

    //! @brief Get the ending position in the global point vector.
    //! @return The index past the last point in this line data.
    inline size_t endpos() const { return p_end; }
  };
  typedef std::vector<LineData> LineDataVector;  // vector of LineData structs

 private:
  LineDataVector lineData_{};

 public:
  //! Create a FoodLineSegmentDetector object.
  //! @param th_low         Lower intensity threshold for magnitude. Range [0..1] (0.004 ~ 1/255).
  //! @param th_high        Higher intensity threshold for magnitude. Range [0..1] (0.004 ~ 1/255).
  //! @param min_pix        Minimum number of supporting pixels for line segment. Range (>= 2)
  //! @param ang_th         Gradient angle tolerance in degrees. (0..90]
  //! @param flags Flags for line direction estimation
  //!                   FBW_NMS - Use Non Maxima Supression for flood detector
  //!                   FBW_PATAN - Use precise atan computation
  LsdFBW(FT th_low = static_cast<FT>(0.004),
         FT th_high = static_cast<FT>(0.012),
         int min_pix = 0,
         FT ang_th = static_cast<FT>(22.5),
         int flags = 0)
      : flags_(flags), th_low_(th_low), th_high_(th_high), angle_th_(ang_th), min_pix_(min_pix) {
    CV_Assert(ang_th <= 90 && ang_th > 0 && th_low <= 1 && th_low > 0 && th_high <= 1 && th_high > 0 &&
              th_high >= th_low && min_pix >= 0);
    init();

    prec_ = static_cast<FT>(CV_PI * ang_th / 180);
    nang_ = ang_th / 180;

    // th_high_ = th_low_ = static_cast<FT>(quant / std::sin(prec_));
  }

  //! @brief Get or set the lower threshold for gradient magnitude.
  //! @param t The new threshold value, or Value::NAV() to only query.
  //! @return The current lower threshold value.
  Value valueThresholdLow(const Value& t = Value::NAV()) {
    if (t.type()) thresholdLow(t.get<FT>());
    return th_low_;
  }

  //! @brief Get the lower threshold for gradient magnitude.
  //! @return The current lower threshold value. Range [0..1] normalized (0.004 ~ 1/255).
  FT thresholdLow() const { return th_low_; }

  //! @brief Set the lower threshold for gradient magnitude.
  //! @param t The new lower threshold value. Range [0..1] normalized (0.004 ~ 1/255).
  void thresholdLow(FT t) { th_low_ = t; }

  //! @brief Get or set the upper threshold for gradient magnitude.
  //! @param t The new threshold value, or Value::NAV() to only query.
  //! @return The current upper threshold value.
  Value valueThresholdHigh(const Value& t = Value::NAV()) {
    if (t.type()) thresholdHigh(t.get<FT>());
    return th_high_;
  }

  //! @brief Get the upper threshold for gradient magnitude.
  //! @return The current upper threshold value. Range [0..1] normalized (0.004 ~ 1/255).
  FT thresholdHigh() const { return th_high_; }

  //! @brief Set the upper threshold for gradient magnitude.
  //! @param t The new upper threshold value. Range [0..1] normalized (0.004 ~ 1/255).
  void thresholdHigh(FT t) { th_high_ = t; }

  //! @brief Set both lower and upper thresholds for gradient magnitude.
  //! @param low The new lower threshold value.
  //! @param high The new upper threshold value.
  void threshold(FT low, FT high) {
    th_low_ = low;
    th_high_ = high;
  }

  //! @brief Get or set the minimum number of supporting pixels for line segment.
  //! @param mp The new minimum pixels value, or Value::NAV() to only query.
  //! @return The current minimum pixels value.
  Value valueMinPixel(const Value& mp = Value::NAV()) {
    if (mp.type()) minPixels(mp.getInt());
    return min_pix_;
  }

  //! @brief Get the minimum number of supporting pixels for line segment.
  //! @return The current minimum pixels value. Typical range [0..X].
  int minPixels() const { return min_pix_; }

  //! @brief Set the minimum number of supporting pixels for line segment.
  //! @param mp The new minimum pixels value. Must be >= 0. Typical range [0..X].
  void minPixels(int mp) { min_pix_ = mp; }

  //! @brief Get or set the gradient angle tolerance in degrees.
  //! @param t The new angle threshold value, or Value::NAV() to only query.
  //! @return The current angle threshold value.
  Value valueAngleThreshold(const Value& t = Value::NAV()) {
    if (t.type()) angleThreshold(t.get<FT>());
    return angle_th_;
  }

  //! @brief Get the gradient angle tolerance in degrees.
  //! @return The current angle threshold value. Range (0..90] degrees.
  FT angleThreshold() const { return angle_th_; }

  //! @brief Set the gradient angle tolerance in degrees.
  //! @param t The new angle threshold value in degrees. Range (0..90].
  void angleThreshold(FT t) { angle_th_ = t; }

  //! @brief Get or set the detection flags controlling algorithm behavior.
  //! @param f The new flags value, or Value::NAV() to only query.
  //! @return The current flags value.
  //! Supported flags:
  //! - FBW_NMS: Use Non-Maxima Suppression for flood detector
  //! - FBW_PATAN: Use precise atan2 computation for angle estimation
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

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;
  using LsdBase<FT, LPT>::imageDataDescriptor;
  using LsdBase<FT, LPT>::imageData;

  virtual void detect(const cv::Mat& src) final {
    // the image
    img_ = src;
    CV_Assert(!img_.empty());

    // clear data
    this->clearData();
    imageData_.clear();

    // init vars
    cols_ = img_.cols;
    rows_ = img_.rows;
    pixel_count_ = cols_ * rows_;

    if (min_pix_ == 0)
      min_pix_ = static_cast<int>(
          -(5 * (log10(static_cast<FT>(cols_)) + log10(static_cast<FT>(rows_))) / 2 + log10(static_cast<FT>(11))) /
          log10(nang_));  // minimal number of points in region that can give a meaningful even
    else
      min_pix_ = min_pix_;

    // the algorithm
    preprocess();
    flags_& FBW_NMS ? seedsNMS() : seeds();
    findRegions();
  }

  virtual const DataDescriptor& imageDataDescriptor() const final {
    static DataDescriptor dsc;
    if (dsc.empty()) {
      dsc.push_back(DataDescriptorEntry("gx", "Gradient in x direction"));
      dsc.push_back(DataDescriptorEntry("gy", "Gradient in y direction"));
      dsc.push_back(DataDescriptorEntry("mag", "Gradient magnitude"));
      dsc.push_back(DataDescriptorEntry("th_dir", "Threshold filtered gradient direction"));
    }
    return dsc;
  }

  //! Get image data.
  virtual const ImageData& imageData() const final {
    if (imageData_.empty()) {
      imageData_.push_back(grad_.gx());
      imageData_.push_back(grad_.gy());
      imageData_.push_back(grad_.magnitude());
      imageData_.push_back(th_dir_);
    }
    return imageData_;
  }

 private:
  inline FT atan2(mag_type a, mag_type b) const {
    return flags_ & FBW_PATAN ? static_cast<FT>(cv::fastAtan2(static_cast<float>(a), static_cast<float>(b)) *
                                                static_cast<float>(CV_PI / 180))
                              : static_cast<FT>(std::atan2(static_cast<FT>(a), static_cast<FT>(b)));
  }

  inline bool isAligned(size_t idx, FT theta, const FT& prec) const {
    FT a = th_dir_.ptr<FT>()[idx];
    if (a == FBW_NOTDEF) return false;

    // It is assumed that 'theta' and 'a' are in the range [-pi,pi] or [0, 2*pi]
    theta = std::abs(theta - a);

    if (theta > static_cast<FT>(CV_PI)) theta = static_cast<FT>(2 * CV_PI) - theta;

    return theta <= prec;
  }

  void preprocess() {
    grad_.process(img_);

    th_dir_.create(rows_, cols_, cv::DataType<FT>::type);
    th_dir_.setTo(FBW_NOTDEF);

    used_.create(rows_, cols_, CV_8U);
    used_.setTo(FBW_NOTUSED);

    seeds_.clear();
    seeds_.reserve(static_cast<size_t>(pixel_count_ / 3));
  }


  // Compute threshold filtered magnitude and direction + seeds, using NMS
  void seedsNMS() {
    const int TG22 = static_cast<int>(0.4142135623730950488016887242097 * (1 << 15) + 0.5);

    const mag_type* pmag = grad_.magnitude().template ptr<mag_type>();
    FT* pth_dir = th_dir_.ptr<FT>();

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
          // compute the angle of the gradient
          pth_dir[idx] = atan2(xs, -ys);

          if (m > high) seeds_.push_back(idx);
        }
      }
    }
  }

  // Compute threshold filtered magnitude and direction + seeds
  void seeds() {
    const mag_type* pmag = grad_.magnitude().template ptr<mag_type>();
    FT* pth_dir = th_dir_.ptr<FT>();

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
          // compute the angle of the gradient
          pth_dir[idx] = atan2(pgx[idx], -pgy[idx]);

          if (mag > high) seeds_.push_back(idx);
        }
      }
    }
  }

  void findRegions() {
    size_t size = static_cast<size_t>(static_cast<FT>(seeds_.size()) * th_high_ / th_low_);
    areas_.clear();
    areas_.reserve(size);
    lineData_.clear();
    lineData_.reserve(size / static_cast<size_t>(min_pix_ * min_pix_) + 100);
    lineSegments_.reserve(lineData_.size());

    FT rangle;
    uchar* pused = used_.ptr<uchar>();
    for_each(seeds_.begin(), seeds_.end(), [&](index_type idx) {
      if (pused[idx] == FBW_USED) return;
      size_t start = areas_.size();
      regionGrow(idx, prec_, rangle);
      // ignore region
      if (areas_.size() - start < static_cast<size_t>(min_pix_)) return;


      LineData ld(start, areas_.size());
      ld.prec = prec_;
      ld.prob = nang_;
      LineSegment l = region2Data(ld);
      float epnx = static_cast<float>(sin(rangle));
      float epny = static_cast<float>(-cos(rangle));
      if (epnx * l.normalX() + epny * l.normalY() >= 0) {
        l.normalFlip();
      }

      lineData_.push_back(ld);
      lineSegments_.push_back(l);
    });
  }

  LineSegment region2Data(LineData& ld) {
    Line l;
    std::vector<PT> tmp;
    tmp.resize(ld.size());
    IndexConvert<PT>::toPoint(areas_.begin() + static_cast<std::ptrdiff_t>(ld.begpos()),
                              areas_.begin() + static_cast<std::ptrdiff_t>(ld.endpos()), tmp.begin(), cols_);


    FIT::fit(&tmp.front(), &tmp.back() + 1, l, grad_.magnitude());

    FT lmin = std::numeric_limits<FT>::max(), lmax = std::numeric_limits<FT>::lowest(),
       wmin = std::numeric_limits<FT>::max(), wmax = std::numeric_limits<FT>::lowest();
    for_each(tmp.begin(), tmp.end(), [&](const PT& p) {
      FT w = l.normalProject(static_cast<FT>(getX(p)), static_cast<FT>(getY(p))),
         len = l.project(static_cast<FT>(getX(p)), static_cast<FT>(getY(p)));
      lmin = std::min(lmin, len);
      lmax = std::max(lmax, len);

      wmin = std::min(wmin, w);
      wmax = std::max(wmax, w);
    });
    ld.wbeg = wmin;
    ld.wend = wmax;
    return LineSegment(l, lmin, lmax);
  }

  void regionGrow(index_type s, const FT& prec, FT& reg_angle) {
    uchar* pused = used_.ptr<uchar>();

    const grad_type* pdx = grad_.gx().template ptr<grad_type>();
    const grad_type* pdy = grad_.gy().template ptr<grad_type>();

    size_t i = areas_.size();
    // Point to this region
    areas_.push_back(s);
    pused[s] = FBW_USED;
    reg_angle = th_dir_.ptr<FT>()[s];

    int sumdx = -pdy[s];
    int sumdy = pdx[s];

    // Try neighboring regions
    for (; i < areas_.size(); ++i) {
      index_type idx = areas_[i];
      index_type adr = idx - 1;
      // test left pixel
      if (pused[adr] != FBW_USED && isAligned(adr, reg_angle, prec)) {
        areas_.push_back(adr);
        pused[adr] = FBW_USED;
        sumdx -= pdy[adr];
        sumdy += pdx[adr];
        reg_angle = atan2(sumdy, sumdx);
      }

      adr = idx + 1;
      // test right pixel
      if (pused[adr] != FBW_USED && isAligned(adr, reg_angle, prec)) {
        areas_.push_back(adr);
        pused[adr] = FBW_USED;
        sumdx -= pdy[adr];
        sumdy += pdx[adr];
        reg_angle = atan2(sumdy, sumdx);
      }

      index_type uadr = idx - static_cast<index_type>(cols_) - 1;
      adr = uadr + 3;
      index_type ladr = idx + static_cast<index_type>(cols_) - 1;

      // test the remaining 6 pixels above and under the current pixel
      for (; uadr != adr; ++uadr, ++ladr) {
        if (pused[uadr] != FBW_USED && isAligned(uadr, reg_angle, prec)) {
          areas_.push_back(uadr);
          pused[uadr] = FBW_USED;
          sumdx -= pdy[uadr];
          sumdy += pdx[uadr];
          reg_angle = atan2(sumdy, sumdx);
        }

        if (pused[ladr] != FBW_USED && isAligned(ladr, reg_angle, prec)) {
          areas_.push_back(ladr);
          pused[ladr] = FBW_USED;
          sumdx -= pdy[ladr];
          sumdy += pdx[ladr];
          reg_angle = atan2(sumdy, sumdx);
        }
      }
    }
  }
};

template <class FT, template <class> class LPT, class PT, class GRAD, class FIT>
const FT LsdFBW<FT, LPT, PT, GRAD, FIT>::FBW_NOTDEF = static_cast<FT>(-5);

}  // namespace lsfm
