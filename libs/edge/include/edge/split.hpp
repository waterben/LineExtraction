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

/*
 *  Fast connected components line detection algorithm
 *  Version 1.3
 *  (C) by Benjamin Wassermann
 */

#pragma once

#include <edge/edge_segment.hpp>
#include <edge/edge_source.hpp>
#include <edge/fit.hpp>

namespace lsfm {
//! @brief Perform a Ramer or Douglas Peucker split to a given edge segment.
//!        The merge options allows to apply a postprocessing merge step to
//!        merge ill formed splits (splitd wrongly made by poorly picked base
//!        line)
template <class FT, class PT = Vec2i, bool MERGE = true>
class RamerSplit : public ValueManager {
  FT dist_;
  int min_len_;

  void init() {
    this->add("split_error_distance", std::bind(&RamerSplit<FT, PT, MERGE>::valueDistance, this, std::placeholders::_1),
              "Error distance.");
    this->add("split_min_length", std::bind(&RamerSplit<FT, PT, MERGE>::valueMinLength, this, std::placeholders::_1),
              "Minimal length (number of pixels).");
  }

 public:
  typedef FT float_type;
  typedef PT point_type;
  typedef std::vector<PT> PointVector;

  RamerSplit(FT dist = 2, int minp = 2) : dist_(dist), min_len_(minp) { init(); }

  RamerSplit(const ValueManager::NameValueVector& options) : dist_(2), min_len_(2) {
    init();
    this->value(options);
  }

  RamerSplit(ValueManager::InitializerList options) : dist_(2), min_len_(2) {
    init();
    this->value(options);
  }

  Value valueDistance(const Value& d = Value::NAV()) {
    if (d.type()) distance(d.get<FT>());
    return dist_;
  }

  FT distance() const { return dist_; }

  void distance(FT d) { dist_ = d; }

  Value valueMinLength(const Value& p = Value::NAV()) {
    if (p.type()) minLength(p.getInt());
    return min_len_;
  }

  int minLength() const { return min_len_; }

  void minLength(int p) { min_len_ = p; }

  template <class GRAD, class NMS>
  void setup(const GRAD& grad, const NMS& nms) {}

  void setup(const EdgeSourceI&) {}

  inline void apply(const EdgeSegmentVector& in, const PointVector& points, EdgeSegmentVector& out) const {
    out.clear();
    out.reserve(in.size() * 3);
    int id = 0;
    if (MERGE) {
      EdgeSegmentVector tmp;
      tmp.reserve(20);
      for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) {
        tmp.clear();
        applyD(seg, points, tmp, id++);
        merge(tmp, points, out);
      });
    } else
      for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) { applyD(seg, points, out, id++); });
  }

  template <class DT>
  inline void apply(const EsdBase<DT, PT>& in, EdgeSegmentVector& out) const {
    apply(in.segments(), in.points(), out);
  }

  template <class DT, class NPT>
  inline void apply(const EsdBase<DT, NPT>& in, const PointVector& points, EdgeSegmentVector& out) const {
    apply(in.segments(), points, out);
  }

  inline void apply(const EdgeSegment& seg, const PointVector& points, EdgeSegmentVector& out, int id = 0) const {
    out.clear();
    if (MERGE) {
      EdgeSegmentVector tmp;
      tmp.reserve(20);
      applyD(seg, points, tmp, id);
      merge(tmp, points, out);
    } else
      applyD(seg, points, out, id);
  }

  void applyD(const EdgeSegment& seg, const PointVector& points, EdgeSegmentVector& out, int id) const {
    size_t beg = seg.begin(), end = seg.end(), s = end - beg;
    if (s < static_cast<size_t>(min_len_)) return;

    if (s < 2 * dist_) {
      out.push_back(seg);
      return;
    }

    size_t max_point, max_count = 0;

    const PT& first = points[beg];
    bool no_gap = true;
    // get direction of line
    PT n = points[end - 1] - first, a;
    // get normal
    FT max_h = 0, h;
    set(n, -getY(n), getX(n));
    ++beg;

    for (; beg != end; ++beg) {
      a = points[beg] - first;
      if ((h = static_cast<FT>(std::abs(getX(n) * getX(a) + getY(n) * getY(a)))) > max_h) {
        max_h = h;
        max_point = static_cast<size_t>(beg);
        max_count = 0;
        no_gap = true;
        continue;
      }
      if (h == max_h && no_gap)
        ++max_count;
      else
        no_gap = false;
    }

    if (max_h > dist_ * l2_norm<FT>(n)) {
      size_t max_end = max_point;

      if (max_count != 0) {
        // corner / center check
        max_end += max_count;
        if (seg.begin() + 3 > max_point) {
          max_end = max_point;
        } else if (max_end + 4 > seg.end()) {
          max_point = max_end;
        } else {
          PT tmp = points[max_end] - points[max_point];
          float mdir = cv::fastAtan2(static_cast<float>(getY(tmp)), static_cast<float>(getX(tmp)));
          tmp = points[max_point - 1] - points[max_point - 3];
          float bdir = cv::fastAtan2(static_cast<float>(getY(tmp)), static_cast<float>(getX(tmp)));
          tmp = points[max_end + 3] - points[max_end + 1];
          float edir = cv::fastAtan2(static_cast<float>(getY(tmp)), static_cast<float>(getX(tmp)));
          int mbdiff = static_cast<int>(mdir - bdir + 360) % 180;
          int mediff = static_cast<int>(mdir - edir + 360) % 180;

          if (mbdiff < 45 && mediff < 45) {
            max_point += max_count / 2;
            max_end = max_point + (max_count % 2);
          } else if (mbdiff > mediff) {
            max_end = max_point + (max_count == 1 && mediff > 45);
          } else {
            max_point = max_end - (max_count == 1 && mbdiff > 45);
          }
        }
      }

      applyD(EdgeSegment(seg.begin(), max_point + 1, seg.flags() & ES_REVERSE), points, out, id);
      applyD(EdgeSegment(max_end, seg.end(), seg.flags() & ES_REVERSE), points, out, id);
    } else
      out.push_back(EdgeSegment(seg.begin(), seg.end(), seg.flags() & ES_REVERSE, id));
  }

  inline void apply(const EdgeSegmentVector& in,
                    const EdgeSegmentVector& units,
                    const PointVector& points,
                    EdgeSegmentVector& out) const {
    out.clear();
    out.reserve(units.size());
    int id = 0;
    for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) { applyD(seg, units, points, out, ++id); });
  }

  inline void apply(const EdgeSegment& seg,
                    const EdgeSegmentVector& units,
                    const PointVector& points,
                    EdgeSegmentVector& out,
                    int id = 0) const {
    out.clear();
    applyD(seg, units, points, out, id);
  }

  template <class DT>
  inline void applyP(const EsdBasePattern<DT, PT>& in, EdgeSegmentVector& out) const {
    apply(in.patternSegments(), in.patterns(), in.points(), out);
  }

  template <class DT, class NPT>
  inline void applyP(const EsdBasePattern<DT, NPT>& in, const PointVector& points, EdgeSegmentVector& out) const {
    apply(in.patternSegments(), in.patterns(), points, out);
  }

  void applyD(const EdgeSegment& seg,
              const EdgeSegmentVector& units,
              const PointVector& points,
              EdgeSegmentVector& out,
              int id) const {
    size_t beg = seg.begin(), end = seg.end(), max_pos = 0, s = seg.size();
    if (s < 2) {
      if (s != 0 && units[beg].size() >= static_cast<size_t>(min_len_))
        out.push_back(EdgeSegment(units[beg].begin(), units[beg].end(), seg.flags() & ES_REVERSE, id));
      return;
    }

    const PT& first = points[units[beg].begin()];

    // get direction of line
    PT n = points[units[end - 1].end() - 1] - first, a = points[units[beg].end() - 1] - first, b;
    // get normal
    FT max_h = 0, h;
    set(n, -getY(n), getX(n));
    ++beg;

    for (; beg != end; ++beg) {
      if ((h = static_cast<FT>(std::abs(getX(n) * getX(a) + getY(n) * getY(a)))) > max_h) {
        max_h = h;
        max_pos = beg;
        a = points[units[beg].end() - 1] - first;
        continue;
      }
      a = points[units[beg].end() - 1] - first;
      b = points[units[beg].begin()] - first;
      if ((h = static_cast<FT>(std::abs(getX(n) * getX(b) + getY(n) * getY(b)))) > max_h) {
        max_h = h;
        max_pos = beg;
      }
    }

    if (max_h > dist_ * l2_norm<FT>(n)) {
      applyD(EdgeSegment(seg.begin(), max_pos, seg.flags()), units, points, out, id);
      applyD(EdgeSegment(max_pos, seg.end(), seg.flags()), units, points, out, id);
    } else
      out.push_back(EdgeSegment(units[seg.begin()].begin(), units[seg.end() - 1].end(), seg.flags() & ES_REVERSE, id));
  }

 private:
  void merge(const EdgeSegmentVector& in, const PointVector& points, EdgeSegmentVector& out) const {
    if (in.size() < 2) {
      if (!in.empty()) out.push_back(in[0]);
      return;
    }

    EdgeSegmentVector::const_iterator beg = in.begin(), end = in.end() - 1;

    PT n, a, b, d;
    FT max_h = 0;
    for (; beg < end; ++beg) {
      if (beg->size() < dist_) {
        out.push_back(*beg);
        continue;
      }

      d = points[beg->begin()];
      n = points[(beg + 1)->end() - 1] - d;
      set(n, -getY(n), getX(n));

      a = points[beg->end() - 1] - d;
      b = points[(beg + 1)->begin()] - d;
      d = a - b;

      if ((beg + 1)->size() < dist_ || std::abs(getX(d)) > 1 || std::abs(getY(d)) > 1) {
        out.push_back(*beg);
        ++beg;
        out.push_back(*beg);
        continue;
      }

      max_h = static_cast<FT>(std::abs(getX(n) * getX(a) + getY(n) * getY(a)));
      if (getX(d) != 0 || getY(d) != 0)
        max_h = std::max(max_h, static_cast<FT>(std::abs(getX(n) * getX(b) + getY(n) * getY(b))));
      if (max_h > dist_ * l2_norm<float_type>(n)) {
        out.push_back(*beg);
      } else {
        out.push_back(EdgeSegment(beg->begin(), (beg + 1)->end(), beg->flags() & ES_REVERSE, beg->id()));
        ++beg;
      }
    }
    if (beg == end) out.push_back(*beg);
  }
};

//! simple split check, without relaxed threshold
template <class FT, class PT = Vec2i>
struct SimpleSplitCheck {
  typedef FT float_type;
  typedef PT point_type;
  typedef int mat_type;
  typedef std::vector<PT> PointVector;

  static inline bool check(FT max_h,
                           FT norm,
                           FT dist_low,
                           const PT& max_point,
                           const PT& first,
                           const PT& last,
                           FT max_len,
                           FT dist_high,
                           mat_type mag_max,
                           const cv::Mat& data) {
    return max_h > dist_low * norm;
  }
};

//! extended split check, with relaxed threshold
template <class FT, class MT, class PT = Vec2i>
struct ExtSplitCheck {
  typedef FT float_type;
  typedef PT point_type;
  typedef MT mat_type;
  typedef std::vector<PT> PointVector;

  static inline bool check(FT max_h,
                           FT norm,
                           FT dist_low,
                           const PT& max_point,
                           const PT& first,
                           const PT& last,
                           FT max_len,
                           FT dist_high,
                           mat_type mag_max,
                           const cv::Mat& data) {
    if (max_h <= dist_low * norm) return false;

    FT add = dist_high - dist_low;
    if (add <= 0) return true;

    // get mean magnitude of area
    const MT* pmag = &data.at<MT>(getY(max_point) - 1, getX(max_point) - 1);
    FT f1 = static_cast<FT>(pmag[0] + pmag[1] + pmag[2] + pmag[data.cols] + pmag[data.cols + 1] + pmag[data.cols + 2] +
                            pmag[2 * data.cols] + pmag[2 * data.cols + 1] + pmag[2 * data.cols + 2]) /
            mag_max;
    // if signal is strong, split
    if (f1 >= 1) return true;

    // get length of line parts -> long parts with less significant magnitude will get more error tolerance up to max
    // distance
    f1 = 1 - f1;
    FT f2 = l2_norm<FT, PT>(first - max_point) / max_len;
    if (f2 > 1) f2 = 1;
    FT f3 = l2_norm<FT, PT>(last - max_point) / max_len;
    if (f3 > 1) f3 = 1;
    FT tmp = add * std::cbrt(f1 * f2 * f3);
    return (max_h > (dist_low + add * std::cbrt(f1 * f2 * f3)) * norm);
  }
};

//! no merging after split
template <class ST>
struct NoMerge {
  typedef ST SplitCheck;
  typedef typename SplitCheck::float_type float_type;
  typedef typename SplitCheck::point_type point_type;
  typedef typename SplitCheck::mat_type mat_type;
  typedef typename SplitCheck::PointVector PointVector;

  static inline void apply(const EdgeSegmentVector& in,
                           const PointVector& points,
                           EdgeSegmentVector& out,
                           float_type max_len,
                           float_type dist_low,
                           float_type dist_high,
                           mat_type mag_max,
                           const cv::Mat& data) {
    out.insert(out.end(), in.begin(), in.end());
  }
};

//! enable merging after split
template <class ST>
struct SimpleMerge {
  typedef ST SplitCheck;
  typedef typename SplitCheck::float_type float_type;
  typedef typename SplitCheck::point_type point_type;
  typedef typename SplitCheck::mat_type mat_type;
  typedef typename SplitCheck::PointVector PointVector;

  static void apply(const EdgeSegmentVector& in,
                    const PointVector& points,
                    EdgeSegmentVector& out,
                    float_type max_len,
                    float_type dist_low,
                    float_type dist_high,
                    mat_type mag_max,
                    const cv::Mat& data) {
    if (in.size() < 2) {
      if (!in.empty()) out.push_back(in[0]);
      return;
    }

    EdgeSegmentVector::const_iterator beg = in.begin(), end = in.end() - 1;

    point_type n, a, b, d, max_point;
    float_type max_h = 0, tmp;
    for (; beg < end; ++beg) {
      if (beg->size() < dist_low) {
        out.push_back(*beg);
        continue;
      }


      const point_type& first = points[beg->begin()];
      const point_type& last = points[(beg + 1)->end() - 1];
      n = last - first;
      set(n, -getY(n), getX(n));

      a = points[beg->end() - 1] - first;
      b = points[(beg + 1)->begin()] - first;

      d = a - b;

      if ((beg + 1)->size() < dist_low || std::abs(getX(d)) > 1 || std::abs(getY(d)) > 1) {
        out.push_back(*beg);
        ++beg;
        out.push_back(*beg);
        continue;
      }

      max_h = static_cast<float_type>(std::abs(getX(n) * getX(a) + getY(n) * getY(a)));
      max_point = points[beg->end() - 1];
      if (getX(d) != 0 || getY(d) != 0) {
        tmp = static_cast<float_type>(std::abs(getX(n) * getX(b) + getY(n) * getY(b)));
        if (tmp > max_h) {
          max_h = tmp;
          max_point = points[(beg + 1)->begin()];
        }
      }

      if (SplitCheck::check(max_h, l2_norm<float_type>(n), dist_low, max_point, first, last, max_len, dist_high,
                            mag_max, data)) {
        out.push_back(*beg);
      } else {
        out.push_back(EdgeSegment(beg->begin(), (beg + 1)->end(), beg->flags() & ES_REVERSE, beg->id()));
        ++beg;
      }
    }
    if (beg == end) out.push_back(*beg);
  }
};

//! @brief Extended Ramer or Douglas Peucker split version.
//!        In this version, the merging and splitting can be customized by
//!        templates. The extended split method allows to choose a relaxed
//!        threshold to prevent splits withon badly localized areas.
template <class MT>
class ExtRamerSplit : public ValueManager {
 public:
  typedef MT Merge;
  typedef typename Merge::SplitCheck SplitCheck;
  typedef typename SplitCheck::float_type float_type;
  typedef typename SplitCheck::point_type point_type;
  typedef typename SplitCheck::mat_type mat_type;
  typedef typename SplitCheck::PointVector PointVector;

 private:
  float_type dist_low_, dist_high_, max_len_;
  int min_len_;
  mat_type mag_max_;
  cv::Mat mag_;

  void init() {
    this->add("split_error_distance", std::bind(&ExtRamerSplit<MT>::valueDistanceLow, this, std::placeholders::_1),
              "Lower error distance.");
    this->add("split_min_length", std::bind(&ExtRamerSplit<MT>::valueMinLength, this, std::placeholders::_1),
              "Minimal length (number of pixels).");
    this->add("split_high_error_distance",
              std::bind(&ExtRamerSplit<MT>::valueDistanceHigh, this, std::placeholders::_1), "Upper error distance.");
  }

 public:
  ExtRamerSplit(float_type dist_low = 2, int minp = 2, float_type dist_high = 6)
      : dist_low_(dist_low), dist_high_(dist_high), min_len_(minp), max_len_(0), mag_max_(0) {
    init();
  }

  ExtRamerSplit(const ValueManager::NameValueVector& options)
      : dist_low_(2), dist_high_(6), min_len_(2), max_len_(0), mag_max_(0) {
    init();
    this->value(options);
  }

  ExtRamerSplit(ValueManager::InitializerList options)
      : dist_low_(2), dist_high_(6), min_len_(2), max_len_(0), mag_max_(0) {
    init();
    this->value(options);
  }

  Value valueDistanceLow(const Value& d = Value::NAV()) {
    if (d.type()) distanceLow(d.get<float_type>());
    return dist_low_;
  }

  float_type distanceLow() const { return dist_low_; }

  void distanceLow(float_type d) { dist_low_ = d; }

  Value valueDistanceHigh(const Value& d = Value::NAV()) {
    if (d.type()) distanceHigh(d.get<float_type>());
    return dist_high_;
  }

  float_type distanceHigh() const { return dist_high_; }

  void distanceHigh(float_type d) { dist_high_ = d; }

  void distance(float_type low, float_type high) {
    dist_low_ = low;
    dist_high_ = high;
  }

  Value valueMinLength(const Value& p = Value::NAV()) {
    if (p.type()) minLength(p.getInt());
    return min_len_;
  }

  int minLength() const { return min_len_; }

  void minLength(int p) { min_len_ = p; }

  template <class GRAD, class NMS>
  void setup(const GRAD& grad, const NMS& nms) {
    mag_ = grad.magnitude();
    mag_max_ = nms.magMax() * 7;  // * 0.75 * 9; //*9 normalize sum
    max_len_ = l2_norm<float_type>(point_type(mag_.cols, mag_.rows)) / 3;
  }

  void setup(const EdgeSourceI& esource) {
    mag_ = esource.magnitude();
    mag_max_ = static_cast<mat_type>(esource.magnitudeMax() * 7);  // * 0.75 * 9; //*9 normalize sum
    max_len_ = l2_norm<float_type>(point_type(mag_.cols, mag_.rows)) / 3;
  }

  inline void apply(const EdgeSegmentVector& in, const PointVector& points, EdgeSegmentVector& out) const {
    out.clear();
    out.reserve(in.size() * 3);
    int id = 0;
    for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) {
      EdgeSegmentVector tmp;
      tmp.reserve(20);
      applyD(seg, points, tmp, id++);
      Merge::apply(tmp, points, out, max_len_, dist_low_, dist_high_, mag_max_, mag_);
    });
  }

  template <class DT>
  inline void apply(const EsdBase<DT, point_type>& in, EdgeSegmentVector& out) const {
    apply(in.segments(), in.points(), out);
  }

  template <class DT, class NPT>
  inline void apply(const EsdBase<DT, NPT>& in, const PointVector& points, EdgeSegmentVector& out) const {
    apply(in.segments(), points, out);
  }

  inline void apply(const EdgeSegment& in, const PointVector& points, EdgeSegmentVector& out, int id = 0) const {
    out.clear();
    EdgeSegmentVector tmp;
    tmp.reserve(20);
    applyD(in, points, tmp, id);
    Merge::apply(tmp, points, out, max_len_, dist_low_, dist_high_, mag_max_, mag_);
  }

  inline void applyD(const EdgeSegment& seg, const PointVector& points, EdgeSegmentVector& out, int id) const {
    size_t beg = seg.begin(), end = seg.end(), s = end - beg;
    if (s < static_cast<size_t>(min_len_)) return;

    if (s < 2 * dist_low_) {
      out.push_back(seg);
      return;
    }

    size_t max_point = 0, max_count = 0;

    const point_type& first = points[beg];
    const point_type& last = points[end - 1];
    bool no_gap = true;
    // get direction of line
    point_type n = last - first, a;
    // get normal
    float_type max_h = 0, h;
    set(n, -getY(n), getX(n));
    ++beg;

    for (; beg != end; ++beg) {
      a = points[beg] - first;
      if ((h = static_cast<float_type>(std::abs(getX(n) * getX(a) + getY(n) * getY(a)))) > max_h) {
        max_h = h;
        max_point = static_cast<size_t>(beg);
        max_count = 0;
        no_gap = true;
        continue;
      }
      if (h == max_h && no_gap)
        ++max_count;
      else
        no_gap = false;
    }

    if (SplitCheck::check(max_h, l2_norm<float_type>(n), dist_low_, points[max_point], first, last, max_len_,
                          dist_high_, mag_max_, mag_)) {
      size_t max_end = max_point;

      if (max_count != 0) {
        // corner / center check
        max_end += max_count;
        if (seg.begin() + 3 > max_point) {
          max_end = max_point;
        } else if (max_end + 4 > seg.end()) {
          max_point = max_end;
        } else {
          point_type tmp = points[max_end] - points[max_point];
          float mdir = cv::fastAtan2(static_cast<float>(getY(tmp)), static_cast<float>(getX(tmp)));
          tmp = points[max_point - 1] - points[max_point - 3];
          float bdir = cv::fastAtan2(static_cast<float>(getY(tmp)), static_cast<float>(getX(tmp)));
          tmp = points[max_end + 3] - points[max_end + 1];
          float edir = cv::fastAtan2(static_cast<float>(getY(tmp)), static_cast<float>(getX(tmp)));
          int mbdiff = static_cast<int>(mdir - bdir + 360) % 180;
          int mediff = static_cast<int>(mdir - edir + 360) % 180;

          if (mbdiff < 45 && mediff < 45) {
            max_point += max_count / 2;
            max_end = max_point + (max_count % 2);
          } else if (mbdiff > mediff) {
            max_end = max_point + (max_count == 1 && mediff > 45);
          } else {
            max_point = max_end - (max_count == 1 && mbdiff > 45);
          }
        }
      }

      applyD(EdgeSegment(seg.begin(), max_point + 1, seg.flags() & ES_REVERSE), points, out, id);
      applyD(EdgeSegment(max_end, seg.end(), seg.flags() & ES_REVERSE), points, out, id);
    } else
      out.push_back(EdgeSegment(seg.begin(), seg.end(), seg.flags() & ES_REVERSE, id));
  }

  inline void apply(const EdgeSegmentVector& in,
                    const EdgeSegmentVector& units,
                    const PointVector& points,
                    EdgeSegmentVector& out) const {
    out.clear();
    out.reserve(units.size());
    int id = 0;
    for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) { applyD(seg, units, points, out, id++); });
  }

  inline void apply(const EdgeSegment& in,
                    const EdgeSegmentVector& units,
                    const PointVector& points,
                    EdgeSegmentVector& out,
                    int id = 0) const {
    out.clear();
    applyD(in, units, points, out, id);
  }

  template <class DT>
  inline void applyP(const EsdBasePattern<DT, point_type>& in, EdgeSegmentVector& out) const {
    apply(in.patternSegments(), in.patterns(), in.points(), out);
  }

  template <class DT, class NPT>
  inline void applyP(const EsdBasePattern<DT, NPT>& in, const PointVector& points, EdgeSegmentVector& out) const {
    apply(in.patternSegments(), in.patterns(), points, out);
  }

  void applyD(const EdgeSegment& seg,
              const EdgeSegmentVector& units,
              const PointVector& points,
              EdgeSegmentVector& out,
              int id) const {
    size_t beg = seg.begin(), end = seg.end(), max_pos = 0, s = seg.size();
    if (s < 2) {
      if (s != 0 && units[beg].size() >= static_cast<size_t>(min_len_))
        out.push_back(EdgeSegment(units[beg].begin(), units[beg].end(), seg.flags() & ES_REVERSE, id));
      return;
    }

    const point_type& first = points[units[beg].begin()];
    const point_type& last = points[units[end - 1].end() - 1];

    // get direction of line
    point_type n = last - first, a = points[units[beg].end() - 1], b, max_point, tmp;
    // get normal
    float_type max_h = 0, h;
    set(n, -getY(n), getX(n));
    ++beg;

    for (; beg != end; ++beg) {
      tmp = a - first;
      if ((h = static_cast<float_type>(std::abs(getX(n) * getX(tmp) + getY(n) * getY(tmp)))) > max_h) {
        max_h = h;
        max_pos = static_cast<size_t>(beg);
        max_point = a;
        a = points[units[beg].end() - 1];
        continue;
      }
      a = points[units[beg].end() - 1];
      b = points[units[beg].begin()];
      tmp = b - first;
      if ((h = static_cast<float_type>(std::abs(getX(n) * getX(tmp) + getY(n) * getY(tmp)))) > max_h) {
        max_h = h;
        max_pos = static_cast<size_t>(beg);
        max_point = b;
      }
    }

    if (SplitCheck::check(max_h, l2_norm<float_type>(n), dist_low_, max_point, first, last, max_len_, dist_high_,
                          mag_max_, mag_)) {
      applyD(EdgeSegment(seg.begin(), max_pos, seg.flags()), units, points, out, id);
      applyD(EdgeSegment(max_pos, seg.end(), seg.flags()), units, points, out, id);
    } else
      out.push_back(EdgeSegment(units[seg.begin()].begin(), units[seg.end() - 1].end(), seg.flags() & ES_REVERSE, id));
  }
};

template <class FT, class PT>
inline FT distanceCheck(const Line<FT>& line, const PT& point) {
  return std::abs(line.distance(getX(point), getY(point)));
}

template <class FT, class PT>
inline FT distanceCheck(const Vec2<FT>& normal, const PT& point) {
  return std::abs(getX(normal) * getX(point) + getY(normal) * getY(point));
}

template <class FT>
inline FT distanceCheck(const Vec2<FT>& normal, const Vec2<FT>& point) {
  return std::abs(getX(normal) * getX(point) + getY(normal) * getY(point));
}

template <class FT, class PT>
inline FT distanceCheck(const Vec2<FT>& normal, const Vec2<FT>& line_point, const PT& point) {
  return distanceCheck(normal, Vec2<FT>(getX(point) - line_point.x(), getY(point) - line_point.y()));
}

template <class FT, class PT = Vec2i, bool LINE_MODE = true, class FIT = EigenFit<FT, PT>>
class LeastSquareSplit : public ValueManager {
  FT dist_;
  int min_len_, mode_;

  void init() {
    this->add("split_error_distance",
              std::bind(&LeastSquareSplit<FT, PT, LINE_MODE, FIT>::valueDistance, this, std::placeholders::_1),
              "Error distance.");
    this->add("split_min_length",
              std::bind(&LeastSquareSplit<FT, PT, LINE_MODE, FIT>::valueMinLength, this, std::placeholders::_1),
              "Minimal length (number of pixels).");
  }

 public:
  typedef FT float_type;
  typedef PT point_type;
  typedef std::vector<PT> PointVector;

  LeastSquareSplit(FT dist = 1, int minp = 15) : dist_(dist), min_len_(minp) { init(); }

  LeastSquareSplit(const ValueManager::NameValueVector& options) : dist_(1), min_len_(15) {
    init();
    this->value(options);
  }

  LeastSquareSplit(ValueManager::InitializerList options) : dist_(1), min_len_(15) {
    init();
    this->value(options);
  }

  Value valueDistance(const Value& d = Value::NAV()) {
    if (d.type()) distance(d.get<FT>());
    return dist_;
  }

  FT distance() const { return dist_; }

  void distance(FT d) { dist_ = d; }

  Value valueMinLength(const Value& p = Value::NAV()) {
    if (p.type()) minLength(p.getInt());
    return min_len_;
  }

  int minLength() const { return min_len_; }

  void minLength(int p) { min_len_ = p; }

  static bool line_mode() { return LINE_MODE; }


  template <class GRAD, class NMS>
  void setup(const GRAD& grad, const NMS& nms) {}

  void setup(const EdgeSourceI&) {}

  inline void apply(const EdgeSegmentVector& in, const PointVector& points, EdgeSegmentVector& out) const {
    out.clear();
    out.reserve(in.size() * 3);
    int id = 0;
    if (LINE_MODE) {
      for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) { applyM1(seg, points, out, id++); });
    } else {
      for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) { applyM2(seg, points, out, id++); });
    }
  }

  inline void apply(const EdgeSegment& in, const PointVector& points, EdgeSegmentVector& out, int id = 0) const {
    out.clear();
    applyD(in, points, out, id);
  }

  template <class DT>
  inline void apply(const EsdBase<DT, PT>& in, EdgeSegmentVector& out) const {
    apply(in.segments(), in.points(), out);
  }

  template <class DT, class NPT>
  inline void apply(const EsdBase<DT, NPT>& in, const PointVector& points, EdgeSegmentVector& out) const {
    apply(in.segments(), points, out);
  }

  inline void applyD(const EdgeSegment& seg, const PointVector& points, EdgeSegmentVector& out, int id) const {
    if (LINE_MODE)
      applyM1(seg, points, out, id);
    else
      applyM2(seg, points, out, id);
  }

  inline void apply(const EdgeSegmentVector& in,
                    const EdgeSegmentVector& units,
                    const PointVector& points,
                    EdgeSegmentVector& out) const {
    out.clear();
    out.reserve(units.size());
    int id = 0;
    if (LINE_MODE) {
      for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) { applyM1(seg, units, points, out, id++); });
    } else {
      for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) { applyM2(seg, units, points, out, id++); });
    }
  }

  inline void apply(const EdgeSegment& seg,
                    const EdgeSegmentVector& units,
                    const PointVector& points,
                    EdgeSegmentVector& out,
                    int id = 0) const {
    out.clear();
    applyD(seg, units, points, out, id);
  }

  template <class DT>
  inline void applyP(const EsdBasePattern<DT, PT>& in, EdgeSegmentVector& out) const {
    apply(in.patternSegments(), in.patterns(), in.points(), out);
  }

  template <class DT, class NPT>
  inline void applyP(const EsdBasePattern<DT, NPT>& in, const PointVector& points, EdgeSegmentVector& out) const {
    apply(in.patternSegments(), in.patterns(), points, out);
  }

  inline void applyD(const EdgeSegment& seg,
                     const EdgeSegmentVector& units,
                     const PointVector& points,
                     EdgeSegmentVector& out,
                     int id) const {
    if (LINE_MODE)
      applyM1(seg, units, points, out, id);
    else
      applyM2(seg, units, points, out, id);
  }

 private:
  void applyM1(const EdgeSegment& seg, const PointVector& points, EdgeSegmentVector& out, int id) const {
    const PT* pPoints = points.data();
    const PT* pbeg;
    const PT* pend;
    size_t beg = seg.begin(), end = seg.end(), s = end - beg, end2 = end - min_len_;
    if (s < static_cast<size_t>(min_len_)) return;

    Line<FT> line;
    for (; beg < end; ++beg) {
      pbeg = pPoints + beg;
      pend = pbeg + min_len_;

      FIT::fit(pbeg, pend, line);
      FT max_dist = 0;
      for (; pbeg != pend; ++pbeg) std::max(max_dist, distanceCheck(line, *pbeg));
      if (max_dist <= dist_) break;
    }

    // nothing found
    if (beg == end2) return;

    end2 = beg + min_len_;

    // add points
    for (; end2 != end; ++end2) {
      if (distanceCheck(line, points[end2]) > dist_) break;
    }

    out.push_back(EdgeSegment(beg, end2, seg.flags() & ES_REVERSE, id));
    if (end2 != end) applyM1(EdgeSegment(end2, end, seg.flags() & ES_REVERSE), points, out, id);
  }

  void applyM2(const EdgeSegment& seg, const PointVector& points, EdgeSegmentVector& out, int id) const {
    const PT* pPoints = points.data();
    const PT* pbeg;
    const PT* pend;
    size_t beg = seg.begin(), end = seg.end(), s = end - beg, end2 = end - min_len_;
    if (s < static_cast<size_t>(min_len_)) return;

    Vec2<FT> normal, centroid;
    FT max_dist, dist_norm;
    for (; beg < end; ++beg) {
      pbeg = pPoints + beg;
      pend = pbeg + min_len_;

      FIT::fit_unorm(pbeg, pend, getX(centroid), getY(centroid), getX(normal), getY(normal));
      max_dist = 0;
      dist_norm = dist_ * l2_norm<FT>(normal);
      for (; pbeg != pend; ++pbeg) std::max(max_dist, distanceCheck(normal, centroid, *pbeg));
      if (max_dist <= dist_norm) break;
    }

    // nothing found
    if (beg == end2) return;

    end2 = beg + min_len_;

    // add points
    for (; end2 != end; ++end2) {
      if (distanceCheck(normal, centroid, points[end2]) > dist_norm) break;
    }

    out.push_back(EdgeSegment(beg, end2, seg.flags() & ES_REVERSE, id));
    if (end2 != end) applyM2(EdgeSegment(end2, end, seg.flags() & ES_REVERSE), points, out, id);
  }

  void applyM1(const EdgeSegment& seg,
               const EdgeSegmentVector& units,
               const PointVector& points,
               EdgeSegmentVector& out,
               int id) const {
    size_t beg = seg.begin(), end = seg.end(), segment_size = end - beg;
    if (segment_size < 2) {
      if (segment_size != 0 && units[beg].size() >= static_cast<size_t>(min_len_)) {
        out.push_back(units[beg]);
        out.back().id(id);
      }
      return;
    }

    size_t unit_size = units[beg].size(), pos = beg + 1;

    const PT* pbeg{};
    const PT* pend{};

    Line<FT> line;
    // try to get initial fit
    for (; beg < end; ++beg) {
      while (unit_size < static_cast<size_t>(min_len_) && pos != end) {
        unit_size += units[pos].size();
        ++pos;
      }

      // segment of all units is still to small
      if (unit_size < static_cast<size_t>(min_len_)) return;

      pbeg = points.data() + units[beg].begin();
      pend = points.data() + units[pos - 1].end();

      // if only one pattern is included, a fit is found
      if (beg + 1 == pos) {
        line = Line<FT>(*pbeg, *pend);
        break;
      }

      FIT::fit(pbeg, pend, line);

      // check if all distances are less threshold
      FT max_dist = distanceCheck(line, points[units[beg].begin()]);
      for (size_t ubeg = beg; ubeg != pos; ++ubeg) {
        std::max(max_dist, distanceCheck(line, points[units[ubeg].end() - 1]));
      }
      if (max_dist <= dist_) break;

      unit_size -= units[beg].size();
    }

    // add units
    for (; pos < end; ++pos) {
      if (distanceCheck(line, points[units[pos].end() - 1]) > dist_) break;
    }

    out.push_back(EdgeSegment(units[beg].begin(), units[pos - 1].end(), units[beg].flags() & ES_REVERSE, id));
    if (pos != end) applyM1(EdgeSegment(pos, end), units, points, out, id);
  }

  void applyM2(const EdgeSegment& seg,
               const EdgeSegmentVector& units,
               const PointVector& points,
               EdgeSegmentVector& out,
               int id) const {
    size_t beg = seg.begin(), end = seg.end(), segment_size = end - beg;
    if (segment_size < 2) {
      if (segment_size != 0 && units[beg].size() >= static_cast<size_t>(min_len_)) {
        out.push_back(units[beg]);
        out.back().id(id);
      }
      return;
    }

    size_t unit_size = units[beg].size(), pos = beg + 1;

    const PT* pbeg;
    const PT* pend;

    Vec2<FT> normal, centroid;
    FT max_dist, dist_norm;
    // try to get initial fit
    for (; beg < end; ++beg) {
      while (unit_size < static_cast<size_t>(min_len_) && pos != end) {
        unit_size += units[pos].size();
        ++pos;
      }

      // segment of all units is still to small
      if (unit_size < static_cast<size_t>(min_len_)) return;

      pbeg = points.data() + units[beg].begin();
      pend = points.data() + units[pos - 1].end();

      // if only one pattern is included, a fit is found
      if (beg + 1 == pos) {
        normal[0] = getX(*(pend - 1)) - getX(*pbeg);
        normal[1] = getY(*(pend - 1)) - getY(*pbeg);
        centroid[0] = getX(*pbeg);
        centroid[1] = getY(*pbeg);
        dist_norm = dist_ * l2_norm<FT>(normal);
        break;
      }

      FIT::fit_unorm(pbeg, pend, getX(centroid), getY(centroid), getX(normal), getY(normal));
      dist_norm = dist_ * l2_norm<FT>(normal);

      // check if all distances are less threshold
      max_dist = distanceCheck(normal, points[units[beg].begin()]);
      for (size_t ubeg = beg; ubeg != pos; ++ubeg) {
        std::max(max_dist, distanceCheck(normal, centroid, points[units[ubeg].end() - 1]));
      }
      if (max_dist <= dist_norm) break;

      unit_size -= units[beg].size();
    }

    // add unit
    for (; pos < end; ++pos) {
      if (distanceCheck(normal, centroid, points[units[pos].end() - 1]) > dist_norm) break;
    }

    out.push_back(EdgeSegment(units[beg].begin(), units[pos - 1].end(), units[beg].flags() & ES_REVERSE, id));
    if (pos != end) applyM2(EdgeSegment(pos, end), units, points, out, id);
  }
};

template <class FT, class PT = Vec2i, class FIT = EigenFit<FT, PT>>
class AdaptiveLeastSquareSplit : public ValueManager {
  FT dist_;
  int min_len_, mode_;

  void init() {
    this->add("split_error_distance",
              std::bind(&AdaptiveLeastSquareSplit<FT, PT, FIT>::valueDistance, this, std::placeholders::_1),
              "Error distance.");
    this->add("split_min_length",
              std::bind(&AdaptiveLeastSquareSplit<FT, PT, FIT>::valueMinLength, this, std::placeholders::_1),
              "Minimal length (number of pixels).");
  }

 public:
  typedef FT float_type;
  typedef PT point_type;
  typedef std::vector<PT> PointVector;

  AdaptiveLeastSquareSplit(FT dist = 1, int minp = 15) : dist_(dist), min_len_(minp) { init(); }

  AdaptiveLeastSquareSplit(const ValueManager::NameValueVector& options) : dist_(1), min_len_(15) {
    init();
    this->value(options);
  }

  AdaptiveLeastSquareSplit(ValueManager::InitializerList options) : dist_(1), min_len_(15) {
    init();
    this->value(options);
  }

  Value valueDistance(const Value& d = Value::NAV()) {
    if (d.type()) distance(d.get<FT>());
    return dist_;
  }

  FT distance() const { return dist_; }

  void distance(FT d) { dist_ = d; }

  Value valueMinLength(const Value& p = Value::NAV()) {
    if (p.type()) minLength(p.getInt());
    return min_len_;
  }

  int minLength() const { return min_len_; }

  void minLength(int p) { min_len_ = p; }

  template <class GRAD, class NMS>
  void setup(const GRAD& grad, const NMS& nms) {}

  void setup(const EdgeSourceI&) {}

  inline void apply(const EdgeSegmentVector& in, const PointVector& points, EdgeSegmentVector& out) const {
    out.clear();
    out.reserve(in.size() * 3);
    int id = 0;
    for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) { applyD(seg, points, out, id++); });
  }

  inline void apply(const EdgeSegment& in, const PointVector& points, EdgeSegmentVector& out, int id = 0) const {
    out.clear();
    applyD(in, points, out, id);
  }

  template <class DT>
  inline void apply(const EsdBase<DT, PT>& in, EdgeSegmentVector& out) const {
    apply(in.segments(), in.points(), out);
  }

  template <class DT, class NPT>
  inline void apply(const EsdBase<DT, NPT>& in, const PointVector& points, EdgeSegmentVector& out) const {
    apply(in.segments(), points, out);
  }

  inline void applyD(const EdgeSegment& seg, const PointVector& points, EdgeSegmentVector& out, int id) const {
    const PT* pPoints = points.data();
    const PT* pbeg;
    const PT* pend;
    size_t beg = seg.begin(), end = seg.end(), s = end - beg, end2 = end - min_len_;
    if (s < static_cast<size_t>(min_len_)) return;

    Line<FT> line;
    FT max_dist = 0;
    for (; beg < end; ++beg) {
      pbeg = pPoints + beg;
      pend = pbeg + min_len_;

      FIT::fit(pbeg, pend, line);
      max_dist = 0;
      for (; pbeg != pend; ++pbeg) std::max(max_dist, distanceCheck(line, *pbeg));
      if (max_dist <= dist_) break;
    }

    // nothing found
    if (beg == end2) return;

    end2 = beg + min_len_;
    pbeg = pPoints + beg;
    pend = pPoints + end2;

    // add points
    for (; end2 < end; ++end2) {
      if (distanceCheck(line, *pend) > dist_) break;

      ++pend;
      // update line fit
      FIT::fit(pbeg, pend, line);

      // test if all points still fit
      max_dist = 0;
      for (const PT* ubeg = pbeg; ubeg != pend; ++ubeg) std::max(max_dist, distanceCheck(line, *ubeg));
      if (max_dist > dist_) break;
    }

    out.push_back(EdgeSegment(beg, end2, seg.flags() & ES_REVERSE, id));
    if (end2 != end) applyD(EdgeSegment(end2, end, seg.flags() & ES_REVERSE), points, out, id);
  }

  inline void apply(const EdgeSegmentVector& in,
                    const EdgeSegmentVector& units,
                    const PointVector& points,
                    EdgeSegmentVector& out) const {
    out.clear();
    out.reserve(units.size());
    int id = 0;
    for_each(in.begin(), in.end(), [&](const EdgeSegment& seg) { applyD(seg, units, points, out, id++); });
  }

  inline void apply(const EdgeSegment& seg,
                    const EdgeSegmentVector& units,
                    const PointVector& points,
                    EdgeSegmentVector& out,
                    int id = 0) const {
    out.clear();
    applyD(seg, units, points, out, id);
  }

  template <class DT>
  inline void applyP(const EsdBasePattern<DT, PT>& in, EdgeSegmentVector& out) const {
    apply(in.patternSegments(), in.patterns(), in.points(), out);
  }

  template <class DT, class NPT>
  inline void applyP(const EsdBasePattern<DT, NPT>& in, const PointVector& points, EdgeSegmentVector& out) const {
    apply(in.patternSegments(), in.patterns(), points, out);
  }

  inline void applyD(const EdgeSegment& seg,
                     const EdgeSegmentVector& units,
                     const PointVector& points,
                     EdgeSegmentVector& out,
                     int id) const {
    size_t beg = seg.begin(), end = seg.end(), segment_size = end - beg;
    if (segment_size < 2) {
      if (segment_size != 0 && units[beg].size() >= static_cast<size_t>(min_len_)) {
        out.push_back(units[beg]);
        out.back().id(id);
      }
      return;
    }

    size_t unit_size = units[beg].size(), pos = beg + 1;

    const PT* pbeg{};
    const PT* pend{};

    Line<FT> line;
    // try to get initial fit
    for (; beg < end; ++beg, pbeg += 2) {
      while (unit_size < static_cast<size_t>(min_len_) && pos != end) {
        unit_size += units[pos].size();
        ++pos;
      }

      // segment of all units is still to small
      if (unit_size < static_cast<size_t>(min_len_)) return;

      pbeg = points.data() + units[beg].begin();
      pend = points.data() + units[pos - 1].end();

      // if only one pattern is included, a fit is found
      if (beg + 1 == pos) {
        line = Line<FT>(*pbeg, *pend);
        break;
      }

      FIT::fit(pbeg, pend, line);

      // check if all distances are less threshold
      FT max_dist = distanceCheck(line, points[units[beg].begin()]);
      for (size_t ubeg = beg; ubeg != pos; ++ubeg) {
        std::max(max_dist, distanceCheck(line, points[units[ubeg].end() - 1]));
      }
      if (max_dist <= dist_) break;

      unit_size -= units[beg].size();
    }

    // add unit
    for (; pos < end; ++pos) {
      if (distanceCheck(line, points[units[beg].end() - 1]) > dist_) break;
      pend = points.data() + units[pos - 1].end();

      // update line fit
      FIT::fit(pbeg, pend, line);

      // test if all points still fit
      FT max_dist = distanceCheck(line, points[units[beg].begin()]);
      for (size_t ubeg = beg; ubeg != pos; ++ubeg) {
        std::max(max_dist, distanceCheck(line, points[units[ubeg].end() - 1]));
      }
      if (max_dist > dist_) break;
    }

    out.push_back(EdgeSegment(units[beg].begin(), units[pos - 1].end(), units[beg].flags() & ES_REVERSE, id));
    if (pos != end) applyD(EdgeSegment(pos, end), units, points, out, id);
  }
};

}  // namespace lsfm
