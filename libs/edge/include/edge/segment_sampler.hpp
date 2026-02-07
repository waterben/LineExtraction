//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file segment_sampler.hpp
/// @brief Edge segment sampling and interpolation utilities.
/// Provides methods to sample points along edge segments and interpolate values
/// from magnitude maps for analysis and validation.

#pragma once

#include <edge/edge_segment.hpp>
#include <edge/index.hpp>

#include <utility>

namespace lsfm {

template <class PT>
class SegmentSampler : public ValueManager {
  SegmentSampler(const SegmentSampler&);

 public:
  typedef PT point_type;
  typedef std::vector<PT> PointVector;

  template <class PV, class COLS>
  inline void process(const PV& v, const COLS& cols) {
    points_.clear();
    points_.reserve(v.size());
    segments_.clear();
    processSingle(v, cols);
  }

  template <class ITER, class COLS>
  inline void process(const ITER& beg, const ITER& end, const COLS& cols) {
    points_.clear();
    points_.reserve(end - beg);
    segments_.clear();
    processImpl(&(*beg), &(*(end - 1)) + 1, cols);
  }

  template <class PVV, class COLS>
  inline void processV(const PVV& vv, const COLS& cols) {
    points_.clear();
    segments_.clear();
    segments_.reserve(vv.size());
    for_each(vv.begin(), vv.end(), [&](const typename PVV::value_type& v) { processSingle(v, cols); });
  }

  template <class COLS>
  inline void process(const EdgeSegmentVector& s, const IndexVector& points, const COLS& cols) {
    points_.clear();
    points_.reserve(points.size());
    segments_.clear();
    segments_.reserve(s.size());
    for_each(s.begin(), s.end(), [&](const EdgeSegment& seg) {
      processImpl(&points[seg.begin()], &points[seg.begin()] + seg.size(), cols, seg.flags());
    });
  }

  const PointVector& points() const { return points_; }

  const EdgeSegmentVector& segments() const { return segments_; }

  virtual std::string name() const = 0;

 protected:
  SegmentSampler() {}

  template <class PV, class COLS>
  inline void processSingle(const PV& v, const COLS& cols) {
    processImpl(&v[0], &v[0] + v.size());
  }

  inline void processImpl(const index_type* beg, const index_type* end, const cv::Mat& cols, int flags = ES_NONE) {
    processImpl(beg, end, cols.cols, flags);
  }

  virtual void processImpl(const index_type* beg, const index_type* end, int cols, int flags = ES_NONE) = 0;

  PointVector points_;
  EdgeSegmentVector segments_;
};


template <class PT>
class UniformSegmentSampler : public SegmentSampler<PT> {
  int dist_, min_points_;
  using SegmentSampler<PT>::points_;
  using SegmentSampler<PT>::segments_;

 public:
  UniformSegmentSampler(int dist = 10, int mp = 15) : dist_(dist), min_points_(mp) {
    this->add("distance", std::bind(&UniformSegmentSampler<PT>::valueDistance, this, std::placeholders::_1),
              "Sampling distance.");
    this->add("min_points", std::bind(&UniformSegmentSampler<PT>::valueMinPoints, this, std::placeholders::_1),
              "Minimum number of points.");
  }

  Value valueDistance(const Value& d = Value::NAV()) {
    if (d.type()) distance(d.getInt());
    return dist_;
  }

  int distance() const { return dist_; }

  void distance(int d) { dist_ = d; }

  Value valueMinPoints(const Value& mp = Value::NAV()) {
    if (mp.type()) minPoints(mp.getInt());
    return min_points_;
  }

  int minPoints() const { return min_points_; }

  void minPoints(int mp) { min_points_ = mp; }

  std::string name() const { return "uniform"; }

 private:
  void processImpl(const index_type* beg, const index_type* end, int cols, int flags = ES_NONE) {
    int size = static_cast<int>(end - beg);
    // segment has to be greater than min points
    if (size < min_points_) return;

    int start = static_cast<int>(this->points_.size());
    // get size of segment
    int steps = size / dist_;

    if (steps == 0) {
      // add first point in segment
      points_.push_back(IndexConvert<PT>::toPoint(*beg, cols));
    } else {
      float dsize = static_cast<float>(size) / steps;
      float dist = 0;
      if (size % dist_ == 0) --steps;

      // walk through segment accessing every 'dist' point
      for (int i = 0; i < steps; ++i, dist += dsize)
        this->points_.push_back(IndexConvert<PT>::toPoint(beg[static_cast<size_t>(dist)], cols));
    }

    // add last point in segment only of not cloesd
    if (!(flags & ES_CLOSED)) points_.push_back(IndexConvert<PT>::toPoint(*(beg + size - 1), cols));

    segments_.push_back(EdgeSegment(start, points_.size(), flags));
  }
};

template <class PT>
class AdaptiveSegmentSampler : public SegmentSampler<PT> {
  const index_type* data_;

  int max_dist_, min_points_, flags_, size_, cols_;
  float err_dist_;

  using SegmentSampler<PT>::points_;
  using SegmentSampler<PT>::segments_;

  std::vector<cv::Point> tmpPoints_;

 public:
  AdaptiveSegmentSampler(int md = 100, int mp = 15, float err = 2) : err_dist_(err), max_dist_(md), min_points_(mp) {
    this->add("max_dist", std::bind(&AdaptiveSegmentSampler<PT>::valueMaxDistance, this, std::placeholders::_1),
              "Maximal sampling distance.");
    this->add("min_points", std::bind(&AdaptiveSegmentSampler<PT>::valueMinPoints, this, std::placeholders::_1),
              "Minimum number of points.");
    this->add("err_dist", std::bind(&AdaptiveSegmentSampler<PT>::valueErrorDistance, this, std::placeholders::_1),
              "Error distance for adptive splitting.");
  }

  Value valueMaxDistance(const Value& d = Value::NAV()) {
    if (d.type()) maxDistance(d.getInt());
    return max_dist_;
  }

  int maxDistance() const { return max_dist_; }

  void maxDistance(int d) { max_dist_ = d; }

  Value valueMinPoints(const Value& mp = Value::NAV()) {
    if (mp.type()) minPoints(mp.getInt());
    return min_points_;
  }

  int minPoints() const { return min_points_; }

  void minPoints(int mp) { min_points_ = mp; }

  Value valueErrorDistance(const Value& d = Value::NAV()) {
    if (d.type()) errorDistance(d.getFloat());
    return err_dist_;
  }

  float errorDistance() const { return err_dist_; }

  void errorDistance(float d) { err_dist_ = d; }

  std::string name() const { return "adaptive"; }

 private:
  void processImpl(const index_type* beg, const index_type* end, int cols, int flags = ES_NONE) {
    int size = static_cast<int>(end - beg);
    if (size < min_points_) return;
    flags_ = flags;
    size_ = size;
    data_ = beg;
    cols_ = cols;
    int start = static_cast<int>(points_.size());
    tmpPoints_.resize(size);
    IndexConvert<cv::Point>::toPoint(beg, end, &tmpPoints_[0], cols);
    points_.push_back(IndexConvert<PT>::toPoint(*data_, cols));
    searchDetect(0, size, PT());
    segments_.push_back(EdgeSegment(start, points_.size(), flags));
  }

  template <class T>
  void searchDetect(int sbeg, int send, T) {
    search(sbeg, send);
  }

  void searchDetect(int sbeg, int send, index_type) { searchI(sbeg, send); }

  void searchDetect(int sbeg, int send, cv::Point) { searchP(sbeg, send); }

  // recursive solution
  void searchP(int sbeg, int send) {
    int s = send - sbeg;

    const PT *beg = tmpPoints_.data() + sbeg, *end = tmpPoints_.data() + send - 1;

    if (s < 2 * err_dist_) {
      if (!(send == size_ && (flags_ & ES_CLOSED))) points_.push_back(beg[s - 1]);
      return;
    }

    int max_point;

    const PT& first = *beg;
    bool no_gap = true;
    // get direction of line
    PT n = *end - first, a;
    // get normal
    int max_h = 0, max_count = 0, h;
    std::swap(n.x, n.y);
    ++beg;

    for (; beg != end; ++beg) {
      a = *beg - first;
      if ((h = std::abs(getX(n) * getX(a) + getY(n) * getY(a))) > max_h) {
        max_h = h;
        max_point = static_cast<int>(beg - tmpPoints_.data());
        max_count = 0;
        no_gap = true;
        continue;
      }
      if (h == max_h && no_gap)
        ++max_count;
      else
        no_gap = false;
    }

    if (max_h > err_dist_ * l2_norm<float, PT>(n)) {
      int max_end = max_point;

      if (max_count != 0) {
        // corner / center check
        max_end += max_count;
        if (sbeg + 3 > max_point) {
          max_end = max_point;
        } else if (max_end + 4 > send) {
          max_point = max_end;
        } else {
          PT tmp = tmpPoints_[max_end] - tmpPoints_[max_point];
          float mdir = cv::fastAtan2(static_cast<float>(tmp.y), static_cast<float>(tmp.x));
          tmp = tmpPoints_[max_point - 1] - tmpPoints_[max_point - 3];
          float bdir = cv::fastAtan2(static_cast<float>(tmp.y), static_cast<float>(tmp.x));
          tmp = tmpPoints_[max_end + 3] - tmpPoints_[max_end + 1];
          float edir = cv::fastAtan2(static_cast<float>(tmp.y), static_cast<float>(tmp.x));
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

      searchP(sbeg, max_point + 1);
      searchP(max_end, send);
    } else {
      if (max_dist_ > 0 && s > max_dist_) {
        beg = &tmpPoints_[sbeg];
        int steps = s / max_dist_;
        float dsize = static_cast<float>(s) / steps;
        float dist = dsize;
        if (s % max_dist_ == 0) --steps;

        // walk through segment accessing every 'dist' point
        for (int i = 1; i < steps; ++i, dist += dsize) points_.push_back(beg[static_cast<size_t>(dist)]);
      }
      if (!(send == size_ && (this->flags_ & ES_CLOSED))) this->points_.push_back(tmpPoints_[send - 1]);
    }
  }

  // recursive solution
  void search(int sbeg, int send) {
    int s = send - sbeg;

    const cv::Point *beg = tmpPoints_.data() + sbeg, *end = tmpPoints_.data() + send - 1;
    cv::Point tmp;

    if (s < 2 * err_dist_) {
      if (!(send == size_ && (flags_ & ES_CLOSED))) {
        tmp = beg[s - 1];
        points_.push_back(PT(tmp.x, tmp.y));
      }
      return;
    }

    int max_point;

    const cv::Point& first = beg;
    bool no_gap = true;
    // get direction of line
    cv::Point n = *end - first, a;
    // get normal
    int max_h = 0, max_count = 0, h;
    std::swap(getX(n), getY(n));
    ++beg;

    for (; beg != end; ++beg) {
      a = *beg - first;
      if ((h = std::abs(getX(n) * getX(a) + getY(n) * getY(a))) > max_h) {
        max_h = h;
        max_point = static_cast<int>(beg - tmpPoints_.data());
        max_count = 0;
        no_gap = true;
        continue;
      }
      if (h == max_h && no_gap)
        ++max_count;
      else
        no_gap = false;
    }

    if (max_h > err_dist_ * l2_norm<float, cv::Point>(n)) {
      int max_end = max_point;

      if (max_count != 0) {
        // corner / center check
        max_end += max_count;
        if (sbeg + 3 > max_point) {
          max_end = max_point;
        } else if (max_end + 4 > send) {
          max_point = max_end;
        } else {
          cv::Point tmp = tmpPoints_[max_end] - tmpPoints_[max_point];
          float mdir = cv::fastAtan2(static_cast<float>(tmp.y), static_cast<float>(tmp.x));
          tmp = tmpPoints_[max_point - 1] - tmpPoints_[max_point - 3];
          float bdir = cv::fastAtan2(static_cast<float>(tmp.y), static_cast<float>(tmp.x));
          tmp = tmpPoints_[max_end + 3] - tmpPoints_[max_end + 1];
          float edir = cv::fastAtan2(static_cast<float>(tmp.y), static_cast<float>(tmp.x));
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

      search(sbeg, max_point + 1);
      search(max_end, send);
    } else {
      if (max_dist_ > 0 && s > max_dist_) {
        beg = &data_[sbeg];
        int steps = s / max_dist_;
        float dsize = static_cast<float>(s) / steps;
        float dist = dsize;
        if (s % max_dist_ == 0) --steps;

        // walk through segment accessing every 'dist' point
        for (int i = 1; i < steps; ++i, dist += dsize) {
          tmp = beg[static_cast<size_t>(dist)];
          this->points_.push_back(PT(tmp.x, tmp.y));
        }
      }
      if (!(send == size_ && (this->flags_ & ES_CLOSED))) {
        tmp = tmpPoints_[send - 1];
        this->points_.push_back(PT(tmp.x, tmp.y));
      }
    }
  }

  // recursive solution
  void searchI(int sbeg, int send) {
    int s = send - sbeg;

    const cv::Point *beg = tmpPoints_.data() + sbeg, *end = tmpPoints_.data() + send - 1;
    cv::Point tmp;

    if (s < 2 * err_dist_) {
      if (!(send == size_ && (flags_ & ES_CLOSED))) {
        tmp = beg[s - 1];
        points_.push_back(data_[sbeg]);
      }
      return;
    }

    int max_point;

    const cv::Point& first = beg;
    bool no_gap = true;
    // get direction of line
    cv::Point n = *end - first, a;
    // get normal
    int max_h = 0, max_count = 0, h;
    std::swap(getX(n), getY(n));
    ++beg;

    for (; beg != end; ++beg) {
      a = *beg - first;
      if ((h = std::abs(getX(n) * getX(a) + getY(n) * getY(a))) > max_h) {
        max_h = h;
        max_point = static_cast<int>(beg - tmpPoints_.data());
        max_count = 0;
        no_gap = true;
        continue;
      }
      if (h == max_h && no_gap)
        ++max_count;
      else
        no_gap = false;
    }

    if (max_h > err_dist_ * l2_norm<float, cv::Point>(n)) {
      int max_end = max_point;

      if (max_count != 0) {
        // corner / center check
        max_end += max_count;
        if (sbeg + 3 > max_point) {
          max_end = max_point;
        } else if (max_end + 4 > send) {
          max_point = max_end;
        } else {
          cv::Point tmp = tmpPoints_[max_end] - tmpPoints_[max_point];
          float mdir = cv::fastAtan2(static_cast<float>(tmp.y), static_cast<float>(tmp.x));
          tmp = tmpPoints_[max_point - 1] - tmpPoints_[max_point - 3];
          float bdir = cv::fastAtan2(static_cast<float>(tmp.y), static_cast<float>(tmp.x));
          tmp = tmpPoints_[max_end + 3] - tmpPoints_[max_end + 1];
          float edir = cv::fastAtan2(static_cast<float>(tmp.y), static_cast<float>(tmp.x));
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

      searchI(sbeg, max_point + 1);
      searchI(max_end, send);
    } else {
      if (max_dist_ > 0 && s > max_dist_) {
        beg = &data_[sbeg];
        int steps = s / max_dist_;
        float dsize = static_cast<float>(s) / steps;
        float dist = dsize;
        if (s % max_dist_ == 0) --steps;

        // walk through segment accessing every 'dist' point
        for (int i = 1; i < steps; ++i, dist += dsize) {
          this->points_.push_back(data_[sbeg + static_cast<int>(dist)]);
        }
      }
      if (!(send == size_ && (this->flags_ & ES_CLOSED))) {
        this->points_.push_back(data_[send - 1]);
      }
    }
  }
};

}  // namespace lsfm
