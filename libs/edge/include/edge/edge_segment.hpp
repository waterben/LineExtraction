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
 *  (C) by Benjamin Wassermann
 */

/// @file edge_segment.hpp
/// @brief Edge segment data structures and operations.
/// Defines edge segment representation and base classes for edge detection algorithms,
/// providing a common interface for different edge detection methods.

#pragma once

#include <edge/edge_source.hpp>
#include <utility/value_manager.hpp>

namespace lsfm {

/// @brief Flags for edge segment properties.
enum {
  ES_NONE = 0,     ///< No special flags
  ES_REVERSE = 1,  ///< Segment points are in reverse order
  ES_CLOSED = 2    ///< Segment forms a closed loop
};

// edge segment class
/*class EdgeSegment {
    uint32_t beg_, end_;
    int flags_, id_;

public:

    EdgeSegment(size_t b = 0, size_t e = 0, int f = ES_NONE, int id = 0)
        : beg_(static_cast<uint32_t>(b)), end_(static_cast<uint32_t>(e)), flags_(f), id_(id) {}

    // number of supporting points
    inline size_t size() const {
        return static_cast<size_t>(end_ - beg_);
    }

    inline size_t begin() const {
        return beg_;
    }

    inline size_t end() const {
        return end_;
    }

    inline void begin(size_t b) {
        beg_ = static_cast<uint32_t>(b);
    }

    inline void end(size_t e) {
        end_ = static_cast<uint32_t>(e);
    }

    inline void size(size_t s) {
        end_ = beg_ + static_cast<uint32_t>(s);
    }

    inline void moveTo(size_t p) {
        move(p - beg_);
    }

    inline void move(ptrdiff_t m) {
        beg_ += static_cast<int32_t>(m);
        end_ += static_cast<int32_t>(m);
    }

    inline size_t first() const{
        return reverse() ? end_ - 1 : beg_;
    }

    inline size_t last() const {
        return reverse() ? beg_ : end_ - 1;
    }

    inline bool reverse() const {
        return (flags_ & ES_REVERSE) != 0;
    }

    inline bool closed() const {
        return (flags_ & ES_CLOSED) != 0;
    }

    inline int flags() const {
        return flags_;
    }

    inline void flags(int f) {
        flags_ = f;
    }

    inline int id() const {
        return id_;
    }

    inline void id(int i) {
        id_ = i;
    }
};*/

class EdgeSegment {
  size_t beg_, end_;
  int flags_, id_;

 public:
  /// @brief Construct an edge segment.
  /// @param b Starting index in the point array
  /// @param e Ending index in the point array (exclusive)
  /// @param f Flags (ES_NONE, ES_REVERSE, ES_CLOSED)
  /// @param id Optional segment identifier
  EdgeSegment(size_t b = 0, size_t e = 0, int f = ES_NONE, int id = 0) : beg_(b), end_(e), flags_(f), id_(id) {}

  /// @brief Get the number of supporting points in this segment.
  /// @return Size of the segment (number of points)
  inline size_t size() const { return end_ - beg_; }

  /// @brief Get the beginning index of the segment.
  /// @return Index of the first point in the point array
  inline size_t begin() const { return beg_; }

  /// @brief Get the ending index of the segment (exclusive).
  /// @return Index one past the last point in the point array
  inline size_t end() const { return end_; }

  /// @brief Set the beginning index of the segment.
  /// @param b New starting index
  inline void begin(size_t b) { beg_ = b; }

  /// @brief Set the ending index of the segment (exclusive).
  /// @param e New ending index
  inline void end(size_t e) { end_ = e; }

  /// @brief Set the size of the segment, adjusting end based on begin.
  /// @param s New size of the segment
  inline void size(size_t s) { end_ = beg_ + s; }

  /// @brief Move the segment to a new position in the point array.
  /// @param p New absolute position for the beginning of the segment
  inline void moveTo(size_t p) {
    if (p >= beg_)
      move(static_cast<ptrdiff_t>(p - beg_));
    else
      move(-static_cast<ptrdiff_t>(beg_ - p));
  }

  /// @brief Move the segment by a relative offset.
  /// @param m Offset to move (positive moves forward, negative backward)
  inline void move(ptrdiff_t m) {
    if (m >= 0) {
      const size_t offset = static_cast<size_t>(m);
      beg_ += offset;
      end_ += offset;
    } else {
      const size_t offset = static_cast<size_t>(-m);
      beg_ -= offset;
      end_ -= offset;
    }
  }

  /// @brief Get the index of the first point considering reverse flag.
  /// @return Index of the first point (respects ES_REVERSE flag)
  inline size_t first() const { return reverse() ? end_ - 1 : beg_; }

  /// @brief Get the index of the last point considering reverse flag.
  /// @return Index of the last point (respects ES_REVERSE flag)
  inline size_t last() const { return reverse() ? beg_ : end_ - 1; }

  /// @brief Check if segment is marked as reversed.
  /// @return True if ES_REVERSE flag is set
  inline bool reverse() const { return (flags_ & ES_REVERSE) != 0; }

  /// @brief Check if segment is marked as closed.
  /// @return True if ES_CLOSED flag is set
  inline bool closed() const { return (flags_ & ES_CLOSED) != 0; }

  /// @brief Get the flags for this segment.
  /// @return Bitwise flags (ES_NONE, ES_REVERSE, ES_CLOSED)
  inline int flags() const { return flags_; }

  /// @brief Set the flags for this segment.
  /// @param f New flags value
  inline void flags(int f) { flags_ = f; }

  /// @brief Get the identifier of this segment.
  /// @return Segment ID
  inline int id() const { return id_; }

  /// @brief Set the identifier of this segment.
  /// @param i New segment ID
  inline void id(int i) { id_ = i; }
};

template <class PT>
inline typename std::vector<PT>::const_iterator begin(const EdgeSegment& seg, const std::vector<PT>& points) {
  return points.cbegin() + seg.begin();
}

template <class PT>
inline typename std::vector<PT>::const_iterator end(const EdgeSegment& seg, const std::vector<PT>& points) {
  return points.cbegin() + seg.end();
}

template <class PT>
inline typename std::vector<PT>::const_reverse_iterator rbegin(const EdgeSegment& seg, const std::vector<PT>& points) {
  return std::vector<PT>::const_reverse_iterator(points.begin() + seg.end());
}

template <class PT>
inline typename std::vector<PT>::const_reverse_iterator rend(const EdgeSegment& seg, const std::vector<PT>& points) {
  return std::vector<PT>::const_reverse_iterator(points.begin() + seg.begin());
}

template <class PT>
inline std::vector<PT> points(const EdgeSegment& seg, const std::vector<PT>& points) {
  std::vector<PT>(begin(seg, points), end(seg, points));
}

template <class PT>
inline std::vector<PT> ordered_points(const EdgeSegment& seg, const std::vector<PT>& points) {
  return seg.reverse() ? std::vector<PT>(rbegin(seg, points), rend(seg, points))
                       : std::vector<PT>(begin(seg, points), end(seg, points));
}

typedef std::vector<EdgeSegment> EdgeSegmentVector;

template <class FT, class MT>
inline FT sum(const EdgeSegment& s, const cv::Mat& mat) {
  FT m = 0;
  if (mat.isContinuous()) {
    const MT* pmat = mat.ptr<MT>();
    for_each(s.begin(), s.end(), [&](size_t idx) { m += pmat[idx]; });
  } else {
    for_each(s.begin(), s.end(), [&](size_t idx) { m += mat.at<MT>(IndexConvert<cv::Point>::toPoint(idx, mat)); });
  }
  return m;
}

template <class FT, class MT>
inline FT mean(const EdgeSegment& s, const cv::Mat& mat) {
  size_t size = s.size();
  if (size < 1) return 0;
  return sum<FT, MT>(s, mat) / size;
}

template <class MT, class PT = index_type>
class EsdBase : public ValueManager {
  EsdBase(const EsdBase&);

 public:
  typedef MT mag_type;
  typedef PT point_type;
  typedef std::vector<PT> PointVector;

  virtual ~EsdBase() {}

  template <class GRAD, class NMS>
  inline void detect(const GRAD& grad, const NMS& nms) {
    detect(nms.directionMap(), grad.magnitude(), nms.seeds());
  }

  inline void detect(const EdgeSourceI& source) { detect(source.directionMap(), source.magnitude(), source.seeds()); }

  virtual void detect(const cv::Mat& dir, const cv::Mat& mag, const IndexVector& seeds) = 0;

  const PointVector& points() const { return points_; }

  virtual const EdgeSegmentVector& segments() const { return segments_; }

  virtual std::string name() const = 0;

 protected:
  EsdBase() : points_(), segments_() {}

  PointVector points_;
  mutable EdgeSegmentVector segments_;
};

template <class MT, class PT = index_type>
class EsdBasePattern : public EsdBase<MT, PT> {
  EsdBasePattern(const EsdBasePattern&);

 public:
  typedef MT mag_type;
  typedef PT point_type;
  typedef std::vector<PT> PointVector;

  using EsdBase<MT, PT>::detect;
  using EsdBase<MT, PT>::points;
  using EsdBase<MT, PT>::segments;

  virtual const EdgeSegmentVector& patterns() const = 0;
  virtual const EdgeSegmentVector& patternSegments() const = 0;

 protected:
  EsdBasePattern() {}
};

template <class T>
inline void dir2Vec4(char dir, Vec2<T>& res) {
  dir2Vec4(dir, *reinterpret_cast<cv::Point_<T>*>(&res[0]));
}

template <class T>
inline Vec2<T> dir2Vec4(char dir) {
  Vec2<T> res;
  dir2Vec4(dir, res);
}

template <class T>
inline void dir2Vec4(char dir, cv::Point_<T>& res) {
  static cv::Point_<T> dirs[] = {cv::Point_<T>(1, 1), cv::Point_<T>(0, 1), cv::Point_<T>(-1, 1), cv::Point_<T>(1, 0),
                                 cv::Point_<T>(1, 1), cv::Point_<T>(0, 1), cv::Point_<T>(-1, 1)};
  static const cv::Point_<T>* pdirs = &dirs[3];
  res = pdirs[dir % 4];
}

template <class T>
inline cv::Point_<T> dir2Vec4(char dir) {
  cv::Point_<T> tmp;
  dir2Vec4(dir, tmp);
  return tmp;
}

template <class T>
inline void dir2Vec8(char dir, Vec2<T>& res) {
  dir2Vec8(dir, *reinterpret_cast<cv::Point_<T>*>(&res[0]));
}

template <class T>
inline Vec2<T> dir2Vec8(char dir) {
  Vec2<T> res;
  dir2Vec8(dir, res);
}

template <class T>
inline void dir2Vec8(char dir, cv::Point_<T>& res) {
  static cv::Point_<T> dirs[] = {
      cv::Point_<T>(1, 1),  cv::Point_<T>(0, 1),  cv::Point_<T>(-1, 1),  cv::Point_<T>(-1, 0), cv::Point_<T>(-1, -1),
      cv::Point_<T>(0, -1), cv::Point_<T>(1, -1), cv::Point_<T>(1, 0),   cv::Point_<T>(1, 1),  cv::Point_<T>(0, 1),
      cv::Point_<T>(-1, 1), cv::Point_<T>(-1, 0), cv::Point_<T>(-1, -1), cv::Point_<T>(0, -1), cv::Point_<T>(1, -1)};
  static const cv::Point_<T>* pdirs = &dirs[7];
  res = pdirs[dir % 8];
}

template <class T>
inline cv::Point_<T> dir2Vec8(char dir) {
  cv::Point_<T> tmp;
  dir2Vec8(dir, tmp);
  return tmp;
}

inline char fixDir4(char dir) {
  static char dirs[] = {1, 2, 3, 0, 1, 2, 3};
  static const char* pdirs = dirs + 3;
  return pdirs[static_cast<unsigned char>(dir) & 3];
}

inline char fixDir8(char dir) {
  static char dirs[] = {1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
  static const char* pdirs = dirs + 7;
  return pdirs[static_cast<unsigned char>(dir) & 7];
}

template <int NUM_DIR>
char fixDir(char dir);

template <>
inline char fixDir<8>(char dir) {
  return fixDir8(dir);
}

template <>
inline char fixDir<4>(char dir) {
  return fixDir4(dir);
}

inline char absDiff8(char dir) {
  static char absDiffMap[] = {1, 2, 3, 4, 3, 2, 1, 0, 1, 2, 3, 4, 3, 2, 1};
  static const char* pdiff = absDiffMap + 7;
  // dir should be in range [-7, 7], use signed arithmetic for correct indexing
  return pdiff[static_cast<int>(dir)];
}

inline char absDiff4(char dir) {
  static char absDiffMap[] = {1, 2, 1, 0, 1, 2, 1};
  static const char* pdiff = absDiffMap + 3;
  // dir should be in range [-3, 3], use signed arithmetic for correct indexing
  return pdiff[static_cast<int>(dir)];
}

template <int NUM_DIR>
char absDiff(char dir);

template <>
inline char absDiff<8>(char dir) {
  return absDiff8(dir);
}

template <>
inline char absDiff<4>(char dir) {
  return absDiff4(dir);
}

template <int NUM_DIR>
char mapDir(float dx, float dy);

template <>
inline char mapDir<8>(float xs, float ys) {
  char dmap = 0;
  // we are within +-pi/8 at x
  if (std::abs(xs) > 0.923879533f) {
    dmap = (xs > 0 ? 2 : 6);
    // we are within +-pi/8 at y
  } else if (std::abs(ys) > 0.923879533f) {
    dmap = (ys > 0 ? 4 : 0);
  } else {
    if (xs > 0)
      dmap = (ys > 0 ? 3 : 1);
    else
      dmap = (ys > 0 ? 5 : 7);
  }
  return dmap;
}

template <>
inline char mapDir<4>(float xs, float ys) {
  char dmap = 0;
  // we are within +-pi/8 at x
  if (std::abs(xs) > 0.923879533f) {
    dmap = 2;
    // we are within +-pi/8 at y
  } else if (std::abs(ys) > 0.923879533f) {
    dmap = 0;
  } else {
    dmap = (xs * ys > 0 ? 3 : 1);
  }
  return dmap;
}

}  // namespace lsfm
