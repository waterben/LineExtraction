//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
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

/// @brief Get a const iterator to the beginning of a segment's points.
/// @tparam PT Point type stored in the vector
/// @param seg Edge segment defining the range
/// @param points Vector of points indexed by the segment
/// @return Const iterator to the first point of the segment
template <class PT>
inline typename std::vector<PT>::const_iterator begin(const EdgeSegment& seg, const std::vector<PT>& points) {
  return points.cbegin() + seg.begin();
}

/// @brief Get a const iterator past the end of a segment's points.
/// @tparam PT Point type stored in the vector
/// @param seg Edge segment defining the range
/// @param points Vector of points indexed by the segment
/// @return Const iterator one past the last point of the segment
template <class PT>
inline typename std::vector<PT>::const_iterator end(const EdgeSegment& seg, const std::vector<PT>& points) {
  return points.cbegin() + seg.end();
}

/// @brief Get a const reverse iterator to the last point of a segment.
/// @tparam PT Point type stored in the vector
/// @param seg Edge segment defining the range
/// @param points Vector of points indexed by the segment
/// @return Const reverse iterator starting at the last point of the segment
template <class PT>
inline typename std::vector<PT>::const_reverse_iterator rbegin(const EdgeSegment& seg, const std::vector<PT>& points) {
  return std::vector<PT>::const_reverse_iterator(points.begin() + seg.end());
}

/// @brief Get a const reverse iterator before the first point of a segment.
/// @tparam PT Point type stored in the vector
/// @param seg Edge segment defining the range
/// @param points Vector of points indexed by the segment
/// @return Const reverse iterator ending before the first point of the segment
template <class PT>
inline typename std::vector<PT>::const_reverse_iterator rend(const EdgeSegment& seg, const std::vector<PT>& points) {
  return std::vector<PT>::const_reverse_iterator(points.begin() + seg.begin());
}

/// @brief Extract the points of a segment as a new vector.
/// @tparam PT Point type stored in the vector
/// @param seg Edge segment defining the range
/// @param points Vector of all points indexed by segments
/// @return New vector containing copies of the segment's points
template <class PT>
inline std::vector<PT> points(const EdgeSegment& seg, const std::vector<PT>& points) {
  std::vector<PT>(begin(seg, points), end(seg, points));
}

/// @brief Extract the points of a segment in logical order.
/// If the segment is marked as reversed, the points are returned in reverse order
/// so that they follow the logical edge direction.
/// @tparam PT Point type stored in the vector
/// @param seg Edge segment defining the range
/// @param points Vector of all points indexed by segments
/// @return New vector containing the segment's points in logical order
template <class PT>
inline std::vector<PT> ordered_points(const EdgeSegment& seg, const std::vector<PT>& points) {
  return seg.reverse() ? std::vector<PT>(rbegin(seg, points), rend(seg, points))
                       : std::vector<PT>(begin(seg, points), end(seg, points));
}

/// @typedef EdgeSegmentVector
/// @brief Vector of edge segments.
typedef std::vector<EdgeSegment> EdgeSegmentVector;

/// @brief Sum matrix values at all indices within an edge segment.
/// @tparam FT Floating-point accumulator type
/// @tparam MT Matrix element data type
/// @param s Edge segment defining the index range
/// @param mat Matrix to sample values from
/// @return Sum of all matrix values at the segment's indices
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

/// @brief Compute the mean of matrix values at all indices within an edge segment.
/// @tparam FT Floating-point accumulator type
/// @tparam MT Matrix element data type
/// @param s Edge segment defining the index range
/// @param mat Matrix to sample values from
/// @return Mean of all matrix values, or 0 if segment is empty
template <class FT, class MT>
inline FT mean(const EdgeSegment& s, const cv::Mat& mat) {
  size_t size = s.size();
  if (size < 1) return 0;
  return sum<FT, MT>(s, mat) / size;
}

/// @brief Base class for Edge Segment Detectors (ESD).
/// Provides a common interface for detecting edge segments from direction maps,
/// magnitude maps, and seed points. Derived classes implement specific detection strategies.
/// @tparam MT Magnitude data type (e.g., float, uchar)
/// @tparam PT Point/index type used to store detected edge points (default: index_type)
template <class MT, class PT = index_type>
class EsdBase : public ValueManager {
  EsdBase(const EsdBase&);

 public:
  /// @typedef mag_type
  /// @brief Magnitude data type
  typedef MT mag_type;

  /// @typedef point_type
  /// @brief Point/index type for detected edge points
  typedef PT point_type;

  /// @typedef PointVector
  /// @brief Vector of detected edge points
  typedef std::vector<PT> PointVector;

  virtual ~EsdBase() {}

  /// @brief Detect edge segments using gradient and NMS results.
  /// Convenience overload that extracts direction map, magnitude, and seeds from objects.
  /// @tparam GRAD Gradient computation type with magnitude() method
  /// @tparam NMS Non-maximum suppression type with directionMap() and seeds() methods
  /// @param grad Gradient computation result
  /// @param nms NMS computation result
  template <class GRAD, class NMS>
  inline void detect(const GRAD& grad, const NMS& nms) {
    detect(nms.directionMap(), grad.magnitude(), nms.seeds());
  }

  /// @brief Detect edge segments using an EdgeSourceI interface.
  /// Convenience overload that extracts data from an edge source.
  /// @param source Edge source providing direction map, magnitude, and seeds
  inline void detect(const EdgeSourceI& source) { detect(source.directionMap(), source.magnitude(), source.seeds()); }

  /// @brief Detect edge segments from direction map, magnitude, and seed indices.
  /// @param dir Direction map (CV_8S) with quantized edge directions
  /// @param mag Magnitude map with edge strength values
  /// @param seeds Vector of seed pixel indices to start tracing from
  virtual void detect(const cv::Mat& dir, const cv::Mat& mag, const IndexVector& seeds) = 0;

  /// @brief Get the detected edge points.
  /// @return Const reference to the vector of detected points
  const PointVector& points() const { return points_; }

  /// @brief Get the detected edge segments.
  /// @return Const reference to the vector of edge segments
  virtual const EdgeSegmentVector& segments() const { return segments_; }

  /// @brief Get the name of this edge segment detector.
  /// @return Identifier string for the detector
  virtual std::string name() const = 0;

 protected:
  EsdBase() : points_(), segments_() {}

  /// @brief Detected edge support points.
  PointVector points_;

  /// @brief Detected edge segments referencing points_.
  mutable EdgeSegmentVector segments_;
};

/// @brief Extended base class for pattern-based Edge Segment Detectors.
/// Adds support for structural pattern extraction alongside standard segment detection.
/// Detectors derive from this when they produce both edge segments and structural patterns.
/// @tparam MT Magnitude data type (e.g., float, uchar)
/// @tparam PT Point/index type (default: index_type)
template <class MT, class PT = index_type>
class EsdBasePattern : public EsdBase<MT, PT> {
  EsdBasePattern(const EsdBasePattern&);

 public:
  /// @typedef mag_type
  /// @brief Magnitude data type
  typedef MT mag_type;

  /// @typedef point_type
  /// @brief Point/index type
  typedef PT point_type;

  /// @typedef PointVector
  /// @brief Vector of detected points
  typedef std::vector<PT> PointVector;

  using EsdBase<MT, PT>::detect;
  using EsdBase<MT, PT>::points;
  using EsdBase<MT, PT>::segments;

  /// @brief Get the extracted primitive patterns.
  /// @return Const reference to the vector of pattern segments
  virtual const EdgeSegmentVector& patterns() const = 0;

  /// @brief Get segments composed of linked patterns.
  /// @return Const reference to the vector of pattern-based segments
  virtual const EdgeSegmentVector& patternSegments() const = 0;

 protected:
  EsdBasePattern() {}
};

/// @brief Convert a 4-direction index to a 2D direction vector (Vec2 overload).
/// @tparam T Coordinate type
/// @param dir Direction index (0-3)
/// @param res Output direction vector
template <class T>
inline void dir2Vec4(char dir, Vec2<T>& res) {
  dir2Vec4(dir, *reinterpret_cast<cv::Point_<T>*>(&res[0]));
}

/// @brief Convert a 4-direction index to a 2D direction vector.
/// @tparam T Coordinate type
/// @param dir Direction index (0-3)
/// @return Direction vector
template <class T>
inline Vec2<T> dir2Vec4(char dir) {
  Vec2<T> res;
  dir2Vec4(dir, res);
}

/// @brief Convert a 4-direction index to a cv::Point_ direction vector.
/// Maps direction indices to unit displacement vectors for 4-connected neighbors.
/// @tparam T Coordinate type
/// @param dir Direction index (0-3, with wraparound support)
/// @param res Output cv::Point_ direction vector
template <class T>
inline void dir2Vec4(char dir, cv::Point_<T>& res) {
  static cv::Point_<T> dirs[] = {cv::Point_<T>(1, 1), cv::Point_<T>(0, 1), cv::Point_<T>(-1, 1), cv::Point_<T>(1, 0),
                                 cv::Point_<T>(1, 1), cv::Point_<T>(0, 1), cv::Point_<T>(-1, 1)};
  static const cv::Point_<T>* pdirs = &dirs[3];
  res = pdirs[dir % 4];
}

/// @brief Convert a 4-direction index to a cv::Point_ direction vector (return overload).
/// @tparam T Coordinate type
/// @param dir Direction index (0-3)
/// @return Direction vector as cv::Point_
template <class T>
inline cv::Point_<T> dir2Vec4(char dir) {
  cv::Point_<T> tmp;
  dir2Vec4(dir, tmp);
  return tmp;
}

/// @brief Convert an 8-direction index to a 2D direction vector (Vec2 overload).
/// @tparam T Coordinate type
/// @param dir Direction index (0-7)
/// @param res Output direction vector
template <class T>
inline void dir2Vec8(char dir, Vec2<T>& res) {
  dir2Vec8(dir, *reinterpret_cast<cv::Point_<T>*>(&res[0]));
}

/// @brief Convert an 8-direction index to a 2D direction vector.
/// @tparam T Coordinate type
/// @param dir Direction index (0-7)
/// @return Direction vector
template <class T>
inline Vec2<T> dir2Vec8(char dir) {
  Vec2<T> res;
  dir2Vec8(dir, res);
}

/// @brief Convert an 8-direction index to a cv::Point_ direction vector.
/// Maps direction indices to unit displacement vectors for 8-connected neighbors.
/// @tparam T Coordinate type
/// @param dir Direction index (0-7, with wraparound support)
/// @param res Output cv::Point_ direction vector
template <class T>
inline void dir2Vec8(char dir, cv::Point_<T>& res) {
  static cv::Point_<T> dirs[] = {
      cv::Point_<T>(1, 1),  cv::Point_<T>(0, 1),  cv::Point_<T>(-1, 1),  cv::Point_<T>(-1, 0), cv::Point_<T>(-1, -1),
      cv::Point_<T>(0, -1), cv::Point_<T>(1, -1), cv::Point_<T>(1, 0),   cv::Point_<T>(1, 1),  cv::Point_<T>(0, 1),
      cv::Point_<T>(-1, 1), cv::Point_<T>(-1, 0), cv::Point_<T>(-1, -1), cv::Point_<T>(0, -1), cv::Point_<T>(1, -1)};
  static const cv::Point_<T>* pdirs = &dirs[7];
  res = pdirs[dir % 8];
}

/// @brief Convert an 8-direction index to cv::Point_ (return overload).
/// @tparam T Coordinate type
/// @param dir Direction index (0-7)
/// @return Direction vector as cv::Point_
template <class T>
inline cv::Point_<T> dir2Vec8(char dir) {
  cv::Point_<T> tmp;
  dir2Vec8(dir, tmp);
  return tmp;
}

/// @brief Fix a 4-direction index to valid range [0,3].
/// Handles wraparound for direction values outside the valid range.
/// @param dir Input direction index (may be out of range)
/// @return Direction index in [0,3]
inline char fixDir4(char dir) {
  static char dirs[] = {1, 2, 3, 0, 1, 2, 3};
  static const char* pdirs = dirs + 3;
  return pdirs[static_cast<unsigned char>(dir) & 3];
}

/// @brief Fix an 8-direction index to valid range [0,7].
/// Handles wraparound for direction values outside the valid range.
/// @param dir Input direction index (may be out of range)
/// @return Direction index in [0,7]
inline char fixDir8(char dir) {
  static char dirs[] = {1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
  static const char* pdirs = dirs + 7;
  return pdirs[static_cast<unsigned char>(dir) & 7];
}

/// @brief Fix a direction index to valid range for NUM_DIR directions.
/// Template dispatch to fixDir4() or fixDir8() based on NUM_DIR.
/// @tparam NUM_DIR Number of directions (4 or 8)
/// @param dir Input direction index
/// @return Direction index in valid range
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

/// @brief Compute absolute direction difference for 8-direction encoding.
/// Returns the minimum angular difference between two direction indices
/// considering the circular nature of directions.
/// @param dir Signed difference between two direction indices (range [-7, 7])
/// @return Absolute difference in [0, 4]
inline char absDiff8(char dir) {
  static char absDiffMap[] = {1, 2, 3, 4, 3, 2, 1, 0, 1, 2, 3, 4, 3, 2, 1};
  static const char* pdiff = absDiffMap + 7;
  // dir should be in range [-7, 7], use signed arithmetic for correct indexing
  return pdiff[static_cast<int>(dir)];
}

/// @brief Compute absolute direction difference for 4-direction encoding.
/// Returns the minimum angular difference between two direction indices.
/// @param dir Signed difference between two direction indices (range [-3, 3])
/// @return Absolute difference in [0, 2]
inline char absDiff4(char dir) {
  static char absDiffMap[] = {1, 2, 1, 0, 1, 2, 1};
  static const char* pdiff = absDiffMap + 3;
  // dir should be in range [-3, 3], use signed arithmetic for correct indexing
  return pdiff[static_cast<int>(dir)];
}

/// @brief Compute absolute direction difference for NUM_DIR directions.
/// Template dispatch to absDiff4() or absDiff8() based on NUM_DIR.
/// @tparam NUM_DIR Number of directions (4 or 8)
/// @param dir Signed difference between two direction indices
/// @return Absolute direction difference
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

/// @brief Map normalized gradient components to a quantized direction index.
/// Template dispatch to mapDir<4> or mapDir<8> based on NUM_DIR.
/// @tparam NUM_DIR Number of direction bins (4 or 8)
/// @param dx Normalized gradient X component
/// @param dy Normalized gradient Y component
/// @return Quantized direction index
template <int NUM_DIR>
char mapDir(float dx, float dy);

/// @brief Map normalized gradient components to an 8-direction index.
/// Quantizes gradient direction vector into one of 8 discrete directions.
/// @param xs Normalized X gradient component
/// @param ys Normalized Y gradient component
/// @return Direction index in [0, 7]
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

/// @brief Map normalized gradient components to a 4-direction index.
/// Quantizes gradient direction vector into one of 4 discrete directions.
/// @param xs Normalized X gradient component
/// @param ys Normalized Y gradient component
/// @return Direction index in [0, 3]
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
