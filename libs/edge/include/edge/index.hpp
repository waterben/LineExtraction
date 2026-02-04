/// @file index.hpp
/// @brief Index and coordinate conversion utilities for edge detection.
/// Provides utilities for converting between linear indices, 2D points, and coordinates
/// in images, along with helper functions for accessing matrix elements by index.

#pragma once

#include <geometry/point.hpp>

namespace lsfm {

/// @typedef index_type
/// @brief Linear index type for image pixels
typedef size_t index_type;

/// @typedef IndexVector
/// @brief Vector of linear indices
typedef std::vector<index_type> IndexVector;

/// @brief Check if two values have opposite signs.
/// @tparam T Numeric type
/// @param a First value
/// @param b Second value
/// @return True if values have opposite signs (a*b < 0)
template <class T>
inline bool neg_sign(T a, T b) {
  return (a ^ b) < 0;
}

/// @brief Check if two floats have opposite signs using bitwise operations.
/// @param a First float value
/// @param b Second float value
/// @return True if values have opposite signs
inline bool neg_sign(float a, float b) { return std::signbit(a) ^ std::signbit(b); }

/// @brief Check if two doubles have opposite signs using bitwise operations.
/// @param a First double value
/// @param b Second double value
/// @return True if values have opposite signs
inline bool neg_sign(double a, double b) { return std::signbit(a) ^ std::signbit(b); }

/// @brief Set a matrix element by linear index.
/// @tparam MT Matrix element data type
/// @param m The matrix to modify
/// @param i Linear index into flattened matrix
/// @param val Value to set
template <class MT>
inline void set(cv::Mat& m, index_type i, MT val) {
  m.ptr<MT>()[i] = val;
}

/// @brief Get a matrix element by linear index.
/// @tparam MT Matrix element data type
/// @param m The matrix to read from
/// @param i Linear index into flattened matrix
/// @param val Output reference to store the value
template <class MT>
inline void get(const cv::Mat& m, index_type i, MT& val) {
  val = m.ptr<MT>()[i];
}

/// @brief Get a matrix element by linear index with direct return.
/// @tparam MT Matrix element data type
/// @param m The matrix to read from
/// @param i Linear index into flattened matrix
/// @return The value at the linear index
template <class MT>
inline MT get(const cv::Mat& m, index_type i) {
  return m.ptr<MT>()[i];
}

/// @brief Get X coordinate from linear index.
/// @tparam T Output coordinate type
/// @param idx Linear index in image
/// @param cols Number of columns in image
/// @return X coordinate (column index)
template <class T>
inline T getX(index_type idx, int cols) {
  return static_cast<T>(idx % static_cast<index_type>(cols));
}

/// @brief Get Y coordinate from linear index.
/// @tparam T Output coordinate type
/// @param idx Linear index in image
/// @param cols Number of columns in image
/// @return Y coordinate (row index)
template <class T>
inline T getY(index_type idx, int cols) {
  return static_cast<T>(idx / static_cast<index_type>(cols));
}

/// @brief Convert linear index to 2D point.
/// @tparam PT Point type (cv::Point, Vec2, etc.)
/// @param idx Linear index in image
/// @param p Output point with (x,y) = (col, row)
/// @param cols Number of columns in image
template <class PT>
inline void index2Point(index_type idx, PT& p, int cols) {
  div_t r = std::div(static_cast<int>(idx), cols);
  set(p, r.rem, r.quot);
  // set(p, static_cast<int>(idx % cols), static_cast<int>(idx / cols));
}

/// @brief Batch convert linear indices to 2D points.
/// @tparam PT Point type
/// @param beg Beginning iterator of indices
/// @param end Ending iterator of indices
/// @param data Output point array
/// @param cols Number of columns in image
template <class PT>
inline void index2Point(const index_type* beg, const index_type* end, PT* data, int cols) {
  for (; beg != end; ++beg, ++data) {
    index2Point(*beg, *data, cols);
  }
}

/// @brief Convert cv::Point to linear index.
/// @param p The 2D point (x,y)
/// @param idx Output linear index
/// @param cols Number of columns in image
inline void point2Index(const cv::Point& p, index_type& idx, int cols) {
  idx = static_cast<index_type>(p.y) * static_cast<index_type>(cols) + static_cast<index_type>(p.x);
}

/// @brief Convert Vec2i to linear index.
/// @param p The 2D point vector
/// @param idx Output linear index
/// @param cols Number of columns in image
inline void point2Index(const Vec2i& p, index_type& idx, int cols) {
  idx = static_cast<index_type>(p.y()) * static_cast<index_type>(cols) + static_cast<index_type>(p.x());
}

/// @brief Convert generic 2D point to linear index with rounding.
/// @tparam PT Generic point type with x() and y() accessors
/// @param p The 2D point
/// @param idx Output linear index
/// @param cols Number of columns in image
template <class PT>
inline void point2Index(const PT& p, index_type& idx, int cols) {
  idx = std::round(getY(p)) * cols + std::round(getX(p));
}

/// @brief Convert cv::Point to linear index with direct return.
/// @param p The 2D point (x,y)
/// @param cols Number of columns in image
/// @return Linear index
inline index_type point2Index(const cv::Point& p, int cols) {
  return static_cast<index_type>(p.y) * static_cast<index_type>(cols) + static_cast<index_type>(p.x);
}

inline index_type point2Index(const Vec2i& p, int cols) {
  return static_cast<index_type>(p.y()) * static_cast<index_type>(cols) + static_cast<index_type>(p.x());
}

template <class PT>
inline index_type point2Index(const PT& p, int cols) {
  return static_cast<index_type>(round(getY(p))) * static_cast<index_type>(cols) +
         static_cast<index_type>(round(getX(p)));
}

template <class PT>
inline void point2Index(const PT* beg, const PT* end, index_type* data, int cols) {
  for (; beg != end; ++beg, ++data) {
    point2Index(*beg, *data, cols);
  }
}

template <class PT>
struct IndexConvert {
  static inline void toPoint(index_type idx, PT& p, int cols) { index2Point(idx, p, cols); }

  static inline void toPoint(index_type idx, PT& p, const cv::Mat& tmp) { index2Point(idx, p, tmp.cols); }

  template <class COL>
  static inline PT toPoint(index_type idx, const COL& cols) {
    PT ret;
    toPoint(idx, ret, cols);
    return ret;
  }

  template <class COL>
  static inline void toPoint(const index_type* beg, const index_type* end, PT* data, const COL& cols) {
    for (; beg != end; ++beg, ++data) toPoint(*beg, *data, cols);
  }

  template <class ITERA, class ITERB, class COL>
  static inline void toPoint(ITERA beg, ITERA end, ITERB data, const COL& cols) {
    for (; beg != end; ++beg, ++data) toPoint(*beg, *data, cols);
  }

  template <class IVT, class VT, class COL>
  static inline VT toPointV(const IVT& idx, const COL& cols) {
    VT ret;
    if (idx.empty()) return ret;
    size_t s = idx.size();
    ret.resize(s);
    toPoint(&idx[0], &idx[0] + s, &ret[0], cols);
    return ret;
  }

  template <class IVT, class VT, class COL>
  static inline void toPointV(const IVT& idx, VT& ret, const COL& cols) {
    if (idx.empty()) return;
    size_t s = idx.size();
    ret.resize(s);
    toPoint(&idx[0], &idx[0] + s, &ret[0], cols);
  }

  template <class ITER, class VT, class COL>
  static inline VT toPointI(ITER beg, ITER end, const COL& cols) {
    VT ret;
    size_t s = std::distance(beg, end);
    ret.reserve(s);
    toPoint(beg, end, ret.begin(), cols);
    return ret;
  }

  template <class ITER, class VT, class COL>
  static inline void toPointI(ITER beg, ITER& end, VT& ret, const COL& cols) {
    size_t s = std::distance(beg, end);
    ret.reserve(s);
    toPoint(beg, end, ret.begin(), cols);
    return ret;
  }

  static inline void toIndex(const PT& p, index_type& idx, int cols) { point2Index(p, idx, cols); }

  static inline index_type toIndex(const PT& p, int cols) { return point2Index(p, cols); }

  static inline void toIndex(const PT& p, index_type& idx, const cv::Mat& tmp) { point2Index(p, idx, tmp.cols); }

  static inline index_type toIndex(const PT& p, const cv::Mat& tmp) { return point2Index(p, tmp.cols); }

  template <class COL>
  static inline void toIndex(const PT* beg, const PT* end, index_type* data, const COL& cols) {
    for (; beg != end; ++beg, ++data) toIndex(*beg, *data, cols);
  }

  template <class ITERA, class ITERB, class COL>
  static inline void toIndex(ITERA beg, ITERA end, ITERB data, const COL& cols) {
    for (; beg != end; ++beg, ++data) toIndex(*beg, *data, cols);
  }

  template <class IVT, class VT, class COL>
  static inline VT toIndexV(const IVT& p, const COL& cols) {
    VT ret;
    if (p.empty()) return ret;
    size_t s = p.size();
    ret.resize(s);
    toIndex(&p[0], &p[0] + s, &ret[0], cols);
    return ret;
  }

  template <class IVT, class VT, class COL>
  static inline void toIndexV(const IVT& p, VT& ret, const COL& cols) {
    if (p.empty()) return;
    size_t s = p.size();
    ret.resize(s);
    toIndex(&p[0], &p[0] + s, &ret[0], cols);
  }

  template <class ITER, class VT, class COL>
  static inline VT toIndexI(ITER beg, ITER end, const COL& cols) {
    VT ret;
    size_t s = std::distance(beg, end);
    ret.reserve(s);
    toIndex(beg, end, ret.begin(), cols);
    return ret;
  }

  template <class ITER, class VT, class COL>
  static inline void toIndexI(ITER beg, ITER end, VT& ret, const COL& cols) {
    size_t s = std::distance(beg, end);
    ret.reserve(s);
    toIndex(beg, end, ret.begin(), cols);
    return ret;
  }

  static inline void convert(
      const index_type* beg, const index_type* end, PT* data, const cv::Mat& mag, const cv::Mat& = cv::Mat()) {
    for (; beg != end; ++beg, ++data) toPoint(*beg, *data, mag);
  }

  static inline void convertDir(
      const index_type* beg, const index_type* end, PT* data, const cv::Mat& mag, const cv::Mat& dir = cv::Mat()) {
    convert(beg, end, data, mag, dir);
  }
};

// specialization for index to index
template <>
struct IndexConvert<index_type> {
  static inline void toPoint(index_type idx, index_type& p, int /*cols*/) { p = idx; }

  static inline void toPoint(index_type idx, index_type& p, const cv::Mat& /*tmp*/) { p = idx; }

  template <class COL>
  static inline index_type toPoint(index_type idx, const COL& cols) {
    return idx;
  }

  template <class COL>
  static inline void toPoint(const index_type* beg, const index_type* end, index_type* data, const COL& cols) {
    for (; beg != end; ++beg, ++data) toPoint(*beg, *data, cols);
  }

  template <class ITERA, class ITERB, class COL>
  static inline void toPoint(ITERA beg, ITERA end, ITERB data, const COL& cols) {
    for (; beg != end; ++beg, ++data) toPoint(*beg, *data, cols);
  }

  template <class IVT, class VT, class COL>
  static inline VT toPointV(const IVT& idx, const COL& cols) {
    VT ret;
    if (idx.empty()) return ret;
    size_t s = idx.size();
    ret.resize(s);
    toPoint(&idx[0], &idx[0] + s, &ret[0], cols);
    return ret;
  }

  template <class IVT, class VT, class COL>
  static inline void toPointV(const IVT& idx, VT& ret, const COL& cols) {
    if (idx.empty()) return;
    size_t s = idx.size();
    ret.resize(s);
    toPoint(&idx[0], &idx[0] + s, &ret[0], cols);
  }

  template <class ITER, class VT, class COL>
  static inline VT toPointI(ITER beg, ITER end, const COL& cols) {
    VT ret;
    size_t s = std::distance(beg, end);
    ret.reserve(s);
    toPoint(beg, end, ret.begin(), cols);
    return ret;
  }

  template <class ITER, class VT, class COL>
  static inline void toPointI(ITER beg, ITER end, VT& ret, const COL& cols) {
    size_t s = std::distance(beg, end);
    ret.reserve(s);
    toPoint(beg, end, ret.begin(), cols);
    return ret;
  }

  static inline void toIndex(const index_type& p, index_type& idx, int /*cols*/) { idx = p; }

  static inline index_type toIndex(const index_type& p, int /*cols*/) { return p; }

  static inline void toIndex(const index_type& p, index_type& idx, const cv::Mat& /*tmp*/) { idx = p; }

  static inline index_type toIndex(const index_type& p, const cv::Mat& /*tmp*/) { return p; }

  template <class COL>
  static inline void toIndex(const index_type* beg, const index_type* end, index_type* data, const COL& cols) {
    for (; beg != end; ++beg, ++data) toIndex(*beg, *data, cols);
  }

  template <class ITERA, class ITERB, class COL>
  static inline void toIndex(ITERA beg, ITERA end, ITERB data, const COL& cols) {
    for (; beg != end; ++beg, ++data) toIndex(*beg, *data, cols);
  }

  template <class IVT, class VT, class COL>
  static inline VT toIndexV(const IVT& p, const COL& cols) {
    VT ret;
    if (p.empty()) return ret;
    size_t s = p.size();
    ret.resize(s);
    toIndex(&p[0], &p[0] + s, &ret[0], cols);
    return ret;
  }

  template <class IVT, class VT, class COL>
  static inline void toIndexV(const IVT& p, VT& ret, const COL& cols) {
    if (p.empty()) return;
    size_t s = p.size();
    ret.resize(s);
    toVec(&p[0], &p[0] + s, &ret[0], cols);
  }

  template <class ITER, class VT, class COL>
  static inline VT toIndexI(ITER beg, ITER end, const COL& cols) {
    VT ret;
    size_t s = std::distance(beg, end);
    ret.reserve(s);
    toIndex(beg, end, ret.begin(), cols);
    return ret;
  }

  template <class ITER, class VT, class COL>
  static inline void toIndexI(ITER beg, ITER end, VT& ret, const COL& cols) {
    size_t s = std::distance(beg, end);
    ret.reserve(s);
    toIndex(beg, end, ret.begin(), cols);
    return ret;
  }

  static inline void convert(const index_type* beg,
                             const index_type* end,
                             index_type* data,
                             const cv::Mat& mag,
                             const cv::Mat& /*dmap*/ = cv::Mat()) {
    for (; beg != end; ++beg, ++data) toPoint(*beg, *data, mag);
  }

  static inline void convertDir(const index_type* beg,
                                const index_type* end,
                                index_type* data,
                                const cv::Mat& mag,
                                const cv::Mat& dir = cv::Mat()) {
    convert(beg, end, data, mag, dir);
  }
};


template <class T>
inline Vec2<T>& toVec2(Vec2<T>& v) {
  return v;
}

template <class T>
inline const Vec2<T>& toVec2(const Vec2<T>& v) {
  return v;
}

template <class T>
inline void toVec2(const Vec2<T>& v, Vec2<T>& ret) {
  ret = v;
}

template <class T>
inline bool is_same(const T&, const T&) {
  return true;
}

template <class A, class B>
inline bool is_same(const A&, const B&) {
  return false;
}

}  // namespace lsfm
