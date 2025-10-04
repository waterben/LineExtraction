
#pragma once

#include <geometry/point.hpp>

namespace lsfm {

typedef size_t index_type;
typedef std::vector<index_type> IndexVector;

template <class T>
inline bool neg_sign(T a, T b) {
  return (a ^ b) < 0;
}

inline bool neg_sign(float a, float b) { return std::signbit(a) ^ std::signbit(b); }

inline bool neg_sign(double a, double b) { return std::signbit(a) ^ std::signbit(b); }

template <class MT>
inline void set(cv::Mat& m, index_type i, MT val) {
  m.ptr<MT>()[i] = val;
}

template <class MT>
inline void get(const cv::Mat& m, index_type i, MT& val) {
  val = m.ptr<MT>()[i];
}

template <class MT>
inline MT get(const cv::Mat& m, index_type i) {
  return m.ptr<MT>()[i];
}

template <class T>
inline T getX(index_type idx, int cols) {
  return idx % cols;
}

template <class T>
inline T getY(index_type idx, int cols) {
  return idx / cols;
}

template <class PT>
inline void index2Point(index_type idx, PT& p, int cols) {
  div_t r = std::div(static_cast<int>(idx), cols);
  set(p, r.rem, r.quot);
  // set(p, static_cast<int>(idx % cols), static_cast<int>(idx / cols));
}

template <class PT>
inline void index2Point(const index_type* beg, const index_type* end, PT* data, int cols) {
  for (; beg != end; ++beg, ++data) {
    index2Point(*beg, *data, cols);
  }
}

inline void point2Index(const cv::Point& p, index_type& idx, int cols) { idx = p.y * cols + p.x; }

inline void point2Index(const Vec2i& p, index_type& idx, int cols) { idx = p.y() * cols + p.x(); }

template <class PT>
inline void point2Index(const PT& p, index_type& idx, int cols) {
  idx = std::round(getY(p)) * cols + std::round(getX(p));
}


inline index_type point2Index(const cv::Point& p, int cols) { return p.y * cols + p.x; }

inline index_type point2Index(const Vec2i& p, int cols) { return p.y() * cols + p.x(); }

template <class PT>
inline index_type point2Index(const PT& p, int cols) {
  return std::round(getY(p)) * cols + std::round(getX(p));
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
      const index_type* beg, const index_type* end, PT* data, const cv::Mat& mag, const cv::Mat& dmap = cv::Mat()) {
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
  static inline void toPoint(index_type idx, index_type& p, int cols) { p = idx; }

  static inline void toPoint(index_type idx, index_type& p, const cv::Mat& tmp) { p = idx; }

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

  static inline void toIndex(const index_type& p, index_type& idx, int cols) { idx = p; }

  static inline index_type toIndex(const index_type& p, int cols) { return p; }

  static inline void toIndex(const index_type& p, index_type& idx, const cv::Mat& tmp) { idx = p; }

  static inline index_type toIndex(const index_type& p, const cv::Mat& tmp) { return p; }

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
                             const cv::Mat& dmap = cv::Mat()) {
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
inline const void toVec2(const Vec2<T>& v, Vec2<T>& ret) {
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
