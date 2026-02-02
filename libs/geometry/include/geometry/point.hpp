/// @file point.hpp
/// @brief Point utilities for image coordinate access and type conversions.
/// Provides helper functions for:
/// - Setting and getting pixel values at point coordinates
/// - Coordinate accessor functions (getX, getY, setX, setY)
/// - Type conversions between cv::Point and Vec2
/// - Point array/vector conversions
#pragma once

#include <geometry/base.hpp>

// #include <opencv2/core/version.hpp>
// #if CV_MAJOR_VERSION == 2
#include <opencv2/core/core.hpp>
// #elif CV_MAJOR_VERSION == 3
//  do opencv 3 code
// #endif

namespace lsfm {

/// @name Pixel value setters
/// @{

/// @brief Set pixel value at cv::Point location.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param p Point coordinates.
/// @param val Value to set.
template <class MT>
inline void set(cv::Mat& m, const cv::Point& p, MT val) {
  m.at<MT>(p) = val;
}

/// @brief Set pixel value at floating-point cv::Point location (rounds coordinates).
/// @tparam FT Point coordinate type.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param p Point coordinates.
/// @param val Value to set.
template <class FT, class MT>
inline void set(cv::Mat& m, const cv::Point_<FT>& p, MT val) {
  m.at<MT>(static_cast<int>(std::round(p.y)), static_cast<int>(std::round(p.x))) = val;
}

/// @brief Set pixel value at Vec2i location.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param v Vector coordinates (x, y).
/// @param val Value to set.
template <class MT>
inline void set(cv::Mat& m, const Vec2i& v, MT val) {
  m.at<MT>(v.y(), v.x()) = val;
}

/// @brief Set pixel value at Vec2 location (rounds coordinates).
/// @tparam FT Vector element type.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param v Vector coordinates.
/// @param val Value to set.
template <class FT, class MT>
inline void set(cv::Mat& m, const Vec2<FT>& v, MT val) {
  m.at<MT>(static_cast<int>(std::round(v.y())), static_cast<int>(std::round(v.x()))) = val;
}
/// @}

/// @name Pixel value getters (by reference)
/// @{

/// @brief Get pixel value at cv::Point location (output parameter).
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param p Point coordinates.
/// @param[out] val Retrieved value.
template <class MT>
inline void get(const cv::Mat& m, const cv::Point& p, MT& val) {
  val = m.at<MT>(p);
}

/// @brief Get pixel value at floating-point cv::Point (output parameter).
/// @tparam FT Point coordinate type.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param p Point coordinates.
/// @param[out] val Retrieved value.
template <class FT, class MT>
inline void get(const cv::Mat& m, const cv::Point_<FT>& p, MT& val) {
  val = m.at<MT>(static_cast<int>(std::round(p.y)), static_cast<int>(std::round(p.x)));
}

/// @brief Get pixel value at Vec2i location (output parameter).
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param v Vector coordinates.
/// @param[out] val Retrieved value.
template <class MT>
inline void get(const cv::Mat& m, const Vec2i& v, MT& val) {
  val = m.at<MT>(v.y(), v.x());
}

/// @brief Get pixel value at Vec2 location (output parameter).
/// @tparam FT Vector element type.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param v Vector coordinates.
/// @param[out] val Retrieved value.
template <class FT, class MT>
inline void get(const cv::Mat& m, const Vec2<FT>& v, MT& val) {
  val = m.at<MT>(static_cast<int>(std::round(v.y())), static_cast<int>(std::round(v.x())));
}
/// @}

/// @name Pixel value getters (const reference return)
/// @{

/// @brief Get pixel value at cv::Point location.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param p Point coordinates.
/// @return Const reference to pixel value.
template <class MT>
inline const MT& get(const cv::Mat& m, const cv::Point& p) {
  return m.at<MT>(p);
}

/// @brief Get pixel value at floating-point cv::Point location.
/// @tparam FT Point coordinate type.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param p Point coordinates.
/// @return Const reference to pixel value.
template <class FT, class MT>
inline const MT& get(const cv::Mat& m, const cv::Point_<FT>& p) {
  return m.at<MT>(std::round(p.y), std::round(p.x));
}

/// @brief Get pixel value at Vec2i location.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param v Vector coordinates.
/// @return Const reference to pixel value.
template <class MT>
inline const MT& get(const cv::Mat& m, const Vec2i& v) {
  return m.at<MT>(v.y(), v.x());
}

/// @brief Get pixel value at Vec2 location.
/// @tparam FT Vector element type.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param v Vector coordinates.
/// @return Const reference to pixel value.
template <class FT, class MT>
inline const MT& get(const cv::Mat& m, const Vec2<FT>& v) {
  return m.at<MT>(std::round(v.y()), std::round(v.x()));
}
/// @}

/// @name Pixel value getters (mutable reference return)
/// @{

/// @brief Get mutable reference to pixel at cv::Point location.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param p Point coordinates.
/// @return Mutable reference to pixel value.
template <class MT>
inline MT& get(cv::Mat& m, const cv::Point& p) {
  return m.at<MT>(p);
}

/// @brief Get mutable reference to pixel at floating-point cv::Point.
/// @tparam FT Point coordinate type.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param p Point coordinates.
/// @return Mutable reference to pixel value.
template <class FT, class MT>
inline MT& get(cv::Mat& m, const cv::Point_<FT>& p) {
  return m.at<MT>(std::round(p.y), std::round(p.x));
}

/// @brief Get mutable reference to pixel at Vec2i location.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param v Vector coordinates.
/// @return Mutable reference to pixel value.
template <class MT>
inline MT& get(cv::Mat& m, const Vec2i& v) {
  return m.at<MT>(v.y(), v.x());
}

/// @brief Get mutable reference to pixel at Vec2 location.
/// @tparam FT Vector element type.
/// @tparam MT Matrix element type.
/// @param m OpenCV matrix.
/// @param v Vector coordinates.
/// @return Mutable reference to pixel value.
template <class FT, class MT>
inline MT& get(cv::Mat& m, const Vec2<FT>& v) {
  return m.at<MT>(std::round(v.y()), std::round(v.x()));
}
/// @}

/// @name Coordinate accessors
/// @{

/// @brief Get mutable x-coordinate from cv::Point.
/// @tparam T Coordinate type.
/// @param p Point reference.
/// @return Mutable reference to x coordinate.
template <class T>
inline T& getX(cv::Point_<T>& p) {
  return p.x;
}

/// @brief Get const x-coordinate from cv::Point.
template <class T>
inline const T& getX(const cv::Point_<T>& p) {
  return p.x;
}

/// @brief Get mutable y-coordinate from cv::Point.
template <class T>
inline T& getY(cv::Point_<T>& p) {
  return p.y;
}

/// @brief Get const y-coordinate from cv::Point.
template <class T>
inline const T& getY(const cv::Point_<T>& p) {
  return p.y;
}

/// @brief Get mutable x-coordinate from Vec2.
template <class T>
inline T& getX(Vec2<T>& p) {
  return p.x();
}

/// @brief Get const x-coordinate from Vec2.
template <class T>
inline const T& getX(const Vec2<T>& p) {
  return p.x();
}

/// @brief Get mutable y-coordinate from Vec2.
template <class T>
inline T& getY(Vec2<T>& p) {
  return p.y();
}

/// @brief Get const y-coordinate from Vec2.
template <class T>
inline const T& getY(const Vec2<T>& p) {
  return p.y();
}

/// @brief Get mutable x-coordinate (overload with unused index).
template <class T>
inline T& getX(cv::Point_<T>& p, int) {
  return p.x;
}

/// @brief Get const x-coordinate (overload with unused index).
template <class T>
inline const T& getX(const cv::Point_<T>& p, int) {
  return p.x;
}

/// @brief Get mutable y-coordinate (overload with unused index).
template <class T>
inline T& getY(cv::Point_<T>& p, int) {
  return p.y;
}

/// @brief Get const y-coordinate (overload with unused index).
template <class T>
inline const T& getY(const cv::Point_<T>& p, int) {
  return p.y;
}

/// @brief Get mutable x-coordinate from Vec2 (overload with unused index).
template <class T>
inline T& getX(Vec2<T>& p, int) {
  return p.x();
}

/// @brief Get const x-coordinate from Vec2 (overload with unused index).
template <class T>
inline const T& getX(const Vec2<T>& p, int) {
  return p.x();
}

/// @brief Get mutable y-coordinate from Vec2 (overload with unused index).
template <class T>
inline T& getY(Vec2<T>& p, int) {
  return p.y();
}

/// @brief Get const y-coordinate from Vec2 (overload with unused index).
template <class T>
inline const T& getY(const Vec2<T>& p, int) {
  return p.y();
}
/// @}


/*template<class T>
inline void setX(cv::Point_<T> &p, T val) {
    p.x = val;
}

template<class T>
inline void setX(Vec2<T> &p, T val) {
    p.x() = val;
}

template<class T>
inline void setY(cv::Point_<T> &p, T val) {
    p.y = val;
}

template<class T>
inline void setY(Vec2<T> &p, T val) {
    p.y() = val;
}*/

/// @name Coordinate setters
/// @{

/// @brief Set x-coordinate of cv::Point with type conversion.
/// @tparam T Point coordinate type.
/// @tparam V Value type.
/// @param p Point to modify.
/// @param val Value to set.
template <class T, class V>
inline void setX(cv::Point_<T>& p, V val) {
  p.x = static_cast<T>(val);
}

/// @brief Set x-coordinate of Vec2 with type conversion.
/// @tparam T Vector element type.
/// @tparam V Value type.
/// @param p Vector to modify.
/// @param val Value to set.
template <class T, class V>
inline void setX(Vec2<T>& p, V val) {
  p.x() = static_cast<T>(val);
}

/// @brief Set y-coordinate of cv::Point with type conversion.
/// @tparam T Point coordinate type.
/// @tparam V Value type.
/// @param p Point to modify.
/// @param val Value to set.
template <class T, class V>
inline void setY(cv::Point_<T>& p, V val) {
  p.y = static_cast<T>(val);
}

/// @brief Set y-coordinate of Vec2 with type conversion.
/// @tparam T Vector element type.
/// @tparam V Value type.
/// @param p Vector to modify.
/// @param val Value to set.
template <class T, class V>
inline void setY(Vec2<T>& p, V val) {
  p.y() = static_cast<T>(val);
}

/*template<class T, template<class> class PT>
inline void set(PT<T> &p, T x, T y) {
    setX(p,x);
    setY(p,y);
}*/

/// @brief Set both coordinates of a point.
/// @tparam PT Point type.
/// @tparam V Value type.
/// @param p Point to modify.
/// @param x X-coordinate value.
/// @param y Y-coordinate value.
template <class PT, class V>
inline void set(PT& p, V x, V y) {
  setX(p, x);
  setY(p, y);
}
/// @}

/// @name Type conversions
/// @{

/// @brief Convert cv::Point to Vec2.
/// @tparam T Coordinate type.
/// @param p Source point.
/// @return Vec2 with same coordinates.
template <class T>
inline Vec2<T> toVec2(const cv::Point_<T>& p) {
  return Vec2<T>(p.x, p.y);
}

/// @brief Convert cv::Point to Vec2 (output parameter).
/// @tparam T Coordinate type.
/// @param p Source point.
/// @param[out] v Destination vector.
template <class T>
inline void toVec2(const cv::Point_<T>& p, Vec2<T>& v) {
  v.x() = p.x;
  v.y() = p.y;
}

/// @brief Convert Vec2 to cv::Point.
/// @tparam T Coordinate type.
/// @param v Source vector.
/// @return cv::Point with same coordinates.
template <class T>
inline cv::Point_<T> toPoint(const Vec2<T>& v) {
  return cv::Point_<T>(v.x(), v.y());
}

/// @brief Convert Vec2 to cv::Point (output parameter).
/// @tparam T Coordinate type.
/// @param v Source vector.
/// @param[out] p Destination point.
template <class T>
inline void toPoint(const Vec2<T>& v, cv::Point_<T>& p) {
  p.x = v.x();
  p.y = v.y();
}
/// @}

/// @name Norm calculations
/// @{

/// @brief Calculate squared Euclidean norm of cv::Point.
/// @tparam T Coordinate type.
/// @param p Point.
/// @return Squared norm (x² + y²).
template <class T>
inline T squaredNorm(cv::Point_<T>& p) {
  return p.x * p.x + p.y * p.y;
}

/// @brief Calculate squared Euclidean norm of Vec2.
/// @tparam T Coordinate type.
/// @param p Vector.
/// @return Squared norm.
template <class T>
inline T squaredNorm(Vec2<T>& p) {
  return p.squaredNorm();
}

/// @brief Calculate L2 norm (Euclidean length) of a point.
/// @tparam FT Return type (floating point).
/// @tparam PT Point type.
/// @param p Point.
/// @return Euclidean norm sqrt(x² + y²).
template <class FT, class PT>
inline FT l2_norm(const PT& p) {
  return std::hypot(static_cast<FT>(getX(p)), static_cast<FT>(getY(p)));
}
/// @}

/// @name Point copying
/// @{

/// @brief Copy point (same type).
template <class A, template <class> class APT>
inline void set(APT<A>& outPt, const APT<A>& inPt) {
  outPt = inPt;
}

/// @brief Copy point with type conversion.
template <class A, template <class> class APT, class B, template <class> class BPT>
inline void set(APT<A>& outPt, const BPT<B>& inPt) {
  setX(outPt, getX(inPt));
  setY(outPt, getY(inPt));
}
/// @}

/*template<class T>
inline void set(T& outPt, const T& inPt) {
    outPt = inPt;
}

template<class A, class B>
inline void set(cv::Point_<A>& outPt, const cv::Point_<B>& inPt) {
    setX(outPt, getX(inPt));
    setY(outPt, getY(inPt));
}

template<class A, class B>
inline void set(Vec2<A>& outPt, const cv::Point_<B>& inPt) {
    setX(outPt, getX(inPt));
    setY(outPt, getY(inPt));
}

template<class A, class B>
inline void set(Vec2<A>& outPt, const Vec2<B>& inPt) {
    setX(outPt, getX(inPt));
    setY(outPt, getY(inPt));
}

template<class A, class B>
inline void set(cv::Point_<A>& outPt, const Vec2<B>& inPt) {
    setX(outPt, getX(inPt));
    setY(outPt, getY(inPt));
}*/

/// @name Point array/vector conversions
/// @{

/// @brief Convert array of points between types.
/// @tparam A Input point element type.
/// @tparam APT Input point template.
/// @tparam B Output point element type.
/// @tparam BPT Output point template.
/// @param inBeg Pointer to first input point.
/// @param inEnd Pointer past last input point.
/// @param outBeg Pointer to first output point.
template <class A, template <class> class APT, class B, template <class> class BPT>
inline void pointConvert(const APT<A>* inBeg, const APT<A>* inEnd, BPT<B>* outBeg) {
  for_each(inBeg, inEnd, [&](const A& p) { set(*outBeg, p); });
}

/*template<class T>
inline void pointConvert(const T* inBeg, const T* inEnd, T* outBeg) {
    for_each(inBeg, inEnd,[&](const T& p) {
       set(*outBeg,p);
    });
}

template<class A, class B>
inline void pointConvert(const cv::Point_<A>* inBeg, const cv::Point_<A>* inEnd, cv::Point_<B>* outBeg) {
    for_each(inBeg, inEnd,[&](const A& p) {
       set(*outBeg,p);
    });
}

template<class A, class B>
inline void pointConvert(const Vec2<A>* inBeg, const Vec2<A>* inEnd, cv::Point_<B>* outBeg) {
    for_each(inBeg, inEnd,[&](const A& p) {
       set(*outBeg,p);
    });
}

template<class A, class B>
inline void pointConvert(const Vec2<A>* inBeg, const Vec2<A>* inEnd, Vec2<B>* outBeg) {
    for_each(inBeg, inEnd,[&](const A& p) {
       set(*outBeg,p);
    });
}

template<class A, class B>
inline void pointConvert(const cv::Point_<A>* inBeg, const cv::Point_<A>* inEnd, Vec2<B>* outBeg) {
    for_each(inBeg, inEnd,[&](const A& p) {
       set(*outBeg,p);
    });
}*/

/// @brief Convert point vector (same type, just copies).
template <class A, template <class> class APT>
static inline void pointConvert(const std::vector<APT<A>>& idxs, std::vector<APT<A>>& res) {
  res = idxs;
}

/// @brief Convert point vector between different types.
/// @tparam A Input point element type.
/// @tparam APT Input point template.
/// @tparam B Output point element type.
/// @tparam BPT Output point template.
/// @param idxs Input vector.
/// @param[out] res Output vector (resized automatically).
template <class A, template <class> class APT, class B, template <class> class BPT>
static inline void pointConvert(const std::vector<APT<A>>& idxs, std::vector<BPT<B>>& res) {
  if (idxs.empty()) return;
  res.resize(idxs.size());
  pointConvert(&idxs[0], &idxs[0] + idxs.size(), &res[0]);
}

/*template<class A>
static inline void pointConvert(const std::vector<cv::Point_<A>> &idxs, std::vector<cv::Point_<A>> &res) {
    res = idxs;
}

template<class A>
static inline void pointConvert(const std::vector<Vec2<A>> &idxs, std::vector<Vec2<A>> &res) {
    res = idxs;
}

template<class A, class B>
static inline void pointConvert(const std::vector<cv::Point_<A>> &idxs, std::vector<cv::Point_<B>> &res) {
    if (idxs.empty())
        return;
    res.resize(idxs.size());
    pointConvert(&idxs[0], &idxs[0] + idxs.size(), &res[0]);
}

template<class A, class B>
static inline void pointConvert(const std::vector<Vec2<A>> &idxs, std::vector<cv::Point_<B>> &res) {
    if (idxs.empty())
        return;
    res.resize(idxs.size());
    pointConvert(&idxs[0], &idxs[0] + idxs.size(), &res[0]);
}

template<class A, class B>
static inline void pointConvert(const std::vector<Vec2<A>> &idxs, std::vector<Vec2<B>> &res) {
    if (idxs.empty())
        return;
    res.resize(idxs.size());
    pointConvert(&idxs[0], &idxs[0] + idxs.size(), &res[0]);
}

template<class A, class B>
static inline void pointConvert(const std::vector<cv::Point_<A>> &idxs, std::vector<Vec2<B>> &res) {
    if (idxs.empty())
        return;
    res.resize(idxs.size());
    pointConvert(&idxs[0], &idxs[0] + idxs.size(), &res[0]);
}*/


/// @brief Convert vector (same type, just copies).
template <class VT>
static inline void pointConvert(const VT& idxs, VT& res) {
  res = idxs;
}

/// @brief Generic vector conversion with automatic type deduction.
/// @tparam IVT Input vector type.
/// @tparam VT Output vector type.
/// @param idxs Input vector.
/// @param[out] res Output vector (resized automatically).
template <class IVT, class VT>
static inline void pointConvert(const IVT& idxs, VT& res) {
  if (idxs.empty()) return;
  res.resize(idxs.size());
  pointConvert(&idxs[0], &idxs[0] + idxs.size(), &res[0]);
}
/// @}

}  // namespace lsfm
