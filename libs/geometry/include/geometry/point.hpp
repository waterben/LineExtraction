
#ifndef _POINT_HPP_
#define _POINT_HPP_
#ifdef __cplusplus

#  include <geometry/base.hpp>

// #include <opencv2/core/version.hpp>
// #if CV_MAJOR_VERSION == 2
#  include <opencv2/core/core.hpp>
// #elif CV_MAJOR_VERSION == 3
//  do opencv 3 code
// #endif

namespace lsfm {

template <class MT>
inline void set(cv::Mat& m, const cv::Point& p, MT val) {
  m.at<MT>(p) = val;
}

template <class FT, class MT>
inline void set(cv::Mat& m, const cv::Point_<FT>& p, MT val) {
  m.at<MT>(std::round(p.y), std::round(p.x)) = val;
}

template <class MT>
inline void set(cv::Mat& m, const Vec2i& v, MT val) {
  m.at<MT>(v.y(), v.x()) = val;
}

template <class FT, class MT>
inline void set(cv::Mat& m, const Vec2<FT>& v, MT val) {
  m.at<MT>(std::round(v.y()), std::round(v.x())) = val;
}

template <class MT>
inline void get(const cv::Mat& m, const cv::Point& p, MT& val) {
  val = m.at<MT>(p);
}

template <class FT, class MT>
inline void get(const cv::Mat& m, const cv::Point_<FT>& p, MT& val) {
  val = m.at<MT>(std::round(p.y), std::round(p.x));
}

template <class MT>
inline void get(const cv::Mat& m, const Vec2i& v, MT& val) {
  val = m.at<MT>(v.y(), v.x());
}

template <class FT, class MT>
inline void get(const cv::Mat& m, const Vec2<FT>& v, MT& val) {
  val = m.at<MT>(std::round(v.y()), std::round(v.x()));
}

template <class MT>
inline const MT& get(const cv::Mat& m, const cv::Point& p) {
  return m.at<MT>(p);
}

template <class FT, class MT>
inline const MT& get(const cv::Mat& m, const cv::Point_<FT>& p) {
  return m.at<MT>(std::round(p.y), std::round(p.x));
}

template <class MT>
inline const MT& get(const cv::Mat& m, const Vec2i& v) {
  return m.at<MT>(v.y(), v.x());
}

template <class FT, class MT>
inline const MT& get(const cv::Mat& m, const Vec2<FT>& v) {
  return m.at<MT>(std::round(v.y()), std::round(v.x()));
}

template <class MT>
inline MT& get(cv::Mat& m, const cv::Point& p) {
  return m.at<MT>(p);
}

template <class FT, class MT>
inline MT& get(cv::Mat& m, const cv::Point_<FT>& p) {
  return m.at<MT>(std::round(p.y), std::round(p.x));
}

template <class MT>
inline MT& get(cv::Mat& m, const Vec2i& v) {
  return m.at<MT>(v.y(), v.x());
}

template <class FT, class MT>
inline MT& get(cv::Mat& m, const Vec2<FT>& v) {
  return m.at<MT>(std::round(v.y()), std::round(v.x()));
}

template <class T>
inline T& getX(cv::Point_<T>& p) {
  return p.x;
}

template <class T>
inline const T& getX(const cv::Point_<T>& p) {
  return p.x;
}

template <class T>
inline T& getY(cv::Point_<T>& p) {
  return p.y;
}

template <class T>
inline const T& getY(const cv::Point_<T>& p) {
  return p.y;
}

template <class T>
inline T& getX(Vec2<T>& p) {
  return p.x();
}

template <class T>
inline const T& getX(const Vec2<T>& p) {
  return p.x();
}

template <class T>
inline T& getY(Vec2<T>& p) {
  return p.y();
}

template <class T>
inline const T& getY(const Vec2<T>& p) {
  return p.y();
}

template <class T>
inline T& getX(cv::Point_<T>& p, int) {
  return p.x;
}

template <class T>
inline const T& getX(const cv::Point_<T>& p, int) {
  return p.x;
}

template <class T>
inline T& getY(cv::Point_<T>& p, int) {
  return p.y;
}

template <class T>
inline const T& getY(const cv::Point_<T>& p, int) {
  return p.y;
}

template <class T>
inline T& getX(Vec2<T>& p, int) {
  return p.x();
}

template <class T>
inline const T& getX(const Vec2<T>& p, int) {
  return p.x();
}

template <class T>
inline T& getY(Vec2<T>& p, int) {
  return p.y();
}

template <class T>
inline const T& getY(const Vec2<T>& p, int) {
  return p.y();
}


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

template <class T, class V>
inline void setX(cv::Point_<T>& p, V val) {
  p.x = static_cast<T>(val);
}

template <class T, class V>
inline void setX(Vec2<T>& p, V val) {
  p.x() = static_cast<T>(val);
}

template <class T, class V>
inline void setY(cv::Point_<T>& p, V val) {
  p.y = static_cast<T>(val);
}

template <class T, class V>
inline void setY(Vec2<T>& p, V val) {
  p.y() = static_cast<T>(val);
}

/*template<class T, template<class> class PT>
inline void set(PT<T> &p, T x, T y) {
    setX(p,x);
    setY(p,y);
}*/

template <class PT, class V>
inline void set(PT& p, V x, V y) {
  setX(p, x);
  setY(p, y);
}

template <class T>
inline Vec2<T> toVec2(const cv::Point_<T>& p) {
  return Vec2<T>(p.x, p.y);
}

template <class T>
inline void toVec2(const cv::Point_<T>& p, Vec2<T>& v) {
  v.x() = p.x;
  v.y() = p.y;
}

template <class T>
inline cv::Point_<T> toPoint(const Vec2<T>& v) {
  return cv::Point_<T>(v.x(), v.y());
}

template <class T>
inline void toPoint(const Vec2<T>& v, cv::Point_<T>& p) {
  p.x = v.x();
  p.y = v.y();
}

template <class T>
inline T squaredNorm(cv::Point_<T>& p) {
  return p.x * p.x + p.y * p.y;
}

template <class T>
inline T squaredNorm(Vec2<T>& p) {
  return p.squaredNorm();
}

//! get l2 norm (length of x and y)
template <class FT, class PT>
inline FT l2_norm(const PT& p) {
  return std::hypot(static_cast<FT>(getX(p)), static_cast<FT>(getY(p)));
}

template <class A, template <class> class APT>
inline void set(APT<A>& outPt, const APT<A>& inPt) {
  outPt = inPt;
}

template <class A, template <class> class APT, class B, template <class> class BPT>
inline void set(APT<A>& outPt, const BPT<B>& inPt) {
  setX(outPt, getX(inPt));
  setY(outPt, getY(inPt));
}

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

template <class A, template <class> class APT>
static inline void pointConvert(const std::vector<APT<A>>& idxs, std::vector<APT<A>>& res) {
  res = idxs;
}

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


template <class VT>
static inline void pointConvert(const VT& idxs, VT& res) {
  res = idxs;
}

template <class IVT, class VT>
static inline void pointConvert(const IVT& idxs, VT& res) {
  if (idxs.empty()) return;
  res.resize(idxs.size());
  pointConvert(&idxs[0], &idxs[0] + idxs.size(), &res[0]);
}
}  // namespace lsfm

#endif
#endif
