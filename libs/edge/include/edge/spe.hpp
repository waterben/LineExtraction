#pragma once

#include <edge/edge_segment.hpp>
#include <edge/index.hpp>
#include <imgproc/interpolate.hpp>

#include <vector>

namespace lsfm {

// Basic worker, applies the pyramid fitting equation
template <class FT, class MT>
struct LinearEstimate {
  typedef MT mag_type;
  typedef FT float_type;

  static inline FT estimate(MT p, MT p_m, MT p_p) { return (p_p - p_m) / (2 * (p - std::min(p_m, p_p))); }
};

// Basic worker, applies the parable fitting equation
template <class FT, class MT>
struct QuadraticEstimate {
  typedef MT mag_type;
  typedef FT float_type;

  static inline FT estimate(MT p, MT p_m, MT p_p) { return (p_p - p_m) / (4 * p - 2 * (p_p + p_m)); }
};

// Basic worker, applies the center of gravity fitting equation
template <class FT, class MT>
struct CoGEstimate {
  typedef MT mag_type;
  typedef FT float_type;

  static inline FT estimate(MT p, MT p_m, MT p_p) { return (p_p - p_m) / (p + p_p + p_m - 3 * std::min(p_m, p_p)); }
};

// Basic worker, applies the sobel fitting equation
template <class FT, class MT>
struct SobelEstimate {
  typedef MT mag_type;
  typedef FT float_type;

  static inline FT estimate(MT p, MT p_m, MT p_p) { return (p_p - p_m) / (2 * p); }
};

// Basic worker, applies the sobel fitting equation for zero crossing
template <class FT, class LT>
struct SobelZCEstimate {
  typedef LT laplace_type;
  typedef FT float_type;

  static inline FT estimate(LT p, LT p_m, LT p_p) {
    return static_cast<FT>(p) / (neg_sign(p, p_p) ? (p - p_p) : (p_m - p));
  }
};

template <class FT,
          class MT,
          template <class> class PT = Vec2,
          template <class, class> class Estimator = QuadraticEstimate,
          template <class, class> class Interpolator = LinearInterpolator>
struct SubPixelEstimator {
  typedef MT mag_type;
  typedef FT float_type;
  typedef PT<int> point_type_int;
  typedef PT<FT> point_type_float;
  typedef PT<int> Pointi;
  typedef PT<FT> Pointf;
  typedef Estimator<FT, MT> estimator_type;
  typedef Interpolator<FT, MT> interpolator_type;

  //!
  // @param p	coordinates of central point
  // @param dir relative coordinates to -+
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(const Pointi& p, const Pointi& dir, const cv::Mat& mag, Pointf& res) {
    Pointf dirf(getX(dir), getY(dir)), pf(getX(p), getY(p));
    res = pf + dirf * estimate(get<MT>(mag, p), get<MT>(mag, p - dir), get<MT>(mag, p + dir));
  }


  //!
  // @param p	coordinates of central point
  // @param dir relative coordinates to -+
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(const Pointf& p, const Pointf& dir, const cv::Mat& mag, Pointf& res) {
    res = p + dir * estimate(interpolator_type::getNB(mag, p), interpolator_type::getNB(mag, p - dir),
                             interpolator_type::getNB(mag, p + dir));
  }


  //!
  // @param p	coordinates of central point
  // @param p_m coordinates of negative point
  // @param p_p coordinates of positive point
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(const Pointi& p, const Pointi& p_m, const Pointi& p_p, const cv::Mat& mag, Pointf& res) {
    Pointi dir(p_p - p);
    Pointf pf(getX(p), getY(p)), dirf(getX(dir), getY(dir));
    res = pf + dirf * estimate(get<MT>(mag, p), get<MT>(mag, p_m), get<MT>(mag, p_p));
  }


  //!
  // @param p	coordinates of central point
  // @param p_m coordinates of negative point
  // @param p_p coordinates of positive point
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(const Pointf& p, const Pointf& p_m, const Pointf& p_p, const cv::Mat& mag, Pointf& res) {
    res = p + (p_p - p) * estimate(interpolator_type::getNB(mag, p), interpolator_type::getNB(mag, p_m),
                                   interpolator_type::getNB(mag, p_p));
  }

  //!
  // @param p	index of central point
  // @param p_m index of negative point
  // @param p_p index of positive point
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(index_type p, index_type p_m, index_type p_p, const cv::Mat& mag, Pointf& res) {
    Pointf pf, p_pf;
    index2Point(p, pf, mag.cols);
    index2Point(p_p, p_pf, mag.cols);

    res = pf + (p_pf - pf) * estimate(mag.ptr<MT>()[p], mag.ptr<MT>()[p_m], mag.ptr<MT>()[p_p]);
  }


  //!
  // @param p	index of central point
  // @param dir relative coordinates to -+
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(index_type p, const Pointi& dir, const cv::Mat& mag, Pointf& res) {
    Pointi p2;
    index2Point(p, p2, mag.cols);
    Pointf pf(getX(p2), getY(p2)), dirf(getX(dir), getY(dir));

    res = pf + dirf * estimate(mag.ptr<MT>()[p], get<MT>(mag, p2 - dir), get<MT>(mag, p2 + dir));
  }


  //!
  // @param p	index of central point
  // @param dir relative coordinates to -+
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(index_type p, const Pointf& dir, const cv::Mat& mag, Pointf& res) {
    Pointf pf;
    index2Point(p, pf, mag.cols);
    res = pf + dir * estimate(mag.ptr<MT>()[p], interpolator_type::getNB(mag, pf - dir),
                              interpolator_type::getNB(mag, pf + dir));
  }


  //!
  // @param p	index of central point
  // @param dir angle in rad
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(index_type p, FT dir, const cv::Mat& mag, Pointf& res) {
    Pointf pf, dirf(std::cos(dir), std::sin(dir));
    index2Point(p, pf, mag.cols);

    res = pf + dirf * estimate(mag.ptr<MT>()[p], interpolator_type::getNB(mag, pf - dirf),
                               interpolator_type::getNB(mag, pf + dirf));
  }


  //!
  // @param p	index of central point
  // @param dir angle in rad
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(const Pointi& p, FT dir, const cv::Mat& mag, Pointf& res) {
    int w = mag.cols;

    Pointf pf(getX(p), getY(p)), dirf(std::cos(dir), std::sin(dir));

    res = pf + dirf * estimate(get<MT>(mag, p), interpolator_type::getNB(mag, pf - dirf),
                               interpolator_type::getNB(mag, pf + dirf));
  }


  //!
  // @param p	index of central point
  // @param dir angle in rad
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(const Pointf& p, FT dir, const cv::Mat& mag, Pointf& res) {
    int w = mag.cols;

    Pointf dirf(std::cos(dir), std::sin(dir));

    res = p + dirf * estimate(interpolator_type::getNB(mag, p), interpolator_type::getNB(mag, p - dirf),
                              interpolator_type::getNB(mag, p + dirf));
  }

  //!
  // @param p	index of central point
  // @param dir map of 8 orientations 0-8
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(index_type p, char dir, const cv::Mat& mag, Pointf& res) {
    Pointi p2, dir2;
    index2Point(p, p2, mag.cols);
    dir2Vec4(dir + 2, dir2);
    Pointf pf(getX(p2), getY(p2)), dirf(getX(dir2), getY(dir2));

    res = pf + dirf * estimate(mag.ptr<MT>()[p], get<MT>(mag, p2 - dir2), get<MT>(mag, p2 + dir2));
  }

  //!
  // @param p	index of central point
  // @param dir map of 8 orientations 0-8
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(const Pointi& p, char dir, const cv::Mat& mag, Pointf& res) {
    Pointi dir2;
    dir2Vec4(dir + 2, dir2);
    Pointf pf(getX(p), getY(p)), dirf(getX(dir2), getY(dir2));

    res = pf + dirf * estimate(get<MT>(mag, p), get<MT>(mag, p - dir2), get<MT>(mag, p + dir2));
  }

  //!
  // @param p	index of central point
  // @param dir map of 8 orientations 0-8
  // @param mag magnitude
  // @param res output, the shifted point
  static inline void estimate(const Pointf& p, char dir, const cv::Mat& mag, Pointf& res) {
    Pointf dirf;
    dir2Vec4(dir + 2, dirf);

    res = p + dirf * estimate(interpolator_type::getNB(mag, p), interpolator_type::getNB(mag, p - dirf),
                              interpolator_type::getNB(mag, p + dirf));
  }

  // Basic worker, applies the pyramid fitting equation
  static inline FT estimate(MT p, MT p_m, MT p_p) { return estimator_type::estimate(p, p_m, p_p); }

  static inline void convertP(
      const Pointi* beg, const Pointi* end, Pointf* data, const cv::Mat& mag, const cv::Mat& dmap) {
    char dir;
    for (; beg != end; ++beg, ++data) {
      dir = get<char>(dmap, *beg);
      if (dir < 0) {
        *data = Pointf(0, 0);
        continue;
      }
      estimate(*beg, dir, mag, *data);
    }
  }

  template <class IVT, class VT>
  static inline void convertP(const IVT& pts, VT& res, const cv::Mat& mag, const cv::Mat& dmap) {
    if (pts.empty()) return;
    res.resize(pts.size());
    convertP(&pts[0], &pts[0] + pts.size(), &res[0], mag, dmap);
  }

  template <class IVT, class VT>
  static inline VT convertP(const IVT& pts, const cv::Mat& mag, const cv::Mat& dmap) {
    VT res;
    convertP(pts, res, mag, dmap);
    return res;
  }

  static inline void convertPDir(
      const Pointi* beg, const Pointi* end, Pointf* data, const cv::Mat& mag, const cv::Mat& dir) {
    FT d;
    for (; beg != end; ++beg, ++data) {
      d = get<FT>(dir, *beg);
      estimate(*beg, d, mag, *data);
    }
  }

  template <class IVT, class VT>
  static inline void convertPDir(const IVT& pts, VT& res, const cv::Mat& mag, const cv::Mat& dir) {
    if (pts.empty()) return;
    res.resize(pts.size());
    convertPDir(&pts[0], &pts[0] + pts.size(), &res[0], mag, dir);
  }

  template <class IVT, class VT>
  static inline VT convertPDir(const IVT& pts, const cv::Mat& mag, const cv::Mat& dir) {
    VT res;
    convertPDir(pts, res, mag, dir);
    return res;
  }

  static inline void convert(
      const index_type* beg, const index_type* end, Pointf* data, const cv::Mat& mag, const cv::Mat& dmap) {
    char dir;
    for (; beg != end; ++beg, ++data) {
      dir = get<char>(dmap, *beg);
      if (dir < 0) {
        *data = Pointf(0, 0);
        continue;
      }
      estimate(*beg, dir, mag, *data);
    }
  }

  template <class IVT, class VT>
  static inline void convert(const IVT& idxs, VT& res, const cv::Mat& mag, const cv::Mat& dmap) {
    if (idxs.empty()) return;
    res.resize(idxs.size());
    convert(&idxs[0], &idxs[0] + idxs.size(), &res[0], mag, dmap);
  }

  template <class IVT, class VT>
  static inline VT convert(const IVT& idxs, const cv::Mat& mag, const cv::Mat& dmap) {
    VT res;
    convert(idxs, res, mag, dmap);
    return res;
  }

  static inline void convertDir(
      const index_type* beg, const index_type* end, Pointf* data, const cv::Mat& mag, const cv::Mat& dir) {
    FT d;
    for (; beg != end; ++beg, ++data) {
      d = get<FT>(dir, *beg);
      estimate(*beg, d, mag, *data);
    }
  }

  template <class IVT, class VT>
  static inline void convertDir(const IVT& idxs, VT& res, const cv::Mat& mag, const cv::Mat& dir) {
    if (idxs.empty()) return;
    res.resize(idxs.size());
    convertDir(&idxs[0], &idxs[0] + idxs.size(), &res[0], mag, dir);
  }

  template <class IVT, class VT>
  static inline VT convertDir(const IVT& idxs, const cv::Mat& mag, const cv::Mat& dir) {
    VT res;
    convertDir(idxs, res, mag, dir);
    return res;
  }
};

template <class FT, class PT = Vec2i, class C = IndexConvert<PT>>
struct PixelEstimator {
  typedef FT float_type;
  typedef PT point_type;
  typedef C converter_type;

  static inline void convert(
      const index_type* beg, const index_type* end, point_type* data, const cv::Mat& mag, const cv::Mat& dmap) {
    converter_type::convert(beg, end, data, mag, dmap);
  }

  template <class IVT, class VT>
  static inline void convert(const IVT& idxs, VT& res, const cv::Mat& mag, const cv::Mat& dmap) {
    if (idxs.empty()) return;
    res.resize(idxs.size());
    convert(&idxs[0], &idxs[0] + idxs.size(), &res[0], mag, dmap);
  }

  template <class IVT, class VT>
  static inline VT convert(const IVT& idxs, const cv::Mat& mag, const cv::Mat& dmap) {
    VT res;
    convert(idxs, res, mag, dmap);
    return res;
  }

  static inline void convertDir(
      const index_type* beg, const index_type* end, point_type* data, const cv::Mat& mag, const cv::Mat& dir) {
    converter_type::convertDir(beg, end, data, mag, dir);
  }

  template <class IVT, class VT>
  static inline void convertDir(const IVT& idxs, VT& res, const cv::Mat& mag, const cv::Mat& dir) {
    if (idxs.empty()) return;
    res.resize(idxs.size());
    convertDir(&idxs[0], &idxs[0] + idxs.size(), &res[0], mag, dir);
  }

  template <class IVT, class VT>
  static inline VT convertDir(const IVT& idxs, const cv::Mat& mag, const cv::Mat& dir) {
    VT res;
    convertDir(idxs, res, mag, dir);
    return res;
  }
};

}  // namespace lsfm
