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
// C by Benjamin Wassermann
//M*/

#pragma once

#include <edge/edge_segment.hpp>
#include <edge/edge_source.hpp>
#include <geometry/line.hpp>

#include <algorithm>
#include <map>
#include <vector>

namespace lsfm {

template <class MT, class FT, class PT, class MAP = std::map<MT, FT> >
class NfaContrast : public ValueManager {
  cv::Mat E_;
  mutable int Nl_;
  mutable MAP h_;
  FT log_e_;

  void eval(const EdgeSegment& seg, const std::vector<PT>& points, EdgeSegmentVector& v, std::vector<FT>& n) const {
    MT u = min_value(seg, points);
    FT vnfa = log_nfa(seg.size(), h_[u], Nl_);
    if (vnfa > log_e_) {
      v.push_back(seg);
      n.push_back(vnfa);
      return;
    }

    size_t lpos = seg.begin(), e = seg.end();
    while (lpos != e && get<MT>(E_, points[lpos]) == u) ++lpos;
    size_t b = lpos;
    for (; b != e; ++b) {
      if (get<MT>(E_, points[b]) == u) {
        eval(EdgeSegment(lpos, b, seg.flags() & ES_REVERSE), points, v, n);
        lpos = b + 1;
        while (lpos != e && get<MT>(E_, points[lpos]) == u) ++lpos;
        b = lpos - 1;
      }
    }

    if (b != lpos) eval(EdgeSegment(lpos, b, seg.flags() & ES_REVERSE), points, v, n);
  }

  MT min_value(const EdgeSegment& seg, const std::vector<PT>& points) const {
    MT u = std::numeric_limits<MT>::max();
    for (size_t i = seg.begin(); i != seg.end(); ++i) u = std::min(u, get<MT>(this->E_, points[i]));
    return u;
  }

  void init() {
    this->add("nfa_log_e", std::bind(&NfaContrast<MT, FT, PT, MAP>::valueLogEps, this, std::placeholders::_1),
              "Epsilon -log10(e).");
  }

 public:
  typedef std::vector<PT> PointVector;

  NfaContrast(FT log_e = 0) : E_(), Nl_(0), h_(), log_e_(log_e) { init(); }

  NfaContrast(const ValueManager::NameValueVector& options) : E_(), Nl_(0), h_(), log_e_(0) {
    init();
    this->value(options);
  }

  NfaContrast(ValueManager::InitializerList options) : E_(), Nl_(0), h_(), log_e_(0) {
    init();
    this->value(options);
  }

  NfaContrast(const cv::Mat& E, FT log_e) : E_(E), log_e_(log_e) { init(); }

  NfaContrast(const cv::Mat& E, const EdgeSegmentVector& segs, FT log_e) : E_(E), Nl_(getNl(segs)), log_e_(log_e) {
    init();
    getH(E, h_);
  }

  Value valueLogEps(const Value& e = Value::NAV()) {
    if (e.type()) log_eps(e.get<FT>());
    return log_e_;
  }

  FT log_eps() const { return log_e_; }

  void log_eps(FT log_e) { log_e_ = log_e; }

  FT epsilon() const { return std::pow(static_cast<FT>(10), -log_e_); }

  void epsilon(FT e) { log_e_ = -log10(e); }

  void update(const cv::Mat& E) { E_ = E; }

  template <class GRAD>
  void update(const GRAD& grad) {
    E_ = grad.magnitude();
  }

  void update(const EdgeSourceI& esource) { E_ = esource.magnitude(); }

  void update(const cv::Mat& E, const EdgeSegmentVector& segs) {
    E_ = E;
    Nl_ = getNl(segs);
    getH(E, h_);
  }


  inline FT eval(const EdgeSegment& seg, const PointVector& points) const {
    return log_nfa(seg.size(), h_[min_value(seg, points)], Nl_);
  }

  inline FT eval(const EdgeSegment& seg, const PointVector& points, FT) const {
    return log_nfa(seg.size(), h_[min_value(seg, points)], Nl_);
  }

  inline FT eval(const EdgeSegment& seg, const PointVector& points, const Line<FT>&) const {
    return log_nfa(seg.size(), h_[min_value(seg, points)], Nl_);
  }

  void eval(const EdgeSegmentVector& in, const PointVector& points, std::vector<FT>& n, bool update = true) const {
    if (update) {
      Nl_ = getNl(in);
      getH(E_, h_);
    }

    n.clear();
    n.reserve(in.size());

    for_each(in.begin(), in.end(),
             [&n, &points, this](const EdgeSegment& seg) { n.push_back(this->eval(seg, points)); });
  }

  template <class DT>
  inline void eval(const EsdBase<DT, PT>& in, std::vector<FT>& n, bool update = true) const {
    eval(in.segments(), in.points(), n, update);
  }

  inline void eval(const EdgeSegmentVector& in,
                   const PointVector& points,
                   std::vector<FT>&,
                   std::vector<FT>& n,
                   bool update = true) const {
    eval(in, points, n, update);
  }

  template <class DT>
  inline void eval(const EsdBase<DT, PT>& in, std::vector<FT>&, std::vector<FT>& n, bool update = true) const {
    eval(in.segments(), in.points(), n, update);
  }

  template <class LV>
  inline void evalLV(
      const EdgeSegmentVector& in, const PointVector& points, const LV&, std::vector<FT>& n, bool update = true) const {
    eval(in, points, n, update);
  }

  void eval(const EdgeSegmentVector& in,
            const PointVector& points,
            EdgeSegmentVector& out,
            std::vector<FT>& n,
            bool update = true) const {
    if (update) {
      Nl_ = getNl(in);
      getH(E_, h_);
    }

    out.clear();
    out.reserve(in.size());
    n.clear();
    n.reserve(in.size());

    for_each(in.begin(), in.end(),
             [&out, &n, &points, this](const EdgeSegment& seg) { this->eval(seg, points, out, n); });
  }

  template <class DT>
  inline void eval(const EsdBase<DT, PT>& in, EdgeSegmentVector& out, std::vector<FT>& n, bool update = true) const {
    eval(in.segments(), in.points(), out, n, update);
  }

  static inline int getNl(const EdgeSegmentVector& segs) {
    int ret = 0;
    for_each(segs.begin(), segs.end(),
             [&ret](const EdgeSegment& seg) { ret += static_cast<int>(seg.size() * (seg.size() - 1)) / 2; });
    return ret;
  }

  static inline int getC(const cv::Mat& E) {
    int C = 0;
    const MT* p = E.ptr<MT>();
    const MT* pe = p + E.rows * E.cols;
    for (; p != pe; ++p) {
      if (*p != 0) ++C;
    }
    return C;
  }

  static inline FT getH(const cv::Mat& E, int C, MT u) {
    int cE = 0;
    const MT* p = E.ptr<MT>();
    const MT* pe = p + E.rows * E.cols;
    for (; p != pe; ++p) {
      if (*p >= u) ++cE;
    }
    return static_cast<FT>(cE) / C;
  }


  static inline FT getH(const cv::Mat& E, MT u) {
    int C = 0, cE = 0;
    const MT* p = E.ptr<MT>();
    const MT* pe = p + E.rows * E.cols;
    for (; p != pe; ++p) {
      if (*p != 0) ++C;
      if (*p >= u) ++cE;
    }
    return static_cast<FT>(cE) / C;
  }

  // this will hold the resuls for all u up to max -> non sparse
  static inline void getH(const cv::Mat& E, MT max, std::vector<FT>& ret) {
    ret.clear();
    ret.resize(max, 0);

    const MT* p = E.ptr<MT>();
    const MT* pe = p + E.rows * E.cols;
    // count values in E
    for (; p != pe; ++p) {
      ++ret[*p];
    }
    // get number of non zero values in E as normalization factor
    FT n = 1 / (static_cast<FT>(E.rows * E.cols) - ret[0]);

    const FT* b = &ret[0];
    FT* e = &ret.back();
    // compute partial sums from back
    for (; e != b; --e) {
      e[-1] += *e;
    }

    // normalize
    for_each(ret.begin(), ret.end(), [&n](FT& v) { v *= n; });
  }

  // this will hold the resuls for all u up to max -> non sparse
  static inline void getH(const cv::Mat& E, std::vector<FT>& ret) {
    double vmin, vmax;
    cv::minMaxIdx(E, &vmin, &vmax);
    getH(E, static_cast<MT>(vmax), ret);
  }

  // this will hold the resuls for all u (sparse)
  static inline void getH(const cv::Mat& E, std::map<MT, FT>& ret) {
    ret.clear();

    const MT* p = E.ptr<MT>();
    const MT* pe = p + E.rows * E.cols;
    // count values in E
    for (; p != pe; ++p) {
      ++ret[*p];
    }

    FT n = 1;
    if (ret.find(0) != ret.end())
      n /= static_cast<FT>(E.rows * E.cols) - ret[0];
    else
      n /= static_cast<FT>(E.rows * E.cols);

    if (ret.size() > 1) {
      auto b = ret.rbegin(), e = ret.rend();
      auto b1 = b;
      ++b1;
      // compute partial sums from back
      for (; b1 != e; ++b, ++b1) {
        b1->second += b->second;
      }
    }

    // normalize
    for_each(ret.begin(), ret.end(), [&n](std::pair<const MT, FT>& v) { v.second *= n; });
  }


  static inline FT nfa(size_t l, FT Hu, int Nl) { return static_cast<FT>(Nl * std::pow(Hu, l)); }

  static inline FT log_nfa(size_t l, FT Hu, int Nl) { return -log10(nfa(l, Hu, Nl)); }
};

template <class GT, class FT, class PT>
class NfaBinom : public ValueManager {
  static constexpr FT RELATIVE_ERROR_FACTOR_NFA = static_cast<FT>(100);
  static constexpr FT MY_LN10 = static_cast<FT>(2.30258509299404568402);

  cv::Mat matx_, maty_;
  FT range_, logNT_, log_e_, th_, p_;

  void eval(const EdgeSegment& seg, const std::vector<FT>& dirVec, EdgeSegmentVector& v, std::vector<FT>& n) const {
    int l = static_cast<int>(seg.size() - 1);
    if (l < 1) return;
    int k;
    size_t pos;

    FT r = range_ > 0 ? range_ : 360;
    max_dist(seg, dirVec, r, pos, k);

    FT vnfa = log_nfaNT(l, k, p_, logNT_);
    if (vnfa > log_e_) {
      v.push_back(seg);
      n.push_back(vnfa);
      return;
    }

    if (pos == seg.begin() || seg.begin() + 1) {
      if (pos + 1 < seg.end() - 1) eval(EdgeSegment(pos + 1, seg.end(), seg.flags() & ES_REVERSE), dirVec, v, n);
    } else if (pos >= seg.end() - 2) {
      if (pos - 1 > seg.begin()) eval(EdgeSegment(seg.begin(), pos, seg.flags() & ES_REVERSE), dirVec, v, n);
    } else {
      if (pos - 1 > seg.begin()) eval(EdgeSegment(seg.begin(), pos, seg.flags() & ES_REVERSE), dirVec, v, n);
      if (pos + 1 < seg.end() - 1) eval(EdgeSegment(pos + 1, seg.end(), seg.flags() & ES_REVERSE), dirVec, v, n);
    }
  }

  inline void max_dist(const EdgeSegment& seg, const std::vector<FT>& dirVec, FT r, size_t& pos, int& k) const {
    pos = seg.begin();
    FT m = 0;
    k = 0;
    size_t e = seg.end();
    FT last = dirVec[pos], tmp, ad;
    for (size_t i = seg.begin() + 1; i != e; ++i) {
      tmp = dirVec[i];
      ad = angleDiff(last, tmp, r);
      if (ad < th_) ++k;
      if (ad > m) {
        pos = i;
        m = ad;
      }
      last = tmp;
    }
  }

  void init() {
    this->add("nfa_log_e", std::bind(&NfaBinom<GT, FT, PT>::valueLogEps, this, std::placeholders::_1),
              "Epsilon -log10(e).");
    this->add("nfa_precision", std::bind(&NfaBinom<GT, FT, PT>::valuePrecision, this, std::placeholders::_1),
              "Precision for nfa.");
  }

 private:
  /** Computes the natural logarithm of the absolute value of
  the gamma function of x using the Lanczos approximation.
  See http://www.rskey.org/gamma.htm
  The formula used is
  @f[
  \Gamma(x) = \frac{ \sum_{n=0}^{N} q_n x^n }{ \Pi_{n=0}^{N} (x+n) }
  (x+5.5)^{x+0.5} e^{-(x+5.5)}
  @f]
  so
  @f[
  \log\Gamma(x) = \log\left( \sum_{n=0}^{N} q_n x^n \right)
  + (x+0.5) \log(x+5.5) - (x+5.5) - \sum_{n=0}^{N} \log(x+n)
  @f]
  and
  q0 = 75122.6331530,
  q1 = 80916.6278952,
  q2 = 36308.2951477,
  q3 = 8687.24529705,
  q4 = 1168.92649479,
  q5 = 83.8676043424,
  q6 = 2.50662827511.
  */
  static FT log_gamma_lanczos(FT x) {
    static FT q[7] = {static_cast<FT>(75122.6331530), static_cast<FT>(80916.6278952), static_cast<FT>(36308.2951477),
                      static_cast<FT>(8687.24529705), static_cast<FT>(1168.92649479), static_cast<FT>(83.8676043424),
                      static_cast<FT>(2.50662827511)};
    FT a = (x + static_cast<FT>(0.5)) * std::log(x + static_cast<FT>(5.5)) - (x + static_cast<FT>(5.5));
    FT b = 0;
    int n;
    for (n = 0; n < 7; n++) {
      a -= std::log(x + static_cast<FT>(n));
      b += q[n] * std::pow(x, static_cast<FT>(n));
    }
    return a + std::log(b);
  }


  /** Computes the natural logarithm of the absolute value of
  the gamma function of x using Windschitl method.
  See http://www.rskey.org/gamma.htm
  The formula used is
  @f[
  \Gamma(x) = \sqrt{\frac{2\pi}{x}} \left( \frac{x}{e}
  \sqrt{ x\sinh(1/x) + \frac{1}{810x^6} } \right)^x
  @f]
  so
  @f[
  \log\Gamma(x) = 0.5\log(2\pi) + (x-0.5)\log(x) - x
  + 0.5x\log\left( x\sinh(1/x) + \frac{1}{810x^6} \right).
  @f]
  This formula is a good approximation when x > 15.
  */
  static inline FT log_gamma_windschitl(FT x) {
    return static_cast<FT>(0.918938533204673f) + (x - static_cast<FT>(0.5)) * std::log(x) - x +
           static_cast<FT>(0.5) * x *
               std::log(x * std::sinh(static_cast<FT>(1) / x) +
                        static_cast<FT>(1) / (static_cast<FT>(810) * std::pow(x, static_cast<FT>(6))));
  }

#define log_gamma(x) ((x) > 15.0 ? log_gamma_windschitl(x) : log_gamma_lanczos(x))
  /*
          static FT log_gamma(FT x) {
              return x > 15 ? log_gamma_windschitl(x) : log_gamma_lanczos(x);
          }
  */


 public:
  typedef std::vector<PT> PointVector;

  NfaBinom(FT log_e = 0, FT p = static_cast<FT>(1.0 / 8))
      : matx_(), maty_(), range_(0), logNT_(0), log_e_(log_e), th_(0), p_(p) {
    init();
  }

  NfaBinom(const ValueManager::NameValueVector& options)
      : matx_(), maty_(), range_(0), logNT_(0), log_e_(0), th_(0), p_(static_cast<FT>(1.0 / 8)) {
    init();
    this->value(options);
  }

  NfaBinom(ValueManager::InitializerList options)
      : matx_(), maty_(), range_(0), logNT_(0), log_e_(0), th_(0), p_(static_cast<FT>(1.0 / 8)) {
    init();
    this->value(options);
  }

  NfaBinom(const cv::Mat& dir, FT r, FT log_e = 0, FT p = static_cast<FT>(1.0 / 8))
      : matx_(dir), range_(r), logNT_(0), log_e_(log_e), th_(0), p_(p) {
    init();
    logNT_ = 2.0 * (log10(static_cast<FT>(dir.cols)) + log10(static_cast<FT>(dir.rows)));
  }

  NfaBinom(const cv::Mat& gx, const cv::Mat& gy, FT log_e = 0, FT p = static_cast<FT>(1.0 / 8))
      : matx_(gx), maty_(gy), range_(0), logNT_(0), log_e_(log_e), th_(0), p_(p) {
    init();
    logNT_ = 2.0 * (log10(static_cast<FT>(gx.cols)) + log10(static_cast<FT>(gx.rows)));
  }

  Value valueLogEps(const Value& e = Value::NAV()) {
    if (e.type()) log_eps(e.get<FT>());
    return log_e_;
  }

  FT log_eps() const { return log_e_; }

  void log_eps(FT log_e) { log_e_ = log_e; }

  FT epsilon() const { return std::pow(static_cast<FT>(10), -log_e_); }

  void epsilon(FT e) { log_e_ = -log10(e); }

  Value valuePrecision(const Value& p = Value::NAV()) {
    if (p.type()) precision(p.get<FT>());
    return p_;
  }

  FT precision() const { return p_; }

  void precision(FT p) {
    p_ = p;
    th_ = range_ > 0 ? range_ * p : 360 * p;
  }

  FT threshold() const { return th_; }

  void update(const cv::Mat& dir, FT range) {
    matx_ = dir;
    range_ = range;
    logNT_ = 2 * (log10(static_cast<FT>(dir.cols)) + log10(static_cast<FT>(dir.rows)));
    th_ = range * p_;
  }

  void update(const cv::Mat& gx, const cv::Mat& gy) {
    matx_ = gx;
    maty_ = gy;
    range_ = 0;
    logNT_ = 2 * (log10(static_cast<FT>(gx.cols)) + log10(static_cast<FT>(gx.rows)));
    th_ = 360 * p_;
  }

  template <class GRAD>
  void update(const GRAD& grad) {
    update(grad.gx(), grad.gy());
  }

  void update(EdgeSourceI& esource) { update(esource.gx(), esource.gy()); }


  inline FT eval(const EdgeSegment& seg, const PointVector& points) const {
    return range_ > 0 ? log_nfaNT(seg.size() - 1, aligned(matx_, seg, points, th_, range_), p_, logNT_)
                      : log_nfaNT(seg.size() - 1, aligned(matx_, maty_, seg, points, th_), p_, logNT_);
  }

  inline FT eval(const EdgeSegment& seg, const PointVector& points, FT dir) const {
    return range_ > 0
               ? log_nfaNT(static_cast<int>(seg.size()), aligned(matx_, seg, points, dir, th_, range_), p_, logNT_)
               : log_nfaNT(static_cast<int>(seg.size()), aligned(matx_, maty_, seg, points, dir, th_), p_, logNT_);
  }

  inline FT eval(const EdgeSegment& seg, const PointVector& points, const Line<FT>& line) const {
    return range_ > 0
               ? log_nfaNT(seg.size(), aligned(matx_, seg, points, line.gradientAnglef() / 360 * range_, th_, range_),
                           p_, logNT_)
               : log_nfaNT(seg.size(), aligned(matx_, maty_, seg, points, line.gradientAnglef(), th_), p_, logNT_);
  }

  void eval(const EdgeSegmentVector& in, const PointVector& points, std::vector<FT>& n) const {
    n.clear();
    n.reserve(in.size());

    for_each(in.begin(), in.end(),
             [&points, &n, this](const EdgeSegment& seg) { n.push_back(this->eval(seg, points)); });
  }

  template <class MT>
  inline void eval(const EsdBase<MT, PT>& in, std::vector<FT>& n) const {
    eval(in.segments(), in.points(), n);
  }

  void eval(const EdgeSegmentVector& in,
            const PointVector& points,
            const std::vector<FT>& dir,
            std::vector<FT>& n) const {
    size_t s = in.size();
    n.clear();
    n.reserve(s);

    for (size_t i = 0; i != s; ++i) {
      n.push_back(eval(in[i], points, dir[i]));
    }
  }

  template <class MT>
  inline void eval(const EsdBase<MT, PT>& in, const std::vector<FT>& dir, std::vector<FT>& n) const {
    eval(in.segments(), in.points(), dir, n);
  }

  template <class LV>
  void evalLV(const EdgeSegmentVector& in, const PointVector& points, const LV& line, std::vector<FT>& n) const {
    size_t s = in.size();
    n.clear();
    n.reserve(s);

    if (range_ > 0 && range_ != 360) {
      FT conv = range_ / 360;
      for (size_t i = 0; i != s; ++i) {
        n.push_back(eval(in[i], points, line[i].gradientAnglef() * conv));
      }
    } else {
      for (size_t i = 0; i != s; ++i) {
        n.push_back(eval(in[i], points, line[i].gradientAnglef()));
      }
    }
  }

  void eval(const EdgeSegmentVector& in, const PointVector& points, EdgeSegmentVector& out, std::vector<FT>& n) const {
    std::vector<FT> dirVec;
    createDirVec(points, dirVec);
    eval(in, dirVec, out, n);
  }

  template <class MT>
  inline void eval(const EsdBase<MT, PT>& in, EdgeSegmentVector& out, std::vector<FT>& n) const {
    eval(in.segments(), in.points(), out, n);
  }

  void eval(const EdgeSegmentVector& in,
            const std::vector<FT>& dirVec,
            EdgeSegmentVector& out,
            std::vector<FT>& n) const {
    out.clear();
    out.reserve(in.size());
    n.clear();
    n.reserve(in.size());

    for_each(in.begin(), in.end(), [&, this](const EdgeSegment& seg) { this->eval(seg, dirVec, out, n); });
  }

  void createDirVec(const PointVector& points, std::vector<FT>& dirVec) const {
    if (points.empty()) return;
    dirVec.clear();
    dirVec.reserve(points.size());

    if (range_ > 0) {
      for_each(points.begin(), points.end(),
               [&dirVec, this](const PT& p) { dirVec.push_back(get<FT>(this->matx_, p)); });
    } else {
      for_each(points.begin(), points.end(), [&dirVec, this](const PT& p) {
        dirVec.push_back(cv::fastAtan2(get<GT>(this->maty_, p), get<GT>(this->matx_, p)));
      });
    }
  }

  inline static FT angleDiff(FT a, FT b) {
    static constexpr FT pi = static_cast<FT>(CV_PI);
    static constexpr FT pi2 = static_cast<FT>(CV_PI * 2);
    FT d = std::abs(a - b);
    if (d > pi2) d -= pi2 * static_cast<int>(d / pi2);
    return d > pi ? pi2 - d : d;
  }

  inline static FT angleDiffD(FT a, FT b) {
    FT d = std::abs(a - b);
    if (d > 360) d -= static_cast<FT>(360) * static_cast<FT>(static_cast<int>(d / 360));
    return d > 180 ? 360 - d : d;
  }

  inline static FT angleDiff(FT a, FT b, FT r) {
    FT d = std::abs(a - b);
    if (d > r) d -= r * static_cast<int>(d / r);
    return d > r / 2 ? r - d : d;
  }

  inline static int aligned(const cv::Mat& dir, const EdgeSegment& seg, const PointVector& points, FT th, FT r) {
    int k = 0;
    size_t s = seg.size();
    FT last = get<FT>(dir, points[0]);
    for (size_t i = 1; i != s; ++i) {
      FT tmp = get<FT>(dir, points[i]);
      if (angleDiff(last, tmp, r) < th) ++k;
      last = tmp;
    }
    return k;
  }

  inline static int aligned(const std::vector<FT>& dirVec, const EdgeSegment& seg, FT th, FT r) {
    int k = 0;
    size_t e = seg.end();
    FT last = dirVec[seg.begin()];
    for (size_t i = seg.begin() + 1; i != e; ++i) {
      FT tmp = dirVec[i];
      if (angleDiff(last, tmp, r) < th) ++k;
      last = tmp;
    }
    return k;
  }

  inline static int aligned(
      const cv::Mat& gx, const cv::Mat& gy, const EdgeSegment& seg, const PointVector& points, FT th) {
    int k = 0;
    size_t s = seg.size();
    FT last = cv::fastAtan2(get<GT>(gy, points[0]), get<GT>(gx, points[0]));
    FT tmp;
    for (size_t i = 1; i != s; ++i) {
      tmp = cv::fastAtan2(get<GT>(gy, points[i]), get<GT>(gx, points[i]));
      if (angleDiffD(last, tmp) < th) ++k;
      last = tmp;
    }
    return k;
  }

  inline static int aligned(
      const cv::Mat& dMap, const EdgeSegment& seg, const PointVector& points, FT dir, FT th, FT r) {
    int k = 0;
    size_t e = seg.end();
    for (size_t i = seg.begin(); i != e; ++i) {
      if (angleDiff(dir, get<FT>(dMap, points[i]), r) < th) ++k;
    }
    return k;
  }

  inline static int aligned(const std::vector<FT>& dirVec, EdgeSegment& seg, FT dir, FT th, FT r) {
    int k = 0;
    size_t e = seg.end();
    for (size_t i = seg.begin(); i != e; ++i) {
      if (angleDiff(dir, dirVec[i], r) < th) ++k;
    }
    return k;
  }

  inline static int aligned(
      const cv::Mat& gx, const cv::Mat& gy, const EdgeSegment& seg, const PointVector& points, FT dir, FT th) {
    int k = 0;
    size_t e = seg.end();
    for (size_t i = seg.begin(); i != e; ++i) {
      PT p = points[i];
      if (angleDiffD(dir, cv::fastAtan2(get<GT>(gy, p), get<GT>(gx, p))) < th) ++k;
    };
    return k;
  }

  /** Compare float by relative error.
  The resulting rounding error after floating point computations
  depend on the specific operations done. The same number computed by
  different algorithms could present different rounding errors. For a
  useful comparison, an estimation of the relative rounding error
  should be considered and compared to a factor times EPS. The factor
  should be related to the accumulated rounding error in the chain of
  computation. Here, as a simplification, a fixed factor is used.
  */
  static int equal(FT a, FT b) {
    FT abs_diff, aa, bb, abs_max;
    /* trivial case */
    if (a == b) return true;
    abs_diff = std::abs(a - b);
    aa = std::abs(a);
    bb = std::abs(b);
    abs_max = aa > bb ? aa : bb;
    /* DBL_MIN is the smallest normalized number, thus, the smallest
    number whose relative error is bounded by DBL_EPSILON. For
    smaller numbers, the same quantization steps as for DBL_MIN
    are used. Then, for smaller numbers, a meaningful "relative"
    error should be computed by dividing the difference by DBL_MIN. */
    if (abs_max < std::numeric_limits<FT>::min()) abs_max = std::numeric_limits<FT>::min();
    /* equal if relative error <= factor x eps */
    return (abs_diff / abs_max) <= (RELATIVE_ERROR_FACTOR_NFA * std::numeric_limits<FT>::epsilon());
  }

  /** Computes -log10(NFA).
  NFA stands for Number of False Alarms:
  @f[
  \mathrm{NFA} = NT \cdot B(n,k,p)
  @f]
  - NT       - number of tests
  - B(n,k,p) - tail of binomial distribution with parameters n,k and p:
  @f[
  B(n,k,p) = \sum_{j=k}^n
  \left(\begin{array}{c}n\\j\end{array}\right)
  p^{j} (1-p)^{n-j}
  @f]
  The value -log10(NFA) is equivalent but more intuitive than NFA:
  - -1 corresponds to 10 mean false alarms
  -  0 corresponds to 1 mean false alarm
  -  1 corresponds to 0.1 mean false alarms
  -  2 corresponds to 0.01 mean false alarms
  -  ...
  Used this way, the bigger the value, better the detection,
  and a logarithmic scale is used.
  @param n,k,p binomial parameters.
  @param logNT logarithm of Number of Tests
  The computation is based in the gamma function by the following
  relation:
  @f[
  \left(\begin{array}{c}n\\k\end{array}\right)
  = \frac{ \Gamma(n+1) }{ \Gamma(k+1) \cdot \Gamma(n-k+1) }.
  @f]
  We use efficient algorithms to compute the logarithm of
  the gamma function.
  To make the computation faster, not all the sum is computed, part
  of the terms are neglected based on a bound to the error obtained
  (an error of 10% in the result is accepted).
  */
  static FT log_nfaNT(int n, int k, FT p, FT logNT) {
    FT tolerance = static_cast<FT>(0.1); /* an error of 10% in the result is accepted */
    FT log1term, term, bin_term, mult_term, bin_tail, err, p_term;
    int i;

    /* check parameters */
    CV_Assert(!(n < 0 || k < 0 || k > n || p <= 0.0 || p >= 1));

    /* trivial cases */
    if (n == 0 || k == 0) return -logNT;
    if (n == k) return -logNT - static_cast<FT>(n) * std::log10(p);

    /* probability term */
    p_term = p / (1 - p);

    /* compute the first term of the series */
    /*
    binomial_tail(n,k,p) = sum_{i=k}^n bincoef(n,i) * p^i * (1-p)^{n-i}
    where bincoef(n,i) are the binomial coefficients.
    But
    bincoef(n,k) = gamma(n+1) / ( gamma(k+1) * gamma(n-k+1) ).
    We use this to compute the first term. Actually the log of it.
    */
    log1term = log_gamma(static_cast<FT>(n + 1)) - log_gamma(static_cast<FT>(k + 1)) -
               log_gamma(static_cast<FT>(n - k + 1)) + static_cast<FT>(k) * log(p) +
               static_cast<FT>(n - k) * std::log(1 - p);
    term = std::exp(log1term);

    /* in some cases no more computations are needed */
    if (equal(term, 0)) {                              /* the first term is almost zero */
      if (static_cast<FT>(k) > static_cast<FT>(n) * p) /* at begin or end of the tail?  */
        return -log1term / MY_LN10 - logNT;            /* end: use just the first term  */
      else
        return -logNT; /* begin: the tail is roughly 1  */
    }

    /* compute more terms if needed */
    bin_tail = term;
    for (i = k + 1; i <= n; i++) {
      /*    As
      term_i = bincoef(n,i) * p^i * (1-p)^(n-i)
      and
      bincoef(n,i)/bincoef(n,i-1) = n-i+1 / i,
      then,
      term_i / term_i-1 = (n-i+1)/i * p/(1-p)
      and
      term_i = term_i-1 * (n-i+1)/i * p/(1-p).
      p/(1-p) is computed only once and stored in 'p_term'.
      */
      bin_term = static_cast<FT>(n - i + 1) / static_cast<FT>(i);
      mult_term = bin_term * p_term;
      term *= mult_term;
      bin_tail += term;
      if (bin_term < 1) {
        /* When bin_term<1 then mult_term_j<mult_term_i for j>i.
        Then, the error on the binomial tail when truncated at
        the i term can be bounded by a geometric series of form
        term_i * sum mult_term_i^j.                            */
        err = term * ((1 - std::pow(mult_term, static_cast<FT>(n - i + 1))) / (1 - mult_term) - 1);
        /* One wants an error at most of tolerance*final_result, or:
        tolerance * abs(-log10(bin_tail)-logNT).
        Now, the error that can be accepted on bin_tail is
        given by tolerance*final_result divided by the derivative
        of -log10(x) when x=bin_tail. that is:
        tolerance * abs(-log10(bin_tail)-logNT) / (1/bin_tail)
        Finally, we truncate the tail if the error is less than:
        tolerance * abs(-log10(bin_tail)-logNT) * bin_tail        */
        if (err < tolerance * std::abs(-log10(bin_tail) - logNT) * bin_tail) break;
      }
    }
    return -log10(bin_tail) - logNT;
  }

  static inline FT log_nfa(int n, int k, FT p, int N) { return log_nfaNT(n, k, p, std::log10(static_cast<FT>(N))); }

  static inline FT nfa(int n, int k, FT p, int N) {
    return -std::pow(10, log_nfaNT(n, k, p, std::log10(static_cast<FT>(N))));
  }
};

template <class GT, class FT, class PT>
class NfaBinom2 : public ValueManager {
  cv::Mat matx_, maty_;
  FT range_, log_e_, th_, p_;
  int N_;

  void eval(const EdgeSegment& seg, const std::vector<FT>& dirVec, EdgeSegmentVector& v, std::vector<FT>& n) const {
    int l = static_cast<int>(seg.size() - 1);
    if (l < 1) return;
    int k;
    size_t pos;

    FT r = range_ > 0 ? range_ : 360;
    max_dist(seg, dirVec, r, pos, k);

    FT vnfa = log_nfa(l, k, p_, N_);
    if (vnfa > log_e_) {
      v.push_back(seg);
      n.push_back(vnfa);
      return;
    }

    if (pos == seg.begin() || seg.begin() + 1) {
      if (pos + 1 < seg.end() - 1) eval(EdgeSegment(pos + 1, seg.end(), seg.flags() & ES_REVERSE), dirVec, v, n);
    } else if (pos >= seg.end() - 2) {
      if (pos - 1 > seg.begin()) eval(EdgeSegment(seg.begin(), pos, seg.flags() & ES_REVERSE), dirVec, v, n);
    } else {
      if (pos - 1 > seg.begin()) eval(EdgeSegment(seg.begin(), pos, seg.flags() & ES_REVERSE), dirVec, v, n);
      if (pos + 1 < seg.end() - 1) eval(EdgeSegment(pos + 1, seg.end(), seg.flags() & ES_REVERSE), dirVec, v, n);
    }
  }

  inline void max_dist(const EdgeSegment& seg, const std::vector<FT>& dirVec, FT r, size_t& pos, int& k) const {
    pos = seg.begin();
    FT m = 0;
    k = 0;
    size_t e = seg.end();
    FT last = dirVec[pos], tmp, ad;
    for (size_t i = seg.begin() + 1; i != e; ++i) {
      tmp = dirVec[i];
      ad = NfaBinom<GT, FT, PT>::angleDiff(last, tmp, r);
      if (ad < th_) ++k;
      if (ad > m) {
        pos = i;
        m = ad;
      }
      last = tmp;
    }
  }

  void init() {
    this->add("nfa_log_e", std::bind(&NfaBinom2<GT, FT, PT>::valueLogEps, this, std::placeholders::_1),
              "Epsilon -log10(e).");
    this->add("nfa_precision", std::bind(&NfaBinom2<GT, FT, PT>::valuePrecision, this, std::placeholders::_1),
              "Precision for nfa.");
  }


 public:
  typedef std::vector<PT> PointVector;

  NfaBinom2(FT log_e = 0, FT p = static_cast<FT>(1.0 / 8))
      : matx_(), maty_(), range_(0), log_e_(log_e), th_(0), p_(p), N_(0) {
    init();
  }

  NfaBinom2(const ValueManager::NameValueVector& options)
      : matx_(), maty_(), range_(0), log_e_(0), th_(0), p_(static_cast<FT>(1.0 / 8)), N_(0) {
    init();
    this->value(options);
  }

  NfaBinom2(ValueManager::InitializerList options)
      : matx_(), maty_(), range_(0), log_e_(0), th_(0), p_(static_cast<FT>(1.0 / 8)), N_(0) {
    init();
    this->value(options);
  }

  NfaBinom2(const cv::Mat& dir, FT r, FT log_e = 0, FT p = static_cast<FT>(1.0 / 8))
      : matx_(dir), range_(r), log_e_(log_e), th_(0), p_(p), N_(0) {
    init();
    N_ = dir.cols * dir.rows;
    N_ = N_ * N_;
  }

  NfaBinom2(const cv::Mat& gx, const cv::Mat& gy, FT log_e = 0, FT p = static_cast<FT>(1.0 / 8))
      : matx_(gx), maty_(gy), range_(0), log_e_(log_e), th_(0), p_(p), N_(0) {
    init();
    N_ = gx.cols * gx.rows;
    N_ = N_ * N_;
  }

  Value valueLogEps(const Value& e = Value::NAV()) {
    if (e.type()) log_eps(e.get<FT>());
    return log_e_;
  }

  FT log_eps() const { return log_e_; }

  void log_eps(FT log_e) { log_e_ = log_e; }

  FT epsilon() const { return std::pow(static_cast<FT>(10), -log_e_); }

  void epsilon(FT e) { log_e_ = -log10(e); }

  Value valuePrecision(const Value& p = Value::NAV()) {
    if (p.type()) precision(p.get<FT>());
    return p_;
  }

  FT precision() const { return p_; }

  void precision(FT p) {
    p_ = p;
    th_ = range_ > 0 ? range_ * p : 360 * p;
  }

  FT threshold() const { return th_; }

  void update(const cv::Mat& dir, FT range) {
    matx_ = dir;
    range_ = range;
    N_ = dir.cols * dir.rows;
    N_ = N_ * N_;
    th_ = range * p_;
  }

  void update(const cv::Mat& gx, const cv::Mat& gy) {
    matx_ = gx;
    maty_ = gy;
    range_ = 0;
    N_ = gx.cols * gx.rows;
    N_ = N_ * N_;
    th_ = 360 * p_;
  }

  template <class GRAD>
  void update(const GRAD& grad) {
    update(grad.gx(), grad.gy());
  }

  void update(EdgeSourceI& esource) { update(esource.gx(), esource.gy()); }


  inline FT eval(const EdgeSegment& seg, const PointVector& points) const {
    return range_ > 0 ? log_nfa(seg.size() - 1, NfaBinom<GT, FT, PT>::aligned(matx_, seg, points, th_, range_), p_, N_)
                      : log_nfa(seg.size() - 1, NfaBinom<GT, FT, PT>::aligned(matx_, maty_, seg, points, th_), p_, N_);
  }

  inline FT eval(const EdgeSegment& seg, const PointVector& points, FT dir) const {
    return range_ > 0 ? log_nfa(static_cast<int>(seg.size()),
                                NfaBinom<GT, FT, PT>::aligned(matx_, seg, points, dir, th_, range_), p_, N_)
                      : log_nfa(static_cast<int>(seg.size()),
                                NfaBinom<GT, FT, PT>::aligned(matx_, maty_, seg, points, dir, th_), p_, N_);
  }

  inline FT eval(const EdgeSegment& seg, const PointVector& points, const Line<FT>& line) const {
    return range_ > 0
               ? log_nfa(seg.size(),
                         NfaBinom<GT, FT, PT>::aligned(matx_, seg, points, line.gradientAnglef() / 360 * range_, th_,
                                                       range_),
                         p_, N_)
               : log_nfa(seg.size(),
                         NfaBinom<GT, FT, PT>::aligned(matx_, maty_, seg, points, line.gradientAnglef(), th_), p_, N_);
  }

  void eval(const EdgeSegmentVector& in, const PointVector& points, std::vector<FT>& n) const {
    n.clear();
    n.reserve(in.size());

    for_each(in.begin(), in.end(),
             [&points, &n, this](const EdgeSegment& seg) { n.push_back(this->eval(seg, points)); });
  }

  template <class MT>
  inline void eval(const EsdBase<MT, PT>& in, std::vector<FT>& n) const {
    eval(in.segments(), in.points(), n);
  }

  void eval(const EdgeSegmentVector& in,
            const PointVector& points,
            const std::vector<FT>& dir,
            std::vector<FT>& n) const {
    size_t s = in.size();
    n.clear();
    n.reserve(s);

    for (size_t i = 0; i != s; ++i) {
      n.push_back(eval(in[i], points, dir[i]));
    }
  }

  template <class MT>
  inline void eval(const EsdBase<MT, PT>& in, const std::vector<FT>& dir, std::vector<FT>& n) const {
    eval(in.segments(), in.points(), dir, n);
  }

  template <class LV>
  void evalLV(const EdgeSegmentVector& in, const PointVector& points, const LV& line, std::vector<FT>& n) const {
    size_t s = in.size();
    n.clear();
    n.reserve(s);

    if (range_ > 0 && range_ != 360) {
      FT conv = range_ / 360;
      for (size_t i = 0; i != s; ++i) {
        n.push_back(eval(in[i], points, line[i].gradientAnglef() * conv));
      }
    } else {
      for (size_t i = 0; i != s; ++i) {
        n.push_back(eval(in[i], points, line[i].gradientAnglef()));
      }
    }
  }

  void eval(const EdgeSegmentVector& in, const PointVector& points, EdgeSegmentVector& out, std::vector<FT>& n) const {
    std::vector<FT> dirVec;
    createDirVec(points, dirVec);
    eval(in, dirVec, out, n);
  }

  template <class MT>
  inline void eval(const EsdBase<MT, PT>& in, EdgeSegmentVector& out, std::vector<FT>& n) const {
    eval(in.segments(), in.points(), out, n);
  }

  void eval(const EdgeSegmentVector& in,
            const std::vector<FT>& dirVec,
            EdgeSegmentVector& out,
            std::vector<FT>& n) const {
    out.clear();
    out.reserve(in.size());
    n.clear();
    n.reserve(in.size());

    for_each(in.begin(), in.end(), [&, this](const EdgeSegment& seg) { this->eval(seg, dirVec, out, n); });
  }

  void createDirVec(const PointVector& points, std::vector<FT>& dirVec) const {
    if (points.empty()) return;
    dirVec.clear();
    dirVec.reserve(points.size());

    if (range_ > 0) {
      for_each(points.begin(), points.end(),
               [&dirVec, this](const PT& p) { dirVec.push_back(get<FT>(this->matx_, p)); });
    } else {
      for_each(points.begin(), points.end(), [&dirVec, this](const PT& p) {
        dirVec.push_back(cv::fastAtan2(get<GT>(this->maty_, p), get<GT>(this->matx_, p)));
      });
    }
  }

  static uint64 binomial(uint64 n, uint64 k) {
    uint64 c = 1, d, i;
    if (k > n) return 0;
    if (k == 0U || k == n) return 1;
    if (k > n - k) k = n - k;  // take advantage of symmetry
    for (i = 1; i <= k; ++i, --n) {
      d = c / i;
      if (d > UINT64_MAX / n)  // return 0 on overflow
        return 0U;
      c = d * n + c % i * n / i;  // split c*n/i into (c/i*i + c%i)*n/i
    }
    return c;
  }

  static FT nfa(int n, int k, FT p, int N) {
    CV_Assert(!(n < 0 || k < 0 || k > n || p <= 0 || p >= 1));

    if (n == 0 || k == 0) return static_cast<FT>(N);
    if (n == k) return static_cast<FT>(N) * std::pow(p, static_cast<FT>(n));

    FT sum = 0;

    for (int i = k; i != n; ++i) {
      sum += static_cast<FT>(binomial(static_cast<uint64>(n), static_cast<uint64>(i))) *
             std::pow(p, static_cast<FT>(i)) * std::pow(p, static_cast<FT>(n - i));
    }
    return static_cast<FT>(N) * sum;
  }

  static inline FT log_nfa(int n, int k, FT p, int N) { return -std::log10(nfa(n, k, p, N)); }
};


}  // namespace lsfm
