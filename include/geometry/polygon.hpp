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


#ifndef _GEOMETRY_POLYGON_HPP_
#define _GEOMETRY_POLYGON_HPP_
#ifdef __cplusplus

#  include "line.hpp"
#  include <opencv2/imgproc/imgproc.hpp>

#  include <vector>


namespace lsfm {

//! Polygon object
template <class FT, template <class> class PT = Vec2>
class Polygon {
  PT<FT> piviot_;
  std::vector<PT<FT>> verticies_;

 public:
  typedef FT float_type;
  typedef PT<FT> point_type;
  typedef std::vector<point_type> VertexVector;

  Polygon(const point_type& piviot = point_type(0, 0)) : piviot_(piviot) {}

  Polygon(const LineSegment2Vector<FT, PT>& segments, const point_type& piviot = point_type(0, 0)) : piviot_(piviot) {
    if (segments.size() < 2) {
      return;
    }
    auto last_seg = segments.end() - 1;
    for (auto iter = segments.begin(); iter != segments.end(); ++iter) {
      point_type p{};
      if (last_seg->intersection(*iter, p)) {
        verticies_.push_back(p);
      }
      last_seg = iter;
    }
  }

  inline VertexVector& verticies() { return verticies_; }

  inline const VertexVector& verticies() const { return verticies_; }

  inline LineSegment2Vector<FT, PT> edges() const {
    LineSegment2Vector<FT, PT> ret;
    if (verticies_.size() == 2)
      ret.push_back(LineSegment2<FT, PT>(verticies_[0], verticies_[1]));
    else if (verticies_.size() > 2) {
      ret.reserve(verticies_.size());
      size_t i = 0;
      for (; i != verticies_.size() - 1; ++i) ret.push_back(LineSegment2<FT, PT>(verticies_[i], verticies_[i + 1]));
      ret.push_back(LineSegment2<FT, PT>(verticies_[i], verticies_[0]));
    }
    return ret;
  }

  inline VertexVector wolrdVerticies() const {
    VertexVector ret;
    ret.reserve(verticies_.size());
    for_each(verticies_.begin(), verticies_.end(), [&, this](point_type& p) { ret.push_back(p + this->piviot_); });
    return ret;
  }

  inline LineSegment2Vector<FT, PT> worldEdges() const {
    LineSegment2Vector<FT, PT> ret;
    if (verticies_.size() == 2)
      ret.push_back(LineSegment2<FT, PT>(verticies_[0] + piviot_, verticies_[1] + piviot_));
    else if (verticies_.size() > 2) {
      VertexVector tmp = wolrdVerticies();
      ret.reserve(tmp.size());
      size_t i = 0;
      for (; i != tmp.size() - 1; ++i) ret.push_back(LineSegment2<FT, PT>(tmp[i], tmp[i + 1]));
      ret.push_back(LineSegment2<FT, PT>(tmp[i], tmp[0]));
    }
    return ret;
  }

  inline void clear() const { verticies_.clear(); }

  inline void push_back(const point_type& vertex) { verticies_.push_back(vertex); }

  inline void push_back_world(const point_type& vertex) { verticies_.push_back(vertex - piviot_); }

  inline bool empty() const { return verticies_.empty(); }

  inline size_t size() const { return verticies_.size(); }

  //! translate polygon
  inline void translate(const point_type& t) { piviot_ += t; }


  //! rotate shape by angle (radian) around piviot
  inline void rotate(FT angle) {
    FT sa = sin(angle), ca = cos(angle);
    for_each(verticies_.begin(), verticies_.end(),
             [&](point_type& p) { set(p, getX(p) * ca - getY(p) * sa, getX(p) * sa + getY(p) * ca); });
  }

  //! rotate shape by angle (radian) around o
  inline void rotate(FT angle, const point_type& o) {
    FT sa = sin(angle), ca = cos(angle);
    point_type t = piviot_ - o;
    for_each(verticies_.begin(), verticies_.end(), [&](point_type& p) {
      p += t;
      set(p, getX(p) * ca - getY(p) * sa, getX(p) * sa + getY(p) * ca);
      p -= t;
    });
  }


  //! scale shape at piviot
  virtual void scale(FT s) {
    for_each(verticies_.begin(), verticies_.end(), [&, this](point_type& p) { p *= s; });
  }

  //! scale shape at o
  virtual void scale(FT s, const point_type& o) {
    point_type t = piviot_ - o;
    for_each(verticies_.begin(), verticies_.end(), [&, this](point_type& p) {
      p += t;
      p *= s;
      p -= t;
    });
  }

  inline point_type& piviot() { return piviot_; }

  inline const point_type& piviot() const { return piviot_; }

  //! translate polygon
  inline void translatePiviot(const point_type& t) {
    for_each(verticies_.begin(), verticies_.end(), [&](point_type& p) { p -= t; });
    piviot_ += t;
  }


  inline void fill(cv::Mat& img, const cv::Scalar& color, int lineType = 8) const {
    std::vector<cv::Point> in;
    in.reserve(verticies_.size());
    point_type tmp;
    for_each(verticies_.begin(), verticies_.end(), [&, this](const point_type& p) {
      tmp = p + this->piviot_;
      in.push_back(cv::Point(std::round(getX(tmp)), std::round(getY(tmp))));
    });
    cv::fillConvexPoly(img, in.data(), static_cast<int>(verticies_.size()), color, lineType);
  }

  inline void fillComplex(cv::Mat& img, const cv::Scalar& color, int lineType = 8) const {
    std::vector<cv::Point> in;
    in.reserve(verticies_.size());
    point_type tmp;

    for_each(verticies_.begin(), verticies_.end(), [&, this](const point_type& p) {
      tmp = p + this->piviot_;
      in.push_back(cv::Point(std::round(getX(tmp)), std::round(getY(tmp))));
    });
    int np = static_cast<int>(verticies_.size());
    const cv::Point* data = in.data();
    cv::fillPoly(img, &data, &np, 1, color, lineType);
  }

  inline void draw(cv::Mat& img, const cv::Scalar& color, int thickness = 1, int lineType = 8) const {
    std::vector<cv::Point> in;
    in.reserve(verticies_.size());
    point_type tmp;
    for_each(verticies_.begin(), verticies_.end(), [&, this](const point_type& p) {
      tmp = p + this->piviot_;
      in.push_back(cv::Point(std::round(getX(tmp)), std::round(getY(tmp))));
    });
    int np = static_cast<int>(verticies_.size());
    const cv::Point* data = in.data();
    cv::polylines(img, &data, &np, 1, true, color, thickness, lineType);
  }

  inline bool isConvex() const {
    if (verticies_.size() < 3) return false;

    point_type p1 = verticies_[0], p2 = verticies_[1], p3 = verticies_[2], d = p2 - p1;
    size_t s = verticies_.size();
    FT res = getX(p3) * getY(d) - getY(p3) * getX(d) + getX(d) * getY(p1) - getY(d) * getX(p1);

    for (size_t i = 1; i < s; ++i) {
      p1 = p2;
      p2 = p3;
      p3 = verticies_[(i + 2) % s];
      d = p2 - p1;

      FT newres = getX(p3) * getY(d) - getY(p3) * getX(d) + getX(d) * getY(p1) - getY(d) * getX(p1);
      if ((newres > 0 && res < 0) || (newres < 0 && res > 0)) return false;
    }
    return true;
  }
};

template <class FT, template <class> class PT = Vec2>
using PolygonVector = std::vector<Polygon<FT, PT>>;

}  // namespace lsfm
#endif
#endif
