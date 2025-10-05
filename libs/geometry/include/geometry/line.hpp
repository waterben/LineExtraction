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

#include <geometry/point.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <string.h>

// for cout
#include <iostream>

namespace lsfm {

//! Line object
//! Note: The normal has direction of corresponding gradient. The gradient
//! direction is always directed from dark to bright. So the normal angle
//! is - PI/2 of line angle (90° counterclockwise to line direction)
//!  Since the positive y axis for images is from top to bottom, a positive rotation is clockwise
template <class FT, template <class> class PT = Vec2>
class Line {
 protected:
  // line internals (normal, distance to coord-origin)
  FT nx_, ny_, d_;

 public:
  typedef FT float_type;
  typedef PT<FT> point_type;

  Line() : nx_(0), ny_(0), d_(0) {}

  template <class AFT, template <class> class APT>
  Line(const Line<AFT, APT>& l) : nx_(l.normalX()), ny_(l.normalY()), d_(l.originDist()) {}

  //! Init Line by normal angle (radian -> line angle + PI/2) and distance to coord-origin
  Line(FT normal_ang, FT distance) : nx_(0), ny_(0), d_(distance) {
    nx_ = cos(normal_ang);
    ny_ = sin(normal_ang);
  }


  //! Init Line by normal and distance to coord-origin
  Line(FT normal_x, FT normal_y, FT distance) : nx_(normal_x), ny_(normal_y), d_(distance) {}


  //! Init Line by normal and starting coords
  Line(FT normal_x, FT normal_y, FT dist_x, FT dist_y) : nx_(normal_x), ny_(normal_y), d_(0) {
    d_ = nx_ * dist_x + ny_ * dist_y;
  }


  //! Init Line by start and end points (as two point_type)
  template <class APT>
  Line(const APT& beg, const APT& end) : nx_(0), ny_(0), d_(0) {
    nx_ = getY(beg) - getY(end);  //-y
    ny_ = getX(end) - getX(beg);  // x
    FT norm = detail::hypot(nx_, ny_);

    if (norm < LIMITS<FT>::eps()) {
      // init as empty line
      nx_ = ny_ = FT(0);
      return;
    }
    nx_ /= norm;
    ny_ /= norm;
    d_ = normalProject(getX(beg), getY(beg));
  }

  //! Init Line by start and end points (as Vec<FT,4>)
  Line(const Vec4<FT>& points) : nx_(0), ny_(0), d_(0) {
    nx_ = points[1] - points[3];  //-y
    ny_ = points[2] - points[0];  // x
    FT norm = detail::hypot(nx_, ny_);

    if (norm < LIMITS<FT>::eps()) {
      // init as empty line
      nx_ = ny_ = FT(0);
      return;
    }
    nx_ /= norm;
    ny_ /= norm;
    d_ = nx_ * points[0] + ny_ * points[1];
  }

  //! Init Line by start and end points as list of x1,y1,x2,y2
  Line(const FT* points) : nx_(0), ny_(0), d_(0) {
    CV_Assert(points);

    nx_ = points[1] - points[3];  //-y
    ny_ = points[2] - points[0];  // x
    FT norm = detail::hypot(nx_, ny_);

    if (norm < LIMITS<FT>::eps()) {
      // init as empty line
      nx_ = ny_ = FT(0);
      return;
    }
    nx_ /= norm;
    ny_ /= norm;
    d_ = nx_ * points[0] + ny_ * points[1];
  }

  bool valid() const { return detail::abs((nx_ * nx_ + ny_ * ny_) - 1) <= LIMITS<FT>::eps(); }

  bool empty() const { return nx_ == FT(0) && ny_ == FT(0); }

  template <class newFT, template <class> class newPT = Vec2>
  Line<newFT, newPT> convertTo() {
    return Line<newFT, newPT>(static_cast<newFT>(nx_), static_cast<newFT>(ny_), static_cast<newFT>(d_));
  }

  ////// Access internal attributes

  //! get distance of line to origin
  inline FT originDist() const { return d_; }

  //! get x part of line distance
  inline FT originX() const { return d_ * nx_; }

  //! get y part of line distance
  inline FT originY() const { return d_ * ny_; }

  //! get x,y part of line distance (origin point of line) as point
  inline point_type origin() const { return point_type(originX(), originY()); }

  //! get x part of line direction
  inline FT directionX() const { return ny_; }

  //! get y part of line direction
  inline FT directionY() const { return -nx_; }

  //! get line direction as point
  inline point_type direction() const { return point_type(ny_, -nx_); }

  //! get x part of normal
  inline FT normalX() const { return nx_; }

  //! get y part of normal
  inline FT normalY() const { return ny_; }

  //! get normal as point
  inline point_type normal() const { return point_type(nx_, ny_); }

  //! get fast angle of line to x-axis (less precision - 0.3°, in degrees[0,360))
  inline FT anglef() const {
    return cv::fastAtan2(static_cast<const float>(getScalar(-nx_)), static_cast<const float>(getScalar(ny_)));
  }

  //! get precise angle of line to x-axis (in radian [-PI,PI))
  inline FT angle() const { return detail::atan2(-nx_, ny_); }

  //! compute angle between two lines
  inline FT angle(const Line<FT, PT>& l) const { return detail::acos(nx_ * l.nx_ + ny_ * l.ny_); }

  //! get fast angle of normal to x-axis (less precision - 0.3°, in degrees[0,360))
  inline FT normalAnglef() const {
    return cv::fastAtan2(static_cast<const float>(getScalar(ny_)), static_cast<const float>(getScalar(nx_)));
  }

  //! get precise angle of line to x-axis (in radian [-PI,PI))
  inline FT normalAngle() const { return detail::atan2(ny_, nx_); }

  //! get fast angle of gradient (less precision - 0.3°, in degrees[0,360))
  inline FT gradientAnglef() const {
    return cv::fastAtan2(static_cast<const float>(-getScalar(ny_)), static_cast<const float>(-getScalar(nx_)));
  }

  //! get precise angle of gradient (in radian [-PI,PI))
  inline FT gradientAngle() const { return detail::atan2(-ny_, -nx_); }

  ////// Compute line / normal function

  //! compute x value of line by given y value
  inline FT x(FT y) const { return (d_ - ny_ * y) / nx_; }

  //! compute y value of line by given x value
  inline FT y(FT x) const { return (d_ - nx_ * x) / ny_; }

  //! compute x value of normal line (90° rotated) by given y value
  inline FT normalX(FT y) const { return (d_ + nx_ * y) / ny_; }

  //! compute y value of normal line (90° rotated) by given x value
  inline FT normalY(FT x) const { return (d_ - ny_ * x) / (-nx_); }

  ////// convert line / normal distances to x-y coords

  //! compute point with distance d along the line (in respect to coord-origin,
  //  add originPoint to get point in resprect to line origin)
  inline point_type lineDist(FT d) const { return point_type(ny_ * d, -nx_ * d); }

  //! compute point with distance d along the line and add starting point
  inline point_type lineDist(FT d, FT x, FT y) const { return point_type(x + ny_ * d, y - nx_ * d); }

  //! compute point with distance d along the line and add starting point
  inline point_type lineDist(FT d, const point_type& start) const {
    return point_type(getX(start) + ny_ * d, getY(start) - nx_ * d);
  }

  //! compute point with distance d along the line and add origin
  inline point_type lineDistOrigin(FT d) const { return lineDist(d, origin()); }


  //! compute point with distance s along the normal line (in respect to coord-origin,
  //  add originPoint to get point in resprect to line origin)
  inline point_type normalLineDist(FT s) const { return point_type(nx_ * s, ny_ * s); }

  //! compute point with distance s along the normal line and add to additional starting point
  inline point_type normalLineDist(FT d, FT x, FT y) const { return point_type(x + nx_ * d, y + ny_ * d); }

  //! compute vec with distance s along the normal line and add to additional starting point
  inline point_type normalLineDist(FT d, const point_type& start) const {
    return point_type(getX(start) + nx_ * d, getY(start) + ny_ * d);
  }


  ////// convert point to line / normal distance with projection

  //! project point by line direction (through coord origin) and compute distance
  inline FT project(FT x, FT y) const { return ny_ * x - nx_ * y; }

  //! project point by line direction (through coord origin) and compute distance
  inline FT project(const point_type& p) const { return ny_ * getX(p) - nx_ * getY(p); }

  //! project point by normal direction at line (through coord origin) and compute distance
  inline FT normalProject(FT x, FT y) const { return nx_ * x + ny_ * y; }

  //! project point by normal direction at line (through coord origin) and compute distance
  inline FT normalProject(const point_type& p) const { return nx_ * getX(p) + ny_ * getY(p); }


  /////// Base change -> rotate ( + translate)

  //! convert point / vector from line coord system to world coord system
  inline point_type line2world(FT x, FT y) const { return point_type(ny_ * x + nx_ * y, ny_ * y - nx_ * x); }

  //! convert point / vector from line coord system to world coord system
  inline point_type line2world(FT x, FT y, FT ox, FT oy) const {
    return point_type(ny_ * x + nx_ * y + ox, ny_ * y - nx_ * x + oy);
  }


  //! convert point / vector from line coord system to world coord system
  inline point_type line2world(const point_type& v) const { return line2world(getX(v), getY(v)); }

  //! convert point / vector from line coord system to world coord system
  inline point_type line2world(const point_type& v, const point_type& o) const {
    return line2world(getX(v), getY(v), getX(o), getY(o));
  }


  //! convert point / vector from world coord system to line coord system
  inline point_type world2line(FT x, FT y) const { return point_type(ny_ * x - nx_ * y, nx_ * x + ny_ * y); }

  //! convert point / vector from world coord system to line coord system
  inline point_type world2lineV(FT x, FT y) const { return point_type(ny_ * x - nx_ * y, nx_ * x + ny_ * y); }

  //! convert point / vector form world coord system to line coord system
  inline point_type world2line(FT x, FT y, FT ox, FT oy) const { return world2line(x - ox, y - oy); }

  //! convert point / vector from world coord system to line coord system
  inline point_type world2line(const point_type& v) const { world2line(getX(v), v[1]); }

  //! convert point / vector from world coord system to line coord system
  inline point_type world2line(const point_type& v, const point_type& o) const { return world2line(v - o); }


  /////// operations: line with point and line
  // to get distance from point to normal (normal always runs through origin), call project(point)
  //! compute distance between point and line
  inline FT distance(FT x, FT y) const { return normalProject(x, y) - d_; }

  //! compute distance between point and line
  inline FT distance(const point_type& v) const { return normalProject(v) - d_; }

  //! check if other line is parallel to this line
  inline bool isParallel(const Line& l) const { return detail::abs(nx_ * l.ny_ - ny_ * l.nx_) <= LIMITS<FT>::eps(); }

  //! check for intersection
  inline bool intersection(const Line& l) const { return !isParallel(l, LIMITS<FT>::eps()); }

  //! check for intersection and compute intersection point if possible
  inline bool intersection(const Line& l, FT& x, FT& y) const {
    FT dn = nx_ * l.ny_ - ny_ * l.nx_;
    if (detail::abs(dn) <= LIMITS<FT>::eps()) return false;
    x = (l.ny_ * d_ - ny_ * l.d_) / dn;
    y = (nx_ * l.d_ - l.nx_ * d_) / dn;
    return true;
  }

  //! check for intersection and compute intersection point if possible
  inline bool intersection(const Line& l, point_type& v) const { return intersection(l, getX(v), getY(v)); }


  ////// manipulation operators

  //! translate line so that it went through point p
  inline void translateTo(FT x, FT y) { d_ = normalProject(x, y); }

  //! translate line so that it went through point p
  inline void translateTo(const point_type& v) { d_ = normalProject(v); }

  //! translate line orthogonal by changing distance
  inline void translateOrtho(FT dist) { d_ += dist; }

  //! tanslate line by point
  inline void translate(FT x, FT y) { this->translateImpl(point_type(x, y)); }

  //! tanslate line by point
  inline void translate(const point_type& dist) { this->translateImpl(dist); }


  //! rotate line by angle (radian) at line origin (shortes point of line to origin ->
  //! just rotate normal)
  inline void rotate(FT angle) {
    FT sa = sin(angle), ca = cos(angle);
    FT tmp = nx_ * ca - ny_ * sa;
    ny_ = ny_ * ca + nx_ * sa;
    nx_ = tmp;
  }

  //! rotate line around line point (radian)
  inline void rotate(FT angle, const FT pivot) { this->rotateImpl(angle, pivot); }

  //! rotate line around point (radian)
  inline void rotate(FT angle, FT x, FT y) { this->rotateImpl(angle, point_type(x, y)); }

  //! rotate line around point (radian)
  inline void rotate(float_type angle, const point_type& pivot) { this->rotateImpl(angle, pivot); }

  //! scale line at origin (0,0)
  virtual void scale(FT s) {
    // update distance
    d_ *= s;
  }

  //! scale line at point
  inline void scale(float_type s, FT x, FT y) { this->scaleImpl(s, point_type(x, y)); }

  //! scale line at point
  inline void scale(float_type s, const point_type& pivot) { this->scaleImpl(s, pivot); }

  //! flip normal
  virtual void normalFlip() {
    nx_ *= FT(-1);
    ny_ *= FT(-1);
    d_ *= FT(-1);
  }


  /////// Output

  //! draw line
  inline void draw(cv::Mat& img,
                   cv::Scalar color,
                   int thickness = 1,
                   int lineType = 8,
                   double normalLength = 0,
                   double tipLength = 0) const {
    this->drawImpl(img, color, thickness, lineType, normalLength, tipLength);
  }


 protected:
  //! tanslate line by point impl
  virtual void translateImpl(const point_type& dist) { d_ += normalProject(dist); }

  //! rotate line around point (radian) impl
  virtual void rotateImpl(FT angle, const point_type& pivot) {
    FT sa = sin(angle), ca = cos(angle);
    point_type p = origin();
    p -= pivot;
    p = point_type(getX(p) * ca - getY(p) * sa, getY(p) * ca + getX(p) * sa);
    p += pivot;
    // rotate normal
    FT tmp = nx_ * ca - ny_ * sa;
    ny_ = ny_ * ca + nx_ * sa;
    nx_ = tmp;

    // update distance
    translateTo(p);
  }

  //! rotate line around line point (radian)
  virtual void rotateImpl(FT angle, FT pivot) {
    FT sa = sin(angle), ca = cos(angle);

    // get point for pivot
    point_type p = lineDist(pivot, origin());

    // rotate normal
    FT tmp = nx_ * ca - ny_ * sa;
    ny_ = ny_ * ca + nx_ * sa;
    nx_ = tmp;

    // update distance using p, since p is still on line
    translateTo(p);
  }

  //! scale line at point impl
  virtual void scaleImpl(FT s, const point_type& pivot) {
    FT dpv = normalProject(pivot);
    d_ -= dpv;
    d_ *= s;
    d_ += dpv;
  }

  //! draw line
  virtual void drawImpl(
      cv::Mat& img, cv::Scalar color, int thickness, int lineType, double normalLength, double tipLength) const;

 public:
  //! Virtual destructor for proper inheritance
  virtual ~Line() = default;
};

template <class FT, template <class> class PT = Vec2>
using Line2 = Line<FT, PT>;

typedef Line2<float> Line2f;
typedef Line2<double> Line2d;

template <class FT, template <class> class PT = Vec2>
using Line2Vector = std::vector<Line2<FT, PT>>;

typedef Line2Vector<float> Line2Vectorf;
typedef Line2Vector<double> Line2Vectord;

/**
 * Line segement object
 */
template <class FT, template <class> class PT = Vec2>
class LineSegment : public Line<FT, PT> {
 protected:
  using Line<FT, PT>::nx_;
  using Line<FT, PT>::ny_;
  using Line<FT, PT>::d_;

  // beg / end of line as distances to starting point
  FT beg_, end_;

  // the octave at which it was detected
  int octave_;

 public:
  typedef FT float_type;
  typedef PT<FT> point_type;

  LineSegment() : Line<FT, PT>(), beg_(0), end_(0), octave_(0) {}

  template <class AFT, template <class> class APT>
  LineSegment(const LineSegment<AFT, APT>& ls, int octave = 0)
      : Line<FT, PT>(ls), beg_(ls.start()), end_(ls.end()), octave_(octave) {}

  //! Init Line by normal angle, distance to origin and line begin and line end
  LineSegment(FT normal_ang, FT distance, FT line_beg, FT line_end, int octave = 0)
      : Line<FT, PT>(normal_ang, distance), beg_(line_beg), end_(line_end), octave_(octave) {
    if (beg_ > end_) flip();
  }

  //! Init Line by normal, distance to origin and line begin and line end
  LineSegment(FT normal_x, FT normal_y, FT distance, FT line_beg, FT line_end, int octave = 0)
      : Line<FT, PT>(normal_x, normal_y, distance), beg_(line_beg), end_(line_end), octave_(octave) {
    if (beg_ > end_) flip();
  }

  //! Init Line by normal, distance to origin and line begin and line end
  LineSegment(FT normal_x, FT normal_y, FT dist_x, FT dist_y, FT line_beg, FT line_end, int octave = 0)
      : Line<FT, PT>(normal_x, normal_y, dist_x, dist_y), beg_(line_beg), end_(line_end), octave_(octave) {
    if (beg_ > end_) flip();
  }

  //! Init Line by normal, distance to origin and line begin and line end
  LineSegment(FT normal_x, FT normal_y, FT dist_x, FT dist_y, FT beg_x, FT beg_y, FT end_x, FT end_y, int octave = 0)
      : Line<FT, PT>(normal_x, normal_y, dist_x, dist_y), beg_(0), end_(0), octave_(octave) {
    beg_ = ny_ * beg_x - nx_ * beg_y;
    end_ = ny_ * end_x - nx_ * end_y;

    if (beg_ > end_) flip();
  }


  //! Init Line by normal, distance to origin and line begin and line end (as point_type)
  LineSegment(
      const point_type& normal, const point_type& dist, const point_type& beg, const point_type& end, int octave = 0)
      : Line<FT, PT>(getX(normal), getY(normal), getX(dist), getY(dist)), beg_(0), end_(0), octave_(octave) {
    beg_ = this->project(beg);
    end_ = this->project(end);

    if (beg_ > end_) flip();
  }


  //! Init Line by start and end points (as two point_type)
  template <class APT>
  LineSegment(const APT& beg, const APT& end, int octave = 0)
      : Line<FT, PT>(beg, end), beg_(0), end_(0), octave_(octave) {
    beg_ = this->project(getX(beg), getY(beg));
    end_ = this->project(getX(end), getY(end));

    if (beg_ > end_) flip();
  }

  //! Init Line by start and end points (as Vec4x list of x1,y1,x2,y2)
  LineSegment(const Vec4<FT>& points, int octave = 0) : Line<FT, PT>(points), beg_(0), end_(0), octave_(octave) {
    beg_ = ny_ * points[0] - nx_ * points[1];
    end_ = ny_ * points[2] - nx_ * points[3];

    if (beg_ > end_) flip();
  }

  //! Init Line by start and end points as list of x1,y1,x2,y2
  LineSegment(const FT* points, int octave = 0) : Line<FT, PT>(points), beg_(0), end_(0), octave_(octave) {
    beg_ = ny_ * points[0] - nx_ * points[1];
    end_ = ny_ * points[2] - nx_ * points[3];

    if (beg_ > end_) flip();
  }

  //! Init Line by Line and endpoints of any point type
  template <class AFT, template <class> class APT, class APT2>
  LineSegment(const Line<AFT, APT>& l, const APT2& beg, const APT2& end, int octave = 0)
      : Line<FT, PT>(l), beg_(0), end_(0), octave_(octave) {
    beg_ = this->project(static_cast<FT>(getX(beg)), static_cast<FT>(getY(beg)));
    end_ = this->project(static_cast<FT>(getX(end)), static_cast<FT>(getY(end)));

    if (beg_ > end_) flip();
  }

  //! Init Line by Line and endpoints
  LineSegment(const Line<FT, PT>& l, FT line_beg, FT line_end, int octave = 0)
      : Line<FT, PT>(l), beg_(line_beg), end_(line_end), octave_(octave) {
    if (beg_ > end_) flip();
  }

  template <class newFT, template <class> class newPT = Vec2>
  LineSegment<newFT, newPT> convertTo() {
    return LineSegment<newFT, newPT>(static_cast<newFT>(nx_), static_cast<newFT>(ny_), static_cast<newFT>(d_),
                                     static_cast<newFT>(beg_), static_cast<newFT>(end_), octave_);
  }

  ////// Access internals


  //! get line start as distance
  inline FT start() const { return beg_; }

  //! get line end as distance
  inline FT end() const { return end_; }

  //! get line center as distance
  inline FT center() const { return beg_ + (end_ - beg_) / FT(2); }

  //! get line start as point
  inline point_type startPoint() const { return this->lineDist(beg_, this->origin()); }
  inline point_type getStartPoint() const { return startPoint(); }

  //! get line end as point
  inline point_type endPoint() const { return this->lineDist(end_, this->origin()); }
  inline point_type getEndPoint() const { return endPoint(); }

  //! get line endpoints of line as Vec4f
  inline Vec4<FT> endPoints() const {
    FT dx = this->originX(), dy = this->originY();
    return Vec4<FT>(dx + ny_ * beg_, dy - nx_ * beg_, dx + ny_ * end_, dy - nx_ * end_);
  }

  //! get line endpoints as vectors
  inline void endPoints(point_type& vbeg, point_type& vend) const {
    FT dx = this->originX(), dy = this->originY();
    set(vbeg, dx + ny_ * beg_, dy - nx_ * beg_);
    set(vend, dx + ny_ * end_, dy - nx_ * end_);
  }

  //! get center of line as point
  inline point_type centerPoint() const { return this->lineDist(this->center(), this->origin()); }

  //! get length of line
  inline FT length() const { return detail::abs(end_ - beg_); }

  //! get octave where it was detected
  inline int octave() const { return octave; }

  //! set the octave where it was detected
  inline void octave(int oct) const { this->octave_ = oct; }

  ////// helpers

  //! check if distance is within line segment range
  inline bool inRange(FT dist) const { return dist >= beg_ && dist <= end_; }

  //! check if distance is within line segment range
  inline bool inRangeTol(FT dist, FT tol) const { return dist >= (beg_ - tol) && dist <= (end_ + tol); }

  //! check if point is within line segment range
  inline bool inRange(FT x, FT y) const {
    FT dist = this->project(x, y);
    return inRange(dist);
  }

  //! check if point is within line segment range
  inline bool inRangeTol(FT x, FT y, FT tol) const {
    FT dist = this->project(x, y);
    return inRangeTol(dist, tol);
  }

  //! check if point is within line segment range
  inline bool inRange(const point_type& v) const { return inRange(getX(v), getY(v)); }

  //! check if point is within line segment range
  inline bool inRangeTol(const point_type& v, FT tol) const { return inRangeTol(getX(v), getY(v), tol); }


  //! trim line to box (default start is 0,0 and max_x is width and max_y is height)
  //! return true for success and false if line has no overlap with box
  inline bool trim2Box(FT max_x, FT max_y, FT min_x = FT(0), FT min_y = FT(0)) {
    // only test left and right
    FT tmp;
    if (this->nx_ == FT(0)) {
      if (this->ny_ < FT(0)) {
        tmp = max_x;
        max_x = -min_x;
        min_x = -tmp;
        tmp = max_y;
        max_y = -min_y;
        min_y = -tmp;
      }

      if (this->d_ > max_y || this->d_ < min_y || (this->beg_ > max_x && this->end_ > max_x) ||
          (this->beg_ < min_x && this->end_ < min_x))
        return false;


      if (this->beg_ > max_x) this->beg_ = max_x;
      if (this->beg_ < min_x) this->beg_ = min_x;

      if (this->end_ > max_x) this->end_ = max_x;
      if (this->end_ < min_x) this->end_ = min_x;
      return true;
    }
    // only test upper and lower
    if (this->ny_ == FT(0)) {
      if (this->nx_ > FT(0)) {
        tmp = max_y;
        max_y = -min_y;
        min_y = -tmp;
      } else {
        tmp = max_x;
        max_x = -min_x;
        min_x = -tmp;
      }

      if (this->d_ > max_x || this->d_ < min_x || (this->beg_ > max_y && this->end_ > max_y) ||
          (this->beg_ < min_y && this->end_ < min_y))
        return false;

      if (this->beg_ > max_y) this->beg_ = max_y;
      if (this->beg_ < min_y) this->beg_ = min_y;

      if (this->end_ > max_y) this->end_ = max_y;
      if (this->end_ < min_y) this->end_ = min_y;
      return true;
    }

    // get endpoints
    point_type pb;
    point_type pe;
    this->endPoints(pb, pe);

    // compute intersections with box lines
    point_type ihl;
    point_type ihu;
    point_type ivl;
    point_type ivr;

    auto zeroPoint = [](point_type& p) {
      setX(p, FT(0));
      setY(p, FT(0));
    };

    zeroPoint(pb);
    zeroPoint(pe);
    zeroPoint(ihl);
    zeroPoint(ihu);
    zeroPoint(ivl);
    zeroPoint(ivr);

    // lower horizontal line
    this->intersection(Line<FT, PT>(FT(0), FT(1), min_y), ihl);

    // upper horizontal line
    this->intersection(Line<FT, PT>(FT(0), FT(1), max_y), ihu);

    // left vertical line
    this->intersection(Line<FT, PT>(FT(1), FT(0), min_x), ivl);

    // right vertical line
    this->intersection(Line<FT, PT>(FT(1), FT(0), max_x), ivr);

    auto trim = [&ihl, &ihu, &ivl, &ivr](point_type& p) -> bool {
      bool ret = false;
      if (getX(p) < getX(ivl)) {
        p = ivl;
        ret = true;
      }

      else if (getX(p) > getX(ivr)) {
        p = ivr;
        ret = true;
      }

      if (getY(p) < getY(ihl)) {
        p = ihl;
        ret = true;
      }

      else if (getY(p) > getY(ihu)) {
        p = ihu;
        ret = true;
      }

      return ret;
    };

    // trim line endpoints
    bool tb = trim(pb);
    bool te = trim(pe);

    // check if line segment is outside
    /*if ((getX(pb) < getX(ivl) && getX(pe) < getX(ivl)) || (getX(pb) > getX(ivr) && getX(pe) > getX(ivr)) ||
        (getY(pb) < getY(ihl) && getY(pe) < getY(ihl)) || (getY(pb) > getY(ihu) && getY(pe) > getY(ihu)))
        return false;*/

    // if both points are outside, they will collapse to the same point outside
    // non collapsed points are inside
    if (pb == pe && (getX(pb) < getX(ivl) || getX(pb) > getX(ivr) || getY(pb) < getY(ihl) || getY(pb) > getY(ihu)))
      return false;

    // store projection if required
    if (tb) this->beg_ = this->project(pb);
    if (te) this->end_ = this->project(pe);

    return true;
  }

  //! Do two lines overlap? Be aware that, due to projection, one line might overlap with another but not the other way
  //! around!
  inline bool checkOverlap(LineSegment<FT, PT> line) {
    // Check if lines overlap
    FT sP = this->start();
    FT eP = this->end();
    if (sP > eP)  // swap
    {
      FT tP = eP;
      eP = sP;
      sP = tP;
    }

    FT pLs = this->project(line.startPoint());
    FT pLe = this->project(line.endPoint());
    // std::cout << "sP " << sP << " eP " << eP << " pLs " << pLs << " pLe " << pLe << std::endl;

    return ((pLs >= sP && pLs <= eP) || (pLe >= sP && pLe <= eP) ||
            (pLs <= sP && pLs <= eP && pLe >= sP && pLe >= eP) || (pLs >= sP && pLs >= eP && pLe <= sP && pLe <= eP));
  }

  ///// line transformations

  //! translate endpoints of line segment (move segment along line)
  inline void translate(FT dist) {
    beg_ += dist;
    end_ += dist;
  }

  //! translate start point of line segment
  inline void translateStart(FT dist) { beg_ += dist; }

  //! translate end point of line segment
  inline void translateEnd(FT dist) { end_ += dist; }

  //! rotate line around line point (radian)
  inline void rotate(FT angle, FT pivot) { this->rotateImpl(angle, pivot); }

  //! scale line at origin (0,0)
  virtual void scale(FT s) {
    Line<FT, PT>::scale(s);
    beg_ *= s;
    end_ *= s;
  }

  //! scale line at point on line
  inline void scale(FT s, FT pivot) {
    // scale endpoints
    beg_ -= pivot;
    beg_ *= s;
    beg_ += pivot;
    end_ -= pivot;
    end_ *= s;
    end_ += pivot;
  }

  virtual void normalFlip() {
    Line<FT, PT>::normalFlip();
    beg_ *= FT(-1);
    end_ *= FT(-1);
    std::swap(beg_, end_);
  }

  //! swap endpoints
  inline void endPointSwap() { std::swap(beg_, end_); }


  /**
   * @brief  Error calculated by Endpoint Distance
   * @param  other line (ground truth)
   * @return added Endpoint error distance
   */
  inline FT error(const Line<FT, PT>& gtLine) const {
    FT a, b;
    return error(gtLine, a, b);
  }

  inline FT error(const Line<FT, PT>& gtLine, FT& a, FT& b) const {
    if (this->empty() || gtLine.empty()) return FT(0);
    a = gtLine.distance(this->startPoint());
    b = gtLine.distance(this->endPoint());

    if (a < FT(0)) a = a * FT(-1.0);
    if (b < FT(0)) b = b * FT(-1.0);

    return a + b;
  }


 protected:
  void flip() {
    Line<FT, PT>::normalFlip();
    beg_ *= FT(-1);
    end_ *= FT(-1);
  }

  //! translate line by vector
  virtual void translateImpl(const point_type& dist) {
    this->d_ += this->normalProject(dist);
    FT tmp = this->project(dist);
    beg_ += tmp;
    end_ += tmp;
  }

  //! rotate line around point (radian)
  virtual void rotateImpl(FT angle, const point_type& pivot) {
    FT sa = sin(angle), ca = cos(angle);

    // rotate point on line segment
    point_type p = this->startPoint();
    p -= pivot;
    p = point_type(getX(p) * ca - getY(p) * sa, getY(p) * ca + getX(p) * sa);
    p += pivot;

    // rotate normal
    FT tmp = nx_ * ca - ny_ * sa;
    ny_ = ny_ * ca + nx_ * sa;
    nx_ = tmp;

    // update distance
    d_ = this->normalProject(p);

    // update end points
    FT dif = this->project(p) - beg_;
    beg_ += dif;
    end_ += dif;
  }

  //! rotate line around line point (radian)
  virtual void rotateImpl(FT angle, FT pivot) {
    FT sa = sin(angle), ca = cos(angle);

    // get point for pivot
    point_type p = this->lineDist(pivot, this->origin());

    // rotate normal
    FT tmp = nx_ * ca - ny_ * sa;
    ny_ = ny_ * ca + nx_ * sa;
    nx_ = tmp;

    // update distance using p, since p is still on line
    d_ = this->normalProject(p);

    // update endpoints (use difference of old and new pivot)
    FT dif = this->project(p) - pivot;
    beg_ += dif;
    end_ += dif;
  }

  //! scale line at point
  virtual void scaleImpl(FT s, const point_type& pivot) {
    // scale distance
    FT dpv = this->normalProject(pivot);
    d_ -= dpv;
    d_ *= s;
    d_ += dpv;

    // scale endpoints
    dpv = this->project(pivot);
    beg_ -= dpv;
    beg_ *= s;
    beg_ += dpv;
    end_ -= dpv;
    end_ *= s;
    end_ += dpv;
  }

  //! draw line
  virtual void drawImpl(
      cv::Mat& img, cv::Scalar color, int thickness, int lineType, double normalLength, double tipLength) const {
    if (this->empty()) return;

    point_type pt1 = this->startPoint(), pt2 = this->endPoint();
    cv::Point cvp2 = cv::Point(static_cast<int>(std::round(getScalar(getX(pt2)))),
                               static_cast<int>(std::round(getScalar(getY(pt2)))));
    cv::line(img,
             cv::Point(static_cast<int>(std::round(getScalar(getX(pt1)))),
                       static_cast<int>(std::round(getScalar(getY(pt1))))),
             cvp2, color, thickness, lineType);

    if (normalLength != 0) {
      point_type p1 = this->normalLineDist(FT(0), centerPoint()),
                 p2 = this->normalLineDist(FT(normalLength), centerPoint());
      cv::line(img,
               cv::Point(static_cast<int>(std::round(getScalar(getX(p1)))),
                         static_cast<int>(std::round(getScalar(getY(p1))))),
               cv::Point(static_cast<int>(std::round(getScalar(getX(p2)))),
                         static_cast<int>(std::round(getScalar(getY(p2))))),
               color, thickness, lineType);
    }

    if (tipLength > 0) {
      const double ang = getScalar(this->angle());

      cv::Point cvp(static_cast<int>(std::round(getScalar(getX(pt2)) + tipLength * -std::cos(ang + CV_PI / 4))),
                    static_cast<int>(std::round(getScalar(getY(pt2)) + tipLength * -std::sin(ang + CV_PI / 4))));
      cv::line(img, cvp, cvp2, color, thickness, lineType);

      cvp.x = static_cast<int>(std::round(getScalar(getX(pt2)) + tipLength * -std::cos(ang - CV_PI / 4)));
      cvp.y = static_cast<int>(std::round(getScalar(getY(pt2)) + tipLength * -std::sin(ang - CV_PI / 4)));
      cv::line(img, cvp, cvp2, color, thickness, lineType);
    }
  }

  template <class U>
  friend std::ostream& operator<<(std::ostream& os, const LineSegment<U>& ls);

 public:
  //! Virtual destructor for proper inheritance
  virtual ~LineSegment() = default;
};

template <class FT>
std::ostream& operator<<(std::ostream& os, const LineSegment<FT>& ls) {
  return os << "S: " << ls.startPoint()[0] << ", " << ls.startPoint()[1] << "  E: " << ls.endPoint()[0] << ", "
            << ls.endPoint()[1];
}


template <class FT, template <class> class PT = Vec2>
using LineSegment2 = LineSegment<FT, PT>;

typedef LineSegment<float> LineSegment2f;
typedef LineSegment<double> LineSegment2d;

template <class FT, template <class> class PT = Vec2>
using LineSegment2Vector = std::vector<LineSegment2<FT, PT>>;

typedef LineSegment2Vector<float> LineSegment2Vectorf;
typedef LineSegment2Vector<double> LineSegment2Vectord;

//! trim line to box (default start is 0,0 and max_x is width and max_y is height)
//! return trimmed line segment (empty segment, if outside box)
template <class FT, template <class> class LPT>
inline LineSegment<FT, LPT> trim2Box(const Line<FT, LPT>& l, FT max_x, FT max_y, FT min_x = FT(0), FT min_y = FT(0)) {
  LineSegment<FT, LPT> ret;
  if (l.empty()) return ret;

  if (detail::abs(l.normalX()) < detail::abs(l.normalY()))
    ret = LineSegment<FT, LPT>(LPT<FT>(min_x, l.y(min_x)), LPT<FT>(max_x, l.y(max_x)));
  else
    ret = LineSegment<FT, LPT>(LPT<FT>(l.x(min_y), min_y), LPT<FT>(l.x(max_y), max_y));
  return ret.trim2Box(max_x, max_y, min_x, min_y) ? ret : LineSegment<FT, LPT>();
}

template <class FT, template <class> class LPT>
void Line<FT, LPT>::drawImpl(
    cv::Mat& img, cv::Scalar color, int thickness, int lineType, double normalLength, double tipLength) const {
  trim2Box<FT, LPT>(*this, FT(img.cols), FT(img.rows)).draw(img, color, thickness, lineType, normalLength, tipLength);
}

}  // namespace lsfm
