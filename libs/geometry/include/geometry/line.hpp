//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann & Manuel Lange
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file line.hpp
/// @brief 2D line and line segment representations.
///
/// Provides Line and LineSegment classes for 2D geometric computations.
/// Lines are represented in normal form: n·p = d, where n is the unit normal,
/// p is a point on the line, and d is the signed distance to the origin.
/// @note The normal has the direction of the corresponding gradient.
///
/// The gradient direction is always from dark to bright, so the normal angle
/// is -PI/2 from the line angle (90° counterclockwise to line direction).
/// Since the positive y-axis for images is top-to-bottom, positive rotation
/// is clockwise.

#pragma once

#include <geometry/point.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <string.h>

// for cout
#include <iostream>

namespace lsfm {

/// @brief 2D infinite line in normal form.
///
/// Represents a 2D line using Hesse normal form: n·p = d, where:
/// - n = (nx, ny) is the unit normal vector
/// - d is the signed perpendicular distance to the origin
/// The normal direction corresponds to the gradient direction (dark to bright).
/// @tparam FT Floating point type (float or double).
/// @tparam PT Point template (default Vec2).
/// @note Line direction is perpendicular to normal: dir = (ny, -nx)
template <class FT, template <class> class PT = Vec2>
class Line {
 protected:
  FT nx_, ny_, d_;  ///< Normal components and signed distance to origin.

 public:
  typedef FT float_type;      ///< Scalar type.
  typedef PT<FT> point_type;  ///< Point type.

  /// @brief Default constructor (zero line).
  Line() : nx_(0), ny_(0), d_(0) {}

  /// @brief Copy constructor with type conversion.
  template <class AFT, template <class> class APT>
  Line(const Line<AFT, APT>& l) : nx_(l.normalX()), ny_(l.normalY()), d_(l.originDist()) {}

  /// @brief Construct from normal angle and distance.
  /// @param normal_ang Normal angle in radians (line angle + PI/2).
  /// @param distance Signed distance to origin.
  Line(FT normal_ang, FT distance) : nx_(0), ny_(0), d_(distance) {
    nx_ = static_cast<FT>(cos(normal_ang));
    ny_ = static_cast<FT>(sin(normal_ang));
  }


  /// @brief Construct from normal components and distance.
  /// @param normal_x Normal x-component.
  /// @param normal_y Normal y-component.
  /// @param distance Signed distance to origin.
  Line(FT normal_x, FT normal_y, FT distance) : nx_(normal_x), ny_(normal_y), d_(distance) {}


  /// @brief Construct from normal and a point on the line.
  /// @param normal_x Normal x-component.
  /// @param normal_y Normal y-component.
  /// @param dist_x X-coordinate of point on line.
  /// @param dist_y Y-coordinate of point on line.
  Line(FT normal_x, FT normal_y, FT dist_x, FT dist_y) : nx_(normal_x), ny_(normal_y), d_(0) {
    d_ = nx_ * dist_x + ny_ * dist_y;
  }


  /// @brief Construct from two points defining the line.
  /// @tparam APT Point type.
  /// @param beg Start point.
  /// @param end End point.
  template <class APT>
  Line(const APT& beg, const APT& end) : nx_(0), ny_(0), d_(0) {
    nx_ = static_cast<FT>(getY(beg) - getY(end));  //-y
    ny_ = static_cast<FT>(getX(end) - getX(beg));  // x
    FT norm = detail::hypot(nx_, ny_);

    if (norm < LIMITS<FT>::eps()) {
      // init as empty line
      nx_ = ny_ = FT(0);
      return;
    }
    nx_ /= norm;
    ny_ /= norm;
    d_ = normalProject(static_cast<FT>(getX(beg)), static_cast<FT>(getY(beg)));
  }

  /// @brief Construct from Vec4 containing endpoints (x1,y1,x2,y2).
  /// @param points Vector with [x1, y1, x2, y2].
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

  /// @brief Construct from raw pointer to endpoints [x1,y1,x2,y2].
  /// @param points Pointer to array of 4 coordinates.
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

  /// @brief Check if line normal is unit length.
  bool valid() const { return detail::abs((nx_ * nx_ + ny_ * ny_) - 1) <= LIMITS<FT>::eps(); }

  /// @brief Check if line is degenerate (zero normal).
  bool empty() const { return nx_ == FT(0) && ny_ == FT(0); }

  /// @brief Convert to different floating-point/point type.
  /// @tparam newFT New floating-point type.
  /// @tparam newPT New point template.
  /// @return Converted line.
  template <class newFT, template <class> class newPT = Vec2>
  Line<newFT, newPT> convertTo() {
    return Line<newFT, newPT>(static_cast<newFT>(nx_), static_cast<newFT>(ny_), static_cast<newFT>(d_));
  }

  /// @name Attribute accessors
  /// @{

  /// @brief Get signed distance from line to origin.
  inline FT originDist() const { return d_; }

  /// @brief Get x-component of closest point to origin.
  inline FT originX() const { return d_ * nx_; }

  /// @brief Get y-component of closest point to origin.
  inline FT originY() const { return d_ * ny_; }

  /// @brief Get closest point on line to origin.
  inline point_type origin() const { return point_type(originX(), originY()); }

  /// @brief Get x-component of line direction.
  inline FT directionX() const { return ny_; }

  /// @brief Get y-component of line direction.
  inline FT directionY() const { return -nx_; }

  /// @brief Get line direction as unit vector.
  inline point_type direction() const { return point_type(ny_, -nx_); }

  /// @brief Get x-component of normal.
  inline FT normalX() const { return nx_; }

  /// @brief Get y-component of normal.
  inline FT normalY() const { return ny_; }

  /// @brief Get normal as unit vector.
  inline point_type normal() const { return point_type(nx_, ny_); }

  /// @brief Get fast line angle to x-axis (degrees, ~0.3° precision).
  inline FT anglef() const {
    return cv::fastAtan2(static_cast<const float>(getScalar(-nx_)), static_cast<const float>(getScalar(ny_)));
  }

  /// @brief Get precise line angle to x-axis (radians [-PI,PI)).
  inline FT angle() const { return detail::atan2(-nx_, ny_); }

  /// @brief Compute angle between this line and another.
  inline FT angle(const Line<FT, PT>& l) const { return detail::acos(nx_ * l.nx_ + ny_ * l.ny_); }

  /// @brief Get fast normal angle to x-axis (degrees, ~0.3° precision).
  inline FT normalAnglef() const {
    return cv::fastAtan2(static_cast<const float>(getScalar(ny_)), static_cast<const float>(getScalar(nx_)));
  }

  /// @brief Get precise normal angle to x-axis (radians [-PI,PI)).
  inline FT normalAngle() const { return detail::atan2(ny_, nx_); }

  /// @brief Get fast gradient angle (degrees, ~0.3° precision).
  inline FT gradientAnglef() const {
    return cv::fastAtan2(static_cast<const float>(-getScalar(ny_)), static_cast<const float>(-getScalar(nx_)));
  }

  /// @brief Get precise gradient angle (radians [-PI,PI)).
  inline FT gradientAngle() const { return detail::atan2(-ny_, -nx_); }
  /// @}

  /// @name Line/normal function evaluation
  /// @{

  /// @brief Compute x value on line for given y.
  inline FT x(FT y) const { return (d_ - ny_ * y) / nx_; }

  /// @brief Compute y value on line for given x.
  inline FT y(FT x) const { return (d_ - nx_ * x) / ny_; }

  /// @brief Compute x value on normal line for given y.
  inline FT normalX(FT y) const { return (d_ + nx_ * y) / ny_; }

  /// @brief Compute y value on normal line for given x.
  inline FT normalY(FT x) const { return (d_ - ny_ * x) / (-nx_); }
  /// @}

  /// @name Distance conversion functions
  /// @{

  /// @brief Get point at distance d along line direction (relative to origin).
  inline point_type lineDist(FT d) const { return point_type(ny_ * d, -nx_ * d); }

  /// @brief Get point at distance d along line from (x,y).
  inline point_type lineDist(FT d, FT x, FT y) const { return point_type(x + ny_ * d, y - nx_ * d); }

  /// @brief Get point at distance d along line from start point.
  inline point_type lineDist(FT d, const point_type& start) const {
    return point_type(getX(start) + ny_ * d, getY(start) - nx_ * d);
  }

  /// @brief Get point at distance d along line from line's origin point.
  inline point_type lineDistOrigin(FT d) const { return lineDist(d, origin()); }


  /// @brief Get point at distance s along normal direction (relative to origin).
  inline point_type normalLineDist(FT s) const { return point_type(nx_ * s, ny_ * s); }

  /// @brief Get point at distance d along normal from (x,y).
  inline point_type normalLineDist(FT d, FT x, FT y) const { return point_type(x + nx_ * d, y + ny_ * d); }

  /// @brief Get point at distance d along normal from start point.
  inline point_type normalLineDist(FT d, const point_type& start) const {
    return point_type(getX(start) + nx_ * d, getY(start) + ny_ * d);
  }
  /// @}


  /// @name Projection functions
  /// @{

  /// @brief Project point onto line direction and get signed distance.
  inline FT project(FT x, FT y) const { return ny_ * x - nx_ * y; }

  /// @brief Project point onto line direction and get signed distance.
  inline FT project(const point_type& p) const { return ny_ * getX(p) - nx_ * getY(p); }

  /// @brief Project point onto normal direction and get signed distance.
  inline FT normalProject(FT x, FT y) const { return nx_ * x + ny_ * y; }

  /// @brief Project point onto normal direction and get signed distance.
  inline FT normalProject(const point_type& p) const { return nx_ * getX(p) + ny_ * getY(p); }
  /// @}


  /// @name Coordinate transformations
  /// @{

  /// @brief Transform point from line coordinates to world coordinates.
  inline point_type line2world(FT x, FT y) const { return point_type(ny_ * x + nx_ * y, ny_ * y - nx_ * x); }

  /// @brief Transform point from line coordinates to world coordinates with offset.
  inline point_type line2world(FT x, FT y, FT ox, FT oy) const {
    return point_type(ny_ * x + nx_ * y + ox, ny_ * y - nx_ * x + oy);
  }


  /// @brief Transform point from line coordinates to world coordinates.
  inline point_type line2world(const point_type& v) const { return line2world(getX(v), getY(v)); }

  /// @brief Transform point from line coordinates to world coordinates with offset.
  inline point_type line2world(const point_type& v, const point_type& o) const {
    return line2world(getX(v), getY(v), getX(o), getY(o));
  }


  /// @brief Transform point from world coordinates to line coordinates.
  inline point_type world2line(FT x, FT y) const { return point_type(ny_ * x - nx_ * y, nx_ * x + ny_ * y); }

  /// @brief Transform vector from world coordinates to line coordinates.
  inline point_type world2lineV(FT x, FT y) const { return point_type(ny_ * x - nx_ * y, nx_ * x + ny_ * y); }

  /// @brief Transform point from world coordinates to line coordinates with offset.
  inline point_type world2line(FT x, FT y, FT ox, FT oy) const { return world2line(x - ox, y - oy); }

  /// @brief Transform point from world coordinates to line coordinates.
  inline point_type world2line(const point_type& v) const { world2line(getX(v), v[1]); }

  /// @brief Transform point from world coordinates to line coordinates with offset.
  inline point_type world2line(const point_type& v, const point_type& o) const { return world2line(v - o); }
  /// @}


  /// @name Point and line operations
  /// @{

  /// @brief Compute signed distance from point to line.
  inline FT distance(FT x, FT y) const { return normalProject(x, y) - d_; }

  /// @brief Compute signed distance from point to line.
  inline FT distance(const point_type& v) const { return normalProject(v) - d_; }

  /// @brief Check if another line is parallel to this line.
  inline bool isParallel(const Line& l) const { return detail::abs(nx_ * l.ny_ - ny_ * l.nx_) <= LIMITS<FT>::eps(); }

  /// @brief Check if lines intersect (not parallel).
  inline bool intersection(const Line& l) const { return !isParallel(l, LIMITS<FT>::eps()); }

  /// @brief Compute intersection point with another line.
  /// @param l Other line.
  /// @param[out] x X-coordinate of intersection.
  /// @param[out] y Y-coordinate of intersection.
  /// @return True if lines intersect, false if parallel.
  inline bool intersection(const Line& l, FT& x, FT& y) const {
    FT dn = nx_ * l.ny_ - ny_ * l.nx_;
    if (detail::abs(dn) <= LIMITS<FT>::eps()) return false;
    x = (l.ny_ * d_ - ny_ * l.d_) / dn;
    y = (nx_ * l.d_ - l.nx_ * d_) / dn;
    return true;
  }

  /// @brief Compute intersection point with another line.
  inline bool intersection(const Line& l, point_type& v) const { return intersection(l, getX(v), getY(v)); }
  /// @}


  /// @name Transformation operations
  /// @{

  /// @brief Translate line to pass through point (x,y).
  inline void translateTo(FT x, FT y) { d_ = normalProject(x, y); }

  /// @brief Translate line to pass through point.
  inline void translateTo(const point_type& v) { d_ = normalProject(v); }

  /// @brief Translate line orthogonally by dist.
  inline void translateOrtho(FT dist) { d_ += dist; }

  /// @brief Translate line by vector (x,y).
  inline void translate(FT x, FT y) { this->translateImpl(point_type(x, y)); }

  /// @brief Translate line by vector.
  inline void translate(const point_type& dist) { this->translateImpl(dist); }


  /// @brief Rotate line about its origin point.
  /// @param angle Rotation angle in radians.
  inline void rotate(FT angle) {
    FT sa = static_cast<FT>(sin(angle)), ca = static_cast<FT>(cos(angle));
    FT tmp = nx_ * ca - ny_ * sa;
    ny_ = ny_ * ca + nx_ * sa;
    nx_ = tmp;
  }

  /// @brief Rotate line about point at distance pivot along line.
  inline void rotate(FT angle, const FT pivot) { this->rotateImpl(angle, pivot); }

  /// @brief Rotate line about point (x,y).
  inline void rotate(FT angle, FT x, FT y) { this->rotateImpl(angle, point_type(x, y)); }

  /// @brief Rotate line about pivot point.
  inline void rotate(float_type angle, const point_type& pivot) { this->rotateImpl(angle, pivot); }

  /// @brief Scale line about origin.
  virtual void scale(FT s) {
    // update distance
    d_ *= s;
  }

  /// @brief Scale line about point (x,y).
  inline void scale(float_type s, FT x, FT y) { this->scaleImpl(s, point_type(x, y)); }

  /// @brief Scale line about pivot point.
  inline void scale(float_type s, const point_type& pivot) { this->scaleImpl(s, pivot); }

  /// @brief Flip normal direction (reverses line orientation).
  virtual void normalFlip() {
    nx_ *= FT(-1);
    ny_ *= FT(-1);
    d_ *= FT(-1);
  }
  /// @}


  /// @name Drawing
  /// @{

  /// @brief Draw line on image.
  /// @param img Output image.
  /// @param color Line color.
  /// @param thickness Line thickness.
  /// @param lineType OpenCV line type (default 8-connected).
  /// @param normalLength Length of normal indicator (0 to disable).
  /// @param tipLength Length of arrow tip (0 to disable).
  inline void draw(cv::Mat& img,
                   cv::Scalar color,
                   int thickness = 1,
                   int lineType = 8,
                   double normalLength = 0,
                   double tipLength = 0) const {
    this->drawImpl(img, color, thickness, lineType, normalLength, tipLength);
  }
  /// @}


 protected:
  /// @brief Implementation of translate by vector.
  virtual void translateImpl(const point_type& dist) { d_ += normalProject(dist); }

  /// @brief Implementation of rotate about point.
  virtual void rotateImpl(FT angle, const point_type& pivot) {
    FT sa = static_cast<FT>(sin(angle)), ca = static_cast<FT>(cos(angle));
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

  /// @brief Implementation of rotate about point on line.
  virtual void rotateImpl(FT angle, FT pivot) {
    FT sa = static_cast<FT>(sin(angle)), ca = static_cast<FT>(cos(angle));

    // get point for pivot
    point_type p = lineDist(pivot, origin());

    // rotate normal
    FT tmp = nx_ * ca - ny_ * sa;
    ny_ = ny_ * ca + nx_ * sa;
    nx_ = tmp;

    // update distance using p, since p is still on line
    translateTo(p);
  }

  /// @brief Implementation of scale about point.
  virtual void scaleImpl(FT s, const point_type& pivot) {
    FT dpv = normalProject(pivot);
    d_ -= dpv;
    d_ *= s;
    d_ += dpv;
  }

  /// @brief Implementation of draw method.
  virtual void drawImpl(
      cv::Mat& img, cv::Scalar color, int thickness, int lineType, double normalLength, double tipLength) const;

 public:
  /// @brief Virtual destructor for proper inheritance.
  virtual ~Line() = default;
};

/// @name Line type aliases
/// @{
template <class FT, template <class> class PT = Vec2>
using Line2 = Line<FT, PT>;  ///< 2D Line alias.

typedef Line2<float> Line2f;   ///< 2D float line.
typedef Line2<double> Line2d;  ///< 2D double line.

template <class FT, template <class> class PT = Vec2>
using Line2Vector = std::vector<Line2<FT, PT>>;  ///< Vector of 2D lines.

typedef Line2Vector<float> Line2Vectorf;   ///< Vector of float lines.
typedef Line2Vector<double> Line2Vectord;  ///< Vector of double lines.
/// @}

/// @brief 2D line segment with finite extent.
///
/// Extends Line with start/end distances along the line direction,
/// defining a finite segment. Supports clipping, overlap detection,
/// and segment-specific transformations.
/// @tparam FT Floating point type.
/// @tparam PT Point template.
template <class FT, template <class> class PT = Vec2>
class LineSegment : public Line<FT, PT> {
 protected:
  using Line<FT, PT>::nx_;
  using Line<FT, PT>::ny_;
  using Line<FT, PT>::d_;

  FT beg_, end_;  ///< Start and end distances along line direction.
  int octave_;    ///< Octave level where segment was detected.

 public:
  typedef FT float_type;      ///< Scalar type.
  typedef PT<FT> point_type;  ///< Point type.

  /// @brief Default constructor (zero segment).
  LineSegment() : Line<FT, PT>(), beg_(0), end_(0), octave_(0) {}

  /// @brief Copy constructor with optional octave.
  template <class AFT, template <class> class APT>
  LineSegment(const LineSegment<AFT, APT>& ls, int octave = 0)
      : Line<FT, PT>(ls), beg_(ls.start()), end_(ls.end()), octave_(octave) {}

  /// @brief Construct from normal angle, distance, and line extents.
  LineSegment(FT normal_ang, FT distance, FT line_beg, FT line_end, int octave = 0)
      : Line<FT, PT>(normal_ang, distance), beg_(line_beg), end_(line_end), octave_(octave) {
    if (beg_ > end_) flip();
  }

  /// @brief Construct from normal components, distance, and line extents.
  LineSegment(FT normal_x, FT normal_y, FT distance, FT line_beg, FT line_end, int octave = 0)
      : Line<FT, PT>(normal_x, normal_y, distance), beg_(line_beg), end_(line_end), octave_(octave) {
    if (beg_ > end_) flip();
  }

  /// @brief Construct from normal, origin point, and line extents.
  LineSegment(FT normal_x, FT normal_y, FT dist_x, FT dist_y, FT line_beg, FT line_end, int octave = 0)
      : Line<FT, PT>(normal_x, normal_y, dist_x, dist_y), beg_(line_beg), end_(line_end), octave_(octave) {
    if (beg_ > end_) flip();
  }

  /// @brief Construct from normal, origin point, and endpoint coordinates.
  LineSegment(FT normal_x, FT normal_y, FT dist_x, FT dist_y, FT beg_x, FT beg_y, FT end_x, FT end_y, int octave = 0)
      : Line<FT, PT>(normal_x, normal_y, dist_x, dist_y), beg_(0), end_(0), octave_(octave) {
    beg_ = ny_ * beg_x - nx_ * beg_y;
    end_ = ny_ * end_x - nx_ * end_y;

    if (beg_ > end_) flip();
  }


  /// @brief Construct from normal, distance origin, and endpoint vectors.
  LineSegment(
      const point_type& normal, const point_type& dist, const point_type& beg, const point_type& end, int octave = 0)
      : Line<FT, PT>(getX(normal), getY(normal), getX(dist), getY(dist)), beg_(0), end_(0), octave_(octave) {
    beg_ = this->project(beg);
    end_ = this->project(end);

    if (beg_ > end_) flip();
  }


  /// @brief Construct from two endpoint points.
  /// @tparam APT Point type.
  /// @param beg Start point.
  /// @param end End point.
  /// @param octave Detection octave (default 0).
  template <class APT>
  LineSegment(const APT& beg, const APT& end, int octave = 0)
      : Line<FT, PT>(beg, end), beg_(0), end_(0), octave_(octave) {
    beg_ = this->project(getX(beg), getY(beg));
    end_ = this->project(getX(end), getY(end));

    if (beg_ > end_) flip();
  }

  /// @brief Construct from Vec4 containing endpoints [x1,y1,x2,y2].
  LineSegment(const Vec4<FT>& points, int octave = 0) : Line<FT, PT>(points), beg_(0), end_(0), octave_(octave) {
    beg_ = ny_ * points[0] - nx_ * points[1];
    end_ = ny_ * points[2] - nx_ * points[3];

    if (beg_ > end_) flip();
  }

  /// @brief Construct from raw pointer to endpoints [x1,y1,x2,y2].
  LineSegment(const FT* points, int octave = 0) : Line<FT, PT>(points), beg_(0), end_(0), octave_(octave) {
    beg_ = ny_ * points[0] - nx_ * points[1];
    end_ = ny_ * points[2] - nx_ * points[3];

    if (beg_ > end_) flip();
  }

  /// @brief Construct from Line and endpoint points.
  template <class AFT, template <class> class APT, class APT2>
  LineSegment(const Line<AFT, APT>& l, const APT2& beg, const APT2& end, int octave = 0)
      : Line<FT, PT>(l), beg_(0), end_(0), octave_(octave) {
    beg_ = this->project(static_cast<FT>(getX(beg)), static_cast<FT>(getY(beg)));
    end_ = this->project(static_cast<FT>(getX(end)), static_cast<FT>(getY(end)));

    if (beg_ > end_) flip();
  }

  /// @brief Construct from Line and distance extents.
  LineSegment(const Line<FT, PT>& l, FT line_beg, FT line_end, int octave = 0)
      : Line<FT, PT>(l), beg_(line_beg), end_(line_end), octave_(octave) {
    if (beg_ > end_) flip();
  }

  /// @brief Convert to different floating-point/point type.
  /// @tparam newFT New floating-point type.
  /// @tparam newPT New point template.
  /// @return Converted line segment.
  template <class newFT, template <class> class newPT = Vec2>
  LineSegment<newFT, newPT> convertTo() {
    return LineSegment<newFT, newPT>(static_cast<newFT>(nx_), static_cast<newFT>(ny_), static_cast<newFT>(d_),
                                     static_cast<newFT>(beg_), static_cast<newFT>(end_), octave_);
  }

  /// @name Attribute accessors
  /// @{

  /// @brief Get start distance along line direction.
  inline FT start() const { return beg_; }

  /// @brief Get end distance along line direction.
  inline FT end() const { return end_; }

  /// @brief Get center distance along line direction.
  inline FT center() const { return beg_ + (end_ - beg_) / FT(2); }

  /// @brief Get start point.
  inline point_type startPoint() const { return this->lineDist(beg_, this->origin()); }
  /// @brief Get start point (alias).
  inline point_type getStartPoint() const { return startPoint(); }

  /// @brief Get end point.
  inline point_type endPoint() const { return this->lineDist(end_, this->origin()); }
  /// @brief Get end point (alias).
  inline point_type getEndPoint() const { return endPoint(); }

  /// @brief Get endpoints as Vec4 [x1,y1,x2,y2].
  inline Vec4<FT> endPoints() const {
    FT dx = this->originX(), dy = this->originY();
    return Vec4<FT>(dx + ny_ * beg_, dy - nx_ * beg_, dx + ny_ * end_, dy - nx_ * end_);
  }

  /// @brief Get endpoints as separate vectors.
  inline void endPoints(point_type& vbeg, point_type& vend) const {
    FT dx = this->originX(), dy = this->originY();
    set(vbeg, dx + ny_ * beg_, dy - nx_ * beg_);
    set(vend, dx + ny_ * end_, dy - nx_ * end_);
  }

  /// @brief Get center point of segment.
  inline point_type centerPoint() const { return this->lineDist(this->center(), this->origin()); }

  /// @brief Get segment length.
  inline FT length() const { return detail::abs(end_ - beg_); }

  /// @brief Get detection octave.
  inline int octave() const { return octave; }

  /// @brief Set detection octave.
  inline void octave(int oct) const { this->octave_ = oct; }
  /// @}

  /// @name Range checking
  /// @{

  /// @brief Check if distance is within segment range.
  inline bool inRange(FT dist) const { return dist >= beg_ && dist <= end_; }

  /// @brief Check if distance is within segment range with tolerance.
  inline bool inRangeTol(FT dist, FT tol) const { return dist >= (beg_ - tol) && dist <= (end_ + tol); }

  /// @brief Check if point projection is within segment range.
  inline bool inRange(FT x, FT y) const {
    FT dist = this->project(x, y);
    return inRange(dist);
  }

  /// @brief Check if point projection is within segment range with tolerance.
  inline bool inRangeTol(FT x, FT y, FT tol) const {
    FT dist = this->project(x, y);
    return inRangeTol(dist, tol);
  }

  /// @brief Check if point projection is within segment range.
  inline bool inRange(const point_type& v) const { return inRange(getX(v), getY(v)); }

  /// @brief Check if point projection is within segment range with tolerance.
  inline bool inRangeTol(const point_type& v, FT tol) const { return inRangeTol(getX(v), getY(v), tol); }
  /// @}


  /// @brief Trim segment to fit within a bounding box.
  /// @param max_x Maximum x (box width).
  /// @param max_y Maximum y (box height).
  /// @param min_x Minimum x (default 0).
  /// @param min_y Minimum y (default 0).
  /// @return True if segment overlaps box, false if entirely outside.
  inline bool trim2Box(FT max_x, FT max_y, FT min_x = FT(0), FT min_y = FT(0)) {
    // Use tolerance-based comparison for nearly horizontal/vertical lines
    const FT tol = LIMITS<FT>::eps() * FT(100);

    // only test left and right (nearly horizontal line)
    FT tmp;
    if (detail::abs(this->nx_) < tol) {
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
    // only test upper and lower (nearly vertical line)
    if (detail::abs(this->ny_) < tol) {
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

    // Initialize intersection points to zero (not the endpoints!)
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

  /// @brief Check if two segments overlap (asymmetric test).
  /// @param line Other segment.
  /// @return True if projections overlap.
  /// @note Due to projection, overlap may not be symmetric.
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

  /// @name Segment transformations
  /// @{

  /// @brief Translate segment along line direction.
  inline void translate(FT dist) {
    beg_ += dist;
    end_ += dist;
  }

  /// @brief Translate start point along line.
  inline void translateStart(FT dist) { beg_ += dist; }

  /// @brief Translate end point along line.
  inline void translateEnd(FT dist) { end_ += dist; }

  /// @brief Rotate segment about point at distance pivot along line.
  inline void rotate(FT angle, FT pivot) { this->rotateImpl(angle, pivot); }

  /// @brief Scale segment about origin.
  virtual void scale(FT s) {
    Line<FT, PT>::scale(s);
    beg_ *= s;
    end_ *= s;
  }

  /// @brief Scale segment about point at distance pivot along line.
  inline void scale(FT s, FT pivot) {
    // scale endpoints
    beg_ -= pivot;
    beg_ *= s;
    beg_ += pivot;
    end_ -= pivot;
    end_ *= s;
    end_ += pivot;
  }

  /// @brief Flip normal direction (reverses segment orientation).
  virtual void normalFlip() {
    Line<FT, PT>::normalFlip();
    beg_ *= FT(-1);
    end_ *= FT(-1);
    std::swap(beg_, end_);
  }

  /// @brief Swap start and end points.
  inline void endPointSwap() { std::swap(beg_, end_); }
  /// @}

  /// @name Error measurement
  /// @{

  /// @brief Calculate endpoint distance error to ground truth line.
  /// @param gtLine Ground truth line.
  /// @return Sum of distances from endpoints to ground truth.
  inline FT error(const Line<FT, PT>& gtLine) const {
    FT a, b;
    return error(gtLine, a, b);
  }

  /// @brief Calculate endpoint distance error with individual distances.
  /// @param gtLine Ground truth line.
  /// @param[out] a Distance from start point to ground truth.
  /// @param[out] b Distance from end point to ground truth.
  /// @return Sum of distances.
  inline FT error(const Line<FT, PT>& gtLine, FT& a, FT& b) const {
    if (this->empty() || gtLine.empty()) return FT(0);
    a = gtLine.distance(this->startPoint());
    b = gtLine.distance(this->endPoint());

    if (a < FT(0)) a = a * FT(-1.0);
    if (b < FT(0)) b = b * FT(-1.0);

    return a + b;
  }
  /// @}


 protected:
  /// @brief Flip orientation (used during construction).
  void flip() {
    Line<FT, PT>::normalFlip();
    beg_ *= FT(-1);
    end_ *= FT(-1);
  }

  /// @brief Implementation of translate by vector.
  virtual void translateImpl(const point_type& dist) {
    this->d_ += this->normalProject(dist);
    FT tmp = this->project(dist);
    beg_ += tmp;
    end_ += tmp;
  }

  /// @brief Implementation of rotate about point.
  virtual void rotateImpl(FT angle, const point_type& pivot) {
    FT sa = static_cast<FT>(sin(angle)), ca = static_cast<FT>(cos(angle));

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

  /// @brief Implementation of rotate about point on line.
  virtual void rotateImpl(FT angle, FT pivot) {
    FT sa = static_cast<FT>(sin(angle)), ca = static_cast<FT>(cos(angle));

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

  /// @brief Implementation of scale about point.
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

  /// @brief Implementation of draw method.
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

  /// @brief Stream output operator for line segments.
  /// @tparam U Floating-point type of the friend function.
  /// @param os Output stream.
  /// @param ls Line segment to output.
  /// @return Reference to the output stream.
  template <class U>
  friend std::ostream& operator<<(std::ostream& os, const LineSegment<U>& ls);

 public:
  /// @brief Virtual destructor for proper inheritance.
  virtual ~LineSegment() = default;
};

/// @brief Stream output operator for LineSegment.
///
/// Outputs the line segment in the format: "S: sx, sy  E: ex, ey"
/// where (sx, sy) is the start point and (ex, ey) is the end point.
/// @tparam FT Floating-point type for coordinates.
/// @param os Output stream.
/// @param ls Line segment to output.
/// @return Reference to the output stream.
template <class FT>
std::ostream& operator<<(std::ostream& os, const LineSegment<FT>& ls) {
  return os << "S: " << ls.startPoint()[0] << ", " << ls.startPoint()[1] << "  E: " << ls.endPoint()[0] << ", "
            << ls.endPoint()[1];
}


/// @brief Alias template for 2D line segment.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template (default Vec2).
template <class FT, template <class> class PT = Vec2>
using LineSegment2 = LineSegment<FT, PT>;

/// @brief Single-precision 2D line segment.
typedef LineSegment<float> LineSegment2f;

/// @brief Double-precision 2D line segment.
typedef LineSegment<double> LineSegment2d;

/// @brief Alias template for vector of 2D line segments.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template (default Vec2).
template <class FT, template <class> class PT = Vec2>
using LineSegment2Vector = std::vector<LineSegment2<FT, PT>>;

/// @brief Vector of single-precision 2D line segments.
typedef LineSegment2Vector<float> LineSegment2Vectorf;

/// @brief Vector of double-precision 2D line segments.
typedef LineSegment2Vector<double> LineSegment2Vectord;

/// @brief Trim an infinite line to a bounding box.
///
/// Creates a line segment by trimming the infinite line to fit within
/// the specified rectangular bounding box. If the line does not intersect
/// the box, returns an empty line segment.
/// @tparam FT Floating-point type for coordinates.
/// @tparam LPT Point type template.
/// @param l Line to trim.
/// @param max_x Maximum x coordinate (typically image width).
/// @param max_y Maximum y coordinate (typically image height).
/// @param min_x Minimum x coordinate (default 0).
/// @param min_y Minimum y coordinate (default 0).
/// @return Line segment trimmed to the bounding box, or empty if outside.
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

/// @brief Implementation of line drawing.
///
/// Draws the line by first trimming it to the image bounds using trim2Box(),
/// then delegating to the line segment's draw method.
/// @param img Image to draw on.
/// @param color Drawing color.
/// @param thickness Line thickness in pixels.
/// @param lineType OpenCV line type (LINE_8, LINE_4, LINE_AA).
/// @param normalLength Length of normal vector visualization (0 to disable).
/// @param tipLength Arrow tip length for normal visualization.
template <class FT, template <class> class LPT>
void Line<FT, LPT>::drawImpl(
    cv::Mat& img, cv::Scalar color, int thickness, int lineType, double normalLength, double tipLength) const {
  trim2Box<FT, LPT>(*this, FT(img.cols), FT(img.rows)).draw(img, color, thickness, lineType, normalLength, tipLength);
}

}  // namespace lsfm
