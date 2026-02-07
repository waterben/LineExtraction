//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file polygon.hpp
/// @brief 2D polygon representation.
///
/// This file provides a class for representing 2D polygons defined by
/// vertices relative to a pivot point. Features include:
/// - Construction from line segments (intersection-based)
/// - Vertex and edge access
/// - Transformation operations (translate, rotate, scale)
/// - OpenCV drawing and filling
/// - Convexity testing
/// @see line.hpp for 2D line segment representation.

#pragma once

#include <geometry/line.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>


namespace lsfm {

/// @brief 2D polygon representation.
///
/// Represents a polygon as a list of vertices relative to a pivot point.
/// The pivot serves as the local coordinate origin for all vertices,
/// allowing efficient translation of the entire polygon.
/// @tparam FT Floating-point type for coordinates.
/// @tparam PT Point type template (default Vec2).
template <class FT, template <class> class PT = Vec2>
class Polygon {
  PT<FT> piviot_;                  ///< Pivot point (local origin).
  std::vector<PT<FT>> verticies_;  ///< Vertices in local coordinates.

 public:
  typedef FT float_type;                         ///< Floating-point type alias.
  typedef PT<FT> point_type;                     ///< Point type alias.
  typedef std::vector<point_type> VertexVector;  ///< Vertex container type.

  /// @brief Construct empty polygon with given pivot.
  /// @param piviot Pivot point (default origin).
  Polygon(const point_type& piviot = point_type(0, 0)) : piviot_(piviot), verticies_() {}

  /// @brief Virtual destructor for proper inheritance.
  virtual ~Polygon() = default;

  /// @brief Construct polygon from line segments.
  ///
  /// Creates vertices at the intersection points of consecutive segments.
  /// Useful for constructing polygons from detected edges.
  /// @param segments Vector of line segments forming the polygon.
  /// @param piviot Pivot point (default origin).
  /// @note Requires at least 2 segments.
  Polygon(const LineSegment2Vector<FT, PT>& segments, const point_type& piviot = point_type(0, 0))
      : piviot_(piviot), verticies_() {
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

  /// @name Vertex Access
  /// @{

  /// @brief Get mutable reference to vertices.
  /// @return Reference to vertex vector (local coordinates).
  inline VertexVector& verticies() { return verticies_; }

  /// @brief Get const reference to vertices.
  /// @return Const reference to vertex vector (local coordinates).
  inline const VertexVector& verticies() const { return verticies_; }

  /// @brief Get polygon edges as line segments.
  ///
  /// Creates line segments connecting consecutive vertices,
  /// including an edge from the last to the first vertex.
  /// @return Vector of line segments (local coordinates).
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

  /// @brief Get vertices in world coordinates.
  /// @return Vertices transformed by pivot offset.
  inline VertexVector wolrdVerticies() const {
    VertexVector ret;
    ret.reserve(verticies_.size());
    for_each(verticies_.begin(), verticies_.end(), [&, this](point_type& p) { ret.push_back(p + this->piviot_); });
    return ret;
  }

  /// @brief Get edges in world coordinates.
  /// @return Line segments transformed by pivot offset.
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

  /// @}

  /// @name Vertex Modification
  /// @{

  /// @brief Clear all vertices.
  inline void clear() const { verticies_.clear(); }

  /// @brief Add vertex in local coordinates.
  /// @param vertex Vertex to add (local coordinates).
  inline void push_back(const point_type& vertex) { verticies_.push_back(vertex); }

  /// @brief Add vertex in world coordinates.
  ///
  /// The vertex is converted to local coordinates by subtracting
  /// the pivot position.
  /// @param vertex Vertex to add (world coordinates).
  inline void push_back_world(const point_type& vertex) { verticies_.push_back(vertex - piviot_); }

  /// @brief Check if polygon has no vertices.
  /// @return True if vertex list is empty.
  inline bool empty() const { return verticies_.empty(); }

  /// @brief Get number of vertices.
  /// @return Vertex count.
  inline size_t size() const { return verticies_.size(); }

  /// @}

  /// @name Transformations
  /// @{

  /// @brief Translate polygon by moving pivot.
  /// @param t Translation vector.
  inline void translate(const point_type& t) { piviot_ += t; }

  /// @brief Rotate polygon around pivot.
  ///
  /// Rotates all vertices around the pivot point.
  /// @param angle Rotation angle in radians.
  inline void rotate(FT angle) {
    FT sa = static_cast<FT>(sin(angle)), ca = static_cast<FT>(cos(angle));
    for_each(verticies_.begin(), verticies_.end(),
             [&](point_type& p) { set(p, getX(p) * ca - getY(p) * sa, getX(p) * sa + getY(p) * ca); });
  }

  /// @brief Rotate polygon around arbitrary point.
  /// @param angle Rotation angle in radians.
  /// @param o Center of rotation.
  inline void rotate(FT angle, const point_type& o) {
    FT sa = sin(angle), ca = cos(angle);
    point_type t = piviot_ - o;
    for_each(verticies_.begin(), verticies_.end(), [&](point_type& p) {
      p += t;
      set(p, getX(p) * ca - getY(p) * sa, getX(p) * sa + getY(p) * ca);
      p -= t;
    });
  }

  /// @brief Scale polygon relative to pivot.
  /// @param s Scale factor.
  virtual void scale(FT s) {
    for_each(verticies_.begin(), verticies_.end(), [&, this](point_type& p) { p *= s; });
  }

  /// @brief Scale polygon relative to arbitrary point.
  /// @param s Scale factor.
  /// @param o Center of scaling.
  virtual void scale(FT s, const point_type& o) {
    point_type t = piviot_ - o;
    for_each(verticies_.begin(), verticies_.end(), [&, this](point_type& p) {
      p += t;
      p *= s;
      p -= t;
    });
  }

  /// @}

  /// @name Pivot Operations
  /// @{

  /// @brief Get mutable reference to pivot.
  /// @return Reference to pivot point.
  inline point_type& piviot() { return piviot_; }

  /// @brief Get const reference to pivot.
  /// @return Const reference to pivot point.
  inline const point_type& piviot() const { return piviot_; }

  /// @brief Move pivot while keeping polygon in place.
  ///
  /// Adjusts all vertices to compensate for the pivot movement,
  /// so the polygon remains at the same world position.
  /// @param t Translation to apply to pivot.
  inline void translatePiviot(const point_type& t) {
    for_each(verticies_.begin(), verticies_.end(), [&](point_type& p) { p -= t; });
    piviot_ += t;
  }

  /// @}


  /// @name Drawing
  /// @{

  /// @brief Fill convex polygon on image.
  ///
  /// Uses OpenCV's fillConvexPoly for efficient filling of convex polygons.
  /// @param img Image to draw on.
  /// @param color Fill color.
  /// @param lineType OpenCV line type (default 8-connected).
  /// @note Only works correctly for convex polygons.
  inline void fill(cv::Mat& img, const cv::Scalar& color, int lineType = 8) const {
    std::vector<cv::Point> in;
    in.reserve(verticies_.size());
    point_type tmp;
    for_each(verticies_.begin(), verticies_.end(), [&, this](const point_type& p) {
      tmp = p + this->piviot_;
      in.push_back(cv::Point(static_cast<int>(std::round(getX(tmp))), static_cast<int>(std::round(getY(tmp)))));
    });
    cv::fillConvexPoly(img, in.data(), static_cast<int>(verticies_.size()), color, lineType);
  }

  /// @brief Fill potentially non-convex polygon on image.
  ///
  /// Uses OpenCV's fillPoly which handles complex (non-convex) polygons.
  /// @param img Image to draw on.
  /// @param color Fill color.
  /// @param lineType OpenCV line type (default 8-connected).
  inline void fillComplex(cv::Mat& img, const cv::Scalar& color, int lineType = 8) const {
    std::vector<cv::Point> in;
    in.reserve(verticies_.size());
    point_type tmp;

    for_each(verticies_.begin(), verticies_.end(), [&, this](const point_type& p) {
      tmp = p + this->piviot_;
      in.push_back(cv::Point(static_cast<int>(std::round(getX(tmp))), static_cast<int>(std::round(getY(tmp)))));
    });
    int np = static_cast<int>(verticies_.size());
    const cv::Point* data = in.data();
    cv::fillPoly(img, &data, &np, 1, color, lineType);
  }

  /// @brief Draw polygon outline on image.
  /// @param img Image to draw on.
  /// @param color Line color.
  /// @param thickness Line thickness in pixels.
  /// @param lineType OpenCV line type (default 8-connected).
  inline void draw(cv::Mat& img, const cv::Scalar& color, int thickness = 1, int lineType = 8) const {
    std::vector<cv::Point> in;
    in.reserve(verticies_.size());
    point_type tmp;
    for_each(verticies_.begin(), verticies_.end(), [&, this](const point_type& p) {
      tmp = p + this->piviot_;
      in.push_back(cv::Point(static_cast<int>(std::round(getX(tmp))), static_cast<int>(std::round(getY(tmp)))));
    });
    int np = static_cast<int>(verticies_.size());
    const cv::Point* data = in.data();
    cv::polylines(img, &data, &np, 1, true, color, thickness, lineType);
  }

  /// @}

  /// @name Geometric Properties
  /// @{

  /// @brief Test if polygon is convex.
  ///
  /// Checks if all cross products of consecutive edges have the
  /// same sign, indicating the polygon turns consistently in one direction.
  /// @return True if polygon is convex, false otherwise.
  /// @note Returns false for polygons with fewer than 3 vertices.
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

  /// @}
};

/// @brief Alias for vector of polygons.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
template <class FT, template <class> class PT = Vec2>
using PolygonVector = std::vector<Polygon<FT, PT>>;

}  // namespace lsfm
