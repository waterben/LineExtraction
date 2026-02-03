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
//M*/

#pragma once

#include <edge/edge_segment.hpp>
#include <lsd/ld_base.hpp>

namespace lsfm {

/// @brief Base class for line segment detectors.
/// Extends the line detector base class with support for line segments and endpoints.
/// Line segments include geometric information like precise endpoints and orientation.
/// @tparam FT Floating-point type (e.g., float, double)
/// @tparam LPT Line point template (default Vec2)
template <class FT, template <class> class LPT = Vec2>
class LsdBase : public LdBase<FT, LPT> {
 public:
  typedef FT float_type;       ///< Floating-point type for coordinates
  typedef LPT<FT> line_point;  ///< Line point type
  template <class A>
  using line_point_template = LPT<A>;  ///< Line point template alias

  typedef typename LdBase<FT, LPT>::Line Line;              ///< Line type (from base class)
  typedef typename LdBase<FT, LPT>::LineVector LineVector;  ///< Vector of lines
  typedef lsfm::LineSegment<FT, LPT> LineSegment;           ///< Line segment type with endpoints
  typedef std::vector<LineSegment> LineSegmentVector;       ///< Vector of line segments
  typedef Vec4<FT> EndPoint;                                ///< Endpoint vector: (x1, y1, x2, y2)
  typedef std::vector<EndPoint> EndPointVector;             ///< Vector of endpoint pairs
  typedef typename LdBase<FT, LPT>::ImageData ImageData;    ///< Auxiliary image data

  /// @brief Virtual destructor for proper cleanup.
  virtual ~LsdBase() {}

  using LdBase<FT, LPT>::detect;
  using LdBase<FT, LPT>::imageDataDescriptor;
  using LdBase<FT, LPT>::imageData;

  /// @brief Convenience method: Detect lines and get line segment results.
  /// @param image Input image
  /// @param l Output vector of line segments
  inline void detect(const cv::Mat& image, LineSegmentVector& l) {
    detect(image);
    l = lineSegments();
  }

  /// @brief Convenience method: Detect lines and get line segments plus auxiliary data.
  /// @param image Input image
  /// @param l Output vector of line segments
  /// @param id Output auxiliary image data (gradients, edges, etc.)
  inline void detect(const cv::Mat& image, LineSegmentVector& l, ImageData& id) {
    detect(image);
    l = lineSegments();
    id = imageData();
  }

  /// @brief Convenience method: Detect lines and get endpoint results.
  /// @param image Input image
  /// @param e Output vector of endpoints (each Vec4 is x1, y1, x2, y2)
  inline void detect(const cv::Mat& image, EndPointVector& e) {
    detect(image);
    e = endPoints();
  }

  /// @brief Convenience method: Detect lines and get endpoints plus auxiliary data.
  /// @param image Input image
  /// @param e Output vector of endpoints
  /// @param id Output auxiliary image data
  inline void detect(const cv::Mat& image, EndPointVector& e, ImageData& id) {
    detect(image);
    e = endPoints();
    id = imageData();
  }

  /// @brief Get detected lines as a line vector (converts from segments if needed).
  /// @return Const reference to vector of detected lines
  virtual const LineVector& lines() const override {
    if (lineSegments_.size() && lines_.empty()) {
      for_each(lineSegments_.begin(), lineSegments_.end(),
               [this](const LineSegment& ls) { this->lines_.push_back(ls); });
    }
    return lines_;
  }

  /// @brief Get detected lines as line segments with full geometric information.
  /// @return Const reference to vector of line segments (includes endpoints and properties)
  virtual const LineSegmentVector& lineSegments() const { return lineSegments_; }

  /// @brief Get detected line endpoints as 4-element vectors.
  /// Each element is (x1, y1, x2, y2) where (x1,y1) and (x2,y2) are the line endpoints.
  /// The orientation is determined by gradient direction.
  /// @return Const reference to vector of endpoint vectors
  /// @example
  /// @code{cpp}
  /// detector->detect(image);
  /// const auto& endpoints = detector->endPoints();
  /// for (const auto& ep : endpoints) {
  ///   float x1 = ep[0], y1 = ep[1];
  ///   float x2 = ep[2], y2 = ep[3];
  /// }
  /// @endcode
  virtual const EndPointVector& endPoints() const {
    if (lineSegments_.size() && endPoints_.empty()) {
      endPoints_.reserve(lineSegments_.size());
      for_each(lineSegments_.begin(), lineSegments_.end(),
               [this](const LineSegment& l) { endPoints_.push_back(l.endPoints()); });
    }
    return endPoints_;
  }

 protected:
  /// @brief Protected constructor for derived classes.
  LsdBase() {}

  using LdBase<FT, LPT>::lines_;
  mutable LineSegmentVector lineSegments_{};  ///< Storage for line segments
  mutable EndPointVector endPoints_{};        ///< Storage for endpoint vectors (lazy computed)

  /// @brief Clear all internal detection data.
  virtual void clearData() override {
    lines_.clear();
    lineSegments_.clear();
    endPoints_.clear();
  }
};

/// @brief Extended interface for line segment detectors with internal data access.
/// Provides access to low-level detection data such as support point lists and edge segments.
/// Useful for detailed analysis and debugging of detection algorithms.
/// @tparam FT Floating-point type (e.g., float, double)
/// @tparam LPT Line point template (default Vec2)
/// @tparam PT Point type for support points (default Vec2i)
template <class FT, template <class> class LPT = Vec2, class PT = Vec2i>
class LsdExt : public LsdBase<FT, LPT> {
 public:
  typedef FT float_type;       ///< Floating-point type
  typedef LPT<FT> line_point;  ///< Line point type
  typedef PT point_type;       ///< Support point type

  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;              ///< Line segment type
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;  ///< Vector of line segments
  typedef typename LsdBase<FT, LPT>::EndPoint EndPoint;                    ///< Endpoint vector
  typedef typename LsdBase<FT, LPT>::EndPointVector EndPointVector;        ///< Vector of endpoints
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;                  ///< Image data type
  typedef std::vector<PT> PointVector;                                     ///< Vector of support points

  /// @brief Virtual destructor for proper cleanup.
  virtual ~LsdExt(){};

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;
  using LdBase<FT, LPT>::imageDataDescriptor;
  using LdBase<FT, LPT>::imageData;

  /// @brief Get edge segments supporting each detected line.
  /// Returns a vector of edge segments where each element corresponds to the support
  /// region for the line at the same index in lineSegments().
  /// @return Const reference to vector of edge segments
  virtual const EdgeSegmentVector& lineSupportSegments() const = 0;

  /// @brief Get all support points used in line detection.
  /// Returns a point list where individual edge segments can index into this array
  /// to retrieve their supporting points.
  /// @return Const reference to vector of support points
  virtual const PointVector& points() const = 0;

  /// @brief Get index arrays for edge segments.
  /// Each edge segment can be indexed to retrieve the indices of its support points
  /// in the points() array.
  /// @return Const reference to vector of point indices
  virtual const IndexVector& indexes() const = 0;

 protected:
  LsdExt() {}
  using LsdBase<FT, LPT>::lines_;
  using LsdBase<FT, LPT>::lineSegments_;
  using LsdBase<FT, LPT>::endPoints_;
};

}  // namespace lsfm
