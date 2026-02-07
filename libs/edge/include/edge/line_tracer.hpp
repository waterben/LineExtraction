//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file line_tracer.hpp
/// @brief Line-oriented edge segment tracing and extraction.
/// Provides methods to trace edges with a preference for line-like structures,
/// breaking curves and emphasizing straight-line segments.

#pragma once

#include <edge/edge_segment.hpp>
#include <geometry/line.hpp>

#include <cstddef>

namespace lsfm {

/// @brief Line-oriented edge tracer for extracting straight edge segments.
/// Traces edges with a preference for line-like structures, breaking at curves
/// and emphasizing straight-line segments from binary or magnitude edge maps.
/// @tparam FT Floating-point type
/// @tparam LPT Line point template (default: Vec2)
/// @tparam PT Point type (default: Vec2i)
/// @tparam DM Direction map quantization (4 or 8, default: 8)
template <class FT, template <class> class LPT = Vec2, class PT = Vec2i, int DM = 8>
class LineTracer : public ValueManager {
 public:
  /// @typedef float_type
  /// @brief Floating-point type
  typedef FT float_type;

  /// @typedef point_type
  /// @brief Point type
  typedef PT point_type;

  /// @typedef PointVector
  /// @brief Vector of points
  typedef std::vector<PT> PointVector;

 private:
  int minPix_, maxGap_, searchSteps_;
  IndexVector indexes_;
  PointVector points_;
  EdgeSegmentVector segments_;

  cv::Mat edgeMap_;

  static int idxSelector_[21];

  void init() {
    this->add("edge_min_pix", std::bind(&LineTracer<FT, LPT, PT, DM>::valueMinPix, this, std::placeholders::_1),
              "Minimum line support pixels. Line LineTracer with less pixels than that are rejected.");
    this->add("edge_max_gap", std::bind(&LineTracer<FT, LPT, PT, DM>::valueMaxGap, this, std::placeholders::_1),
              "Maximum allowed gap between points on the same line to link them.");
    this->add("edge_Search_steps",
              std::bind(&LineTracer<FT, LPT, PT, DM>::valueSearchSteps, this, std::placeholders::_1),
              "Number of left and right searching steps of a given coodinate (Range: 0-10).");
  }

  void clear() {
    segments_.clear();
    indexes_.clear();
    points_.clear();
  }

 public:
  LineTracer(int minPix = 10, int maxGap = 3, int searchSteps = 2)
      : minPix_(minPix), maxGap_(maxGap), searchSteps_(searchSteps), indexes_(), points_(), segments_(), edgeMap_() {
    init();
  }

  LineTracer(const ValueManager::NameValueVector& options)
      : minPix_(10), maxGap_(3), searchSteps_(2), indexes_(), points_(), segments_(), edgeMap_() {
    init();
    this->value(options);
  }

  LineTracer(ValueManager::InitializerList options)
      : minPix_(10), maxGap_(3), searchSteps_(2), indexes_(), points_(), segments_(), edgeMap_() {
    init();
    this->value(options);
  }

  Value valueMaxGap(const Value& v = Value::NAV()) {
    if (v.type()) maxGap_ = v.getInt();
    return maxGap_;
  }

  Value valueMinPix(const Value& v = Value::NAV()) {
    if (v.type()) minPix_ = v.getInt();
    return minPix_;
  }

  Value valueSearchSteps(const Value& v = Value::NAV()) {
    if (v.type() && v.getInt() >= 0 && v.getInt() < 11) searchSteps_ = v.getInt();
    return searchSteps_;
  }

  inline const EdgeSegmentVector& segments() const { return segments_; }

  inline const IndexVector& indexes() const { return indexes_; }

  inline const PointVector& points() const { return points_; }

  inline cv::Mat edgeMap() const { return edgeMap_; }

  inline void traceBinary(const Line2Vector<FT, LPT>& lines, const cv::Mat& edgeMap) {
    clear();
    edgeMap_ = edgeMap.clone();
    segments_.reserve(lines.size() * 10);
    points_.reserve(lines.size() * 1000);
    indexes_.reserve(lines.size() * 1000);
    int id = 0;
    for_each(lines.begin(), lines.end(), [&, this](const Line<FT, LPT>& line) {
      traceBinary(line, edgeMap_, this->points_, this->indexes_, this->segments_, id++);
    });
  }

  inline void traceDirmap(const Line2Vector<FT, LPT>& lines, const cv::Mat& edgeMap) {
    clear();
    edgeMap_ = edgeMap.clone();
    segments_.reserve(lines.size() * 10);
    points_.reserve(lines.size() * 1000);
    indexes_.reserve(lines.size() * 1000);
    int id = 0;
    for_each(lines.begin(), lines.end(), [&, this](const Line<FT, LPT>& line) {
      traceDirmap(line, edgeMap_, this->points_, this->indexes_, this->segments_, id++);
    });
  }

  inline void traceDirmap(const Line2Vector<FT, LPT>& lines, const cv::Mat& edgeMap, const cv::Mat& dirMap) {
    clear();
    edgeMap_ = edgeMap.clone();
    segments_.reserve(lines.size() * 10);
    points_.reserve(lines.size() * 1000);
    indexes_.reserve(lines.size() * 1000);
    int id = 0;
    for_each(lines.begin(), lines.end(), [&, this](const Line<FT, LPT>& line) {
      traceDirmap(line, edgeMap_, dirMap, this->points_, this->indexes_, this->segments_, id++);
    });
  }

  inline void traceBinary(const LineSegment2Vector<FT, LPT>& lines, const cv::Mat& edgeMap) {
    clear();
    edgeMap_ = edgeMap.clone();
    segments_.reserve(lines.size() * 10);
    points_.reserve(lines.size() * 1000);
    indexes_.reserve(lines.size() * 1000);
    int id = 0;
    for_each(lines.begin(), lines.end(), [&, this](const LineSegment<FT, LPT>& line) {
      traceBinary(line, edgeMap_, this->points_, this->indexes_, this->segments_, id++);
    });
  }

  inline void traceDirmap(const LineSegment2Vector<FT, LPT>& lines, const cv::Mat& edgeMap) {
    clear();
    edgeMap_ = edgeMap.clone();
    segments_.reserve(lines.size() * 10);
    points_.reserve(lines.size() * 1000);
    indexes_.reserve(lines.size() * 1000);
    int id = 0;
    for_each(lines.begin(), lines.end(), [&, this](const LineSegment<FT, LPT>& line) {
      traceDirmap(line, edgeMap_, this->points_, this->indexes_, this->segments_, id++);
    });
  }

  inline void traceDirmap(const LineSegment2Vector<FT, LPT>& lines, const cv::Mat& edgeMap, const cv::Mat& dirMap) {
    clear();
    edgeMap_ = edgeMap.clone();
    segments_.reserve(lines.size() * 10);
    points_.reserve(lines.size() * 1000);
    indexes_.reserve(lines.size() * 1000);
    int id = 0;
    for_each(lines.begin(), lines.end(), [&, this](const LineSegment<FT, LPT>& line) {
      traceDirmap(line, edgeMap_, dirMap, this->points_, this->indexes_, this->segments_, id++);
    });
  }

  inline void traceBinary(const Line<FT, LPT>& line,
                          cv::Mat edgeMap,
                          PointVector& outSupportPoints,
                          IndexVector& outSupportIndexes,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    traceBinaryTrimmed(
        trim2Box<FT>(line, static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                     static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)),
        edgeMap, outSupportPoints, outSupportIndexes, outSegments, id);
  }

  inline void traceDirmap(const Line<FT, LPT>& line,
                          cv::Mat edgeMap,
                          PointVector& outSupportPoints,
                          IndexVector& outSupportIndexes,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    traceDirmapTrimmed(
        trim2Box<FT>(line, static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                     static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)),
        edgeMap, outSupportPoints, outSupportIndexes, outSegments, id);
  }

  inline void traceDirmap(const Line<FT, LPT>& line,
                          cv::Mat edgeMap,
                          const cv::Mat& dirMap,
                          PointVector& outSupportPoints,
                          IndexVector& outSupportIndexes,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    traceDirmapTrimmed(
        trim2Box<FT>(line, static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                     static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)),
        edgeMap, dirMap, outSupportPoints, outSupportIndexes, outSegments, id);
  }

  inline void traceBinary(const Line<FT, LPT>& line,
                          cv::Mat edgeMap,
                          PointVector& outSupportPoints,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    traceBinaryTrimmed(
        trim2Box<FT>(line, static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                     static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)),
        edgeMap, outSupportPoints, outSegments, id);
  }

  inline void traceDirmap(const Line<FT, LPT>& line,
                          cv::Mat edgeMap,
                          PointVector& outSupportPoints,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    traceDirmapTrimmed(
        trim2Box<FT>(line, static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                     static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)),
        edgeMap, outSupportPoints, outSegments, id);
  }

  inline void traceDirmap(const Line<FT, LPT>& line,
                          cv::Mat edgeMap,
                          const cv::Mat& dirMap,
                          PointVector& outSupportPoints,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    traceDirmapTrimmed(
        trim2Box<FT>(line, static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                     static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)),
        edgeMap, dirMap, outSupportPoints, outSegments, id);
  }

  inline void traceBinary(const LineSegment<FT, LPT>& line,
                          cv::Mat edgeMap,
                          PointVector& outSupportPoints,
                          IndexVector& outSupportIndexes,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    LineSegment<FT, LPT> lineTrimmed = line;
    if (lineTrimmed.trim2Box(static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                             static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)))
      traceBinaryTrimmed(lineTrimmed, edgeMap, outSupportPoints, outSupportIndexes, outSegments, id);
  }

  inline void traceDirmap(const LineSegment<FT, LPT>& line,
                          cv::Mat edgeMap,
                          PointVector& outSupportPoints,
                          IndexVector& outSupportIndexes,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    LineSegment<FT, LPT> lineTrimmed = line;
    if (lineTrimmed.trim2Box(static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                             static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)))
      traceDirmapTrimmed(lineTrimmed, edgeMap, outSupportPoints, outSupportIndexes, outSegments, id);
  }

  inline void traceDirmap(const LineSegment<FT, LPT>& line,
                          cv::Mat edgeMap,
                          const cv::Mat& dirMap,
                          PointVector& outSupportPoints,
                          IndexVector& outSupportIndexes,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    LineSegment<FT, LPT> lineTrimmed = line;
    if (lineTrimmed.trim2Box(static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                             static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)))
      traceDirmapTrimmed(lineTrimmed, edgeMap, dirMap, outSupportPoints, outSupportIndexes, outSegments, id);
  }

  inline void traceBinary(const LineSegment<FT, LPT>& line,
                          cv::Mat edgeMap,
                          PointVector& outSupportPoints,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    LineSegment<FT, LPT> lineTrimmed = line;
    if (lineTrimmed.trim2Box(static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                             static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)))
      traceBinaryTrimmed(lineTrimmed, edgeMap, outSupportPoints, outSegments, id);
  }

  inline void traceDirmap(const LineSegment<FT, LPT>& line,
                          cv::Mat edgeMap,
                          PointVector& outSupportPoints,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    LineSegment<FT, LPT> lineTrimmed = line;
    if (lineTrimmed.trim2Box(static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                             static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)))
      traceDirmapTrimmed(lineTrimmed, edgeMap, outSupportPoints, outSegments, id);
  }

  inline void traceDirmap(const LineSegment<FT, LPT>& line,
                          cv::Mat edgeMap,
                          const cv::Mat& dirMap,
                          PointVector& outSupportPoints,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    LineSegment<FT, LPT> lineTrimmed = line;
    if (lineTrimmed.trim2Box(static_cast<FT>(edgeMap.cols - searchSteps_), static_cast<FT>(edgeMap.rows - searchSteps_),
                             static_cast<FT>(searchSteps_), static_cast<FT>(searchSteps_)))
      traceDirmapTrimmed(lineTrimmed, edgeMap, dirMap, outSupportPoints, outSegments, id);
  }

  inline void traceBinaryTrimmed(const LineSegment<FT, LPT>& line,
                                 cv::Mat edgeMap,
                                 PointVector& outSupportPoints,
                                 IndexVector& outSupportIndexes,
                                 EdgeSegmentVector& outSegments,
                                 int id = 0) {
    if (line.empty()) return;

    LPT<FT> delta, startPoint, endPoint;
    size_t gapCount = 0, beg = outSupportIndexes.size(), pixCount = 0, steps = 0;

    if (prepare(line, delta, startPoint, endPoint, steps)) {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelBinaryX(startPoint, edgeMap, outSupportPoints, outSupportIndexes)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    } else {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelBinaryY(startPoint, edgeMap, outSupportPoints, outSupportIndexes)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    }
  }

  inline void traceDirmapTrimmed(const LineSegment<FT, LPT>& line,
                                 cv::Mat edgeMap,
                                 PointVector& outSupportPoints,
                                 IndexVector& outSupportIndexes,
                                 EdgeSegmentVector& outSegments,
                                 int id = 0) {
    if (line.empty()) return;

    char dir = mapDir<DM>(static_cast<float>(line.normalX()), static_cast<float>(line.normalY()));
    LPT<FT> delta, startPoint, endPoint;
    size_t gapCount = 0, beg = outSupportIndexes.size(), pixCount = 0, steps = 0;

    if (prepare(line, delta, startPoint, endPoint, steps)) {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelDirmapX(startPoint, edgeMap, dir, outSupportPoints, outSupportIndexes)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    } else {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelDirmapY(startPoint, edgeMap, dir, outSupportPoints, outSupportIndexes)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    }
  }

  inline void traceDirmapTrimmed(const LineSegment<FT, LPT>& line,
                                 cv::Mat edgeMap,
                                 const cv::Mat& dirMap,
                                 PointVector& outSupportPoints,
                                 IndexVector& outSupportIndexes,
                                 EdgeSegmentVector& outSegments,
                                 int id = 0) {
    if (line.empty()) return;

    char dir = mapDir<DM>(static_cast<float>(line.normalX()), static_cast<float>(line.normalY()));
    LPT<FT> delta, startPoint, endPoint;
    size_t gapCount = 0, beg = outSupportIndexes.size(), pixCount = 0, steps = 0;

    if (prepare(line, delta, startPoint, endPoint, steps)) {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelDirmapX(startPoint, edgeMap, dirMap, dir, outSupportPoints, outSupportIndexes)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    } else {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelDirmapY(startPoint, edgeMap, dirMap, dir, outSupportPoints, outSupportIndexes)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    }
  }

  inline void traceBinaryTrimmed(const LineSegment<FT, LPT>& line,
                                 cv::Mat edgeMap,
                                 PointVector& outSupportPoints,
                                 EdgeSegmentVector& outSegments,
                                 int id = 0) {
    if (line.empty()) return;

    LPT<FT> delta, startPoint, endPoint;
    size_t gapCount = 0, beg = outSupportPoints.size(), pixCount = 0, steps = 0;

    if (prepare(line, delta, startPoint, endPoint, steps)) {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelBinaryX(startPoint, edgeMap, outSupportPoints)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    } else {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelBinaryY(startPoint, edgeMap, outSupportPoints)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    }
  }

  inline void traceDirmapTrimmed(const LineSegment<FT, LPT>& line,
                                 cv::Mat edgeMap,
                                 PointVector& outSupportPoints,
                                 EdgeSegmentVector& outSegments,
                                 int id = 0) {
    if (line.empty()) return;

    char dir = mapDir<DM>(static_cast<float>(line.normalX()), -static_cast<float>(line.normalY()));
    LPT<FT> delta, startPoint, endPoint;
    size_t gapCount = 0, beg = outSupportPoints.size(), pixCount = 0, steps = 0;

    if (prepare(line, delta, startPoint, endPoint, steps)) {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelDirmapX(startPoint, edgeMap, dir, outSupportPoints)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    } else {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelDirmapY(startPoint, edgeMap, dir, outSupportPoints)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    }
  }

  inline void traceDirmapTrimmed(const LineSegment<FT, LPT>& line,
                                 cv::Mat edgeMap,
                                 const cv::Mat& dirMap,
                                 PointVector& outSupportPoints,
                                 EdgeSegmentVector& outSegments,
                                 int id = 0) {
    if (line.empty()) return;

    char dir = mapDir<DM>(static_cast<float>(line.normalX()), -static_cast<float>(line.normalY()));
    LPT<FT> delta, startPoint, endPoint;
    size_t gapCount = 0, beg = outSupportPoints.size(), pixCount = 0, steps = 0;

    if (prepare(line, delta, startPoint, endPoint, steps)) {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelDirmapX(startPoint, edgeMap, dirMap, dir, outSupportPoints)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    } else {
      for (size_t i = 0; i != steps; ++i, startPoint += delta) {
        if (checkPixelDirmapY(startPoint, edgeMap, dirMap, dir, outSupportPoints)) {
          gapCount = 0;
          ++pixCount;
        } else
          ++gapCount;
        if (gapCount > static_cast<size_t>(maxGap_)) {
          if (pixCount >= static_cast<size_t>(minPix_)) {
            outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
          }
          beg += pixCount;
          pixCount = 0;
          gapCount = 0;
        }
      }
      if (pixCount >= static_cast<size_t>(minPix_)) {
        outSegments.push_back(EdgeSegment(beg, beg + pixCount, id));
      }
    }
  }

  inline void tracePoints(const index_type* beg,
                          const index_type* end,
                          EdgeSegmentVector& outSegments,
                          size_t cols,
                          size_t offset = 0,
                          int id = 0) {
    const index_type* last = beg;
    const PT lastPoint, currentPoint;
    index2Point(*beg, lastPoint, cols);
    ptrdiff_t dist = 0;
    ++beg;
    for (; beg < end; ++beg) {
      index2Point(*beg, currentPoint, cols);
      if (std::abs(getX(lastPoint) - getX(currentPoint)) > maxGap_ ||
          std::abs(getY(lastPoint) - getY(currentPoint)) > maxGap_) {
        dist = beg - last;
        if (dist > minPix_) outSegments.push_back(EdgeSegment(offset, offset + dist));
        offset += dist;
        last = beg;
        lastPoint = currentPoint;
      }
    }
    dist = beg - last;
    if (dist > minPix_) outSegments.push_back(EdgeSegment(offset, offset + dist));
  }

  inline void tracePoints(const EdgeSegment& seg,
                          const IndexVector& supportPoints,
                          EdgeSegmentVector& outSegments,
                          size_t cols,
                          int id = 0) {
    if (supportPoints.empty() || supportPoints.size() < seg.end()) return;
    tracePoints(supportPoints.data() + seg.begin(), supportPoints.data() + seg.end(), outSegments, 0, id);
  }

  inline void tracePoints(const IndexVector& supportPoints, EdgeSegmentVector& outSegments, size_t cols, int id = 0) {
    if (supportPoints.empty()) return;

    tracePoints(supportPoints.data(), supportPoints.data() + supportPoints.size(), outSegments, 0, cols, id);
  }

  inline void tracePoints(const index_type* beg,
                          const index_type* end,
                          EdgeSegmentVector& outSegments,
                          PointVector& outPoints,
                          size_t cols,
                          size_t offset = 0,
                          int id = 0) {
    const index_type* last = beg;
    const PT lastPoint, currentPoint;
    index2Point(*beg, lastPoint, cols);
    outPoints.clear();
    outPoints.reserve(end - beg);
    outPoints.push_back(lastPoint);
    ptrdiff_t dist = 0;
    ++beg;
    for (; beg < end; ++beg) {
      index2Point(*beg, currentPoint, cols);
      outPoints.push_back(currentPoint);
      if (std::abs(getX(lastPoint) - getX(currentPoint)) > maxGap_ ||
          std::abs(getY(lastPoint) - getY(currentPoint)) > maxGap_) {
        dist = beg - last;
        if (dist > minPix_) outSegments.push_back(EdgeSegment(offset, offset + dist));
        offset += dist;
        last = beg;
        lastPoint = currentPoint;
      }
    }
    dist = beg - last;
    if (dist > minPix_) outSegments.push_back(EdgeSegment(offset, offset + dist));
  }

  inline void tracePoints(const EdgeSegment& seg,
                          const IndexVector& supportPoints,
                          EdgeSegmentVector& outSegments,
                          PointVector& outPoints,
                          size_t cols,
                          int id = 0) {
    if (supportPoints.empty() || supportPoints.size() < seg.end()) return;
    tracePoints(supportPoints.data() + seg.begin(), supportPoints.data() + seg.end(), outSegments, outPoints, 0, cols,
                id);
  }

  inline void tracePoints(const IndexVector& supportPoints,
                          EdgeSegmentVector& outSegments,
                          PointVector& outPoints,
                          size_t cols,
                          int id = 0) {
    if (supportPoints.empty()) return;

    tracePoints(supportPoints.data(), supportPoints.data() + supportPoints.size(), outSegments, outPoints, 0, cols, id);
  }

  inline void tracePoints(const PT* beg, const PT* end, EdgeSegmentVector& outSegments, size_t offset = 0, int id = 0) {
    const PT* last = beg;
    ptrdiff_t dist = 0;
    ++beg;
    for (; beg < end; ++beg) {
      if (std::abs(getX(*last) - getX(*beg)) > maxGap_ || std::abs(getY(*last) - getY(*beg)) > maxGap_) {
        dist = beg - last;
        if (dist > minPix_) outSegments.push_back(EdgeSegment(offset, offset + dist));
        offset += dist;
        last = beg;
      }
    }
    dist = beg - last;
    if (dist > minPix_) outSegments.push_back(EdgeSegment(offset, offset + dist));
  }

  inline void tracePoints(const EdgeSegment& seg,
                          const std::vector<PT>& supportPoints,
                          EdgeSegmentVector& outSegments,
                          int id = 0) {
    if (supportPoints.empty() || supportPoints.size() < seg.end()) return;

    tracePoints(supportPoints.data() + seg.begin(), supportPoints.data() + seg.end(), outSegments, 0, id);
  }

  inline void tracePointList(const std::vector<PT>& supportPoints, EdgeSegmentVector& outSegments, int id = 0) {
    if (supportPoints.empty()) return;
    tracePoints(supportPoints.data(), supportPoints.data() + supportPoints.size(), outSegments, 0, id);
  }

 private:
  inline bool prepare(
      const LineSegment<FT, LPT>& line, LPT<FT>& delta, LPT<FT>& startPoint, LPT<FT>& endPoint, size_t& steps) {
    delta = line.direction();
    line.endPoints(startPoint, endPoint);
    LPT<FT> delta_abs(std::abs(getX(delta)), std::abs(getY(delta)));
    // y is main dir
    if (getY(delta_abs) > getX(delta_abs)) {
      FT scale = 1;
      scale /= getY(delta_abs);
      delta *= scale;
      if (getY(startPoint) < getY(endPoint)) {
        setY(startPoint, std::ceil(getY(startPoint)));
        setY(endPoint, std::floor(getY(endPoint)));
      } else {
        setY(startPoint, std::floor(getY(startPoint)));
        setY(endPoint, std::ceil(getY(endPoint)));
      }

      setX(startPoint, line.x(getY(startPoint)));
      setX(endPoint, line.x(getY(endPoint)));
      steps = static_cast<size_t>(std::abs(getY(endPoint) - getY(startPoint)));
      return false;
    }

    // x is main dir
    FT scale = 1;
    scale /= getX(delta_abs);
    delta *= scale;
    if (getX(startPoint) < getX(endPoint)) {
      setX(startPoint, std::ceil(getX(startPoint)));
      setX(endPoint, std::floor(getX(endPoint)));
    } else {
      setX(startPoint, std::floor(getX(startPoint)));
      setX(endPoint, std::ceil(getX(endPoint)));
    }

    setY(startPoint, line.y(getX(startPoint)));
    setY(endPoint, line.y(getX(endPoint)));
    steps = static_cast<size_t>(std::abs(getX(endPoint) - getX(startPoint)));
    return true;
  }

  inline bool checkPixelBinaryX(const LPT<FT>& pos,
                                cv::Mat& map,
                                PointVector& outSupportPoints,
                                IndexVector& outSupportIndexes) {
    PT ipos(static_cast<int>(getX(pos)), static_cast<int>(std::round(getY(pos))));
    index_type idx = point2Index(ipos, map.cols);

    uint8_t* val = map.ptr<uint8_t>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx]) {
        val[idx] = 0;
        outSupportPoints.push_back(ipos);
        outSupportIndexes.push_back(idx);
        return true;
      }
      getX(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = idxSelector_[i];
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelBinaryX(const LPT<FT>& pos, cv::Mat& map, PointVector& outSupportPoints) {
    PT ipos(static_cast<int>(getX(pos)), static_cast<int>(std::round(getY(pos))));
    index_type idx = point2Index(ipos, map.cols);

    uint8_t* val = map.ptr<uint8_t>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx]) {
        val[idx] = 0;
        outSupportPoints.push_back(ipos);
        return true;
      }
      getX(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = idxSelector_[i];
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelBinaryY(const LPT<FT>& pos,
                                cv::Mat& map,
                                PointVector& outSupportPoints,
                                IndexVector& outSupportIndexes) {
    PT ipos(static_cast<int>(std::round(getX(pos))), static_cast<int>(getY(pos)));
    index_type idx = point2Index(ipos, map.cols);

    uint8_t* val = map.ptr<uint8_t>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx]) {
        val[idx] = 0;
        outSupportPoints.push_back(ipos);
        outSupportIndexes.push_back(idx);
        return true;
      }
      getY(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = static_cast<std::ptrdiff_t>(idxSelector_[i]) * map.cols;
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelBinaryY(const LPT<FT>& pos, cv::Mat& map, PointVector& outSupportPoints) {
    PT ipos(static_cast<int>(std::round(getX(pos))), static_cast<int>(getY(pos)));
    index_type idx = point2Index(ipos, map.cols);

    uint8_t* val = map.ptr<uint8_t>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx]) {
        val[idx] = 0;
        outSupportPoints.push_back(ipos);
        return true;
      }
      getY(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = static_cast<std::ptrdiff_t>(idxSelector_[i]) * map.cols;
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelDirmapX(
      const LPT<FT>& pos, cv::Mat& map, char dir, PointVector& outSupportPoints, IndexVector& outSupportIndexes) {
    PT ipos(static_cast<int>(getX(pos)), static_cast<int>(std::round(getY(pos))));
    index_type idx = point2Index(ipos, map.cols);

    char* val = map.ptr<char>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx] > -1) {
        // stop directly if we find badly orientated data
        if (absDiff<DM>(dir - val[idx]) > 1) return false;
        val[idx] = -1;
        outSupportPoints.push_back(ipos);
        outSupportIndexes.push_back(idx);
        return true;
      }
      getX(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = idxSelector_[i];
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelDirmapX(const LPT<FT>& pos, cv::Mat& map, char dir, PointVector& outSupportPoints) {
    PT ipos(static_cast<int>(getX(pos)), static_cast<int>(std::round(getY(pos))));
    index_type idx = point2Index(ipos, map.cols);

    char* val = map.ptr<char>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx] > -1) {
        // stop directly if we find badly orientated data
        if (absDiff<DM>(dir - val[idx]) > 1) return false;
        val[idx] = -1;
        outSupportPoints.push_back(ipos);
        return true;
      }
      getX(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = idxSelector_[i];
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelDirmapY(
      const LPT<FT>& pos, cv::Mat& map, char dir, PointVector& outSupportPoints, IndexVector& outSupportIndexes) {
    PT ipos(static_cast<int>(std::round(getX(pos))), static_cast<int>(getY(pos)));
    index_type idx = point2Index(ipos, map.cols);

    char* val = map.ptr<char>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx] > -1) {
        // stop directly if we find badly orientated data
        if (absDiff<DM>(dir - val[idx]) > 1) return false;
        val[idx] = -1;
        outSupportPoints.push_back(ipos);
        outSupportIndexes.push_back(idx);
        return true;
      }
      getY(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = static_cast<std::ptrdiff_t>(idxSelector_[i]) * map.cols;
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelDirmapY(const LPT<FT>& pos, cv::Mat& map, char dir, PointVector& outSupportPoints) {
    PT ipos(static_cast<int>(std::round(getX(pos))), static_cast<int>(getY(pos)));
    index_type idx = point2Index(ipos, map.cols);

    char* val = map.ptr<char>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx] > -1) {
        // stop directly if we find badly orientated data
        if (absDiff<DM>(dir - val[idx]) > 1) return false;
        val[idx] = -1;
        outSupportPoints.push_back(ipos);
        return true;
      }
      getY(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = static_cast<std::ptrdiff_t>(idxSelector_[i]) * map.cols;
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelDirmapX(const LPT<FT>& pos,
                                cv::Mat& edge,
                                const cv::Mat& dirMap,
                                char dir,
                                PointVector& outSupportPoints,
                                IndexVector& outSupportIndexes) {
    PT ipos(static_cast<int>(getX(pos)), static_cast<int>(std::round(getY(pos))));
    index_type idx = point2Index(ipos, edge.cols);

    uint8_t* val = edge.ptr<uint8_t>();
    const char* pdir = dirMap.ptr<char>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx]) {
        // stop directly if we find badly orientated data
        if (absDiff<DM>(dir - pdir[idx]) > 1) return false;
        val[idx] = 0;
        outSupportPoints.push_back(ipos);
        outSupportIndexes.push_back(idx);
        return true;
      }
      getX(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = idxSelector_[i];
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelDirmapX(
      const LPT<FT>& pos, cv::Mat& edge, const cv::Mat& dirMap, char dir, PointVector& outSupportPoints) {
    PT ipos(static_cast<int>(getX(pos)), static_cast<int>(std::round(getY(pos))));
    index_type idx = point2Index(ipos, edge.cols);

    uint8_t* val = edge.ptr<uint8_t>();
    const char* pdir = dirMap.ptr<char>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx]) {
        // stop directly if we find badly orientated data
        if (absDiff<DM>(dir - pdir[idx]) > 1) return false;
        val[idx] = 0;
        outSupportPoints.push_back(ipos);
        return true;
      }
      getX(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = idxSelector_[i];
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelDirmapY(const LPT<FT>& pos,
                                cv::Mat& edge,
                                const cv::Mat& dirMap,
                                char dir,
                                PointVector& outSupportPoints,
                                IndexVector& outSupportIndexes) {
    PT ipos(static_cast<int>(std::round(getX(pos))), static_cast<int>(getY(pos)));
    index_type idx = point2Index(ipos, edge.cols);

    uint8_t* val = edge.ptr<uint8_t>();
    const char* pdir = dirMap.ptr<char>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx]) {
        // stop directly if we find badly orientated data
        if (absDiff<DM>(dir - pdir[idx]) > 1) return false;
        val[idx] = 0;
        outSupportPoints.push_back(ipos);
        outSupportIndexes.push_back(idx);
        return true;
      }
      getY(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = static_cast<std::ptrdiff_t>(idxSelector_[i]) * edge.cols;
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }

  inline bool checkPixelDirmapY(
      const LPT<FT>& pos, cv::Mat& edge, const cv::Mat& dirMap, char dir, PointVector& outSupportPoints) {
    PT ipos(static_cast<int>(std::round(getX(pos))), static_cast<int>(getY(pos)));
    index_type idx = point2Index(ipos, edge.cols);

    uint8_t* val = edge.ptr<uint8_t>();
    const char* pdir = dirMap.ptr<char>();
    int end = searchSteps_ * 2 + 1;
    for (int i = 0; i < end; ++i) {
      if (val[idx]) {
        // stop directly if we find badly orientated data
        if (absDiff<DM>(dir - pdir[idx]) > 1) return false;
        val[idx] = 0;
        outSupportPoints.push_back(ipos);
        return true;
      }
      getY(ipos) += idxSelector_[i];
      const std::ptrdiff_t offset = static_cast<std::ptrdiff_t>(idxSelector_[i]) * edge.cols;
      idx = static_cast<index_type>(static_cast<std::ptrdiff_t>(idx) + offset);
    }
    return false;
  }
};

template <class FT, template <class> class LPT, class PT, int DM>
int LineTracer<FT, LPT, PT, DM>::idxSelector_[21] = {1,   -2, 3,   -4, 5,   -6, 7,   -8, 9,   -10, 11,
                                                     -12, 13, -14, 15, -16, 17, -18, 19, -20, 21};
}  // namespace lsfm
