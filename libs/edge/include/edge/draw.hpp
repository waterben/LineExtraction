//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file draw.hpp
/// @brief Visualization and drawing utilities for edge detection results.
/// Provides functions to visualize edges, segments, and related data structures.

#pragma once

#include <edge/edge_segment.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <utility/range.hpp>

namespace lsfm {

// Draw functions for edge-specific types
inline void drawSegment(cv::Mat& img,
                        const EdgeSegment& seg,
                        const IndexVector& points,
                        const cv::Vec3b& color = cv::Vec3b(0, 0, 255)) {
  if (img.channels() == 1) cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
  cv::Vec3b* pimg = img.ptr<cv::Vec3b>();
  for (size_t i = seg.begin(); i != seg.end(); ++i) {
    pimg[points[i]] = color;
  }
}

inline void drawSegment(cv::Mat& img,
                        const EdgeSegmentVector& segs,
                        const IndexVector& points,
                        const cv::Vec3b& color = cv::Vec3b(0, 0, 255)) {
  if (img.channels() == 1) cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
  for_each(segs.begin(), segs.end(), [&](const EdgeSegment& seg) { drawSegment(img, seg, points, color); });
}

}  // namespace lsfm
