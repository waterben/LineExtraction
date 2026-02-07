//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file hysteresis.hpp
/// @brief Hysteresis thresholding for edge detection.
/// Implements hysteresis thresholding algorithms that connect pixels above a high threshold
/// with neighboring pixels above a lower threshold, enabling robust edge detection.

#pragma once

#include <edge/index.hpp>
#include <opencv2/core/core.hpp>


namespace lsfm {

namespace detail {

/// @brief Apply hysteresis thresholding to a direction/classification map.
/// Tracks seed pixels and expands to neighboring pixels that meet criteria, marking them as valid edges.
/// @param map Input/output classification map (8-bit signed), modified to mark valid edgels with -1
/// @param dmap Direction/classification map indicating edge pixels and their directions
/// @param edgels Input/output vector of seed indices; expanded with newly found edgels
inline void hysteresis_map(cv::Mat& map, const cv::Mat& dmap, IndexVector& edgels) {
  dmap.copyTo(map);

  char* pmap = map.ptr<char>();
  size_t visited = 0;

  ptrdiff_t mapstep = map.cols;
  // remove seeds
  for_each(edgels.begin(), edgels.end(), [&pmap](index_type i) { pmap[i] = -1; });

  // now track the edges (hysteresis thresholding)
  while (visited != edgels.size()) {
    index_type i = edgels[visited];
    char* m = pmap + i;
    ++visited;

    if (m[-1] > -1) {
      m[-1] = -1;
      edgels.push_back(i - 1);
    }
    if (m[1] > -1) {
      m[1] = -1;
      edgels.push_back(static_cast<size_t>(i + 1));
    }
    if (m[-mapstep - 1] > -1) {
      m[-mapstep - 1] = -1;
      edgels.push_back(static_cast<size_t>(static_cast<ptrdiff_t>(i) - mapstep - 1));
    }
    if (m[-mapstep] > -1) {
      m[-mapstep] = -1;
      edgels.push_back(static_cast<size_t>(static_cast<ptrdiff_t>(i) - mapstep));
    }
    if (m[-mapstep + 1] > -1) {
      m[-mapstep + 1] = -1;
      edgels.push_back(static_cast<size_t>(static_cast<ptrdiff_t>(i) - mapstep + 1));
    }
    if (m[mapstep - 1] > -1) {
      m[mapstep - 1] = -1;
      edgels.push_back(static_cast<size_t>(static_cast<ptrdiff_t>(i) + mapstep - 1));
    }
    if (m[mapstep] > -1) {
      m[mapstep] = -1;
      edgels.push_back(static_cast<size_t>(static_cast<ptrdiff_t>(i) + mapstep));
    }
    if (m[mapstep + 1] > -1) {
      m[mapstep + 1] = -1;
      edgels.push_back(static_cast<size_t>(static_cast<ptrdiff_t>(i) + mapstep + 1));
    }
  }
}
}  // namespace detail

/// @brief Perform hysteresis edge linking and return the expanded edgel indices.
/// Starting from seed edgels, expands to 8-connected neighbors that have valid
/// direction values (>= 0), adding them to the edgels vector.
/// @param dmap Direction map (CV_8S) with quantized edge directions
/// @param edgels Input/output vector of edgel indices; seeds on input, expanded set on output
inline void hysteresis_edgels(const cv::Mat& dmap, IndexVector& edgels) {
  cv::Mat map;
  detail::hysteresis_map(map, dmap, edgels);
}

/// @brief Perform hysteresis edge linking and return direction-encoded map.
/// Expands seeds via 8-connected neighbors in the direction map, then produces
/// an output map where only linked edgels retain their direction values.
/// @param dmap Direction map (CV_8S) with quantized edge directions
/// @param edgels Input/output vector of seed indices; expanded with linked edgels
/// @return Direction map with only hysteresis-linked edgels retaining their values
inline cv::Mat hysteresis(const cv::Mat& dmap, IndexVector& edgels) {
  cv::Mat map;
  detail::hysteresis_map(map, dmap, edgels);
  map.setTo(-1);
  char* pmap = map.ptr<char>();
  const char* pdmap = dmap.ptr<char>();
  for_each(edgels.begin(), edgels.end(), [&pmap, &pdmap](index_type i) { pmap[i] = pdmap[i]; });
  return map;
}

/// @brief Perform hysteresis edge linking and return a binary edge map.
/// Expands seeds via 8-connected neighbors, producing a binary output where
/// all linked edgels are marked with the specified value.
/// @param dmap Direction map (CV_8S) with quantized edge directions
/// @param edgels Input/output vector of seed indices; expanded with linked edgels
/// @param val Value to mark edge pixels in the output map (default: 1)
/// @return Binary edge map (CV_8U) with linked edgels marked as val
inline cv::Mat hysteresis_binary(const cv::Mat& dmap, IndexVector& edgels, uchar val = 1) {
  cv::Mat map;
  dmap.convertTo(map, CV_8U);

  uint8_t* pmap = map.ptr<uint8_t>();
  size_t visited = 0;

  ptrdiff_t mapstep = map.cols;
  // remove seeds
  for_each(edgels.begin(), edgels.end(), [&pmap](index_type i) { pmap[i] = 0; });

  // now track the edges (hysteresis thresholding)
  while (visited != edgels.size()) {
    ptrdiff_t i = static_cast<ptrdiff_t>(edgels[visited]);
    uint8_t* m = pmap + i;
    ++visited;

    if (m[-1]) {
      m[-1] = 0;
      edgels.push_back(static_cast<size_t>(i - 1));
    }
    if (m[1]) {
      m[1] = 0;
      edgels.push_back(static_cast<size_t>(i + 1));
    }
    if (m[-mapstep - 1]) {
      m[-mapstep - 1] = 0;
      edgels.push_back(static_cast<size_t>(i - mapstep - 1));
    }
    if (m[-mapstep]) {
      m[-mapstep] = 0;
      edgels.push_back(static_cast<size_t>(i - mapstep));
    }
    if (m[-mapstep + 1]) {
      m[-mapstep + 1] = 0;
      edgels.push_back(static_cast<size_t>(i - mapstep + 1));
    }
    if (m[mapstep - 1]) {
      m[mapstep - 1] = 0;
      edgels.push_back(static_cast<size_t>(i + mapstep - 1));
    }
    if (m[mapstep]) {
      m[mapstep] = 0;
      edgels.push_back(static_cast<size_t>(i + mapstep));
    }
    if (m[mapstep + 1]) {
      m[mapstep + 1] = 0;
      edgels.push_back(static_cast<size_t>(i + mapstep + 1));
    }
  }
  map.setTo(0);
  pmap = map.ptr<uint8_t>();
  for_each(edgels.begin(), edgels.end(), [&pmap, &val](index_type i) { pmap[i] = val; });
  return map;
}

/// @brief Perform hysteresis edge linking (const edgels overload).
/// Creates a copy of edgels before performing hysteresis, leaving the input unchanged.
/// @param dmap Direction map (CV_8S) with quantized edge directions
/// @param edgels Seed indices (not modified)
/// @return Direction map with only hysteresis-linked edgels retaining their values
inline cv::Mat hysteresis(const cv::Mat& dmap, const IndexVector& edgels) {
  IndexVector eg = edgels;
  return hysteresis(dmap, eg);
}

/// @brief Perform binary hysteresis edge linking (const edgels overload).
/// Creates a copy of edgels before performing hysteresis, leaving the input unchanged.
/// @param dmap Direction map (CV_8S) with quantized edge directions
/// @param edgels Seed indices (not modified)
/// @param val Value to mark edge pixels in the output map (default: 1)
/// @return Binary edge map (CV_8U) with linked edgels marked as val
inline cv::Mat hysteresis_binary(const cv::Mat& dmap, const IndexVector& edgels, uchar val = 1) {
  IndexVector eg = edgels;
  return hysteresis_binary(dmap, eg, val);
}

/// @brief Perform hysteresis edge linking (const alias).
/// Alias for hysteresis() with const edgels to clarify intent.
/// @param dmap Direction map (CV_8S) with quantized edge directions
/// @param edgels Seed indices (not modified)
/// @return Direction map with only hysteresis-linked edgels
inline cv::Mat hysteresis_const(const cv::Mat& dmap, const IndexVector& edgels) {
  IndexVector eg = edgels;
  return hysteresis(dmap, eg);
}

/// @brief Perform binary hysteresis edge linking (const alias).
/// Alias for hysteresis_binary() with const edgels to clarify intent.
/// @param dmap Direction map (CV_8S) with quantized edge directions
/// @param edgels Seed indices (not modified)
/// @param val Value to mark edge pixels (default: 1)
/// @return Binary edge map (CV_8U)
inline cv::Mat hysteresis_binary_const(const cv::Mat& dmap, const IndexVector& edgels, uchar val = 1) {
  IndexVector eg = edgels;
  return hysteresis_binary(dmap, eg, val);
}

}  // namespace lsfm
