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

#include <edge/index.hpp>
#include <opencv2/core/core.hpp>


namespace lsfm {

namespace detail {
//! compute hysteresis of seeds and dmap
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
      edgels.push_back(i + 1);
    }
    if (m[-mapstep - 1] > -1) {
      m[-mapstep - 1] = -1;
      edgels.push_back(i - mapstep - 1);
    }
    if (m[-mapstep] > -1) {
      m[-mapstep] = -1;
      edgels.push_back(i - mapstep);
    }
    if (m[-mapstep + 1] > -1) {
      m[-mapstep + 1] = -1;
      edgels.push_back(i - mapstep + 1);
    }
    if (m[mapstep - 1] > -1) {
      m[mapstep - 1] = -1;
      edgels.push_back(i + mapstep - 1);
    }
    if (m[mapstep] > -1) {
      m[mapstep] = -1;
      edgels.push_back(i + mapstep);
    }
    if (m[mapstep + 1] > -1) {
      m[mapstep + 1] = -1;
      edgels.push_back(i + mapstep + 1);
    }
  }
}
}  // namespace detail

//! compute hysteresis of seeds and dmap
inline void hysteresis_edgels(const cv::Mat& dmap, IndexVector& edgels) {
  cv::Mat map;
  detail::hysteresis_map(map, dmap, edgels);
}

//! compute hysteresis of seeds and dmap
inline cv::Mat hysteresis(const cv::Mat& dmap, IndexVector& edgels) {
  cv::Mat map;
  detail::hysteresis_map(map, dmap, edgels);
  map.setTo(-1);
  char* pmap = map.ptr<char>();
  const char* pdmap = dmap.ptr<char>();
  for_each(edgels.begin(), edgels.end(), [&pmap, &pdmap](index_type i) { pmap[i] = pdmap[i]; });
  return map;
}

//! compute binary hysteresis of seeds and dmap
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
    index_type i = edgels[visited];
    uint8_t* m = pmap + i;
    ++visited;

    if (m[-1]) {
      m[-1] = 0;
      edgels.push_back(i - 1);
    }
    if (m[1]) {
      m[1] = 0;
      edgels.push_back(i + 1);
    }
    if (m[-mapstep - 1]) {
      m[-mapstep - 1] = 0;
      edgels.push_back(i - mapstep - 1);
    }
    if (m[-mapstep]) {
      m[-mapstep] = 0;
      edgels.push_back(i - mapstep);
    }
    if (m[-mapstep + 1]) {
      m[-mapstep + 1] = 0;
      edgels.push_back(i - mapstep + 1);
    }
    if (m[mapstep - 1]) {
      m[mapstep - 1] = 0;
      edgels.push_back(i + mapstep - 1);
    }
    if (m[mapstep]) {
      m[mapstep] = 0;
      edgels.push_back(i + mapstep);
    }
    if (m[mapstep + 1]) {
      m[mapstep + 1] = 0;
      edgels.push_back(i + mapstep + 1);
    }
  }
  map.setTo(0);
  pmap = map.ptr<uint8_t>();
  for_each(edgels.begin(), edgels.end(), [&pmap, &val](index_type i) { pmap[i] = val; });
  return map;
}

//! compute hysteresis of seeds and dmap
inline cv::Mat hysteresis(const cv::Mat& dmap, const IndexVector& edgels) {
  IndexVector eg = edgels;
  return hysteresis(dmap, eg);
}

//! compute binary hysteresis of seeds and dmap
inline cv::Mat hysteresis_binary(const cv::Mat& dmap, const IndexVector& edgels, uchar val = 1) {
  IndexVector eg = edgels;
  return hysteresis_binary(dmap, eg, val);
}

//! compute hysteresis of seeds and dmap
inline cv::Mat hysteresis_const(const cv::Mat& dmap, const IndexVector& edgels) {
  IndexVector eg = edgels;
  return hysteresis(dmap, eg);
}

//! compute binary hysteresis of seeds and dmap
inline cv::Mat hysteresis_binary_const(const cv::Mat& dmap, const IndexVector& edgels, uchar val = 1) {
  IndexVector eg = edgels;
  return hysteresis_binary(dmap, eg, val);
}

}  // namespace lsfm
