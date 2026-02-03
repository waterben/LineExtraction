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

/// @file FeatureTools.hpp
/// @brief Feature utility functions for matching and filtering operations.
/// Provides helpers for keypoint binning, candidate filtering, and feature organization.


#pragma once

#include <opencv2/core/core.hpp>

#include <map>
#include <sstream>
#include <string>

namespace lsfm {


/// @brief Get best keypoints from spatial bins.
/// Distributes keypoints into a grid of bins and returns the highest-response keypoints from each bin.
/// @tparam FT Float type
/// @tparam GT Geometric type (e.g., cv::KeyPoint)
/// @param kpBins Output vector of vectors containing binned keypoints
/// @param keypoints Input keypoints to distribute into bins
/// @param width Image width
/// @param height Image height
/// @param wBins Number of horizontal bins
/// @param hBins Number of vertical bins
/// @param ptsPerBin Maximum keypoints per bin
/// @param sorted Whether input keypoints are pre-sorted by response (default: true)
template <class FT, class GT>
void getBestKeypointsInBins(std::vector<std::vector<GT>>& kpBins,
                            std::vector<GT>& keypoints,
                            const int& width,
                            const int& height,
                            const int wBins,
                            const int hBins,
                            const int ptsPerBin,
                            const bool sorted = true) {
  if (!sorted) std::sort(keypoints.begin(), keypoints.end(), [](GT a, GT b) { return b.response < a.response; });

  kpBins.assign(wBins * hBins, std::vector<GT>());

  for_each(keypoints.begin(), keypoints.end(), [&](GT& kp) {
    int wB, hB;
    wB = static_cast<int>(static_cast<FT>(kp.pt.x) / (static_cast<FT>(width) / static_cast<FT>(wBins)));
    wB = (wB >= wBins ? wBins - 1 : wB);
    wB = (wB < 0 ? 0 : wB);
    hB = static_cast<int>(static_cast<FT>(kp.pt.y) / (static_cast<FT>(height) / static_cast<FT>(hBins)));
    hB = (hB >= hBins ? hBins - 1 : hB);
    hB = (hB < 0 ? 0 : hB);

    kpBins[(wB + (hB * wBins))].push_back(kp);
  });

  for_each(kpBins.begin(), kpBins.end(), [&ptsPerBin](std::vector<GT>& bin) {
    if (bin.size() > ptsPerBin) bin.resize(ptsPerBin);
  });
}

//! get the "best" keypoints of each bin, returns a vector, number of bins, width first
template <class FT, class GT>
void getBestKeypointsInBins(std::vector<GT>& kps,
                            std::vector<GT>& keypoints,
                            const int& width,
                            const int& height,
                            const int wBins,
                            const int hBins,
                            const int ptsPerBin,
                            const bool sorted = true) {
  std::vector<std::vector<GT>> kpBins;
  getBestKeypointsInBins<FT>(kpBins, keypoints, width, height, wBins, hBins, ptsPerBin, sorted);
  kps.clear();
  for_each(kpBins.begin(), kpBins.end(),
           [&kps](std::vector<GT>& bin) { kps.insert(kps.end(), bin.begin(), bin.end()); });
}


//! filter out keypoints, which have no candidates for matching - to save time on descriptor creation afterwards
template <class GV, class FMV, class MV>
void filterCandidateKeypoints(GV& keypoints0, GV& keypoints1, FMV& candidates, const MV& mLeft_, const MV& mRight_) {
  std::vector<int> map1, map2;
  map1.assign(mLeft_.size(), -1);
  map2.assign(mRight_.size(), -1);

  GV kn0new, kn1new;
  int kpNr = 0;
  for (int i = 0; i < mLeft_.size(); ++i) {
    if (mLeft_[i]) {
      map1[i] = kpNr;
      ++kpNr;
      kn0new.push_back(keypoints0[i]);
    }
  }
  kpNr = 0;
  for (int i = 0; i < mRight_.size(); ++i) {
    if (mRight_[i]) {
      map2[i] = kpNr;
      ++kpNr;
      kn1new.push_back(keypoints1[i]);
    }
  }
  for (int i = 0; i < candidates.size(); ++i) {
    candidates[i].queryIdx = map1[candidates[i].queryIdx];
    candidates[i].matchIdx = map2[candidates[i].matchIdx];
  }
  keypoints0 = kn0new;
  keypoints1 = kn1new;
}


}  // namespace lsfm
