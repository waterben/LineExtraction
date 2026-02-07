//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
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

/// @brief Get best keypoints from spatial bins as a flat vector.
/// Distributes keypoints into a grid of bins and returns the highest-response
/// keypoints concatenated into a single output vector.
/// @tparam FT Float type
/// @tparam GT Geometric type (e.g., cv::KeyPoint)
/// @param kps Output vector containing the best keypoints from all bins
/// @param keypoints Input keypoints to distribute into bins
/// @param width Image width
/// @param height Image height
/// @param wBins Number of horizontal bins
/// @param hBins Number of vertical bins
/// @param ptsPerBin Maximum keypoints per bin
/// @param sorted Whether input keypoints are pre-sorted by response (default: true)
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


/// @brief Filter out keypoints that have no matching candidates.
/// Removes keypoints without match candidates and remaps candidate indices,
/// reducing the number of descriptors that need to be created.
/// @tparam GV Geometric vector type
/// @tparam FMV Feature match vector type
/// @tparam MV Match count vector type
/// @param keypoints0 Left keypoints (modified in-place to contain only matched)
/// @param keypoints1 Right keypoints (modified in-place to contain only matched)
/// @param candidates Match candidates (indices are remapped in-place)
/// @param mLeft_ Per-keypoint match count for left set
/// @param mRight_ Per-keypoint match count for right set
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
