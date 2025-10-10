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

/*
 *  (C) by Benjamin Wassermann
 */

#pragma once

#include <edge/edge_segment.hpp>

#include <cstddef>

namespace lsfm {

template <class MT, int NUM_DIR = 8>
class EsdSimple : public EsdBase<MT, index_type> {
  cv::Mat dir_{};
  char* pdir_{nullptr};

#ifdef DRAW_MODE
  cv::Mat draw{};
  cv::Vec3b col{};
#endif

  short dmapStore_[20]{};

  const short* dmap{nullptr};
  const short* pdmap{nullptr};
  const short* rvdmap{nullptr};
  const short* fwdmap{nullptr};
  const MT* pmag_{nullptr};

  int min_pix_{};

  using EsdBase<MT, index_type>::points_;
  using EsdBase<MT, index_type>::segments_;

 public:
  EsdSimple(int minPix = 10) : EsdBase<MT, index_type>(), min_pix_(minPix) {
    dmap = &dmapStore_[8];

    rvdmap = dmap - 4;
    fwdmap = dmap;

    this->add("edge_min_pixels", std::bind(&EsdSimple<MT, NUM_DIR>::valueMinPixels, this, std::placeholders::_1),
              "Minimal number of support pixels.");
  }

  EsdSimple(const EsdSimple&) = delete;
  EsdSimple& operator=(const EsdSimple&) = delete;

  Value valueMinPixels(const Value& mp = Value::NAV()) {
    if (mp.type()) minPixels(mp.getInt());
    return min_pix_;
  }

  int minPixels() const { return min_pix_; }

  void minPixels(int mp) { min_pix_ = mp; }

  using EsdBase<MT, index_type>::detect;

  void detect(const cv::Mat& dir, const cv::Mat& mag, const IndexVector& seeds) {
    points_.clear();
    points_.reserve(seeds.size() * 3);
    segments_.clear();
    segments_.reserve(seeds.size() / 2);
    dir_ = dir.clone();
    pdir_ = dir_.ptr<char>();
    pmag_ = mag.ptr<MT>();

#ifdef DRAW_MODE
    draw.create(dir_.size(), CV_8UC3);
    draw.setTo(0);
#endif

    dmapStore_[0] = dmapStore_[8] = dmapStore_[16] = 1;
    dmapStore_[1] = dmapStore_[9] = dmapStore_[17] = static_cast<short>(dir.cols + 1);
    dmapStore_[2] = dmapStore_[10] = dmapStore_[18] = static_cast<short>(dir.cols);
    dmapStore_[3] = dmapStore_[11] = dmapStore_[19] = static_cast<short>(dir.cols - 1);
    dmapStore_[4] = dmapStore_[12] = -1;
    dmapStore_[5] = dmapStore_[13] = static_cast<short>(-1 - dir.cols);
    dmapStore_[6] = dmapStore_[14] = static_cast<short>(-dir.cols);
    dmapStore_[7] = dmapStore_[15] = static_cast<short>(1 - dir.cols);


    for_each(seeds.begin(), seeds.end(), [&](index_type idx) {
      char direction = pdir_[idx];
      if (direction < 0) return;

      size_t seg_beg = points_.size(), seg_end = points_.size();
#ifdef DRAW_MODE
      cv::RNG& rng = cv::theRNG();
      col = cv::Vec3b(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
#endif

      // check for fw points
      pdmap = fwdmap;
      if (!findAdjacent(idx)) {
        // no fw points found, do rv search
        pdmap = rvdmap;
        extractSegment(idx);
        seg_end = this->points_.size();
        if (seg_end - seg_beg > static_cast<size_t>(min_pix_))
          segments_.push_back(EdgeSegment(seg_beg, seg_end, ES_REVERSE));
        return;
      }

      // check for rv points
      pdmap = rvdmap;
      index_type ridx = findAdjacent(idx);

      if (!ridx) {
        // if no rv points found, do fw search
        pdmap = fwdmap;
        extractSegment(idx);
        seg_end = this->points_.size();
        if (seg_end - seg_beg > static_cast<size_t>(min_pix_)) segments_.push_back(EdgeSegment(seg_beg, seg_end));
        return;
      }

      // do rv first
      extractSegment(ridx);

      // closed check
      if (this->points_.back() == idx) {
        seg_end = this->points_.size();
        if (seg_end - seg_beg > static_cast<size_t>(min_pix_))
          segments_.push_back(EdgeSegment(seg_beg, seg_end, ES_REVERSE | ES_CLOSED));
        return;
      }

      std::reverse(this->points_.begin() + static_cast<typename std::vector<index_type>::difference_type>(seg_beg),
                   this->points_.end());

      // do fw
      pdmap = fwdmap;
      extractSegment(idx);
      seg_end = this->points_.size();
      if (seg_end - seg_beg > static_cast<size_t>(min_pix_)) segments_.push_back(EdgeSegment(seg_beg, seg_end));
    });
  }

  std::string name() const { return "simple"; }

 private:
  // check for vaild adjacent pixel by given direction and retun new index
  inline index_type checkAdjacent(index_type idx, char dir) {
    const int dirIndex = static_cast<int>(dir);
    const ptrdiff_t offset = pdmap[dirIndex];
    index_type nidx = static_cast<index_type>(static_cast<ptrdiff_t>(idx) + offset);
    char ndir = pdir_[nidx];
    // is pixel already used / not set and direction is -+1
    if (ndir < 0 || absDiff<NUM_DIR>(dir - ndir) > 1) return 0;
    return nidx;
  }

#ifndef NO_EDGE_THICK_CHECK

  // check for thick lines and remove pixels
  inline void checkThick(index_type idx, char dir) {
    const int dirIndex = static_cast<int>(dir);
    const ptrdiff_t offset = pdmap[dirIndex];
    index_type nidx = static_cast<index_type>(static_cast<ptrdiff_t>(idx) + offset);
    if (pdir_[nidx] < 0) return;
    pdir_[nidx] = -3;
  }

#  ifndef NO_GRADIENT_MAX_CHECK
  // find pixel near (fw + left, fw + right) current pixel
  index_type findNear(index_type idx, char dir) {
    index_type nidxp = checkAdjacent(idx, dir + 1);
    index_type nidxn = checkAdjacent(idx, dir - 1);
    if (nidxp == 0) {
      if (nidxn == 0) return 0;
      dir -= 2;
      nidxp = nidxn;
    } else if (nidxn == 0) {
      dir += 2;
    } else {
      if (pmag_[nidxn] > pmag_[nidxp]) {
        dir -= 2;
        nidxp = nidxn;
      } else
        dir += 2;
    }

    checkThick(idx, dir);
    return nidxp;
  }
#  else
  index_type findNear(index_type idx, char dir) {
    index_type nidx = checkAdjacent(idx, dir + 1);
    if (nidx) {
      checkThick(idx, dir + 2);
      return nidx;
    }
    nidx = checkAdjacent(idx, dir - 1);
    if (nidx) checkThick(idx, dir - 2);
    return nidx;
  }
#  endif

  // find adjacent pixel
  inline index_type findAdjacent(index_type idx) {
    char dir = pdir_[idx];
    index_type nidx = checkAdjacent(idx, dir);
    // try to find next pixel direction of edge map
    if (nidx) {
      // check for bad pixels (only on odd dirs - diagonals)
      if (dir % 2) {
        checkThick(idx, dir - 1);
        checkThick(idx, dir + 1);
      }
      return nidx;
    }
    return findNear(idx, dir);
  }
#else

#  ifndef NO_GRADIENT_MAX_CHECK
  index_type findNear(index_type idx, char dir) {
    index_type nidxp = checkAdjacent(idx, dir + 1);
    index_type nidxn = checkAdjacent(idx, dir - 1);
    return nidxp == 0 ? nidxn : nidxn == 0 ? nidxp : pmag_[nidxn] > pmag_[nidxp] ? nidxn : nidxp;
  };
#  else
  index_type findNear(index_type idx, char dir) {
    index_type nidx = checkAdjacent(idx, dir + 1);
    return nidx ? nidx : checkAdjacent(idx, dir - 1);
  };
#  endif

  // find next pixel without thickness check
  inline index_type findAdjacent(index_type idx) {
    char dir = pdir_[idx];
    index_type nidx = checkAdjacent(idx, dir);
    return nidx ? nidx : findNear(idx, dir);
  }
#endif
  void extractSegment(index_type idx) {
    index_type tmp;
    while (idx) {
#ifdef DRAW_MODE
      draw.ptr<cv::Vec3b>()[idx] = col;
      cv::imshow("draw", draw);
      cv::waitKey(1);
#endif
      points_.push_back(idx);
      tmp = idx;
      idx = findAdjacent(idx);
      pdir_[tmp] = -2;
    }
  }
};

}  // namespace lsfm
