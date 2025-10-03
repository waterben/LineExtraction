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

#ifndef _EDGE_DRAWING_HPP_
#define _EDGE_DRAWING_HPP_
#ifdef __cplusplus

#  include <edge/edge_segment.hpp>

namespace lsfm {

template <class MT, int NUM_DIR = 8>
class EsdDrawing : public EsdBase<MT, index_type> {
  cv::Mat dir_;
  char* pdir_;

#  ifdef DRAW_MODE
  cv::Mat draw;
  cv::Vec3b col;
#  endif

  short dmapStore_[20];

  const short* dmap;
  const short* pdmap;
  const short* rvdmap;
  const short* fwdmap;
  const MT* pmag_;

  int minPixels_;
  float magTh_, magMul_;
#  ifndef NO_ADDED_SEEDS
  IndexVector addedSeeds_;
#  endif

  using EsdBase<MT, index_type>::points_;
  using EsdBase<MT, index_type>::segments_;

 public:
  EsdDrawing(int minPix = 10, float magMul = 3, float magTh = 5)
      : EsdBase<MT, index_type>(), minPixels_(minPix), magMul_(magMul), magTh_(magTh) {
    dmap = &dmapStore_[8];
    rvdmap = dmap - 4;
    fwdmap = dmap;

    this->add("edge_min_pixels", std::bind(&EsdDrawing<MT, NUM_DIR>::valueMinPixel, this, std::placeholders::_1),
              "Minimal number of support pixels.");
    this->add("edge_mag_mul", std::bind(&EsdDrawing<MT, NUM_DIR>::valueMagMul, this, std::placeholders::_1),
              "Magnitude multiplicator.");
    this->add("edge_mag_th", std::bind(&EsdDrawing<MT, NUM_DIR>::valueMagThreshold, this, std::placeholders::_1),
              "Magnitude threshold.");
  }

  Value valueMinPixel(const Value& mp = Value::NAV()) {
    if (mp.type()) minPixels(mp.getInt());
    return minPixels_;
  }

  int minPixels() const { return minPixels_; }

  void minPixels(int mp) { minPixels_ = mp; }

  Value valueMagMul(const Value& mm = Value::NAV()) {
    if (mm.type()) magMul(mm.getFloat());
    return magMul_;
  }

  float magMul() const { return magMul_; }

  void magMul(float mm) { magMul_ = mm; }

  Value valueMagThreshold(const Value& mt = Value::NAV()) {
    if (mt.type()) magThresold(mt.getFloat());
    return magTh_;
  }

  float magThresold() const { return magTh_; }

  void magThresold(float mt) { magTh_ = mt; }

  using EsdBase<MT, index_type>::detect;

  void detect(const cv::Mat& dir, const cv::Mat& mag, const IndexVector& seeds) {
    pmag_ = mag.ptr<MT>();
    points_.clear();
    points_.reserve(seeds.size() * 3);
    segments_.clear();
    segments_.reserve(seeds.size() / 2);
    dir_ = dir.clone();
    dir_.row(0).setTo(-2);
    dir_.row(dir.rows - 1).setTo(-2);
    dir_.col(0).setTo(-2);
    dir_.col(dir.cols - 1).setTo(-2);
    pdir_ = dir_.ptr<char>();
#  ifndef NO_ADDED_SEEDS
    addedSeeds_.clear();
#  endif

#  ifdef DRAW_MODE
    draw.create(dir_.size(), CV_8UC3);
    draw.setTo(0);
#  endif

    dmapStore_[0] = dmapStore_[8] = dmapStore_[16] = 1;
    dmapStore_[1] = dmapStore_[9] = dmapStore_[17] = static_cast<short>(dir.cols + 1);
    dmapStore_[2] = dmapStore_[10] = dmapStore_[18] = static_cast<short>(dir.cols);
    dmapStore_[3] = dmapStore_[11] = dmapStore_[19] = static_cast<short>(dir.cols - 1);
    dmapStore_[4] = dmapStore_[12] = -1;
    dmapStore_[5] = dmapStore_[13] = static_cast<short>(-1 - dir.cols);
    dmapStore_[6] = dmapStore_[14] = static_cast<short>(-dir.cols);
    dmapStore_[7] = dmapStore_[15] = static_cast<short>(1 - dir.cols);

    for_each(seeds.begin(), seeds.end(), [&](index_type idx) { search(idx); });

#  ifndef NO_ADDED_SEEDS
    size_t c = 0;
    while (c != addedSeeds_.size()) search(addedSeeds_[c++]);
#  endif

    // std::cout << "draw - added seeds: " << addedSeeds_.size() << std::endl;
  }

  std::string name() const { return "draw"; }

 private:
  // check for vaild adjacent pixel by given direction and retun new index
  inline index_type checkAdjacent(index_type idx, char& dir) {
    char dirn = dir - 1;
    char dirp = dir + 1;

    index_type nidx = idx + pdmap[dir];
    index_type nidxn = idx + pdmap[dirn];
    index_type nidxp = idx + pdmap[dirp];

    MT v = pmag_[nidx];
    MT vn = pmag_[nidxn];
    MT vp = pmag_[nidxp];

    if (vn > v) {
      if (vp > vn) {
        v = vp;
        nidx = nidxp;
        dir = fixDir<NUM_DIR>(dirp);
#  ifndef NO_ADDED_SEEDS
        addedSeeds_.push_back(nidxn);
#  endif
      } else {
        v = vn;
        nidx = nidxn;
        dir = fixDir<NUM_DIR>(dirn);
#  ifndef NO_ADDED_SEEDS
        if (vp > v) addedSeeds_.push_back(nidxp);
#  endif
      }
    } else if (vp > v) {
      v = vp;
      nidx = nidxp;
      dir = fixDir<NUM_DIR>(dirp);
    }

    // is pixel already used or border or no magnitude
    if (v < magTh_ || pdir_[nidx] < -1 || v > magMul_ * pmag_[idx]) return 0;

    return nidx;
  }

#  ifndef NO_EDGE_THICK_CHECK

  // check for thick lines and remove pixels
  inline void checkThick(index_type idx, char dir) {
    index_type nidx = idx + pdmap[dir];
    if (pdir_[nidx] < -1) return;
    pdir_[nidx] = -4;
  }


  // find adjacent pixel
  inline index_type findAdjacent(index_type idx, char& dir) {
    index_type nidx = checkAdjacent(idx, dir);
    // try to find next pixel direction of edge map
    if (nidx) {
      // check for bad pixels (only on odd dirs - diagonals)
      if (dir % 2) {
        checkThick(idx, dir - 1);
        checkThick(idx, dir + 1);
      }
    }
    return nidx;
  }
#  else

  // find next pixel without thickness check
  inline index_type findAdjacent(index_type idx, char& dir) { return checkAdjacent(idx, dir); }
#  endif
  // find next pixel without thickness check
  inline index_type findAdjacent(index_type idx) {
    char dir = pdir_[idx];
    return (dir < 0) ? 0 : findAdjacent(idx, dir);
  }

  void extractSegment(index_type idx) {
    char dir = pdir_[idx];

    index_type tmp;
    while (idx) {
#  ifdef DRAW_MODE
      draw.ptr<cv::Vec3b>()[idx] = col;
      cv::imshow("draw", draw);
      cv::waitKey(1);
#  endif
      points_.push_back(idx);
      tmp = idx;
      idx = findAdjacent(idx, dir);
      pdir_[tmp] = -3;
    }
  }

  void search(index_type idx) {
    char dir = pdir_[idx];
    if (dir < 0) return;

    size_t seg_beg = points_.size(), seg_end = points_.size();
#  ifdef DRAW_MODE
    cv::RNG& rng = cv::theRNG();
    col = cv::Vec3b(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
#  endif

    // check for fw points
    pdmap = fwdmap;
    if (!findAdjacent(idx)) {
      // no fw points found, do rv search
      pdmap = rvdmap;
      extractSegment(idx);
      seg_end = this->points_.size();
      if (seg_end - seg_beg > minPixels_) segments_.push_back(EdgeSegment(seg_beg, seg_end, ES_REVERSE));
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
      if (seg_end - seg_beg > minPixels_) segments_.push_back(EdgeSegment(seg_beg, seg_end));
      return;
    }

    // do rv first
    extractSegment(ridx);

    // closed check
    if (this->points_.back() == idx) {
      seg_end = this->points_.size();
      if (seg_end - seg_beg > minPixels_) segments_.push_back(EdgeSegment(seg_beg, seg_end, ES_REVERSE | ES_CLOSED));
      return;
    }

    std::reverse(this->points_.begin() + seg_beg, this->points_.end());

    // do fw
    pdmap = fwdmap;
    extractSegment(idx);
    seg_end = this->points_.size();
    if (seg_end - seg_beg > minPixels_) segments_.push_back(EdgeSegment(seg_beg, seg_end));
  }
};

}  // namespace lsfm
#endif
#endif
