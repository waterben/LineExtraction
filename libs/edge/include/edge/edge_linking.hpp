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

/// @file edge_linking.hpp
/// @brief Edge linking algorithm with continuity optimization.
/// Implements edge segment detection by linking adjacent edgels while optimizing
/// for spatial continuity and magnitude consistency.

#pragma once

#include <edge/edge_segment.hpp>

#include <cstddef>
// #define NO_EDGE_THICK_CHECK
// #define NO_GRADIENT_MAX_CHECK

namespace lsfm {

/// @brief Edge Segment Detector using edge linking with continuity optimization.
/// Links edgels (edge pixels) into connected segments by tracing along edge directions
/// while maintaining continuity and enforcing magnitude constraints. Supports both
/// continuous linking and pattern-based linking strategies.
/// @tparam MT Magnitude data type (typically float or uchar)
/// @tparam NUM_DIR Number of directions to consider (4 or 8)
/// @tparam USE_CORNER_RULE Whether to apply corner rule for linking
template <class MT, int NUM_DIR = 8, bool USE_CORNER_RULE = false>
class EsdLinking : public EsdBase<MT, index_type> {
  cv::Mat dir_;          ///< Encoded direction map for each edge pixel
  char* pdir_{nullptr};  ///< Pointer to direction data for faster access

#ifdef DRAW_MODE
  cv::Mat draw;   ///< Debug visualization of segment linking
  cv::Vec3b col;  ///< Current segment color for debug visualization
#endif

  short dmapStore_[20];        ///< Direction offset lookup table (size 20 for wraparound)
  char abs_diffmapStore_[15];  ///< Precomputed absolute difference map for direction angles

  const short* dmap{nullptr};    ///< Pointer to direction offset map (centered at index 8)
  const short* pdmap{nullptr};   ///< Current direction map pointer (either rvdmap or fwdmap)
  const short* rvdmap{nullptr};  ///< Reverse direction map (backward traversal)
  const short* fwdmap{nullptr};  ///< Forward direction map (forward traversal)
  const MT* pmag_{nullptr};      ///< Pointer to gradient magnitude data

  int minPixels_,  ///< Minimum segment length filter
      maxGap_;     ///< Maximum allowed gap when linking segments
  float magMul_,   ///< Magnitude multiplier for determining search penalty
      magTh_;      ///< Magnitude threshold for edge acceptance
#ifndef NO_ADDED_SEEDS
  IndexVector addedSeeds_;  ///< Cache of seed points added during search for deferred processing
#endif

  using EsdBase<MT, index_type>::points_;
  using EsdBase<MT, index_type>::segments_;

 public:
  /// @brief Construct an edge linking detector with continuity optimization.
  /// @param minPix Minimum number of pixels for a valid segment (default: 10)
  /// @param maxGap Maximum allowed gap in pixels when connecting edges (default: 3)
  /// @param magMul Magnitude multiplier for gap penalty (default: 3.0)
  /// @param magTh Magnitude threshold for edge acceptance (default: 5.0)
  EsdLinking(int minPix = 10, int maxGap = 3, float magMul = 3, float magTh = 5)
      : EsdBase<MT, index_type>(),
        dir_(),
        minPixels_(minPix),
        maxGap_(maxGap),
        magMul_(magMul),
        magTh_(magTh),
        addedSeeds_() {
    dmap = &dmapStore_[8];
    rvdmap = dmap - 4;
    fwdmap = dmap;

    this->add("edge_min_pixels",
              std::bind(&EsdLinking<MT, NUM_DIR, USE_CORNER_RULE>::valueMinPixel, this, std::placeholders::_1),
              "Minimal number of support pixels.");
    this->add("edge_max_gap",
              std::bind(&EsdLinking<MT, NUM_DIR, USE_CORNER_RULE>::valueMaxGap, this, std::placeholders::_1),
              "Maximum pixel number of gaps.");
    this->add("edge_mag_mul",
              std::bind(&EsdLinking<MT, NUM_DIR, USE_CORNER_RULE>::valueMagMul, this, std::placeholders::_1),
              "Magnitude multiplicator.");
    this->add("edge_mag_th",
              std::bind(&EsdLinking<MT, NUM_DIR, USE_CORNER_RULE>::valueMagThreshold, this, std::placeholders::_1),
              "Magnitude threshold.");
  }

  EsdLinking(const EsdLinking&) = delete;
  EsdLinking& operator=(const EsdLinking&) = delete;

  /// @brief Get or set minimum pixels via value interface.
  /// @param mp Value object with new minimum pixels (optional)
  /// @return Current minimum pixels value
  Value valueMinPixel(const Value& mp = Value::NAV()) {
    if (mp.type()) minPixels(mp.getInt());
    return minPixels_;
  }

  /// @brief Get minimum pixels for valid segment.
  /// @return Minimum pixel count
  int minPixels() const { return minPixels_; }

  /// @brief Set minimum pixels for valid segment.
  /// @param mp New minimum pixel count
  void minPixels(int mp) { minPixels_ = mp; }

  /// @brief Get or set maximum gap via value interface.
  /// @param mg Value object with new maximum gap (optional)
  /// @return Current maximum gap value
  Value valueMaxGap(const Value& mg = Value::NAV()) {
    if (mg.type()) maxGap(mg.getInt());
    return maxGap_;
  }

  /// @brief Get maximum allowed gap in pixels.
  /// @return Maximum gap value
  int maxGap() const { return maxGap_; }

  /// @brief Set maximum allowed gap in pixels.
  /// @param mg New maximum gap value
  void maxGap(int mg) { maxGap_ = mg; }

  /// @brief Get or set magnitude multiplier via value interface.
  /// @param mm Value object with new magnitude multiplier (optional)
  /// @return Current magnitude multiplier value
  Value valueMagMul(const Value& mm = Value::NAV()) {
    if (mm.type()) magMul(mm.getFloat());
    return magMul_;
  }

  /// @brief Get magnitude multiplier for gap penalty.
  /// @return Magnitude multiplier value
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
#ifndef NO_ADDED_SEEDS
    addedSeeds_.clear();
#endif

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

    if (USE_CORNER_RULE) {
      for_each(seeds.begin(), seeds.end(), [&](index_type idx) { searchC(idx); });

#ifndef NO_ADDED_SEEDS
      size_t c = 0;
      while (c != addedSeeds_.size()) searchC(addedSeeds_[c++]);
#endif
    } else {
      for_each(seeds.begin(), seeds.end(), [&](index_type idx) { search(idx); });

#ifndef NO_ADDED_SEEDS
      size_t c = 0;
      while (c != addedSeeds_.size()) search(addedSeeds_[c++]);
#endif
    }

    // std::cout << "link - added seeds: " << addedSeeds_.size() << std::endl;
  }

  std::string name() const { return "link"; }

 private:
  /// @brief Check if an adjacent pixel is valid and unvisited in specified direction.
  /// Validates that neighbor has not been processed (pdir >= 0) and that its gradient
  /// direction is compatible with current. May add seed if direction mismatch detected.
  /// @param idx Current pixel index
  /// @param dir Desired movement direction
  /// @return Adjacent pixel index if valid, 0 otherwise
  inline index_type checkAdjacent(index_type idx, char dir) {
    const ptrdiff_t offset = pdmap[static_cast<int>(dir)];
    index_type nidx = static_cast<index_type>(static_cast<ptrdiff_t>(idx) + offset);
    char ndir = pdir_[nidx];
    // is pixel already used / not set and direction is -+1
    // if (ndir < 0 || absDiff<NUM_DIR>(dir-ndir > 1)
    //    return 0;
    if (ndir < 0) return 0;
    if (absDiff<NUM_DIR>(dir - ndir) > 1) {
#ifndef NO_ADDED_SEEDS
      addedSeeds_.push_back(idx);
#endif
      return 0;
    }
    return nidx;
  }

  // check for vaild adjacent pixel by magnitude and given direction and retun new index
  inline index_type checkAdjacentMag(index_type idx, char& dir) {
    char dirn = dir - 1;
    char dirp = dir + 1;

    const ptrdiff_t offset = pdmap[static_cast<int>(dir)];
    const ptrdiff_t offsetN = pdmap[static_cast<int>(dirn)];
    const ptrdiff_t offsetP = pdmap[static_cast<int>(dirp)];
    index_type nidx = static_cast<index_type>(static_cast<ptrdiff_t>(idx) + offset);
    index_type nidxn = static_cast<index_type>(static_cast<ptrdiff_t>(idx) + offsetN);
    index_type nidxp = static_cast<index_type>(static_cast<ptrdiff_t>(idx) + offsetP);

    MT v = pmag_[nidx];
    MT vn = pmag_[nidxn];
    MT vp = pmag_[nidxp];

    if (vn > v) {
      if (vp > vn) {
        v = vp;
        nidx = nidxp;
        dirn = fixDir<NUM_DIR>(dirp);
#ifndef NO_ADDED_SEEDS
        addedSeeds_.push_back(nidxn);
#endif
      } else {
        v = vn;
        nidx = nidxn;
        dirn = fixDir<NUM_DIR>(dirn);
#ifndef NO_ADDED_SEEDS
        if (vp > v) addedSeeds_.push_back(nidxp);
#endif
      }
    } else if (vp > v) {
      v = vp;
      nidx = nidxp;
      dirn = fixDir<NUM_DIR>(dirp);
    }

    // is pixel already used or border or no magnitude
    const float vf = static_cast<float>(v);
    const float vmag = static_cast<float>(pmag_[idx]);
    if (vf < magTh_ || pdir_[nidx] < -1 || vf > magMul_ * vmag) return 0;
    dir = dirn;
    return nidx;
  }

#ifndef NO_EDGE_THICK_CHECK

  // check for thick lines and remove pixels
  inline void checkThick(index_type idx, char dir) {
    const ptrdiff_t offset = pdmap[static_cast<int>(dir)];
    index_type nidx = static_cast<index_type>(static_cast<ptrdiff_t>(idx) + offset);
    if (pdir_[nidx] < 0) return;
    pdir_[nidx] = -4;
  }

  // check for thick lines and remove pixels
  inline void checkThickMag(index_type idx, char dir) {
    const ptrdiff_t offset = pdmap[static_cast<int>(dir)];
    index_type nidx = static_cast<index_type>(static_cast<ptrdiff_t>(idx) + offset);
    if (pdir_[nidx] < -1) return;
    pdir_[nidx] = -4;
  }

#  ifndef NO_GRADIENT_MAX_CHECK
  // find pixel near (fw + left, fw + right) current pixel
  inline index_type findNear(index_type idx, char& dir) {
    index_type nidxp = checkAdjacent(idx, dir + 1);
    index_type nidxn = checkAdjacent(idx, dir - 1);
    if (nidxn == 0 && nidxp == 0) {
      nidxp = checkAdjacentMag(idx, dir);
      if (nidxp) {
        // check for bad pixels (only on odd dirs - diagonals)
        if (dir % 2) {
          checkThickMag(idx, dir - 1);
          checkThickMag(idx, dir + 1);
        }
      }
    } else if (nidxp == 0) {
      checkThick(idx, dir - 2);
      nidxp = nidxn;
      --dir;
    } else if (nidxn == 0) {
      checkThick(idx, dir + 2);
      ++dir;
    } else {
      if (pmag_[nidxn] > pmag_[nidxp]) {
#    ifndef NO_ADDED_SEEDS
        addedSeeds_.push_back(nidxp);
#    endif
        nidxp = nidxn;
        checkThick(idx, dir - 2);
        --dir;
      } else {
#    ifndef NO_ADDED_SEEDS
        addedSeeds_.push_back(nidxn);
#    endif
        checkThick(idx, dir + 2);
        ++dir;
      }
    }

    return nidxp;
  }
#  else
  // find pixel near (fw + left, fw + right) current pixel
  inline index_type findNear(index_type idx, char& dir) {
    ++dir;
    index_type nidx = checkAdjacent(idx, dir);
    if (nidx) {
      checkThick(idx, dir + 1);
      return nidx;
    }
    dir -= 2;
    nidx = checkAdjacent(idx, dir);
    if (nidx) {
      checkThick(idx, dir - 1);
      return nidx;
    }
    ++dir;
    nidx = checkAdjacentMag(idx, dir);
    if (nidx) {
      // check for bad pixels (only on odd dirs - diagonals)
      if (dir % 2) {
        checkThickMag(idx, dir - 1);
        checkThickMag(idx, dir + 1);
      }
    }
    return nidx;
  }
#  endif

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
      return nidx;
    }
    return findNear(idx, dir);
  }

#else

#  ifndef NO_GRADIENT_MAX_CHECK
  inline index_type findNear(index_type idx, char& dir) {
    char dirp = dir + 1;
    char dirn = dir - 1;
    index_type nidxp = checkAdjacent(idx, dirp);
    index_type nidxn = checkAdjacent(idx, dirn);
    if (nidxp == 0 && nidxn == 0) {
      nidxp = checkAdjacentMag(idx, dir);
    } else if (pmag_[nidxn] > pmag_[nidxp]) {
      nidxp = nidxn;
      dir = dirn;
    } else {
      dir = dirp;
    }
    return nidxp;
  };
#  else
  inline index_type findNear(index_type idx, char& dir) {
    ++dir;
    index_type nidx = checkAdjacent(idx, dir);
    if (nidx == 0) {
      dir -= 2;
      nidx = checkAdjacent(idx, dir);
    }
    if (nidx == 0) {
      ++dir;
      nidx = checkAdjacentMag(idx, dir);
    }
    return nidx;
  };
#  endif

  // find next pixel without thickness check
  inline index_type findAdjacent(index_type idx, char& dir) {
    index_type nidx = checkAdjacent(idx, dir);
    return nidx ? nidx : findNear(idx, dir);
  }
#endif

  // find next pixel (simple, for direct call)
  inline index_type findAdjacent(index_type idx) {
    char dir = pdir_[idx];
    if (dir < 0) return 0;
    index_type nidx = checkAdjacent(idx, dir);
    if (nidx == 0) {
      nidx = checkAdjacent(idx, dir - 1);
    }
    if (nidx == 0) {
      nidx = checkAdjacent(idx, dir + 1);
    }
    return nidx;
  }

  /// @brief Extract a connected segment with continuity optimization.
  /// Traverses the edge chain using findAdjacent with direction refinement. Handles gaps
  /// up to maxGap_ pixels and tracks direction changes for continuity. Marks visited
  /// pixels as used (pdir = -3).
  /// @param idx Starting pixel index for segment extraction
  void extractSegment(index_type idx) {
    char tdir = -1, dir = 0;
    int gap = 0;

    while (idx && gap < maxGap_) {
      tdir = pdir_[idx];
      if (tdir > -1) {
        dir = tdir;
        gap = 0;
      } else
        ++gap;

#ifdef DRAW_MODE
      draw.ptr<cv::Vec3b>()[idx] = col;
      cv::imshow("draw", draw);
      cv::waitKey(1);
#endif
      points_.push_back(idx);
      pdir_[idx] = -3;
      idx = findAdjacent(idx, dir);
    }
  }

  /// @brief Process seed pixel for segment linking with continuity optimization.
  /// Explores both forward and reverse directions from seed, handles closure detection,
  /// and validates minimum pixel count before creating segment. Uses direction maps
  /// (rvdmap/fwdmap) for bi-directional tracing.
  /// @param idx Seed pixel index to begin segment extraction
  void search(index_type idx) {
    char dir = pdir_[idx];
    if (dir < 0) return;

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
      if (seg_end - seg_beg > static_cast<size_t>(minPixels_))
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
      if (seg_end - seg_beg > static_cast<size_t>(minPixels_)) segments_.push_back(EdgeSegment(seg_beg, seg_end));
      return;
    }

    // do rv first
    extractSegment(ridx);

    // closed check
    if (this->points_.back() == idx) {
      seg_end = this->points_.size();
      if (seg_end - seg_beg > static_cast<size_t>(minPixels_))
        segments_.push_back(EdgeSegment(seg_beg, seg_end, ES_REVERSE | ES_CLOSED));
      return;
    }

    std::reverse(this->points_.begin() + static_cast<typename IndexVector::difference_type>(seg_beg),
                 this->points_.end());

    // do fw
    pdmap = fwdmap;
    extractSegment(idx);
    /// @brief Extract segment starting at pixel with specified direction using corner rule.
    /// Overloaded convenience method that reads direction from pdir_[idx].
    /// @param idx Seed pixel index for segment extraction
    inline void extractSegmentC(index_type idx) { extractSegmentC(idx, pdir_[idx]); }

    /// @brief Extract segment with corner-detection rule enabled.
    /// Corner rule: processes pixels where gradient direction changes abruptly,
    /// allowing continuity across corners. Uses DataPair struct to track multiple
    /// candidate directions and validates consistency.
    /// @param idx Starting pixel index
    /// @param ndir Initial neighbor direction to explore    if (seg_end - seg_beg > static_cast<size_t>(minPixels_))
    /// segments_.push_back(EdgeSegment(seg_beg, seg_end));
  }


  inline void extractSegmentC(index_type idx) { extractSegmentC(idx, pdir_[idx]); }

  void extractSegmentC(index_type idx, char ndir) {
    struct DataPair {
      char dir, ndir;
      size_t idx;
    };

    /*auto draw = [&, this](const DataPair &i1, const DataPair & i2, const DataPair & i3, const DataPair & i4, bool
    refused = false) { cv::Mat tmp(this->dir_.size(), CV_8UC3); tmp.setTo(0); cv::Vec3b *p = tmp.ptr<cv::Vec3b>();
        std::cout << (int)i1.ndir << ", " << (int)i2.ndir << ", " << (int)i3.ndir << ", " << (int)i4.ndir << std::endl;

        if (refused) {
            p[i1.idx] = cv::Vec3b(255, 255, 255);
            p[i2.idx] = cv::Vec3b(255, 255, 255);
            p[i3.idx] = cv::Vec3b(255, 255, 255);
            p[i4.idx] = cv::Vec3b(255, 255, 255);
        }
        else {
            p[i1.idx] = cv::Vec3b(0, 255, 0);
            p[i2.idx] = cv::Vec3b(255, 255, 0);
            p[i3.idx] = cv::Vec3b(0, 0, 255);
            p[i4.idx] = cv::Vec3b(0, 0, 255);
        }

        cv::imshow("linkc draw", tmp);

        cv::Mat tmp2(400, 400, CV_8UC3);
        tmp2.setTo(0);
        cv::Point p1, p2, p3, p4;
        index2Point(i1.idx, p1, dir_.cols);
        index2Point(i2.idx, p2, dir_.cols);
        index2Point(i3.idx, p3, dir_.cols);
        index2Point(i4.idx, p4, dir_.cols);

        int minx = dir_.cols, miny = dir_.rows;
        minx = std::min(minx, p1.x);
        minx = std::min(minx, p2.x);
        minx = std::min(minx, p3.x);
        minx = std::min(minx, p4.x);

        miny = std::min(miny, p1.y);
        miny = std::min(miny, p2.y);
        miny = std::min(miny, p3.y);
        miny = std::min(miny, p4.y);

        p1.x -= minx;
        p1.y -= miny;
        p2.x -= minx;
        p2.y -= miny;
        p3.x -= minx;
        p3.y -= miny;
        p4.x -= minx;
        p4.y -= miny;
        p1 *= 100;
        p2 *= 100;
        p3 *= 100;
        p4 *= 100;

        tmp2.rowRange(p1.y, p1.y + 100).colRange(p1.x, p1.x + 100).setTo(cv::Vec3b(0, 255, 0));
        tmp2.rowRange(p2.y, p2.y + 100).colRange(p2.x, p2.x + 100).setTo(cv::Vec3b(255, 255, 0));
        tmp2.rowRange(p3.y, p3.y + 100).colRange(p3.x, p3.x + 100).setTo(cv::Vec3b(0, 0, 255));
        tmp2.rowRange(p4.y, p4.y + 100).colRange(p4.x, p4.x + 100).setTo(cv::Vec3b(0, 0, 255));

        cv::imshow("linkc zoom", tmp2);
        cv::waitKey();
    };*/

    DataPair s0, s1, s2, s3;
    s0.dir = pdir_[idx];
    s0.ndir = ndir;
    s0.idx = idx;
    int gap = 0;

    points_.push_back(idx);
    pdir_[idx] = -3;

    // fill array
    s1.ndir = s0.dir;
    s1.idx = findAdjacent(s0.idx, s1.ndir);
    if (s1.idx) {
      s1.dir = pdir_[s1.idx];
      if (s1.dir < 0) {
        s1.dir = s1.ndir;
        ++gap;
      } else
        gap = 0;
    } else
      return;

    s2.ndir = s1.dir;
    s2.idx = findAdjacent(s1.idx, s2.ndir);
    if (s2.idx) {
      s2.dir = pdir_[s2.idx];
      if (s2.dir < 0) {
        s2.dir = s2.ndir;
        ++gap;
      } else
        gap = 0;
    } else {
      points_.push_back(s1.idx);
      return;
    }

    s3.ndir = s2.dir;
    s3.idx = findAdjacent(s2.idx, s3.ndir);
    if (s3.idx) {
      s3.dir = pdir_[s3.idx];
      if (s3.dir < 0) {
        s3.dir = s3.ndir;
        ++gap;
      } else
        gap = 0;
    } else {
      points_.push_back(s2.idx);
      return;
    }

    bool add = true;
    while (s3.idx && gap < maxGap_) {
      // draw(s0, s1, s2, s3);
      //  check for corner rule
      char diff_s = absDiff<NUM_DIR>(s0.ndir - s1.ndir);
      char diff_fs = absDiff<NUM_DIR>(s0.ndir - s2.ndir);
      //                         |
      // check for hard corner  _|
      if (diff_s > 1 && diff_fs > 1) {
        add = false;
        // draw(s0, s1, s2, s3,true);
        break;
      }

      //                           |
      //                           |
      // check for hard corners  _/
      char diff_fs2 = absDiff<NUM_DIR>(s0.ndir - s3.ndir);

      if (diff_fs > 1 && diff_fs2 > 1) {
        //      \     _
        //       \     \    __      |
        // case _/    _/    _/    __|
        if ((diff_fs > 2 && diff_fs2 > 2) || diff_s == 0) {
          // add s to current pattern
          // set map and add point to list
          points_.push_back(s1.idx);
          pdir_[s1.idx] = -3;
          s1 = s2;
        }
        add = false;
        // draw(s0, s1, s2, s3, true);
        break;
      }
      //                            |
      //                           /
      // check for hard corner   _|
      if (diff_s == 2 && s1.ndir == s3.ndir && diff_fs > 1) {
        add = false;
        // draw(s0, s1, s2, s3, true);
        break;
      }

      points_.push_back(s1.idx);
      pdir_[s1.idx] = -3;

      s0 = s1;
      s1 = s2;
      s2 = s3;

      s3.ndir = s3.dir;
      s3.idx = findAdjacent(s3.idx, s3.ndir);
      s3.dir = pdir_[s3.idx];
      if (s3.dir < 0) {
        s3.dir = s3.ndir;
        ++gap;
      } else
        gap = 0;
    }

    if (add) {
      points_.push_back(s1.idx);
      pdir_[s1.idx] = -3;
      points_.push_back(s2.idx);
      pdir_[s2.idx] = -3;
      if (s3.idx) {
        points_.push_back(s3.idx);
        pdir_[s3.idx] = -3;
      }
    } else {
      addedSeeds_.push_back(s1.idx);
    }
  }

  void searchC(index_type idx) {
    char dir = pdir_[idx];
    if (dir < 0) return;

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
      extractSegmentC(idx);
      seg_end = this->points_.size();
      if (seg_end - seg_beg > static_cast<size_t>(minPixels_))
        segments_.push_back(EdgeSegment(seg_beg, seg_end, ES_REVERSE));
      return;
    }

    // check for rv points
    pdmap = rvdmap;
    index_type ridx = findAdjacent(idx);

    if (!ridx) {
      // if no rv points found, do fw search
      pdmap = fwdmap;
      extractSegmentC(idx);
      seg_end = this->points_.size();
      if (seg_end - seg_beg > static_cast<size_t>(minPixels_)) segments_.push_back(EdgeSegment(seg_beg, seg_end));
      return;
    }

    // do rv first
    extractSegmentC(ridx);

    // closed check
    if (this->points_.back() == idx) {
      seg_end = this->points_.size();
      if (seg_end - seg_beg > static_cast<size_t>(minPixels_))
        segments_.push_back(EdgeSegment(seg_beg, seg_end, ES_REVERSE | ES_CLOSED));
      return;
    }

    std::reverse(this->points_.begin() + static_cast<typename IndexVector::difference_type>(seg_beg),
                 this->points_.end());

    // do fw
    pdmap = fwdmap;
    extractSegmentC(idx);
    seg_end = this->points_.size();
    if (seg_end - seg_beg > static_cast<size_t>(minPixels_)) segments_.push_back(EdgeSegment(seg_beg, seg_end));
  }
};

}  // namespace lsfm
