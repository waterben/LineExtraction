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

#ifndef _EDGE_PATTERN_HPP_
#define _EDGE_PATTERN_HPP_
#ifdef __cplusplus

#  include <edge/edge_segment.hpp>
// #define NO_EDGE_THICK_CHECK
// #define NO_GRADIENT_MAX_CHECK
// #define NO_ADDED_SEEDS

namespace lsfm {

template <class MT, int NUM_DIR = 8, bool USE_CORNER_RULE = false>
class EsdPattern : public EsdBasePattern<MT, index_type> {
  cv::Mat dir_;
  char* pdir_;

#  ifdef DRAW_MODE
  cv::Mat draw;
  cv::Vec3b col;
#  endif

  short dmapStore_[20];
  char abs_diffmapStore_[15];

  const short* dmap;
  const short* pdmap;
  const short* rvdmap;
  const short* fwdmap;
  const MT* pmag_;

  int minPixels_, maxGap_, patTol_;
  float magMul_, magTh_;
#  ifndef NO_ADDED_SEEDS
  IndexVector addedSeeds_;
#  endif

  using EsdBase<MT, index_type>::points_;
  using EsdBase<MT, index_type>::segments_;
  EdgeSegmentVector patterns_;
  EdgeSegmentVector patternSegments_;

 public:
  struct Primitive {
    Primitive(ushort s = 0, char d = -1, char dn = -1) : size(s), dir(d), dir_next(dn) {}

    // primitive size
    ushort size;
    // primitive direction
    char dir;
    // direction change to next primitive
    char dir_next;

    //! match PatternPrimitive by tolerance
    inline bool match(const Primitive& rhs, int tol) const {
      return dir == rhs.dir && (size == rhs.size || std::abs(size - rhs.size) <= tol);
    }

    //! compare operator
    inline bool operator==(const Primitive& rhs) const {
      return size == rhs.size && dir == rhs.dir && dir_next == rhs.dir_next;
    }

    inline bool operator!=(const Primitive& rhs) const { return !operator==(rhs); }
  };

 private:
  std::vector<Primitive> primitives_;

 public:
  typedef std::vector<Primitive> PrimitiveVector;

  EsdPattern(int minPix = 10, int maxGap = 3, float magMul = 3, float magTh = 5, int pat_tol = 2)
      : EsdBasePattern<MT, index_type>(),
        minPixels_(minPix),
        maxGap_(maxGap),
        magMul_(magMul),
        magTh_(magTh),
        patTol_(pat_tol) {
    dmap = &dmapStore_[8];
    rvdmap = dmap - 4;
    fwdmap = dmap;

    this->add("edge_min_pixels",
              std::bind(&EsdPattern<MT, NUM_DIR, USE_CORNER_RULE>::valueMinPixel, this, std::placeholders::_1),
              "Minimal number of support pixels.");
    this->add("edge_max_gap",
              std::bind(&EsdPattern<MT, NUM_DIR, USE_CORNER_RULE>::valueMaxGap, this, std::placeholders::_1),
              "Maximum pixel number of gaps.");
    this->add("edge_mag_mul",
              std::bind(&EsdPattern<MT, NUM_DIR, USE_CORNER_RULE>::valueMagMul, this, std::placeholders::_1),
              "Magnitude multiplicator.");
    this->add("edge_mag_th",
              std::bind(&EsdPattern<MT, NUM_DIR, USE_CORNER_RULE>::valueMagThreshold, this, std::placeholders::_1),
              "Magnitude threshold.");
    this->add("edge_pat_tol",
              std::bind(&EsdPattern<MT, NUM_DIR, USE_CORNER_RULE>::valuePatThreshold, this, std::placeholders::_1),
              "Pattern tolerance.");
  }

  Value valueMinPixel(const Value& mp = Value::NAV()) {
    if (mp.type()) minPixels(mp.getInt());
    return minPixels_;
  }

  int minPixels() const { return minPixels_; }

  void minPixels(int mp) { minPixels_ = mp; }

  Value valueMaxGap(const Value& mg = Value::NAV()) {
    if (mg.type()) maxGap(mg.getInt());
    return maxGap_;
  }

  int maxGap() const { return maxGap_; }

  void maxGap(int mg) { maxGap_ = mg; }

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

  Value valuePatThreshold(const Value& pl = Value::NAV()) {
    if (pl.type()) patternThreshold(pl.getInt());
    return patTol_;
  }

  int patternThreshold() const { return patTol_; }

  void patternThreshold(int pl) { patTol_ = pl; }

  using EsdBasePattern<MT, index_type>::detect;

  void detect(const cv::Mat& dir, const cv::Mat& mag, const IndexVector& seeds) {
    pmag_ = mag.ptr<MT>();
    points_.clear();
    points_.reserve(seeds.size() * 3);
    segments_.clear();
    primitives_.clear();
    primitives_.reserve(seeds.size() / 2);
    patterns_.clear();
    patterns_.reserve(seeds.size() / 2);
    patternSegments_.clear();
    patternSegments_.reserve(seeds.size() / 3);
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


    if (USE_CORNER_RULE) {
      for_each(seeds.begin(), seeds.end(), [&](index_type idx) { searchC(idx); });

      size_t c = 0;
      while (c != addedSeeds_.size()) searchC(addedSeeds_[c++]);
    } else {
      for_each(seeds.begin(), seeds.end(), [&](index_type idx) { search(idx); });

#  ifndef NO_ADDED_SEEDS
      size_t c = 0;
      while (c != addedSeeds_.size()) search(addedSeeds_[c++]);
#  endif
    }

    // std::cout << "link - added seeds: " << addedSeeds_.size() << std::endl;
  }

  std::string name() const { return "pattern"; }

  const EdgeSegmentVector& segments() const {
    // convert pattern segments to point segments
    if (segments_.empty()) {
      segments_.reserve(patternSegments_.size());
      for_each(patternSegments_.begin(), patternSegments_.end(), [this](const EdgeSegment& e) {
        segments_.push_back(
            EdgeSegment(this->patterns_[e.begin()].begin(), this->patterns_[e.end() - 1].end(), e.flags()));
      });
    }
    return segments_;
  }
  const PrimitiveVector& primitives() const { return primitives_; }


  const EdgeSegmentVector& patterns() const { return patterns_; }

  const EdgeSegmentVector& patternSegments() const { return patternSegments_; }

 private:
  struct Pattern {
    Pattern(size_t b = 0, size_t e = 0) : seg(b, e) {}

    Primitive prim;
    EdgeSegment seg;
  };

  // check for vaild adjacent pixel by given direction and retun new index
  inline index_type checkAdjacent(index_type idx, char dir) {
    index_type nidx = idx + pdmap[dir];
    char ndir = pdir_[nidx];
    // is pixel already used / not set and direction is -+1
    // if (ndir < 0 || absDiff<NUM_DIR>(dir-ndir) > 1)
    //    return 0;
    if (ndir < 0) return 0;
    if (absDiff<NUM_DIR>(dir - ndir) > 1) {
#  ifndef NO_ADDED_SEEDS
      addedSeeds_.push_back(idx);
#  endif
      return 0;
    }
    return nidx;
  }

  // check for vaild adjacent pixel by magnitude and given direction and retun new index
  inline index_type checkAdjacentMag(index_type idx, char& dir) {
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
        dirn = dirp;
#  ifndef NO_ADDED_SEEDS
        addedSeeds_.push_back(nidxn);
#  endif
      } else {
        v = vn;
        nidx = nidxn;
#  ifndef NO_ADDED_SEEDS
        if (vp > v) addedSeeds_.push_back(nidxp);
#  endif
      }
    } else if (vp > v) {
      v = vp;
      nidx = nidxp;
      dirn = dirp;
    }

    // is pixel already used or border or no magnitude
    if (v < magTh_ || pdir_[nidx] < -1 || v > magMul_ * pmag_[idx]) return 0;
    dir = dirn;
    return nidx;
  }

#  ifndef NO_EDGE_THICK_CHECK

  // check for thick lines and remove pixels
  inline void checkThick(index_type idx, char dir) {
    index_type nidx = idx + pdmap[dir];
    if (pdir_[nidx] < 0) return;
    pdir_[nidx] = -4;
  }

  // check for thick lines and remove pixels
  inline void checkThickMag(index_type idx, char dir) {
    index_type nidx = idx + pdmap[dir];
    if (pdir_[nidx] < -1) return;
    pdir_[nidx] = -4;
  }

#    ifndef NO_GRADIENT_MAX_CHECK
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
#      ifndef NO_ADDED_SEEDS
        addedSeeds_.push_back(nidxp);
#      endif
        nidxp = nidxn;
        checkThick(idx, dir - 2);
        --dir;
      } else {
#      ifndef NO_ADDED_SEEDS
        addedSeeds_.push_back(nidxn);
#      endif
        checkThick(idx, dir + 2);
        ++dir;
      }
    }

    return nidxp;
  }
#    else
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
#    endif

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

#  else

#    ifndef NO_GRADIENT_MAX_CHECK
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
#    else
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
#    endif

  // find next pixel without thickness check
  inline index_type findAdjacent(index_type idx, char& dir) {
    index_type nidx = checkAdjacent(idx, dir);
    return nidx ? nidx : findNear(idx, dir);
  }
#  endif

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

  void extractSegment(index_type idx, Pattern lp, Pattern p, char& ls_dir) {
    char tdir, dir = pdir_[idx];
    // can happen e.g. by thick check, that is removing a found pixel for fw/rv search -> double check
    if (dir < 0) return;
    int gap = 0;
    size_t nidx;

    auto checkEnd = [&]() {
      // only need to add current pattern with termination flag
      if (lp.prim.size == 0) {
        if (p.prim.size) {
          p.seg.end(points_.size());
          patterns_.push_back(p.seg);
          primitives_.push_back(p.prim);
        }
      }
      // current pattern fits to last, merge and reset
      else if (lp.prim.match(p.prim, patTol_)) {
        lp.seg.end(points_.size());
        patterns_.push_back(lp.seg);
        primitives_.push_back(lp.prim);
        lp = Pattern();
      }
      // add last and current pattern and reset
      else {
        patterns_.push_back(lp.seg);
        primitives_.push_back(lp.prim);
        ;
        lp = Pattern();
        if (p.prim.size) {
          p.seg.end(points_.size());
          patterns_.push_back(p.seg);
          primitives_.push_back(p.prim);
        }
      }
    };

    while (idx && gap < maxGap_) {
      nidx = findAdjacent(idx, dir);
#  ifdef DRAW_MODE
      draw.ptr<cv::Vec3b>()[idx] = col;
      if (nidx) draw.ptr<cv::Vec3b>()[nidx] = cv::Vec3b(255, 255, 255);
      cv::imshow("draw", draw);
      cv::waitKey(1);
#  endif
      points_.push_back(idx);
      pdir_[idx] = -3;
      dir = fixDir8(dir);
      // update pattern pixel count and segments states
      ++p.prim.size;

      // if we have more than two pixels and a direction change, check pattern
      if (p.prim.size > 1 && ls_dir != dir) {
        // get new direction
        p.prim.dir_next = dir;

        // if direction change is to big, stop pattern
        if (absDiff<NUM_DIR>(ls_dir - dir) > 1) {
          checkEnd();
        }
        // if no last pattern exist, just store current pattern as last and continue
        else if (lp.prim.size == 0) {
          lp = p;
          lp.seg.end(points_.size());
        } else if (lp.prim.match(p.prim, patTol_)) {
          // current pattern equals last pattern, update lp size and prim size
          if (lp.prim.dir_next == p.prim.dir_next) {
            float repeat = static_cast<float>(lp.seg.size()) / lp.prim.size + 1;
            lp.seg.end(points_.size());
            lp.prim.size = static_cast<ushort>(round(lp.seg.size() / repeat));
          }
          // no more repeat, but current element fits to last, add last pattern and reset to no last pattern
          else {
            lp.seg.end(points_.size());
            patterns_.push_back(lp.seg);
            primitives_.push_back(lp.prim);
            lp = Pattern();
          }
        }
        // no repeat, no fit -> store last pattern to list and replace with current pattern
        else {
          //  check if last pattern fits to current and previous  -> split
          size_t lp_size = lp.seg.size();
          if (lp.prim.size == lp_size && lp_size > 1 && patterns_.size() && lp.prim.dir == p.prim.dir &&
              lp.prim.dir_next == p.prim.dir_next) {
            EdgeSegment& blps = patterns_.back();
            Primitive& blpp = primitives_.back();
            if (lp.prim.dir == blpp.dir) {
              unsigned int s = blpp.size + p.prim.size;
              if (lp_size <= s + 2 * patTol_ && lp_size >= s - 2 * patTol_) {
                unsigned int blpadd =
                    static_cast<unsigned int>(round(static_cast<double>(lp.seg.size() * blpp.size) / s));
                blps.end(blps.end() + blpadd);
                p.seg.begin(blps.end());
                goto update_lp;
              }
            }
          }

          patterns_.push_back(lp.seg);
          primitives_.push_back(lp.prim);
        update_lp:
          lp = p;
          lp.seg.end(points_.size());
        }
        // reset current pattern
        p = Pattern(points_.size());
      }
      // update main direction of pattern
      else {
        p.prim.dir = dir;
      }

      ls_dir = dir;
      tdir = pdir_[nidx];
      if (tdir > -1) {
        dir = tdir;
        gap = 0;
      } else
        ++gap;

      idx = nidx;
    }
    checkEnd();
  }

  void pattern_reverse(EdgeSegmentVector::iterator beg, EdgeSegmentVector::iterator end) {
    size_t po_beg = beg->begin();
    size_t po_end = (end - 1)->end();
    std::reverse(points_.begin() + po_beg, points_.begin() + po_end);
    while (beg < end) {
      --end;
      EdgeSegment tmp = *beg;
      end->moveTo(po_beg);
      po_beg += end->size();
      *beg = *end;
      po_end -= tmp.size();
      tmp.moveTo(po_end);
      *end = tmp;
      ++beg;
    }
  };

  void search(index_type idx) {
    char dir = pdir_[idx];
    if (dir < 0) return;
    char ls_dir = -1;
    size_t pat_seg_beg = patterns_.size(), point_seg_beg = points_.size();
    Pattern lp, p(point_seg_beg);
#  ifdef DRAW_MODE
    cv::RNG& rng = cv::theRNG();
    col = cv::Vec3b(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
#  endif

    // check for fw points
    pdmap = fwdmap;
    if (!findAdjacent(idx)) {
      // no fw points found, do rv search
      pdmap = rvdmap;
      extractSegment(idx, lp, p, ls_dir);
      if (patterns_.size() > pat_seg_beg && points_.size() - point_seg_beg > minPixels_)
        patternSegments_.push_back(EdgeSegment(pat_seg_beg, patterns_.size(), ES_REVERSE));
      return;
    }

    // check for rv points
    pdmap = rvdmap;
    index_type ridx = findAdjacent(idx);
    if (!ridx) {
      // if no rv points found, do fw search
      pdmap = fwdmap;
      extractSegment(idx, lp, p, ls_dir);
      if (patterns_.size() > pat_seg_beg && points_.size() - point_seg_beg > minPixels_)
        patternSegments_.push_back(EdgeSegment(pat_seg_beg, patterns_.size()));
      return;
    }

    // do rv first
    extractSegment(ridx, lp, p, ls_dir);

    // closed check
    if (points_.back() == idx) {
      if (patterns_.size() > pat_seg_beg && points_.size() - point_seg_beg > minPixels_)
        patternSegments_.push_back(EdgeSegment(pat_seg_beg, patterns_.size(), ES_REVERSE | ES_CLOSED));
      return;
    }

    pattern_reverse(patterns_.begin() + pat_seg_beg, patterns_.end());
    std::reverse(primitives_.begin() + pat_seg_beg, primitives_.end());

    // get current pattern
    p.seg = patterns_.back();
    p.prim = primitives_.back();
    ls_dir = p.prim.dir;

    patterns_.pop_back();
    primitives_.pop_back();

    // do fw
    pdmap = fwdmap;
    extractSegment(idx, lp, p, ls_dir);
    if (patterns_.size() > pat_seg_beg && points_.size() - point_seg_beg > minPixels_)
      patternSegments_.push_back(EdgeSegment(pat_seg_beg, patterns_.size()));
  }

  inline void addPatternSegments(size_t pat_seg_beg, size_t point_seg_beg, int flags = 0) {
    if (patterns_.size() > pat_seg_beg && points_.size() - point_seg_beg > minPixels_) {
      if (patterns_.size() - pat_seg_beg == 1)
        patternSegments_.push_back(EdgeSegment(pat_seg_beg, patterns_.size(), flags));
      else if (patterns_.size() - pat_seg_beg == 2) {
        if (absDiff<NUM_DIR>(primitives_[pat_seg_beg].dir - primitives_[pat_seg_beg + 1].dir) > 1 &&
            patterns_[pat_seg_beg].size() > 3 && patterns_[pat_seg_beg + 1].size() > 3) {
          patternSegments_.push_back(EdgeSegment(pat_seg_beg, pat_seg_beg + 1, flags));
          patternSegments_.push_back(EdgeSegment(pat_seg_beg + 1, pat_seg_beg + 2, flags));
        } else
          patternSegments_.push_back(EdgeSegment(pat_seg_beg, patterns_.size(), flags));
      } else {
        size_t tmp = pat_seg_beg;
        for (size_t i = pat_seg_beg + 1; i < patterns_.size() - 1; ++i) {
          if (absDiff<NUM_DIR>(primitives_[i - 1].dir - primitives_[i].dir) > 1 && patterns_[i - 1].size() > 3 &&
              patterns_[i].size() > 3) {
            patternSegments_.push_back(EdgeSegment(pat_seg_beg, i, flags & ES_CLOSED));
            pat_seg_beg = i;
            continue;
          }
          if (patterns_[i].size() < 3 && absDiff<NUM_DIR>(primitives_[i - 1].dir - primitives_[i + 1].dir) > 1 &&
              patterns_[i - 1].size() > 3 && patterns_[i + 1].size() > 3) {
            patternSegments_.push_back(EdgeSegment(pat_seg_beg, i, flags & ES_CLOSED));
            pat_seg_beg = i + 1;
          }
        }
        if (pat_seg_beg < patterns_.size())
          patternSegments_.push_back(
              EdgeSegment(pat_seg_beg, patterns_.size(), pat_seg_beg == tmp ? flags : flags & ES_CLOSED));
      }
    }
  }

  void searchC(index_type idx) {
    char dir = pdir_[idx];
    if (dir < 0) return;
    char ls_dir = -1;
    size_t pat_seg_beg = patterns_.size(), point_seg_beg = points_.size();
    Pattern lp, p(point_seg_beg);
#  ifdef DRAW_MODE
    cv::RNG& rng = cv::theRNG();
    col = cv::Vec3b(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
#  endif

    // check for fw points
    pdmap = fwdmap;
    if (!findAdjacent(idx)) {
      // no fw points found, do rv search
      pdmap = rvdmap;
      extractSegment(idx, lp, p, ls_dir);
      addPatternSegments(pat_seg_beg, point_seg_beg, ES_REVERSE);
      return;
    }

    // check for rv points
    pdmap = rvdmap;
    index_type ridx = findAdjacent(idx);
    if (!ridx) {
      // if no rv points found, do fw search
      pdmap = fwdmap;
      extractSegment(idx, lp, p, ls_dir);
      addPatternSegments(pat_seg_beg, point_seg_beg);
      return;
    }

    // do rv first
    extractSegment(ridx, lp, p, ls_dir);

    // closed check
    if (points_.back() == idx) {
      addPatternSegments(pat_seg_beg, point_seg_beg, ES_REVERSE | ES_CLOSED);
      return;
    }

    pattern_reverse(patterns_.begin() + pat_seg_beg, patterns_.end());
    std::reverse(primitives_.begin() + pat_seg_beg, primitives_.end());

    // get current pattern
    p.seg = patterns_.back();
    p.prim = primitives_.back();
    ls_dir = p.prim.dir;

    patterns_.pop_back();
    primitives_.pop_back();

    // do fw
    pdmap = fwdmap;
    extractSegment(idx, lp, p, ls_dir);
    addPatternSegments(pat_seg_beg, point_seg_beg);
  }
};

}  // namespace lsfm
#endif
#endif
