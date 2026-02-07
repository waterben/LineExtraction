//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file edge_drawing.hpp
/// @brief Edge segment detector using drawing-based approach.
/// Implements edge detection by simulating edge drawing/tracing along gradient directions.

#pragma once

#include <edge/edge_segment.hpp>

#include <cstddef>

namespace lsfm {

/// @brief Edge segment detector using drawing-based edge tracing.
/// Detects edge segments by tracing along gradient directions from seed points,
/// following the strongest magnitude path forward and backward. Supports both
/// 4-directional and 8-directional tracing.
/// @tparam MT Magnitude matrix element type
/// @tparam NUM_DIR Number of quantized directions (4 or 8, default: 8)
template <class MT, int NUM_DIR = 8>
class EsdDrawing : public EsdBase<MT, index_type> {
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

  int minPixels_{};
  float magTh_{}, magMul_{};
#ifndef NO_ADDED_SEEDS
  IndexVector addedSeeds_{};
#endif

  using EsdBase<MT, index_type>::points_;
  using EsdBase<MT, index_type>::segments_;

 public:
  /// @brief Construct an EsdDrawing detector.
  /// @param minPix Minimum number of support pixels for a valid segment
  /// @param magMul Maximum magnitude multiplicator between adjacent pixels
  /// @param magTh Minimum absolute magnitude threshold for edge pixels
  EsdDrawing(int minPix = 10, float magMul = 3, float magTh = 5)
      : EsdBase<MT, index_type>(), minPixels_(minPix), magTh_(magTh), magMul_(magMul) {
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

  EsdDrawing(const EsdDrawing&) = delete;
  EsdDrawing& operator=(const EsdDrawing&) = delete;

  /// @brief Get or set the minimum pixel count parameter.
  /// @param mp New value (pass Value::NAV() to query without setting)
  /// @return Current minimum pixel count
  Value valueMinPixel(const Value& mp = Value::NAV()) {
    if (mp.type()) minPixels(mp.getInt());
    return minPixels_;
  }

  /// @brief Get the minimum pixel count for a valid segment.
  /// @return Minimum pixel count
  int minPixels() const { return minPixels_; }

  /// @brief Set the minimum pixel count for a valid segment.
  /// @param mp New minimum pixel count
  void minPixels(int mp) { minPixels_ = mp; }

  /// @brief Get or set the magnitude multiplicator parameter.
  /// @param mm New value (pass Value::NAV() to query without setting)
  /// @return Current magnitude multiplicator
  Value valueMagMul(const Value& mm = Value::NAV()) {
    if (mm.type()) magMul(mm.getFloat());
    return magMul_;
  }

  /// @brief Get the magnitude multiplicator.
  /// @return Magnitude multiplicator
  float magMul() const { return magMul_; }

  /// @brief Set the magnitude multiplicator.
  /// @param mm New magnitude multiplicator
  void magMul(float mm) { magMul_ = mm; }

  /// @brief Get or set the magnitude threshold parameter.
  /// @param mt New value (pass Value::NAV() to query without setting)
  /// @return Current magnitude threshold
  Value valueMagThreshold(const Value& mt = Value::NAV()) {
    if (mt.type()) magThresold(mt.getFloat());
    return magTh_;
  }

  /// @brief Get the magnitude threshold.
  /// @return Magnitude threshold
  float magThresold() const { return magTh_; }

  /// @brief Set the magnitude threshold.
  /// @param mt New magnitude threshold
  void magThresold(float mt) { magTh_ = mt; }

  using EsdBase<MT, index_type>::detect;

  /// @brief Detect edge segments by tracing from seed points.
  /// Traces along gradient directions from each seed, building edge segments
  /// by following the strongest magnitude path forward and backward.
  /// @param dir Quantized direction map (CV_8S)
  /// @param mag Edge magnitude map
  /// @param seeds Seed point indices to start tracing from
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

    for_each(seeds.begin(), seeds.end(), [&](index_type idx) { search(idx); });

#ifndef NO_ADDED_SEEDS
    size_t c = 0;
    while (c != addedSeeds_.size()) search(addedSeeds_[c++]);
#endif

    // std::cout << "draw - added seeds: " << addedSeeds_.size() << std::endl;
  }

  /// @brief Get the name of this detector.
  /// @return "draw"
  std::string name() const { return "draw"; }

 private:
  /// @brief Check for valid adjacent pixel in a given direction.
  /// Examines the three candidate neighbors (dir-1, dir, dir+1) and
  /// selects the one with highest magnitude, updating the direction.
  /// @param idx Current pixel index
  /// @param dir Current direction (updated to reflect chosen neighbor)
  /// @return Index of the best adjacent pixel, or 0 if none is valid
  inline index_type checkAdjacent(index_type idx, char& dir) {
    char dirn = dir - 1;
    char dirp = dir + 1;

    const int dirIndex = static_cast<int>(dir);
    const int dirnIndex = static_cast<int>(dirn);
    const int dirpIndex = static_cast<int>(dirp);
    const ptrdiff_t offset = pdmap[dirIndex];
    const ptrdiff_t offsetN = pdmap[dirnIndex];
    const ptrdiff_t offsetP = pdmap[dirpIndex];
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
        dir = fixDir<NUM_DIR>(dirp);
#ifndef NO_ADDED_SEEDS
        addedSeeds_.push_back(nidxn);
#endif
      } else {
        v = vn;
        nidx = nidxn;
        dir = fixDir<NUM_DIR>(dirn);
#ifndef NO_ADDED_SEEDS
        if (vp > v) addedSeeds_.push_back(nidxp);
#endif
      }
    } else if (vp > v) {
      v = vp;
      nidx = nidxp;
      dir = fixDir<NUM_DIR>(dirp);
    }

    // is pixel already used or border or no magnitude
    const float vFloat = static_cast<float>(v);
    const float baseMag = static_cast<float>(pmag_[idx]);
    if (vFloat < magTh_ || pdir_[nidx] < -1 || vFloat > magMul_ * baseMag) return 0;

    return nidx;
  }

#ifndef NO_EDGE_THICK_CHECK

  /// @brief Check for thick lines and mark pixels for removal.
  /// Marks adjacent pixels in the given direction as invalid to thin edges.
  /// @param idx Current pixel index
  /// @param dir Direction to check for thickness
  inline void checkThick(index_type idx, char dir) {
    const int dirIndex = static_cast<int>(dir);
    const ptrdiff_t offset = pdmap[dirIndex];
    index_type nidx = static_cast<index_type>(static_cast<ptrdiff_t>(idx) + offset);
    if (pdir_[nidx] < -1) return;
    pdir_[nidx] = -4;
  }


  /// @brief Find the next adjacent pixel along the edge, with thickness check.
  /// Calls checkAdjacent and optionally removes thick-line pixels on diagonals.
  /// @param idx Current pixel index
  /// @param dir Current direction (may be updated)
  /// @return Index of the next adjacent pixel, or 0 if none found
  inline index_type findAdjacent(index_type idx, char& dir) {
    index_type nidx = checkAdjacent(idx, dir);
    // check for bad pixels (only on odd dirs - diagonals)
    if (nidx) {
      if (dir % 2) {
        checkThick(idx, dir - 1);
        checkThick(idx, dir + 1);
      }
    }
    return nidx;
  }
#else

  /// @brief Find the next adjacent pixel without thickness check.
  /// @param idx Current pixel index
  /// @param dir Current direction (may be updated)
  /// @return Index of the next adjacent pixel, or 0 if none found
  inline index_type findAdjacent(index_type idx, char& dir) { return checkAdjacent(idx, dir); }
#endif

  /// @brief Find the next adjacent pixel using the stored direction map.
  /// @param idx Current pixel index
  /// @return Index of the next adjacent pixel, or 0 if direction is invalid
  inline index_type findAdjacent(index_type idx) {
    char dir = pdir_[idx];
    return (dir < 0) ? 0 : findAdjacent(idx, dir);
  }

  /// @brief Extract a segment by tracing from a starting pixel.
  /// Follows adjacent pixels in the current direction, appending each
  /// pixel index to the points vector and marking it as visited.
  /// @param idx Starting pixel index
  void extractSegment(index_type idx) {
    char dir = pdir_[idx];

    index_type tmp;
    while (idx) {
#ifdef DRAW_MODE
      draw.ptr<cv::Vec3b>()[idx] = col;
      cv::imshow("draw", draw);
      cv::waitKey(1);
#endif
      points_.push_back(idx);
      tmp = idx;
      idx = findAdjacent(idx, dir);
      pdir_[tmp] = -3;
    }
  }

  /// @brief Search for an edge segment starting from a seed pixel.
  /// Attempts forward then backward tracing from the seed, building
  /// a complete edge segment. Handles closed contour detection.
  /// @param idx Seed pixel index
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

    std::reverse(this->points_.begin() + static_cast<typename std::vector<index_type>::difference_type>(seg_beg),
                 this->points_.end());

    // do fw
    pdmap = fwdmap;
    extractSegment(idx);
    seg_end = this->points_.size();
    if (seg_end - seg_beg > static_cast<size_t>(minPixels_)) segments_.push_back(EdgeSegment(seg_beg, seg_end));
  }
};

}  // namespace lsfm
