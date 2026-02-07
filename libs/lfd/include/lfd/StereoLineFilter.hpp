//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file StereoLineFilter.hpp
/// @brief Stereo line filtering with geometric constraints.
/// Filters stereo line matches using epipolar geometry and spatial binning.

#pragma once

#include <geometry/line.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <algorithm>
#include <array>
#include <math.h>

namespace lsfm {


/// @brief Stereo line filter using binning and geometric constraints.
/// Uses spatial binning, x-axis sorting, angle consistency, and Y overlap
/// constraints to filter out bad stereo line match candidates.
/// @tparam FT Float type for computations
/// @tparam GV Geometric vector type (e.g., std::vector<LineSegment<FT>>)
/// @tparam bins Number of vertical spatial bins (default: 12)
/// @tparam DM Descriptor match type (default: DescriptorMatch<FT>)
template <class FT, class GV, unsigned int bins = 12, class DM = DescriptorMatch<FT> >
class StereoLineFilter : public FeatureFilter<FT>, public OptionManager {
  FT maxDisPx_;     ///< Maximum line endpoint distance in pixels
  FT angleTh_;      ///< Maximum angle difference between matching lines
  FT minYOverlap_;  ///< Minimum Y overlap ratio (0-1)
  int height_;      ///< Image height for binning

  /// @brief Pre-computed line data for efficient filtering.
  struct LineData {
    /// @brief Construct line data.
    /// @param b Start point of the line
    /// @param e End point of the line
    /// @param a Angle in degrees (0-360)
    /// @param s Scale factor (inverse of normal X component)
    LineData(const lsfm::Vec2<FT>& b = lsfm::Vec2<FT>(), const lsfm::Vec2<FT>& e = lsfm::Vec2<FT>(), FT a = 0, FT s = 0)
        : beg(b), end(e), angle(a), scale(s) {}

    lsfm::Vec2<FT> beg;  ///< Start point
    lsfm::Vec2<FT> end;  ///< End point
    FT angle;            ///< Line angle in degrees
    FT scale;            ///< Scale factor
  };

  std::vector<LineData> ldLeft_;   ///< Pre-computed data for left lines
  std::vector<LineData> ldRight_;  ///< Pre-computed data for right lines

  /// @brief Spatial bins for candidate lookup.
  /// First index: quadrant (4 angle regions), second: vertical bin.
  std::array<std::array<std::vector<int>, bins>, 4> bins_;

  static constexpr FT H_LINE_TOL = static_cast<FT>(20);          ///< Horizontal line angle tolerance in degrees
  static constexpr FT MIN_VERTICAL_LENGTH = static_cast<FT>(6);  ///< Minimum vertical projection length

 public:
  typedef FT float_type;                   ///< Float type used
  typedef GV geometric_vector;             ///< Geometric vector type
  typedef LineSegment<FT> geometric_type;  ///< Geometric element type


  /// @brief Construct a stereo line filter.
  /// @param height Image height (must be >= 0)
  /// @param maxDisPx Maximum distance between endpoints in pixels
  /// @param angleTh Maximum angle difference between matching lines in degrees
  /// @param minYOverlap Minimum Y overlap ratio (0-1)
  StereoLineFilter(int height, FT maxDisPx = 10000, FT angleTh = 45, FT minYOverlap = 0.5)
      : maxDisPx_(maxDisPx),
        angleTh_(angleTh),
        minYOverlap_(minYOverlap),
        height_(height),
        ldLeft_(),
        ldRight_(),
        bins_() {
    CV_Assert(height >= 0 && maxDisPx_ >= 0 && angleTh > 0 && minYOverlap > 0 && minYOverlap <= 1);

    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("height", height, "int", "Image height."));
    this->options_.push_back(
        OptionManager::OptionEntry("maxDisPx", maxDisPx, type, "Maximal line distance in pixels."));
    this->options_.push_back(
        OptionManager::OptionEntry("angleTh", angleTh, type, "Maximal angle between two corresponding lines."));
    this->options_.push_back(OptionManager::OptionEntry(
        "minYOverlap", minYOverlap, type, "Minimal Y overlap between two corresponding lines (Range 0-1)"));
  }

  /// @brief Train the filter with left and right line sets.
  /// @param left Left image line segments
  /// @param right Right image line segments
  void train(const GV& left, const GV& right) {
    if (height_ <= 0) std::cout << "SLF: Height must not be zero!" << std::endl;
    trainSide(left, ldLeft_);
    trainSide(right, ldRight_);
  }

  /// @brief Filter a match candidate using geometric constraints.
  /// Checks stereo disparity direction, maximum distance, angle similarity,
  /// and Y overlap between the line pair.
  /// @param ld Left line data
  /// @param rd Right line data
  /// @return True if the match should be rejected
  virtual bool filter(const LineData& ld, const LineData& rd) const {
    // if(height_ <= 0) std::cout << "SLF: Height must not be zero!" << std::endl;
    assert(height_ > 0);

    // vertical length too short?
    if (fabs(ld.beg.y() - ld.end.y()) < static_cast<FT>(MIN_VERTICAL_LENGTH) ||
        fabs(rd.beg.y() - rd.end.y()) < static_cast<FT>(MIN_VERTICAL_LENGTH)) {
      //                std::cout << " too short on vertical length";
      //                return true;
    }

    // x's of endpoints in left image have to be >= than on the right
    if (ld.beg.x() < rd.beg.x() || ld.end.x() < rd.end.x()) {
      //                std::cout << " stereo constraint not met, left more right...";
      return true;
    }

    // if distance between left x and right x > maxDist_, filter
    if (ld.beg.x() - rd.beg.x() > maxDisPx_ || ld.end.x() - rd.end.x() > maxDisPx_) {
      //                std::cout << " x-Distance too high";
      return true;
    }
    /*
                if(lsLeft.normalY() > 0.985 || lsLeft.normalY() < -0.985 ||
                   lsRight.normalY() > 0.985 || lsRight.normalY() < -0.985 )
                    return false;
    */
    // skip lines that are nearly horizontal
    FT angleMod180 = static_cast<FT>(fmod(ld.angle, 180));
    if (angleMod180 < H_LINE_TOL || angleMod180 > (static_cast<FT>(180) - H_LINE_TOL)) {
      //                std::cout << " too horizontal";
      //                return true;
    }

    // skip lines with unsimilar orientations
    FT adiff = fabs(ld.angle - rd.angle);
    adiff += (adiff > 180) ? -360 : 0;
    adiff = fabs(adiff);
    if (adiff > angleTh_) {
      //                std::cout << " unsimilar orientation";
      return true;
    }

    // scale angle diff to increase the threshold
    // for more horizontal lines up to pi / 4
    // if (adiff > std::min(angleTh_ * ld.scale, static_cast<FT>(45) ) )
    // if (adiff > angleTh_)
    //    return true;

    /*
    // skip line pairs with main direction changes
    if ((ld.angle < 90 && rd.angle > 90) ||  (ld.angle > 90 && rd.angle < 90) ||
        (ld.angle < 180 && rd.angle > 180) || (ld.angle > 180 && rd.angle < 180) ||
        (ld.angle < 270 && rd.angle > 270) ||(ld.angle > 270 && rd.angle < 270))
        return true;
    */

    // no y overlap (remember that lines may also be kicked out if they are going in opposite directions)
    if (!((ld.beg.y() <= rd.end.y() && rd.beg.y() <= ld.end.y()) ||
          (ld.beg.y() >= rd.end.y() && rd.beg.y() >= ld.end.y()))) {
      //                std::cout << " no y Overlap at all";
      return true;
    }

    // min y Overlap Ratio for one of both lines required
    FT l1 = fabs(ld.end.y() - ld.beg.y()) * minYOverlap_, l2 = fabs(rd.end.y() - rd.beg.y()) * minYOverlap_, l3;

    if (rd.beg.y() > ld.beg.y())
      l3 = fabs(ld.end.y() - rd.beg.y());
    else
      l3 = fabs(rd.end.y() - ld.beg.y());
    if (l3 < l1 && l3 < l2) {
      //                std::cout << " not enough y Overlap";
      return true;
    }

    return false;
  }

  /// @brief Filter a match candidate by index.
  /// @param lfIdx Left feature index
  /// @param rfIdx Right feature index
  /// @return True if the match should be rejected
  virtual bool filter(int lfIdx, int rfIdx) const {
    const LineData& ld = ldLeft_[static_cast<size_t>(lfIdx)];
    const LineData& rd = ldRight_[static_cast<size_t>(rfIdx)];
    return filter(ld, rd);
  }

  /// @brief Filter a match candidate from line segment objects.
  /// @param l Left line segment
  /// @param r Right line segment
  /// @return True if the match should be rejected
  virtual bool filter(geometric_type l, geometric_type r) const {
    const LineData& ld = LineData(l.startPoint(), l.endPoint(), l.anglef(), std::abs(1 / l.normalX()));
    const LineData& rd = LineData(r.startPoint(), r.endPoint(), r.anglef(), std::abs(1 / r.normalX()));
    return filter(ld, rd);
  }

  /// @brief Get pre-computed left line data.
  /// @return Const reference to left line data vector
  inline const std::vector<LineData>& ldLeft() const { return ldLeft_; }

  /// @brief Get pre-computed right line data.
  /// @return Const reference to right line data vector
  inline const std::vector<LineData>& ldRight() const { return ldRight_; }

  using FeatureFilter<FT>::create;

  /// @brief Create match candidates using spatial binning.
  /// Generates candidates filtered by bin proximity and stereo constraints.
  /// @tparam FMV Feature match vector type
  /// @param left Left line segments
  /// @param right Right line segments
  /// @param matches Output match candidates
  template <class FMV>
  void create(const GV& left, const GV& right, FMV& matches) {
    if (height_ <= 0) std::cout << "SLF: Height must not be zero!" << std::endl;
    matches.clear();
    matches.reserve(left.size() * right.size() / 10);

    FT bstep = static_cast<FT>(height_) / bins;

    trainSide(left, ldLeft_);
    trainSideAndBins(right, bstep, ldRight_);

    std::vector<char> ridxList;
    size_t lsize = ldRight_.size();
    ridxList.reserve(lsize);

    int size = static_cast<int>(ldLeft_.size());
    for (int lidx = 0; lidx != size; ++lidx) {
      const LineData ld = ldLeft_[static_cast<size_t>(lidx)];
      ridxList.assign(lsize, 0);

      const std::array<std::vector<int>, bins>& qbins =
          this->bins_[static_cast<size_t>(static_cast<int>(ld.angle / 90) % 4)];

      int start = static_cast<int>(ld.beg.y()) / bstep, end = static_cast<int>(ld.end.y()) / bstep;

      if (start > end) std::swap(start, end);
      ++end;

      if (end > static_cast<int>(bins)) end = static_cast<int>(bins);

      if (end < 0) end = 0;

      if (start > static_cast<int>(bins)) start = static_cast<int>(bins);

      if (start < 0) start = 0;

      for (; start < end; ++start) {
        for_each(qbins[static_cast<size_t>(start)].begin(), qbins[static_cast<size_t>(start)].end(),
                 [&, this](int ridx) {
                   if (!ridxList[static_cast<size_t>(ridx)] && !this->filter(lidx, ridx)) {
                     matches.push_back(typename FMV::value_type(lidx, ridx));
                     ++ridxList[static_cast<size_t>(ridx)];
                   }
                 });
      }
    }
  }

  /// @brief Create match candidates with per-feature match counts.
  /// Generates candidates and tracks how many matches each feature has.
  /// @tparam FMV Feature match vector type
  /// @tparam MV Match count vector type
  /// @param left Left line segments
  /// @param right Right line segments
  /// @param matches Output match candidates
  /// @param lm Per-feature match count for left set
  /// @param rm Per-feature match count for right set
  template <class FMV, class MV>
  void create(const GV& left, const GV& right, FMV& matches, MV& lm, MV& rm) {
    if (height_ <= 0) std::cout << "SLF: Height must not be zero!" << std::endl;
    matches.clear();
    matches.reserve(left.size() * right.size() / 10);

    lm.resize(left.size(), 0);
    rm.resize(right.size(), 0);

    FT bstep = static_cast<FT>(height_) / bins;

    trainSide(left, ldLeft_);
    trainSideAndBins(right, bstep, ldRight_);

    std::vector<char> ridxList;
    size_t lsize = ldRight_.size();
    ridxList.reserve(lsize);

    int size = static_cast<int>(ldLeft_.size());
    for (int lidx = 0; lidx != size; ++lidx) {
      const LineData ld = ldLeft_[static_cast<size_t>(lidx)];
      ridxList.assign(lsize, 0);

      const std::array<std::vector<int>, bins>& qbins =
          this->bins_[static_cast<size_t>(static_cast<int>(ld.angle / 90) % 4)];

      int start = static_cast<int>(ld.beg[1] / bstep), end = static_cast<int>(ld.end[1] / bstep);

      if (start > end) std::swap(start, end);
      ++end;

      if (end > static_cast<int>(bins)) end = static_cast<int>(bins);

      if (end < 0) end = 0;

      if (start > static_cast<int>(bins)) start = static_cast<int>(bins);

      if (start < 0) start = 0;

      for (; start < end; ++start) {
        for_each(qbins[static_cast<size_t>(start)].begin(), qbins[static_cast<size_t>(start)].end(),
                 [&, this](int ridx) {
                   if (!ridxList[static_cast<size_t>(ridx)] && !this->filter(lidx, ridx)) {
                     matches.push_back(typename FMV::value_type(lidx, ridx));
                     ++lm[static_cast<size_t>(lidx)];
                     ++rm[static_cast<size_t>(ridx)];
                     ++ridxList[static_cast<size_t>(ridx)];
                   }
                 });
      }
    }
  }

 private:
  /// @brief Pre-compute line data for one side.
  /// @param lines Input line segments
  /// @param data Output pre-computed line data
  inline void trainSide(const GV& lines, std::vector<LineData>& data) {
    data.clear();
    data.reserve(lines.size());
    for_each(lines.begin(), lines.end(), [&](const geometric_type& line) {
      data.push_back(LineData(line.startPoint(), line.endPoint(), line.anglef(), std::abs(1 / line.normalX())));
    });
  }

  /// @brief Pre-compute line data and populate spatial bins.
  /// @param lines Input line segments
  /// @param bstep Bin step size in pixels
  /// @param data Output pre-computed line data
  inline void trainSideAndBins(const GV& lines, FT bstep, std::vector<LineData>& data) {
    // clear line data
    data.clear();
    data.reserve(lines.size());
    // clear bin data
    for_each(bins_.begin(), bins_.end(), [&lines](std::array<std::vector<int>, bins>& b) {
      for_each(b.begin(), b.end(), [&lines](std::vector<int>& v) {
        v.clear();
        v.reserve(lines.size() / 10);
      });
    });

    int size = static_cast<int>(lines.size());
    for (int idx = 0; idx != size; ++idx) {
      const geometric_type& line = lines[static_cast<size_t>(idx)];
      LineData ld(line.startPoint(), line.endPoint(), line.anglef(), std::fabs(1 / line.normalX()));
      data.push_back(ld);

      std::array<std::vector<int>, bins>& qbins = this->bins_[static_cast<size_t>(static_cast<int>(ld.angle / 90) % 4)];
      int start = static_cast<int>(ld.beg[1] / bstep), end = static_cast<int>(ld.end[1] / bstep);

      if (start > end) std::swap(start, end);
      ++end;

      if (end > static_cast<int>(bins)) end = static_cast<int>(bins);

      if (end < 0) end = 0;

      if (start > static_cast<int>(bins)) start = static_cast<int>(bins);

      if (start < 0) start = 0;


      for (; start < end; ++start) {
        qbins[static_cast<size_t>(start)].push_back(idx);
      }
    }
  }
};

}  // namespace lsfm
