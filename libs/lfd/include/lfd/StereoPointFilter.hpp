//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file StereoPointFilter.hpp
/// @brief Stereo point filtering with epipolar constraints.
/// Provides filtering for stereo point correspondence matching.

#pragma once

#include <geometry/line.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <algorithm>
#include <array>

namespace lsfm {

/// @brief Stereo point filter using binning and x-axis constraints.
/// Uses spatial binning and stereo constraints (left x >= right x)
/// to filter out bad point match candidates.
/// @tparam FT Float type for computations
/// @tparam GV Geometric vector type (e.g., std::vector<cv::KeyPoint>)
/// @tparam GT Geometric element type (default: GV::value_type)
/// @tparam DM Descriptor match type (default: DescriptorMatch<FT>)
template <class FT, class GV, class GT = typename GV::value_type, class DM = DescriptorMatch<FT>>
class StereoPointFilter : public FeatureFilter<FT>, public OptionManager {
  int height_;  ///< Image height for binning

  /// @brief Pre-computed point data for efficient filtering.
  struct PointData {
    /// @brief Construct point data from a 2D vector.
    /// @param p Point position
    /// @param s Scale factor
    PointData(const lsfm::Vec2<FT>& p = lsfm::Vec2<FT>(), FT s = 0) : position(p), scale(s) {}

    /// @brief Construct point data from an OpenCV point.
    /// @param p OpenCV 2D point
    /// @param s Scale factor
    PointData(const cv::Point2f& p, FT s = 0) : position(lsfm::Vec2<FT>(p.x, p.y)), scale(s) {}

    lsfm::Vec2<FT> position;  ///< Point position
    FT scale;                 ///< Scale factor
  };

  std::vector<PointData> pdLeft_;   ///< Pre-computed data for left points
  std::vector<PointData> pdRight_;  ///< Pre-computed data for right points
  int bins;                         ///< Number of vertical bins

  /// @brief Spatial bins for efficient candidate lookup.
  /// Indexed by vertical bin, each containing point indices.
  std::vector<std::vector<int>> bins_;

 public:
  typedef FT float_type;        ///< Float type used
  typedef GV geometric_vector;  ///< Geometric vector type
  typedef GT geometric_type;    ///< Geometric element type


  /// @brief Construct a stereo point filter.
  /// @param height Image height (must be > 0)
  /// @param binNum Number of vertical bins for spatial partitioning
  StereoPointFilter(int height = 1, int binNum = 12) : height_(height), bins(binNum) {
    CV_Assert(height > 0);
    if (bins > height_) bins = height_;

    bins_.assign(bins, std::vector<int>());

    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("height", height, "int", "Image height."));
  }

  /// @brief Train the filter with left and right point sets.
  /// @param left Left image points
  /// @param right Right image points
  void train(const GV& left, const GV& right) {
    trainSide(left, pdLeft_);
    trainSide(right, pdRight_);
  }

  /// @brief Filter a match candidate by checking stereo constraints.
  /// Rejects matches where left point x-coordinate is less than right.
  /// @param lfIdx Left feature index
  /// @param rfIdx Right feature index
  /// @return True if the match should be rejected
  virtual bool filter(int lfIdx, int rfIdx) const {
    const PointData& ld = pdLeft_[lfIdx];
    const PointData& rd = pdRight_[rfIdx];

    // x of endpoints in left image have to be >= x on the right
    if (ld.position.x() < rd.position.x()) return true;

    return false;
  }

  /// @brief Get pre-computed left point data.
  /// @return Const reference to left point data vector
  inline const std::vector<PointData>& ldLeft() const { return pdLeft_; }

  /// @brief Get pre-computed right point data.
  /// @return Const reference to right point data vector
  inline const std::vector<PointData>& ldRight() const { return pdRight_; }

  using FeatureFilter<FT>::create;
  /*
          template<class FMV>
          void create(const GV& left, const GV& right, FMV& matches) {
              matches.clear();
              matches.reserve(left.size() * right.size() / 10);

              FT bstep = static_cast<FT>(height_) / bins;

              trainSide(left,pdLeft_);
              trainSideAndBins(right, bstep, pdRight_);

              std::vector<char> ridxList;
              size_t lsize = pdRight_.size();
              ridxList.reserve(lsize);

              int size = static_cast<int>(pdLeft_.size());
              for (int lidx = 0; lidx != size; ++lidx) {
                  const PointData ld = pdLeft_[lidx];
                  ridxList.assign(lsize,0);

                  const std::array<std::vector<int>,bins>& qbins = this->bins_[static_cast<int>(ld.angle / 90) % 4];

                  int start = static_cast<int>(ld.beg.y()) / bstep, end = static_cast<int>(ld.end.y()) / bstep;

                  if (start > end)
                      std::swap(start,end);
                  ++end;

                  if (end > bins)
                      end = bins;

                  if (end < 0)
                      end = 0;

                  if (start > bins)
                      start = bins;

                  if (start < 0)
                      start = 0;

                  for (; start < end; ++start) {
                      for_each(qbins[start].begin(), qbins[start].end(), [&,this](int ridx) {
                          if (!ridxList[ridx] && !this->filter(lidx,ridx)) {
                              matches.push_back(typename FMV::value_type(lidx,ridx));
                              ++ridxList[ridx];
                          }
                      });
                  }
              }
          }
  */
  /// @brief Create match candidates using spatial binning and stereo constraints.
  /// Generates match candidates filtered by bin proximity and stereo consistency,
  /// and tracks per-point match counts.
  /// @tparam FMV Feature match vector type
  /// @tparam MV Match count vector type
  /// @param left Left image points
  /// @param right Right image points
  /// @param matches Output match candidates
  /// @param lm Per-point match count for left set
  /// @param rm Per-point match count for right set
  template <class FMV, class MV>
  void create(const GV& left, const GV& right, FMV& matches, MV& lm, MV& rm) {
    matches.clear();
    matches.reserve(left.size() * right.size() / 10);

    const int searchHeightBins = 1;

    lm.resize(left.size(), 0);
    rm.resize(right.size(), 0);

    // FT bstep = static_cast<FT>(height_) / bins;

    trainSide(left, pdLeft_);
    trainSideAndBins(right, pdRight_);

    std::vector<char> ridxList;
    size_t lsize = pdRight_.size();
    ridxList.reserve(lsize);

    int size = static_cast<int>(pdLeft_.size());
    for (int lidx = 0; lidx != size; ++lidx) {
      const PointData pd = pdLeft_[lidx];
      ridxList.assign(lsize, 0);

      const std::vector<std::vector<int>>& qbins = this->bins_;

      int yBin = (pd.position.y() / static_cast<FT>(height_)) * static_cast<FT>(bins);
      yBin = std::min(std::max(0, yBin), bins);
      int start = std::max(0, yBin - searchHeightBins), end = std::min(yBin + searchHeightBins, bins);

      for (; start < end; ++start) {
        for_each(qbins[start].begin(), qbins[start].end(), [&, this](int ridx) {
          if (!ridxList[ridx] && !this->filter(lidx, ridx)) {
            matches.push_back(typename FMV::value_type(lidx, ridx));
            ++lm[lidx];
            ++rm[ridx];
            ++ridxList[ridx];
          }
        });
      }
    }
  }

 protected:
  /// @brief Handle option value changes.
  /// @param name Option name
  /// @param value New option value
  void setOptionImpl(const std::string& name, FT value) {
    /*if (name == "k") {
        if (value >= 0 && value <= std::numeric_limits<int>::max()) {
            k_ = static_cast<int>(value);
            this->options_[0].value = k_;
        }
    }
    else if (name == "radius") {
        if (value >= 0 && value <= std::numeric_limits<float_type>::max()) {
            radius_ = static_cast<float_type>(value);
            this->options_[1].value = radius_;
        }
    }*/
  }

 private:
  /// @brief Pre-compute point data for one side.
  /// @param points Input points
  /// @param data Output pre-computed point data
  inline void trainSide(const GV& points, std::vector<PointData>& data) {
    data.clear();
    data.reserve(points.size());
    for_each(points.begin(), points.end(), [&](const geometric_type& point) { data.push_back(PointData(point.pt)); });
  }

  /// @brief Pre-compute point data and populate spatial bins.
  /// @param points Input points
  /// @param data Output pre-computed point data
  inline void trainSideAndBins(const std::vector<geometric_type>& points, std::vector<PointData>& data) {
    // clear line data
    data.clear();
    data.reserve(points.size());
    // clear bin data
    for_each(bins_.begin(), bins_.end(), [&points](std::vector<int>& v) {
      v.clear();
      v.reserve(points.size() / 10);
    });

    int size = static_cast<int>(points.size());
    for (int idx = 0; idx != size; ++idx) {
      const geometric_type& point = points[idx];
      PointData pd(point.pt);
      data.push_back(pd);


      int binY = point.pt.y / height_ * (bins);
      this->bins_.at(binY).push_back(idx);

      /*
      std::array<std::vector<int>,bins>& qbins = this->bins_[static_cast<int>(pd.angle / 90) % 4];
      int start = static_cast<int>(pd.beg[1] / bstep), end = static_cast<int>(pd.end[1] / bstep);

      if (start > end)
          std::swap(start,end);
      ++end;

      if (end > bins)
          end = bins;

      if (end < 0)
          end = 0;

      if (start > bins)
          start = bins;

      if (start < 0)
          start = 0;


      for (; start < end; ++start) {
          qbins[start].push_back(idx);
      }
      */
    }
  }
};

}  // namespace lsfm
