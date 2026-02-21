//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file MotionPointFilter.hpp
/// @brief Motion-based filtering for point feature correspondence.
/// Provides filtering for temporal point feature matching.

#pragma once

#include <geometry/point.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <algorithm>
#include <array>

namespace lsfm {


/// @brief Motion-based point filter using spatial binning.
/// Uses 2D spatial binning and movement estimation to filter
/// out bad temporal point match candidates.
/// @tparam FT Float type for computations
/// @tparam GV Geometric vector type (e.g., std::vector<cv::KeyPoint>)
/// @tparam GT Geometric element type (default: GV::value_type)
/// @tparam bins Number of spatial bins per dimension (default: 12)
/// @tparam DM Descriptor match type (default: DescriptorMatch<FT>)
template <class FT,
          class GV,
          class GT = typename GV::value_type,
          unsigned int bins = 12,
          class DM = DescriptorMatch<FT>>
class MotionPointFilter : public FeatureFilter<FT>, public OptionManager {
  int height_;  ///< Image height
  int width_;   ///< Image width

  /// @brief Pre-computed point data for efficient filtering.
  struct PointData {
    /// @brief Construct from a 2D vector.
    /// @param p Point position
    /// @param s Scale factor
    PointData(const lsfm::Vec2<FT>& p = lsfm::Vec2<FT>(), FT s = 0) : position(p), scale(s) {}

    /// @brief Construct from an OpenCV point.
    /// @param p OpenCV 2D point
    /// @param s Scale factor
    PointData(const cv::Point2f& p, FT s = 0) : position(lsfm::Vec2<FT>(p.x, p.y)), scale(s) {}

    lsfm::Vec2<FT> position;  ///< Point position
    FT scale;                 ///< Scale factor
  };

  std::vector<PointData> newPoints_;  ///< Pre-computed data for new frame points
  std::vector<PointData> oldPoints_;  ///< Pre-computed data for previous frame points

  /// @brief 2D spatial bins for candidate lookup.
  /// Indexed by [x_bin][y_bin], each containing point indices.
  std::array<std::array<std::vector<int>, bins>, bins> bins_;

 public:
  typedef FT float_type;        ///< Float type used
  typedef GV geometric_vector;  ///< Geometric vector type
  typedef GT geometric_type;    ///< Geometric element type

  /*
          struct Motion {
              Motion(const FT xDir = 0, const FT yDir = 0) : x(xDir), y(yDir) {}

              FT x, y;
          };
  */
  /// @brief Construct a motion point filter.
  /// @param width Image width (must be >= 0)
  /// @param height Image height (must be >= 0)
  MotionPointFilter(int width = 1, int height = 1) : height_(height), width_(width) {
    CV_Assert(height_ >= 0 && width_ >= 0);

    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    //            this->options_.push_back(OptionManager::OptionEntry("maxPixelDist", maxPixDist, type, "Maximal
    //            distance between two corresponding Points."));
  }

  /// @brief Train the filter with new and old point sets.
  /// @param newPoints Current frame points
  /// @param oldPoints Previous frame points
  void train(const GV& newPoints, const GV& oldPoints) {
    trainSide(newPoints, newPoints_);
    trainSide(oldPoints, oldPoints_);
    // motionEstimate_ = motionEstimate;
  }

  /// @brief Filter a match candidate.
  /// Currently accepts all candidates (filtering done by binning).
  /// @param nfIdx New frame point index
  /// @param ofIdx Old frame point index
  /// @return True if the match should be rejected
  virtual bool filter(int nfIdx, int ofIdx) const {
    // both filters given by binning
    //            return false;


    const PointData& nd = newPoints_[nfIdx];
    const PointData& od = oldPoints_[ofIdx];
    /*
                FT maxPixelDistSQ = maxPixelDist_ * maxPixelDist_;

                FT xDist = nd.beg.x() - (od.beg.x() );
                xDist = xDist * xDist;
                FT yDist = nd.beg.y() - (od.beg.y() );
                yDist = yDist * yDist;
                if(maxPixelDistSQ < (xDist + yDist)){
                    xDist = nd.end.x() - (od.end.x() );
                    xDist = xDist * xDist;
                    yDist = nd.end.y() - (od.end.y() );
                    yDist = yDist * yDist;
                    if(maxPixelDistSQ < (xDist + yDist)){
                        return true;
                    }
                }
    */

    // skip Points with unsimilar orientations
    //            FT adiff = abs(nd.angle - od.angle);
    //            if (adiff > angleTh_)
    //                return true;


    return false;
  }


  //! create match set by filter
  //! only for Points with existing Model, thus they can be projected
  /*        template<class FMV, class GV3>
          void create(const GV& newPoints, const GV3& models, cv::Mat * projMat, FMV& matches) {

              matches.clear();
              matches.reserve(newPoints.size() * models.size() / 10);

              // calculate predicted positions
              //GV predictions;
              std::vector<LineSegment<FT>> predictions;

              predictions.reserve(models.size());
              for(int i = 0; i < models.size(); ++i){
                  predictions.push_back(models.at(i).project2Line(projMat));
              }

              // clear bins
              for_each(bins_.begin(), bins_.end(), [&newPoints](std::array<std::array<std::vector<int>,bins>,bins> &bs)
     { for_each(bs.begin(), bs.end(), [&newPoints](std::array<std::vector<int>,bins> &b) { for_each(b.begin(), b.end(),
     [&newPoints](std::vector<int> &v) { v.clear(); v.reserve(newPoints.size() / bins);
                      });
                  });
              });

              FT bstep_h = static_cast<FT>(height_) / bins;
              FT bstep_w = static_cast<FT>(width_) / bins;

              trainSide(newPoints, newPoints_);
              trainSideAndBins(predictions, oldPoints_, bstep_h, bstep_w);

              std::vector<char> oidxList;
              size_t nsize = newPoints_.size();
              oidxList.reserve(nsize);

              for(int nidx = 0; nidx < newPoints_.size(); ++nidx){
                  const PointData ld = newPoints_[nidx];
                  oidxList.assign(nsize,0);

                  std::array<std::array<std::vector<int>,bins>,bins>& qbins = this->bins_[static_cast<int>(ld.angle /
     (360 / angleBins)) % angleBins];

                  std::vector<std::pair<int, int>> pixel, pixelNeighbors;
                  pixelOfLine((ld.beg.x() / width_ ) * static_cast<FT>(bins), (ld.beg.y() / height_ ) *
     static_cast<FT>(bins), (ld.end.x() / width_ ) * static_cast<FT>(bins), (ld.end.y() / height_ ) *
     static_cast<FT>(bins), pixel); int searchRange = 1; getNeighbouringBins<bins>(pixel, pixelNeighbors, searchRange);

                  for_each(pixelNeighbors.begin(), pixelNeighbors.end(), [this, &qbins, &nidx, &matches,
     &oidxList](std::pair<int, int> &p){

                      std::vector<int> &bin = qbins.at(p.first).at(p.second);
                      for_each(bin.begin(), bin.end(), [this, &matches, &oidxList, &nidx](int oidx){

                          if (!oidxList[nidx] && !this->filter(nidx,oidx)) {
                              matches.push_back(typename FMV::value_type(nidx,oidx));
                           //   ++lm[lidx];
                           //   ++rm[ridx];
                              ++oidxList[nidx];
                          }

                      });
                  });
              }

          }
   */

  /// @brief Create match candidates using average movement estimation.
  /// Shifts previous points by estimated movement, bins them spatially,
  /// and generates candidates from neighboring bins.
  /// @tparam FMV Feature match vector type
  /// @tparam MV Match count vector type
  /// @param newPoints Current frame points
  /// @param previousPoints Previous frame points
  /// @param avgMovement Average 2D movement estimate (dx, dy)
  /// @param matches Output match candidates
  /// @param nm Per-point match count for new frame
  /// @param pm Per-point match count for previous frame
  template <class FMV, class MV>
  void create(const GV& newPoints,
              const GV& previousPoints,
              const std::pair<FT, FT>& avgMovement,
              FMV& matches,
              MV& nm,
              MV& pm) {
    matches.clear();
    matches.reserve(newPoints.size() * previousPoints.size() / 10);

    nm.resize(newPoints.size(), 0);
    pm.resize(previousPoints.size(), 0);

    // calculate predicted positions
    // GV predictions;
    std::vector<geometric_type> predictions;
    predictions.reserve(previousPoints.size());
    for (int i = 0; i < previousPoints.size(); ++i) {
      geometric_type pt(previousPoints.at(i));
      pt.pt.x += avgMovement.first;
      pt.pt.y += avgMovement.second;

      // predictions.push_back(geometric_type(previousPoints.at(i).pt.x + avgMovement.first, previousPoints.at(i).pt.y +
      // avgMovement.second));
      predictions.push_back(pt);
    }

    /*  Already done in trainSideAndBins
    // clear bins
    for_each(bins_.begin(), bins_.end(), [&newPoints](std::array<std::vector<int>,bins> &b) {
        for_each(b.begin(), b.end(), [&newPoints](std::vector<int> &v) {
            v.clear();
            v.reserve(newPoints.size() / bins);
        });
    });
*/
    //            FT bstep_h = static_cast<FT>(height_) / bins;
    //            FT bstep_w = static_cast<FT>(width_) / bins;

    trainSide(newPoints, newPoints_);
    trainSideAndBins(predictions, oldPoints_);

    std::vector<char> pIdxList;
    size_t nsize = newPoints_.size();
    size_t pSize = previousPoints.size();
    pIdxList.reserve(pSize);

    for (int nIdx = 0; nIdx < newPoints_.size(); ++nIdx) {
      const PointData pd = newPoints_[nIdx];
      pIdxList.assign(pSize, 0);

      std::vector<std::pair<int, int>> pixel, pixelNeighbors;
      //                pixelOfLine((ld.beg.x() / width_ ) * static_cast<FT>(bins), (ld.beg.y() / height_ ) *
      //                static_cast<FT>(bins), (ld.end.x() / width_ ) * static_cast<FT>(bins), (ld.end.y() / height_ ) *
      //                static_cast<FT>(bins), pixel);
      pixel.push_back(std::pair<int, int>((pd.position.x() / width_) * static_cast<FT>(bins),
                                          (pd.position.y() / height_) * static_cast<FT>(bins)));
      int searchRange = 1;
      getNeighbouringBins<bins>(pixel, pixelNeighbors, searchRange);

      // loop through same and neighboring angle-bins
      //                int aBinIdx = static_cast<int>(ld.angle);// (360 / angleBins)) % angleBins;
      //                for(int bidx = -1; bidx <= 1; ++bidx){

      //                    int currABin = (aBinIdx + bidx);// % angleBins;
      //                    currABin = (currABin < 0 ? (currABin + angleBins) : currABin);
      std::array<std::array<std::vector<int>, bins>, bins>& aBin = this->bins_;

      for_each(pixelNeighbors.begin(), pixelNeighbors.end(),
               [this, &aBin, &nIdx, &matches, &pIdxList, &nm, &pm](std::pair<int, int>& p) {
                 std::vector<int>& bin = aBin.at(p.first).at(p.second);
                 for_each(bin.begin(), bin.end(), [this, &matches, &pIdxList, &nIdx, &nm, &pm](int pIdx) {
                   if (!pIdxList[pIdx] && !this->filter(nIdx, pIdx)) {
                     matches.push_back(typename FMV::value_type(nIdx, pIdx));
                     ++nm[nIdx];
                     ++pm[pIdx];
                     ++pIdxList[pIdx];
                   }
                 });
               });
      //                }
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

  /// @brief Pre-compute point data and populate 2D spatial bins.
  /// @param points Input points (typically shifted by movement estimate)
  /// @param data Output pre-computed point data
  inline void trainSideAndBins(const std::vector<geometric_type>& points, std::vector<PointData>& data) {
    // clear point data
    data.clear();
    data.reserve(points.size());
    // clear bins
    for_each(bins_.begin(), bins_.end(), [&points](std::array<std::vector<int>, bins>& b) {
      for_each(b.begin(), b.end(), [&points](std::vector<int>& v) {
        v.clear();
        v.reserve(points.size() / bins);
      });
    });

    int size = static_cast<int>(points.size());
    for (int idx = 0; idx != size; ++idx) {
      const geometric_type& point = points[idx];
      PointData pd(point.pt);
      data.push_back(pd);

      // std::array<std::array<std::vector<int>,bins>,bins>& qbins = this->bins_;

      int binX = point.pt.x / width_ * (bins - 1);
      int binY = point.pt.y / height_ * (bins - 1);
      this->bins_.at(binX).at(binY).push_back(idx);

      /*
                      std::vector<std::pair<int, int>> pixel;

      /*                pixelOfLine((ld.beg.x() / width_ ) * (bins-1), (ld.beg.y() / height_ ) * (bins-1), (ld.end.x() /
      width_ ) * (bins-1), (ld.end.y() / height_ ) * (bins-1), pixel);

                      for_each(pixel.begin(), pixel.end(), [&qbins, &ld, &idx](std::pair<int, int> &p){
                          // these checks ensure that the array size is not exceeded, alternatively cut the line?
                          if(p.first >= 0 && p.first < bins && p.second >= 0 && p.second < bins)
                              ((qbins.at(p.first)).at(p.second)).push_back(idx);
                      });
      */

      /*
      int start = static_cast<int>(ld.beg.y() / bstep_h), end = static_cast<int>(ld.end.y() / bstep_h);

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
