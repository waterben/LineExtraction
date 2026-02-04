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

//! stereo Point filter -> uses binning and x-axis sorting + constrains to sort
//! out bad line match candidates
template <class FT, class GV, class GT = typename GV::value_type, class DM = DescriptorMatch<FT>>
class StereoPointFilter : public FeatureFilter<FT>, public OptionManager {
  int height_;

  struct PointData {
    PointData(const lsfm::Vec2<FT>& p = lsfm::Vec2<FT>(), FT s = 0) : position(p), scale(s) {}
    PointData(const cv::Point2f& p, FT s = 0) : position(lsfm::Vec2<FT>(p.x, p.y)), scale(s) {}

    lsfm::Vec2<FT> position;
    FT scale;
  };

  std::vector<PointData> pdLeft_, pdRight_;
  int bins;

  // first rotation (4 areas in coordsystem), then number of bins, then variable sized vectors with candidates
  std::vector<std::vector<int>> bins_;

 public:
  typedef FT float_type;
  typedef GV geometric_vector;
  typedef GT geometric_type;


  StereoPointFilter(int height = 1, int binNum = 12) : height_(height), bins(binNum) {
    CV_Assert(height > 0);
    if (bins > height_) bins = height_;

    bins_.assign(bins, std::vector<int>());

    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("height", height, "int", "Image height."));
  }

  void train(const GV& left, const GV& right) {
    trainSide(left, pdLeft_);
    trainSide(right, pdRight_);
  }

  virtual bool filter(int lfIdx, int rfIdx) const {
    const PointData& ld = pdLeft_[lfIdx];
    const PointData& rd = pdRight_[rfIdx];

    // x of endpoints in left image have to be >= x on the right
    if (ld.position.x() < rd.position.x()) return true;

    return false;
  }

  inline const std::vector<PointData>& ldLeft() const { return pdLeft_; }

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
  inline void trainSide(const GV& points, std::vector<PointData>& data) {
    data.clear();
    data.reserve(points.size());
    for_each(points.begin(), points.end(), [&](const geometric_type& point) { data.push_back(PointData(point.pt)); });
  }

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
