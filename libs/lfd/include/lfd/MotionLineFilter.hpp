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


#pragma once

#include <geometry/line.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <algorithm>
#include <array>
#include <assert.h>

namespace lsfm {

//! Walk along a line and get every pixel that is touched, with floatingpoint accuracy
template <class FT>
void pixelOfLine(FT x1, FT y1, FT x2, FT y2, std::vector<std::pair<int, int>>& pixel) {
  if (x1 > x2) {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  if (x1 < 0) x1 = FT(0);
  if (y1 < 0) y1 = FT(0);
  if (x2 < 0) x2 = FT(0);
  if (y2 < 0) y2 = FT(0);

  FT dirX = x2 - x1;
  FT dirY = y2 - y1;

  FT currX = x1, currY = y1;

  int prevX = static_cast<int>(std::floor(currX));
  int prevY = static_cast<int>(std::floor(currY));
  pixel.push_back(std::pair<int, int>(std::floor(prevX), std::floor(prevY)));

  if (dirY < 0) {
    while (prevX < static_cast<int>(std::floor(x2)) || prevY > static_cast<int>(std::floor(y2))) {
      FT xStep, yStep;

      if (std::floor(currX) == std::floor(x2) || dirX == 0) {
        xStep = std::numeric_limits<FT>::max();
      } else {
        xStep = std::abs((std::floor(currX + 1.0) - currX) / dirX);
      }
      if (std::floor(currY) == std::floor(y2) || dirY == 0) {
        yStep = std::numeric_limits<FT>::max();
      } else {
        yStep = std::max((currY == static_cast<FT>(std::floor(currY)) ? currY - static_cast<FT>(1)
                                                                      : static_cast<FT>(std::floor(currY))),
                         y2);
        yStep = std::abs((yStep - currY) / dirY);
      }

      if (xStep > yStep) {
        currX += yStep * dirX;
        currY = std::max((currY == static_cast<FT>(std::floor(currY)) ? currY - static_cast<FT>(1)
                                                                      : static_cast<FT>(std::floor(currY))),
                         y2);

        prevX = static_cast<int>(std::floor(currX));
        // dont jump two borders at once
        if (static_cast<int>(std::floor(currY)) == prevY) {
          prevY = static_cast<int>(std::floor(currY - 1));
        } else {
          prevY = static_cast<int>(std::floor(currY));
        }

      } else {
        currX = std::floor(currX + 1.0);
        currY += xStep * dirY;

        prevX = static_cast<int>(std::floor(currX));
        prevY = static_cast<int>(std::floor(currY));
      }
      pixel.push_back(std::pair<int, int>(prevX, prevY));
    }
  } else {
    while (prevX < static_cast<int>(std::floor(x2)) || prevY < static_cast<int>(std::floor(y2))) {
      FT xStep, yStep;

      if (std::floor(currX) == std::floor(x2) || dirX == 0) {
        xStep = std::numeric_limits<FT>::max();
      } else {
        xStep = std::abs((std::floor(currX + 1.0) - currX) / dirX);
      }
      if (std::floor(currY) == std::floor(y2)) {
        yStep = std::numeric_limits<FT>::max();
      } else {
        yStep = std::abs((std::floor(currY + 1.0) - currY) / dirY);
      }

      if (xStep > yStep) {
        currX += yStep * dirX;
        currY = std::floor(currY + 1.0);

        prevX = static_cast<int>(std::floor(currX));
        prevY = static_cast<int>(std::floor(currY));
      } else {
        currX = std::floor(currX + 1.0);
        currY += xStep * dirY;

        prevX = static_cast<int>(std::floor(currX));
        prevY = static_cast<int>(std::floor(currY));
      }
      pixel.push_back(std::pair<int, int>(prevX, prevY));
    }
  }
}

//! calculate the neighbouring bins to a set of bins
template <int nrBinsX, int nrBinsY = nrBinsX>
void getNeighbouringBins(const std::vector<std::pair<int, int>>& pixel,
                         std::vector<std::pair<int, int>>& pixelNeighbours,
                         const int searchRange) {
  std::array<std::vector<char>, nrBinsX> used;
  for (int i = 0; i < used.size(); ++i) {
    used.at(i).assign(nrBinsY, 0);
  }

  for_each(pixel.begin(), pixel.end(), [&used, &searchRange, &pixelNeighbours](const std::pair<int, int>& p) {
    for (int x = std::max(0, p.first - searchRange); x <= std::min(nrBinsX - 1, p.first + searchRange); ++x) {
      for (int y = std::max(0, p.second - searchRange); y <= std::min(nrBinsY - 1, p.second + searchRange); ++y) {
        if (used.at(x).at(y) == 0) {
          used.at(x).at(y) = 1;
          pixelNeighbours.push_back(std::pair<int, int>(x, y));
        }
      }
    }
  });
}


//! stereo line filter -> uses binning and x-axis sorting + constrains to sort
//! out bad line match candidates
template <class FT, class GV, unsigned int bins = 12, unsigned int angleBins = 12, class DM = DescriptorMatch<FT>>
class MotionLineFilter : public FeatureFilter<FT>, public OptionManager {
  FT maxPixelDist_, angleTh_;
  int height_, width_;

  struct LineData {
    LineData(const lsfm::Vec2<FT>& b = lsfm::Vec2<FT>(), const lsfm::Vec2<FT>& e = lsfm::Vec2<FT>(), FT a = 0, FT s = 0)
        : beg(b), end(e), angle(a), scale(s) {}

    lsfm::Vec2<FT> beg, end;
    FT angle, scale;
  };


  std::vector<LineData> newLines_, oldLines_;

  // first rotation (4 areas in coordsystem), then number of bins, then variable sized vectors with candidates
  std::array<std::array<std::array<std::vector<int>, bins>, bins>, angleBins> bins_;

 public:
  typedef FT float_type;
  typedef GV geometric_vector;
  typedef LineSegment<FT> geometric_type;

  /*
          struct Motion {
              Motion(const FT xDir = 0, const FT yDir = 0) : x(xDir), y(yDir) {}

              FT x, y;
          };
  */
  MotionLineFilter(FT maxPixDist = 200, FT angleTh = 5, int width = 0, int height = 0)
      : maxPixelDist_(maxPixDist), angleTh_(angleTh), height_(height), width_(width) {
    CV_Assert(height_ >= 0 && width_ >= 0);

    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("maxPixelDist", maxPixDist, type,
                                                        "Maximal distance between two corresponding lines."));
    // this->options_.push_back(OptionManager::OptionEntry("minOverlap", minOverlap, type, "Minimal Y overlap between
    // two corresponding lines."));
  }

  void train(const GV& newLines, const GV& oldLines) {
    trainSide(newLines, newLines_);
    trainSide(oldLines, oldLines_);
    // motionEstimate_ = motionEstimate;
  }

  virtual bool filter(int nfIdx, int ofIdx) const {
    // both filters given by binning
    //            return false;


    const LineData& nd = newLines_[nfIdx];
    const LineData& od = oldLines_[ofIdx];
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
    // skip lines with unsimilar orientations
    FT adiff = abs(nd.angle - od.angle);
    if (adiff > angleTh_) return true;

    return false;
  }


  //! create match set by filter
  //! only for lines with existing Model, thus they can be projected
  template <class FMV, class GV3>
  void create(const GV& newLines, const GV3& models, cv::Mat* projMat, FMV& matches) {
    matches.clear();
    matches.reserve(newLines.size() * models.size() / 10);

    // calculate predicted positions
    // GV predictions;
    std::vector<LineSegment<FT>> predictions;

    predictions.reserve(models.size());
    for (int i = 0; i < models.size(); ++i) {
      predictions.push_back(models.at(i).project2Line(projMat));
    }

    // clear bins
    for_each(bins_.begin(), bins_.end(), [&newLines](std::array<std::array<std::vector<int>, bins>, bins>& bs) {
      for_each(bs.begin(), bs.end(), [&newLines](std::array<std::vector<int>, bins>& b) {
        for_each(b.begin(), b.end(), [&newLines](std::vector<int>& v) {
          v.clear();
          v.reserve(newLines.size() / bins);
        });
      });
    });

    FT bstep_h = static_cast<FT>(height_) / bins;
    FT bstep_w = static_cast<FT>(width_) / bins;

    trainSide(newLines, newLines_);
    trainSideAndBins(predictions, oldLines_, bstep_h, bstep_w);

    std::vector<char> oidxList;
    size_t nsize = newLines_.size();
    oidxList.reserve(nsize);

    for (int nidx = 0; nidx < newLines_.size(); ++nidx) {
      const LineData ld = newLines_[nidx];
      oidxList.assign(nsize, 0);

      std::array<std::array<std::vector<int>, bins>, bins>& qbins =
          this->bins_[static_cast<int>(ld.angle / (360 / angleBins)) % angleBins];

      std::vector<std::pair<int, int>> pixel, pixelNeighbours;
      pixelOfLine((ld.beg.x() / width_) * static_cast<FT>(bins), (ld.beg.y() / height_) * static_cast<FT>(bins),
                  (ld.end.x() / width_) * static_cast<FT>(bins), (ld.end.y() / height_) * static_cast<FT>(bins), pixel);
      int searchRange = 1;
      getNeighbouringBins<bins>(pixel, pixelNeighbours, searchRange);

      for_each(pixelNeighbours.begin(), pixelNeighbours.end(),
               [this, &qbins, &nidx, &matches, &oidxList](std::pair<int, int>& p) {
                 std::vector<int>& bin = qbins.at(p.first).at(p.second);
                 for_each(bin.begin(), bin.end(), [this, &matches, &oidxList, &nidx](int oidx) {
                   if (!oidxList[nidx] && !this->filter(nidx, oidx)) {
                     matches.push_back(typename FMV::value_type(nidx, oidx));
                     //   ++lm[lidx];
                     //   ++rm[ridx];
                     ++oidxList[nidx];
                   }
                 });
               });
    }
  }


  //! track lines, only with average 2D Image-movement estimation
  template <class FMV, class MV>
  void create(
      const GV& newLines, const GV& previousLines, const std::pair<FT, FT>& avgMovement, FMV& matches, MV& nm, MV& pm) {
    matches.clear();
    matches.reserve(newLines.size() * previousLines.size() / 10);

    nm.resize(newLines.size(), 0);
    pm.resize(previousLines.size(), 0);

    // calculate predicted positions
    // GV predictions;
    std::vector<LineSegment<FT>> predictions;
    predictions.reserve(previousLines.size());
    for (int i = 0; i < previousLines.size(); ++i) {
      predictions.push_back(LineSegment<FT>(lsfm::Vec2<FT>(previousLines.at(i).startPoint().x() + avgMovement.first,
                                                           previousLines.at(i).startPoint().y() + avgMovement.second),
                                            lsfm::Vec2<FT>(previousLines.at(i).endPoint().x() + avgMovement.first,
                                                           previousLines.at(i).endPoint().y() + avgMovement.second)));
    }

    // clear bins
    for_each(bins_.begin(), bins_.end(), [&newLines](std::array<std::array<std::vector<int>, bins>, bins>& bs) {
      for_each(bs.begin(), bs.end(), [&newLines](std::array<std::vector<int>, bins>& b) {
        for_each(b.begin(), b.end(), [&newLines](std::vector<int>& v) {
          v.clear();
          v.reserve(newLines.size() / bins);
        });
      });
    });

    FT bstep_h = static_cast<FT>(height_) / bins;
    FT bstep_w = static_cast<FT>(width_) / bins;

    trainSide(newLines, newLines_);
    trainSideAndBins(predictions, oldLines_, bstep_h, bstep_w);

    std::vector<char> pIdxList;
    size_t nsize = newLines_.size();
    size_t pSize = previousLines.size();
    pIdxList.reserve(pSize);

    for (int nIdx = 0; nIdx < newLines_.size(); ++nIdx) {
      const LineData ld = newLines_[nIdx];
      pIdxList.assign(pSize, 0);

      std::vector<std::pair<int, int>> pixel, pixelNeighbours;
      pixelOfLine((ld.beg.x() / width_) * static_cast<FT>(bins), (ld.beg.y() / height_) * static_cast<FT>(bins),
                  (ld.end.x() / width_) * static_cast<FT>(bins), (ld.end.y() / height_) * static_cast<FT>(bins), pixel);
      int searchRange = 1;
      getNeighbouringBins<bins>(pixel, pixelNeighbours, searchRange);

      // loop through same and neighbouring angle-bins
      int aBinIdx = static_cast<int>(ld.angle / (360 / angleBins)) % angleBins;
      for (int bidx = -1; bidx <= 1; ++bidx) {
        int currABin = (aBinIdx + bidx) % angleBins;
        currABin = (currABin < 0 ? (currABin + angleBins) : currABin);
        std::array<std::array<std::vector<int>, bins>, bins>& aBin = this->bins_[currABin];

        for_each(pixelNeighbours.begin(), pixelNeighbours.end(),
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
  inline void trainSide(const GV& lines, std::vector<LineData>& data) {
    data.clear();
    data.reserve(lines.size());
    for_each(lines.begin(), lines.end(), [&](const geometric_type& line) {
      data.push_back(LineData(line.startPoint(), line.endPoint(), line.anglef()));
    });
  }

  inline void trainSideAndBins(std::vector<LineSegment<FT>>& lines,
                               std::vector<LineData>& data,
                               FT bstep_h,
                               FT bstep_w) {
    // clear line data
    data.clear();
    data.reserve(lines.size());
    // clear bins
    for_each(bins_.begin(), bins_.end(), [&lines](std::array<std::array<std::vector<int>, bins>, bins>& bs) {
      for_each(bs.begin(), bs.end(), [&lines](std::array<std::vector<int>, bins>& b) {
        for_each(b.begin(), b.end(), [&lines](std::vector<int>& v) {
          v.clear();
          v.reserve(lines.size() / bins);
        });
      });
    });

    int size = static_cast<int>(lines.size());
    for (int idx = 0; idx != size; ++idx) {
      const geometric_type& line = lines[idx];
      LineData ld(line.startPoint(), line.endPoint(), line.anglef(), std::abs(1 / line.normalX()));
      data.push_back(ld);

      std::array<std::array<std::vector<int>, bins>, bins>& qbins =
          this->bins_[static_cast<int>(ld.angle / (360 / angleBins)) % angleBins];

      std::vector<std::pair<int, int>> pixel;

      // This happens when lines reach over borders - for example projected lines
      /*                if((ld.beg.x() / width_ ) * (bins-1) >= 12){
                          std::cout << "1should not happen" << std::endl;
                      }
                      if((ld.beg.y() / height_ ) * (bins-1) >= 12){
                          std::cout << "2should not happen" << std::endl;
                      }
                      if((ld.end.x() / width_ ) * (bins-1) >= 12){
                          std::cout << "3should not happen" << std::endl;
                      }
                      if((ld.end.y() / height_ ) * (bins-1) >= 12){
                          std::cout << "4should not happen" << std::endl;
                      }
      */
      // just getting the bins, not the neighbours, neighbours are calculated in the calling function for the other set
      pixelOfLine((ld.beg.x() / width_) * (bins - 1), (ld.beg.y() / height_) * (bins - 1),
                  (ld.end.x() / width_) * (bins - 1), (ld.end.y() / height_) * (bins - 1), pixel);

      for_each(pixel.begin(), pixel.end(), [&qbins, &ld, &idx](std::pair<int, int>& p) {
        // these checks ensure that the array size is not exceeded, alternatively cut the line?
        // if(p.first >= 0 && p.first < bins && p.second >= 0 && p.second < bins)
        assert(p.first >= 0 && p.first < bins && p.second >= 0 && p.second < bins);
        ((qbins.at(p.first)).at(p.second)).push_back(idx);
      });


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

  // Motion motionEstimate_;
};


}  // namespace lsfm
