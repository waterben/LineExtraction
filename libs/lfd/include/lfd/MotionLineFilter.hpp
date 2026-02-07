//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file MotionLineFilter.hpp
/// @brief Motion-based line filtering for temporal correspondence.
/// Filters line matches based on pixel distance and angle constraints with binning optimization.

#pragma once

#include <geometry/line.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <algorithm>
#include <array>
#include <assert.h>

namespace lsfm {

/// @brief Walk along a line and get pixels that are touched with floating-point accuracy.
/// @tparam FT Float type
/// @param x1 Start x coordinate
/// @param y1 Start y coordinate
/// @param x2 End x coordinate
/// @param y2 End y coordinate
/// @param pixel Output vector of (x, y) pixel coordinates touched by the line
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

/// @brief Calculate the neighbouring bins to a set of bins.
/// Expands a set of bin coordinates by including all bins within the specified
/// search range, avoiding duplicates.
/// @tparam nrBinsX Number of bins along the X axis
/// @tparam nrBinsY Number of bins along the Y axis (defaults to nrBinsX)
/// @param pixel Input vector of bin coordinates
/// @param pixelNeighbours Output vector of neighbouring bin coordinates
/// @param searchRange Number of bins to expand in each direction
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


/// @brief Motion-based line filter using spatial binning and angle constraints.
/// Uses binning and x-axis sorting with constraints to filter out bad line
/// match candidates for temporal line correspondence.
/// @tparam FT Floating-point type
/// @tparam GV Geometric vector type (container of line segments)
/// @tparam bins Number of spatial bins per axis (default: 12)
/// @tparam angleBins Number of angular bins (default: 12)
/// @tparam DM Descriptor match type (default: DescriptorMatch<FT>)
template <class FT, class GV, unsigned int bins = 12, unsigned int angleBins = 12, class DM = DescriptorMatch<FT>>
class MotionLineFilter : public FeatureFilter<FT>, public OptionManager {
  /// @brief Maximum pixel distance for correspondence and angle threshold (degrees).
  FT maxPixelDist_, angleTh_;
  /// @brief Image dimensions (height and width) for bin calculation.
  int height_, width_;

  /// @brief Internal storage for line segment data used during filtering.
  struct LineData {
    /// @brief Construct a LineData instance.
    /// @param b Start point of the line segment
    /// @param e End point of the line segment
    /// @param a Angle of the line segment in degrees
    /// @param s Scale factor of the line segment
    LineData(const lsfm::Vec2<FT>& b = lsfm::Vec2<FT>(), const lsfm::Vec2<FT>& e = lsfm::Vec2<FT>(), FT a = 0, FT s = 0)
        : beg(b), end(e), angle(a), scale(s) {}

    lsfm::Vec2<FT> beg, end;  ///< Start and end points of the line segment.
    FT angle, scale;          ///< Angle (degrees) and scale factor.
  };


  /// @brief Cached line data for new and old (previous) frames.
  std::vector<LineData> newLines_, oldLines_;

  /// @brief 3D bin array indexed by [angleBin][xBin][yBin] storing candidate line indices.
  std::array<std::array<std::array<std::vector<int>, bins>, bins>, angleBins> bins_;

 public:
  typedef FT float_type;                   ///< Floating-point type used for computations.
  typedef GV geometric_vector;             ///< Container type for geometric line data.
  typedef LineSegment<FT> geometric_type;  ///< Line segment type used as geometric primitive.

  /*
          struct Motion {
              Motion(const FT xDir = 0, const FT yDir = 0) : x(xDir), y(yDir) {}

              FT x, y;
          };
  */
  /// @brief Construct a MotionLineFilter with given constraints.
  /// @param maxPixDist Maximum pixel distance for line correspondence
  /// @param angleTh Angle threshold in degrees for filtering
  /// @param width Image width in pixels
  /// @param height Image height in pixels
  MotionLineFilter(FT maxPixDist = 200, FT angleTh = 5, int width = 0, int height = 0)
      : maxPixelDist_(maxPixDist), angleTh_(angleTh), height_(height), width_(width) {
    CV_Assert(height_ >= 0 && width_ >= 0);

    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("maxPixelDist", maxPixDist, type,
                                                        "Maximal distance between two corresponding lines."));
    // this->options_.push_back(OptionManager::OptionEntry("minOverlap", minOverlap, type, "Minimal Y overlap between
    // two corresponding lines."));
  }

  /// @brief Train the filter with new and old line sets.
  /// @param newLines Line segments from the current frame
  /// @param oldLines Line segments from the previous frame
  void train(const GV& newLines, const GV& oldLines) {
    trainSide(newLines, newLines_);
    trainSide(oldLines, oldLines_);
    // motionEstimate_ = motionEstimate;
  }

  /// @brief Filter a candidate match based on angle similarity.
  /// @param nfIdx Index of the line in the new frame
  /// @param ofIdx Index of the line in the old frame
  /// @return True if the match should be rejected, false if accepted
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


  /// @brief Create match set by filter using projected 3D models.
  /// Only for lines with an existing model that can be projected to 2D.
  /// @tparam FMV Feature match vector type
  /// @tparam GV3 3D geometric vector type (container of 3D line models)
  /// @param newLines Current frame line segments
  /// @param models 3D line models to project
  /// @param projMat Projection matrix for 3D-to-2D projection
  /// @param matches Output vector of line matches
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


  /// @brief Track lines using average 2D image movement estimation.
  /// @tparam FMV Feature match vector type
  /// @tparam MV Match count vector type
  /// @param newLines Current frame line segments
  /// @param previousLines Previous frame line segments
  /// @param avgMovement Average (x, y) movement between frames
  /// @param matches Output vector of line matches
  /// @param nm Output match count per new line
  /// @param pm Output match count per previous line
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
  /// @brief Set a filter option by name.
  /// @param name Option name to set
  /// @param value New value for the option
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
  /// @brief Extract line data from a set of line segments.
  /// @param lines Input line segments
  /// @param data Output vector of extracted LineData
  inline void trainSide(const GV& lines, std::vector<LineData>& data) {
    data.clear();
    data.reserve(lines.size());
    for_each(lines.begin(), lines.end(), [&](const geometric_type& line) {
      data.push_back(LineData(line.startPoint(), line.endPoint(), line.anglef()));
    });
  }

  /// @brief Extract line data and populate spatial bins for a set of lines.
  /// @param lines Input line segments (potentially projected)
  /// @param data Output vector of extracted LineData
  /// @param bstep_h Bin step size along the height axis
  /// @param bstep_w Bin step size along the width axis
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
