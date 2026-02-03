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


/// @file GlobalRotationFilter.hpp
/// @brief Global rotation-based feature filtering.
/// Provides filtering based on global rotation constraints between feature sets.

#pragma once

#include <geometry/line.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <algorithm>
#include <valarray>

namespace lsfm {

//! simple brute force matcher -> compute distances between all candidates
template <class FT, class GV>
class GlobalRotationFilter : public FeatureFilter<FT>, public OptionManager {
  struct LineData {
    LineData(FT a = 0, FT l = 0) : angle(a), length(l) {}

    FT angle, length;
  };

  std::vector<LineData> ldLeft_, ldRight_;

 public:
  typedef FT float_type;
  typedef GV geometric_vector;
  typedef LineSegment<FT> geometric_type;

  static const FT TwoPI;

  GlobalRotationFilter()
      : rotUsable_(false),
        rot_(0),
        resScale_(10),
        histDiff_(static_cast<FT>(0.49)),
        lengthDiff_(static_cast<FT>(0.5)),
        angleTh_(static_cast<FT>(CV_PI / 8)),
        lengthTh_(4) {
    // std::string type = (sizeof(float_type) > 4 ? "FT" : "float");
    // this->options_.push_back(OptionManager::OptionEntry("k", k, "int", "Number of nearest neighbors."));
    // this->options_.push_back(OptionManager::OptionEntry("radius", radius, type, "Max distance between
    // descriptors."));
  }

  //! train filter
  void train(const GV& left, const GV& right) {
    l = &left;
    r = &right;

    ldLeft_.resize(left.size());
    ldRight_.resize(right.size());

    FT rotationAngle = TwoPI;

    // step 1: compute the angle histogram of lines in the left and right images
    unsigned int dim = static_cast<unsigned int>(360 / resScale_);  // number of the bins of histogram
    unsigned int index;                                             // index in the histogram

    FT direction;
    FT scalar = 180 / static_cast<FT>(resScale_ * CV_PI);      // used when compute the index
    FT angleShift = static_cast<FT>(resScale_ * CV_PI) / 360;  // make sure zero is the middle of the interval

    std::valarray<FT> angleHistLeft(static_cast<FT>(0), dim);
    std::valarray<FT> angleHistRight(static_cast<FT>(0), dim);
    std::valarray<FT> lengthLeft(
        static_cast<FT>(0), dim);  // lengthLeft[i] store the total line length of all the lines in the ith angle bin.
    std::valarray<FT> lengthRight(static_cast<FT>(0), dim);


    for (unsigned int i = 0; i < left.size(); ++i) {
      direction = (ldLeft_[i].angle = left[i].angle()) + static_cast<FT>(CV_PI) + angleShift;
      direction = direction < TwoPI ? direction : (direction - TwoPI);
      index = static_cast<unsigned int>(direction * scalar);
      ++angleHistLeft[index];
      lengthLeft[index] += (ldLeft_[i].length = left[i].length());
    }

    for (unsigned int i = 0; i < right.size(); ++i) {
      direction = (ldRight_[i].angle = right[i].angle()) + static_cast<FT>(CV_PI) + angleShift;
      direction = direction < TwoPI ? direction : (direction - TwoPI);
      index = static_cast<unsigned int>(direction * scalar);
      ++angleHistRight[index];
      lengthRight[index] += (ldRight_[i].length = right[i].length());
    }

    FT tmp =
        static_cast<FT>((1 / cv::norm(cv::_InputArray(&angleHistLeft[0], static_cast<int>(angleHistLeft.size())))));
    angleHistLeft *= tmp;
    tmp = static_cast<FT>((1 / cv::norm(cv::_InputArray(&angleHistRight[0], static_cast<int>(angleHistRight.size())))));
    angleHistRight *= tmp;
    tmp = static_cast<FT>((1 / cv::norm(cv::_InputArray(&lengthLeft[0], static_cast<int>(lengthLeft.size())))));
    lengthLeft *= tmp;
    tmp = static_cast<FT>((1 / cv::norm(cv::_InputArray(&lengthRight[0], static_cast<int>(lengthRight.size())))));
    lengthRight *= tmp;


    // step 2: find shift to decide the approximate global rotation
    std::vector<FT> difVec(dim);  // the difference vector between left histogram and shifted right histogram
    FT minDif = 10;               // the minimal angle histogram difference
    FT secondMinDif = 10;         // the second minimal histogram difference
    unsigned int minShift = std::numeric_limits<unsigned int>::max();  // the shift of right angle histogram when
                                                                       // minimal difference achieved
    unsigned int secondMinShift;  // the shift of right angle histogram when second minimal difference achieved

    std::vector<FT> lengthDifVec(dim);  // the length difference vector between left and right
    FT minLenDif = 10;                  // the minimal length difference
    FT secondMinLenDif = 10;            // the second minimal length difference
    unsigned int minLenShift = std::numeric_limits<unsigned int>::max();  // the shift of right length vector when
                                                                          // minimal length difference achieved
    unsigned int
        secondMinLenShift;  // the shift of right length vector when the second minimal length difference achieved

    FT normOfVec;
    for (unsigned int shift = 0; shift < dim; ++shift) {
      for (unsigned int j = 0; j < dim; ++j) {
        index = j + shift;
        index = index < dim ? index : (index - dim);
        difVec[j] = angleHistLeft[j] - angleHistRight[index];
        lengthDifVec[j] = lengthLeft[j] - lengthRight[index];
      }

      // find the minShift and secondMinShift for angle histogram
      normOfVec = static_cast<FT>(cv::norm(difVec));
      if (normOfVec < secondMinDif) {
        if (normOfVec < minDif) {
          secondMinDif = minDif;
          secondMinShift = minShift;
          minDif = normOfVec;
          minShift = shift;
        } else {
          secondMinDif = normOfVec;
          secondMinShift = shift;
        }
      }

      // find the minLenShift and secondMinLenShift of length vector
      normOfVec = static_cast<FT>(cv::norm(lengthDifVec));
      if (normOfVec < secondMinLenDif) {
        if (normOfVec < minLenDif) {
          secondMinLenDif = minLenDif;
          secondMinLenShift = minLenShift;
          minLenDif = normOfVec;
          minLenShift = shift;
        } else {
          secondMinLenDif = normOfVec;
          secondMinLenShift = shift;
        }
      }
    }


    rotationAngle = minShift * resScale_;
    if (rotationAngle > 90 && 360 - rotationAngle > 90) {
      // In most case we believe the rotation angle between two image pairs should belong to [-Pi/2, Pi/2]
      rotationAngle = rotationAngle - 180;
    }
    rotationAngle = static_cast<FT>(rotationAngle * CV_PI) / 180;

    // check whether there exist an approximate global rotation angle between image pair
    rotUsable_ = (minDif < histDiff_ && minLenDif < lengthDiff_);

    // std::cout << "minimal histgram distance = " << minDif << ", Approximate global rotation angle = " <<
    // rotationAngle << endl;
    rot_ = rotationAngle;
  }

  virtual bool filter(int lfIdx, int rfIdx) const {
    const LineData& l = ldLeft_[lfIdx];
    const LineData& r = ldRight_[rfIdx];

    // only length check
    if (!rotUsable_) return (std::abs(l.length - r.length) / std::min(l.length, r.length)) > lengthTh_;

    FT adiff = std::abs(l.angle + rot_ - r.angle);
    return (std::abs(TwoPI - adiff) > angleTh_ && adiff > angleTh_) ||
           ((std::abs(l.length - r.length) / std::min(l.length, r.length)) > lengthTh_);
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
  FT rot_;
  bool rotUsable_;

  FT resScale_;
  FT histDiff_;
  FT lengthDiff_;

  FT lengthTh_;
  FT angleTh_;

  const GV *l, *r;
};

template <class FT, class GV>
const FT GlobalRotationFilter<FT, GV>::TwoPI = static_cast<FT>(2 * CV_PI);

}  // namespace lsfm
