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

#include <lfd/FeatureMatcher.hpp>
#include <lfd/LRDescriptor.hpp>
#include <lfd/StereoLineFilter.hpp>

namespace lsfm {


//! stereo line Matcher
template <class FT, class GV = std::vector<LineSegment<FT>>, class DC_Helper = GchGradImgInterpolate<FT>>
class StereoLineMatcher : public OptionManager {
 public:
  typedef FT float_type;

  typedef GV geometric_vector;
  typedef typename geometric_vector::value_type geometric_type;

  typedef FdcGenericLR<FT, geometric_type, DC_Helper> descriptor_creator;
  typedef typename descriptor_creator::descriptor_type descriptor_type;
  typedef std::vector<descriptor_type> descriptor_vector;

  typedef DescriptorMatch<FT> match_type;
  typedef std::vector<match_type> match_vector;


 private:
  StereoLineFilter<FT, GV> slf;
  FmBruteForce<FT, descriptor_type, match_type> bfm;

  descriptor_creator *creatorL, *creatorR;
  typename descriptor_creator::FdcPtr creatorLPtr, creatorRPtr;

  descriptor_vector dscLeft_, dscRight_;
  std::vector<size_t> mLeft_, mRight_;

  FT distTh_;

 public:
  typedef typename descriptor_creator::FdcPtr FdcPtr;


  StereoLineMatcher(descriptor_creator& cL,
                    descriptor_creator& cR,
                    int height,
                    FT maxDist = 10000,
                    FT angleTh = 5,
                    FT minYOverlap = 0.5,
                    FT distTh = 0,
                    FT r = 0,
                    int kk = 0)
      : slf(height, maxDist, angleTh, minYOverlap), bfm(r, kk), creatorL(&cL), creatorR(&cR), distTh_(distTh) {
    CV_Assert(distTh_ >= 0);
    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("distTh", distTh, type, "Distance threshold (0 = auto)."));
  }

  StereoLineMatcher(const FdcPtr& cL = FdcPtr(),
                    const FdcPtr& cR = FdcPtr(),
                    int height = 0,
                    FT maxDist = 10000,
                    FT angleTh = 5,
                    FT minYOverlap = 0.5,
                    FT distTh = 0,
                    FT r = 0,
                    int kk = 0)
      : slf(height, maxDist, angleTh, minYOverlap),
        bfm(r, kk),
        creatorL(0),
        creatorR(0),
        creatorLPtr(cL),
        creatorRPtr(cR),
        distTh_(distTh) {
    if (!creatorLPtr.empty()) creatorL = dynamic_cast<descriptor_creator*>(creatorLPtr.operator->());
    if (!creatorRPtr.empty()) creatorR = dynamic_cast<descriptor_creator*>(creatorRPtr.operator->());

    CV_Assert(distTh_ >= 0);
    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("distTh", distTh, type, "Distance threshold (0 = auto)."));
  }

  void match(const geometric_vector& left,
             const geometric_vector& right,
             const descriptor_vector& dscL,
             const descriptor_vector& dscR,
             match_vector& matches) {
    match_vector candidates;
    slf.create(left, right, candidates);
    final(left, right, dscL, dscR, candidates, matches);
  }

  void match(const geometric_vector& left, const geometric_vector& right, match_vector& matches) {
    CV_Assert(creatorL != 0 && creatorR != 0);
    match_vector candidates;
    slf.create(left, right, candidates, mLeft_, mRight_);
    creatorL->createList(left, mLeft_, dscLeft_);
    creatorR->createList(right, mRight_, dscRight_);
    final(left, right, dscLeft_, dscRight_, candidates, matches);
  }

  StereoLineFilter<FT, GV>& getFilter() { return slf; }


  FmBruteForce<FT, descriptor_type, std::vector<match_vector>>& getMatcher() { return bfm; }

  descriptor_vector getDescriptorLeft() { return dscLeft_; }
  descriptor_vector getDescriptorRight() { return dscRight_; }


 protected:
  void final(const geometric_vector& left,
             const geometric_vector& right,
             const descriptor_vector& dscL,
             const descriptor_vector& dscR,
             const match_vector& candidates,
             match_vector& matches) {
    bfm.trainMatches(dscL, dscR, candidates);

    const match_vector& candidatesL = bfm.graph().distances;
    const std::vector<size_t>& relationsL = bfm.graph().relations;

    match_vector candidatesR = candidatesL;
    std::vector<size_t> relationsR;

    size_t count = 0;
    size_t startL = 0;
    FT mean = 0;
    for_each(relationsL.begin() + 1, relationsL.end(), [&](size_t endL) {
      if (endL == startL) return;

      ++count;
      mean += candidatesL[startL].distance;
      startL = endL;
    });

    std::sort(candidatesR.begin(), candidatesR.end(),
              [](const match_type& fml, const match_type& fmr) { return fml.matchIdx < fmr.matchIdx; });

    relationsR.resize(dscR.size() + 1);
    relationsR[0] = 0;
    auto diter = candidatesR.begin();
    auto dend = diter;
    auto dbeg = diter;
    int idx = 0;

    for_each(candidatesR.begin(), candidatesR.end(), [&](const match_type& fm) {
      if (idx != fm.matchIdx) {
        if (diter != dend) {
          // sort by distance
          std::sort(diter, dend);
          relationsR[idx + 1] = dend - dbeg;
          mean += diter->distance;
          diter = dend;
          ++idx;
          ++count;
        }
        for (; idx < fm.matchIdx; ++idx) relationsR[idx + 1] = relationsR[idx];
      }
      ++dend;
    });

    if (idx != dscR.size()) {
      if (diter != dend) {
        // sort by distance
        std::sort(diter, dend);
        relationsR[idx + 1] = dend - dbeg;
        mean += diter->distance;
        ++idx;
        ++count;
      }
      for (; idx < dscR.size(); ++idx) relationsR[idx + 1] = relationsR[idx];
    }

    mean /= count;
    mean *= 2;

    matches.reserve(candidatesL.size() / 10);
    startL = 0;
    for_each(relationsL.begin() + 1, relationsL.end(), [&](size_t endL) {
      size_t sizeL = endL - startL;
      if (sizeL == 0) return;
      const match_type& fmL = candidatesL[startL];

      size_t startR = relationsR[fmL.matchIdx];

      // if (sizeR == 0)
      //     return;

      const match_type& fmR = candidatesR[startR];

      FT llow = left[fmL.queryIdx].length(), lhigh = right[fmL.matchIdx].length();
      if (llow > lhigh) std::swap(llow, lhigh);

      // L-R-Check
      if (fmR.queryIdx == fmL.queryIdx) {
        /*
        if (lsim > 0.05) {
            if (sizeR == 1 && sizeL == 1)
                matches.push_back(fmL);
            else if ((candidatesR[startR+1].distance / fmR.distance) > 2 && (candidatesL[startL+1].distance /
        fmL.distance) > 2) matches.push_back(fmL);
        }
        */
        matches.push_back(fmL);
      }
      startL = endL;
    });
  }

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
};

}  // namespace lsfm
