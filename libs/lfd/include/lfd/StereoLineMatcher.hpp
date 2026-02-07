//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file StereoLineMatcher.hpp
/// @brief Stereo line matcher with geometric filtering.
/// Combines stereo filtering and matching for stereo reconstruction.

#pragma once

#include <lfd/FeatureMatcher.hpp>
#include <lfd/LRDescriptor.hpp>
#include <lfd/StereoLineFilter.hpp>

namespace lsfm {


/// @brief Stereo line matcher combining geometric filtering and descriptor matching.
/// Uses StereoLineFilter for candidate generation and FmBruteForce for
/// descriptor-based matching with left-right consistency check.
/// @tparam FT Float type for computations
/// @tparam GV Geometric vector type (default: std::vector<LineSegment<FT>>)
/// @tparam DC_Helper Descriptor computation helper (default: GchGradImgInterpolate<FT>)
template <class FT, class GV = std::vector<LineSegment<FT>>, class DC_Helper = GchGradImgInterpolate<FT>>
class StereoLineMatcher : public OptionManager {
  // Deleted copy operations due to pointer members
  StereoLineMatcher(const StereoLineMatcher&) = delete;
  StereoLineMatcher& operator=(const StereoLineMatcher&) = delete;

 public:
  typedef FT float_type;                                                   ///< Float type used
  typedef GV geometric_vector;                                             ///< Geometric vector type
  typedef typename geometric_vector::value_type geometric_type;            ///< Geometric element type
  typedef FdcGenericLR<FT, geometric_type, DC_Helper> descriptor_creator;  ///< Descriptor creator type
  typedef typename descriptor_creator::descriptor_type descriptor_type;    ///< Descriptor type
  typedef std::vector<descriptor_type> descriptor_vector;                  ///< Descriptor vector type
  typedef DescriptorMatch<FT> match_type;                                  ///< Match result type
  typedef std::vector<match_type> match_vector;                            ///< Match result vector type


 private:
  StereoLineFilter<FT, GV> slf;                       ///< Stereo line filter for candidates
  FmBruteForce<FT, descriptor_type, match_type> bfm;  ///< Brute force matcher

  descriptor_creator *creatorL, *creatorR;                       ///< Raw descriptor creator pointers
  typename descriptor_creator::FdcPtr creatorLPtr, creatorRPtr;  ///< Owned descriptor creator pointers

  descriptor_vector dscLeft_;   ///< Left descriptors
  descriptor_vector dscRight_;  ///< Right descriptors
  std::vector<size_t> mLeft_;   ///< Left match counts
  std::vector<size_t> mRight_;  ///< Right match counts

  FT distTh_;  ///< Distance threshold (0 = auto)

 public:
  typedef typename descriptor_creator::FdcPtr FdcPtr;  ///< Descriptor creator pointer type

  /// @brief Construct with reference descriptor creators.
  /// @param cL Left descriptor creator (not owned)
  /// @param cR Right descriptor creator (not owned)
  /// @param height Image height
  /// @param maxDist Maximum line distance in pixels
  /// @param angleTh Maximum angle difference between matching lines
  /// @param minYOverlap Minimum Y overlap ratio (0-1)
  /// @param distTh Descriptor distance threshold (0 = auto)
  /// @param r Radius for radius matching
  /// @param kk Number of nearest neighbors
  StereoLineMatcher(descriptor_creator& cL,
                    descriptor_creator& cR,
                    int height,
                    FT maxDist = 10000,
                    FT angleTh = 5,
                    FT minYOverlap = 0.5,
                    FT distTh = 0,
                    FT r = 0,
                    int kk = 0)
      : slf(height, maxDist, angleTh, minYOverlap),
        bfm(r, kk),
        creatorL(&cL),
        creatorR(&cR),
        dscLeft_(),
        dscRight_(),
        mLeft_(),
        mRight_(),
        distTh_(distTh) {
    CV_Assert(distTh_ >= 0);
    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("distTh", distTh, type, "Distance threshold (0 = auto)."));
  }

  /// @brief Construct with shared pointer descriptor creators.
  /// @param cL Left descriptor creator (shared ownership)
  /// @param cR Right descriptor creator (shared ownership)
  /// @param height Image height (default: 0)
  /// @param maxDist Maximum line distance in pixels (default: 10000)
  /// @param angleTh Maximum angle difference (default: 5)
  /// @param minYOverlap Minimum Y overlap ratio (default: 0.5)
  /// @param distTh Descriptor distance threshold (default: 0)
  /// @param r Radius for radius matching (default: 0)
  /// @param kk Number of nearest neighbors (default: 0)
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
        creatorL(nullptr),
        creatorR(nullptr),
        creatorLPtr(cL),
        creatorRPtr(cR),
        dscLeft_(),
        dscRight_(),
        mLeft_(),
        mRight_(),
        distTh_(distTh) {
    if (!creatorLPtr.empty()) creatorL = dynamic_cast<descriptor_creator*>(creatorLPtr.operator->());
    if (!creatorRPtr.empty()) creatorR = dynamic_cast<descriptor_creator*>(creatorRPtr.operator->());

    CV_Assert(distTh_ >= 0);
    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("distTh", distTh, type, "Distance threshold (0 = auto)."));
  }

  /// @brief Match with pre-computed descriptors.
  /// @param left Left line segments
  /// @param right Right line segments
  /// @param dscL Left descriptors
  /// @param dscR Right descriptors
  /// @param matches Output verified matches
  void match(const geometric_vector& left,
             const geometric_vector& right,
             const descriptor_vector& dscL,
             const descriptor_vector& dscR,
             match_vector& matches) {
    match_vector candidates;
    slf.create(left, right, candidates);
    final(left, right, dscL, dscR, candidates, matches);
  }

  /// @brief Match with automatic descriptor creation.
  /// Creates candidates, computes descriptors, and performs matching.
  /// @param left Left line segments
  /// @param right Right line segments
  /// @param matches Output verified matches
  void match(const geometric_vector& left, const geometric_vector& right, match_vector& matches) {
    CV_Assert(creatorL != nullptr && creatorR != nullptr);
    match_vector candidates;
    slf.create(left, right, candidates, mLeft_, mRight_);
    creatorL->createList(left, mLeft_, dscLeft_);
    creatorR->createList(right, mRight_, dscRight_);
    final(left, right, dscLeft_, dscRight_, candidates, matches);
  }

  /// @brief Get the stereo line filter.
  /// @return Reference to the internal stereo line filter
  StereoLineFilter<FT, GV>& getFilter() { return slf; }

  /// @brief Get the brute force matcher.
  /// @return Reference to the internal brute force matcher
  FmBruteForce<FT, descriptor_type, std::vector<match_vector>>& getMatcher() { return bfm; }

  /// @brief Get left descriptors.
  /// @return Copy of the left descriptor vector
  descriptor_vector getDescriptorLeft() { return dscLeft_; }

  /// @brief Get right descriptors.
  /// @return Copy of the right descriptor vector
  descriptor_vector getDescriptorRight() { return dscRight_; }


 protected:
  /// @brief Perform final matching with left-right consistency check.
  /// Builds distance graph from both directions and applies L-R check.
  /// @param left Left line segments
  /// @param right Right line segments
  /// @param dscL Left descriptors
  /// @param dscR Right descriptors
  /// @param candidates Pre-filtered candidate matches
  /// @param matches Output verified matches
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
    size_t idx = 0;

    for_each(candidatesR.begin(), candidatesR.end(), [&](const match_type& fm) {
      const size_t match_idx = static_cast<size_t>(fm.matchIdx);
      if (idx != match_idx) {
        if (diter != dend) {
          // sort by distance
          std::sort(diter, dend);
          relationsR[idx + 1] = static_cast<size_t>(dend - dbeg);
          mean += diter->distance;
          diter = dend;
          ++idx;
          ++count;
        }
        for (; idx < match_idx; ++idx) relationsR[idx + 1] = relationsR[idx];
      }
      ++dend;
    });

    if (idx != dscR.size()) {
      if (diter != dend) {
        // sort by distance
        std::sort(diter, dend);
        relationsR[idx + 1] = static_cast<size_t>(dend - dbeg);
        mean += diter->distance;
        ++idx;
        ++count;
      }
      for (; idx < dscR.size(); ++idx) relationsR[idx + 1] = relationsR[idx];
    }

    mean /= static_cast<FT>(count);
    mean *= 2;

    matches.reserve(candidatesL.size() / 10);
    startL = 0;
    for_each(relationsL.begin() + 1, relationsL.end(), [&](size_t endL) {
      size_t sizeL = endL - startL;
      if (sizeL == 0) return;
      const match_type& fmL = candidatesL[startL];

      size_t startR = relationsR[static_cast<size_t>(fmL.matchIdx)];

      // if (sizeR == 0)
      //     return;

      const match_type& fmR = candidatesR[startR];

      FT llow = left[static_cast<size_t>(fmL.queryIdx)].length(),
         lhigh = right[static_cast<size_t>(fmL.matchIdx)].length();
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

  /// @brief Handle option value changes.
  /// @param name Option name
  /// @param value New option value
  void setOptionImpl(const std::string& name, FT value) {
    static_cast<void>(name);
    static_cast<void>(value);
    /*if (name == "k") {
        if (value >= 0 && value <= std::numeric_limits<int>::max()) {
            k_ =
    static_cast<int>(value);
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
