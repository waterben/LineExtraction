//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file MotionPointMatcher.hpp
/// @brief Motion-based matcher for temporal point correspondence.
/// Combines point filtering and matching for motion estimation.

#pragma once

#include <lfd/FeatureMatcher.hpp>
#include <lfd/LRDescriptor.hpp>
#include <lfd/MotionPointFilter.hpp>

namespace lsfm {


/// @brief Motion-based point matcher combining filtering and descriptor matching.
/// Uses spatial filtering, brute force matching, and left-right consistency
/// check for temporal point correspondence.
/// @tparam FT Float type for computations
/// @tparam GV Geometric vector type (e.g., std::vector<cv::KeyPoint>)
/// @tparam descriptor_type Descriptor type (default: FdMat<FT>)
template <class FT, class GV, class descriptor_type = lsfm::FdMat<FT>>
class MotionPointMatcher : public OptionManager {
 public:
  typedef FT float_type;                                         ///< Float type used
  typedef GV geometric_vector;                                   ///< Geometric vector type
  typedef typename geometric_vector::value_type geometric_type;  ///< Geometric element type
  typedef std::vector<descriptor_type> descriptor_vector;        ///< Descriptor vector type
  typedef DescriptorMatch<FT> match_type;                        ///< Match result type
  typedef std::vector<match_type> match_vector;                  ///< Match result vector type


 private:
  MotionLineFilter<FT, GV> mlf;                       ///< Motion line filter for candidates
  FmBruteForce<FT, descriptor_type, match_type> bfm;  ///< Brute force matcher
  descriptor_vector dscLeft_;                         ///< New frame descriptors
  descriptor_vector dscRight_;                        ///< Previous frame descriptors
  std::vector<size_t> mLeft_;                         ///< New frame match counts
  std::vector<size_t> mRight_;                        ///< Previous frame match counts
  FT distTh_;                                         ///< Distance threshold

 public:
  /// @brief Construct a motion point matcher.
  /// @param width Image width
  /// @param height Image height
  /// @param angleTh Angle threshold in degrees
  /// @param distTh Distance threshold (0 = auto)
  /// @param r Radius for radius matching
  /// @param kk Number of nearest neighbors
  MotionPointMatcher(int width = 0, int height = 0, FT angleTh = 5, FT distTh = 1000, FT r = 0, int kk = 0)
      : mlf(distTh, angleTh, width, height), bfm(r, kk), distTh_(distTh) {
    /*
    if (!creatorLPtr.empty())
        creatorL = dynamic_cast<descriptor_creator*>(creatorLPtr.operator->()); // make it compatible with cv2 and 3
    if (!creatorRPtr.empty())
        creatorR = dynamic_cast<descriptor_creator*>(creatorRPtr.operator->());
    */
    CV_Assert(distTh_ >= 0);
    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("distTh", distTh, type, "Distance threshold (0 = auto)."));
  }

  /*
          template <class GV3>
          void match(const geometric_vector& newLines, const GV3& models, const cv::Mat * projMat, const
     descriptor_vector& dscL, const descriptor_vector& dscR, match_vector &matches) { match_vector candidates;
              geometric_vector projections;
              mlf.create(newLines, models, projMat, candidates, projections);
              final(newLines, previousLines, dscL, dscR, candidates, matches);
          }
          */
  /// @brief Match with pre-computed candidates and descriptors.
  /// @param newGeometry Current frame features
  /// @param previousGeometry Previous frame features
  /// @param candidates Pre-computed candidate matches
  /// @param dscL Current frame descriptors
  /// @param dscR Previous frame descriptors
  /// @param matches Output verified matches
  void match(const geometric_vector& newGeometry,
             const geometric_vector& previousGeometry,
             match_vector candidates,
             const descriptor_vector& dscL,
             const descriptor_vector& dscR,
             match_vector& matches) {
    //            match_vector candidates;
    //            mlf.create(newLines, previousLines, avgMotion, candidates);
    final(newGeometry, previousGeometry, dscL, dscR, candidates, matches);
  }

  /// @brief Match with automatic filtering using movement estimation.
  /// Creates candidates via MotionPointFilter with average movement,
  /// then performs descriptor matching.
  /// @tparam MPF Motion point filter type
  /// @param newGeometry Current frame features
  /// @param previousGeometry Previous frame features
  /// @param dscL Current frame descriptors
  /// @param dscR Previous frame descriptors
  /// @param avgMovement Average 2D movement estimate (dx, dy)
  /// @param width Image width
  /// @param height Image height
  /// @param matches Output verified matches
  template <class MPF = lsfm::MotionPointFilter<FT, std::vector<cv::KeyPoint>>>
  void match(const geometric_vector& newGeometry,
             const geometric_vector& previousGeometry,
             const descriptor_vector& dscL,
             const descriptor_vector& dscR,
             const std::pair<FT, FT>& avgMovement,
             const int width,
             const int height,
             match_vector& matches) {
    MPF mpf(width, height);
    std::vector<size_t> mLeft_, mRight_;
    std::vector<DescriptorMatch<FT>> candidates;
    mpf.create(newGeometry, previousGeometry, avgMovement, candidates, mLeft_, mRight_);

    match(newGeometry, previousGeometry, candidates, dscL, dscR, matches);
  }

  /*
          void match(const geometric_vector& newGeomerty, const geometric_vector& previousGeometry, std::pair<FT, FT>
  avgMotion, match_vector &matches) {
  //            CV_Assert(creatorL != 0 && creatorR != 0);
              if(newGeomerty.size() <= 0 || previousGeometry.size() <= 0)
                  return;
              match_vector candidates;
              mlf.create(newGeomerty, previousGeometry, avgMotion, candidates, mLeft_, mRight_);
  //            creatorL->createList(newLines, mLeft_,dscLeft_);
  //            creatorR->createList(previousLines, mRight_, dscRight_);
              final(newGeomerty, previousGeometry, dscLeft_, dscRight_, candidates, matches);
          }
  */
  /// @brief Get the motion line filter.
  /// @return Reference to the internal filter
  MotionLineFilter<FT, GV>& getFilter() { return mlf; }

  /// @brief Get the brute force matcher.
  /// @return Reference to the internal matcher
  FmBruteForce<FT, descriptor_type, std::vector<match_vector>>& getMatcher() { return bfm; }

  /// @brief Get current frame descriptors.
  /// @return Const reference to new frame descriptors
  const descriptor_vector& getDscNew() const { return dscLeft_; }

  /// @brief Get previous frame descriptors.
  /// @return Const reference to previous frame descriptors
  const descriptor_vector& getDscPrev() const { return dscRight_; }


 protected:
  /// @brief Perform final matching with left-right consistency check.
  /// Uses statistical thresholding (mean distance) for validation.
  /// @param newLines Current frame features
  /// @param previousLines Previous frame features
  /// @param dscN Current frame descriptors
  /// @param dscP Previous frame descriptors
  /// @param candidates Pre-filtered candidate matches
  /// @param matches Output verified matches
  void final(const geometric_vector& newLines,
             const geometric_vector& previousLines,
             const descriptor_vector& dscN,
             const descriptor_vector& dscP,
             const match_vector& candidates,
             match_vector& matches) {
    // query Index -> newLines; match Index -> previousLines

    bfm.trainMatches(dscN, dscP, candidates);

    const match_vector& candidatesNew = bfm.graph().distances;
    const std::vector<size_t>& relationsNew = bfm.graph().relations;

    match_vector candidatesPrev = candidatesNew;
    std::vector<size_t> relationsPrev;

    // sort by matchIdx
    std::sort(candidatesPrev.begin(), candidatesPrev.end(),
              [](const match_type& fmn, const match_type& fmp) { return fmn.matchIdx < fmp.matchIdx; });

    int count = 0;
    relationsPrev.resize(previousLines.size() + 1);
    relationsPrev[0] = 0;
    auto diter = candidatesPrev.begin();
    auto dend = diter;
    auto dbeg = diter;
    int idx = 0;
    FT mean = 0;
    FT valSum = 0, valSumSq = 0;

    // sort each block of candidates by distance
    for_each(candidatesPrev.begin(), candidatesPrev.end(), [&](const match_type& fm) {
      if (idx != fm.matchIdx) {
        if (diter != dend) {
          // sort by distance
          std::sort(diter, dend);
          relationsPrev[idx + 1] = dend - dbeg;
          valSum += diter->distance;
          valSumSq += diter->distance * diter->distance;
          diter = dend;
          ++idx;
          ++count;
        }
        for (; idx < fm.matchIdx; ++idx) relationsPrev[idx + 1] = relationsPrev[idx];
      }
      ++dend;
    });


    if (idx != previousLines.size()) {
      if (diter != dend) {
        // sort by distance
        std::sort(diter, dend);
        relationsPrev[idx + 1] = dend - dbeg;
        valSum += diter->distance;
        valSumSq += diter->distance * diter->distance;
        ++idx;
        ++count;
      }
      for (; idx < previousLines.size(); ++idx) relationsPrev[idx + 1] = relationsPrev[idx];
    }

    if (count > 0) mean = valSum / count;
    //            mean = mean * 1;
    FT variance = (count * valSumSq - (valSum * valSum)) / (count * (count - 1));  // approx
                                                                                   /*
                                                                                               FT var = 0;
                                                                                               for (int idx = 0; idx < previousLines.size(); ++idx)
                                                                                                   var += (candidatesPrev.at(relationsPrev[idx]).distance - mean) *
                                                                                      (candidatesPrev.at(relationsPrev[idx]).distance - mean);                                                                                var /= count;                                                                                std::cout << "variance: " << variance << "
                                                                                      var: " << var << std::endl;
                                                                                   */
    FT stdDeviation = std::sqrt(variance);
    //            std::cout << "stdDeviation: " << stdDeviation << "  mean: " << mean << std::endl;

    matches.reserve(candidatesNew.size() / 10);
    size_t startN = 0;
    for_each(relationsNew.begin() + 1, relationsNew.end(), [&](size_t endN) {
      size_t sizeN = endN - startN;
      if (sizeN == 0) return;

      const match_type& fmN = candidatesNew[startN];

      size_t startP = relationsPrev[fmN.matchIdx];
      size_t sizeP = relationsPrev[fmN.matchIdx + 1] - startP;

      if (sizeP == 0) return;

      const match_type& fmP = candidatesPrev[startP];
      /*                FT llow = newLines[fmN.queryIdx].length(), lhigh = previousLines[fmN.matchIdx].length();
                      if (llow > lhigh)
                          std::swap(llow, lhigh);
                      FT lsim = llow / lhigh;
      */
      // L-R-Check    TODO: Check if mean is a good value
      if (fmP.queryIdx == fmN.queryIdx && fmP.distance <= mean) {
        matches.push_back(fmN);
      }

      /*
      if (fmP.queryIdx == fmN.queryIdx && (fmN.distance < mean || true)) {
          if (lsim > 0.75) {
              if (sizeP == 1 && sizeN == 1)
                  matches.push_back(fmN);
              else if ((candidatesPrev[startP+1].distance / fmP.distance) > 5 && (candidatesNew[startN+1].distance /
      fmN.distance) > 5) matches.push_back(fmN);
          }
      }
      */

      startN = endN;
    });
  }

  /// @brief Legacy matching with stricter length similarity check.
  /// @param newLines Current frame features
  /// @param previousLines Previous frame features
  /// @param dscN Current frame descriptors
  /// @param dscP Previous frame descriptors
  /// @param candidates Pre-filtered candidate matches
  /// @param matches Output verified matches
  /// @deprecated Use final() instead
  void oldFinal(const geometric_vector& newLines,
                const geometric_vector& previousLines,
                const descriptor_vector& dscN,
                const descriptor_vector& dscP,
                const match_vector& candidates,
                match_vector& matches) {
    bfm.trainMatches(dscN, dscP, candidates);

    const match_vector& candidatesNew = bfm.graph().distances;
    const std::vector<size_t>& relationsNew = bfm.graph().relations;

    match_vector candidatesPrev = candidatesNew;
    std::vector<size_t> relationsPrev;

    size_t count = 0;
    size_t startN = 0;
    FT mean = 0;
    for_each(relationsNew.begin() + 1, relationsNew.end(), [&](size_t endN) {
      if (endN == startN) return;

      ++count;
      mean += candidatesNew[startN].distance;
      if (isnan(mean)) std::cout << "Nan problem!" << std::endl;
      startN = endN;
    });

    std::sort(candidatesPrev.begin(), candidatesPrev.end(),
              [](const match_type& fmn, const match_type& fmp) { return fmn.matchIdx < fmp.matchIdx; });

    relationsPrev.resize(dscP.size() + 1);
    relationsPrev[0] = 0;
    auto diter = candidatesPrev.begin();
    auto dend = diter;
    auto dbeg = diter;
    int idx = 0;

    for_each(candidatesPrev.begin(), candidatesPrev.end(), [&](const match_type& fm) {
      if (idx != fm.matchIdx) {
        if (diter != dend) {
          // sort by distance
          std::sort(diter, dend);
          relationsPrev[idx + 1] = dend - dbeg;
          mean += diter->distance;
          diter = dend;
          ++idx;
          ++count;
        }
        for (; idx < fm.matchIdx; ++idx) relationsPrev[idx + 1] = relationsPrev[idx];
      }
      ++dend;
    });

    if (idx != dscP.size()) {
      if (diter != dend) {
        // sort by distance
        std::sort(diter, dend);
        relationsPrev[idx + 1] = dend - dbeg;
        mean += diter->distance;
        ++idx;
        ++count;
      }
      for (; idx < dscP.size(); ++idx) relationsPrev[idx + 1] = relationsPrev[idx];
    }

    mean /= count;
    mean *= 2;

    matches.reserve(candidatesNew.size() / 10);
    startN = 0;
    for_each(relationsNew.begin() + 1, relationsNew.end(), [&](size_t endN) {
      size_t sizeN = endN - startN;
      if (sizeN == 0) return;

      const match_type& fmN = candidatesNew[startN];

      size_t startP = relationsPrev[fmN.matchIdx];
      size_t sizeP = relationsPrev[fmN.matchIdx + 1] - startP;

      if (sizeP == 0) return;

      const match_type& fmP = candidatesPrev[startP];
      FT llow = newLines[fmN.queryIdx].length(), lhigh = previousLines[fmN.matchIdx].length();
      if (llow > lhigh) std::swap(llow, lhigh);
      FT lsim = llow / lhigh;

      //                if(fmN.distance < mean)
      //                    matches.push_back(fmN);

      // L-R-Check
      if (fmP.queryIdx == fmN.queryIdx && fmN.distance < mean) {
        if (lsim > 0.75) {
          if (sizeP == 1 && sizeN == 1)
            matches.push_back(fmN);
          else if ((candidatesPrev[startP + 1].distance / fmP.distance) > 5 &&
                   (candidatesNew[startN + 1].distance / fmN.distance) > 5)
            matches.push_back(fmN);
        }
      }

      startN = endN;
    });
    //            for_each(candidatesNext.begin(), candidatesNext.end(), [&matches](match_type m){
    //                matches.push_back(m);
    //            });
  }

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
};

}  // namespace lsfm
