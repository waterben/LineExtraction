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

/// @file FeatureFilter.hpp
/// @brief Feature matching and filtering classes and structures.
/// Provides basic feature match structures and filtering infrastructure for matching pairs.

#pragma once

#include <algorithm>
#include <iterator>
#include <limits>

namespace lsfm {
// #define TRIM_2D_MATCHES

/// @brief Filter state enumeration for feature matches
enum {
  FS_NONE = 0,  ///< No filter applied
  FS_MASKED
};  ///< Match has been filtered out (masked)

/// @brief Basic feature match structure template.
/// @tparam FT Float type for distance calculations
template <class FT>
struct FeatureMatch {
  /// @brief Construct a feature match.
  /// @param qi Query descriptor index (default: -1)
  /// @param mi Matched descriptor index (default: -1)
  /// @param fs Filter state enumeration (default: FS_NONE)
  FeatureMatch(int qi = -1, int mi = -1, int fs = FS_NONE) : queryIdx(qi), matchIdx(mi), filterState(fs) {}

  int queryIdx;     ///< Query descriptor index
  int matchIdx;     ///< Matched descriptor index
  int filterState;  ///< Filter state (FS_NONE or FS_MASKED)
};

/// @brief Descriptor match structure with distance metric.
/// @tparam FT Float type for distance calculations
template <class FT>
struct DescriptorMatch : public FeatureMatch<FT> {
  /// @brief Construct a descriptor match with distance.
  /// @param qi Query descriptor index (default: -1)
  /// @param mi Matched descriptor index (default: -1)
  /// @param fs Filter state enumeration (default: FS_NONE)
  /// @param dst Distance between descriptors (default: max value)
  DescriptorMatch(int qi = -1, int mi = -1, int fs = FS_NONE, FT dst = std::numeric_limits<FT>::max())
      : FeatureMatch<FT>(qi, mi, fs), distance(dst) {}

  /// @brief Construct a descriptor match with indices and distance.
  /// @param qi Query descriptor index
  /// @param mi Matched descriptor index
  /// @param dst Distance between descriptors
  DescriptorMatch(int qi, int mi, FT dst) : FeatureMatch<FT>(qi, mi, FS_NONE), distance(dst) {}

  FT distance;  ///< Distance between query descriptor and matched descriptor

  /// @brief Compare two descriptor matches by distance (ascending order).
  /// @param m The other descriptor match
  /// @return True if this match's distance is less than m's distance
  inline bool operator<(const DescriptorMatch<FT>& m) const { return distance < m.distance; }

  /// @brief Compare descriptor match distance with a threshold.
  /// @param f The distance threshold
  /// @return True if this match's distance is less than the threshold
  inline bool operator<(const FT& f) const { return distance < f; }
};


/// @brief Base class for feature match filtering.
/// Provides interface for filtering matches and support for mask-based operations.
/// Can be applied during matching to filter candidates while building match vectors.
/// @tparam FT Float type for distance calculations
template <class FT>
class FeatureFilter {
 public:
  typedef FT float_type;  ///< Float type alias

  virtual ~FeatureFilter() {}

  /// @brief Check if a feature match should be filtered out.
  /// @param lfIdx Left feature index
  /// @param rfIdx Right feature index
  /// @return True if the match should be filtered out, false otherwise
  virtual bool filter(int lfIdx, int rfIdx) const = 0;

  /// @brief Apply filter to a single match and update its filter state.
  /// Sets filter state to FS_MASKED if the match should be filtered.
  /// @param fm Reference to the feature match to filter
  inline void filter(FeatureMatch<FT>& fm) const {
    if (fm.filterState == FS_MASKED || this->filter(fm.queryIdx, fm.matchIdx)) fm.filterState = FS_MASKED;
  }

  /// @brief Apply filter to a vector or list of matches.
  /// Updates filter state for all matches in the container.
  /// @tparam FMV Feature match vector type
  /// @param matches Reference to the matches container
  template <class FMV>
  void filterList(FMV& matches) const {
    for_each(matches.begin(), matches.end(), [this](FeatureMatch<FT>& fm) { this->filter(fm); });
  }

  /// @brief Filter matches and store unfiltered results in output container.
  /// Preserves ordering and skips filtered (masked) matches.
  /// @tparam FMV Feature match vector type
  /// @param matches Input matches vector
  /// @param out Output container for unfiltered matches
  template <class FMV>
  void filterList(FMV& matches, FMV& out) const {
    out.clear();
    out.reserve(matches.size());
    for_each(matches.begin(), matches.end(), [this, &out](FeatureMatch<FT>& fm) {
      if (!this->filter(fm.matchIdx, fm.queryIdx)) out.push_back(fm);
    });
  }

  /// @brief Create a match set by applying filter.
  /// Generates all pairs (lIdx, rIdx) for 0 <= lIdx < lSize, 0 <= rIdx < rSize
  /// where filter(lIdx, rIdx) returns false.
  /// @tparam FMV Feature match vector type
  /// @param lSize Size of left feature set
  /// @param rSize Size of right feature set
  /// @param matches Output container for generated matches
  template <class FMV>
  void create(size_t lSize, size_t rSize, FMV& matches) const {
    createDetail(lSize, rSize, matches,
                 static_cast<typename std::iterator_traits<typename FMV::iterator>::iterator_category*>(0));
  }

  /// @brief Create match set with mask vectors.
  /// Generates matches and inverted mask vectors (!=0 means not masked, ==0 means masked).
  /// Counts number of non-filtered matches for each feature.
  /// @tparam FMV Feature match vector type
  /// @tparam MV Mask vector type
  /// @param lSize Size of left feature set
  /// @param rSize Size of right feature set
  /// @param matches Output container for generated matches
  /// @param lm Output left mask vector (1 per valid match for this left feature, 0 otherwise)
  /// @param rm Output right mask vector (1 per valid match for this right feature, 0 otherwise)
  template <class FMV, class MV>
  void create(size_t lSize, size_t rSize, FMV& matches, MV& lm, MV& rm) const {
    createDetail(lSize, rSize, matches, lm, rm,
                 static_cast<typename std::iterator_traits<typename FMV::iterator>::iterator_category*>(0));
  }


  /// @brief Generate mask vectors indicating valid matches.
  /// Creates inverted masks (!=0 means not masked, ==0 means masked).
  /// Counts number of non-filtered matches for each feature.
  /// @tparam MV Mask vector type
  /// @param lSize Size of left feature set
  /// @param rSize Size of right feature set
  /// @param lm Output left mask vector
  /// @param rm Output right mask vector
  template <class MV>
  inline void mask(size_t lSize, size_t rSize, MV& lm, MV& rm) const {
    lm.resize(lSize, 0);
    rm.resize(rSize, 0);
    auto lmIter = lm.begin();
    for (size_t i = 0; i != lSize; ++i, ++lmIter) {
      auto rmIter = rm.begin();
      for (size_t j = 0; j != rSize; ++j, ++rmIter) {
        if (!this->filter(static_cast<int>(i), static_cast<int>(j))) {
          ++(*lmIter);
          ++(*rmIter);
        }
      }
    }
  }

 private:
  /// @brief Helper for creating match set with forward iterators.
  /// @tparam FMV Feature match vector type
  template <class FMV>
  inline void createDetail(size_t lSize, size_t rSize, FMV& matches, std::forward_iterator_tag*) const {
    matches.clear();
    for (size_t i = 0; i != lSize; ++i) {
      for (size_t j = 0; j != rSize; ++j) {
        if (!this->filter(i, j)) matches.push_back(typename FMV::value_type(i, j));
      }
    }
  }

  /// @brief Helper for creating match set with random access iterators.
  /// Reserves space to improve performance.
  /// @tparam FMV Feature match vector type
  template <class FMV>
  inline void createDetail(size_t lSize, size_t rSize, FMV& matches, std::random_access_iterator_tag*) const {
    matches.clear();
    matches.reserve(lSize * rSize / 10);
    for (size_t i = 0; i != lSize; ++i) {
      for (size_t j = 0; j != rSize; ++j) {
        if (!this->filter(static_cast<int>(i), static_cast<int>(j)))
          matches.push_back(typename FMV::value_type(static_cast<int>(i), static_cast<int>(j)));
      }
    }
  }

  /// @brief Helper for creating match set with masks and forward iterators.
  /// @tparam FMV Feature match vector type
  /// @tparam MV Mask vector type
  template <class FMV, class MV>
  inline void createDetail(size_t lSize, size_t rSize, FMV& matches, MV& lm, MV& rm, std::forward_iterator_tag*) const {
    lm.resize(lSize, 0);
    rm.resize(rSize, 0);
    matches.clear();
    auto lmIter = lm.begin();
    for (size_t i = 0; i != lSize; ++i, ++lmIter) {
      auto rmIter = rm.begin();
      for (size_t j = 0; j != rSize; ++j, ++rmIter) {
        if (!this->filter(i, j)) {
          matches.push_back(typename FMV::value_type(i, j));
          ++(*lmIter);
          ++(*rmIter);
        }
      }
    }
  }

  /// @brief Helper for creating match set with masks and random access iterators.
  /// Reserves space to improve performance.
  /// @tparam FMV Feature match vector type
  /// @tparam MV Mask vector type
  template <class FMV, class MV>
  inline void createDetail(
      size_t lSize, size_t rSize, FMV& matches, MV& lm, MV& rm, std::random_access_iterator_tag*) const {
    lm.resize(lSize, 0);
    rm.resize(rSize, 0);
    matches.clear();
    matches.reserve(rSize * lSize / 10);
    auto lmIter = lm.begin();
    for (size_t i = 0; i != lSize; ++i, ++lmIter) {
      auto rmIter = rm.begin();
      for (size_t j = 0; j != rSize; ++j, ++rmIter) {
        if (!this->filter(static_cast<int>(i), static_cast<int>(j))) {
          matches.push_back(typename FMV::value_type(static_cast<int>(i), static_cast<int>(j)));
          ++(*lmIter);
          ++(*rmIter);
        }
      }
    }
  }
};

}  // namespace lsfm
