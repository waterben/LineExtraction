//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file FeatureMatcher.hpp
/// @brief Brute force matcher for feature descriptors.
/// Provides template-based matching with distance computation, k-NN search, and radius search capabilities.

#pragma once

#include <lfd/FeatureFilter.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <algorithm>

namespace lsfm {


/// @brief Simple brute force descriptor matcher.
/// Computes distances between all query and candidate descriptors with support
/// for k-NN, radius search, and mask-based filtering.
/// @tparam FT Floating-point type used for distance computations.
/// @tparam DT Descriptor type supporting a `distance()` method.
/// @tparam DM Match type storing query index, match index, and distance (defaults to DescriptorMatch<FT>).
template <class FT, class DT, class DM = DescriptorMatch<FT>>
class FmBruteForce : public OptionManager {
 public:
  typedef FT float_type;                         ///< Floating-point type for distances.
  typedef DT descriptor_type;                    ///< Descriptor type.
  typedef DM match_type;                         ///< Match result type.
  typedef std::vector<match_type> match_vector;  ///< Vector of match results.


  /// @brief Distance graph storing match results between query and candidate descriptors.
  struct Graph {
    /// @brief Construct a new Graph.
    /// @param r Maximum distance radius (0 means unlimited).
    /// @param kk Number of nearest neighbors (0 means unlimited).
    Graph(float_type r = 0, int kk = 0) : distances(), relations(), k(kk), radius(r) {}

    match_vector distances;         ///< Sorted match distances for all query descriptors.
    std::vector<size_t> relations;  ///< Cumulative index boundaries per query descriptor.
    int k;                          ///< Maximum number of nearest neighbors stored per query.
    float_type radius;              ///< Overall maximum distance across all stored matches.
  };

  /// @brief Construct a brute force matcher.
  /// @param radius Maximum distance between descriptors (0 means no limit).
  /// @param k Number of best matches to keep per query descriptor (0 means keep all).
  FmBruteForce(float_type radius = 0, int k = 0) : graph_(), k_(k), radius_(radius) {
    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("k", k, "int", "Number of nearest neighbors."));
    this->options_.push_back(OptionManager::OptionEntry("radius", radius, type, "Max distance between descriptors."));
  }

  /// @brief Train the matcher by computing a distance graph between query and match descriptors.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  template <template <class, class...> class DV, class... DVArgs>
  void train(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc) {
    auto ms = mDsc.size();
    graph_.distances.resize(qDsc.size() * ms);
    graph_.relations.resize(qDsc.size());
    graph_.relations.resize(qDsc.size() + 1);
    graph_.relations[0] = 0;
    // store max dist
    float_type mdist = 0;
    // loop query descriptors
    auto end = qDsc.end();
    auto iter = qDsc.begin();
    auto diter = graph_.distances.begin();
    auto dbeg = diter;
    for (size_t i = 0; iter != end; ++i, ++iter) {
      // loop match descriptors
      auto miter = mDsc.begin();
      for (size_t j = 0; j != ms; ++j, ++miter)
        diter[static_cast<std::ptrdiff_t>(j)] =
            match_type(static_cast<int>(i), static_cast<int>(j), iter->distance(*miter));
      auto dend = diter + static_cast<std::ptrdiff_t>(ms);
      // sort by distance
      std::sort(diter, dend);
      // check if we have set k nearest neighbors, if so, resize to k
      if (k_ > 0 && k_ < static_cast<int>(ms)) dend = diter + static_cast<std::ptrdiff_t>(k_);
      // check if we have set a serach radius, if so, sort out distances > radius
      if (radius_ > 0 && ms > 0 && (dend - 1)->distance > radius_)
        dend = std::upper_bound(diter, dend, match_type(0, 0, radius_));
      // get max distance in this set
      if (dend != diter) mdist = std::max(mdist, (dend - 1)->distance);
      graph_.relations[i + 1] = static_cast<size_t>(dend - dbeg);
      diter = dend;
    }

    graph_.distances.resize(static_cast<size_t>(diter - dbeg));
    // max num of nn for every match vector
    graph_.k = k_ > 0 ? k_ : static_cast<int>(ms);
    // overall max dist
    graph_.radius = mdist;
  }

  /// @brief Train the matcher with inverted masks.
  /// Inverted masks: non-zero values are included, zero values are masked out.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Mask vector type (indexable, non-zero means included).
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param qMask Inverted mask for query descriptors.
  /// @param mMask Inverted mask for match descriptors.
  template <template <class, class...> class DV, class... DVArgs, class MV>
  void train(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, const MV& qMask, const MV& mMask) {
    auto ms = mDsc.size();
    graph_.distances.resize(qDsc.size() * ms);
    graph_.relations.resize(qDsc.size() + 1);
    graph_.relations[0] = 0;
    // store max dist
    float_type mdist = 0;
    // loop query descriptors
    auto end = qDsc.end();
    auto iter = qDsc.begin();
    auto diter = graph_.distances.begin();
    auto dbeg = diter;
    for (int i = 0; iter != end; ++i, ++iter) {
      if (!qMask[i]) {
        graph_.relations[i + 1] = graph_.relations[i];
        continue;
      }
      // loop match descriptors
      auto miter = mDsc.begin();
      auto dend = diter;
      for (int j = 0; j != ms; ++j, ++miter)
        if (mMask[j]) *(dend++) = match_type(i, j, iter->distance(*miter));
      // sort by distance
      std::sort(diter, dend);
      // check if we have set k nearest neighbors, if so, resize to k
      if (k_ > 0 && k_ < static_cast<int>(dend - diter)) dend = diter + (static_cast<size_t>(k_));
      // check if we have set a serach radius, if so, sort out distances > radius
      if (radius_ > 0 && dend != diter && (dend - 1)->distance > radius_)
        dend = std::upper_bound(diter, dend, match_type(0, 0, radius_));
      // get max distance in this set
      if (dend != diter) mdist = std::max(mdist, (dend - 1)->distance);
      graph_.relations[i + 1] = dend - dbeg;
      diter = dend;
    }

    graph_.distances.resize(static_cast<size_t>(diter - dbeg));
    // max num of nn for every match vector
    graph_.k = k_ > 0 ? k_ : static_cast<int>(ms);
    // overall max dist
    graph_.radius = mdist;
  }

  /// @brief Train the matcher and apply a feature filter to exclude specific pairs.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param ff Feature filter that returns true for pairs to exclude.
  template <template <class, class...> class DV, class... DVArgs>
  void train(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, const FeatureFilter<FT>& ff) {
    auto ms = mDsc.size();
    graph_.distances.resize(qDsc.size() * ms);
    graph_.relations.resize(qDsc.size() + 1);
    graph_.relations[0] = 0;
    // store max dist
    float_type mdist = 0;
    // loop query descriptors
    auto end = qDsc.end();
    auto iter = qDsc.begin();
    auto diter = graph_.distances.begin();
    auto dbeg = diter;
    for (int i = 0; iter != end; ++i, ++iter) {
      // loop match descriptors
      auto miter = mDsc.begin();
      auto dend = diter;
      for (int j = 0; j != ms; ++j, ++miter) {
        if (ff.filter(i, j)) continue;
        *(dend++) = match_type(i, j, iter->distance(*miter));
      }
      // sort by distance
      std::sort(diter, dend);
      // check if we have set k nearest neighbors, if so, resize to k
      if (k_ > 0 && k_ < static_cast<int>(dend - diter)) dend = diter + (static_cast<size_t>(k_));
      // check if we have set a serach radius, if so, sort out distances > radius
      if (radius_ > 0 && dend != diter && (dend - 1)->distance > radius_)
        dend = std::upper_bound(diter, dend, match_type(0, 0, radius_));
      // get max distance in this set
      if (dend != diter) mdist = std::max(mdist, (dend - 1)->distance);
      graph_.relations[i + 1] = dend - dbeg;
      diter = dend;
    }

    graph_.distances.resize(static_cast<size_t>(diter - dbeg));
    // max num of nn for every match vector
    graph_.k = k_ > 0 ? k_ : static_cast<int>(ms);
    // overall max dist
    graph_.radius = mdist;
  }


  /// @brief Train the matcher with inverted masks and a feature filter.
  /// Inverted masks: non-zero values are included, zero values are masked out.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Mask vector type (indexable, non-zero means included).
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param qMask Inverted mask for query descriptors.
  /// @param mMask Inverted mask for match descriptors.
  /// @param ff Feature filter that returns true for pairs to exclude.
  template <template <class, class...> class DV, class... DVArgs, class MV>
  void train(const DV<DT, DVArgs...>& qDsc,
             const DV<DT, DVArgs...>& mDsc,
             const MV& qMask,
             const MV& mMask,
             const FeatureFilter<FT>& ff) {
    auto ms = mDsc.size();
    graph_.distances.resize(qDsc.size() * ms);
    graph_.relations.resize(qDsc.size() + 1);
    graph_.relations[0] = 0;
    // store max dist
    float_type mdist = 0;
    // loop query descriptors
    auto end = qDsc.end();
    auto iter = qDsc.begin();
    auto diter = graph_.distances.begin();
    auto dbeg = diter;
    for (int i = 0; iter != end; ++i, ++iter) {
      if (!qMask[i]) {
        graph_.relations[i + 1] = graph_.relations[i];
        continue;
      }
      // loop match descriptors
      auto miter = mDsc.begin();
      auto dend = diter;
      for (int j = 0; j != ms; ++j, ++miter) {
        if (!mMask[j] || ff.filter(i, j)) continue;
        *(dend++) = match_type(i, j, iter->distance(*miter));
      }
      // sort by distance
      std::sort(diter, dend);
      // check if we have set k nearest neighbors, if so, resize to k
      if (k_ > 0 && k_ < static_cast<int>(dend - diter)) dend = diter + (static_cast<size_t>(k_));
      // check if we have set a serach radius, if so, sort out distances > radius
      if (radius_ > 0 && dend != diter && (dend - 1)->distance > radius_)
        dend = std::upper_bound(diter, dend, match_type(0, 0, radius_));
      // get max distance in this set
      if (dend != diter) mdist = std::max(mdist, (dend - 1)->distance);
      graph_.relations[i + 1] = dend - dbeg;
      diter = dend;
    }

    graph_.distances.resize(static_cast<size_t>(diter - dbeg));
    // max num of nn for every match vector
    graph_.k = k_ > 0 ? k_ : static_cast<int>(ms);
    // overall max dist
    graph_.radius = mdist;
  }


  /// @brief Train the matcher using a pre-existing match structure.
  /// Optionally sorts matches by query index before training.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Match vector type (must be sortable and indexable).
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param matches Pre-existing match structure (will be sorted if needed).
  /// @param isSorted Whether matches are already sorted by query index.
  template <template <class, class...> class DV, class... DVArgs, class MV>
  void trainMatches(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, MV& matches, bool isSorted) {
    if (!isSorted) {
      std::sort(matches.begin(), matches.end(),
                [](decltype(matches[0])& fml, decltype(matches[0])& fmr) { return fml.queryIdx < fmr.queryIdx; });
    }
    trainMatches(qDsc, mDsc, matches);
  }

  /// @brief Train the matcher using a pre-existing sorted match structure.
  /// Matches must be pre-sorted by query index in ascending order.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Match vector type (must be indexable, sorted by queryIdx).
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param matches Pre-existing match structure sorted by query index.
  template <template <class, class...> class DV, class... DVArgs, class MV>
  void trainMatches(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, const MV& matches) {
    graph_.distances.resize(matches.size());
    graph_.relations.resize(qDsc.size() + 1);
    graph_.relations[0] = 0;
    // store max dist
    float_type mdist = 0;
    auto diter = graph_.distances.begin();
    auto dend = diter;
    auto dbeg = diter;
    size_t idx = 0;

    for_each(matches.begin(), matches.end(), [&](decltype(matches[0])& fm) {
      const size_t query_idx = static_cast<size_t>(fm.queryIdx);
      if (idx != query_idx) {
        if (diter != dend) {
          // sort by distance
          std::sort(diter, dend);
          // check if we have set k nearest neighbors, if so, resize to k
          if (k_ > 0 && k_ < static_cast<int>(dend - diter)) dend = diter + (static_cast<size_t>(k_));
          // check if we have set a serach radius, if so, sort out distances > radius
          if (radius_ > 0 && dend != diter && (dend - 1)->distance > radius_)
            dend = std::upper_bound(diter, dend, match_type(0, 0, radius_));
          // get max distance in this set
          if (dend != diter) mdist = std::max(mdist, (dend - 1)->distance);
          graph_.relations[idx + 1] = static_cast<size_t>(dend - dbeg);
          diter = dend;
          ++idx;
        }
        for (; idx < query_idx; ++idx) graph_.relations[idx + 1] = graph_.relations[idx];
      }
      if (fm.filterState == FS_MASKED) return;
      *dend = match_type(fm.queryIdx, fm.matchIdx,
                         qDsc[static_cast<size_t>(fm.queryIdx)].distance(mDsc[static_cast<size_t>(fm.matchIdx)]));
      ++dend;
    });

    if (idx != qDsc.size()) {
      if (diter != dend) {
        // sort by distance
        std::sort(diter, dend);
        // check if we have set k nearest neighbors, if so, resize to k
        if (k_ > 0 && k_ < static_cast<int>(dend - diter)) dend = diter + (static_cast<size_t>(k_));
        // check if we have set a serach radius, if so, sort out distances > radius
        if (radius_ > 0 && dend != diter && (dend - 1)->distance > radius_)
          dend = std::upper_bound(diter, dend, match_type(0, 0, radius_));
        // get max distance in this set
        if (dend != diter) mdist = std::max(mdist, (dend - 1)->distance);
        graph_.relations[idx + 1] = static_cast<size_t>(dend - dbeg);
        diter = dend;
        ++idx;
      }
      for (; idx < qDsc.size(); ++idx) graph_.relations[idx + 1] = graph_.relations[idx];
    }

    graph_.distances.resize(static_cast<size_t>(diter - dbeg));
    // max num of nn for every match vector
    graph_.k = k_ > 0 ? k_ : static_cast<int>(mDsc.size());
    // overall max dist
    graph_.radius = mdist;
  }


  /// @brief Train and return all matches between query and candidate descriptors.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @return Reference to the internal match vector.
  template <template <class, class...> class DV, class... DVArgs>
  const match_vector& match(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc) {
    train(qDsc, mDsc);
    return graph_.distances;
  }

  /// @brief Train and return matches with a feature filter applied.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param ff Feature filter that returns true for pairs to exclude.
  /// @return Reference to the internal match vector.
  template <template <class, class...> class DV, class... DVArgs>
  const match_vector& match(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, const FeatureFilter<FT>& ff) {
    train(qDsc, mDsc, ff);
    return graph_.distances;
  }

  /// @brief Train and return matches with inverted masks applied.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Mask vector type (indexable, non-zero means included).
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param lm Inverted mask for query (left) descriptors.
  /// @param rm Inverted mask for match (right) descriptors.
  /// @return Reference to the internal match vector.
  template <template <class, class...> class DV, class... DVArgs, class MV>
  const match_vector& match(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, const MV& lm, const MV& rm) {
    train(qDsc, mDsc, lm, rm);
    return graph_.distances;
  }

  /// @brief Train and return matches with inverted masks and a feature filter.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Mask vector type (indexable, non-zero means included).
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param lm Inverted mask for query (left) descriptors.
  /// @param rm Inverted mask for match (right) descriptors.
  /// @param ff Feature filter that returns true for pairs to exclude.
  /// @return Reference to the internal match vector.
  template <template <class, class...> class DV, class... DVArgs, class MV>
  const match_vector& match(const DV<DT, DVArgs...>& qDsc,
                            const DV<DT, DVArgs...>& mDsc,
                            const MV& lm,
                            const MV& rm,
                            const FeatureFilter<FT>& ff) {
    train(qDsc, mDsc, lm, rm, ff);
    return graph_.distances;
  }

  /// @brief Train using a pre-existing match list and return the distance results.
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Match vector type.
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param mv Pre-existing match list.
  /// @return Reference to the internal match vector.
  template <template <class, class...> class DV, class... DVArgs, class MV>
  const match_vector& matchList(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, const MV& mv) {
    train(qDsc, mDsc, mv);
    return graph_.distances;
  }

  /// @brief Find the single best match for each query descriptor (static version).
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param[out] matches Output vector filled with the best match per query.
  template <template <class, class...> class DV, class... DVArgs>
  static void match(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, match_vector& matches) {
    auto ms = mDsc.size();
    matches.clear();
    matches.reserve(qDsc.size());
    match_type mt;
    float_type dst = 0;

    // loop query descriptors
    auto end = qDsc.end();
    auto iter = qDsc.begin();
    for (int i = 0; iter != end; ++i, ++iter) {
      // loop match descriptors
      auto miter = mDsc.begin();
      mt = match_type(i, 0, iter->distance(*miter));
      ++miter;
      for (int j = 1; j != ms; ++j, ++miter) {
        dst = iter->distance(*miter);
        // std::cout << i << ", " << j << ": " << dst << std::endl;
        // std::cout << " (" << iter->data[0] << "," << iter->data[1] << "," << iter->data[2] << "," << iter->data[3] <<
        // "," << iter->data[4] << "," << iter->data[5] << "," << iter->data[6] << "," << iter->data[7] << "), " << " ("
        // << miter->data[0] << "," << miter->data[1] << "," << miter->data[2] << "," << miter->data[3] << "," <<
        // miter->data[4] << "," << miter->data[5] << "," << miter->data[6] << "," << miter->data[7] << ")" <<
        // std::endl;
        if (dst < mt.distance) mt = match_type(i, j, dst);
      }
      matches.push_back(mt);
    }
  }

  /// @brief Find the single best match per query descriptor with a feature filter (static version).
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param ff Feature filter that returns true for pairs to exclude.
  /// @param[out] matches Output vector filled with the best match per query.
  template <template <class, class...> class DV, class... DVArgs>
  static void match(const DV<DT, DVArgs...>& qDsc,
                    const DV<DT, DVArgs...>& mDsc,
                    const FeatureFilter<FT>& ff,
                    match_vector& matches) {
    auto ms = mDsc.size();
    matches.clear();
    matches.reserve(qDsc.size());
    match_type mt;
    float_type dst = 0;

    // loop query descriptors
    auto end = qDsc.end();
    auto iter = qDsc.begin();
    for (int i = 0; iter != end; ++i, ++iter) {
      // loop match descriptors
      auto miter = mDsc.begin();
      mt = match_type();
      for (int j = 0; j != ms; ++j, ++miter) {
        if (ff.filter(i, j)) continue;
        dst = iter->distance(*miter);
        if (dst < mt.distance) mt = match_type(i, j, dst);
      }
      if (mt.matchIdx >= 0) matches.push_back(mt);
    }
  }

  /// @brief Find the single best match per query descriptor with inverted masks (static version).
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Mask vector type (indexable, non-zero means included).
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param qMask Inverted mask for query descriptors.
  /// @param mMask Inverted mask for match descriptors.
  /// @param[out] matches Output vector filled with the best match per query.
  template <template <class, class...> class DV, class... DVArgs, class MV>
  static void match(const DV<DT, DVArgs...>& qDsc,
                    const DV<DT, DVArgs...>& mDsc,
                    const MV& qMask,
                    const MV& mMask,
                    match_vector& matches) {
    auto ms = mDsc.size();
    matches.clear();
    matches.reserve(qDsc.size());
    match_type mt;
    float_type dst = 0;

    // loop query descriptors
    auto end = qDsc.end();
    auto iter = qDsc.begin();
    for (int i = 0; iter != end; ++i, ++iter) {
      if (!qMask[i]) continue;
      // loop match descriptors
      auto miter = mDsc.begin();
      mt = match_type();
      for (int j = 0; j != ms; ++j, ++miter) {
        if (!mMask[j]) continue;
        dst = iter->distance(*miter);
        if (dst < mt.distance) mt = match_type(i, j, dst);
      }
      if (mt.matchIdx >= 0) matches.push_back(mt);
    }
  }

  /// @brief Find the single best match per query descriptor with masks and a filter (static version).
  /// @tparam DV Container template for descriptor vectors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Mask vector type (indexable, non-zero means included).
  /// @param qDsc Query descriptors.
  /// @param mDsc Match (candidate) descriptors.
  /// @param qMask Inverted mask for query descriptors.
  /// @param mMask Inverted mask for match descriptors.
  /// @param ff Feature filter that returns true for pairs to exclude.
  /// @param[out] matches Output vector filled with the best match per query.
  template <template <class, class...> class DV, class... DVArgs, class MV>
  static void match(const DV<DT, DVArgs...>& qDsc,
                    const DV<DT, DVArgs...>& mDsc,
                    const MV& qMask,
                    const MV& mMask,
                    const FeatureFilter<FT>& ff,
                    match_vector& matches) {
    auto ms = mDsc.size();
    matches.clear();
    matches.reserve(qDsc.size());
    match_type mt;
    float_type dst = 0;

    // loop query descriptors
    auto end = qDsc.end();
    auto iter = qDsc.begin();
    for (int i = 0; iter != end; ++i, ++iter) {
      if (!qMask[i]) continue;
      // loop match descriptors
      auto miter = mDsc.begin();
      mt = match_type();
      for (int j = 0; j != ms; ++j, ++miter) {
        if (!mMask[j] || ff.filter(i, j)) continue;
        dst = iter->distance(*miter);
        if (dst < mt.distance) mt = match_type(i, j, dst);
      }
      if (mt.matchIdx >= 0) matches.push_back(mt);
    }
  }


  /// @brief Get the distance graph containing all computed match distances.
  /// @return Const reference to the internal distance graph.
  const Graph& graph() const { return graph_; }


  /// @brief Get the k best matches for each query descriptor in increasing distance order.
  /// @param[out] matches Output vector filled with the k-nearest matches per query.
  /// @param k Number of nearest neighbors to retrieve (must be > 0).
  void knn(match_vector& matches, int k) {
    CV_Assert(k > 0);
    if (k >= graph_.k) {
      matches = graph_.distances;
      return;
    }

    auto iter = graph_.distances.cbegin();
    matches.reserve(graph_.distances.size());
    size_t last = 0;
    for (size_t i = 1; i != graph_.relations.size(); ++i) {
      int size = static_cast<int>(graph_.relations[i] - last);
      last = graph_.relations[i];
      int j = 0, jn = size > k ? k : size;
      for (; j < jn; ++j, ++iter) matches.push_back(*iter);
      iter += size - jn;
    }
  }

  /// @brief Find matches within a maximum distance radius for each query descriptor.
  /// Results are returned in increasing order of distance.
  /// @param[out] matches Output vector filled with matches within the radius.
  /// @param radius Maximum distance threshold (must be > 0).
  void radius(match_vector& matches, float_type radius) {
    CV_Assert(radius > 0);

    if (radius >= graph_.radius) {
      matches = graph_.distances;
      return;
    }

    match_type mt;
    mt.distance = radius;

    auto iter = graph_.distances.cbegin();
    matches.reserve(graph_.distances.size());
    size_t last = 0;
    for (size_t i = 1; i != graph_.relations.size(); ++i) {
      int size = static_cast<int>(graph_.relations[i] - last);
      if (size == 0) continue;
      last = graph_.relations[i];
      matches.insert(matches.end(), iter, std::upper_bound(iter, iter + size, mt));
      iter += size;
    }
  }

  /// @brief Find the single best (closest) match for each query descriptor.
  /// @param[out] matches Output vector filled with the best match per query.
  void best(match_vector& matches) {
    auto iter = graph_.distances.cbegin();
    matches.reserve(graph_.relations.size());
    size_t last = 0;
    for (size_t i = 1; i != graph_.relations.size(); ++i) {
      int size = static_cast<int>(graph_.relations[i] - last);
      if (size == 0) continue;
      last = graph_.relations[i];
      matches.push_back(*iter);
      iter += size;
    }
  }

 protected:
  /// @brief Apply an option value by name.
  /// @param name Option name ("k" or "radius").
  /// @param value New value for the option.
  void setOptionImpl(const std::string& name, double value) {
    if (name == "k") {
      if (value >= 0 && value <= std::numeric_limits<int>::max()) {
        k_ = static_cast<int>(value);
        this->options_[0].value = k_;
      }
    } else if (name == "radius") {
      if (value >= 0 && value <= std::numeric_limits<float_type>::max()) {
        radius_ = static_cast<float_type>(value);
        this->options_[1].value = radius_;
      }
    }
  }

 private:
  Graph graph_;        ///< Internal distance graph storing match results.
  int k_;              ///< Number of nearest neighbors to keep (0 = all).
  float_type radius_;  ///< Maximum match distance threshold (0 = no limit).
};

}  // namespace lsfm
