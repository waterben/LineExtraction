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

#include <lfd/FeatureFilter.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <algorithm>

namespace lsfm {


//! simple brute force matcher -> compute distances between all candidates
template <class FT, class DT, class DM = DescriptorMatch<FT>>
class FmBruteForce : public OptionManager {
 public:
  typedef FT float_type;
  typedef DT descriptor_type;
  typedef DM match_type;
  typedef std::vector<match_type> match_vector;


  struct Graph {
    Graph(float_type r = 0, int kk = 0) : k(kk), radius(r) {}

    match_vector distances;
    std::vector<size_t> relations;
    int k;
    float_type radius;
  };

  //! init object with radius (max distance between descriptors) and k (k best matches
  //! for every query descriptor). If radius or k = 0 (default), all matches are stored
  FmBruteForce(float_type radius = 0, int k = 0) : k_(k), radius_(radius) {
    std::string type = (sizeof(float_type) > 4 ? "double" : "float");
    this->options_.push_back(OptionManager::OptionEntry("k", k, "int", "Number of nearest neighbors."));
    this->options_.push_back(OptionManager::OptionEntry("radius", radius, type, "Max distance between descriptors."));
  }

  //! train matcher (e.g. compute distance graph)
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

  //! train matcher (e.g. compute distance graph)
  //! //! using inverted masks (inverted masks, != 0 is not masked, == 0 is masked)
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

  //! train matcher and apply filter
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


  //! train matcher and apply filter
  //! using inverted masks (inverted masks, != 0 is not masked, == 0 is masked)
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


  //! train matcher (e.g. compute distance graph)
  //! using given match structure
  template <template <class, class...> class DV, class... DVArgs, class MV>
  void trainMatches(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, MV& matches, bool isSorted) {
    if (!isSorted) {
      std::sort(matches.begin(), matches.end(),
                [](decltype(matches[0])& fml, decltype(matches[0])& fmr) { return fml.queryIdx < fmr.queryIdx; });
    }
    trainMatches(qDsc, mDsc, matches);
  }

  //! train matcher (e.g. compute distance graph)
  //! using given match structure
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


  //! get matches
  template <template <class, class...> class DV, class... DVArgs>
  const match_vector& match(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc) {
    train(qDsc, mDsc);
    return graph_.distances;
  }

  template <template <class, class...> class DV, class... DVArgs>
  const match_vector& match(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, const FeatureFilter<FT>& ff) {
    train(qDsc, mDsc, ff);
    return graph_.distances;
  }

  template <template <class, class...> class DV, class... DVArgs, class MV>
  const match_vector& match(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, const MV& lm, const MV& rm) {
    train(qDsc, mDsc, lm, rm);
    return graph_.distances;
  }

  template <template <class, class...> class DV, class... DVArgs, class MV>
  const match_vector& match(const DV<DT, DVArgs...>& qDsc,
                            const DV<DT, DVArgs...>& mDsc,
                            const MV& lm,
                            const MV& rm,
                            const FeatureFilter<FT>& ff) {
    train(qDsc, mDsc, lm, rm, ff);
    return graph_.distances;
  }

  template <template <class, class...> class DV, class... DVArgs, class MV>
  const match_vector& matchList(const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, const MV& mv) {
    train(qDsc, mDsc, mv);
    return graph_.distances;
  }

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


  //! get graph (all distances between all descriptors)
  const Graph& graph() const { return graph_; }


  //! get k best matches for each query descriptor (in increasing order of distances).
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

  //! Find best matches for each query descriptor which have distance less than
  //! maxDistance (in increasing order of distances).
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

  //! Find best match for each query descriptor
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
  Graph graph_;
  int k_;
  float_type radius_;
};

}  // namespace lsfm
