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


#ifndef _LFD_FEATUREFILTER_HPP_
#define _LFD_FEATUREFILTER_HPP_
#ifdef __cplusplus

#include <iterator>

namespace lsfm {
//#define TRIM_2D_MATCHES

    enum {
        FS_NONE = 0,
        FS_MASKED
    };

    template<class FT>
    struct FeatureMatch
    {
        FeatureMatch(int qi = -1, int mi = -1, int fs = FS_NONE) : queryIdx(qi), matchIdx(mi), filterState(fs) {}

        int queryIdx; // query descriptor index
        int matchIdx; // match descriptor index

        int filterState; // indicate filter state
    };

    template<class FT>
    struct DescriptorMatch : public FeatureMatch<FT>
    {
        DescriptorMatch(int qi = -1, int mi = -1, int fs = FS_NONE, FT dst = std::numeric_limits<FT>::max()) : FeatureMatch<FT>(qi, mi, fs), distance(dst) {}
        DescriptorMatch(int qi, int mi, FT dst) : FeatureMatch<FT>(qi, mi, FS_NONE), distance(dst) {}
        
        FT distance;  // distance between query descriptor and matched descriptor

        // less is better
        inline bool operator<(const DescriptorMatch<FT> &m) const {
            return distance < m.distance;
        }

        inline bool operator<(const FT &f) const {
            return distance < f;
        }

    };

    

    //! feature filter base class to enable filtering on matches
    //! can also be applied to matcher, to filter out matches while creating the vectors
    template<class FT>
    class FeatureFilter {
    public:
        typedef FT float_type;
        virtual ~FeatureFilter() {}

        //! check if feature match has to be filtered
        virtual bool filter(int lfIdx, int rfIdx) const = 0;

        //! check if feature match has to be filtered (sets filter state to FS_MASKED)
        inline void filter(FeatureMatch<FT>& fm) const {
            if (fm.filterState == FS_MASKED || this->filter(fm.queryIdx, fm.matchIdx))
                fm.filterState = FS_MASKED;
        }

        //! apply filter to vector or list of matches
        template<class FMV>
        void filterList(FMV& matches) const {
            for_each(matches.begin(), matches.end(), [this](FeatureMatch<FT>& fm){
                this->filter(fm);
            });
        }

        //! apply filter to vector or list of matches
        template<class FMV>
        void filterList(FMV& matches, FMV& out) const {
            out.clear();
            out.reserve(matches.size());
            for_each(matches.begin(), matches.end(), [this, &out](FeatureMatch<FT>& fm){
                if (!this->filter(fm.matchIdx, fm.queryIdx))
                    out.push_back(fm);
            });
        }

        //! create match set by filter
        template<class FMV>
        void create(size_t lSize, size_t rSize, FMV& matches) const {
            createDetail(lSize, rSize, matches, (typename std::iterator_traits<typename FMV::iterator>::iterator_category*)0);
        }

        //! create match set by filter and mask vectors (inverted masks, != 0 is not masked, == 0 is masked, count the number of non filtered)
        template<class FMV, class MV>
        void create(size_t lSize, size_t rSize, FMV& matches, MV& lm, MV& rm) const {
            createDetail(lSize, rSize, matches, lm, rm, (typename std::iterator_traits<typename FMV::iterator>::iterator_category*)0);
        }


        //! create mask vectors (inverted masks, != 0 is not masked, == 0 is masked, count the number of non filtered)
        template<class MV>
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
        template<class FMV>
        inline void createDetail(size_t lSize, size_t rSize, FMV& matches, std::forward_iterator_tag*) const {
            matches.clear();
            for (size_t i = 0; i != lSize; ++i) {
                for (size_t j = 0; j != rSize; ++j) {
                    if (!this->filter(i, j))
                        matches.push_back(typename FMV::value_type(i, j));
                }
            }
        }

        template<class FMV>
        inline void createDetail(size_t lSize, size_t rSize, FMV& matches, std::random_access_iterator_tag*) const {
            matches.clear();
            matches.reserve(lSize*rSize / 10);
            for (size_t i = 0; i != lSize; ++i) {
                for (size_t j = 0; j != rSize; ++j) {
                    if (!this->filter(i, j))
                        matches.push_back(typename FMV::value_type(i, j));
                }
            }
        }

        template<class FMV, class MV>
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

        template<class FMV, class MV>
        inline void createDetail(size_t lSize, size_t rSize, FMV& matches, MV& lm, MV& rm, std::random_access_iterator_tag*) const {
            lm.resize(lSize, 0);
            rm.resize(rSize, 0);
            matches.clear();
            matches.reserve(rSize*lSize / 10);
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

}
#endif
#endif
