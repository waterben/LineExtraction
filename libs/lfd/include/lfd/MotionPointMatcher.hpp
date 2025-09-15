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


#ifndef _LFD_MOTIONPOINTMATCHER_HPP_
#define _LFD_MOTIONPOINTMATCHER_HPP_
#ifdef __cplusplus

#include <lfd/MotionPointFilter.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <lfd/LRDescriptor.hpp>

namespace lsfm {

    
    //! motion Point Matcher
    template<class FT, class GV, class descriptor_type = lsfm::FdMat<FT> >
    class MotionPointMatcher : public OptionManager
    {


    public:
        typedef FT float_type;

        typedef GV geometric_vector;
        typedef typename geometric_vector::value_type geometric_type;

//        typedef DC descriptor_creator;
//        typedef typename descriptor_creator::descriptor_type descriptor_type;
        typedef std::vector<descriptor_type> descriptor_vector;

        typedef DescriptorMatch<FT> match_type;
        typedef std::vector<match_type> match_vector;


    private:
        MotionLineFilter<FT,GV> mlf;
        FmBruteForce<FT,descriptor_type,match_type> bfm;

//        descriptor_creator *creatorL, *creatorR;
//        typename descriptor_creator::FdcPtr creatorLPtr, creatorRPtr;

        descriptor_vector dscLeft_, dscRight_;
        std::vector<size_t> mLeft_, mRight_;

        FT distTh_;

    public:

//        typedef typename descriptor_creator::FdcPtr FdcPtr;

/*
        MotionPointMatcher(descriptor_creator& cL, descriptor_creator& cR, int width = 0, int height = 0, FT angleTh = 5, FT distTh = 1000, FT r = 0, int kk = 0) :
            mlf(distTh, angleTh, width, height), bfm(r,kk), creatorL(&cL), creatorR(&cR), distTh_(distTh)  {
            CV_Assert(distTh_ >= 0);
            std::string type = (sizeof(float_type) > 4 ? "double" : "float");
            this->options_.push_back(OptionManager::OptionEntry("distTh", distTh, type, "Distance threshold (0 = auto)."));
        }
*/
        MotionPointMatcher(int width = 0, int height = 0, FT angleTh = 5, FT distTh = 1000, FT r = 0, int kk = 0) :
            mlf(distTh, angleTh, width, height), bfm(r,kk), distTh_(distTh)  {
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
        void match(const geometric_vector& newLines, const GV3& models, const cv::Mat * projMat, const descriptor_vector& dscL, const descriptor_vector& dscR, match_vector &matches) {
            match_vector candidates;
            geometric_vector projections;
            mlf.create(newLines, models, projMat, candidates, projections);
            final(newLines, previousLines, dscL, dscR, candidates, matches);
        }
        */
        void match(const geometric_vector& newGeometry, const geometric_vector& previousGeometry, match_vector candidates, const descriptor_vector& dscL, const descriptor_vector& dscR, match_vector &matches) {
//            match_vector candidates;
//            mlf.create(newLines, previousLines, avgMotion, candidates);
            final(newGeometry, previousGeometry, dscL, dscR, candidates, matches);
        }

        template<class MPF = lsfm::MotionPointFilter<FT, std::vector<cv::KeyPoint>> >   // TODO: Add predicions by projection
        void match(const geometric_vector& newGeometry, const geometric_vector& previousGeometry, const descriptor_vector& dscL, const descriptor_vector& dscR,  const std::pair<FT, FT> &avgMovement, const int width, const int height, match_vector &matches) {
            MPF mpf( width, height );
            std::vector<size_t> mLeft_, mRight_;
            std::vector<DescriptorMatch<FT>> candidates;
            mpf.create(newGeometry, previousGeometry, avgMovement, candidates, mLeft_, mRight_);

            match(newGeometry, previousGeometry, candidates,dscL, dscR,  matches);
        }

/*
        void match(const geometric_vector& newGeomerty, const geometric_vector& previousGeometry, std::pair<FT, FT> avgMotion, match_vector &matches) {
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
        MotionLineFilter<FT,GV>& getFilter() {
            return mlf;
        }


        FmBruteForce<FT,descriptor_type,std::vector<match_vector>>& getMatcher() {
            return bfm;
        }

        const descriptor_vector & getDscNew() const{
            return dscLeft_;
        }

        const descriptor_vector & getDscPrev() const{
            return dscRight_;
        }



    protected:


        void final(const geometric_vector& newLines, const geometric_vector& previousLines, const descriptor_vector& dscN, const descriptor_vector& dscP, const match_vector& candidates, match_vector &matches) {
            // query Index -> newLines; match Index -> previousLines

            bfm.trainMatches(dscN,dscP,candidates);

            const match_vector& candidatesNew = bfm.graph().distances;
            const std::vector<size_t>& relationsNew = bfm.graph().relations;

            match_vector candidatesPrev = candidatesNew;
            std::vector<size_t> relationsPrev;

            // sort by matchIdx
            std::sort(candidatesPrev.begin(), candidatesPrev.end(),[](const match_type& fmn, const match_type& fmp){
               return fmn.matchIdx < fmp.matchIdx;
            });

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
                    for (; idx < fm.matchIdx; ++idx)
                        relationsPrev[idx + 1] = relationsPrev[idx];
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
                for (; idx < previousLines.size(); ++idx)
                    relationsPrev[idx + 1] = relationsPrev[idx];
            }

            if(count > 0)
              mean = valSum / count;
//            mean = mean * 1;
            FT variance = ( count * valSumSq - (valSum * valSum) ) / ( count * (count - 1 ));   // approx
/*
            FT var = 0;
            for (int idx = 0; idx < previousLines.size(); ++idx)
                var += (candidatesPrev.at(relationsPrev[idx]).distance - mean) * (candidatesPrev.at(relationsPrev[idx]).distance - mean);
            var /= count;
            std::cout << "variance: " << variance << "  var: " << var << std::endl;
*/
            FT stdDeviation = std::sqrt(variance);
//            std::cout << "stdDeviation: " << stdDeviation << "  mean: " << mean << std::endl;

            matches.reserve(candidatesNew.size() / 10);
            size_t startN = 0;
            for_each(relationsNew.begin()+1, relationsNew.end(), [&](size_t endN) {
                size_t sizeN = endN - startN;
                if (sizeN == 0)
                    return;

                const match_type& fmN = candidatesNew[startN];

                size_t startP = relationsPrev[fmN.matchIdx];
                size_t sizeP = relationsPrev[fmN.matchIdx + 1] - startP;

                if (sizeP == 0)
                    return;

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
                        else if ((candidatesPrev[startP+1].distance / fmP.distance) > 5 && (candidatesNew[startN+1].distance / fmN.distance) > 5)
                            matches.push_back(fmN);
                    }
                }
                */

                startN = endN;
            });




        }

        void oldFinal(const geometric_vector& newLines, const geometric_vector& previousLines, const descriptor_vector& dscN, const descriptor_vector& dscP, const match_vector& candidates, match_vector &matches) {
            bfm.trainMatches(dscN,dscP,candidates);

            const match_vector& candidatesNew = bfm.graph().distances;
            const std::vector<size_t>& relationsNew = bfm.graph().relations;

            match_vector candidatesPrev = candidatesNew;
            std::vector<size_t> relationsPrev;

            size_t count = 0;
            size_t startN = 0;
            FT mean = 0;
            for_each(relationsNew.begin()+1, relationsNew.end(), [&](size_t endN) {
                if (endN == startN)
                    return;

                ++count;
                mean += candidatesNew[startN].distance;
                if(isnan(mean))
                    std::cout << "Nan problem!" << std::endl;
                startN = endN;
            });

            std::sort(candidatesPrev.begin(), candidatesPrev.end(),[](const match_type& fmn, const match_type& fmp){
               return fmn.matchIdx < fmp.matchIdx;
            });

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
                    for (; idx < fm.matchIdx; ++idx)
                        relationsPrev[idx + 1] = relationsPrev[idx];
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
                for (; idx < dscP.size(); ++idx)
                    relationsPrev[idx + 1] = relationsPrev[idx];
            }

            mean /= count;
            mean *= 2;

            matches.reserve(candidatesNew.size() / 10);
            startN = 0;
            for_each(relationsNew.begin()+1, relationsNew.end(), [&](size_t endN) {
                size_t sizeN = endN - startN;
                if (sizeN == 0)
                    return;

                const match_type& fmN = candidatesNew[startN];

                size_t startP = relationsPrev[fmN.matchIdx];
                size_t sizeP = relationsPrev[fmN.matchIdx + 1] - startP;

                if (sizeP == 0)
                    return;

                const match_type& fmP = candidatesPrev[startP];
                FT llow = newLines[fmN.queryIdx].length(), lhigh = previousLines[fmN.matchIdx].length();
                if (llow > lhigh)
                    std::swap(llow, lhigh);
                FT lsim = llow / lhigh;

//                if(fmN.distance < mean)
//                    matches.push_back(fmN);

                // L-R-Check
                if (fmP.queryIdx == fmN.queryIdx && fmN.distance < mean) {
                    if (lsim > 0.75) {
                        if (sizeP == 1 && sizeN == 1)
                            matches.push_back(fmN);
                        else if ((candidatesPrev[startP+1].distance / fmP.distance) > 5 && (candidatesNew[startN+1].distance / fmN.distance) > 5)
                            matches.push_back(fmN);
                    }
                }

                startN = endN;
            });
//            for_each(candidatesNext.begin(), candidatesNext.end(), [&matches](match_type m){
//                matches.push_back(m);
//            });
        }

        void setOptionImpl(const std::string &name, FT value) {
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

}
#endif
#endif
