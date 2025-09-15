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


#ifndef _LFD_MOTIONPOINTFILTER_HPP_
#define _LFD_MOTIONPOINTFILTER_HPP_
#ifdef __cplusplus

#include <algorithm>
#include <array>
#include <geometry/point.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <utility/option_manager.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace lsfm {


    //! stereo Point filter -> uses binning and x-axis sorting + constrains to sort
    //! out bad Point match candidates
    template<class FT, class GV, class GT = typename GV::value_type, unsigned int bins = 12, class DM = DescriptorMatch<FT>>
    class MotionPointFilter : public FeatureFilter<FT> , public OptionManager
    {
        int height_, width_;

        struct PointData {
            PointData(const lsfm::Vec2<FT>& p = lsfm::Vec2<FT>(), FT s = 0) : position(p), scale(s) {}
            PointData(const cv::Point2f& p, FT s = 0) : position(lsfm::Vec2<FT>(p.x,p.y)), scale(s) {}
            
            lsfm::Vec2<FT> position;
            FT scale;
        };


        std::vector<PointData> newPoints_, oldPoints_;

        // first rotation (4 areas in coordsystem), then number of bins, then variable sized vectors with candidates
        std::array<std::array<std::vector<int>,bins>,bins> bins_;

    public:

        typedef FT float_type;
        typedef GV geometric_vector;
        typedef GT geometric_type;

/*
        struct Motion {
            Motion(const FT xDir = 0, const FT yDir = 0) : x(xDir), y(yDir) {}

            FT x, y;
        };
*/
        MotionPointFilter(int width = 1, int height = 1) :
            height_(height), width_(width) {
            CV_Assert(height_ >= 0 && width_ >= 0);

            std::string type = (sizeof(float_type) > 4 ? "double" : "float");
//            this->options_.push_back(OptionManager::OptionEntry("maxPixelDist", maxPixDist, type, "Maximal distance between two corresponding Points."));
        }

        void train(const GV& newPoints, const GV& oldPoints) {
            trainSide(newPoints,newPoints_);
            trainSide(oldPoints,oldPoints_);
            //motionEstimate_ = motionEstimate;

        }
        
        virtual bool filter(int nfIdx, int ofIdx) const {

            // both filters given by binning
//            return false;


            const PointData &nd = newPoints_[nfIdx];
            const PointData &od = oldPoints_[ofIdx];
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

            // skip Points with unsimilar orientations
//            FT adiff = abs(nd.angle - od.angle);
//            if (adiff > angleTh_)
//                return true;



            return false;
        }


        //! create match set by filter
        //! only for Points with existing Model, thus they can be projected
/*        template<class FMV, class GV3>
        void create(const GV& newPoints, const GV3& models, cv::Mat * projMat, FMV& matches) {

            matches.clear();
            matches.reserve(newPoints.size() * models.size() / 10);

            // calculate predicted positions
            //GV predictions;
            std::vector<LineSegment<FT>> predictions;

            predictions.reserve(models.size());
            for(int i = 0; i < models.size(); ++i){
                predictions.push_back(models.at(i).project2Line(projMat));
            }

            // clear bins
            for_each(bins_.begin(), bins_.end(), [&newPoints](std::array<std::array<std::vector<int>,bins>,bins> &bs) {
                for_each(bs.begin(), bs.end(), [&newPoints](std::array<std::vector<int>,bins> &b) {
                    for_each(b.begin(), b.end(), [&newPoints](std::vector<int> &v) {
                        v.clear();
                        v.reserve(newPoints.size() / bins);
                    });
                });
            });

            FT bstep_h = static_cast<FT>(height_) / bins;
            FT bstep_w = static_cast<FT>(width_) / bins;

            trainSide(newPoints, newPoints_);
            trainSideAndBins(predictions, oldPoints_, bstep_h, bstep_w);

            std::vector<char> oidxList;
            size_t nsize = newPoints_.size();
            oidxList.reserve(nsize);

            for(int nidx = 0; nidx < newPoints_.size(); ++nidx){
                const PointData ld = newPoints_[nidx];
                oidxList.assign(nsize,0);

                std::array<std::array<std::vector<int>,bins>,bins>& qbins = this->bins_[static_cast<int>(ld.angle / (360 / angleBins)) % angleBins];

                std::vector<std::pair<int, int>> pixel, pixelNeighbours;
                pixelOfLine((ld.beg.x() / width_ ) * static_cast<FT>(bins), (ld.beg.y() / height_ ) * static_cast<FT>(bins), (ld.end.x() / width_ ) * static_cast<FT>(bins), (ld.end.y() / height_ ) * static_cast<FT>(bins), pixel);
                int searchRange = 1;
                getNeighbouringBins<bins>(pixel, pixelNeighbours, searchRange);

                for_each(pixelNeighbours.begin(), pixelNeighbours.end(), [this, &qbins, &nidx, &matches, &oidxList](std::pair<int, int> &p){

                    std::vector<int> &bin = qbins.at(p.first).at(p.second);
                    for_each(bin.begin(), bin.end(), [this, &matches, &oidxList, &nidx](int oidx){

                        if (!oidxList[nidx] && !this->filter(nidx,oidx)) {
                            matches.push_back(typename FMV::value_type(nidx,oidx));
                         //   ++lm[lidx];
                         //   ++rm[ridx];
                            ++oidxList[nidx];
                        }

                    });
                });
            }

        }
 */

        //! track Points, only with average 2D Image-movement estimation
        template<class FMV, class MV>
        void create(const GV& newPoints, const GV& previousPoints, const std::pair<FT, FT> &avgMovement, FMV& matches, MV& nm, MV& pm) {

            matches.clear();
            matches.reserve(newPoints.size() * previousPoints.size() / 10);

            nm.resize(newPoints.size(), 0);
            pm.resize(previousPoints.size(), 0);

            // calculate predicted positions
            // GV predictions;
            std::vector<geometric_type> predictions;
            predictions.reserve(previousPoints.size());
            for(int i = 0; i < previousPoints.size(); ++i){
                geometric_type pt(previousPoints.at(i));
                pt.pt.x += avgMovement.first;
                pt.pt.y += avgMovement.second;

                //predictions.push_back(geometric_type(previousPoints.at(i).pt.x + avgMovement.first, previousPoints.at(i).pt.y + avgMovement.second));
                predictions.push_back(pt);
            }

            /*  Already done in trainSideAndBins
            // clear bins
            for_each(bins_.begin(), bins_.end(), [&newPoints](std::array<std::vector<int>,bins> &b) {
                for_each(b.begin(), b.end(), [&newPoints](std::vector<int> &v) {
                    v.clear();
                    v.reserve(newPoints.size() / bins);
                });
            });
*/
//            FT bstep_h = static_cast<FT>(height_) / bins;
//            FT bstep_w = static_cast<FT>(width_) / bins;

            trainSide(newPoints, newPoints_);
            trainSideAndBins(predictions, oldPoints_);

            std::vector<char> pIdxList;
            size_t nsize = newPoints_.size();
            size_t pSize = previousPoints.size();
            pIdxList.reserve(pSize);

            for(int nIdx = 0; nIdx < newPoints_.size(); ++nIdx){
                const PointData pd = newPoints_[nIdx];
                pIdxList.assign(pSize,0);

                std::vector<std::pair<int, int>> pixel, pixelNeighbours;
//                pixelOfLine((ld.beg.x() / width_ ) * static_cast<FT>(bins), (ld.beg.y() / height_ ) * static_cast<FT>(bins), (ld.end.x() / width_ ) * static_cast<FT>(bins), (ld.end.y() / height_ ) * static_cast<FT>(bins), pixel);
                pixel.push_back(std::pair<int, int>((pd.position.x() / width_ ) * static_cast<FT>(bins), (pd.position.y() / height_ ) * static_cast<FT>(bins)));
                int searchRange = 1;
                getNeighbouringBins<bins>(pixel, pixelNeighbours, searchRange);

                // loop through same and neighbouring angle-bins
//                int aBinIdx = static_cast<int>(ld.angle);// (360 / angleBins)) % angleBins;
//                for(int bidx = -1; bidx <= 1; ++bidx){

//                    int currABin = (aBinIdx + bidx);// % angleBins;
//                    currABin = (currABin < 0 ? (currABin + angleBins) : currABin);
                    std::array<std::array<std::vector<int>,bins>,bins>& aBin = this->bins_;

                    for_each(pixelNeighbours.begin(), pixelNeighbours.end(), [this, &aBin, &nIdx, &matches, &pIdxList, &nm, &pm](std::pair<int, int> &p){

                        std::vector<int> &bin = aBin.at(p.first).at(p.second);
                        for_each(bin.begin(), bin.end(), [this, &matches, &pIdxList, &nIdx, &nm, &pm](int pIdx){

                            if (!pIdxList[pIdx] && !this->filter(nIdx,pIdx)) {
                                matches.push_back(typename FMV::value_type(nIdx,pIdx));
                                ++nm[nIdx];
                                ++pm[pIdx];
                                ++pIdxList[pIdx];
                            }

                        });
                    });
//                }
            }
        }

    protected:

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
        inline void trainSide(const GV& points, std::vector<PointData> &data) {
            data.clear();
            data.reserve(points.size());
            for_each(points.begin(),points.end(),[&](const geometric_type& point){
                data.push_back(PointData(point.pt));
            });
        }

        inline void trainSideAndBins(const std::vector<geometric_type>& points, std::vector<PointData> &data) {
            // clear point data
            data.clear();
            data.reserve(points.size());
            // clear bins
            for_each(bins_.begin(), bins_.end(), [&points](std::array<std::vector<int>,bins> &b) {
                for_each(b.begin(), b.end(), [&points](std::vector<int> &v) {
                    v.clear();
                    v.reserve(points.size() / bins);
                });
            });

            int size = static_cast<int>(points.size());
            for (int idx = 0; idx != size; ++idx) {
                const geometric_type& point = points[idx];
                PointData pd(point.pt);
                data.push_back(pd);

                //std::array<std::array<std::vector<int>,bins>,bins>& qbins = this->bins_;

                int binX = point.pt.x / width_ * (bins-1);
                int binY = point.pt.y / height_ * (bins-1);
                this->bins_.at(binX).at(binY).push_back(idx);

/*
                std::vector<std::pair<int, int>> pixel;

/*                pixelOfLine((ld.beg.x() / width_ ) * (bins-1), (ld.beg.y() / height_ ) * (bins-1), (ld.end.x() / width_ ) * (bins-1), (ld.end.y() / height_ ) * (bins-1), pixel);

                for_each(pixel.begin(), pixel.end(), [&qbins, &ld, &idx](std::pair<int, int> &p){
                    // these checks ensure that the array size is not exceeded, alternatively cut the line?
                    if(p.first >= 0 && p.first < bins && p.second >= 0 && p.second < bins)
                        ((qbins.at(p.first)).at(p.second)).push_back(idx);
                });
*/

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


    };


}
#endif
#endif
