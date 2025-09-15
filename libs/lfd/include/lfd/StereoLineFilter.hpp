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


#ifndef _LFD_STEREOLINEFILTER_HPP_
#define _LFD_STEREOLINEFILTER_HPP_
#ifdef __cplusplus

#include <algorithm>
#include <geometry/line.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <utility/option_manager.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <array>
#include <math.h>

namespace lsfm {

    
    //! stereo line filter -> uses binning and x-axis sorting + constrains to sort
    //! out bad line match candidates
    template<class FT, class GV,  unsigned int bins = 12, class DM = DescriptorMatch<FT> >
    class StereoLineFilter : public FeatureFilter<FT> , public OptionManager
    {
        FT maxDisPx_, angleTh_, minYOverlap_;
        int height_;

        struct LineData {
            LineData(const lsfm::Vec2<FT>& b = lsfm::Vec2<FT>(), const lsfm::Vec2<FT>& e = lsfm::Vec2<FT>(), FT a = 0, FT s = 0) : beg(b), end(e), angle(a), scale(s) {}
            
            lsfm::Vec2<FT> beg, end;
            FT angle, scale;
        };

        std::vector<LineData> ldLeft_, ldRight_;

        // first rotation (4 areas in coordsystem), then number of bins, then variable sized vectors with candidates
        std::array<std::array<std::vector<int>,bins>,4> bins_;

        static constexpr FT H_LINE_TOL = static_cast<FT>(20);  // Angle in Degree
        static constexpr FT MIN_VERTICAL_LENGTH = static_cast<FT>(6);

    public:
        typedef FT float_type;
        typedef GV geometric_vector;
        typedef LineSegment<FT> geometric_type;


        StereoLineFilter(int height, FT maxDisPx = 10000, FT angleTh = 45, FT minYOverlap = 0.5) :
            height_(height), maxDisPx_(maxDisPx), angleTh_(angleTh), minYOverlap_(minYOverlap) {
            CV_Assert(height >= 0 && maxDisPx_ >= 0 && angleTh > 0 && minYOverlap > 0 && minYOverlap <= 1);

            std::string type = (sizeof(float_type) > 4 ? "double" : "float");
            this->options_.push_back(OptionManager::OptionEntry("height", height, "int", "Image height."));
            this->options_.push_back(OptionManager::OptionEntry("maxDisPx", maxDisPx, type, "Maximal line distance in pixels."));
            this->options_.push_back(OptionManager::OptionEntry("angleTh", angleTh, type, "Maximal angle between two corresponding lines."));
            this->options_.push_back(OptionManager::OptionEntry("minYOverlap", minYOverlap, type, "Minimal Y overlap between two corresponding lines (Range 0-1)"));
        }

        void train(const GV& left, const GV& right) {
            if(height_ <= 0) std::cout << "SLF: Height must not be zero!" << std::endl;
            trainSide(left,ldLeft_);
            trainSide(right,ldRight_);
        }

        virtual bool filter(const LineData &ld, const LineData &rd) const {
            //if(height_ <= 0) std::cout << "SLF: Height must not be zero!" << std::endl;
            assert(height_ > 0);

            // vertical length too short?
            if ( fabs( ld.beg.y() - ld.end.y() ) < static_cast<FT>(MIN_VERTICAL_LENGTH) || fabs( rd.beg.y() - rd.end.y() ) < static_cast<FT>(MIN_VERTICAL_LENGTH) ){
//                std::cout << " too short on vertical length";
//                return true;
            }

            // x's of endpoints in left image have to be >= than on the right
            if (ld.beg.x() < rd.beg.x() || ld.end.x() < rd.end.x()){
//                std::cout << " stereo constraint not met, left more right...";
                return true;
            }

            // if distance between left x and right x > maxDist_, filter
            if (ld.beg.x() - rd.beg.x() > maxDisPx_ || ld.end.x() - rd.end.x() > maxDisPx_){
//                std::cout << " x-Distance too high";
                return true;
            }
/*
            if(lsLeft.normalY() > 0.985 || lsLeft.normalY() < -0.985 ||
               lsRight.normalY() > 0.985 || lsRight.normalY() < -0.985 )
                return false;
*/
            // skip lines that are nearly horizontal
            FT angleMod180 = fmod(ld.angle,180);
            if ( angleMod180 < H_LINE_TOL || angleMod180 > (static_cast<FT>(180) - H_LINE_TOL) ) {
//                std::cout << " too horizontal";
//                return true;
            }

            // skip lines with unsimilar orientations
            FT adiff = fabs(ld.angle - rd.angle);
            adiff += (adiff > 180) ? -360 : 0;
            adiff = fabs(adiff);
            if (adiff > angleTh_){
//                std::cout << " unsimilar orientation";
                return true;
            }

            // scale angle diff to increase the threshold
            // for more horizontal lines up to pi / 4
            //if (adiff > std::min(angleTh_ * ld.scale, static_cast<FT>(45) ) )
            //if (adiff > angleTh_)
            //    return true;

            /*
            // skip line pairs with main direction changes
            if ((ld.angle < 90 && rd.angle > 90) ||  (ld.angle > 90 && rd.angle < 90) ||
                (ld.angle < 180 && rd.angle > 180) || (ld.angle > 180 && rd.angle < 180) ||
                (ld.angle < 270 && rd.angle > 270) ||(ld.angle > 270 && rd.angle < 270))
                return true;
            */

            // no y overlap (remember that lines may also be kicked out if they are going in opposite directions)
            if (!((ld.beg.y() <= rd.end.y() && rd.beg.y() <= ld.end.y()) || (ld.beg.y() >= rd.end.y() && rd.beg.y() >= ld.end.y()))){
//                std::cout << " no y Overlap at all";
                return true;
            }

            // min y Overlap Ratio for one of both lines required
            FT l1 = fabs(ld.end.y() - ld.beg.y()) * minYOverlap_, l2 = fabs(rd.end.y() - rd.beg.y()) * minYOverlap_, l3;

            if (rd.beg.y() > ld.beg.y())
                l3 = fabs(ld.end.y() - rd.beg.y());
            else
                l3 = fabs(rd.end.y() - ld.beg.y());
            if (l3 < l1 && l3 < l2){
//                std::cout << " not enough y Overlap";
                return true;
            }

            return false;
        }

        virtual bool filter(int lfIdx, int rfIdx) const {

            const LineData &ld = ldLeft_[lfIdx];
            const LineData &rd = ldRight_[rfIdx];
            return filter(ld, rd);
        }

        virtual bool filter(geometric_type l, geometric_type r) const {
            const LineData &ld = LineData(l.startPoint(),l.endPoint(),l.anglef(), std::abs(1 / l.normalX()));
            const LineData &rd = LineData(r.startPoint(),r.endPoint(),r.anglef(), std::abs(1 / r.normalX()));
            return filter(ld, rd);
        }

        inline const std::vector<LineData>& ldLeft() const {
            return ldLeft_;
        }

        inline const std::vector<LineData>& ldRight() const {
            return ldRight_;
        }

        using  FeatureFilter<FT>::create;

        template<class FMV>
        void create(const GV& left, const GV& right, FMV& matches) {
            if(height_ <= 0) std::cout << "SLF: Height must not be zero!" << std::endl;
            matches.clear();
            matches.reserve(left.size() * right.size() / 10);

            FT bstep = static_cast<FT>(height_) / bins;

            trainSide(left,ldLeft_);
            trainSideAndBins(right, bstep, ldRight_);

            std::vector<char> ridxList;
            size_t lsize = ldRight_.size();
            ridxList.reserve(lsize);

            int size = static_cast<int>(ldLeft_.size());
            for (int lidx = 0; lidx != size; ++lidx) {
                const LineData ld = ldLeft_[lidx];
                ridxList.assign(lsize,0);

                const std::array<std::vector<int>,bins>& qbins = this->bins_[static_cast<int>(ld.angle / 90) % 4];

                int start = static_cast<int>(ld.beg.y()) / bstep, end = static_cast<int>(ld.end.y()) / bstep;

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
                    for_each(qbins[start].begin(), qbins[start].end(), [&,this](int ridx) {
                        if (!ridxList[ridx] && !this->filter(lidx,ridx)) {
                            matches.push_back(typename FMV::value_type(lidx,ridx));
                            ++ridxList[ridx];
                        }
                    });
                }
            }
        }

        template<class FMV, class MV>
        void create(const GV& left, const GV& right, FMV& matches, MV& lm, MV& rm) {
            if(height_ <= 0) std::cout << "SLF: Height must not be zero!" << std::endl;
            matches.clear();
            matches.reserve(left.size() * right.size() / 10);

            lm.resize(left.size(), 0);
            rm.resize(right.size(), 0);

            FT bstep = static_cast<FT>(height_) / bins;

            trainSide(left,ldLeft_);
            trainSideAndBins(right, bstep, ldRight_);

            std::vector<char> ridxList;
            size_t lsize = ldRight_.size();
            ridxList.reserve(lsize);

            int size = static_cast<int>(ldLeft_.size());
            for (int lidx = 0; lidx != size; ++lidx) {
                const LineData ld = ldLeft_[lidx];
                ridxList.assign(lsize,0);

                const std::array<std::vector<int>,bins>& qbins = this->bins_[static_cast<int>(ld.angle / 90) % 4];

                int start = static_cast<int>(ld.beg[1] / bstep), end = static_cast<int>(ld.end[1] / bstep);

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
                    for_each(qbins[start].begin(), qbins[start].end(), [&, this](int ridx) {
                        if (!ridxList[ridx] && !this->filter(lidx,ridx)) {
                            matches.push_back(typename FMV::value_type(lidx,ridx));
                            ++lm[lidx];
                            ++rm[ridx];
                            ++ridxList[ridx];
                        }
                    });
                }
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
        inline void trainSide(const GV& lines, std::vector<LineData> &data) {
            data.clear();
            data.reserve(lines.size());
            for_each(lines.begin(),lines.end(),[&](const geometric_type& line){
                data.push_back(LineData(line.startPoint(),line.endPoint(),line.anglef(), std::abs(1 / line.normalX())));
            });
        }

        inline void trainSideAndBins(const GV& lines, FT bstep, std::vector<LineData> &data) {
            // clear line data
            data.clear();
            data.reserve(lines.size());
            // clear bin data
            for_each(bins_.begin(), bins_.end(), [&lines](std::array<std::vector<int>,bins> &b) {
               for_each(b.begin(), b.end(), [&lines](std::vector<int> &v) {
                    v.clear();
                    v.reserve(lines.size() / 10);
               });
            });

            int size = static_cast<int>(lines.size());
            for (int idx = 0; idx != size; ++idx) {
                const geometric_type& line = lines[idx];
                LineData ld(line.startPoint(),line.endPoint(),line.anglef(), std::fabs(1 / line.normalX()));
                data.push_back(ld);

                std::array<std::vector<int>,bins>& qbins = this->bins_[static_cast<int>(ld.angle / 90) % 4];
                int start = static_cast<int>(ld.beg[1] / bstep), end = static_cast<int>(ld.end[1] / bstep);

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
            }
        }
    };

}
#endif
#endif
