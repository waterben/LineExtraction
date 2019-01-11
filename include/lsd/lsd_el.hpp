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

#ifndef _LSD_EL_HPP_
#define _LSD_EL_HPP_
#ifdef __cplusplus

#include <lsd/lsd_base.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <edge/edge_source.hpp>
#include <edge/edge_linking.hpp>
#include <edge/split.hpp>
#include <edge/fit.hpp>
#include <edge/nfa.hpp>
#include <edge/spe.hpp>

namespace lsfm {

    //! Enable nfa validation
    static const int EL_USE_NFA = 1;
    //! Use precise direction map for sub pixel esitmation
    static const int EL_USE_PRECISE_SPE = 2;

    template<class FT, template<class> class LPT = Vec2, class PT = LPT<int>,
        class ESOURCE = EdgeSourceGRAD<DerivativeGradient<uchar, short, int, FT, SobelDerivative, 
            QuadraticMagnitude>, NonMaximaSuppression<short, int, FT>>,
        class EDGE = EsdLinking<int, ESOURCE::NUM_DIR>, //EsdLinking<int,true>, 
        class NFA = NfaBinom<short, FT, index_type>,
        class SPE = PixelEstimator<FT,PT>,
        class SPLIT = RamerSplit<FT, PT>,// RamerSplit<FT, PT>, ExtRamerSplit<SimpleMerge<ExtSplitCheck<FT,int,PT>>>, LeastSquareSplit<FT,PT>,
        class FIT = FitLine<EigenFit<FT, PT>> // MEstimatorFitLine<FT,PT>
    >
    class LsdEL: public LsdExt<FT,LPT,PT>
    {
        ESOURCE esource_;
        EDGE edge_;
        SPLIT split_;
        FIT fit_;
        NFA nfa_;
        int flags_;

        typename LsdExt<FT,LPT,PT>::PointVector points_;
        EdgeSegmentVector segments_;

        mutable typename LsdBase<FT,LPT>::ImageData imageData_;
        
        void init() {
            this->addManager(esource_);
            this->addManager(edge_);
            this->addManager(split_);
            this->addManager(fit_);
            this->addManager(nfa_);

            this->add("line_flags", std::bind(&LsdEL<FT, LPT, PT, ESOURCE, EDGE, NFA, SPE, SPLIT, FIT>::valueFlags, this, std::placeholders::_1),
                "Flags for line detector: 0 - none, 1 - use nfa, 2 - use precise sub pixel estimation.");

        }

        virtual void clearData() final {
            LsdBase<FT, LPT>::clearData();
            imageData_.clear();
        }

        using LsdBase<FT,LPT>::endPoints_;
        using LsdBase<FT,LPT>::lineSegments_;

    public:
        typedef FT float_type;
        typedef LPT<FT> line_point;
        typedef PT point_type;

        typedef typename LsdBase<FT, LPT>::Line Line;
        typedef typename LsdBase<FT, LPT>::LineVector LineVector;
        typedef typename LsdBase<FT,LPT>::LineSegment LineSegment;
        typedef typename LsdBase<FT,LPT>::LineSegmentVector LineSegmentVector;
        typedef typename LsdBase<FT,LPT>::ImageData ImageData;
        typedef typename LsdExt<FT,LPT,PT>::PointVector PointVector;

        LsdEL(FT th_low = static_cast<FT>(0.004), FT th_high = static_cast<FT>(0.012), int min_pix = 10, FT dist = 2, int min_len = 5, FT log_eps = 0, int flags = 0) :
            esource_({ NV("nms_th_low", th_low), NV("nms_th_high", th_high) }), edge_(min_pix), nfa_(log_eps), split_(dist, min_len), flags_(flags) {
            init();
        }

        LsdEL(ValueManager::InitializerList options)  {
            init();
            this->value(options);
        }

        LsdEL(ValueManager::NameValueVector options) {
            init();
            this->value(options);
        }

        Value valueFlags(const Value &f = Value::NAV()) { if (f.type()) flags(f.getInt()); return flags_; }

        int flags() const { return flags_; }

        void flags(int f) {
            flags_ = f;
        }

        using LsdBase<FT,LPT>::detect;
        using LsdBase<FT,LPT>::lines;
        using LsdBase<FT, LPT>::lineSegments;
        using LsdBase<FT,LPT>::endPoints;
        using LsdBase<FT,LPT>::imageDataDescriptor;
        using LsdBase<FT,LPT>::imageData;

        virtual void detect(const cv::Mat& image) final {
            clearData();
            
            esource_.process(image);
            edge_.detect(esource_);
            if (flags_ & EL_USE_PRECISE_SPE)
                SPE::convertDir(edge_.points(), points_, esource_.magnitude(), esource_.direction());
            else
                SPE::convert(edge_.points(), points_, esource_.magnitude(), esource_.directionMap());

            split_.setup(esource_);
            if (flags_ & EL_USE_NFA) {
                nfa_.update(esource_);
                EdgeSegmentVector tmpSeg;
                std::vector<FT> tmpNfa;
                nfa_.eval(edge_, tmpSeg, tmpNfa);
                split_.apply(tmpSeg, points_, segments_);
            }
            else {
                split_.apply(edge_.segments(), points_, segments_);
            }

            lineSegments_.reserve(segments_.size());
            for_each(segments_.begin(), segments_.end(), [this](const EdgeSegment &seg) {
                Line l;
                fit_.apply(this->points_.data() + seg.begin(), this->points_.data() + seg.end(), l);
                
                //std::cout << "reverse: " << (seg.reverse() ? "true" : "false") << std::endl;
                const PT &first = this->points_[seg.first()];
                const PT &last = this->points_[seg.last()];

                //lastx - firstx = dx = -ny = -dx = firstx - lastx
                //lasty - firsty = dy = nx
                // correct direction of line
                /*int epnx = getY(first) - getY(last);
                int epny = getX(last) - getX(first);
                if (epnx * l.normalX() + epny * l.normalY() < 0)
                    l.normalFlip();*/

                lineSegments_.push_back(LineSegment(l, first, last));
            });
        }

        virtual const DataDescriptor& imageDataDescriptor() const final {
            static DataDescriptor dsc;
            if (dsc.empty()) {
                dsc.push_back(DataDescriptorEntry("gx", "Gradient in x direction"));
                dsc.push_back(DataDescriptorEntry("gy", "Gradient in y direction"));
                dsc.push_back(DataDescriptorEntry("dir", "Gradient direction"));
                dsc.push_back(DataDescriptorEntry("mag", "Gradient magnitude"));
                dsc.push_back(DataDescriptorEntry("edge_map", "Edge map, indicating if pixel is on edge or not (also giving direction 0-7)"));
            }
            return dsc;
        }

        virtual const ImageData& imageData() const final {
            if (imageData_.empty()) {
                imageData_.push_back(esource_.gx());
                imageData_.push_back(esource_.gy());
                imageData_.push_back(esource_.direction());
                imageData_.push_back(esource_.magnitude());
                imageData_.push_back(esource_.directionMap());
            }
            return imageData_;
        }

        const EdgeSegmentVector& segments() const {
            return edge_.segments();
        }

        virtual const EdgeSegmentVector& lineSupportSegments() const final {
            return segments_;
        }

        virtual const PointVector& points() const final {
            return points_;
        }

        virtual const IndexVector& indexes() const final {
            return edge_.points();
        }

        ESOURCE& edgeSource() {
            return esource_;
        }

        const ESOURCE& edgeSource() const {
            return esource_;
        }

        EDGE& edge() {
            return edge_;
        }

        const EDGE& edge() const {
            return edge_;
        }

        SPLIT& split() {
            return split_;
        }

        const SPLIT& split() const {
            return split_;
        }
        
        FIT& fit() {
            return fit_;
        }

        const FIT& fit() const {
            return fit_;
        }

        NFA& nfa() {
            return nfa_;
        }
        const NFA& nfa() const {
            return nfa_;
        }
    };
 
}
#endif
#endif
