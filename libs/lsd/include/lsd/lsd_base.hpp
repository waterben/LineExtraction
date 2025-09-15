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

#ifndef _LSD_BASE_HPP_
#define _LSD_BASE_HPP_
#ifdef __cplusplus

#include <lsd/ld_base.hpp>
#include <edge/edge_segment.hpp>

namespace lsfm {

    //! line segment detector base class
    template<class FT, template<class> class LPT = Vec2>
    class LsdBase : public LdBase<FT, LPT>
    {
    public:
        typedef FT float_type;
        typedef LPT<FT> line_point;
        template<class A>
        using line_point_template = LPT<A> ;

        typedef typename LdBase<FT, LPT>::Line Line;
        typedef typename LdBase<FT, LPT>::LineVector LineVector;
        typedef lsfm::LineSegment<FT, LPT> LineSegment;
        typedef std::vector<LineSegment> LineSegmentVector;
        typedef Vec4<FT> EndPoint;
        typedef std::vector<EndPoint> EndPointVector;
        typedef typename LdBase<FT, LPT>::ImageData ImageData;

        virtual ~LsdBase() {}

        using LdBase<FT, LPT>::detect;
        using LdBase<FT, LPT>::imageDataDescriptor;
        using LdBase<FT, LPT>::imageData;


        // interface helpers
        inline void detect(const cv::Mat& image, LineSegmentVector& l) { detect(image); l = lineSegments(); }
        inline void detect(const cv::Mat& image, LineSegmentVector& l, ImageData& id) { detect(image); l = lineSegments(); id = imageData(); }

        inline void detect(const cv::Mat& image, EndPointVector& e) { detect(image); e = endPoints(); }
        inline void detect(const cv::Mat& image, EndPointVector& e, ImageData& id) { detect(image); e = endPoints(); id = imageData(); }

        //! Get detected lines as line vector.
        virtual const LineVector& lines() const override {
            if (lineSegments_.size() && lines_.empty()) {
                for_each(lineSegments_.begin(), lineSegments_.end(), [this](const LineSegment& ls) {
                    this->lines_.push_back(ls);
                });
            }
            return lines_;
        }
        
        //! Get detected lines as line vector.
        virtual const LineSegmentVector& lineSegments() const { return lineSegments_; }

        
        //! Get detected lines as line point vector.
        //! Return: A vector of Vec4f elements specifying the beginning and ending point of a line segment.
        //!         Where Vec4f is (x1, y1, x2, y2), point 1 is the start, point 2 - end.
        //!         Returned lines are strictly oriented depending on the gradient.
        virtual const EndPointVector& endPoints() const {  
            if (lineSegments_.size() && endPoints_.empty()) {
                endPoints_.reserve(lineSegments_.size());
                for_each(lineSegments_.begin(), lineSegments_.end(), [this](const LineSegment& l) {
                    endPoints_.push_back(l.endPoints());
                });
            }
            return endPoints_;
        }   
        
    protected:
        LsdBase() {}
        using LdBase<FT, LPT>::lines_;
        mutable LineSegmentVector lineSegments_;
        mutable EndPointVector endPoints_;

        virtual void clearData() override {
            lines_.clear();
            lineSegments_.clear();
            endPoints_.clear();
        }
    };

    //! extended interface to access internal data like point lists or edge segments
    template<class FT, template<class> class LPT = Vec2, class PT = Vec2i>
    class LsdExt : public LsdBase<FT, LPT>
    {
    public:
        typedef FT float_type;
        typedef LPT<FT> line_point;
        typedef PT point_type;

        typedef typename LsdBase<FT,LPT>::LineSegment LineSegment;
        typedef typename LsdBase<FT,LPT>::LineSegmentVector LineSegmentVector;
        typedef typename LsdBase<FT,LPT>::EndPoint EndPoint;
        typedef typename LsdBase<FT,LPT>::EndPointVector EndPointVector;
        typedef typename LsdBase<FT,LPT>::ImageData ImageData;
        typedef std::vector<PT> PointVector;

        virtual ~LsdExt() {};

        using LsdBase<FT,LPT>::detect;
        using LsdBase<FT,LPT>::lines;
        using LsdBase<FT, LPT>::lineSegments;
        using LsdBase<FT,LPT>::endPoints;
        using LdBase<FT, LPT>::imageDataDescriptor;
        using LdBase<FT, LPT>::imageData;

        
        //! get line support segment list as edge segment type
        //! has same size as lines and same indicies -> support list i correponds to line segment i
        virtual const EdgeSegmentVector& lineSupportSegments() const = 0;

        //! get point list -> a single edge segment refers to a support point list
        virtual const PointVector& points() const = 0;

        //! get index list -> a single edge segment refer to a support index list
        virtual const IndexVector& indexes() const = 0;

    protected:
        LsdExt() {}
        using LsdBase<FT, LPT>::lines_;
        using LsdBase<FT, LPT>::lineSegments_;
        using LsdBase<FT, LPT>::endPoints_;
    };

}
#endif
#endif
