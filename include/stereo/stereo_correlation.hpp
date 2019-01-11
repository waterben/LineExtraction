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
//
//M*/

#ifndef _STEREO_CORRELATION_HPP_
#define _STEREO_CORRELATION_HPP_
#ifdef __cplusplus

// #include <utility/interpolate.hpp>
#include <geometry/line.hpp>
#include <geometry/line3.hpp>
#include <line_tracker/analyzed_line.hpp>
// #include <dlib/optimization.h>


namespace lsfm {


    class StereoCorrelation
    {
    public:
        AnalyzedLine *leftLine;
        AnalyzedLine *rightLine;

        double shift;
        double shear;

        // too close to horizontal for stereo usage
        bool horizontal = false;

        LineSegment3<double> line3d;


        //TODO: integrate subpixel accuracy
        double getDisparityY(double y){
            double patchHeight = std::max(1.0, (std::round(this->getCommonMaxY()) - std::round(this->getCommonMinY())));
            double halfPatchHeight = (patchHeight/2.0) - 0.5;
            double currentShear =  (y - this->getCommonMinY()) - halfPatchHeight;
            currentShear = (currentShear / halfPatchHeight) * shear;

            return std::abs(this->getLeftLine().x(y) - (this->getRightLine().x(y) + shift + currentShear));
        }

        lsfm::LineSegment<double> getLeftLine(){
            return this->leftLine->line;
        }

        lsfm::LineSegment<double> getRightLine(){
            return this->rightLine->line;
        }

        double getCommonMinY(){
            return (this->getLeftLine().startPoint().y() < this->getRightLine().endPoint().y() ? std::max(this->getLeftLine().startPoint().y(), this->getRightLine().startPoint().y()) : std::max(this->getLeftLine().endPoint().y(), this->getRightLine().endPoint().y()));
        }

        double getCommonMaxY(){
            return (this->getLeftLine().startPoint().y() < this->getRightLine().endPoint().y() ? std::min(this->getLeftLine().endPoint().y(), this->getRightLine().endPoint().y()) : std::min(this->getLeftLine().startPoint().y(), this->getRightLine().startPoint().y()));
        }


    };

}
#endif
#endif
