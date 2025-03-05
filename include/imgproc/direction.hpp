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
// C by Benjamin Wassermann
//M*/

#ifndef _DIRECTION_HPP_
#define _DIRECTION_HPP_
#ifdef __cplusplus

#  include "../utility/range.hpp"
#  include "polar.hpp"
#  include <opencv2/imgproc/imgproc.hpp>

#  include <string>


namespace lsfm {
    
    //! Precise Direction class
    //! Use GT to define Derivative/Gradient Type (short, float or double)
    //! Use DT to define Direction Type (float or double)
    template<class GT = short, class DT = float>
    struct Direction  {

        typedef GT grad_type;
        typedef DT dir_type;
        typedef Range<DT> DirectionRange;

        //! Compute precise direction with atan2.
        inline static DT process(GT gx, GT gy) {
            return std::atan2(static_cast<DT>(gy), static_cast<DT>(gx));
        }

        //! Compute precise direction.
        inline static void process(const GT* gx, const GT* gy, DT* dir, size_t size) {
            for (size_t i = 0; i != size; ++i)
                dir[i] = Direction<GT, DT>::process(gx[i], gy[i]);
        }

        //! Compute precise direction.
        inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& dir) {
            CV_Assert(gx.type() == gy.type() && (gx.type() & CV_MAT_DEPTH_MASK) == cv::DataType<GT>::type &&
                gx.rows == gy.rows && gx.cols == gy.cols);
            dir.create(gx.size(), CV_MAKETYPE(cv::DataType<DT>::type, gx.channels()));
            Direction<GT,DT>::process(gx.ptr<GT>(), gy.ptr<GT>(), dir.ptr<DT>(), gx.rows*gx.cols*gx.channels());
        }

        //! Get direction range (-PI,PI)
        inline static const DirectionRange& range() {
            static DirectionRange r(static_cast<DT>(-CV_PI), static_cast<DT>(CV_PI));
            return r;
        }

        //! Get name of direction method
        inline static const std::string name() {
            return "dir";
        }

    };
    

    //! Fast Direction class
    //! Use GT to define Derivative/Gradient Type (short, float or double)
    //! Use DT to define Direction Type (float or double)
    template<class GT = short, class DT = float>
    struct FastDirection {

        typedef GT grad_type;
        typedef DT dir_type;
        typedef Range<DT> DirectionRange;


        //! Compute fast direction with fastAtan2.
        inline static DT process(GT gx, GT gy) {
            return cv::fastAtan2(static_cast<float>(gy), static_cast<float>(gx));
        }

        //! Compute fast direction.
        inline static void process(const GT* gx, const GT* gy, DT* dir, size_t size) {
            for (size_t i = 0; i != size; ++i)
                dir[i] = FastDirection<GT,DT>::process(gx[i], gy[i]);
        }

#if (CV_MAJOR_VERSION < 3)

        inline static void process(const float* gx, const float* gy, float* dir, size_t size) {
            cv::fastAtan2(gy, gx, dir, static_cast<int>(size), true);
        }
#endif

        //! process direction using cv::Mat objects
        inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& dir) {
            CV_Assert(gx.type() == gy.type() && (gx.type() & CV_MAT_DEPTH_MASK) == cv::DataType<GT>::type &&
                gx.rows == gy.rows && gx.cols == gy.cols);
			int type = gx.type() & CV_MAT_DEPTH_MASK;
			if ((type == CV_32F || type == CV_64F) && type == cv::DataType<DT>::type)
				cv::phase(gx, gy, dir, true);
			else {
				dir.create(gx.size(), CV_MAKETYPE(cv::DataType<DT>::type, gx.channels()));
				FastDirection<GT, DT>::process(gx.ptr<GT>(), gy.ptr<GT>(), dir.ptr<DT>(), gx.rows*gx.cols*gx.channels());
			}
        }

        //! get direction range (0,360)
        inline static const DirectionRange& range()  {
            static DirectionRange r(0, 360);
            return r;
        }

        //! get name of direction method
        inline static const std::string name() {
            return "fdir";
        }
    };
}
#endif
#endif
