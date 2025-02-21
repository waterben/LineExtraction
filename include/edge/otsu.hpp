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

#ifndef _OTSU_HPP_
#define _OTSU_HPP_
#ifdef __cplusplus

#include <utility/range.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

namespace lsfm {
    
    //! Compute threshold based on Otsu method
    template<class IT, int N = 256, class FT = float>
    class ThresholdOtsu  {
        Range<IT> intensityRange_;
        FT scale_;
    public:
        typedef IT img_type;

        ThresholdOtsu(IT rMax = std::numeric_limits<IT>::max()) : intensityRange_(0,rMax) {
            scale_ = static_cast<FT>(N) / intensityRange_.size();
        }

        ThresholdOtsu(const Range<IT> &r) : intensityRange_(r) {
            scale_ = static_cast<FT>(N) / intensityRange_.size();

        }

        //! Compute threshold
        inline IT process(const cv::Mat &mag) {
            CV_Assert(mag.type() == cv::DataType<IT>::type);

            cv::Size size = mag.size();
            if( mag.isContinuous() )
            {
                size.width *= size.height;
                size.height = 1;
            }

            int i, j, h[N] = {0}, idx;
            for( i = 0; i < size.height; ++i )
            {
                const IT* src = mag.ptr<IT>(i);
                for(j = 0; j < size.width; ++j ) {
                    idx = static_cast<int>(src[j]*scale_);
                    if (idx < N)
                        h[idx]++;
                }
            }

            FT mu = 0, scale = static_cast<FT>(1.)/(size.width*size.height);
            for( i = 0; i < N; ++i )
                mu += i*static_cast<FT>(h[i]);

            mu *= scale;
            FT mu1 = 0, q1 = 0;
            FT max_sigma = 0, max_val = 0;

            for( i = 0; i < N; ++i )
            {
                FT p_i, q2, mu2, sigma;

                p_i = h[i]*scale;
                mu1 *= q1;
                q1 += p_i;
                q2 = static_cast<FT>(1.) - q1;

                if( std::min(q1,q2) < std::numeric_limits<FT>::epsilon() || std::max(q1,q2) > static_cast<FT>(1.) - std::numeric_limits<FT>::epsilon() )
                    continue;

                mu1 = (mu1 + i*p_i)/q1;
                mu2 = (mu - q1*mu1)/q2;
                sigma = q1*q2*(mu1 - mu2)*(mu1 - mu2);
                if( sigma > max_sigma )
                {
                    max_sigma = sigma;
                    max_val = static_cast<FT>(i);
                }
            }

            return static_cast<IT>(max_val / scale_);
        }


        //! Get name of threshold method
        inline static const std::string name() {
            return "otsu";
        }

    };

    //! Compute threshold based on Otsu method
    template<class FT>
    class ThresholdOtsu<uchar,256,FT>  {
    public:
        typedef uchar img_type;

        ThresholdOtsu() {}

        //! Compute threshold
        inline uchar process(const cv::Mat &mag) {
            CV_Assert(mag.type() == cv::DataType<uchar>::type);

            cv::Size size = mag.size();
            if( mag.isContinuous() )
            {
                size.width *= size.height;
                size.height = 1;
            }

            const int N = 256;
            int i, j, h[N] = {0};
            for( i = 0; i < size.height; ++i )
            {
                const uchar* src = mag.ptr(i);
                for(j = 0; j < size.width; ++j )
                    h[src[j]]++;
            }

            FT mu = 0, scale = static_cast<FT>(1.)/(size.width*size.height);
            for( i = 0; i < N; ++i )
                mu += i*static_cast<FT>(h[i]);

            mu *= scale;
            FT mu1 = 0, q1 = 0;
            FT max_sigma = 0, max_val = 0;

            for( i = 0; i < N; ++i )
            {
                FT p_i, q2, mu2, sigma;

                p_i = h[i]*scale;
                mu1 *= q1;
                q1 += p_i;
                q2 = static_cast<FT>(1) - q1;

                if( std::min(q1,q2) < std::numeric_limits<FT>::epsilon() || std::max(q1,q2) > static_cast<FT>(1.) - std::numeric_limits<FT>::epsilon() )
                    continue;

                mu1 = (mu1 + i*p_i)/q1;
                mu2 = (mu - q1*mu1)/q2;
                sigma = q1*q2*(mu1 - mu2)*(mu1 - mu2);
                if( sigma > max_sigma )
                {
                    max_sigma = sigma;
                    max_val = static_cast<FT>(i);
                }
            }

            return static_cast<uchar>(max_val);
        }


        //! Get name of threshold method
        inline static const std::string name() {
            return "otsu";
        }

    };
    
}
#endif
#endif
