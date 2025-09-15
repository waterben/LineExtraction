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

#ifndef _MAGNITUDE_HPP_
#define _MAGNITUDE_HPP_
#ifdef __cplusplus

#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>
#include <string>

namespace lsfm {
	enum NormType { NONE = 0, NORM_L1, NORM_L2, NORM_L2SQR };
    //! Qudratic Magnitude class
    //! Use GT to define Derivative/Gradient Type (short, float or double)
    //! Use MT to define Magnitude Type (int float or double)
    template<class GT = short, class MT = int>
    struct QuadraticMagnitude {
        QuadraticMagnitude() {}

        typedef GT grad_type;
        typedef MT mag_type;

        //! Compute qudatric magnitude val*val.
        inline static MT single(GT val) {
            return static_cast<MT>(val)* val;
        }

		//! overload for thresholds
		inline static double singled(double val) {
			return val* val;
		}

        //! Compute qudatric magnitude gx*gx + gy*gy.
        inline static MT process(GT gx, GT gy) {
            return static_cast<MT>(gx)* gx + static_cast<MT>(gy)* gy;
        }

        //! Compute qudatric magnitude gx*gx + gy*gy.
        inline static void process(const GT* gx, const GT* gy, MT* qmag, size_t size) {
            for (size_t i = 0; i != size; ++i)
                qmag[i] = QuadraticMagnitude<GT, MT>::process(gx[i], gy[i]);
        }

        //! Compute qudatric magnitude gx*gx + gy*gy.
        //! If gx,gy are of type short or int, qmag should be of type int
        //! If gx,gy are of type float/double, qmag also should be of type float/double
        inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& qmag) {
            CV_Assert(gx.type() == gy.type() && (gx.type() & CV_MAT_DEPTH_MASK) == cv::DataType<GT>::type &&
                gx.rows == gy.rows && gx.cols == gy.cols);
            qmag.create(gx.size(), CV_MAKETYPE(cv::DataType<MT>::type, gx.channels()));
            QuadraticMagnitude<GT, MT>::process(gx.ptr<GT>(), gy.ptr<GT>(), qmag.ptr<MT>(), gx.rows*gx.cols*gx.channels());
        }

        inline static MT max(const DerivativeMax<GT>& dm, GT intensity = 1) {
            return QuadraticMagnitude<GT, MT>::process(dm.max_1st * intensity, dm.max_3rd * intensity);
        }

        //! get name of magnitude operator
        inline static const std::string name() {
            return "qmag";
        }

		inline static NormType normType() {
			return NormType::NORM_L2SQR;
		}
    };

    //! Magnitude class
    //! Use GT to define Derivative/Gradient Type (short, float or double)
    //! Use MT to define Magnitude Type (float or double)
    template<class GT = short, class MT = float>
    struct Magnitude {

        Magnitude() {}

        typedef GT grad_type;
        typedef MT mag_type;

        //! Compute absolute magnitude sqrt(val*val).
        inline static MT single(GT val) {
            return std::sqrt(static_cast<MT>(val)*val);
        }

		//! overload for thresholds
		inline static double singled(double val) {
			return std::sqrt(val*val);
		}

        //! Compute magnitude sqrt(gx*gx + gy*gy).
        inline static  MT process(GT gx, GT gy) {
            return std::sqrt(static_cast<MT>(gx)* gx + static_cast<MT>(gy)* gy);
        }

        //! Compute magnitude sqrt(gx*gx + gy*gy).
        inline static void process(const GT* gx, const GT* gy, MT* mag, size_t size) {
            for (size_t i = 0; i != size; ++i)
                mag[i] = process(gx[i], gy[i]);
        }

        //! Compute magnitude sqrt(gx*gx + gy*gy).
        //! mag must be of type float or double
        inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& mag) {
            int type = gx.type() & CV_MAT_DEPTH_MASK;
            if ((type == CV_32F || type == CV_64F) && type == cv::DataType<MT>::type)
                cv::magnitude(gx, gy, mag);
            else {
                QuadraticMagnitude<GT, MT>::process(gx, gy, mag);
                cv::sqrt(mag, mag);
            }
        }

        inline static MT max(const DerivativeMax<GT>& dm, GT intensity = 1) {
            return Magnitude<GT, MT>::process(dm.max_1st * intensity, dm.max_3rd * intensity);
        }

        //! get name of magnitude operator
        inline static const std::string name() {
            return "mag";
        }

		inline static NormType normType() {
			return NormType::NORM_L2;
		}
    };



    template<class GT = short, class MT = int>
    struct AbsoluteMagnitude  {
    public:
        AbsoluteMagnitude() {}

        typedef GT grad_type;
        typedef MT mag_type;

        //! Compute absolute magnitude |val|.
        inline static MT single(GT val) {
            return std::abs(static_cast<MT>(val));
        }

		//! overload for thresholds
		inline static double singled(double val) {
			return std::abs(val);
		}

        //! Compute absolute magnitude |gx| + |gy|.
        inline static MT process(GT gx, GT gy) {
            return std::abs(static_cast<MT>(gx)) + std::abs(static_cast<MT>(gy));
        }

        //! Compute absolute magnitude |gx| + |gy|.
        inline static void process(const GT* gx, const GT* gy, MT* amag, size_t size) {
            for (size_t i = 0; i != size; ++i)
                amag[i] = AbsoluteMagnitude<GT, MT>::process(gx[i], gy[i]);
        }

        //! Compute absolute magnitude |gx| + |gy|.
        //! If gx,gy are of type short or int, qmag should be of type int
        //! If gx,gy are of type float/double, qmag also should be of type float/double
        inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& amag) {
            CV_Assert(gx.type() == gy.type() && (gx.type() & CV_MAT_DEPTH_MASK) == cv::DataType<GT>::type &&
                gx.rows == gy.rows && gx.cols == gy.cols);
            //amag.create(gx.size(), CV_MAKETYPE(cv::DataType<MT>::type, gx.channels()));
            //AbsoluteMagnitude<GT, MT>::process(gx.ptr<GT>(), gy.ptr<GT>(), amag.ptr<MT>(), gx.rows*gx.cols*gx.channels());
			cv::add(cv::abs(gx), cv::abs(gy), amag);
        }

        inline static MT max(const DerivativeMax<GT>& dm, GT intensity = 1) {
            return std::max(AbsoluteMagnitude<GT, MT>::process(dm.max_1st, dm.max_3rd),
                AbsoluteMagnitude<GT, MT>::process(dm.max_2nd * intensity, dm.max_2nd * intensity));
        }

        //! get name of magnitude operator
        inline const std::string name() {
            return "amag";
        }

		inline static NormType normType() {
			return NormType::NORM_L1;
		}
    };



}
#endif
#endif
