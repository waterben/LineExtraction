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

#ifndef _STEREO_LINE_OPTIMIZER_HPP_
#define _STEREO_LINE_OPTIMIZER_HPP_
#ifdef __cplusplus

#include <utility/interpolate.hpp>
#include <geometry/line.hpp>
#include <dlib/optimization.h>

// for imshow
#include <opencv2/highgui/highgui.hpp>

#define OPTIMIZATION_WIDTH 9

namespace lsfm {

    /* Gaussian pdf */
    template <typename T>
    T normal_pdf(T x, T mu, T s)
    {
        static const T inv_sqrt_2pi = static_cast<T>(0.3989422804014327);
        T a = (x - mu) / s;

        return inv_sqrt_2pi / s * std::exp(-T(0.5) * a * a);
    }

    template<class mat_type, class T>
    inline void calculateGaussianRowMat(T sigma, T mu, cv::Mat_<mat_type>& gaussianRowMat){
        for(int i = 0; i < OPTIMIZATION_WIDTH; i++) {
            T xVal = normal_pdf(static_cast<T>(i) - static_cast<T>(OPTIMIZATION_WIDTH / 2.0 + 0.5), mu, sigma);
            gaussianRowMat.template at<mat_type>(0, i + 0) = xVal;
        }
    }


    template<class mat_type, class T>
    inline double calcShiftShearLinearCV(const double& shift, const double& shear, const cv::Mat_<mat_type>& imLeftMod, const cv::Mat_<mat_type>& imRightMod, const LineSegment<T>& leftLine, const LineSegment<T>& rightLine, const cv::Mat_<mat_type>& gaussianRow) {

        double commonEndY, commonStartY;

        if(leftLine.startPoint().y()< leftLine.endPoint().y()){
            commonEndY = std::min(leftLine.endPoint().y(), rightLine.endPoint().y());
            commonStartY = std::max(leftLine.startPoint().y(), rightLine.startPoint().y());
        } else {
            commonStartY = std::max(leftLine.endPoint().y(), rightLine.endPoint().y());
            commonEndY = std::min(leftLine.startPoint().y(), rightLine.startPoint().y());
        }

        LineSegment<T> shiftedLine(Vec2<T>(rightLine.startPoint().x() + shear + shift, rightLine.startPoint().y()), Vec2<T>(rightLine.endPoint().x() - shear + shift, rightLine.endPoint().y()));

        //double commonStepWidth;// = std::min(stepWidthLeft, stepWidthRight);
        double commonStepWidth = OPTIMIZATION_WIDTH;

        //this->patchSizeLl.height
        int patchHeight = std::max(1.0, (std::round(commonEndY) - std::round(commonStartY)));
        if(patchHeight < 2)
            return std::numeric_limits<T>::max();

//        double halfPatchHeight = ((double)patchHeight/2.0) - 0.5;

        cv::Size extractedShearPatchSize(commonStepWidth, patchHeight);
        cv::Mat extractedPatchLlShear(cv::Mat::zeros(extractedShearPatchSize, CV_32F));
        cv::Mat extractedPatchRlShear(cv::Mat::zeros(extractedShearPatchSize, CV_32F));//(cv::Mat::zeros(patchSizeRl, CV_32F ));

        {
            double j = 0;
            int y = std::round(commonStartY);
            for(int i = 0; i < patchHeight; i++){

                cv::Rect columnRect(0,j,commonStepWidth,1);
                cv::Mat dstRoi = extractedPatchLlShear(columnRect);

                cv::getRectSubPix(imLeftMod, columnRect.size(), cv::Point2d(leftLine.x(y), y), dstRoi);

                dstRoi = extractedPatchRlShear(columnRect);

//                double currentShear = (double) i - halfPatchHeight;
//                currentShear = (currentShear / halfPatchHeight) * shear;

                //lsfm::getRectSubPix(imRightMod, columnRect.size(), lsfm::Point2d(rightLine.x(y) + shift + currentShear, y), dstRoi);
                cv::getRectSubPix(imRightMod, columnRect.size(), cv::Point2d(shiftedLine.x(y), y), dstRoi);

                ++j;
                ++y;
            }
        }

        cv::Mat diffPatch;

     // square - would pow be faster?
     //   extractedPatchLlShear = extractedPatchLlShear.mul(extractedPatchLlShear);
     //   extractedPatchRlShear = extractedPatchRlShear.mul(extractedPatchRlShear);

        cv::absdiff(extractedPatchLlShear, extractedPatchRlShear, diffPatch);

        for(int i = 0; i < diffPatch.rows; i++){
            cv::Mat dstRoi = diffPatch(cv::Rect(0,i,commonStepWidth,1));
            dstRoi = dstRoi.mul(gaussianRow);
    //        dstRoi = dstRoi.mul(dstRoi);  //square
        }

        // increase size for displaying
/*        lsfm::resize(extractedPatchLlShear, extractedPatchLlShear, Size(), 15.0, 15.0, INTER_NEAREST);
        extractedPatchLlShear.convertTo(extractedPatchLlShear, CV_8UC1);
        lsfm::imshow("LshearOutputL", extractedPatchLlShear);

        lsfm::resize(extractedPatchRlShear, extractedPatchRlShear, Size(), 15.0, 15.0, INTER_NEAREST);
        extractedPatchRlShear.convertTo(extractedPatchRlShear, CV_8UC1);
        lsfm::imshow("RshearOutputR", extractedPatchRlShear);
*/

        return cv::sum( diffPatch )[0];
    }


    template<class mat_type, class T>
    inline double calcShiftShearLinear(const double& shift, const double& shear, const cv::Mat_<mat_type>& imLeftMod, const cv::Mat_<mat_type>& imRightMod, const LineSegment<T>& leftLine, const LineSegment<T>& rightLine) {

            double commonEndY, commonStartY;

            if(leftLine.startPoint().y < leftLine.endPoint().y){
                commonEndY = std::min(leftLine.endPoint().y, rightLine.endPoint().y);
                commonStartY = std::max(leftLine.startPoint().y, rightLine.startPoint().y);
            } else {
                commonStartY = std::max(leftLine.endPoint().y, rightLine.endPoint().y);
                commonEndY = std::min(leftLine.startPoint().y, rightLine.startPoint().y);
            }

            LineSegment<T> shiftedLine(Point_<T>(rightLine.startPoint().x + shear + shift, rightLine.startPoint().y), Point_<T>(rightLine.endPoint().x - shear + shift, rightLine.endPoint().y));

            //this->patchSizeLl.height
            int patchHeight = std::max(1.0, (std::round(commonEndY) - std::round(commonStartY)));
            if(patchHeight < 2)
                return 10000000000000000000.0;

            cv::Mat interpolateRowLeft(cv::Mat::zeros(patchHeight,4, CV_32F));
            cv::Mat interpolateRowLeft1(cv::Mat::zeros(patchHeight,4, CV_32F));
            cv::Mat interpolateRowLeft2(cv::Mat::zeros(patchHeight,4, CV_32F));
//            cv::Mat interpolatedCubicDisplayLeft(cv::Mat::zeros(patchHeight,1, CV_32F));

            cv::Mat interpolateRowRight(cv::Mat::zeros(patchHeight,4, CV_32F));
            cv::Mat interpolateRowRight1(cv::Mat::zeros(patchHeight,4, CV_32F));
            cv::Mat interpolateRowRight2(cv::Mat::zeros(patchHeight,4, CV_32F));
//            cv::Mat interpolatedCubicDisplayRight(cv::Mat::zeros(patchHeight,1, CV_32F));


            cv::Mat leftTest, rightTest;
            imLeftMod.convertTo(leftTest, CV_64F);
            imRightMod.convertTo(rightTest, CV_64F);

            //lsfm::interpolate_cubic()
            double squaredDiff = 0.0;

            {
                double j = 0;
                int y = std::round(commonStartY);
              //  double halfPatchHeight = ((double)patchHeight/2.0) - 0.5;
                for(int i = 0; i < patchHeight; i++){

              //      double currentShear = (double) i - halfPatchHeight;
              //      currentShear = (currentShear / halfPatchHeight) * shear;

                    {
                        int iX = std::floor(leftLine.x(y)) - 1;
                        for(int matX = 0; matX < 4; matX++, ++iX){
                            //check if iX inside
                            if(iX < 0 || iX >= imLeftMod.cols){
                                interpolateRowLeft.at<float>(j,matX) = 0.0;
                                std::cout << "changing: " << j << "  and  " << matX << " out of bounds1: " << iX << std::endl;
                            } else {

                                interpolateRowLeft.at<float>(j,matX) = imLeftMod.template at<float>(y, iX);
                                interpolateRowLeft1.at<float>(j,matX) = imLeftMod.template at<float>(y, iX-1);
                                interpolateRowLeft2.at<float>(j,matX) = imLeftMod.template at<float>(y, iX+1);

                            }
                            //++iX;
                        }

                        iX = std::floor(shiftedLine.x(y)) - 1;
                        for(int matX = 0; matX < 4; matX++){
                            //check if iX inside
                            if(iX < 0 || iX >= imRightMod.cols){
                                interpolateRowRight.at<float>(j,matX) = 0.0;
                       //         std::cout << "changing: " << j << "  and  " << matX << " out of bounds2: " << iX  << " shift: " << shift  << " currentShear: " << currentShear << std::endl;
                            } else {
                              //  float test = imRightMod.template at<float>(y, iX);
                                interpolateRowRight.at<float>(j,matX) = imRightMod.template at<float>(y, iX);
                                interpolateRowRight1.at<float>(j,matX) = imRightMod.template at<float>(y, iX-1);
                                interpolateRowRight2.at<float>(j,matX) = imRightMod.template at<float>(y, iX+1);

                            }
                        ++iX;
                        }

                    }


                    float arrLeft[2] = {interpolateRowLeft.at<float>(j,1), interpolateRowLeft.at<float>(j,2)};
                    float arrLeft1[2] = {interpolateRowLeft1.at<float>(j,1), interpolateRowLeft1.at<float>(j,2)};
                    float arrLeft2[2] = {interpolateRowLeft2.at<float>(j,1), interpolateRowLeft2.at<float>(j,2)};
                    double tmpValLeft = lsfm::interpolate_linear<float>(arrLeft, leftLine.x(y) - std::floor(leftLine.x(y)) );
                    double tmpValLeft1 = lsfm::interpolate_linear<float>(arrLeft1, leftLine.x(y) - std::floor(leftLine.x(y)) );
                    double tmpValLeft2 = lsfm::interpolate_linear<float>(arrLeft2, leftLine.x(y) - std::floor(leftLine.x(y)) );
//                    interpolatedCubicDisplayLeft.at<float>(j,0) = tmpValLeft;

                    float arrRight[2] = {interpolateRowRight.at<float>(j,1), interpolateRowRight.at<float>(j,2)};
                    float arrRight1[2] = {interpolateRowRight1.at<float>(j,1), interpolateRowRight1.at<float>(j,2)};
                    float arrRight2[2] = {interpolateRowRight2.at<float>(j,1), interpolateRowRight2.at<float>(j,2)};
                    //std::cout << "val: " << this->shiftedLine.line.x(y)  + shift + currentShear - std::floor(this->shiftedLine.line.x(y) + shift + currentShear)  << std::endl;
                    double tmpValRight = lsfm::interpolate_linear<float>(arrRight, shiftedLine.x(y) - std::floor(shiftedLine.x(y)) );
                    double tmpValRight1 = lsfm::interpolate_linear<float>(arrRight1, shiftedLine.x(y) - std::floor(shiftedLine.x(y)) );
                    double tmpValRight2 = lsfm::interpolate_linear<float>(arrRight2, shiftedLine.x(y) - std::floor(shiftedLine.x(y)) );
 //                   interpolatedCubicDisplayRight.at<float>(j,0) = tmpValRight;
                    //std::cout << "tmpValRight: " << tmpValRight << std::endl;
                    double squaredVal = std::abs(tmpValLeft - tmpValRight);
                    squaredVal += std::abs(tmpValLeft1 - tmpValRight1);
                    squaredVal += std::abs(tmpValLeft2 - tmpValRight2);
                    //squaredVal /= 3.0;
                    squaredVal *= squaredVal;

                    squaredDiff += squaredVal;

                    ++j;
                    ++y;
                }
            }

            return squaredDiff;// diffSum;
    }


    template<class mat_type, class T>
    inline double calcShiftShearCubic(const double& shift, const double& shear, const cv::Mat_<mat_type>& imLeftMod, const cv::Mat_<mat_type>& imRightMod, const LineSegment<T>& leftLine, const LineSegment<T>& rightLine) {

        double commonEndY, commonStartY;

        if(leftLine.startPoint().y < leftLine.endPoint().y){
            commonEndY = std::min(leftLine.endPoint().y, rightLine.endPoint().y);
            commonStartY = std::max(leftLine.startPoint().y, rightLine.startPoint().y);
        } else {
            commonStartY = std::max(leftLine.endPoint().y, rightLine.endPoint().y);
            commonEndY = std::min(leftLine.startPoint().y, rightLine.startPoint().y);
        }

        LineSegment<T> shiftedLine(Point_<T>(rightLine.startPoint().x + shear + shift, rightLine.startPoint().y), Point_<T>(rightLine.endPoint().x - shear + shift, rightLine.endPoint().y));

        int patchHeight = std::max(1.0, (std::round(commonEndY) - std::round(commonStartY)));

        cv::Mat interpolateRowLeft(cv::Mat::zeros(patchHeight,4, CV_32F));
        cv::Mat interpolatedCubicDisplayLeft(cv::Mat::zeros(patchHeight,1, CV_32F));

        cv::Mat interpolateRowRight(cv::Mat::zeros(patchHeight,4, CV_32F));
        cv::Mat interpolatedCubicDisplayRight(cv::Mat::zeros(patchHeight,1, CV_32F));


        //lsfm::interpolate_cubic()
        double squaredDiff = 0.0;

        {
            double j = 0;
            int y = std::round(commonStartY);
      //      double halfPatchHeight = (double)patchHeight/2.0 - 0.5;
            for(int i = 0; i < patchHeight; i++){

      //          double currentShear = (double) i - (double)halfPatchHeight;
      //          currentShear = (currentShear / (halfPatchHeight)) * shear;

                {
                    int iX = std::floor(leftLine.x(y)) - 1;
                    for(int matX = 0; matX < 4; matX++, ++iX){
                        //check if iX inside
                        if(iX < 0 || iX >= imLeftMod.cols){
                            interpolateRowLeft.at<float>(j,matX) = 0.0;
                            std::cout << "changing: " << j << "  and  " << matX << " out of bounds: " << iX << std::endl;
                        } else {

                            interpolateRowLeft.at<float>(j,matX) = imLeftMod.template at<float>(y, iX);

                        }
                        //++iX;
                    }

                    iX = std::floor(shiftedLine.x(y)) - 1;
                    for(int matX = 0; matX < 4; matX++){
                        //check if iX inside
                        if(iX < 0 || iX >= imRightMod.cols){
                            interpolateRowRight.at<float>(j,matX) = 0.0;
                            std::cout << "changing: " << j << "  and  " << matX << " out of bounds: " << iX << std::endl;
                        } else {

                            interpolateRowRight.at<float>(j,matX) = imRightMod.template at<float>(y, iX);

                        }
                    ++iX;
                    }

                }


             //   float arrLeft[4] = { interpolateRowLeft.at<float>(j,0), interpolateRowLeft.at<float>(j,1), interpolateRowLeft.at<float>(j,2), interpolateRowLeft.at<float>(j,3) };
                double tmpValLeft = lsfm::interpolate_cubic<float>(&interpolateRowLeft.at<float>(j,0), leftLine.x(y) - std::floor(leftLine.x(y)) );
                interpolatedCubicDisplayLeft.at<float>(j,0) = tmpValLeft;

             //   float arrRight[4] = { interpolateRowRight.at<float>(j,0), interpolateRowRight.at<float>(j,1), interpolateRowRight.at<float>(j,2), interpolateRowRight.at<float>(j,3) };
                //double tmpValRight = lsfm::interpolate_cubic<float>(arrRight, shiftedLine.x(y) - std::floor(shiftedLine.x(y)) );
                double tmpValRight = lsfm::interpolate_cubic<float>(&interpolateRowRight.at<float>(j,0), shiftedLine.x(y) - std::floor(shiftedLine.x(y)) );
                interpolatedCubicDisplayRight.at<float>(j,0) = tmpValRight;

                double squaredVal = tmpValLeft - tmpValRight;
                squaredVal *= squaredVal;

                squaredDiff += squaredVal;


                ++j;
                ++y;
            }
        }
/*
        lsfm::resize(interpolatedCubicDisplayLeft, interpolatedCubicDisplayLeft, Size(), 15.0, 15.0, INTER_NEAREST);
        interpolatedCubicDisplayLeft.convertTo(interpolatedCubicDisplayLeft, CV_8UC1);
        lsfm::imshow("interpolateMatDisplay", interpolatedCubicDisplayLeft);

        lsfm::resize(interpolatedCubicDisplayRight, interpolatedCubicDisplayRight, Size(), 15.0, 15.0, INTER_NEAREST);
        interpolatedCubicDisplayRight.convertTo(interpolatedCubicDisplayRight, CV_8UC1);
        lsfm::imshow("interpolatedCubicDisplayRight", interpolatedCubicDisplayRight);
*/
     //   std::cout << std::setprecision(25) << "  squaredDiff:: " << squaredDiff << " shift " << shift << " shear " << shear << std::endl;


        return squaredDiff;// diffSum;
    }



    template <
        class mat_type,
        class T,
        class search_strategy_type = dlib::bfgs_search_strategy,
        class stop_strategy_type = dlib::objective_delta_stop_strategy
    >
    LineSegment<T> optimizeStereopairLinear(const cv::Mat_<mat_type>& imLeftMod, const cv::Mat_<mat_type>& imRightMod,
                    const LineSegment<T>& leftLine,const LineSegment<T>& rightLine, T& shift, T& shear,
        T ret = std::numeric_limits<T>::max(), T shift_lower = -1, T shift_upper = 1, T shear_lower = -1, T shear_upper = 1,
        double derivative_prec = 1e-2,
//        typename InterpolationHelper<T, mat_type>::func_type_point interpolate_op = LinearInterpolator<T, mat_type>::get,
//        typename MeanHelper<mat_type, T>::func_type mean_op = mean<mat_type, T>,
        search_strategy_type search = dlib::bfgs_search_strategy(),
        stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-3)) {


            typedef dlib::matrix<T, 0, 1> column_vector;
            auto eval = [&](const column_vector& v) -> T {
                return calcShiftShearLinear<mat_type, T>(v(0), v(1), imLeftMod, imRightMod, leftLine, rightLine);
            };


        column_vector starting_point(2), lower(2), upper(2);
        starting_point = shift, shear;
        lower = shift_lower, shear_lower;
        upper = shift_upper, shear_upper;
        ret = dlib::find_min_box_constrained(search, stop,
            eval, dlib::derivative(eval, derivative_prec), starting_point, lower, upper);

        shift = starting_point(0);
        shear = starting_point(1);

        LineSegment<T> shiftedLine(Point_<T>(rightLine.startPoint().x + shear + shift, rightLine.startPoint().y), Point_<T>(rightLine.endPoint().x - shear + shift, rightLine.endPoint().y));

        return shiftedLine;
    }


    template <
        class mat_type,
        class T,
        class search_strategy_type = dlib::bfgs_search_strategy,
        class stop_strategy_type = dlib::objective_delta_stop_strategy
    >
    LineSegment<T> optimizeStereopairLinearCV(const cv::Mat_<mat_type>& imLeftMod, const cv::Mat_<mat_type>& imRightMod,
                    const LineSegment<T>& leftLine,const LineSegment<T>& rightLine, T& shift, T& shear,
        T ret = std::numeric_limits<T>::max(), T shift_lower = -1, T shift_upper = 1, T shear_lower = -1, T shear_upper = 1,
        double derivative_prec = 1e-2,
   //     typename InterpolationHelper<T, mat_type>::func_type_point interpolate_op = LinearInterpolator<T, mat_type>::get,
    //        typename MeanHelper<mat_type, T>::func_type mean_op = mean<mat_type, T>,
        search_strategy_type search = dlib::bfgs_search_strategy(),
        stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-3)) {


        cv::Size gaussianRowSize(OPTIMIZATION_WIDTH, 1);
        cv::Mat_<mat_type> gaussianRowMat(cv::Mat::zeros(gaussianRowSize, CV_32F));
        calculateGaussianRowMat<mat_type, T>(3.0, 0.0, gaussianRowMat);

        typedef dlib::matrix<T, 0, 1> column_vector;
        auto eval = [&](const column_vector& v) -> T {
            return calcShiftShearLinearCV<mat_type, T>(v(0), v(1), imLeftMod, imRightMod, leftLine, rightLine, gaussianRowMat);
        };

        column_vector starting_point(2), lower(2), upper(2);
        starting_point = shift, shear;
        lower = shift_lower, shear_lower;
        upper = shift_upper, shear_upper;

        ret = dlib::find_min_box_constrained(search, stop,
            eval, dlib::derivative(eval, derivative_prec), starting_point, lower, upper);

        shift = starting_point(0);
        shear = starting_point(1);

        LineSegment<T> shiftedLine(Vec2<T>(rightLine.startPoint().x() + shear + shift, rightLine.startPoint().y()), Vec2<T>(rightLine.endPoint().x() - shear + shift, rightLine.endPoint().y()));

        return shiftedLine;
    }


    template <
        class mat_type,
        class T,
        class search_strategy_type = dlib::bfgs_search_strategy,
        class stop_strategy_type = dlib::objective_delta_stop_strategy
    >
    LineSegment<T> optimizeStereopairCubic(const cv::Mat_<mat_type>& imLeftMod, const cv::Mat_<mat_type>& imRightMod,
                    const LineSegment<T>& leftLine,const LineSegment<T>& rightLine, T& shift, T& shear,
        T ret = std::numeric_limits<T>::max(), T shift_lower = -1, T shift_upper = 1, T shear_lower = -1, T shear_upper = 1,
        double derivative_prec = 1e-2,
     //   typename InterpolationHelper<T, mat_type>::func_type_point interpolate_op = LinearInterpolator<T, mat_type>::get,
    //        typename MeanHelper<mat_type, T>::func_type mean_op = mean<mat_type, T>,
        search_strategy_type search = dlib::bfgs_search_strategy(),
        stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-3)) {


            typedef dlib::matrix<T, 0, 1> column_vector;
            auto eval = [&](const column_vector& v) -> T {
                return calcShiftShearCubic<mat_type, T>(v(0), v(1), imLeftMod, imRightMod, leftLine, rightLine);
            };


        column_vector starting_point(2), lower(2), upper(2);
        starting_point = shift, shear;
        lower = shift_lower, shear_lower;
        upper = shift_upper, shear_upper;
        ret = dlib::find_min_box_constrained(search, stop,
            eval, dlib::derivative(eval, derivative_prec), starting_point, lower, upper);

        shift = starting_point(0);
        shear = starting_point(1);

        LineSegment<T> shiftedLine(Point_<T>(rightLine.startPoint().x + shear + shift, rightLine.startPoint().y), Point_<T>(rightLine.endPoint().x - shear + shift, rightLine.endPoint().y));

        return shiftedLine;
    }



    template <
        class mat_type,
        class T
    >
    LineSegment<T> optimizeStereopairFullRange(const cv::Mat_<mat_type>& imLeftMod, const cv::Mat_<mat_type>& imRightMod,
                                     const LineSegment<T>& leftLine,const LineSegment<T>& rightLine, T& shift, T& shear) {

        typedef dlib::matrix<double, 0, 1> column_vector;

        double currentMin;
        cv::Size gaussianRowSize(OPTIMIZATION_WIDTH, 1);
        cv::Mat_<float> gaussianRowMat(cv::Mat::zeros(gaussianRowSize, CV_32F));
        calculateGaussianRowMat<float, double>(3.0, 0.0, gaussianRowMat);

        //cv::Mat visualization(cv::Mat::zeros(201,201, CV_32F));
        //lsfm::FileStorage fs("vismat.yml", lsfm::FileStorage::WRITE);

        column_vector starting_point(2);
        starting_point = -1.0, -1.0; // , 0.0;
       // double currentMin = this->stereoLineOptimizer.calcShiftShear(-1.0,-1.0);
        currentMin = 1000000000.0;//(*this)(starting_point);
        double currentMinShear = -1.0;
        double currentMinShift = -1.0;

        for(double i = -1, iCtr=0; i <= 1.001; i+=0.01, ++iCtr){
            std::cout << i << std::endl;
            for(double j = -1, jCtr=0; j <= 1.001; j +=0.01, ++jCtr){

                starting_point(0) = i;
                starting_point(1) = j;
                //double tmpMin1 = (*this)(starting_point);
                double shift = i, shear = j;    // prevent functions from mutating i or j
                //double tmpMin = optimizeStereopair<float,double>(imLeftMod, imRightMod, leftLine.line, rightLine.line, shift, shear);
                //double tmpMin = calcShiftShearCubic<float,double>(shift, shear, imLeftMod, imRightMod, leftLine.line, rightLine.line);
                double tmpMin = calcShiftShearLinear<float,double>(shift, shear, imLeftMod, imRightMod, leftLine, rightLine);
                //double tmpMin = calcShiftShearLinearCV<float,double>(shift, shear, imLeftMod, imRightMod, leftLine, rightLine, gaussianRowMat);
                if(tmpMin < currentMin){
                    currentMin = tmpMin;
                    currentMinShear = i;
                    currentMinShift = j;
                }

                //visualization.at<float>(iCtr,jCtr) = tmpMin;
              //  std::cout << "i " << i << " j " << j << " orig: " << tmpMin1 << " new: " << tmpMin << std::endl;
            }


            //std::cout << "correctMin: " << currentMin <<  " currentMinShear: " << currentMinShear <<  " currentMinRot: " << currentMinShift << std::endl;

            //fs << "mat" << visualization;
            //fs.release();

        }
        shift = currentMinShift;
        shear = currentMinShear;
        return LineSegment<T>(Point_<T>(rightLine.startPoint().x + shear + shift, rightLine.startPoint().y), Point_<T>(rightLine.endPoint().x - shear + shift, rightLine.endPoint().y));
    }

}
#endif
#endif
