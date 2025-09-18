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

// detect junction or corner areas and skip them for optimization -> less error for wide range interpolation like cubic....

#ifndef _LINE_OPTIMIZER_HPP_
#define _LINE_OPTIMIZER_HPP_
#ifdef __cplusplus

#include <imgproc/mean.hpp>
#include <dlib/optimization.h>

namespace lsfm {
    template <
        class mat_type,
        class search_strategy_type = dlib::bfgs_search_strategy,
        class stop_strategy_type = dlib::objective_delta_stop_strategy
        >
        struct LineOptimizer {

        //! @brief optimize line for better fitting to gradient maginutde image (rotation and orthogonal translation to line)
        //! @param mag is the magnitude image as mat, it could also be the quadratic magnitude as integer
        //! @param l is the line segment to optimize parameters for
        //! @param d distnance in pixels, parameter to optimize
        //! @param r radian for rotation, parameter to optimize
        //! @param d_lower lower searching bound for distance parameter optimization
        //! @param d_upper upper searching bound for distance parameter optimization
        //! @param r_lower lower searching bound for rotation parameter optimization
        //! @param r_upper upper searching bound for rotation parameter optimization
        //! @param mean_param option for mean calculation (see mean variants)
        //! @param derivative_prec delta for computing approximating derivatives (done with dlib helper)
        //! @param interpolate_op operator for interpolaton (function pointer to linear or cubic interpolation)
        //! @param mean_op operator for mean computation (function pointer to variants of mean - step over line by 
        //!        fixed distance or use fixed samples and compute distance to have always the same number of points)
        //! @param search search strategy of optimizer (see dlib for more informations)
        //! @param stop stop strategy of optimizer (see dlib for more informations)
        //! @return error for found values
        template<class FT, template<class> class LPT>
        static inline double optimize(const cv::Mat& mag, const LineSegment<FT, LPT>& l, FT& d, FT& r,
            double d_lower = -1, double d_upper = 1, double r_lower = -CV_PI / 180, double r_upper = CV_PI / 180,
            double mean_param = 1, double derivative_prec = 1e-7,
            typename MeanHelper<double,LPT>::func_type mean_op = Mean<double, mat_type, LinearInterpolator<double, mat_type>>::process,
            search_strategy_type search = dlib::bfgs_search_strategy(),
            stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {

            CV_Assert(cv::DataType<mat_type>::type == mag.type());

            typedef dlib::matrix<double, 0, 1> column_vector;
            auto eval = [&](const column_vector& v) -> double {
                LineSegment<double, LPT> tmp = l;
                tmp.translateOrtho(v(0));
                tmp.rotate(v(1), tmp.center());
                return mean_op(mag, tmp, mean_param);
            };

            column_vector starting_point(2), lower(2), upper(2);
            starting_point = d, r;
            lower = d_lower, r_lower;
            upper = d_upper, r_upper;
            double ret = dlib::find_max_box_constrained(search, stop,
                eval, dlib::derivative(eval, derivative_prec), starting_point, lower, upper);
            d = starting_point(0);
            r = starting_point(1);
            return ret;

        }

        //! This variant will optimize the line object instead of just calculating the optimized parameters for the line
        template<class FT, template<class> class LPT>
        static inline double optimize_line(const cv::Mat& mag, LineSegment<FT, LPT>& l,
                double d_lower = -1, double d_upper = 1, double r_lower = -CV_PI / 180, double r_upper = CV_PI / 180,
                double mean_param = 1, double derivative_prec = 1e-7,
                typename MeanHelper<double,LPT>::func_type mean_op = Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
                search_strategy_type search = dlib::bfgs_search_strategy(),
                stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
            FT d = 0, r = 0;
            double ret = optimize(mag,l, d, r, d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op, search, stop);
            l.translateOrtho(d);
            l.rotate(r, l.center());
            return ret;
        }

        //! This variant will optimize the line object instead of just calculating the optimized parameters for the line
        template<class FT, template<class> class LPT, class LV>
        static inline void optimizeLV(const cv::Mat& mag, LV& in,
            double d_lower = -1, double d_upper = 1, double r_lower = -CV_PI / 180, double r_upper = CV_PI / 180,
            FT mean_param = 1, double derivative_prec = 1e-7,
            typename MeanHelper<double,LPT>::func_type mean_op = Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
            search_strategy_type search = dlib::bfgs_search_strategy(),
            stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
            for (size_t i = 0; i != in.size(); ++i)
                optimize_line(mag, in[i], d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op, search, stop);
        }

        //! This variant will optimize the line object instead of just calculating the optimized parameters for the line
        template<class FT, template<class> class LPT, class LV>
        static inline void optimizeLV(const cv::Mat& mag, LV& in, std::vector<double> &err,
            double d_lower = -1, double d_upper = 1, double r_lower = -CV_PI / 180, double r_upper = CV_PI / 180,
            double mean_param = 1, double derivative_prec = 1e-7,
            typename MeanHelper<FT,LPT>::func_type mean_op = Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
            search_strategy_type search = dlib::bfgs_search_strategy(),
            stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
            err.resize(in.size());
            for (size_t i = 0; i != in.size(); ++i)
                err[i] = optimize_line(mag, in[i], d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op, search, stop);
        }

        //! This variant will optimize the line object instead of just calculating the optimized parameters for the line
        template <class FT, template <class> class LPT, class LV>
        static inline void optimizeLV(const cv::Mat& mag,
                                      const LV& in,
                                      LV& out,
                                      double d_lower = -1,
                                      double d_upper = 1,
                                      double r_lower = -CV_PI / 180,
                                      double r_upper = CV_PI / 180,
                                      double mean_param = 1,
                                      double derivative_prec = 1e-7,
                                      typename MeanHelper<FT, LPT>::func_type mean_op =
                                          Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
                                      search_strategy_type search = dlib::bfgs_search_strategy(),
                                      stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
          out.resize(in.size());
          for (size_t i = 0; i != in.size(); ++i) {
            out[i] = in[i];
            optimize_line(mag, out[i], d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec, mean_op, search,
                          stop);
          }
        }

        //! This variant will optimize the line object instead of just calculating the optimized parameters for the line
        template <class FT, template <class> class LPT, class LV>
        static inline void optimizeLV(const cv::Mat& mag,
                                      const LV& in,
                                      LV& out,
                                      std::vector<double>& err,
                                      double d_lower = -1,
                                      double d_upper = 1,
                                      double r_lower = -CV_PI / 180,
                                      double r_upper = CV_PI / 180,
                                      double mean_param = 1,
                                      double derivative_prec = 1e-7,
                                      typename MeanHelper<double, LPT>::func_type mean_op =
                                          Mean<double, mat_type, LinearInterpolator<double, mat_type>>::processs,
                                      search_strategy_type search = dlib::bfgs_search_strategy(),
                                      stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {
          out.resize(in.size());
          err.resize(in.size());
          for (size_t i = 0; i != in.size(); ++i) {
            out[i] = in[i];
            err[i] = optimize_line(mag, out[i], d_lower, d_upper, r_lower, r_upper, mean_param, derivative_prec,
                                   mean_op, search, stop);
          }
        }
    };
    

}
#endif
#endif
