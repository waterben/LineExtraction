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

#ifndef _FIT_LINE_HPP_
#define _FIT_LINE_HPP_
#ifdef __cplusplus

#include <opencv2/imgproc/imgproc.hpp>
#include "../geometry/line.hpp"
#include "../utility/value_manager.hpp"
#include "index.hpp"
#include <vector>
#include <string>

namespace lsfm {

    template<class FT, class PT>
    inline void covariance(const PT *beg, const PT *end, FT &sx, FT &sy, FT &sxy, FT &cx, FT &cy) {
        int count = 0;
        FT sum_x = 0, sum_y = 0;

        std::for_each(beg, end, [&](const PT& lp) {
            ++count;
            sum_x += getX(lp);
            sum_y += getY(lp);
        });

        cx = static_cast<FT>(sum_x) / count;
        cy = static_cast<FT>(sum_y) / count;
        sx = 0; sy = 0; sxy = 0;

        std::for_each(beg, end, [&](const PT& lp) {
            FT dx = static_cast<FT>(getX(lp)) - cx;
            FT dy = static_cast<FT>(getY(lp)) - cy;

            sxy -= dx * dy;
            sx += dx * dx;
            sy += dy * dy;
        });
    }

    template<class FT, class PT, class DT = FT>
    inline void covariance(const PT *beg, const PT *end, FT &sx, FT &sy, FT &sxy, FT &cx, FT &cy, const cv::Mat& data) {
        FT count = 0;
        FT sum_x = 0, sum_y = 0;
        sx = 0; sy = 0; sxy = 0;

        if (data.empty()) {
            std::for_each(beg, end, [&](const PT& lp) {
                ++count;
                sum_x += getX(lp);
                sum_y += getY(lp);
            });

            cx = static_cast<FT>(sum_x) / count;
            cy = static_cast<FT>(sum_y) / count;

            std::for_each(beg, end, [&](const PT& lp) {
                FT dx = static_cast<FT>(getX(lp)) - cx;
                FT dy = static_cast<FT>(getY(lp)) - cy;

                sxy -= dx * dy;
                sx += dx * dx;
                sy += dy * dy;
            });
        }
        else {
            std::for_each(beg, end, [&](const PT& lp) {
                FT m = static_cast<FT>(data.at<DT>(getY(lp), getX(lp)));
                count += m;
                sum_x += getX(lp) * m;
                sum_y += getY(lp) * m;
            });

            cx = static_cast<FT>(sum_x) / count;
            cy = static_cast<FT>(sum_y) / count;

            std::for_each(beg, end, [&](const PT& lp) {
                FT m = static_cast<FT>(data.at<DT>(getY(lp), getX(lp)));

                FT dx = static_cast<FT>(getX(lp)) - cx;
                FT dy = static_cast<FT>(getY(lp)) - cy;

                sxy -= m * dx * dy;
                sx += m * dx * dx;
                sy += m * dy * dy;
            });
        }
    }




    
    
    //! fit line implementation using regression
    template<class FT, class PT, class DT = FT>
    class RegressionFit {
    public:

        typedef FT float_type;
        typedef PT point_type;
        typedef DT data_type;
        
        static const std::string name() {
            return "Regression";
        }

        static inline void fit_unorm(FT sx, FT sy, FT sxy, FT &nx, FT &ny) {

            if (sx > sy) {
                nx = sxy;
                ny = sx;
            }
            else {
                nx = sy;
                ny = sxy;
            }
        }
        
        static inline void fit(FT sx, FT sy, FT sxy, FT &nx, FT &ny) {

            fit_unorm(sx, sy, sxy, nx, ny);
            FT norm = hypot(nx, ny);
            nx /= norm;
            ny /= norm;
        }

        static inline void fit_unorm(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny) {
            FT sx, sy, sxy;

            covariance(beg, end, sx, sy, sxy, cx, cy);
            fit_unorm(sx, sy, sxy, nx, ny);
        }

        static inline void fit(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny) {
            FT sx, sy, sxy;

            covariance(beg, end, sx, sy, sxy, cx, cy);
            fit(sx, sy, sxy, nx, ny);
        }

        static inline void fit(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny, const cv::Mat& data) {
            FT sx, sy, sxy;

            covariance<FT,PT,DT>(beg, end, sx, sy, sxy, cx, cy, data);
            fit(sx, sy, sxy, nx, ny);
        }

        template<template<class> class LPT>
        static inline void fit(const PT *beg, const PT *end, Line<FT, LPT>& l) {
            FT cx, cy, nx, ny;

            fit(beg, end, cx, cy, nx, ny);
            l = Line<FT, LPT>(nx, ny, cx, cy);
        }

        template<template<class> class LPT>
        static inline void fit(const PT *beg, const PT *end, Line<FT, LPT>& l, const cv::Mat& data) {
            FT cx, cy, nx, ny;

            fit(beg, end, cx, cy, nx, ny, data);
            l = Line<FT, LPT>(nx, ny, cx, cy);
        }

    };
        
          
    //! fit line implementation using eigen value descompsition
    template<class FT, class PT, class DT = FT>
    class EigenFit {
    public:

        typedef FT float_type;
        typedef PT point_type;
        typedef DT data_type;

        EigenFit()  {}

        static std::string name() {
            return "Eigen";
        }
        
        static inline void eigen(FT sx, FT sy, FT sxy, FT &e1, FT &e2) {
            FT lambda = static_cast<FT>(0.5) * std::sqrt((sx - sy) * (sx - sy) + 4 * sxy * sxy);
            FT p = static_cast<FT>(0.5) * (sx + sy);
            e1 = p + lambda;
            e2 = p - lambda;
        }

        // gives the direction of the smallest eigenvalue, the direction for largest is eigenvalue is dy,-dx
        static inline void eigen(FT sx, FT sy, FT sxy, FT &e1, FT &e2, FT &dx, FT &dy) {
            eigen(sx, sy, sxy, e1, e2);
            fit(sx, sy, sxy, dx, dy);
        }

        static inline void fit_unorm(FT sx, FT sy, FT sxy, FT &nx, FT &ny) {

            // Compute smallest eigenvalue
            FT lambda = static_cast<FT>(0.5) * (sx + sy - std::sqrt((sx - sy) * (sx - sy) + 4 * sxy * sxy));

            if (sx > sy) {
                nx = sxy;
                ny = sx - lambda;
            }
            else {
                nx = sy - lambda;
                ny = sxy;
            }

        }

        static inline void fit(FT sx, FT sy, FT sxy, FT &nx, FT &ny) {
            
            fit_unorm(sx, sy, sxy, nx, ny);
            FT norm = hypot(nx, ny);
            nx /= norm;
            ny /= norm;
            
        }

        static inline void fit(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny) {
            FT sx, sy, sxy;

            covariance(beg, end, sx, sy, sxy, cx, cy);
            fit(sx, sy, sxy, nx, ny);
        }

        static inline void fit_unorm(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny) {
            FT sx, sy, sxy;

            covariance(beg, end, sx, sy, sxy, cx, cy);
            fit_unorm(sx, sy, sxy, nx, ny);
        }

        static inline void fit(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny, const cv::Mat& data) {
            FT sx, sy, sxy;

            covariance<FT,PT,DT>(beg, end, sx, sy, sxy, cx, cy, data);
            fit(sx, sy, sxy, nx, ny);
        }

        template<template<class> class LPT>
        static inline void fit(const PT *beg, const PT *end, Line<FT, LPT>& l) {
            FT cx, cy, nx, ny;

            fit(beg, end, cx, cy, nx, ny);
            l = Line<FT, LPT>(nx, ny, cx, cy);
        }

        template<template<class> class LPT>
        static inline void fit(const PT *beg, const PT *end, Line<FT, LPT>& l, const cv::Mat& data) {
            FT cx, cy, nx, ny;

            fit(beg, end, cx, cy, nx, ny, data);
            l = Line<FT, LPT>(nx, ny, cx, cy);
        }

    };

    //! fit line implementation using opencv eigen
    template<class FT, class PT, class DT = FT>
    class EigenCVFit {
    public:

        typedef FT float_type;
        typedef PT point_type;
        typedef DT data_type;

        EigenCVFit()  {}

        static std::string name() {
            return "EigenCV";
        }

        static inline void eigen(FT sx, FT sy, FT sxy, FT &e1, FT &e2) {
            cv::Matx<FT, 2, 2> M(sx, sxy, sxy, sy);

            cv::Mat E, V;
            cv::eigen(M, E, V);
            e1 = E.at<FT>(0, 0);
            e2 = E.at<FT>(1, 0);
        }

        // gives the direction of the smallest eigenvalue, the direction for largest is eigenvalue is dy,-dx
        static inline void eigen(FT sx, FT sy, FT sxy, FT &e1, FT &e2, FT &dx, FT &dy) {
            cv::Matx<FT, 2, 2> M(sx, sxy, sxy, sy);

            cv::Mat E, V;
            cv::eigen(M, E, V);
            e1 = E.at<FT>(0, 0);
            e2 = E.at<FT>(1, 0);
            dx = -V.at<FT>(1, 0);
            dy = V.at<FT>(1, 1);
        }

        static inline void fit(FT sx, FT sy, FT sxy, FT &nx, FT &ny) {
            cv::Matx<FT, 2, 2> M(sx, sxy, sxy, sy);

            cv::Mat E, V;
            cv::eigen(M, E, V);
            nx = -V.at<FT>(1, 0);
            ny = V.at<FT>(1, 1);
        }

        static inline void fit_unorm(FT sx, FT sy, FT sxy, FT &nx, FT &ny) {
            fit(sx, sy, sxy, nx, ny);
        }
       
        static inline void fit(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny) {
            FT sx, sy, sxy;

            covariance(beg, end, sx, sy, sxy, cx, cy);
            fit(sx, sy, sxy, nx, ny);
        }

        static inline void fit_unorm(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny) {
            fit(beg, end, cx, cy, nx, ny);
        }

        static inline void fit(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny, const cv::Mat& data) {
            FT sx, sy, sxy;

            covariance<FT,PT,DT>(beg, end, sx, sy, sxy, cx, cy, data);
            fit(sx, sy, sxy, nx, ny);
        }

        template<template<class> class LPT>
        static inline void fit(const PT *beg, const PT *end, Line<FT, LPT>& l) {
            FT cx, cy, nx, ny;

            fit(beg, end, cx, cy, nx, ny);
            l = Line<FT, LPT>(nx, ny, cx, cy);
        }

        template<template<class> class LPT>
        static inline void fit(const PT *beg, const PT *end, Line<FT, LPT>& l, const cv::Mat& data) {
            FT cx, cy, nx, ny;

            fit(beg, end, cx, cy, nx, ny, data);
            l = Line<FT, LPT>(nx, ny, cx, cy);
        }
    };

    //! fit line base class
    template<class FIT>
    class FitLine : public ValueManager {
        FitLine(const FitLine&);
    protected:
        FIT fit_;

    public:
        
        typedef FIT fit_type;
        typedef typename fit_type::float_type float_type;
        typedef typename fit_type::point_type point_type;

        FitLine() {}

        FitLine(const ValueManager::NameValueVector &options)  {}

        FitLine(ValueManager::InitializerList options)  {}

        template<class ITER, template<class> class LPT>
        inline void apply(ITER beg, ITER end, Line<float_type, LPT>& l) const {
            // type of ITER must have a conversion operator to Point
            std::vector<point_type> tmp(beg, end);
            fit_.fit(&tmp.front(), &tmp.back()+1, l);
        }

        template<class ITER, template<class> class LPT>
        inline void apply(ITER beg, ITER end, Line<float_type, LPT>& l, const cv::Mat& data) const {
            // type of ITER must have a conversion operator to Point
            std::vector<point_type> tmp(beg, end);
            fit_.fit(&tmp.front(), &tmp.back() + 1, l, data);
        }

        template<template<class> class LPT>
        inline void apply(const point_type *beg, const point_type *end, Line<float_type,LPT>& l) const {
            fit_.fit(beg, end, l);
        }

        template<template<class> class LPT>
        inline void apply(const point_type *beg, const point_type *end, Line<float_type,LPT>& l, const cv::Mat& data) const {
            fit_.fit(beg, end, l, data);
        }

        template<template<class> class LPT>
        inline void apply(typename std::vector<index_type>::const_iterator beg, typename std::vector<index_type>::const_iterator end, Line<float_type,LPT>& l) const {
            std::vector<point_type> tmp;
            tmp.resize(std::distance(beg, end));
            index2Point(beg, end, &tmp.front());
            fit_.fit(&tmp.front(), &tmp.back() + 1, l);
        }

        template<template<class> class LPT>
        inline void apply(typename std::vector<index_type>::const_iterator beg, typename std::vector<index_type>::const_iterator end, Line<float_type,LPT>& l, const cv::Mat& data) const {
            std::vector<point_type> tmp;
            tmp.resize(std::distance(beg, end));
            index2Point(beg, end, &tmp.front());
            fit_.fit(&tmp.front(), &tmp.back() + 1, l, data);
        }

        template<template<class> class LPT>
        inline void apply(typename std::vector<point_type>::const_iterator beg, typename std::vector<point_type>::const_iterator end, Line<float_type,LPT>& l) const {
            fit_.fit(&(*beg), &(*(end-1))+1, l);
        }

        template<template<class> class LPT>
        inline void apply(typename std::vector<point_type>::const_iterator beg, typename std::vector<point_type>::const_iterator end, Line<float_type, LPT>& l, const cv::Mat& data) const {
            fit_.fit(&(*beg), &(*(end - 1)) + 1, l, data);
        }


        template<class VEC, template<class> class LPT>
        inline void apply(const VEC& p, Line<float_type,LPT>& l) const {
            apply(&p[0], &p[0] + p.size(), l);
        }

        template<class VEC, template<class> class LPT>
        inline void apply(const VEC& p, Line<float_type,LPT>& l, const cv::Mat& data) const {
            apply(&p[0], &p[0] + p.size(), l, data);
        }

        template<template<class> class LPT>
        inline void apply(const EdgeSegmentVector& segments, const std::vector<point_type>& points, std::vector<Line<float_type,LPT>>& lines) const {
            lines.clear();
            lines.reserve(segments.size());
            for_each(segments.begin(), segments.end(), [&](const EdgeSegment &seg) {
                Line<float_type, LPT> l;
                fit_.fit(points.data() + seg.begin(), points.data() + seg.end(), l);
                lines.push_back(l);
            });
        }

        template<template<class> class LPT>
        inline void apply(const EdgeSegmentVector& segments, const std::vector<point_type>& points, std::vector<LineSegment<float_type, LPT>>& lines) const {
            lines.clear();
            lines.reserve(segments.size());
            for_each(segments.begin(), segments.end(), [&](const EdgeSegment &seg) {
                Line<float_type, LPT> l;
                fit_.fit(points.data() + seg.begin(), points.data() + seg.end(), l);
                const point_type &first = points[seg.first()];
                const point_type &last = points[seg.last()];
                lines.push_back(LineSegment<float_type, LPT>(l, first, last));
            });
        }

        //! get name of fit line operator
        virtual const std::string name() const { return FIT::name(); }
    };
    
    //! fit line implementation using M-Esitmater of opencv
    template<class FT, class PT>
    class MEstimatorFit {

    public:
        typedef FT float_type;
        typedef PT point_type;

        int dist;
        double param, reps, aeps;

        static std::string name() {
            return "M-Estimator";
        }
    
        MEstimatorFit(int d = CV_DIST_L2, double p = 0, double r = 0.001, double a = 0.001) :
        dist(d), param(p), reps(r), aeps(a) {}

        void fit(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny) const {
            cv::Vec<float, 4> res;
            std::vector<cv::Point> tmp;
            tmp.reserve(end - beg);
            for (const PT *i = beg; i != end; ++i)
                tmp.push_back(cv::Point(getX(*i), getY(*i)));
            cv::fitLine(cv::Mat(tmp), res, dist, param, reps, aeps);
            nx = -res[1];
            ny = res[0];
            cx = res[2];
            cy = res[3];
        }

        void fit(const PT *beg, const PT *end, FT &cx, FT &cy, FT &nx, FT &ny, const cv::Mat& data) const {
            fit(beg, end, cx, cy, nx, ny);
        }

        template<template<class> class LPT>
        void fit(const PT *beg, const PT *end, Line<FT, LPT>& l) const {
            cv::Vec<float, 4> res;
            std::vector<cv::Point> tmp;
            tmp.reserve(end - beg);
            for (const PT *i = beg; i != end; ++i)
                tmp.push_back(cv::Point(getX(*i), getY(*i)));
            cv::fitLine(cv::Mat(tmp), res, dist, param, reps, aeps);
            l = Line<FT, LPT>(-res[1], res[0], res[2], res[3]);
        }

        template<template<class> class LPT>
        void fit(const PT *beg, const PT *end, Line<FT, LPT>& l, const cv::Mat& data) const {
            fit(beg, end, l);
        }
    };
    
    //! M-Esitmater Point specialization
    template<class FT>
    class MEstimatorFit<FT, cv::Point> {

    public:
        int dist;
        double param, reps, aeps;

        typedef FT float_type;
        typedef cv::Point point_type;
    
        MEstimatorFit(int d = CV_DIST_L2, double p = 0, double r = 0.001, double a = 0.001) :
            dist(d), param(p), reps(r), aeps(a) {
        }

        static std::string name() {
            return "M-Estimator";
        }

        void fit(const cv::Point *beg, const cv::Point *end, FT &cx, FT &cy, FT &nx, FT &ny) const {
            cv::Vec<float, 4> res;
            cv::Mat tmp(end - beg, 2, CV_32SC1, const_cast<cv::Point*>(beg));
            cv::fitLine(tmp, res, dist, param, reps, aeps);
            nx = -res[1];
            ny = res[0];
            cx = res[2];
            cy = res[3];
        }

        void fit(const cv::Point *beg, const cv::Point *end, FT &cx, FT &cy, FT &nx, FT &ny, const cv::Mat& data) const {
            fit(beg, end, cx, cy, nx, ny);
        }

        template<template<class> class LPT>
        void fit(const cv::Point *beg, const cv::Point *end, Line<FT, LPT>& l) const {
            cv::Vec<float, 4> res;
            cv::Mat tmp(end - beg, 2, CV_32SC1, const_cast<cv::Point*>(beg));
            cv::fitLine(tmp, res, dist, param, reps, aeps);
            l = Line<FT, LPT>(-res[1], res[0], res[2], res[3]);
        }

        template<template<class> class LPT>
        void fit(const cv::Point *beg, const cv::Point *end, Line<FT, LPT>& l, const cv::Mat& data) const {
            fit(beg, end, l);
        }
    };

    //! fit line implementation using M-Esitmater of opencv
    template<class FT, class PT>
    class MEstimatorFitLine : public FitLine<MEstimatorFit<FT,PT>> {
        using FitLine<MEstimatorFit<FT, PT>>::fit_;

        void init() {
            fit_.dist = CV_DIST_L2;
            fit_.param = 0;
            fit_.reps = 0.001;
            fit_.aeps = 0.001;

            this->add("fit_distance", std::bind(&MEstimatorFitLine<FT, PT>::valueDist, this, std::placeholders::_1),
                "Distance used by the M-estimator.");

            this->add("fit_param", std::bind(&MEstimatorFitLine<FT, PT>::valueParam, this, std::placeholders::_1),
                "Numerical parameter for some types of distances. If it is 0, an optimal value is chosen.");

            this->add("fit_reps", std::bind(&MEstimatorFitLine<FT, PT>::valueREps, this, std::placeholders::_1),
                "Sufficient accuracy for the radius (distance between the coordinate origin and the line)");

            this->add("fit_aeps", std::bind(&MEstimatorFitLine<FT, PT>::valueAEps, this, std::placeholders::_1),
                "Sufficient accuracy for the angle.");
        }
    public:
        using FitLine<MEstimatorFit<FT, PT>>::apply;

        MEstimatorFitLine(int d = CV_DIST_L2, double p = 0, double r = 0.001, double a = 0.001) {
            init();

            CV_Assert(d > 0 && d < 8 && p >= 0 && r > 0 && a > 0);

            fit_.dist = d;
            fit_.param = p;
            fit_.reps = r;
            fit_.aeps = a;
        }

        MEstimatorFitLine(const ValueManager::NameValueVector &options) {
            init();
            this->value(options);
        }

        MEstimatorFitLine(ValueManager::InitializerList options) {
            init();
            this->value(options);
        }

        Value valueDist(const Value &d = Value::NAV()) { if (d.type()) distance(d.getInt()); return fit_.dist; }

        int distance() const { return fit_.dist; }

        void distance(int d) {
            if (d > 0 && d < 8)
                fit_.dist = d;
        }

        Value valueParam(const Value &p = Value::NAV()) { if (p.type()) param(p.getDouble()); return fit_.param; }

        double param() const { return fit_.param; }

        void param(double p) {
            fit_.param = p;
        }

        Value valueREps(const Value &reps = Value::NAV()) { if (reps.type()) rEps(reps.getDouble()); return fit_.reps; }

        double rEps() const { return fit_.reps; }

        void rEps(double reps) {
            if (reps > 0)
                fit_.reps = reps;
        }

        Value valueAEps(const Value &aeps = Value::NAV()) { if (aeps.type()) aEps(aeps.getDouble()); return fit_.aeps; }

        double aEps() const { return fit_.aeps; }

        void aEps(double aeps) {
            if (aeps > 0)
                fit_.aeps = aeps;
        }

    };
}
#endif
#endif
