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


#ifndef _GEOMETRY_STEREOCV_HPP_
#define _GEOMETRY_STEREOCV_HPP_
#ifdef __cplusplus

#include <geometry/stereo.hpp>
#include <geometry/cameracv.hpp>


namespace lsfm {

    //! traingulation methods implemented with opencv functions (based on point triangulation only)
    template<class FT>
    class StereoCV {

    protected:
        cv::Matx34<FT> projL_, projR_;

    public:

        typedef FT float_type;

        StereoCV(const Matx34<FT> &projL, const Matx34<FT> &projR) : projL_(projL.data()), projR_(projR.data())  {}

        StereoCV(const Camera<FT> &camL, const Camera<FT> &camR) : projL_(camL.projM().data()), projR_(camR.projM().data())  {}

        //! compute 3d point from two rays from camera origin through corresponding stereo pixels
        inline Vec3<FT> triangulate(const Vec2<FT> &pointL, const Vec2<FT> &pointR) const {
            cv::Mat_<FT> v;
            std::vector<Vec2<FT>> camLpnts, camRpnts;
            camLpnts.reserve(1);
            camRpnts.reserve(1);

            camLpnts.push_back(pointL);
            camRpnts.push_back(pointR);

            triangulatePoints(projL_, projR_, cv::Mat(camLpnts), cv::Mat(camRpnts), v);

            if (detail::abs(v(3)) < LIMITS<FT>::tau())
                return Vec3<FT>(0,0,0);
            v /= v(3);
            return Vec3<FT>(v(0),v(1),v(2));
        }

        //! compute 3d points from point vector
        template <template<class, class...> class V, class... V1Args, class... V2Args>
        inline void triangulate(const V<Vec2<FT>,V1Args...> &pointL, const V<Vec2<FT>,V1Args...> &pointR, V<Vec3<FT>,V2Args...>& ret) const {
            CV_Assert(pointL.size() == pointR.size());
            cv::Mat_<FT> v;
            triangulatePoints(projL_, projR_, cv::Mat(pointL), cv::Mat(pointR), v);

            ret.resize(pointL.size());
            cv::Mat tmp(ret);
            cv::fromHomogeneous<FT>(v,tmp,true);
        }

        //! compute 3d points from mat
        /*inline cv::Mat triangulate(const cv::Mat &pointL, const cv::Mat &pointR) {
            CV_Assert(pointL.rows == pointR.rows && pointL.type() == pointR.type());
            cv::Mat_<FT> v, ret;
            triangulatePoints(projL_, projR_, pointL, pointR, v);

            fromHomogeneous<FT>(v,ret,true);
            return ret;
        }*/

        //! compute 3d line from two planes from camera origin through corresponding stereo lines
        inline Line3<FT> triangulate(const Line<FT> &lineL, const Line<FT> &lineR) const {
            if (detail::abs(lineL.normalX()) < LIMITS<FT>::tau() || detail::abs(lineR.normalX()) < LIMITS<FT>::tau())
                return Line3<FT>();

            cv::Mat_<FT> points4Dresult;
            std::vector<Vec2<FT>> camLpnts, camRpnts;
            camLpnts.reserve(2);
            camRpnts.reserve(2);

            camLpnts.push_back(Vec2<FT>(lineL.x()(0), 0));
            camLpnts.push_back(Vec2<FT>(lineL.x()(100), 100));
            camRpnts.push_back(Vec2<FT>(lineR.x()(0), 0));
            camRpnts.push_back(Vec2<FT>(lineR.x()(100), 100));

            triangulatePoints(projL_, projR_, cv::Mat(camLpnts), cv::Mat(camRpnts), points4Dresult);


            cv::Mat_<FT> v1 = points4Dresult.col(0), v2 = points4Dresult.col(1);
            if (detail::abs(v1(3)) < LIMITS<FT>::tau() || detail::abs(v2(3)) < LIMITS<FT>::tau())
                return Line3<FT>();
            v1 /= v1(3);
            v2 /= v2(3);
            return Line3<FT>(Vec3<FT>(v1(0), v1(1), v1(2)),Vec3<FT>(v2(0) - v1(0), v2(1) - v1(1), v2(2) - v1(2)));
        }

        //! compute 2d line pairs to 3d line vector
        template <template<class, class...> class V, class... V1Args, class... V2Args>
        inline void triangulate(const V<Line<FT>,V1Args...> &linesL, const V<Line<FT>,V1Args...> &linesR, V<Line3<FT>,V2Args...>& ret) const {
            CV_Assert(linesL.size() == linesR.size());

            size_t size = linesL.size();
            std::vector<Vec3<FT>> vp3;
            std::vector<Vec2<FT>> camLpnts, camRpnts;
            camLpnts.reserve(size * 2);
            camRpnts.reserve(size * 2);

            for (size_t i = 0; i != size; ++i) {
                if (detail::abs(linesL[i].normalX()) < LIMITS<FT>::tau() || detail::abs(linesR[i].normalX()) < LIMITS<FT>::tau()) {
                    camLpnts.push_back(Vec2<FT>(0,0));
                    camLpnts.push_back(Vec2<FT>(0,0));
                    camRpnts.push_back(Vec2<FT>(0,0));
                    camRpnts.push_back(Vec2<FT>(0,0));
                    continue;
                }


                camLpnts.push_back(Vec2<FT>(linesL[i].x(0), 0));
                camLpnts.push_back(Vec2<FT>(linesL[i].x(100), 100));
                camRpnts.push_back(Vec2<FT>(linesR[i].x(0), 0));
                camRpnts.push_back(Vec2<FT>(linesR[i].x(100), 100));
            }

            triangulate(camLpnts,camRpnts,vp3);
            ret.clear();
            ret.reserve(size);

            size = vp3.size();
            for (size_t i = 0; i != size; i += 2) {
                ret.push_back(Line3<FT>::twoPoint(vp3[i],vp3[i+1]));
            }
        }


        //! compute 2d line (subclass) pairs to 3d line vector
        template<class LV>
        inline void triangulateV(const LV &linesL, const LV &linesR, std::vector<Line3<FT>>& ret) const {
            CV_Assert(linesL.size() == linesR.size());

            size_t size = linesL.size();
            std::vector<Vec3<FT>> vp3;
            std::vector<Vec2<FT>> camLpnts, camRpnts;
            camLpnts.reserve(size * 2);
            camRpnts.reserve(size * 2);

            for (size_t i = 0; i != size; ++i) {
                if (detail::abs(linesL[i].normalX()) < LIMITS<FT>::tau() || detail::abs(linesR[i].normalX()) < LIMITS<FT>::tau()) {
                    camLpnts.push_back(Vec2<FT>(0,0));
                    camLpnts.push_back(Vec2<FT>(0,0));
                    camRpnts.push_back(Vec2<FT>(0,0));
                    camRpnts.push_back(Vec2<FT>(0,0));
                    continue;
                }


                camLpnts.push_back(Vec2<FT>(linesL[i].x(0), 0));
                camLpnts.push_back(Vec2<FT>(linesL[i].x(100), 100));
                camRpnts.push_back(Vec2<FT>(linesR[i].x(0), 0));
                camRpnts.push_back(Vec2<FT>(linesR[i].x(100), 100));
            }

            triangulate(camLpnts,camRpnts,vp3);
            ret.clear();
            ret.reserve(size);

            size = vp3.size();
            for (size_t i = 0; i != size; i += 2) {
                ret.push_back(Line3<FT>::twoPoint(vp3[i],vp3[i+1]));
            }
        }


        //! compute 3d line segment from two planes from camera origin through corresponding stereo line segments
        inline LineSegment3<FT> triangulate(const LineSegment<FT> &lineL, const LineSegment<FT> &lineR) const {
            if (detail::abs(lineL.normalX()) < LIMITS<FT>::tau() || detail::abs(lineR.normalX()) < LIMITS<FT>::tau())
                    return LineSegment3<FT>();

            Vec2<FT> lstart = lineL.startPoint(), lend = lineL.endPoint(),
                       rstart = lineR.startPoint(), rend = lineR.endPoint();

            FT starty = std::min(std::min(std::min(lstart.y(), lend.y()), rstart.y()), rend.y());
            FT endy = std::max(std::max(std::max(lstart.y(), lend.y()), rstart.y()), rend.y());

            if (lstart.y() > lend.y())
                std::swap(starty, endy);


            cv::Mat_<FT> points4Dresult;
            std::vector<Vec2<FT>> camLpnts, camRpnts;
            camLpnts.reserve(2);
            camRpnts.reserve(2);

            camLpnts.push_back(Vec2<FT>(lineL.x(starty), starty));
            camLpnts.push_back(Vec2<FT>(lineL.x(endy), endy));
            camRpnts.push_back(Vec2<FT>(lineR.x(starty), starty));
            camRpnts.push_back(Vec2<FT>(lineR.x(endy), endy));

            triangulatePoints(projL_, projR_, camLpnts, camRpnts, points4Dresult);


            cv::Mat_<FT> v1 = points4Dresult.col(0), v2 = points4Dresult.col(1);
            if (detail::abs(v1(3)) < LIMITS<FT>::tau() || detail::abs(v2(3)) < LIMITS<FT>::tau())
                return LineSegment3<FT>();
            v1 /= v1(3);
            v2 /= v2(3);
            return LineSegment3<FT>(Vec3<FT>(v1(0), v1(1), v1(2)),Vec3<FT>(v2(0) - v1(0), v2(1) - v1(1), v2(2) - v1(2)));
        }



        //! compute 2d line segment pairs to 3d line segment vector
        template <template<class, class...> class V, class... V1Args, class... V2Args>
        inline void triangulate(const V<LineSegment<FT>,V1Args...> &linesL, const V<LineSegment<FT>,V1Args...> &linesR, V<LineSegment3<FT>,V2Args...>& ret) const {
            CV_Assert(linesL.size() == linesR.size());

            size_t size = linesL.size();
            std::vector<Vec3<FT>> vp3;
            std::vector<Vec2<FT>> camLpnts, camRpnts;
            camLpnts.reserve(size * 2);
            camRpnts.reserve(size * 2);

            for (size_t i = 0; i != size; ++i) {
                if (detail::abs(linesL[i].normalX()) < LIMITS<FT>::tau() || detail::abs(linesR[i].normalX()) < LIMITS<FT>::tau()) {
                    camLpnts.push_back(Vec2<FT>(0,0));
                    camLpnts.push_back(Vec2<FT>(0,0));
                    camRpnts.push_back(Vec2<FT>(0,0));
                    camRpnts.push_back(Vec2<FT>(0,0));
                    continue;
                }

                Vec2<FT> lstart = linesL[i].startPoint(), lend = linesL[i].endPoint(),
                           rstart = linesR[i].startPoint(), rend = linesR[i].endPoint();

                FT starty = std::min(std::min(std::min(lstart.y(), lend.y()), rstart.y()), rend.y());
                FT endy = std::max(std::max(std::max(lstart.y(), lend.y()), rstart.y()), rend.y());

                if (lstart.y() > lend.y())
                    std::swap(starty, endy);


                camLpnts.push_back(Vec2<FT>(linesL[i].x(starty), starty));
                camLpnts.push_back(Vec2<FT>(linesL[i].x(endy), endy));
                camRpnts.push_back(Vec2<FT>(linesR[i].x(starty), starty));
                camRpnts.push_back(Vec2<FT>(linesR[i].x(endy), endy));
            }

            triangulate(camLpnts,camRpnts,vp3);
            ret.clear();
            ret.reserve(size);

            size = vp3.size();
            for (size_t i = 0; i != size; i += 2) {
                ret.push_back(LineSegment3<FT>(vp3[i],vp3[i+1]));
            }
        }


        //! compute 2d line segment (subclass) pairs to 3d line segment vector
        template<class LV>
        inline void triangulateV(const LV &linesL, const LV &linesR, std::vector<LineSegment3<FT>>& ret) const {
            CV_Assert(linesL.size() == linesR.size());

            size_t size = linesL.size();
            std::vector<Vec3<FT>> vp3;
            std::vector<Vec2<FT>> camLpnts, camRpnts;
            camLpnts.reserve(size * 2);
            camRpnts.reserve(size * 2);

            for (size_t i = 0; i != size; ++i) {
                if (detail::abs(linesL[i].normalX()) < LIMITS<FT>::tau() || detail::abs(linesR[i].normalX()) < LIMITS<FT>::tau()) {
                    camLpnts.push_back(Vec2<FT>(0,0));
                    camLpnts.push_back(Vec2<FT>(0,0));
                    camRpnts.push_back(Vec2<FT>(0,0));
                    camRpnts.push_back(Vec2<FT>(0,0));
                    continue;
                }

                Vec2<FT> lstart = linesL[i].startPoint(), lend = linesL[i].endPoint(),
                           rstart = linesR[i].startPoint(), rend = linesR[i].endPoint();

                FT starty = std::min(std::min(std::min(lstart.y(), lend.y()), rstart.y()), rend.y());
                FT endy = std::max(std::max(std::max(lstart.y(), lend.y()), rstart.y()), rend.y());

                if (lstart.y() > lend.y())
                    std::swap(starty, endy);


                camLpnts.push_back(Vec2<FT>(linesL[i].x(starty), starty));
                camLpnts.push_back(Vec2<FT>(linesL[i].x(endy), endy));
                camRpnts.push_back(Vec2<FT>(linesR[i].x(starty), starty));
                camRpnts.push_back(Vec2<FT>(linesR[i].x(endy), endy));
            }

            triangulate(camLpnts,camRpnts,vp3);
            ret.clear();
            ret.reserve(size);

            size = vp3.size();
            for (size_t i = 0; i != size; i += 2) {
                ret.push_back(LineSegment<FT>(vp3[i],vp3[i+1]));
            }
        }
    };

    typedef StereoCV<float> StereoCVf;
    typedef StereoCV<double> StereoCVd;

}
#endif
#endif
