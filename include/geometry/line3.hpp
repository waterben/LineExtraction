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


#ifndef _GEOMETRY_LINE3_HPP_
#define _GEOMETRY_LINE3_HPP_
#ifdef __cplusplus

#include "../geometry/line.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>


namespace lsfm {
   
    /**
     * Line3 object
     */
    template<class FT>
    class Line3 {
    protected:
        // line internals
        Vec3<FT> p_, v_;

    public:
        typedef FT float_type;

        Line3() : p_(FT(0),FT(0),FT(0)), v_(FT(0),FT(0),FT(0)) {}

        //! Init Line by Point and direction Vector (without normalization)
        Line3(const Vec3<FT> &pnt, const Vec3<FT> &vec, bool) :
            p_(pnt), v_(vec) {}

        //! Init Line by Point and direction Vector
        Line3(const Vec3<FT> &pnt, const Vec3<FT> &vec) :
            p_(pnt), v_(vec) {
            // normalize direction
            FT n = detail::sqrt(v_.dot(v_));
            if (n < LIMITS<FT>::tau()) {
                v_ = Vec3<FT>(FT(0),FT(0),FT(0));
                p_ = v_;
                return;
            }
            v_ /= n;
        }

        template< class newFT >
        Line3<newFT> convertTo(){
            return Line3<newFT>(Vec3<newFT>(static_cast<newFT>(p_[0]), static_cast<newFT>(p_[1]), static_cast<newFT>(p_[2])), Vec3<newFT>(static_cast<newFT>(v_[0]), static_cast<newFT>(v_[1]), static_cast<newFT>(v_[2])));
        }

        static Line3<FT> twoPoint(const Vec3<FT> &beg, const Vec3<FT> &end) {
            return Line3<FT>(beg,end-beg);
        }

        bool valid() const {
            return detail::abs(v_.dot(v_) - 1) <= LIMITS<FT>::tau();
        }

        bool empty() const {
            return v_.dot(v_) == FT(0);
        }

        //! get line direction vector
        inline const Vec3<FT>& direction() const {
            return v_;
        }

        //! get line origin
        inline const Vec3<FT>& origin() const {
            return p_;
        }

        //! momentum for Pluecker
        inline Vec3<FT> momentum() const{
            return p_.cross(v_);
        }

        //! compute shortest distance between plane defined by normal from
        //! line direction and start point of line and a Point
        inline FT normalDistance(const Vec3<FT>& p) const {
            return v_.dot(p - p_);
        }

        //! compute shortest distance between plane defined by normal from
        //! line direction and start point of line and a line
        inline FT normalDistance(const Line3<FT>& l) const {
             Vec3<FT> w = p_ - l.p_;
             FT b = v_.dot(l.v_);
             FT D = 1 - b*b;

             return D > LIMITS<FT>::tau() ? (b*l.v_.dot(w) - v_.dot(w)) / D : 0;
        }

        // see http://geomalgorithms.com/a02-_lines.html
        //! compute projected point on line by shortest distance
        inline Vec3<FT> nearestPointOnLine(const Vec3<FT>& p) const {
            return p_ + v_ * normalDistance(p);
        }

        // see http://geomalgorithms.com/a07-_distance.html
        //! compute projected point on this line by shortest distance between lines, return "distance" on line
        inline Vec3<FT> nearestPointOnLine(const Line3<FT>& l, FT & sc) const {
             Vec3<FT> w = p_ - l.p_;
             FT b = v_.dot(l.v_);
             FT D = 1 - b*b;
             //FT sc = 0;

             if (D > LIMITS<FT>::tau()) {
                sc = (b*l.v_.dot(w) - v_.dot(w)) / D;
             }

             return p_ + (v_ * sc);
        }

        //! compute projected point on this line by shortest distance between lines, don't return "distance" on line
        inline Vec3<FT> nearestPointOnLine(const Line3<FT>& l) const {
             FT sc = 0;
             return nearestPointOnLine(l, sc);
        }

        //! compute projected points on lines by shortest distance between lines
        inline void nearestPointOnLine(const Line3<FT>& l, Vec3<FT>& p1, Vec3<FT>& p2) const {
             Vec3<FT> w = p_ - l.p_;
             FT b = v_.dot(l.v_);
             FT e = l.v_.dot(w);
             FT D = 1 - b*b;
             FT sc = 0, tc = e;

             if (D > LIMITS<FT>::tau()) {
                FT d = v_.dot(w);
                sc = (b*e - d) / D;
                tc = (e - b*d) / D;
             }

             p1 = p_ + (sc * v_);
             p2 = l.p_ + (tc * l.v_);
        }

        //! compute point with distance d along the line (in respect to coord-origin)
        inline Vec3<FT> distance(FT d) const {
            return v_*d;
        }

        //! compute point with distance d along the line in respect to given point
        inline Vec3<FT> distance(FT d, const Vec3<FT>& p) const {
            return p + v_*d;
        }

        //! compute point with distance d along the line in respect to line origin
        inline Vec3<FT> distanceOrigin(FT d) const {
            return distance(d,p_);
        }


        //! compute shortest distance between Line and Point
        inline FT distance(const Vec3<FT>& p) const {
            Vec3<FT> d = p - nearestPointOnLine(p) ;
            return detail::sqrt(d.dot(d));
        }


        //! compute shortest distance between two lines
        inline FT distance(const Line3<FT>& l) const {
             Vec3<FT> w = p_ - l.p_;
             FT b = v_.dot(l.v_);
             FT e = l.v_.dot(w);
             FT D = FT(1) - b*b;
             FT sc = FT(0), tc = e;

             if (D > LIMITS<FT>::tau()) {
                FT d = v_.dot(w);
                sc = (b*e - d) / D;
                tc = (e - b*d) / D;
             }

             Vec3<FT> dP = w + (sc * v_) - (tc * l.v_);
             return detail::sqrt(dP.dot(dP));
        }

        //! rotate around start point
        inline Line3<FT>& rotateDirection(const Matx33<FT>& rotMat) {
            v_ = rotMat * v_;
            return *this;
        }

        //! rotate around start point
        inline Line3<FT>& rotateDirection(const Vec3<FT>& rot) {
            return rotate(rodrigues(rot));
        }

        //! rotate around origin
        inline Line3<FT>& rotate(const Matx33<FT>& rotMat) {
            p_ = rotMat * p_;
            v_ = rotMat * v_;
            return *this;
        }

        //! rotate around origin
        inline Line3<FT>& rotate(const Vec3<FT>& rot) {
            return rotate(rodrigues(rot));
        }

        //! rotate around given pivot
        inline Line3<FT>& rotate(const Matx33<FT>& rotMat, const Vec3<FT>& pivot) {
            p_ = (rotMat * (p_ - pivot)) + pivot;
            v_ = rotMat * v_;
            return *this;
        }

        //! rotate around given pivot
        inline Line3<FT>& rotate(const Vec3<FT>& rot, const Vec3<FT>& pivot) {
            return rotate(rodrigues(rot));
        }

        //! translate line
        inline Line3<FT>& translate(const Vec3<FT>& trans) {
            p_ += trans;
            return *this;
        }

        //! compute angle between two line3 in radians
        inline FT angle(const Line3<FT> &line3) const {
            return detail::acos(v_.dot(line3.v_));
        }

        //! compute normal from two lines
        inline Vec3<FT> normal(Line3<FT> &line3) const {
            return v_.cross(line3.v_);
        }

        //! flip direction
        virtual void flip() {
            v_ *= FT(-1);
        }

        //! cayley coordinates
        inline void cayley(FT &w, Vec3<FT> &s) const {
            cayleyRepresentationFromPluecker(momentum(), v_, w, s);
        }

        inline Vec4<FT> cayley() const {
            Vec4<FT> ret;
            cayleyRepresentationFromPluecker(momentum(), v_, ret[0], *reinterpret_cast<Vec3<FT>*>(&ret[1]));
            return ret;
        }

        //! get Cayley representation from pluecker coordinates line
        static void cayleyRepresentationFromPluecker(const Vec3<FT> &m, const Vec3<FT> &l, Vec4<FT> &c){
            cayleyRepresentationFromPluecker(m,l,c[0],*reinterpret_cast<Vec3<FT>*>(&c[1]));
        }

        template<typename T, typename _Matrix_Type_>
        static _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, T epsilon = lsfm::LIMITS<FT>::tau())
        {
            Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
            T tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
            return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
        }


        //! get Cayley representation from pluecker coordinates line
        static void cayleyRepresentationFromPluecker(const Vec3<FT> &m, const Vec3<FT> &l, FT &w, Vec3<FT> &s){
             static const Matx33<FT> I = Matx33<FT>::Identity();

            // L2 norm
            w = detail::sqrt(m.dot(m));

            Matx33<FT> Q;
            Q(0,0) = l[0];
            Q(1,0) = l[1];
            Q(2,0) = l[2];

            if(w <= LIMITS<FT>::tau()){
                Vec3<FT> e1, e2, tmp;

                // create orthogonal base
                tmp[0] = -l[1];    // shift and negate
                tmp[1] = l[2];
                tmp[2] = l[0];
                e1 = l.cross(tmp);
                e2 = l.cross(e1);

                Q(0,1) = e1[0];
                Q(1,1) = e1[1];
                Q(2,1) = e1[2];

                Q(0,2) = e2[0];
                Q(1,2) = e2[1];
                Q(2,2) = e2[2];
                std::cout << "shift and negate reached!" << std::endl;

            } else {

                Vec3<FT> lXm = l.cross(m);
                FT lXmNorm = detail::sqrt(lXm.dot(lXm));

                Q(0,1) = m[0] / w;
                Q(1,1) = m[1] / w;
                Q(2,1) = m[2] / w;

                /*if (lXmNorm < LIMITS<FT>::tau()) {
                    // l is 0 -> set s to 0 and return to prevent div by zero
                    s[0] = 0;
                    s[1] = 0;
                    s[2] = 0;
                    return;
                } else {*/
                    Q(0,2) = lXm[0] / lXmNorm;
                    Q(1,2) = lXm[1] / lXmNorm;
                    Q(2,2) = lXm[2] / lXmNorm;
                //}
            }

            Matx33<FT> sx = (Q - I)*((Q + I).inverse());

            if(!(sx.allFinite())){  // happens only very rarely
                Eigen::Matrix<FT,Eigen::Dynamic, Eigen::Dynamic> QpI = (Q + I);
                Eigen::Matrix<FT,Eigen::Dynamic, Eigen::Dynamic> QpIPseudoInv = pseudoInverse<FT>(QpI);

                sx = (Q - I)*(QpIPseudoInv);
                w = w * -1;
            }

            s[0] = sx(2,1);
            s[1] = sx(0,2);
            s[2] = sx(1,0);
        }

        //! Pluecker Coordinates from Cayley Representation
        inline static void plueckerCoordinatesFromCayley(const Vec4<FT> &c, Vec3<FT> &m, Vec3<FT> &l){
            plueckerCoordinatesFromCayley(c[0],*reinterpret_cast<const Vec3<FT>*>(&c[1]),m,l);
        }

        //! Pluecker Coordinates from Cayley Representation
        inline static void plueckerCoordinatesFromCayley(const FT w, const Vec3<FT> &s, Vec3<FT> &m, Vec3<FT> &l){

            static const Matx33<FT> I = Matx33<FT>::Identity();

            Matx33<FT> Q, sx = Matx33<FT>::Zero();
            Vec3<FT> sv(s);
            
            FT sNormSquared = s.dot(s);

            sx(2,1) = s[0];
            sx(0,2) = s[1];
            sx(1,0) = s[2];

            sx(1,2) = -s[0];
            sx(2,0) = -s[1];
            sx(0,1) = -s[2];

            Q = ((FT(1) - sNormSquared) * I + FT(2) * sx + FT(2)*sv*sv.transpose()) / (FT(1)+sNormSquared);

            l[0] = Q(0,0);
            l[1] = Q(1,0);
            l[2] = Q(2,0);

            m[0] = Q(0,1) * w;
            m[1] = Q(1,1) * w;
            m[2] = Q(2,1) * w;

        }

        //! Create 3dLine from Pluecker coordinates
        static Line3<FT> lineFromPluecker(Vec3<FT> &m, Vec3<FT> &l) {
            return Line3<FT>(l.cross(m),l, true);
        }

        //! Create 3dLine from cayley coordinates
        static Line3<FT> lineFromCayley(const Vec4<FT> &c) {
            return lineFromCayley(c[0],*reinterpret_cast<const Vec3<FT>*>(&c[1]));
        }

        //! Create 3dLine from cayley coordinates
        static Line3<FT> lineFromCayley(FT w, const Vec3<FT> &s) {
            Vec3<FT> m, l;
            plueckerCoordinatesFromCayley(w, s, m, l);
            return lineFromPluecker(m,l);
        }

    };


    typedef Line3<float> Line3f;
    typedef Line3<double> Line3d;

    /**
    * Line3 segement object
    */
    template<class FT>
    class LineSegment3 : public Line3<FT> {
    protected:
        using Line3<FT>::p_;
        using Line3<FT>::v_;
        
        // beg / end of line as distances to starting point
        FT beg_, end_;

        void swapDir() {
            Line3<FT>::flip();
            beg_ *= -1;
            end_ *= -1;
        }

    public:
        typedef FT float_type;

        LineSegment3() :
            Line3<FT>(), beg_(0), end_(0) {}

        //! Init line segment by point, direction and begin and end of line
        LineSegment3(const Vec3<FT> &p, const Vec3<FT> &dir, FT beg, FT end) :
            Line3<FT>(p,dir), beg_(beg), end_(end) {
            if (beg > end)
                swapDir();
        }

        //! Init line segment by line begin and line end
        LineSegment3(const Vec3<FT> &line_begin, const Vec3<FT> &line_end) : Line3<FT>(), beg_(0), end_(0){

            //Eigen::DenseBase<FT,3,1> test = line_end - line_begin;
            //
            Vec3<FT> dir = Matx<FT,3,1>(line_end - line_begin);
            //Vec3<FT> dir = line_end - line_begin;
            FT n = detail::sqrt(dir.dot(dir));
            if (n < LIMITS<FT>::tau())
                return;

            p_ = line_begin;
            v_ = dir * (1.0 / n);

            end_ = v_.dot(dir);
        }

        //! Init line segment by point, direction and two points that are projected on line
        LineSegment3(const Vec3<FT> &p, const Vec3<FT> &dir, const Vec3<FT> &beg, const Vec3<FT> &end) :
            Line3<FT>(p,dir) {
            beg_ = normalDistance(beg);
            end_ = normalDistance(end);
            if (beg > end)
                swapDir();
        }

        //! Init line segment by line and two distances on line
        LineSegment3(const Line3<FT> &l, FT beg, FT end) :
            Line3<FT>(l), beg_(beg), end_(end) {
            if (beg > end)
                swapDir();
        }

        //! Init line segment by line and two points that are projected on line
        LineSegment3(const Line3<FT> &l, const Vec3<FT> &beg, const Vec3<FT> &end) :
            Line3<FT>(l) {
            beg_ = normalDistance(beg);
            end_ = normalDistance(end);
            if (beg > end)
                swapDir();
        }

        template< class newFT >
        LineSegment3<newFT> convertTo(){
            return LineSegment3<newFT>(Vec3<newFT>(static_cast<newFT>(p_[0]), static_cast<newFT>(p_[1]), static_cast<newFT>(p_[2])), Vec3<newFT>(static_cast<newFT>(v_[0]), static_cast<newFT>(v_[1]), static_cast<newFT>(v_[2])), static_cast<newFT>(beg_), static_cast<newFT>(end_) );
        }

        Vec3<FT> startPoint() const {
            return Vec3<FT>(p_ + v_ * beg_);
        }

        cv::Mat startPoint_cv(int type) {
            Vec3<FT> sp(p_ + v_ * beg_);
            cv::Mat m(3,1,type);
            m.at<FT>(0) = sp[0]; m.at<FT>(1) = sp[1]; m.at<FT>(2) = sp[2];
            return m;
        }

        Vec3<FT> endPoint() const {
            return Vec3<FT>(p_ + v_ * end_);
        }

        cv::Mat endPoint_cv(int type) {
            Vec3<FT> ep(p_ + v_ * end_);
            cv::Mat m(3,1,type);
            m.at<FT>(0) = ep[0]; m.at<FT>(1) = ep[1]; m.at<FT>(2) = ep[2];
            return m;
        }

        Vec3<FT> centerPoint() const {
            return Vec3<FT>(p_ + v_ * beg_ + ((end_ - beg_) / 2) * v_);
        }

        cv::Mat centerPoint_cv(int type) {
            Vec3<FT> cp(p_ + v_ * beg_ + ((end_ - beg_) / 2) * v_);
            cv::Mat m(3,1,type);
            m.at<FT>(0) = cp[0]; m.at<FT>(1) = cp[1]; m.at<FT>(2) = cp[2];
            return m;
        }

        //! get length of line
        inline FT length() const {
            return detail::abs(end_ - beg_);
        }

        //! get internal beg_ value
        FT getBegin(){
            return beg_;
        }
        //! get internal end_ value
        FT getEnd(){
            return end_;
        }

        // see http://geomalgorithms.com/a07-_distance.html
        //! compute projected point on this line by shortest distance between lines, while inside segment, otherwise shortest distance to endpoint (similar to nearestPointOnLine)
        inline Vec3<FT> nearestPointOnLineSegment(const Line3<FT>& l) const {
            FT a = beg_, b = end_, sc = 0;
            this->nearestPointOnLine(l, sc);

             if(a > b)
                 std::swap(a,b);
             if(sc < a)
                 sc = a;
             else if(sc > b)
                 sc = b;

             return p_ + (v_ * sc);
        }

        //! swap endpoints
        inline void endPointSwap() {
            std::swap(beg_, end_);
        }

        //! flip direction
        virtual void flip() {
            Line3<FT>::flip();
            beg_ *= FT(-1);
            end_ *= FT(-1);
            std::swap(beg_, end_);
        }

        /**
        * @brief  Error calculated by Squared Endpoint Distance
        * @param  other line (ground truth)
        * @return squared error distance
        */
        inline FT error(const Line3<FT> &gtLine) const {
            if (this->empty() || gtLine.empty())
                return FT(0);
            FT ds = gtLine.distance(this->startPoint());
            FT de = gtLine.distance(this->endPoint());
            return (ds * ds + de * de);
        }

        template<class U>
        friend std::ostream & operator<<(std::ostream &os, const LineSegment3<U>& ls3);

    };

    template<class FT>
    std::ostream & operator<<(std::ostream &os, const LineSegment3<FT>& ls3)
    {
        return os << "S: " << ls3.startPoint()[0] << ", " << ls3.startPoint()[1] << ", " << ls3.startPoint()[2] << "  E: " << ls3.endPoint()[0] << ", " << ls3.endPoint()[1] << ", " << ls3.endPoint()[2] << ", " ;
    }

    typedef LineSegment3<float> LineSegment3f;
    typedef LineSegment3<double> LineSegment3d;

}
#endif
#endif
