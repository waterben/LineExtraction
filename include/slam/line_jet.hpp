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

#ifndef _LINE_JET_HPP_
#define _LINE_JET_HPP_
#ifdef __cplusplus

#include "../geometry/line3.hpp"
#include <ceres/rotation.h>
#include <ceres/ceres.h>
#include <ceres/jet.h>

namespace lsfm {
    template <typename T, int N>
    struct LIMITS<ceres::Jet<T,N>> {
        static inline ceres::Jet<T,N> tau() { return ceres::Jet<T,N>(1.0E-7); }
        static inline ceres::Jet<T,N> min() { return ceres::Jet<T,N>(std::numeric_limits<T>::min()); }
        static inline ceres::Jet<T,N> eps() { return ceres::Jet<T,N>(std::numeric_limits<T>::epsilon()); }
    };

    //! Check for Nan and Inf
    static bool checkNanInf(double dbl){
        if(std::isnan(dbl)){
            std::cout << " nan problem with standard double! " << std::endl;
            return true;
        }
        return false;
    }

    template<class T>
    static bool checkNanInf(T jet){
        if(std::isnan(jet.a)){
            std::cout << " nan problem! " << std::endl;
            return true;
        }

        if(std::isinf(jet.a)){
            std::cout << " inf problem! " << std::endl;
            return true;
        }

        for(int i = 0; i < jet.v.size(); ++i){
            if(std::isnan(jet.v(i,0))){
                std::cout << " na problem! " << std::endl;
                return true;
            }
            if(std::isinf(jet.v(i,0))){
                std::cout << " i problem! " << std::endl;
                return true;
            }
        }
        return false;
    }

    template<class jetT, class FT, int rows, int cols>
    Matx<jetT,rows,cols> convert2Jet(const Matx<FT,rows,cols> &m) {
        Matx<jetT,rows,cols> ret;
        for (int i = 0; i != m.SizeAtCompileTime; ++i)
            ret[i] = jetT(m[i]);
        return ret;
    }

    template<class jetT, class FT>
    Matx33<jetT> convert2Jet(const Matx33<FT> &m) {
        return Matx33<jetT>(jetT(m[0]),jetT(m[1]),jetT(m[2]),
                            jetT(m[3]),jetT(m[4]),jetT(m[5]),
                            jetT(m[6]),jetT(m[7]),jetT(m[8]));
    }

    template<class jetT, class FT>
    Vec3<jetT> convert2Jet(const Vec3<FT> &m) {
        return Vec3<jetT>(jetT(m[0]),jetT(m[1]),jetT(m[2]));
    }

    template<class jetT, class FT>
    Vec2<jetT> convert2Jet(const Vec2<FT> &m) {
        return Vec2<jetT>(jetT(m[0]),jetT(m[1]));
    }

    template<class jetT, class FT, int rows, int cols>
    void convert2Jet(const Matx<FT,rows,cols> &m, Matx<jetT,rows,cols> &ret) {
        for (int i = 0; i != m.SizeAtCompileTime; ++i)
            ret[i] = jetT(m[i]);
    }

    template<class jetT, class FT>
    Matx33<jetT> convert2Jet(const Matx33<FT> &m, Matx33<FT> &ret) {
        ret = Matx33<jetT>(jetT(m[0]),jetT(m[1]),jetT(m[2]),
                            jetT(m[3]),jetT(m[4]),jetT(m[5]),
                            jetT(m[6]),jetT(m[7]),jetT(m[8]));
    }

    template<class jetT, class FT>
    void convert2Jet(const Vec3<FT> &m, Vec3<FT> &ret) {
        ret = Vec3<jetT>(jetT(m[0]),jetT(m[1]),jetT(m[2]));
    }

    template<class jetT, class FT>
    void convert2Jet(const Vec2<FT> &m, Vec2<FT> &ret) {
        ret = Vec2<jetT>(jetT(m[0]),jetT(m[1]));
    }


    /**
    * @brief  Error calculated by Squared Endpoint Distance
    * @param  other line (ground truth)
    * @return left and right error distances
    */
    template<class jetT>
    inline void errorSED(const jetT modelLine[3], const jetT ps[3], const jetT pe[3], jetT res[2]){
        jetT xyNorm = ceres::sqrt(modelLine[0] * modelLine[0] + modelLine[1] * modelLine[1]);

        jetT dotPs = ps[0] * modelLine[0] + ps[1] * modelLine[1] + ps[2] * modelLine[2];
        jetT dotPe = pe[0] * modelLine[0] + pe[1] * modelLine[1] + pe[2] * modelLine[2];

        res[0] = dotPs / xyNorm;
        res[1] = dotPe / xyNorm;

        /*if(checkNanInf(res[0]) || checkNanInf(res[1])){

            res[0] = dotPs;
            res[1] = dotPe;

            std::cout << " xyNorm: " << xyNorm << std::endl;
            std::cout << " dotPs: " << dotPs << std::endl;
            std::cout << " dotPe: " << dotPe << std::endl;
            std::cout << " modelLine[0]: " << modelLine[0] << std::endl << std::endl;
            std::cout << " modelLine[1]: " << modelLine[1]  << std::endl << std::endl;
            std::cout << " modelLine[2]: " << modelLine[2]  << std::endl << std::endl << std::endl;
//            std::cout << " modelLine[0]  * modelLine[0]: " << modelLine[0]  * modelLine[0] << std::endl;
//            std::cout << " modelLine[1]: " << modelLine[0] * modelLine[0] + modelLine[1] * modelLine[1] << std::endl << std::endl << std::endl;

        }*/

    }

    /**
    * @brief  Error calculated by Squared Endpoint Distance
    * @param  other line (ground truth)
    * @return squared error distance
    */
    template<class jetT>
    inline jetT errorSED(const jetT modelLine[3], const jetT ps[3], const jetT pe[3]){

        jetT tmp[2];
        errorSED(modelLine,ps,pe, tmp);
        return (tmp[0] * tmp[0] + tmp[1] * tmp[1]);
    }

    //! get pluecker coordinates of line
    template<class jetT>
    inline void plueckerCoordinates(const jetT pnt[3], const jetT vec[3], jetT m[3], jetT l[3]){
        jetT normSq = ceres::sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
        l[0] = vec[0] / normSq;
        l[1] = vec[1] / normSq;
        l[2] = vec[2] / normSq;
        ceres::CrossProduct(pnt, l, m);
    }

    //! get Cayley representation from pluecker coordinates line
    template<class jetT>
    inline void cayleyRepresentationFromPluecker(const jetT m[3], const jetT l[3], jetT &w, jetT s[3]){
        // T tau = std::numeric_limits<T>::min() * 3;
         static const jetT tau = static_cast<jetT>(1.0E-7);
         static const Eigen::Matrix<jetT, 3, 3> I = Eigen::Matrix<jetT, 3, 3>::Identity();

        // L2 norm
        w = ceres::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);


        Eigen::Matrix<jetT, 3, 3> Q = Eigen::Matrix<jetT, 3, 3>::Zero();
        Q(0,0) = l[0];
        Q(1,0) = l[1];
        Q(2,0) = l[2];

        if(w <= tau){
            jetT e1[3], e2[3], tmp[3];

            // create orthogonal base
            tmp[0] = -l[1];    // shift and negate
            tmp[1] = l[2];
            tmp[2] = l[0];

            ceres::CrossProduct(l, tmp, e1);
            ceres::CrossProduct(l, e1, e2);

            Q(0,1) = e1[0];
            Q(1,1) = e1[1];
            Q(2,1) = e1[2];

            Q(0,2) = e2[0];
            Q(1,2) = e2[1];
            Q(2,2) = e2[2];

        } else {

            jetT lXm[3];
            ceres::CrossProduct(l, m, lXm);
            jetT lXmNorm = ceres::sqrt(lXm[0] * lXm[0] + lXm[1] * lXm[1] + lXm[2] * lXm[2]);

            Q(0,1) = m[0] / w;
            Q(1,1) = m[1] / w;
            Q(2,1) = m[2] / w;

            Q(0,2) = lXm[0] / lXmNorm;
            Q(1,2) = lXm[1] / lXmNorm;
            Q(2,2) = lXm[2] / lXmNorm;

        }

        Eigen::Matrix<jetT, 3, 3> sx = Eigen::Matrix<jetT, 3, 3>::Zero();
        sx = (Q - I)*((Q + I).inverse().eval());

        s[0] = sx(2,1);
        s[1] = sx(0,2);
        s[2] = sx(1,0);
    }

    //! Pluecker Coordinates from Cayley Representation
    template<class jetT>
    inline void plueckerCoordinatesFromCayley(const jetT w, const jetT s[3], jetT m[3], jetT l[3]){
        static const Eigen::Matrix<jetT, 3, 3> I = Eigen::Matrix<jetT, 3, 3>::Identity();

        Eigen::Matrix<jetT, 3, 3> Q = Eigen::Matrix<jetT,3,3>::Zero(),
                sx = Eigen::Matrix<jetT,3,3>::Zero();

        Eigen::Matrix<jetT, 3, 1> sMat;
        sMat(0,0) = s[0];
        sMat(1,0) = s[1];
        sMat(2,0) = s[2];

        jetT sNormSquared = s[0] * s[0] + s[1] * s[1] + s[2] * s[2];

        sx(2,1) = s[0];
        sx(0,2) = s[1];
        sx(1,0) = s[2];

        sx(1,2) = -s[0];
        sx(2,0) = -s[1];
        sx(0,1) = -s[2];

        Q = ((jetT(1.0) - sNormSquared) * I + jetT(2) * sx + jetT(2)*sMat*sMat.transpose());
        Q = Q / (jetT(1) + sNormSquared);

        l[0] = Q(0,0);
        l[1] = Q(1,0);
        l[2] = Q(2,0);

        m[0] = Q(0,1) * w;
        m[1] = Q(1,1) * w;
        m[2] = Q(2,1) * w;

    }

    //! calculates cofK from K, matrices should be initialized to the correct size
    template<class jetT, class matT>
    inline void cofKcalculation(const matT &K, matT &cofK){
        cofK(0,0) = K(1,1);
        cofK(1,1) = K(0,0);
        cofK(0,2) = jetT(0);
        cofK(1,2) = jetT(0);
        cofK(2,0) = -K(1,1) * K(0,2);
        cofK(2,1) = -K(0,0) * K(1,2);
        cofK(2,2) = K(0,0) * K(1,1);
        cofK(0,1) = jetT(0);
        cofK(1,0) = jetT(0);
    }

    //! Project Pluecker line, with c being the translation vector, m being the momentum and l is the normed direction vector
    template<class jetT>
    inline void projectPlueckerCof(const Eigen::Matrix<jetT, 3, 3> &cofK, const Eigen::Matrix<jetT, 3, 3> &R, const jetT c[3], const jetT m[3], const jetT l[3], jetT line[3]){

        jetT mcl[3];
        ceres::CrossProduct(c, l, mcl);
        mcl[0] = m[0] - mcl[0];
        mcl[1] = m[1] - mcl[1];
        mcl[2] = m[2] - mcl[2];

        Eigen::Matrix<jetT, 3, 1> mclMat;
        mclMat(0,0) = mcl[0];
        mclMat(1,0) = mcl[1];
        mclMat(2,0) = mcl[2];

        Eigen::Matrix<jetT, 3, 1> lineMat = (cofK * R) * mclMat;

        line[0] = lineMat(0,0);
        line[1] = lineMat(1,0);
        line[2] = lineMat(2,0);

    }


    //! Project Pluecker line, with c being the translation vector, m being the momentum and l is the normed direction vector
    template<class jetT>
    inline void projectPluecker(const Eigen::Matrix<jetT, 3, 3> &K, const Eigen::Matrix<jetT, 3, 3> &R, const jetT c[3], const jetT m[3], const jetT l[3], jetT line[3]){

        Eigen::Matrix<jetT, 3, 3> cofK;
        cofKcalculation<jetT, Eigen::Matrix<jetT, 3, 3>>(K, cofK);
        projectPlueckerCof(cofK, R, c, m, l, line);
    }




}
#endif
#endif
