#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#ifdef __cplusplus
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <algorithm>

#include "../geometry/line.hpp"
#include "../geometry/line3.hpp"
#include "../geometry/pose.hpp"
//#include "../utility/stereo_correlation.hpp"

#include "../calc_PnL/PnL.h"
#include "../calc_PnL/R_and_T.h"
#include "../calc_PnL/calc_PnL.h"
#include "../calc_PnL/calc_PnL_types.h"
#include "../calc_PnL/calc_PnL_emxAPI.h"


//namespace lsfm {

    //
    // Arguments    : const double x[3]
    // Return Type  : double
    //

    template <typename T>
    T norm(const T x[3])
    {
      T y;
      T scale;
      int k;
      T absxk;
      T t;
      y = 0.0;
      scale = 2.2250738585072014E-308;
      for (k = 0; k < 3; k++) {
        absxk = fabs(x[k]);
        if (absxk > scale) {
          t = scale / absxk;
          y = 1.0 + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      return scale * sqrt(y);
    }

    template <typename T>
    cv::Mat_<T> homogenizeVector(cv::Mat_<T> &translationVector){
        int lastId = translationVector.size().height - 1;
        for(int i = 0; i < lastId; i++){
            translationVector(i) = translationVector(i) / translationVector(lastId);
        }
        if(lastId >= 0)
            translationVector(lastId) = 1;
        return translationVector(cv::Rect(0,0,1,lastId));
    }   

    //! Calculates Rotation and Translation, needs a minimum of 3/4 Lines
    template<class FT> //                                                                                            camera matrix                return: new pose
    static FT PnLpose(const std::vector<lsfm::LineSegment<FT>> &lines2, const std::vector<lsfm::Line3<FT>> &lines3, const lsfm::Matx33<FT> &P1, lsfm::Pose<FT>& newPose, bool autoChooseLines){

        int numLines = lines3.size();
        double minimalReprojectionError = std::numeric_limits<double>::max();
        if(numLines < 4)
            return minimalReprojectionError;
        std::vector<double> xs, xe, V, P;

        for(int i = 0; i < lines3.size(); i++){
            const lsfm::LineSegment<FT> *line2 = &lines2.at(i);
            const lsfm::Line3<FT> *line3 = &lines3.at(i);

            double xstart = static_cast<double>((line2->startPoint().x() - P1(0,2)) / P1(0,0));
            double ystart = static_cast<double>((line2->startPoint().y() - P1(1,2)) / P1(1,1));
            double xend = static_cast<double>((line2->endPoint().x() - P1(0,2)) / P1(0,0));
            double yend = static_cast<double>((line2->endPoint().y() - P1(1,2)) / P1(1,1));

            xs.push_back(xstart);
            xs.push_back(ystart);
            xs.push_back(1);

            xe.push_back(xend);
            xe.push_back(yend);
            xe.push_back(1);

            V.push_back(line3->direction().x());
            V.push_back(line3->direction().y());
            V.push_back(line3->direction().z());

            P.push_back(line3->origin().x());
            P.push_back(line3->origin().y());
            P.push_back(line3->origin().z());
        }

        emxArray_real_T * xsw = emxCreateWrapper_real_T(xs.data(),3,numLines);
        emxArray_real_T * xew = emxCreateWrapper_real_T(xe.data(),3,numLines);
        emxArray_real_T * Vw = emxCreateWrapper_real_T(V.data(),3,numLines);
        emxArray_real_T * Pw = emxCreateWrapper_real_T(P.data(),3,numLines);

        int rot_cw_size[2];

        double rot_cw_data[9], pos_cw_data[3];
        PnL(xsw,  xew,  Vw,  Pw, autoChooseLines, rot_cw_data, rot_cw_size, pos_cw_data, &minimalReprojectionError);
        /*
        if(minimalReprojectionError > 100){
            std::cout << " err " << std::endl;
        }
        */
        emxDestroyArray_real_T(Pw);
        emxDestroyArray_real_T(Vw);
        emxDestroyArray_real_T(xsw);
        emxDestroyArray_real_T(xew);

        newPose.origin(lsfm::Vec3<FT>(static_cast<FT>(pos_cw_data[0]),static_cast<FT>(pos_cw_data[1]),static_cast<FT>(pos_cw_data[2])));
        cv::Mat_<double> rot_cw_data_left(3,3), rodriguesRot(3,1);
        for(int i = 0; i < 9; i++)
            rot_cw_data_left.at<double>(i) = rot_cw_data[i];
        cv::Rodrigues(rot_cw_data_left, rodriguesRot);
        newPose.orientation(lsfm::Vec3<FT>(static_cast<FT>(-rodriguesRot(0,0)), static_cast<FT>(-rodriguesRot(1,0)), static_cast<FT>(-rodriguesRot(2,0))));

        return static_cast<FT>(minimalReprojectionError);
    }

    //! Calculates Rotation and Translation, needs a minimum of 3/4 Lines and an initialization Pose
    template<class FT>
    static void R_and_Tpose(const std::vector<lsfm::LineSegment<FT>> &lines2, const std::vector<lsfm::Line3<FT>> &lines3, const lsfm::Matx33<FT> &P1, const lsfm::Pose<FT> &initPose, lsfm::Pose<FT>& newPose){

        int numLines = lines3.size();
        //double minimalReprojectionError = std::numeric_limits<double>::max();
        if(numLines < 4)
            return;
        std::vector<double> xs, xe, start3Dw, end3Dw;

        for(int i = 0; i < lines3.size(); i++){
            const lsfm::LineSegment<FT> *line2 = &lines2.at(i);
            const lsfm::Line3<FT> *line3 = &lines3.at(i);

            double xstart = static_cast<double>((line2->startPoint().x() - P1(0,2)) / P1(0,0));
            double ystart = static_cast<double>((line2->startPoint().y() - P1(1,2)) / P1(1,1));
            double xend = static_cast<double>((line2->endPoint().x() - P1(0,2)) / P1(0,0));
            double yend = static_cast<double>((line2->endPoint().y() - P1(1,2)) / P1(1,1));

            xs.push_back(xstart);
            xs.push_back(ystart);
            xs.push_back(1);

            xe.push_back(xend);
            xe.push_back(yend);
            xe.push_back(1);
            /*  No endpoints needed
            start3Dw.push_back(line3->startPoint().x());
            start3Dw.push_back(line3->startPoint().y());
            start3Dw.push_back(line3->startPoint().z());

            end3Dw.push_back(line3->endPoint().x());
            end3Dw.push_back(line3->endPoint().y());
            end3Dw.push_back(line3->endPoint().z());
            */
            start3Dw.push_back(line3->origin().x());
            start3Dw.push_back(line3->origin().y());
            start3Dw.push_back(line3->origin().z());

            end3Dw.push_back(line3->origin().x() + line3->direction().x());
            end3Dw.push_back(line3->origin().y() + line3->direction().y());
            end3Dw.push_back(line3->origin().z() + line3->direction().z());
        }

        emxArray_real_T * xsw = emxCreateWrapper_real_T(xs.data(),3,numLines);
        emxArray_real_T * xew = emxCreateWrapper_real_T(xe.data(),3,numLines);
        emxArray_real_T * P1w = emxCreateWrapper_real_T(start3Dw.data(),3,numLines);
        emxArray_real_T * P2w = emxCreateWrapper_real_T(end3Dw.data(),3,numLines);

        cv::Mat_<double> initRot_RnT(3,3), rodriguesRnT(3,1), initPosition(3,1);
        rodriguesRnT(0) = static_cast<double>(-initPose.orientation()[0]);
        rodriguesRnT(1) = static_cast<double>(-initPose.orientation()[1]);
        rodriguesRnT(2) = static_cast<double>(-initPose.orientation()[2]);
        initPosition(0) = static_cast<double>(initPose.origin()[0]);
        initPosition(1) = static_cast<double>(initPose.origin()[1]);
        initPosition(2) = static_cast<double>(initPose.origin()[2]);
        cv::Rodrigues(rodriguesRnT, initRot_RnT);
//        initRot_RnT = initRot_RnT.t();

        double initPos_cw[3];
        double initRot_cw[9];
        initPos_cw[0] = initPosition(0); initPos_cw[1] = initPosition(1); initPos_cw[2] = initPosition(2);
        for(int i = 0; i < 9; i++)
            initRot_cw[i] = initRot_RnT(i);


        double rot_cw_data[9], pos_cw_data[3];
        R_and_T(xsw, xew,
                P1w, P2w,
                initRot_cw, initPos_cw, 40.0,
                0.1, rot_cw_data, pos_cw_data);


        newPose.origin(lsfm::Vec3<FT>(static_cast<FT>(pos_cw_data[0]),static_cast<FT>(pos_cw_data[1]),static_cast<FT>(pos_cw_data[2])));
        cv::Mat_<double> rot_cw_data_left(3,3), rodriguesRot(3,1);
        for(int i = 0; i < 9; i++)
            rot_cw_data_left.at<double>(i) = rot_cw_data[i];
        cv::Rodrigues(rot_cw_data_left, rodriguesRot);
        newPose.orientation(lsfm::Vec3<FT>(static_cast<FT>(-rodriguesRot(0,0)), static_cast<FT>(-rodriguesRot(1,0)), static_cast<FT>(-rodriguesRot(2,0))));

        emxDestroyArray_real_T(P1w);
        emxDestroyArray_real_T(P2w);
        emxDestroyArray_real_T(xsw);
        emxDestroyArray_real_T(xew);

    }

    template <typename T>
    cv::Mat_<T> getProjectionMatrix(const cv::Mat_<T> cameraMatrix, const cv::Mat_<T> rotationMatrix, const cv::Mat_<T> translationVector){
        cv::Mat_<T> tVec;
        translationVector.copyTo(tVec);

        cv::Mat_<T> projMat = cv::Mat_<T>::zeros(4,4);
        cv::Mat_<T> newCamMat = cv::Mat_<T>::zeros(3,4);
        cv::Mat_<T> tmpDest = newCamMat(cv::Rect(0,0,3,3));

        cameraMatrix.copyTo(tmpDest);
        tmpDest = projMat(cv::Rect(0,0,3,3));
        rotationMatrix.copyTo(tmpDest);

        // homogenize
        if(tVec.size().height == 4)
            tVec = homogenizeVector(tVec);

        tVec = -tVec;
//        projMat(3,3) = 1;
        tmpDest = projMat(cv::Rect(3,0,1,3));
        tVec.copyTo(tmpDest);

        projMat = newCamMat * projMat;
        return projMat;
    }

    template <typename T>
    T reprojectionErrorPnL(emxArray_real_T *xs, emxArray_real_T *xe, emxArray_real_T *Vw,
                           emxArray_real_T *Pw, T rot_cw_data[9],
                           T pos_cw[3]){
        // check the reprojection error.

        T c_temp[3];
        T temp[3];
        T nc1[3];
        int itmp;
        // T RzRxRot[9];
        T mtmp;
        T b_d3;
        T b_xs[3];


        // Transpose
        T rot_cw_data_T[9];
        for (int ix = 0; ix < 3; ix++) {
          for (int itmp = 0; itmp < 3; itmp++) {
            rot_cw_data_T[ix + 3 * itmp] = rot_cw_data[itmp + 3 * ix];
          }
        }

        int n = xs->size[1];
        T reprojectionError = 0.0;
   //     std::cout << "npe: " << n << std::endl;
        for (int i = 0; i < n; i++) {
          for (int ix = 0; ix < 3; ix++) {
            temp[ix] = Pw->data[ix + Pw->size[0] * i] - pos_cw[ix];
          }

          c_temp[0] = temp[1] * Vw->data[2 + Vw->size[0] * i] - temp[2] *
            Vw->data[1 + Vw->size[0] * i];
          c_temp[1] = temp[2] * Vw->data[Vw->size[0] * i] - temp[0] *
            Vw->data[2 + Vw->size[0] * i];
          c_temp[2] = temp[0] * Vw->data[1 + Vw->size[0] * i] - temp[1] *
            Vw->data[Vw->size[0] * i];
          for (int ix = 0; ix < 3; ix++) {
            nc1[ix] = 0.0;
            for (itmp = 0; itmp < 3; itmp++) {
              nc1[ix] += rot_cw_data_T[ix + 3 * itmp] * c_temp[itmp]; // RzRxRot
            }
          }

          // line projection function
          mtmp = 0.0;
          for (int ix = 0; ix < 3; ix++) {
            mtmp += nc1[ix] * xs->data[ix + xs->size[0] * i];
          }

          b_d3 = 0.0;
          for (int ix = 0; ix < 3; ix++) {
            b_d3 += nc1[ix] * xe->data[ix + xe->size[0] * i];
          }

          for (int ix = 0; ix < 3; ix++) {
            b_xs[ix] = xs->data[ix + xs->size[0] * i] - xe->data[ix +
              xe->size[0] * i];
          }
          T addition = norm(b_xs) / 3.0 * ((mtmp * mtmp + mtmp *
                                            b_d3) + b_d3 * b_d3) / (nc1[0] * nc1[0] + nc1[1] * nc1[1]);
          reprojectionError += addition;
   //       std::cout << "additionpe: " << addition << std::endl;
        }
        return reprojectionError;
    }


    template <typename T>
    T reprojectionErrorPnL(const std::vector<lsfm::LineSegment<T>> &lines2, const std::vector<lsfm::Line3<T>> &lines3, const lsfm::Matx33<T> &P1, const lsfm::Pose<T> &pose){
        std::vector<double> xs, xe, V, P;
        int numLines = lines3.size();
        if(lines3.size() <= 0)
            return std::numeric_limits<T>::max();

        for(int i = 0; i < lines3.size(); i++){
            const lsfm::LineSegment<T> *line2 = &lines2.at(i);
            const lsfm::Line3<T> *line3 = &lines3.at(i);

            double xstart = static_cast<double>((line2->startPoint().x() - P1(0,2)) / P1(0,0));
            double ystart = static_cast<double>((line2->startPoint().y() - P1(1,2)) / P1(1,1));
            double xend = static_cast<double>((line2->endPoint().x() - P1(0,2)) / P1(0,0));
            double yend = static_cast<double>((line2->endPoint().y() - P1(1,2)) / P1(1,1));

            xs.push_back(xstart);
            xs.push_back(ystart);
            xs.push_back(1);

            xe.push_back(xend);
            xe.push_back(yend);
            xe.push_back(1);

            V.push_back(line3->direction().x());
            V.push_back(line3->direction().y());
            V.push_back(line3->direction().z());

            P.push_back(line3->origin().x());
            P.push_back(line3->origin().y());
            P.push_back(line3->origin().z());
        }

        cv::Mat_<double> initRot_RnT(3,3), rodriguesRnT(3,1), initPosition(3,1);
        rodriguesRnT(0) = -pose.orientation()[0];
        rodriguesRnT(1) = -pose.orientation()[1];
        rodriguesRnT(2) = -pose.orientation()[2];
        initPosition(0) = pose.origin()[0];
        initPosition(1) = pose.origin()[1];
        initPosition(2) = pose.origin()[2];
        cv::Rodrigues(rodriguesRnT, initRot_RnT);
//        initRot_RnT = initRot_RnT.t();

        double initPos_cw[3];
        double initRot_cw[9];
        initPos_cw[0] = initPosition(0); initPos_cw[1] = initPosition(1); initPos_cw[2] = initPosition(2);
        for(int i = 0; i < 9; i++)
            initRot_cw[i] = initRot_RnT(i);

        emxArray_real_T * xsw = emxCreateWrapper_real_T(xs.data(),3,numLines);
        emxArray_real_T * xew = emxCreateWrapper_real_T(xe.data(),3,numLines);
        emxArray_real_T * Vw = emxCreateWrapper_real_T(V.data(),3,numLines);
        emxArray_real_T * Pw = emxCreateWrapper_real_T(P.data(),3,numLines);

        T error = reprojectionErrorPnL(xsw, xew, Vw, Pw, initRot_cw, initPos_cw);

        emxDestroyArray_real_T(Vw);
        emxDestroyArray_real_T(Pw);
        emxDestroyArray_real_T(xsw);
        emxDestroyArray_real_T(xew);

        return error;
    }


#endif
#endif // POSE_ESTIMATOR_H
