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
//M*/

#pragma once

#include <arpack++/arlsmat.h>
#include <arpack++/arlssym.h>
#include <geometry/line.hpp>

#include <map>

namespace lsfm {

template <class FT, class DT>
class PairwiseLineMatcher {
  struct CompareL {
    bool operator()(const FT& lhs, const FT& rhs) const { return lhs > rhs; }
  };

  typedef std::multimap<FT, unsigned int, CompareL> EigenMAP;

  struct LineData {
    LineData(){};
    LineData(FT a, const Vec2<FT>& s, const Vec2<FT>& e) : angle(a), start(s), end(e){};

    FT angle;
    Vec2<FT> start, end;
  };

  std::vector<LineData> lineDataLeft_;
  std::vector<LineData> lineDataRight_;

 public:
  typedef FT float_type;
  typedef DT descriptor_type;
  typedef LineSegment<FT> geometric_type;


  PairwiseLineMatcher()
      : relativeAngleDifferenceThreshold_(static_cast<FT>(0.7854)),
        intersectionRationDifThreshold_(1),
        projectionRationDifThreshold_(1),
        descriptorDifThreshold_(static_cast<FT>(0.35)),
        weightOfMeanEigenVec_(static_cast<FT>(0.1)),
        minOfEigenVec_(0){};

  template <class GV, template <class, class...> class DV, class... DVArgs, class MV>
  void match1D(
      const GV& left, const GV& right, const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, MV& cm, MV& ret) {
    buildAdjacencyMatrix_(left, right, qDsc, mDsc, cm);
    matchingResultFromPrincipalEigenvector_(left, right, cm, ret);
  }

  template <class GV, template <class, class...> class DV, class... DVArgs, class MVV, class MV>
  void match2D(const GV& left,
               const GV& right,
               const DV<DT, DVArgs...>& qDsc,
               const DV<DT, DVArgs...>& mDsc,
               const MVV& cm,
               MV& ret) {
    typename MVV::value_type mv;
    decltype(mv.size()) sz = 0;
    for_each(cm.begin(), cm.end(), [&sz](const typename MVV::value_type& v) { sz += v.size(); });
    mv.reserve(sz);
    for_each(cm.begin(), cm.end(),
             [&mv](const typename MVV::value_type& v) { mv.insert(mv.end(), v.begin(), v.end()); });
    std::cout << "candidates: " << mv.size() << std::endl;
    match1D(left, right, qDsc, mDsc, mv, ret);
  }

  static const FT TwoPI;
  static const FT PI;
  static const FT Inf;

 private:
  //! Build the symmetric non-negative adjacency matrix M, whose nodes are the potential assignments a = (i_l, j_r)
  //! and whose weights on edges measure the agreements between pairs of potential assignments. That is where the
  //! pairwise constraints are applied(c.f. A spectral technique for correspondence problems using pairwise constraints,
  //! M.Leordeanu).
  template <class GV, template <class, class...> class DV, class... DVArgs, class MV>
  void buildAdjacencyMatrix_(
      const GV& left, const GV& right, const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, MV& cm) {
    int numLineLeft = static_cast<int>(left.size());
    int numLineRight = static_cast<int>(right.size());

    // precompute often used data
    lineDataLeft_.clear();
    lineDataLeft_.reserve(static_cast<size_t>(numLineLeft));
    lineDataRight_.clear();
    lineDataRight_.reserve(static_cast<size_t>(numLineLeft));

    for_each(left.begin(), left.end(), [this](const geometric_type& l) {
      this->lineDataLeft_.push_back(LineData(l.angle(), l.startPoint(), l.endPoint()));
    });

    for_each(right.begin(), right.end(), [this](const geometric_type& l) {
      this->lineDataRight_.push_back(LineData(l.angle(), l.startPoint(), l.endPoint()));
    });


    // Build the adjacency matrix which reflect the geometric constraints between nodes.
    // The matrix is stored in the Compressed Sparse Column(CSC) format.
    int dim = static_cast<int>(cm.size());  // Dimension of the problem.
    int nnz = 0;                            // Number of nonzero elements in adjacenceMat.
    // adjacenceVec only store the lower part of the adjacency matrix which is a symmetric matrix.
    //                     | 0  1  0  2  0 |
    //                     | 1  0  3  0  1 |
    // eg:  adjMatrix =    | 0  3  0  2  0 |
    //                     | 2  0  2  0  3 |
    //                     | 0  1  0  3  0 |
    //     adjacenceVec = [0,1,0,2,0,0,3,0,1,0,2,0,0,3,0]
    //

    std::vector<FT> adjacenceVec(dim * (dim + 1) / 2, 0);

    // In order to save computational time, the following variables are used to store
    // the pairwise geometric information which has been computed and will be reused many times
    // later. The reduction of computational time is at the expenses of memory consumption.
    struct Data {
      Data() : bComputed(false), intersecRatio(Inf), projRatio(Inf) {}

      bool bComputed;    // flags to show whether the ith pair of left / right image has already been computed.
      FT intersecRatio;  // the ratios of intersection point and the line in the left / right pair
      FT projRatio;      // the point to line distance divided by the projected length of line in the left / right pair.
    };

    std::vector<Data> vecDataLeft(numLineLeft * numLineLeft, Data());
    std::vector<Data> vecDataRight(numLineRight * numLineRight, Data());
    // cv::Mat_<FT> dis(numLineLeft, numLineRight, Inf);

    int idLeft1, idLeft2;    // the id of lines in the left pair
    int idRight1, idRight2;  // the id of lines in the right pair

    FT relativeAngleLeft, relativeAngleRight;  // the relative angle of each line pair
    FT relativeAngleDif, iRatioDif, pRatioDif;

    Vec2<FT> interSectionPoint;
    FT length1, length2, dis1, dis2;
    Vec2<FT> sc, se;
    FT similarity;

    for (int j = 0; j != dim; ++j) {  // column
      idLeft1 = cm[j].queryIdx;
      idRight1 = cm[j].matchIdx;
      for (int i = j + 1; i != dim; ++i) {  // row
        idLeft2 = cm[i].queryIdx;
        idRight2 = cm[i].matchIdx;

        if ((idLeft1 == idLeft2) || (idRight1 == idRight2)) {
          continue;  // not satisfy the one to one match condition
        }

        const LineData& lineDataLeft1 = lineDataLeft_[idLeft1];
        const LineData& lineDataLeft2 = lineDataLeft_[idLeft2];
        const LineData& lineDataRight1 = lineDataRight_[idRight1];
        const LineData& lineDataRight2 = lineDataRight_[idRight2];

        // first compute the relative angle between left pair and right pair.
        relativeAngleLeft = lineDataLeft1.angle - lineDataLeft2.angle;
        relativeAngleLeft = (relativeAngleLeft < PI) ? relativeAngleLeft : (relativeAngleLeft - TwoPI);
        relativeAngleLeft = (relativeAngleLeft > (-PI)) ? relativeAngleLeft : (relativeAngleLeft + TwoPI);

        relativeAngleRight = lineDataRight1.angle - lineDataRight2.angle;
        relativeAngleRight = (relativeAngleRight < PI) ? relativeAngleRight : (relativeAngleRight - TwoPI);
        relativeAngleRight = (relativeAngleRight > (-PI)) ? relativeAngleRight : (relativeAngleRight + TwoPI);

        relativeAngleDif = fabs(relativeAngleLeft - relativeAngleRight);
        relativeAngleDif = (relativeAngleDif < PI) ? relativeAngleDif : (TwoPI - relativeAngleDif);

        if (relativeAngleDif > relativeAngleDifferenceThreshold_) {
          continue;  // the relative angle difference is too large;
        }


        const geometric_type& lineLeft1 = left[idLeft1];
        const geometric_type& lineLeft2 = left[idLeft2];
        const geometric_type& lineRight1 = right[idRight1];
        const geometric_type& lineRight2 = right[idRight2];

        Data& dataLeft1 = vecDataLeft[idLeft1 * numLineLeft + idLeft2];
        Data& dataLeft2 = vecDataLeft[idLeft2 * numLineLeft + idLeft1];
        Data& dataRight1 = vecDataRight[idRight1 * numLineRight + idRight2];
        Data& dataRight2 = vecDataRight[idRight2 * numLineRight + idRight1];

        dis1 = cm[j].distance;
        if (dis1 == Inf) cm[j].distance = dis1 = qDsc[idLeft1].distance(mDsc[idRight1]);

        dis2 = cm[i].distance;
        if (dis2 == Inf) cm[i].distance = dis2 = qDsc[idLeft2].distance(mDsc[idRight2]);

        if (dis1 > descriptorDifThreshold_ || dis2 > descriptorDifThreshold_) {
          continue;  // descriptor distance is to large (better to filter by distance before!)
        }

        // at last, check the intersect point ratio and point to line distance ratio
        // check whether the geometric information of pairs (idLeft1,idLeft2) and (idRight1,idRight2) have already been
        // computed.
        if (!dataLeft1.bComputed) {  // have not been computed yet
          length1 = lineRight1.length();
          length2 = lineLeft2.length();

          if (lineLeft1.intersection(lineLeft2, interSectionPoint)) {
            sc = interSectionPoint - lineDataLeft1.start;
            se = lineDataLeft1.end - lineDataLeft1.start;
            dataLeft1.intersecRatio = (sc.x() * se.x() + sc.y() * se.y()) / (length1 * length1);

            sc = interSectionPoint - lineDataLeft2.start;
            se = lineDataLeft2.end - lineDataLeft2.start;
            dataLeft2.intersecRatio = (sc.x() * se.x() + sc.y() * se.y()) / (length2 * length2);
          }


          // project the end points of line1 onto line2 and compute their distances to line2;
          dataLeft1.projRatio =
              (lineLeft2.distance(lineDataLeft1.start) + lineLeft2.distance(lineDataLeft1.end)) / length2;
          dataLeft2.projRatio =
              (lineLeft1.distance(lineDataLeft2.start) + lineLeft1.distance(lineDataLeft2.end)) / length1;

          // mark them as computed
          dataLeft1.bComputed = true;
          dataLeft2.bComputed = true;
        }


        if (!dataRight1.bComputed) {  // have not been computed yet
          length1 = lineRight1.length();
          length2 = lineRight2.length();

          if (lineRight1.intersection(lineRight2, interSectionPoint)) {
            sc = interSectionPoint - lineDataRight1.start;
            se = lineDataRight1.end - lineDataRight1.start;
            dataRight1.intersecRatio = (sc.x() * se.x() + sc.y() * se.y()) / (length1 * length1);

            sc = interSectionPoint - lineDataRight2.start;
            se = lineDataRight2.end - lineDataRight2.start;
            dataRight2.intersecRatio = (sc.x() * se.x() + sc.y() * se.y()) / (length2 * length2);
          }


          // project the end points of line1 onto line2 and compute their distances to line2;
          dataRight1.projRatio =
              (lineRight2.distance(lineDataRight1.start) + lineRight2.distance(lineDataRight1.end)) / length2;
          dataRight2.projRatio =
              (lineRight1.distance(lineDataRight2.start) + lineRight1.distance(lineDataRight2.end)) / length1;

          // mark them as computed
          dataRight1.bComputed = true;
          dataRight2.bComputed = true;
        }

        pRatioDif =
            MIN(fabs(dataLeft1.projRatio - dataRight1.projRatio), fabs(dataLeft2.projRatio - dataRight2.projRatio));
        if (pRatioDif > projectionRationDifThreshold_) {
          continue;  // the projection length ratio difference is too large;
        }

        if ((dataLeft1.intersecRatio == Inf) || (dataLeft2.intersecRatio == Inf) || (dataRight1.intersecRatio == Inf) ||
            (dataRight2.intersecRatio == Inf)) {
          // don't consider the intersection length ratio
          similarity = 4 - dis1 / descriptorDifThreshold_ - dis2 / descriptorDifThreshold_ -
                       pRatioDif / projectionRationDifThreshold_ - relativeAngleDif / relativeAngleDifferenceThreshold_;
          adjacenceVec[(2 * dim - j - 1) * j / 2 + i] = similarity;
          nnz++;
        } else {
          iRatioDif = MIN(fabs(dataLeft1.intersecRatio - dataRight1.intersecRatio),
                          fabs(dataLeft2.intersecRatio - dataRight2.intersecRatio));
          if (iRatioDif > intersectionRationDifThreshold_) {
            continue;  // the intersection length ratio difference is too large;
          }
          // now compute the similarity score between two line pairs.
          similarity = 5 - dis1 / descriptorDifThreshold_ - dis2 / descriptorDifThreshold_ -
                       iRatioDif / intersectionRationDifThreshold_ - pRatioDif / projectionRationDifThreshold_ -
                       relativeAngleDif / relativeAngleDifferenceThreshold_;
          adjacenceVec[(2 * dim - j - 1) * j / 2 + i] = similarity;
          nnz++;
        }
      }
    }

    // pointer to an array that stores the nonzero elements of Adjacency matrix.
    FT* adjacenceMat = new FT[nnz];
    // pointer to an array that stores the row indices of the non-zeros in adjacenceMat.
    int* irow = new int[nnz];
    // pointer to an array of pointers to the beginning of each column of adjacenceMat.
    int* pcol = new int[dim + 1];
    int idOfNNZ = 0;  // the order of none zero element
    pcol[0] = 0;
    int tempValue;
    for (int j = 0; j < dim; j++) {    // column
      for (int i = j; i < dim; i++) {  // row
        tempValue = (2 * dim - j - 1) * j / 2 + i;
        if (adjacenceVec[tempValue] != 0) {
          adjacenceMat[idOfNNZ] = adjacenceVec[tempValue];
          irow[idOfNNZ] = i;
          idOfNNZ++;
        }
      }
      pcol[j + 1] = idOfNNZ;
    }

    /* Second step, solve the principal eigenvector of the adjacency matrix using Arpack lib.
     */
    ARluSymMatrix<FT> arMatrix(dim, nnz, adjacenceMat, irow, pcol);
    ARluSymStdEig<FT> dprob(2, arMatrix,
                            "LM");  // Defining what we need: the first eigenvector of arMatrix with largest magnitude.
    // Finding eigenvalues and eigenvectors.
    dprob.FindEigenvectors();
    // cout << "Number of 'converged' eigenvalues  : " << dprob.ConvergedEigenvalues() << endl;
    eigenMap_.clear();

    FT meanEigenVec = 0;
    if (dprob.EigenvectorsFound()) {
      FT value;
      for (int j = 0; j < dim; j++) {
        value = fabs(dprob.Eigenvector(1, j));
        meanEigenVec += value;
        eigenMap_.insert(std::make_pair(value, j));
      }
    }
    minOfEigenVec_ = weightOfMeanEigenVec_ * meanEigenVec / dim;
    delete[] adjacenceMat;
    delete[] irow;
    delete[] pcol;
  }


  //! Get the final matching from the principal eigenvector.
  template <class GV, class MV>
  void matchingResultFromPrincipalEigenvector_(const GV& left, const GV& right, const MV& cm, MV& matchResult) {
    matchResult.clear();
    matchResult.reserve(cm.size());

    double matchScore1 = 0;
    typename EigenMAP::iterator iter;
    int id, idLeft2, idRight2, dim = static_cast<int>(cm.size());
    FT sideValueL, sideValueR;
    Vec2<FT> point;
    FT relativeAngleLeft, relativeAngleRight;  // the relative angle of each line pair
    FT relativeAngleDif;


    // store eigenMap for debug
    /*std::fstream resMap;
    ostringstream fileNameMap;
    fileNameMap << "eigenVec.txt";
    resMap.open(fileNameMap.str().c_str(), std::ios::out);

    Matrix<double> mat(linesInLeft.size(), linesInRight.size());
    mat.SetZero();
    for (iter = eigenMap_.begin(); iter != eigenMap_.end(); iter++){
        id = iter->second;
        resMap << cm[id].queryIdx << "    " << cm[id].matchIdx << "   " << iter->first << endl;
        mat[cm[id].queryIdx][cm[id].matchIdx] = iter->first;
    }
    mat.Save("eigenMap.txt");
    resMap.flush();
    resMap.close();*/


    /*first try, start from the top element in eigenmap */
    while (1) {
      iter = eigenMap_.begin();
      // if the top element in the map has small value, then there is no need to continue find more matching line pairs;
      if (iter->first < minOfEigenVec_) {
        break;
      }
      id = iter->second;
      const auto& match = cm[id];
      int idLeft1 = match.queryIdx;
      int idRight1 = match.matchIdx;
      ;
      matchResult.push_back(match);
      matchScore1 += iter->first;
      eigenMap_.erase(iter++);

      const LineData& lineDataLeft1 = lineDataLeft_[idLeft1];
      const LineData& lineDataRight1 = lineDataRight_[idRight1];

      // remove all potential assignments in conflict with top matched line pair
      Vec2<FT> e_sLeft = lineDataLeft1.end - lineDataLeft1.start;
      Vec2<FT> e_sRight = lineDataRight1.end - lineDataRight1.start;
      FT coefLeft = sqrt(e_sLeft.x() * e_sLeft.x() + e_sLeft.y() * e_sLeft.y());
      FT coefRight = sqrt(e_sRight.x() * e_sRight.x() + e_sRight.y() * e_sRight.y());
      for (; iter->first >= minOfEigenVec_;) {
        id = iter->second;
        idLeft2 = cm[id].queryIdx;
        idRight2 = cm[id].matchIdx;

        // check one to one match condition
        if ((idLeft1 == idLeft2) || (idRight1 == idRight2)) {
          eigenMap_.erase(iter++);
          continue;  // not satisfy the one to one match condition
        }

        const LineData& lineDataLeft2 = lineDataLeft_[idLeft2];
        const LineData& lineDataRight2 = lineDataRight_[idRight2];

        // check sidedness constraint, the middle point of line2 should lie on the same side of line1.
        // sideValue = (y-ys)*(xe-xs)-(x-xs)*(ye-ys);
        point = 0.5 * (lineDataLeft2.start + lineDataLeft2.end);
        sideValueL =
            (point.y() - lineDataLeft1.start.y()) * e_sLeft.x() - (point.x() - lineDataLeft1.start.x()) * e_sLeft.y();
        sideValueL = sideValueL / coefLeft;

        point = 0.5 * (lineDataRight2.start + lineDataRight2.end);
        sideValueR =
            (point.y() - lineDataRight1.start.y()) * e_sLeft.x() - (point.x() - lineDataRight1.start.x()) * e_sLeft.y();
        sideValueR = sideValueR / coefRight;

        if (sideValueL * sideValueR < 0 && fabs(sideValueL) > 5 &&
            fabs(sideValueR) > 5) {  // have the different sign, conflict happens.
          eigenMap_.erase(iter++);
          continue;
        }

        // first compute the relative angle between left pair and right pair.
        relativeAngleLeft = lineDataLeft1.angle - lineDataLeft2.angle;
        relativeAngleLeft = (relativeAngleLeft < PI) ? relativeAngleLeft : (relativeAngleLeft - TwoPI);
        relativeAngleLeft = (relativeAngleLeft > (-PI)) ? relativeAngleLeft : (relativeAngleLeft + TwoPI);

        relativeAngleRight = lineDataRight1.angle - lineDataRight2.angle;
        relativeAngleRight = (relativeAngleRight < PI) ? relativeAngleRight : (relativeAngleRight - TwoPI);
        relativeAngleRight = (relativeAngleRight > (-PI)) ? relativeAngleRight : (relativeAngleRight + TwoPI);

        relativeAngleDif = fabs(relativeAngleLeft - relativeAngleRight);
        relativeAngleDif = (relativeAngleDif < PI) ? relativeAngleDif : (TwoPI - relativeAngleDif);

        if (relativeAngleDif > relativeAngleDifferenceThreshold_) {
          eigenMap_.erase(iter++);
          continue;  // the relative angle difference is too large;
        }
        iter++;
      }
    }  // end while(stillLoop)
  }

 private:
  FT relativeAngleDifferenceThreshold_;
  FT intersectionRationDifThreshold_;
  FT projectionRationDifThreshold_;
  FT descriptorDifThreshold_;
  FT weightOfMeanEigenVec_;
  FT minOfEigenVec_;  // the acceptable minimal value in the principal eigen vector;

  //! construct a map to store the principal eigenvector and its index.
  //! each pair in the map is in this form (eigenvalue, index);
  //! Note that, we use eigenvalue as key in the map and index as their value.
  //! This is because the map need be sorted by the eigenvalue rather than index
  //! for our purpose.
  EigenMAP eigenMap_;
};


template <class FT, class DT>
const FT PairwiseLineMatcher<FT, DT>::TwoPI = static_cast<FT>(2 * CV_PI);

template <class FT, class DT>
const FT PairwiseLineMatcher<FT, DT>::PI = static_cast<FT>(CV_PI);

template <class FT, class DT>
const FT PairwiseLineMatcher<FT, DT>::Inf = std::numeric_limits<FT>::max();

}  // namespace lsfm
