//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file PairwiseLineMatcher.hpp
/// @brief Pairwise line matcher for stereo or temporal correspondence.
/// Provides matching between pairs of line sets with optional filtering.

#pragma once

#include <arpack++/arlsmat.h>
#include <arpack++/arlssym.h>
#include <geometry/line.hpp>

#include <map>

namespace lsfm {

/// @brief Pairwise line matcher using spectral techniques for stereo or temporal line correspondence.
///
/// Matches line segments between two images by building a symmetric adjacency matrix
/// encoding pairwise geometric constraints and solving for its principal eigenvector.
/// Based on the spectral technique by M. Leordeanu for correspondence problems using
/// pairwise constraints.
/// @tparam FT Floating-point type used for geometric computations.
/// @tparam DT Descriptor type used for line appearance matching.
template <class FT, class DT>
class PairwiseLineMatcher {
  /// @brief Comparator for descending order in the eigen-value map.
  struct CompareL {
    /// @brief Compare two values in descending order.
    /// @param lhs Left-hand side value.
    /// @param rhs Right-hand side value.
    /// @return True if lhs is greater than rhs.
    bool operator()(const FT& lhs, const FT& rhs) const { return lhs > rhs; }
  };

  /// @brief Multimap storing eigenvalues (descending) mapped to their indices.
  typedef std::multimap<FT, unsigned int, CompareL> EigenMAP;

  /// @brief Precomputed geometric data for a single line segment.
  struct LineData {
    /// @brief Default constructor.
    LineData(){};

    /// @brief Construct LineData from angle and endpoints.
    /// @param a Angle of the line segment.
    /// @param s Start point of the line segment.
    /// @param e End point of the line segment.
    LineData(FT a, const Vec2<FT>& s, const Vec2<FT>& e) : angle(a), start(s), end(e){};

    FT angle;        ///< Angle of the line segment.
    Vec2<FT> start;  ///< Start point of the line segment.
    Vec2<FT> end;    ///< End point of the line segment.
  };

  /// @brief Precomputed geometric data for left image lines.
  std::vector<LineData> lineDataLeft_;
  /// @brief Precomputed geometric data for right image lines.
  std::vector<LineData> lineDataRight_;

 public:
  typedef FT float_type;                   ///< Floating-point type.
  typedef DT descriptor_type;              ///< Descriptor type for line appearance.
  typedef LineSegment<FT> geometric_type;  ///< Geometric primitive type (line segment).


  /// @brief Construct a PairwiseLineMatcher with default thresholds.
  PairwiseLineMatcher()
      : relativeAngleDifferenceThreshold_(static_cast<FT>(0.7854)),
        intersectionRationDifThreshold_(1),
        projectionRationDifThreshold_(1),
        descriptorDifThreshold_(static_cast<FT>(0.35)),
        weightOfMeanEigenVec_(static_cast<FT>(0.1)),
        minOfEigenVec_(0){};

  /// @brief Match lines using a 1D candidate list.
  ///
  /// Builds the adjacency matrix from the candidate matches and extracts the
  /// final correspondences from its principal eigenvector.
  /// @tparam GV Container type for geometric line segments.
  /// @tparam DV Container template for descriptors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Container type for match results.
  /// @param left Line segments from the left (query) image.
  /// @param right Line segments from the right (match) image.
  /// @param qDsc Descriptors for the left (query) lines.
  /// @param mDsc Descriptors for the right (match) lines.
  /// @param cm Candidate matches (input); distances may be updated in place.
  /// @param ret Output container for the final match results.
  template <class GV, template <class, class...> class DV, class... DVArgs, class MV>
  void match1D(
      const GV& left, const GV& right, const DV<DT, DVArgs...>& qDsc, const DV<DT, DVArgs...>& mDsc, MV& cm, MV& ret) {
    buildAdjacencyMatrix_(left, right, qDsc, mDsc, cm);
    matchingResultFromPrincipalEigenvector_(left, right, cm, ret);
  }

  /// @brief Match lines using a 2D candidate matrix (multiple match lists).
  ///
  /// Flattens the 2D candidate structure into a single list and delegates
  /// to match1D for adjacency matrix construction and eigenvector extraction.
  /// @tparam GV Container type for geometric line segments.
  /// @tparam DV Container template for descriptors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MVV Container-of-containers type for 2D candidate matches.
  /// @tparam MV Container type for match results.
  /// @param left Line segments from the left (query) image.
  /// @param right Line segments from the right (match) image.
  /// @param qDsc Descriptors for the left (query) lines.
  /// @param mDsc Descriptors for the right (match) lines.
  /// @param cm 2D candidate match structure to flatten and process.
  /// @param ret Output container for the final match results.
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

  static const FT TwoPI;  ///< Constant: 2 * pi.
  static const FT PI;     ///< Constant: pi.
  static const FT Inf;    ///< Constant: infinity (numeric max of FT).

 private:
  /// @brief Build the symmetric non-negative adjacency matrix.
  ///
  /// Constructs matrix M whose nodes are potential assignments a = (i_l, j_r)
  /// and whose edge weights measure pairwise agreement between assignments.
  /// Applies geometric constraints (angle, intersection ratio, projection ratio)
  /// and descriptor distance filtering.
  /// @see "A spectral technique for correspondence problems using pairwise
  ///       constraints" by M. Leordeanu.
  /// @tparam GV Container type for geometric line segments.
  /// @tparam DV Container template for descriptors.
  /// @tparam DVArgs Additional template arguments for the descriptor container.
  /// @tparam MV Container type for candidate matches.
  /// @param left Line segments from the left (query) image.
  /// @param right Line segments from the right (match) image.
  /// @param qDsc Descriptors for the left (query) lines.
  /// @param mDsc Descriptors for the right (match) lines.
  /// @param cm Candidate matches; distances may be lazily computed and updated.
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


  /// @brief Extract final line matches from the principal eigenvector.
  ///
  /// Iteratively selects the highest-scoring assignment from the eigenvector map,
  /// enforces one-to-one constraints and sidedness/angle consistency, and removes
  /// conflicting candidates.
  /// @tparam GV Container type for geometric line segments.
  /// @tparam MV Container type for match results.
  /// @param left Line segments from the left (query) image.
  /// @param right Line segments from the right (match) image.
  /// @param cm Candidate matches used to build the adjacency matrix.
  /// @param matchResult Output container for the accepted match correspondences.
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
  FT relativeAngleDifferenceThreshold_;  ///< Max allowed relative angle difference between line pairs.
  FT intersectionRationDifThreshold_;    ///< Max allowed intersection ratio difference.
  FT projectionRationDifThreshold_;      ///< Max allowed projection ratio difference.
  FT descriptorDifThreshold_;            ///< Max allowed descriptor distance for a valid match.
  FT weightOfMeanEigenVec_;              ///< Weight factor applied to mean eigenvector for threshold.
  FT minOfEigenVec_;                     ///< Minimum acceptable value in the principal eigenvector.

  /// @brief Map storing the principal eigenvector entries and their indices.
  ///
  /// Each entry is (eigenvalue, index). The eigenvalue is used as the key so
  /// that the map is sorted by eigenvalue in descending order, enabling
  /// greedy extraction of the best assignments.
  EigenMAP eigenMap_;
};


template <class FT, class DT>
const FT PairwiseLineMatcher<FT, DT>::TwoPI = static_cast<FT>(2 * CV_PI);

template <class FT, class DT>
const FT PairwiseLineMatcher<FT, DT>::PI = static_cast<FT>(CV_PI);

template <class FT, class DT>
const FT PairwiseLineMatcher<FT, DT>::Inf = std::numeric_limits<FT>::max();

}  // namespace lsfm
