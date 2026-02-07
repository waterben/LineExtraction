//*****************************************************************************************
/// \copyright (c) 2016-2026 Manuel Lange & Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file GlobalRotationFilter.hpp
/// @brief Global rotation-based feature filtering.
/// Provides filtering based on global rotation constraints between feature sets.

#pragma once

#include <geometry/line.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/option_manager.hpp>

#include <algorithm>
#include <valarray>

namespace lsfm {

/// @brief Global rotation-based feature filter.
/// Computes an approximate global rotation angle between two image feature sets
/// using angle and length histograms, then filters matches based on rotation
/// and length consistency.
/// @tparam FT Float type for computations
/// @tparam GV Geometric vector type containing line segments
template <class FT, class GV>
class GlobalRotationFilter : public FeatureFilter<FT>, public OptionManager {
  /// @brief Pre-computed line data for filtering.
  struct LineData {
    /// @brief Construct line data.
    /// @param a Line angle in radians
    /// @param l Line length
    LineData(FT a = 0, FT l = 0) : angle(a), length(l) {}

    FT angle;   ///< Line angle in radians
    FT length;  ///< Line length in pixels
  };

  std::vector<LineData> ldLeft_;   ///< Pre-computed data for left image lines
  std::vector<LineData> ldRight_;  ///< Pre-computed data for right image lines

 public:
  typedef FT float_type;                   ///< Float type used
  typedef GV geometric_vector;             ///< Geometric vector type
  typedef LineSegment<FT> geometric_type;  ///< Geometric element type

  static const FT TwoPI;  ///< Constant for 2 * PI

  /// @brief Construct a global rotation filter with default parameters.
  GlobalRotationFilter()
      : rotUsable_(false),
        rot_(0),
        resScale_(10),
        histDiff_(static_cast<FT>(0.49)),
        lengthDiff_(static_cast<FT>(0.5)),
        angleTh_(static_cast<FT>(CV_PI / 8)),
        lengthTh_(4) {
    // std::string type = (sizeof(float_type) > 4 ? "FT" : "float");
    // this->options_.push_back(OptionManager::OptionEntry("k", k, "int", "Number of nearest neighbors."));
    // this->options_.push_back(OptionManager::OptionEntry("radius", radius, type, "Max distance between
    // descriptors."));
  }

  /// @brief Train the filter by computing the global rotation between two sets.
  /// Builds angle and length histograms to estimate the approximate global
  /// rotation angle and determines whether the rotation estimate is reliable.
  /// @param left Left image line segments
  /// @param right Right image line segments
  void train(const GV& left, const GV& right) {
    l = &left;
    r = &right;

    ldLeft_.resize(left.size());
    ldRight_.resize(right.size());

    FT rotationAngle = TwoPI;

    // step 1: compute the angle histogram of lines in the left and right images
    unsigned int dim = static_cast<unsigned int>(360 / resScale_);  // number of the bins of histogram
    unsigned int index;                                             // index in the histogram

    FT direction;
    FT scalar = 180 / static_cast<FT>(resScale_ * CV_PI);      // used when compute the index
    FT angleShift = static_cast<FT>(resScale_ * CV_PI) / 360;  // make sure zero is the middle of the interval

    std::valarray<FT> angleHistLeft(static_cast<FT>(0), dim);
    std::valarray<FT> angleHistRight(static_cast<FT>(0), dim);
    std::valarray<FT> lengthLeft(
        static_cast<FT>(0), dim);  // lengthLeft[i] store the total line length of all the lines in the ith angle bin.
    std::valarray<FT> lengthRight(static_cast<FT>(0), dim);


    for (unsigned int i = 0; i < left.size(); ++i) {
      direction = (ldLeft_[i].angle = left[i].angle()) + static_cast<FT>(CV_PI) + angleShift;
      direction = direction < TwoPI ? direction : (direction - TwoPI);
      index = static_cast<unsigned int>(direction * scalar);
      ++angleHistLeft[index];
      lengthLeft[index] += (ldLeft_[i].length = left[i].length());
    }

    for (unsigned int i = 0; i < right.size(); ++i) {
      direction = (ldRight_[i].angle = right[i].angle()) + static_cast<FT>(CV_PI) + angleShift;
      direction = direction < TwoPI ? direction : (direction - TwoPI);
      index = static_cast<unsigned int>(direction * scalar);
      ++angleHistRight[index];
      lengthRight[index] += (ldRight_[i].length = right[i].length());
    }

    FT tmp =
        static_cast<FT>((1 / cv::norm(cv::_InputArray(&angleHistLeft[0], static_cast<int>(angleHistLeft.size())))));
    angleHistLeft *= tmp;
    tmp = static_cast<FT>((1 / cv::norm(cv::_InputArray(&angleHistRight[0], static_cast<int>(angleHistRight.size())))));
    angleHistRight *= tmp;
    tmp = static_cast<FT>((1 / cv::norm(cv::_InputArray(&lengthLeft[0], static_cast<int>(lengthLeft.size())))));
    lengthLeft *= tmp;
    tmp = static_cast<FT>((1 / cv::norm(cv::_InputArray(&lengthRight[0], static_cast<int>(lengthRight.size())))));
    lengthRight *= tmp;


    // step 2: find shift to decide the approximate global rotation
    std::vector<FT> difVec(dim);  // the difference vector between left histogram and shifted right histogram
    FT minDif = 10;               // the minimal angle histogram difference
    FT secondMinDif = 10;         // the second minimal histogram difference
    unsigned int minShift = std::numeric_limits<unsigned int>::max();  // the shift of right angle histogram when
                                                                       // minimal difference achieved
    unsigned int secondMinShift;  // the shift of right angle histogram when second minimal difference achieved

    std::vector<FT> lengthDifVec(dim);  // the length difference vector between left and right
    FT minLenDif = 10;                  // the minimal length difference
    FT secondMinLenDif = 10;            // the second minimal length difference
    unsigned int minLenShift = std::numeric_limits<unsigned int>::max();  // the shift of right length vector when
                                                                          // minimal length difference achieved
    unsigned int
        secondMinLenShift;  // the shift of right length vector when the second minimal length difference achieved

    FT normOfVec;
    for (unsigned int shift = 0; shift < dim; ++shift) {
      for (unsigned int j = 0; j < dim; ++j) {
        index = j + shift;
        index = index < dim ? index : (index - dim);
        difVec[j] = angleHistLeft[j] - angleHistRight[index];
        lengthDifVec[j] = lengthLeft[j] - lengthRight[index];
      }

      // find the minShift and secondMinShift for angle histogram
      normOfVec = static_cast<FT>(cv::norm(difVec));
      if (normOfVec < secondMinDif) {
        if (normOfVec < minDif) {
          secondMinDif = minDif;
          secondMinShift = minShift;
          minDif = normOfVec;
          minShift = shift;
        } else {
          secondMinDif = normOfVec;
          secondMinShift = shift;
        }
      }

      // find the minLenShift and secondMinLenShift of length vector
      normOfVec = static_cast<FT>(cv::norm(lengthDifVec));
      if (normOfVec < secondMinLenDif) {
        if (normOfVec < minLenDif) {
          secondMinLenDif = minLenDif;
          secondMinLenShift = minLenShift;
          minLenDif = normOfVec;
          minLenShift = shift;
        } else {
          secondMinLenDif = normOfVec;
          secondMinLenShift = shift;
        }
      }
    }


    rotationAngle = minShift * resScale_;
    if (rotationAngle > 90 && 360 - rotationAngle > 90) {
      // In most case we believe the rotation angle between two image pairs should belong to [-Pi/2, Pi/2]
      rotationAngle = rotationAngle - 180;
    }
    rotationAngle = static_cast<FT>(rotationAngle * CV_PI) / 180;

    // check whether there exist an approximate global rotation angle between image pair
    rotUsable_ = (minDif < histDiff_ && minLenDif < lengthDiff_);

    // std::cout << "minimal histgram distance = " << minDif << ", Approximate global rotation angle = " <<
    // rotationAngle << endl;
    rot_ = rotationAngle;
  }

  /// @brief Filter a match candidate by index.
  /// Checks angle consistency (accounting for global rotation) and length ratio.
  /// If no reliable rotation was found, only length is checked.
  /// @param lfIdx Left feature index
  /// @param rfIdx Right feature index
  /// @return True if the match should be rejected
  virtual bool filter(int lfIdx, int rfIdx) const {
    const LineData& l = ldLeft_[lfIdx];
    const LineData& r = ldRight_[rfIdx];

    // only length check
    if (!rotUsable_) return (std::abs(l.length - r.length) / std::min(l.length, r.length)) > lengthTh_;

    FT adiff = std::abs(l.angle + rot_ - r.angle);
    return (std::abs(TwoPI - adiff) > angleTh_ && adiff > angleTh_) ||
           ((std::abs(l.length - r.length) / std::min(l.length, r.length)) > lengthTh_);
  }

 protected:
  /// @brief Handle option value changes.
  /// @param name Option name
  /// @param value New option value
  void setOptionImpl(const std::string& name, FT value) {
    /*if (name == "k") {
        if (value >= 0 && value <= std::numeric_limits<int>::max()) {
            k_ = static_cast<int>(value);
            this->options_[0].value = k_;
        }
    }
    else if (name == "radius") {
        if (value >= 0 && value <= std::numeric_limits<float_type>::max()) {
            radius_ = static_cast<float_type>(value);
            this->options_[1].value = radius_;
        }
    }*/
  }

 private:
  FT rot_;          ///< Estimated global rotation angle in radians
  bool rotUsable_;  ///< Whether the rotation estimate is reliable

  FT resScale_;    ///< Histogram bin resolution in degrees
  FT histDiff_;    ///< Max angle histogram difference for usable rotation
  FT lengthDiff_;  ///< Max length histogram difference for usable rotation

  FT lengthTh_;  ///< Length ratio threshold for filtering
  FT angleTh_;   ///< Angle threshold in radians for filtering

  const GV *l, *r;  ///< Pointers to the training geometry sets
};

template <class FT, class GV>
const FT GlobalRotationFilter<FT, GV>::TwoPI = static_cast<FT>(2 * CV_PI);

}  // namespace lsfm
