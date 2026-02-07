//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file direction.hpp
/// @brief Direction computation policies for image gradients.

#pragma once

#include <imgproc/polar.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/range.hpp>

#include <string>


namespace lsfm {

/// @brief Precise direction computation using atan2.
///
/// Computes gradient direction with full precision, returning angles
/// in the range [-PI, PI] radians.
/// @tparam GT Gradient input type (short, float, or double)
/// @tparam DT Direction output type (float or double)
template <class GT = short, class DT = float>
struct Direction {
  typedef GT grad_type;              ///< Gradient input type
  typedef DT dir_type;               ///< Direction output type
  typedef Range<DT> DirectionRange;  ///< Direction range type

  /// @brief Compute direction from gradient components.
  /// @param gx X-direction gradient
  /// @param gy Y-direction gradient
  /// @return Direction angle in radians [-PI, PI]
  inline static DT process(GT gx, GT gy) { return std::atan2(static_cast<DT>(gy), static_cast<DT>(gx)); }

  /// @brief Compute direction for arrays.
  /// @param gx Pointer to X-gradient array
  /// @param gy Pointer to Y-gradient array
  /// @param[out] dir Output direction array
  /// @param size Number of elements to process
  inline static void process(const GT* gx, const GT* gy, DT* dir, size_t size) {
    for (size_t i = 0; i != size; ++i) dir[i] = Direction<GT, DT>::process(gx[i], gy[i]);
  }

  /// @brief Compute direction from cv::Mat gradient images.
  /// @param gx X-direction gradient matrix
  /// @param gy Y-direction gradient matrix
  /// @param[out] dir Output direction matrix (float or double)
  inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& dir) {
    CV_Assert(gx.type() == gy.type() && (gx.type() & CV_MAT_DEPTH_MASK) == cv::DataType<GT>::type &&
              gx.rows == gy.rows && gx.cols == gy.cols);
    dir.create(gx.size(), CV_MAKETYPE(cv::DataType<DT>::type, gx.channels()));
    Direction<GT, DT>::process(
        gx.ptr<GT>(), gy.ptr<GT>(), dir.ptr<DT>(),
        static_cast<size_t>(gx.rows) * static_cast<size_t>(gx.cols) * static_cast<size_t>(gx.channels()));
  }

  /// @brief Get direction value range.
  /// @return Range [-PI, PI]
  inline static const DirectionRange& range() {
    static DirectionRange r(static_cast<DT>(-CV_PI), static_cast<DT>(CV_PI));
    return r;
  }

  /// @brief Get direction method name.
  /// @return "dir"
  inline static const std::string name() { return "dir"; }
};


/// @brief Fast direction computation using cv::fastAtan2.
///
/// Computes gradient direction with lookup table approximation, returning
/// angles in the range [0, 360] degrees. Faster but less precise than Direction.
/// @tparam GT Gradient input type (short, float, or double)
/// @tparam DT Direction output type (float or double)
template <class GT = short, class DT = float>
struct FastDirection {
  typedef GT grad_type;              ///< Gradient input type
  typedef DT dir_type;               ///< Direction output type
  typedef Range<DT> DirectionRange;  ///< Direction range type


  /// @brief Compute fast direction from gradient components.
  /// @param gx X-direction gradient
  /// @param gy Y-direction gradient
  /// @return Direction angle in degrees [0, 360]
  inline static DT process(GT gx, GT gy) { return cv::fastAtan2(static_cast<float>(gy), static_cast<float>(gx)); }

  /// @brief Compute fast direction for arrays.
  /// @param gx Pointer to X-gradient array
  /// @param gy Pointer to Y-gradient array
  /// @param[out] dir Output direction array
  /// @param size Number of elements to process
  inline static void process(const GT* gx, const GT* gy, DT* dir, size_t size) {
    for (size_t i = 0; i != size; ++i) dir[i] = FastDirection<GT, DT>::process(gx[i], gy[i]);
  }

#if (CV_MAJOR_VERSION < 3)
  /// @brief Optimized batch direction computation for OpenCV < 3.
  inline static void process(const float* gx, const float* gy, float* dir, size_t size) {
    cv::fastAtan2(gy, gx, dir, static_cast<int>(size), true);
  }
#endif

  /// @brief Compute fast direction from cv::Mat gradient images.
  ///
  /// Uses cv::phase() for float types for efficiency.
  /// @param gx X-direction gradient matrix
  /// @param gy Y-direction gradient matrix
  /// @param[out] dir Output direction matrix
  inline static void process(const cv::Mat& gx, const cv::Mat& gy, cv::Mat& dir) {
    CV_Assert(gx.type() == gy.type() && (gx.type() & CV_MAT_DEPTH_MASK) == cv::DataType<GT>::type &&
              gx.rows == gy.rows && gx.cols == gy.cols);
    int type = gx.type() & CV_MAT_DEPTH_MASK;
    if ((type == CV_32F || type == CV_64F) && type == cv::DataType<DT>::type)
      cv::phase(gx, gy, dir, true);
    else {
      dir.create(gx.size(), CV_MAKETYPE(cv::DataType<DT>::type, gx.channels()));
      FastDirection<GT, DT>::process(
          gx.ptr<GT>(), gy.ptr<GT>(), dir.ptr<DT>(),
          static_cast<size_t>(gx.rows) * static_cast<size_t>(gx.cols) * static_cast<size_t>(gx.channels()));
    }
  }

  /// @brief Get direction value range.
  /// @return Range [0, 360]
  inline static const DirectionRange& range() {
    static DirectionRange r(0, 360);
    return r;
  }

  /// @brief Get direction method name.
  /// @return "fdir"
  inline static const std::string name() { return "fdir"; }
};
}  // namespace lsfm
