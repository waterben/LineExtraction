//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann & Manuel Lange
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file vision.hpp
/// @brief Computer vision utilities for camera decomposition.
///
/// Provides utility functions for decomposing camera projection matrices
/// using OpenCV's calibration functions.

#pragma once

#include <geometry/base.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace lsfm {

/// @brief Decompose camera projection matrix into intrinsics and extrinsics.
///
/// Extracts camera matrix (intrinsics), rotation matrix, and translation
/// from a 3x4 projection matrix using OpenCV's decomposeProjectionMatrix.
/// @tparam FT Floating-point type.
/// @param proj 3x4 projection matrix P = K[R|t].
/// @param cam Output 3x3 camera intrinsic matrix K.
/// @param trans Output translation vector (camera position in world).
/// @param rot Output 3x3 rotation matrix R.
template <class FT>
inline void decomposeProjectionMatrix(const Matx34<FT>& proj, Matx33<FT>& cam, Vec3<FT>& trans, Matx33<FT>& rot) {
  cv::Vec<FT, 4> tmp;
  cv::decomposeProjectionMatrix(cv::Mat(3, 4, cv::DataType<FT>::type, const_cast<FT*>(&proj[0])),
                                cv::Mat(3, 3, cv::DataType<FT>::type, &cam[0]),
                                cv::Mat(3, 3, cv::DataType<FT>::type, &rot[0]), tmp);
  rot.transposeInPlace();
  trans = Vec3<FT>(tmp[0] / tmp[3], tmp[1] / tmp[3], tmp[2] / tmp[3]);
}

}  // namespace lsfm
