/// @file matlab_helpers_gui.hpp
/// @brief GUI utilities for MATLAB-like functions (requires OpenCV highgui).
///
/// This header provides GUI-dependent utilities like showMat() that require
/// OpenCV's highgui module. It is separated from matlab_helpers.hpp to avoid
/// forcing GUI dependencies on headless builds.
///
/// Include this header only in GUI applications, not in library code.

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <utility/matlab_helpers.hpp>

namespace lsfm {

/// @brief Display a matrix in a window with optional normalization.
/// @param name Window name.
/// @param out Matrix to display.
/// @param normalize Normalization mode (IMG_NORM_FALSE/TRUE/AUTO).
/// @param refresh Whether to call waitKey to refresh display.
///
/// @note This function requires OpenCV highgui module (GUI support).
///       Link against lib_utility_gui in Bazel or add highgui in CMake.
inline void showMat(const std::string& name, const cv::Mat& out, int normalize = IMG_NORM_AUTO, bool refresh = true) {
  if (normalize == IMG_NORM_AUTO)
    normalize = (out.type() == CV_8U || out.channels() > 1) ? IMG_NORM_FALSE : IMG_NORM_TRUE;

  cv::Mat cpy;
  if (normalize == IMG_NORM_TRUE) {
    cpy = normalizeMat(out);
  } else
    cpy = out;

  cv::imshow(name, cpy);
  if (refresh) cv::waitKey(1);
}

}  // namespace lsfm
