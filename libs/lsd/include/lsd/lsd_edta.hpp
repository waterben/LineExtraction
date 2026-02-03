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

#include <lsd/lsd_base.hpp>

struct LS {
  double sx{}, sy{}, ex{}, ey{};  // Start & end coordinates of the line segment
};

// from EDLinesLib.lib
LS* DetectLinesByED(unsigned char* srcImg, int width, int height, int* pNoLS);

namespace lsfm {

/// @brief Wrapper for the original ED (Edge Drawing) algorithm.
/// This class provides a simple wrapper around the classic Edge Drawing algorithm,
/// which is the basis for the more advanced EDLines algorithm.
/// This implementation calls the C-based ED detection library.
///
/// **Note:** Deprecated in favor of LsdEDLZ (EDLines) which provides better features.
///
/// @tparam FT Floating-point type (float, double)
/// @tparam LPT Line point template (default Vec2)
template <class FT, template <class> class LPT = Vec2>
class LsdEDTA : public LsdBase<FT, LPT> {
 public:
  typedef FT float_type;                                                   ///< Floating-point type
  typedef LPT<FT> line_point;                                              ///< Line point type
  typedef typename LsdBase<FT, LPT>::Line Line;                            ///< Line type
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;                ///< Vector of lines
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;              ///< Line segment type
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;  ///< Vector of segments
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;                  ///< Image data type

  /// @brief Create an ED (Edge Drawing) detector.
  /// @deprecated Use LsdEDLZ (EDLines) instead, which provides better performance and features.
  LsdEDTA() {}

  /// @brief Create an ED detector from initializer list.
  /// @param options Initializer list with parameter name/value pairs
  /// @deprecated Use LsdEDLZ (EDLines) instead
  LsdEDTA(ValueManager::InitializerList options) {}

  /// @brief Create an ED detector from parameter vector.
  /// @param options Vector with parameter name/value pairs
  /// @deprecated Use LsdEDLZ (EDLines) instead
  LsdEDTA(const ValueManager::NameValueVector& options) {}

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;
  using LsdBase<FT, LPT>::imageDataDescriptor;
  using LsdBase<FT, LPT>::imageData;

  /// @brief Detect line segments using the Edge Drawing algorithm.
  /// @param image Input image (8-bit single-channel or will be converted)
  virtual void detect(const cv::Mat& image) final {
    cv::Mat img = image;
    CV_Assert(!img.empty());

    if (img.channels() != 1) cvtColor(img, img, cv::COLOR_BGR2GRAY);

    // Convert image uchar
    if (img.type() != CV_8U) img.convertTo(img, CV_8U);

    clearData();

    int lnum;
    LS* line = DetectLinesByED(img.ptr<uchar>(), img.cols, img.rows, &lnum);

    lineSegments_.reserve(lnum);
    for (int i = 0; i != lnum; ++i) {
      lineSegments_.push_back(LineSegment(LPT<FT>(static_cast<FT>(line[i].sx), static_cast<FT>(line[i].sy)),
                                          LPT<FT>(static_cast<FT>(line[i].ex), static_cast<FT>(line[i].ey))));
    }
  }
};

}  // namespace lsfm
