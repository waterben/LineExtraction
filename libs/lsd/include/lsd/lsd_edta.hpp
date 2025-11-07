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

template <class FT, template <class> class LPT = Vec2>
class LsdEDTA : public LsdBase<FT, LPT> {
 public:
  typedef FT float_type;
  typedef LPT<FT> line_point;
  typedef typename LsdBase<FT, LPT>::Line Line;
  typedef typename LsdBase<FT, LPT>::LineVector LineVector;
  typedef typename LsdBase<FT, LPT>::LineSegment LineSegment;
  typedef typename LsdBase<FT, LPT>::LineSegmentVector LineSegmentVector;
  typedef typename LsdBase<FT, LPT>::ImageData ImageData;

  LsdEDTA() {}

  LsdEDTA(ValueManager::InitializerList options) {}

  LsdEDTA(const ValueManager::NameValueVector& options) {}

  using LsdBase<FT, LPT>::detect;
  using LsdBase<FT, LPT>::lines;
  using LsdBase<FT, LPT>::lineSegments;
  using LsdBase<FT, LPT>::endPoints;
  using LsdBase<FT, LPT>::imageDataDescriptor;
  using LsdBase<FT, LPT>::imageData;

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
