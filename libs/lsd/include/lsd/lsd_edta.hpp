//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file lsd_edta.hpp
/// @brief Wrapper for the original ED (Edge Drawing) algorithm.

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
