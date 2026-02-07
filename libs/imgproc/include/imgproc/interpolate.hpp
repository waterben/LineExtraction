//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file interpolate.hpp
/// @brief Interpolation functions and classes for sub-pixel image access.

#pragma once

#include <geometry/point.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace lsfm {

/// @defgroup interpolation Interpolation Functions and Classes
/// @brief Sub-pixel interpolation for image access.
///
/// This module provides various interpolation methods for accessing image values
/// at non-integer (sub-pixel) positions. Interpolators are template structs that
/// provide static methods for 1D and 2D interpolation with optional boundary handling.
/// @{

// helpers to read data from ptr or mat

/// @brief Read value at position x from source with boundary handling.
/// @tparam MT Matrix element type
/// @param src Source row pointer
/// @param cols Number of columns
/// @param x X coordinate (may be out of bounds)
/// @param border_type OpenCV border type (BORDER_CONSTANT, BORDER_REPLICATE, etc.)
/// @param border_val Value to use for BORDER_CONSTANT
/// @return Interpolated value at x
/// @see http://docs.opencv.org/modules/imgproc/doc/filtering.html
template <class MT>
inline MT readValX(const MT* src, int cols, int x, int border_type, MT border_val = 0) {
  if (x >= 0 && x < cols) return src[x];
  return border_type == cv::BORDER_CONSTANT ? border_val : src[cv::borderInterpolate(x, cols, border_type)];
}

/// @brief Read value at position x from source with replicate border.
/// @tparam MT Matrix element type
/// @param src Source row pointer
/// @param cols Number of columns
/// @param x X coordinate (clamped to [0, cols-1])
/// @return Value at clamped position
template <class MT>
inline MT readValX(const MT* src, int cols, int x) {
  return src[x < 0 ? 0 : x >= cols ? cols - 1 : x];
}

/// @brief Read value at position y from column with boundary handling.
/// @tparam MT Matrix element type
/// @param src Source pointer (column start)
/// @param rows Number of rows
/// @param cols Row stride (number of columns)
/// @param y Y coordinate (may be out of bounds)
/// @param border_type OpenCV border type
/// @param border_val Value to use for BORDER_CONSTANT
/// @return Value at y with boundary handling
template <class MT>
inline MT readValY(const MT* src, int rows, int cols, int y, int border_type, MT border_val = 0) {
  if (y >= 0 && y < rows) return src[y * cols];
  return border_type == cv::BORDER_CONSTANT ? border_val : src[cv::borderInterpolate(y, rows, border_type) * cols];
}

/// @brief Read value at position y from column with replicate border.
/// @tparam MT Matrix element type
/// @param src Source pointer (column start)
/// @param rows Number of rows
/// @param cols Row stride
/// @param y Y coordinate (clamped to [0, rows-1])
/// @return Value at clamped position
template <class MT>
inline MT readValY(const MT* src, int rows, int cols, int y) {
  return src[y < 0 ? 0 : y >= rows ? (rows - 1) * cols : y * cols];
}

/// @brief Read value at position (x, y) from matrix with boundary handling.
/// @tparam MT Matrix element type
/// @param src Source matrix
/// @param y Row coordinate (may be out of bounds)
/// @param x Column coordinate (may be out of bounds)
/// @param border_type OpenCV border type
/// @param border_val Value to use for BORDER_CONSTANT
/// @return Value at (x, y) with boundary handling
template <class MT>
inline MT readVal(const cv::Mat& src, int y, int x, int border_type, MT border_val = 0) {
  if (y >= 0 && y < src.rows && x >= 0 && x < src.cols) return src.at<MT>(y, x);
  return border_type == cv::BORDER_CONSTANT ? border_val
                                            : src.at<MT>(cv::borderInterpolate(y, src.rows, border_type),
                                                         cv::borderInterpolate(x, src.cols, border_type));
}

/// @brief Read value at position (x, y) from matrix with replicate border.
/// @tparam MT Matrix element type
/// @param src Source matrix
/// @param y Row coordinate (clamped)
/// @param x Column coordinate (clamped)
/// @return Value at clamped position
template <class MT>
inline MT readVal(const cv::Mat& src, int y, int x) {
  return src.at<MT>(y < 0 ? 0 : y >= src.rows ? src.rows - 1 : y, x < 0 ? 0 : x >= src.cols ? src.cols - 1 : x);
}

/// @brief Read value at position (x, y) with separate x/y boundary handling.
/// @tparam MT Matrix element type
/// @param src Source matrix
/// @param y Row coordinate
/// @param x Column coordinate
/// @param border_type_x X boundary type
/// @param border_type_y Y boundary type
/// @param border_val_x X boundary value for BORDER_CONSTANT
/// @param border_val_y Y boundary value for BORDER_CONSTANT
/// @return Value at (x, y) with boundary handling
template <class MT>
inline MT readVal(
    const cv::Mat& src, int y, int x, int border_type_x, int border_type_y, MT border_val_x = 0, MT border_val_y = 0) {
  if (y >= 0 && y < src.rows && x >= 0 && x < src.cols) return src.at<MT>(y, x);

  if (border_type_x == cv::BORDER_CONSTANT && (x < 0 || x >= src.cols)) return border_val_x;
  if (border_type_y == cv::BORDER_CONSTANT && (y < 0 || y >= src.rows)) return border_val_y;

  return src.at<MT>(cv::borderInterpolate(y, src.rows, border_type_y),
                    cv::borderInterpolate(x, src.cols, border_type_x));
}

/// @brief Nearest neighbor interpolation (floor).
///
/// Returns the value at the integer position obtained by truncating
/// the floating-point coordinates. Fast but low quality.
/// @tparam FT Floating point type for coordinates
/// @tparam MT Matrix element type
/// @tparam RT Return type (default: FT)
template <class FT, class MT, class RT = FT>
struct NearestInterpolator {
  typedef FT float_type;                 ///< Coordinate type
  typedef MT mat_type;                   ///< Matrix element type
  static constexpr int BorderStart = 0;  ///< Extra border needed at start
  static constexpr int BorderEnd = 0;    ///< Extra border needed at end
  ////////// 1D interpolation

  /// @brief Get 1D interpolation at position fx (no boundary check).
  /// @param src Source row pointer
  /// @param fx Floating point x coordinate
  /// @return Value at floor(fx)
  static inline RT getXNB(const MT* src, FT fx) { return src[static_cast<int>(fx)]; }

  /// @brief Get 1D interpolation at position fy (no boundary check).
  /// @param src Source column pointer
  /// @param cols Row stride
  /// @param fy Floating point y coordinate
  /// @return Value at floor(fy)
  static inline RT getYNB(const MT* src, int cols, FT fy) { return src[static_cast<int>(fy) * cols]; }

  /// @brief Get 1D interpolation at position fx with boundary handling.
  /// @param src Source row pointer
  /// @param cols Number of columns
  /// @param fx Floating point x coordinate
  /// @return Value at floor(fx) with replicate border
  static inline RT getX(const MT* src, int cols, FT fx) { return readValX<MT>(src, cols, static_cast<int>(fx)); }

  /// @brief Get 1D interpolation at position fy with boundary handling.
  /// @param src Source column pointer
  /// @param cols Row stride
  /// @param rows Number of rows
  /// @param fy Floating point y coordinate
  /// @return Value at floor(fy) with replicate border
  static inline RT getY(const MT* src, int cols, int rows, FT fy) {
    return readValY<MT>(src, rows, cols, static_cast<int>(fy));
  }

  /// @brief Get 1D interpolation at fx with custom boundary handling.
  /// @param src Source row pointer
  /// @param cols Number of columns
  /// @param fx Floating point x coordinate
  /// @param border_type OpenCV border type
  /// @param border_val Value for BORDER_CONSTANT
  /// @return Value at floor(fx) with specified boundary handling
  static inline RT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    return readValX<MT>(src, cols, static_cast<int>(fx), border_type, border_val);
  }

  /// @brief Get 1D interpolation at fy with custom boundary handling.
  /// @param src Source column pointer
  /// @param cols Row stride
  /// @param rows Number of rows
  /// @param fy Floating point y coordinate
  /// @param border_type OpenCV border type
  /// @param border_val Value for BORDER_CONSTANT
  /// @return Value at floor(fy) with specified boundary handling
  static inline RT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    return readValY<MT>(src, rows, cols, static_cast<int>(fy), border_type, border_val);
  }

  /// @brief Get 2D interpolation at (fx, fy) without boundary check.
  /// @param src Source matrix
  /// @param fx Floating point x coordinate
  /// @param fy Floating point y coordinate
  /// @return Value at (floor(fx), floor(fy))
  static inline RT getNB(const cv::Mat& src, FT fx, FT fy) {
    return src.at<MT>(static_cast<int>(fy), static_cast<int>(fx));
  }

  /// @brief Get 2D interpolation at point p without boundary check.
  /// @param src Source matrix
  /// @param p Point as array [x, y]
  /// @return Value at floor(p)
  static inline RT getNB(const cv::Mat& src, const FT* p) {
    return NearestInterpolator<FT, MT>::getNB(src, p[0], p[1]);
  }

  /// @brief Get 2D interpolation at point p without boundary check.
  /// @tparam PT Point type with x, y accessors
  /// @param src Source matrix
  /// @param p Point with x, y coordinates
  /// @return Value at floor(p)
  template <class PT>
  static inline RT getNB(const cv::Mat& src, const PT& p) {
    return NearestInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }

  /// @brief Get 2D interpolation at (fx, fy) with replicate boundary.
  /// @param src Source matrix
  /// @param fx Floating point x coordinate
  /// @param fy Floating point y coordinate
  /// @return Value at floor coordinates with boundary handling
  static inline RT get(const cv::Mat& src, FT fx, FT fy) {
    return readVal<MT>(src, static_cast<int>(fy), static_cast<int>(fx));
  }

  /// @brief Get 2D interpolation at point p with replicate boundary.
  /// @param src Source matrix
  /// @param p Point as array [x, y]
  /// @return Value with boundary handling
  static inline RT get(const cv::Mat& src, const FT* p) { return NearestInterpolator<FT, MT>::get(src, p[0], p[1]); }

  /// @brief Get 2D interpolation at point p with replicate boundary.
  /// @tparam PT Point type with x, y accessors
  /// @param src Source matrix
  /// @param p Point with x, y coordinates
  /// @return Value with boundary handling
  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p) {
    return NearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }

  /// @brief Get 2D interpolation with custom boundary handling.
  /// @param src Source matrix
  /// @param fx Floating point x coordinate
  /// @param fy Floating point y coordinate
  /// @param border_type OpenCV border type
  /// @param border_val Value for BORDER_CONSTANT
  /// @return Value with specified boundary handling
  static inline RT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    return static_cast<FT>(readVal<MT>(src, static_cast<int>(fy), static_cast<int>(fx), border_type, border_val));
  }

  /// @brief Get 2D interpolation at point with custom boundary handling.
  /// @param src Source matrix
  /// @param p Point as array [x, y]
  /// @param border_type OpenCV border type
  /// @param border_val Border value
  /// @return Value with boundary handling
  static inline RT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return NearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  /// @brief Get 2D interpolation at point with custom boundary handling.
  /// @tparam PT Point type with x, y accessors
  /// @param src Source matrix
  /// @param p Point
  /// @param border_type OpenCV border type
  /// @param border_val Border value
  /// @return Value with boundary handling
  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return NearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  /// @brief Get 2D interpolation with separate x/y boundary handling.
  /// @param src Source matrix
  /// @param fx Floating point x coordinate
  /// @param fy Floating point y coordinate
  /// @param border_type_x X boundary type
  /// @param border_type_y Y boundary type
  /// @param border_val_x X border value
  /// @param border_val_y Y border value
  /// @return Value with boundary handling
  static inline RT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    return static_cast<FT>(readVal<MT>(src, static_cast<int>(fy), static_cast<int>(fx), border_type_x, border_type_y,
                                       border_val_x, border_val_y));
  }

  /// @brief Get 2D interpolation at point with separate x/y boundary handling.
  /// @param src Source matrix
  /// @param p Point as array [x, y]
  /// @param border_type_x X boundary type
  /// @param border_type_y Y boundary type
  /// @param border_val_x X border value
  /// @param border_val_y Y border value
  /// @return Value with boundary handling
  static inline RT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return NearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x, border_val_y);
  }

  /// @brief Get 2D interpolation at point with separate x/y boundary handling.
  /// @tparam PT Point type with x, y accessors
  /// @param src Source matrix
  /// @param p Point
  /// @param border_type_x X boundary type
  /// @param border_type_y Y boundary type
  /// @param border_val_x X border value
  /// @param border_val_y Y border value
  /// @return Value with boundary handling
  template <class PT>
  static inline RT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return NearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y,
                                            border_val_x, border_val_y);
  }
};

/// @brief Rounded nearest neighbor interpolation.
///
/// Returns the value at the nearest integer position (using std::round).
/// Better quality than NearestInterpolator for centered sampling.
/// @tparam FT Floating point type for coordinates
/// @tparam MT Matrix element type
/// @tparam RT Return type (default: FT)
template <class FT, class MT, class RT = FT>
struct RoundNearestInterpolator {
  typedef FT float_type;  ///< Coordinate type
  typedef MT mat_type;    ///< Matrix element type

  static constexpr int BorderStart = 0;  ///< Extra border needed at start
  static constexpr int BorderEnd = 0;    ///< Extra border needed at end

  static inline RT getXNB(const MT* src, FT fx) { return src[static_cast<int>(round(fx))]; }

  static inline RT getYNB(const MT* src, int cols, FT fy) { return src[static_cast<int>(round(fy)) * cols]; }

  static inline RT getX(const MT* src, int cols, FT fx) { return readValX<MT>(src, cols, static_cast<int>(round(fx))); }

  static inline RT getY(const MT* src, int cols, int rows, FT fy) {
    return readValY<MT>(src, rows, cols, static_cast<int>(round(fy)));
  }

  static inline RT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    return readValX<MT>(src, cols, static_cast<int>(round(fx)), border_type, border_val);
  }

  static inline RT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    return readValY<MT>(src, rows, cols, static_cast<int>(round(fy)), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src, FT fx, FT fy) {
    return readVal<MT>(src, static_cast<int>(round(fy)), static_cast<int>(round(fx)));
  }

  static inline RT get(const cv::Mat& src, const FT* p) {
    return RoundNearestInterpolator<FT, MT>::get(src, p[0], p[1]);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p) {
    return RoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }


  static inline RT getNB(const cv::Mat& src, FT fx, FT fy) {
    return src.at<MT>(static_cast<int>(round(fy)), static_cast<int>(round(fx)));
  }

  static inline RT getNB(const cv::Mat& src, const FT* p) {
    return RoundNearestInterpolator<FT, MT>::getNB(src, p[0], p[1]);
  }

  template <class PT>
  static inline RT getNB(const cv::Mat& src, const PT& p) {
    return RoundNearestInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }


  static inline RT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    return readVal<MT>(src, static_cast<int>(round(fy)), static_cast<int>(round(fx)), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return RoundNearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return RoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    return static_cast<FT>(readVal<MT>(src, static_cast<int>(round(fy)), static_cast<int>(round(fx)), border_type_x,
                                       border_type_y, border_val_x, border_val_y));
  }

  static inline RT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return RoundNearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x,
                                                 border_val_y);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return RoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y,
                                                 border_val_x, border_val_y);
  }
};

/// @brief Fast rounded nearest neighbor interpolation.
///
/// Similar to RoundNearestInterpolator but uses +0.5 instead of std::round
/// for better performance with positive coordinates.
/// @tparam FT Floating point type for coordinates
/// @tparam MT Matrix element type
/// @tparam RT Return type (default: FT)
template <class FT, class MT, class RT = FT>
struct FastRoundNearestInterpolator {
  typedef FT float_type;  ///< Coordinate type
  typedef MT mat_type;    ///< Matrix element type

  static constexpr int BorderStart = 0;  ///< Extra border needed at start
  static constexpr int BorderEnd = 0;    ///< Extra border needed at end

  static inline RT getXNB(const MT* src, FT fx) { return src[static_cast<int>(fx + 0.5)]; }

  static inline RT getYNB(const MT* src, int cols, FT fy) { return src[static_cast<int>(fy + 0.5) * cols]; }

  static inline RT getX(const MT* src, int cols, FT fx) { return readValX<MT>(src, cols, static_cast<int>(fx + 0.5)); }

  static inline RT getY(const MT* src, int cols, int rows, FT fy) {
    return readValY<MT>(src, rows, cols, static_cast<int>(fy + 0.5));
  }

  static inline RT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    return readValX<MT>(src, cols, static_cast<int>(fx + 0.5), border_type, border_val);
  }

  static inline RT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    return readValY<MT>(src, rows, cols, static_cast<int>(fy + 0.5), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src, FT fx, FT fy) {
    return readVal<MT>(src, static_cast<int>(fy + 0.5), static_cast<int>(fx + 0.5));
  }

  static inline RT get(const cv::Mat& src, const FT* p) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, p[0], p[1]);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline RT getNB(const cv::Mat& src, FT fx, FT fy) {
    return src.at<MT>(static_cast<int>(fy + 0.5), static_cast<int>(fx + 0.5));
  }

  static inline RT getNB(const cv::Mat& src, const FT* p) {
    return FastRoundNearestInterpolator<FT, MT>::getNB(src, p[0], p[1]);
  }

  template <class PT>
  static inline RT getNB(const cv::Mat& src, const PT& p) {
    return FastRoundNearestInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline RT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    return readVal<MT>(src, static_cast<int>(fy + 0.5), static_cast<int>(fx + 0.5), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  static inline RT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    return readVal<MT>(src, static_cast<int>(fy + 0.5), static_cast<int>(fx + 0.5), border_type_x, border_type_y,
                       border_val_x, border_val_y);
  }

  static inline RT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x,
                                                     border_val_y);
  }

  template <class PT>
  static inline RT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return FastRoundNearestInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y,
                                                     border_val_x, border_val_y);
  }
};


/// @brief 1D linear interpolation between two values.
/// @tparam FT Floating point result type
/// @tparam MT Input data type
/// @param data Pointer to array of 2 values to be interpolated
/// @param x Position between value 0 and 1 (0 returns data[0], 1 returns data[1])
/// @return Linearly interpolated value
template <class FT, class MT>
inline FT interpolate_linear(const MT* data, FT x) {
  return (static_cast<FT>(data[1]) - static_cast<FT>(data[0])) * x + static_cast<FT>(data[0]);
}

/// @brief 1D linear interpolation using Vec<MT, 2>.
/// @tparam FT Floating point result type
/// @tparam MT Input data type
/// @param data Vector of 2 values
/// @param x Position [0, 1]
/// @return Linearly interpolated value
template <class FT, class MT>
inline FT interpolate_linear(const Vec<MT, 2>& data, FT x) {
  return interpolate_linear<FT, MT>(&getX(data), x);
}

/// @brief 1D linear interpolation between two explicit values.
/// @tparam FT Floating point result type
/// @tparam MT Input data type
/// @param a First value
/// @param b Second value
/// @param x Position [0, 1]
/// @return Linearly interpolated value: (b - a) * x + a
template <class FT, class MT>
inline FT interpolate_linear(MT a, MT b, FT x) {
  return (b - a) * x + a;
}

/// @brief Bilinear interpolation.
///
/// Provides smooth interpolation using weighted average of 4 neighboring pixels.
/// Better quality than nearest neighbor but slower.
/// @tparam FT Floating point type for coordinates and result
/// @tparam MT Matrix element type
template <class FT, class MT>
struct LinearInterpolator {
  typedef FT float_type;  ///< Coordinate type
  typedef MT mat_type;    ///< Matrix element type

  static constexpr int BorderStart = 0;  ///< Extra border needed at start
  static constexpr int BorderEnd = 1;    ///< Extra border needed at end

  static inline FT getXNB(const MT* src, FT fx) {
    int xi = static_cast<int>(fx);
    return interpolate_linear<FT, MT>(src + xi, fx - xi);
  }

  static inline FT getYNB(const MT* src, int cols, FT fy) {
    int yi = static_cast<int>(fy) * cols;
    return interpolate_linear<FT, MT>(src[yi], src[yi + cols], fy - static_cast<int>(fy));
  }

  static inline FT getX(const MT* src, int cols, FT fx) {
    int xi = static_cast<int>(fx);
    return interpolate_linear<FT, MT>(readValX(src, cols, xi), readValX(src, cols, xi + 1), fx - xi);
  }

  static inline FT getY(const MT* src, int cols, int rows, FT fy) {
    int yi = static_cast<int>(fy);
    return interpolate_linear<FT, MT>(readValY(src, rows, cols, yi), readValY(src, rows, cols, yi + 1), fy - yi);
  }

  static inline FT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    int xi = static_cast<int>(fx);
    return interpolate_linear<FT, MT>(readValX(src, cols, xi, border_type, border_val),
                                      readVal(src, cols, xi + 1, border_type, border_val), fx - xi);
  }

  static inline FT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    int yi = static_cast<int>(fy);
    return interpolate_linear<FT, MT>(readValY(src, rows, cols, yi, border_type, border_val),
                                      readVal(src, rows, cols, yi + 1, border_type, border_val), fy - yi);
  }

  static inline FT get(const cv::Mat& src, FT fx, FT fy) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= static_cast<FT>(xi);
    fy -= static_cast<FT>(yi);

    if (fx == 0 || fy == 0) {
      // interpolate with 2 pixels along y
      if (fy != 0) return interpolate_linear<FT, MT>({readVal<MT>(src, yi, xi), readVal<MT>(src, yi + 1, xi)}, fy);

      // interpolate with 2 pixels along x
      if (fx != 0) {
        if (xi >= 0 && xi < src.cols - 1) return interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx);
        return interpolate_linear<FT, MT>({readVal<MT>(src, yi, xi), readVal<MT>(src, yi, xi + 1)}, fx);
      }

      // no interpolation required, get entry from src
      return static_cast<FT>(readVal<MT>(src, yi, xi));
    }

    // interpolate with 4 pixels
    if (xi >= 0 && xi < src.cols - 1 && yi >= 0 && yi < src.rows - 1)
      return interpolate_linear<FT, FT>({interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx),
                                         interpolate_linear<FT, MT>(&src.at<MT>(yi + 1, xi), fx)},
                                        fy);
    return interpolate_linear<FT, FT>(
        {interpolate_linear<FT, MT>({readVal<MT>(src, yi, xi), readVal<MT>(src, yi, xi + 1)}, fx),
         interpolate_linear<FT, MT>({readVal<MT>(src, yi + 1, xi), readVal<MT>(src, yi + 1, xi + 1)}, fx)},
        fy);
  }

  static inline FT get(const cv::Mat& src, const FT* p) { return LinearInterpolator<FT, MT>::get(src, p[0], p[1]); }

  template <class PT>
  static inline FT get(const cv::Mat& src, const PT& p) {
    return LinearInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline FT getNB(const cv::Mat& src, FT fx, FT fy) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);
    fx -= static_cast<FT>(xi);
    fy -= static_cast<FT>(yi);

    /*if (fx == 0 || fy == 0) {
        // interpolate with 2 pixels along y
        if (fy != 0)
            return interpolate_linear<FT, MT>(src.at<MT>(yi, xi), src.at<MT>(yi + 1, xi), fy);

        // interpolate with 2 pixels along x
        if (fx != 0)
            return interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx);

        // no interpolation required, get entry from src
        static_cast<FT>(readVal<MT>(src, yi, xi));
    }*/
    return interpolate_linear<FT, FT>(interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx),
                                      interpolate_linear<FT, MT>(&src.at<MT>(yi + 1, xi), fx), fy);
  }

  static inline FT getNB(const cv::Mat& src, const FT* p) { return LinearInterpolator<FT, MT>::getNB(src, p[0], p[1]); }

  template <class PT>
  static inline FT getNB(const cv::Mat& src, const PT& p) {
    return LinearInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline FT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= static_cast<FT>(xi);
    fy -= static_cast<FT>(yi);

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 2 pixels along y
      if (fy != 0)
        return interpolate_linear<FT, MT>(
            {readVal<MT>(src, yi, xi, border_type, border_val), readVal<MT>(src, yi + 1, xi, border_type, border_val)},
            fy);

      // interpolate with 2 pixels along x
      if (fx != 0) {
        if (xi >= 0 && xi < src.cols - 1) return interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx);
        return interpolate_linear<FT, MT>(
            {readVal<MT>(src, yi, xi, border_type, border_val), readVal<MT>(src, yi, xi + 1, border_type, border_val)},
            fx);
      }

      static_cast<FT>(readVal<MT>(src, yi, xi));
    }


    // interpolate with 4 pixels
    if (xi >= 0 && xi < src.cols - 1 && yi >= 0 && yi < src.rows - 1)
      return interpolate_linear<FT, FT>({interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx),
                                         interpolate_linear<FT, MT>(&src.at<MT>(yi + 1, xi), fx)},
                                        fy);
    return interpolate_linear<FT, FT>(
        {interpolate_linear<FT, MT>(
             {readVal<MT>(src, yi, xi, border_type, border_val), readVal<MT>(src, yi, xi + 1, border_type, border_val)},
             fx),
         interpolate_linear<FT, MT>({readVal<MT>(src, yi + 1, xi, border_type, border_val),
                                     readVal<MT>(src, yi + 1, xi + 1, border_type, border_val)},
                                    fx)},
        fy);
  }

  static inline FT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return LinearInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  template <class PT>
  static inline FT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return LinearInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  static inline FT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= static_cast<FT>(xi);
    fy -= static_cast<FT>(yi);

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 2 pixels along y
      if (fy != 0)
        return interpolate_linear<FT, MT>(
            {readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
             readVal<MT>(src, yi + 1, xi, border_type_x, border_type_y, border_val_x, border_val_y)},
            fy);

      // interpolate with 2 pixels along x
      if (fx != 0) {
        if (xi >= 0 && xi < src.cols - 1) return interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx);
        return interpolate_linear<FT, MT>(
            {readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
             readVal<MT>(src, yi, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y)},
            fx);
      }

      static_cast<FT>(readVal<MT>(src, yi, xi));
    }

    // interpolate with 4 pixels
    if (xi >= 0 && xi < src.cols - 1 && yi >= 0 && yi < src.rows - 1)
      return interpolate_linear<FT, FT>({interpolate_linear<FT, MT>(&src.at<MT>(yi, xi), fx),
                                         interpolate_linear<FT, MT>(&src.at<MT>(yi + 1, xi), fx)},
                                        fy);
    return interpolate_linear<FT, FT>(
        {interpolate_linear<FT, MT>(
             {readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y)},
             fx),
         interpolate_linear<FT, MT>(
             {readVal<MT>(src, yi + 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 1, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y)},
             fx)},
        fy);
  }

  static inline FT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return LinearInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x, border_val_y);
  }

  template <class PT>
  static inline FT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return LinearInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y,
                                           border_val_x, border_val_y);
  }
};


/// @brief 1D cubic interpolation using Catmull-Rom spline.
///
/// Interpolates between 4 sample points with C1 continuity.
/// @see http://www.paulinternet.nl/?page=bicubic
/// @tparam FT Floating point result type
/// @tparam MT Input data type
/// @param data Pointer to array of 4 values to interpolate
/// @param x Position between value 1 and 2 (data[1] to data[2])
/// @return Cubic interpolated value
template <class FT, class MT>
inline FT interpolate_cubic(const MT* data, FT x) {
  return static_cast<FT>(data[1] + 0.5 * x *
                                       (data[2] - data[0] +
                                        x * (2.0 * data[0] - 5.0 * data[1] + 4.0 * data[2] - data[3] +
                                             x * (3.0 * (data[1] - data[2]) + data[3] - data[0]))));
}

/// @brief 1D cubic interpolation using Vec<MT, 4>.
/// @tparam FT Floating point result type
/// @tparam MT Input data type
/// @param data Vector of 4 values
/// @param x Position [0, 1] between data[1] and data[2]
/// @return Cubic interpolated value
template <class FT, class MT>
inline FT interpolate_cubic(const Vec<MT, 4>& data, FT x) {
  return interpolate_cubic(data.val, x);
}

/// @brief 1D cubic interpolation with explicit 4 values.
/// @tparam FT Floating point result type
/// @tparam MT Input data type
/// @param a First value (before interpolation region)
/// @param b Second value (start of interpolation)
/// @param c Third value (end of interpolation)
/// @param d Fourth value (after interpolation region)
/// @param x Position [0, 1] between b and c
/// @return Cubic interpolated value
template <class FT, class MT>
inline FT interpolate_cubic(MT a, MT b, MT c, MT d, FT x) {
  return static_cast<FT>(b + 0.5 * x * (c - a + x * (2.0 * a - 5.0 * b + 4.0 * c - d + x * (3.0 * (b - c) + d - a))));
}

/// @brief Bicubic interpolation using Catmull-Rom splines.
///
/// Uses 16 neighboring pixels (4x4) for smooth interpolation with C1 continuity.
/// Highest quality but slowest of the standard interpolators.
/// @tparam FT Floating point type for coordinates and result
/// @tparam MT Matrix element type
template <class FT, class MT>
struct CubicInterpolator {
  typedef FT float_type;  ///< Coordinate type
  typedef MT mat_type;    ///< Matrix element type

  static constexpr int BorderStart = 1;  ///< Extra border needed at start (1 pixel)
  static constexpr int BorderEnd = 2;    ///< Extra border needed at end (2 pixels)

  static inline FT getXNB(const MT* src, FT fx) {
    int xi = static_cast<int>(fx);
    return interpolate_cubic<FT, MT>(src + xi - 1, fx - xi);
  }

  static inline FT getYNB(const MT* src, int cols, FT fy) {
    int yi = static_cast<int>(fy);
    src += yi * cols;
    return interpolate_cubic<FT, MT>(src[-cols], src[0], src[cols], src[2 * cols], fy - yi);
  }

  static inline FT getX(const MT* src, int cols, FT fx) {
    int xi = static_cast<int>(fx);

    // border case
    if (xi < 1 || xi > cols - 3)
      return interpolate_cubic<FT, MT>(readValX<MT>(src, cols, xi - 1), readValX<MT>(src, cols, xi),
                                       readValX<MT>(src, cols, xi + 1), readValX<MT>(src, cols, xi + 2), fx - xi);

    return interpolate_cubic<FT, MT>(src + xi - 1, fx - xi);
  }

  static inline FT getY(const MT* src, int cols, int rows, FT fy) {
    int yi = static_cast<int>(fy);
    fy -= yi;

    // border case
    if (yi < 1 || yi > rows - 3)
      return interpolate_cubic<FT, MT>(readValY<MT>(src, rows, cols, yi - 1), readValY<MT>(src, rows, cols, yi),
                                       readValY<MT>(src, rows, cols, yi + 1), readValY<MT>(src, rows, cols, yi + 2),
                                       fy);
    src += yi * cols;
    return interpolate_cubic<FT, MT>(src[-cols], src[0], src[cols], src[2 * cols], fy);
  }


  static inline FT getX(const MT* src, int cols, FT fx, int border_type, MT border_val = 0) {
    int xi = static_cast<int>(fx);
    fx -= static_cast<FT>(xi);

    // if (fx == 0)
    //     static_cast<FT>(readValX(src, cols, xi), border_type, border_val);

    // border case
    if (xi < 1 || xi > cols - 3)
      return interpolate_cubic<FT, MT>(readValX<MT>(src, cols, xi - 1, border_type, border_val),
                                       readValX<MT>(src, cols, xi, border_type, border_val),
                                       readValX<MT>(src, cols, xi + 1, border_type, border_val),
                                       readValX<MT>(src, cols, xi + 2, border_type, border_val), fx);

    return interpolate_cubic<FT, MT>(src + xi - 1, fx);
  }

  static inline FT getY(const MT* src, int cols, int rows, FT fy, int border_type, MT border_val = 0) {
    int yi = static_cast<int>(fy);
    fy -= yi;

    // if (fy == 0)
    //     static_cast<FT>(readValY(src, rows, cols, yi), border_type, border_val);

    // border case
    if (yi < 1 || yi > rows - 3)
      return interpolate_cubic<FT, MT>(readValY<MT>(src, rows, cols, yi - 1, border_type, border_val),
                                       readValY<MT>(src, rows, cols, yi, border_type, border_val),
                                       readValY<MT>(src, rows, cols, yi + 1, border_type, border_val),
                                       readValY<MT>(src, rows, cols, yi + 2, border_type, border_val), fy);

    src += yi * cols;
    return interpolate_cubic<FT, MT>(src[-cols], src[0], src[cols], src[2 * cols], fy);
  }

  static inline FT get(const cv::Mat& src, FT fx, FT fy) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= static_cast<FT>(xi);
    fy -= static_cast<FT>(yi);

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 4 pixels along y
      if (fy != 0)
        return interpolate_cubic<FT, MT>(readVal<MT>(src, yi - 1, xi), readVal<MT>(src, yi, xi),
                                         readVal<MT>(src, yi + 1, xi), readVal<MT>(src, yi + 2, xi), fy);


      // interpolate with 4 pixels along x
      if (fx != 0) {
        if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
          // interpolate with 4 pixels along x
          return interpolate_cubic<FT, MT>(readVal<MT>(src, yi, xi - 1), readVal<MT>(src, yi, xi),
                                           readVal<MT>(src, yi, xi + 1), readVal<MT>(src, yi, xi + 2), fx);

        return interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx);
      }

      return static_cast<FT>(readVal<MT>(src, yi, xi));
    }

    // border case
    if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
      return interpolate_cubic<FT, FT>(
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi - 1, xi - 1), readVal<MT>(src, yi - 1, xi),
                                    readVal<MT>(src, yi - 1, xi + 1), readVal<MT>(src, yi - 1, xi + 2), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi, xi - 1), readVal<MT>(src, yi, xi),
                                    readVal<MT>(src, yi, xi + 1), readVal<MT>(src, yi, xi + 2), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi + 1, xi - 1), readVal<MT>(src, yi + 1, xi),
                                    readVal<MT>(src, yi + 1, xi + 1), readVal<MT>(src, yi + 1, xi + 2), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi + 2, xi - 1), readVal<MT>(src, yi + 2, xi),
                                    readVal<MT>(src, yi + 2, xi + 1), readVal<MT>(src, yi + 2, xi + 2), fx),
          fy);

    // interpolate with 16 pixels
    return interpolate_cubic<FT, FT>(interpolate_cubic<FT, MT>(&src.at<MT>(yi - 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 2, xi - 1), fx), fy);
  }

  static inline FT get(const cv::Mat& src, const FT* p) { return CubicInterpolator<FT, MT>::get(src, p[0], p[1]); }

  template <class PT>
  static inline FT get(const cv::Mat& src, const PT& p) {
    return CubicInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline FT getNB(const cv::Mat& src, FT fx, FT fy) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= static_cast<FT>(xi);
    fy -= static_cast<FT>(yi);

    if (fx == 0 || fy == 0) {
      if (fy != 0)
        return interpolate_cubic<FT, MT>(src.at<MT>(yi - 1, xi), src.at<MT>(yi, xi), src.at<MT>(yi + 1, xi),
                                         src.at<MT>(yi + 2, xi), fy);

      if (fx != 0) return interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx);

      return static_cast<FT>(src.at<MT>(yi, xi));
    }

    // interpolate with 16 pixels
    return interpolate_cubic<FT, FT>(interpolate_cubic<FT, MT>(&src.at<MT>(yi - 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 2, xi - 1), fx), fy);
  }

  static inline FT getNB(const cv::Mat& src, const FT* p) { return CubicInterpolator<FT, MT>::getNB(src, p[0], p[1]); }

  template <class PT>
  static inline FT getNB(const cv::Mat& src, const PT& p) {
    return CubicInterpolator<FT, MT>::getNB(src, lsfm::getX(p), lsfm::getY(p));
  }

  static inline FT get(const cv::Mat& src, FT fx, FT fy, int border_type, MT border_val = 0) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= xi;
    fy -= yi;

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 4 pixels along y
      if (fy != 0)
        return interpolate_cubic<FT, MT>(readVal<MT>(src, yi - 1, xi, border_type, border_val),
                                         readVal<MT>(src, yi, xi, border_type, border_val),
                                         readVal<MT>(src, yi + 1, xi, border_type, border_val),
                                         readVal<MT>(src, yi + 2, xi, border_type, border_val), fy);


      // interpolate with 4 pixels along x
      if (fx != 0) {
        // border case
        if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
          return interpolate_cubic<FT, MT>(readVal<MT>(src, yi, xi - 1, border_type, border_val),
                                           readVal<MT>(src, yi, xi, border_type, border_val),
                                           readVal<MT>(src, yi, xi + 1, border_type, border_val),
                                           readVal<MT>(src, yi, xi + 2, border_type, border_val), fx);

        return interpolate_cubic<FT, MT>(src.ptr<MT>(yi) + xi - 1, fx);
      }

      return static_cast<FT>(readVal<MT>(src, yi, xi));
    }

    // border case
    if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
      return interpolate_cubic<FT, FT>(
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi - 1, xi - 1, border_type, border_val),
                                    readVal<MT>(src, yi - 1, xi, border_type, border_val),
                                    readVal<MT>(src, yi - 1, xi + 1, border_type, border_val),
                                    readVal<MT>(src, yi - 1, xi + 2, border_type, border_val), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi, xi - 1, border_type, border_val),
                                    readVal<MT>(src, yi, xi, border_type, border_val),
                                    readVal<MT>(src, yi, xi + 1, border_type, border_val),
                                    readVal<MT>(src, yi, xi + 2, border_type, border_val), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi + 1, xi - 1, border_type, border_val),
                                    readVal<MT>(src, yi + 1, xi, border_type, border_val),
                                    readVal<MT>(src, yi + 1, xi + 1, border_type, border_val),
                                    readVal<MT>(src, yi + 1, xi + 2, border_type, border_val), fx),
          interpolate_cubic<FT, MT>(readVal<MT>(src, yi + 2, xi - 1, border_type, border_val),
                                    readVal<MT>(src, yi + 2, xi, border_type, border_val),
                                    readVal<MT>(src, yi + 2, xi + 1, border_type, border_val),
                                    readVal<MT>(src, yi + 2, xi + 2, border_type, border_val), fx),
          fy);

    // interpolate with 16 pixels
    return interpolate_cubic<FT, FT>(interpolate_cubic<FT, MT>(&src.at<MT>(yi - 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 2, xi - 1), fx), fy);
  }

  static inline FT get(const cv::Mat& src, const FT* p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return CubicInterpolator<FT, MT>::get(src, p[0], p[1], border_type, border_val);
  }

  template <class PT>
  static inline FT get(const cv::Mat& src, const PT& p, int border_type, cv::Scalar border_val = cv::Scalar()) {
    return CubicInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type, border_val);
  }

  static inline FT get(const cv::Mat& src,
                       FT fx,
                       FT fy,
                       int border_type_x,
                       int border_type_y,
                       MT border_val_x = 0,
                       MT border_val_y = 0) {
    int xi = static_cast<int>(fx);
    int yi = static_cast<int>(fy);

    fx -= xi;
    fy -= yi;

    // no interpolation required, get entry from src
    if (fx == 0 || fy == 0) {
      // interpolate with 4 pixels along y
      if (fy != 0)
        return interpolate_cubic<FT, MT>(
            readVal<MT>(src, yi - 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
            readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
            readVal<MT>(src, yi + 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
            readVal<MT>(src, yi + 2, xi, border_type_x, border_type_y, border_val_x, border_val_y), fy);
      // interpolate with 4 pixels along x
      if (fx != 0) {
        if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
          return interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx);

        return interpolate_cubic<FT, MT>(src.ptr<MT>(yi) + xi - 1, fx);
      }

      return static_cast<FT>(readVal<MT>(src, yi, xi));
    }


    // border case
    if (yi < 1 || yi > src.rows - 3 || xi < 1 || xi > src.cols - 3)
      return interpolate_cubic<FT, FT>(
          interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi - 1, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi - 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi - 1, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi - 1, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx),
          interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx),
          interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi + 1, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 1, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 1, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 1, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx),
          interpolate_cubic<FT, MT>(
              readVal<MT>(src, yi + 2, xi - 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 2, xi, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 2, xi + 1, border_type_x, border_type_y, border_val_x, border_val_y),
              readVal<MT>(src, yi + 2, xi + 2, border_type_x, border_type_y, border_val_x, border_val_y), fx),
          fy);

    // interpolate with 16 pixels
    return interpolate_cubic<FT, FT>(interpolate_cubic<FT, MT>(&src.at<MT>(yi - 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 1, xi - 1), fx),
                                     interpolate_cubic<FT, MT>(&src.at<MT>(yi + 2, xi - 1), fx), fy);
  }

  static inline FT get(const cv::Mat& src,
                       const FT* p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return CubicInterpolator<FT, MT>::get(src, p[0], p[1], border_type_x, border_type_y, border_val_x, border_val_y);
  }

  template <class PT>
  static inline FT get(const cv::Mat& src,
                       const PT& p,
                       int border_type_x,
                       int border_type_y,
                       cv::Scalar border_val_x = cv::Scalar(),
                       cv::Scalar border_val_y = cv::Scalar()) {
    return CubicInterpolator<FT, MT>::get(src, lsfm::getX(p), lsfm::getY(p), border_type_x, border_type_y, border_val_x,
                                          border_val_y);
  }
};

/// @brief Interpolator wrapper with fixed border handling.
///
/// Wraps any interpolator with compile-time fixed border type and value,
/// eliminating the need to specify boundary parameters at each call.
/// @tparam Interpolator Base interpolator class
/// @tparam BorderType OpenCV border type (default: cv::BORDER_DEFAULT)
/// @tparam BorderValue Value for BORDER_CONSTANT (default: 0)
template <class Interpolator, int BorderType = cv::BORDER_DEFAULT, int BorderValue = 0>
struct FixedBorderInterpolator {
  typedef typename Interpolator::float_type float_type;  ///< Coordinate type from base
  typedef typename Interpolator::mat_type mat_type;      ///< Matrix type from base


  static inline float_type getX(const mat_type* src, int cols, float_type fx) {
    return Interpolator::getX(src, cols, fx, BorderType, BorderValue);
  }

  static inline float_type getY(const mat_type* src, int rows, float_type fy) {
    return Interpolator::getY(src, rows, fy, BorderType, BorderValue);
  }

  static inline float_type get(const cv::Mat& src, float_type fx, float_type fy) {
    return Interpolator::get(src, fx, fy, BorderType, BorderValue);
  }

  static inline float_type get(const cv::Mat& src, const float_type* p) {
    return Interpolator::get(src, p, BorderType, BorderValue);
  }

  template <template <class> class PT>
  static inline float_type get(const cv::Mat& src, const PT<float_type>& p) {
    return Interpolator::get(src, p, BorderType, BorderValue);
  }
};

/// @brief Interpolator wrapper with separate x/y border handling.
///
/// Wraps any interpolator with compile-time fixed but separate border types
/// and values for x and y axes.
/// @tparam Interpolator Base interpolator class
/// @tparam BorderTypeX X-axis border type
/// @tparam BorderTypeY Y-axis border type
/// @tparam BorderValueX X-axis border value
/// @tparam BorderValueY Y-axis border value
template <class Interpolator,
          int BorderTypeX = cv::BORDER_DEFAULT,
          int BorderTypeY = cv::BORDER_DEFAULT,
          int BorderValueX = 0,
          int BorderValueY = 0>
struct Fixed2BorderInterpolator {
  typedef typename Interpolator::float_type float_type;
  typedef typename Interpolator::mat_type mat_type;

  static inline float_type getX(const mat_type* src, int cols, float_type fx) {
    return Interpolator::getX(src, cols, fx, BorderTypeX, BorderValueX);
  }

  static inline float_type getY(const mat_type* src, int rows, float_type fy) {
    return Interpolator::getY(src, rows, fy, BorderTypeX, BorderValueX);
  }

  static inline float_type get(const cv::Mat& src, float_type fx, float_type fy) {
    return Interpolator::get(src, fx, fy, BorderTypeX, BorderTypeY, BorderValueX, BorderValueY);
  }

  static inline float_type get(const cv::Mat& src, const float_type* p) {
    return Interpolator::get(src, p, BorderTypeX, BorderTypeY, BorderValueX, BorderValueY);
  }

  template <template <class> class PT>
  static inline float_type get(const cv::Mat& src, const PT<float_type>& p) {
    return Interpolator::get(src, p, BorderTypeX, BorderTypeY, BorderValueX, BorderValueY);
  }
};


/// @brief Helper for templated interpolation function pointers.
///
/// Provides type aliases for function pointers to interpolation functions,
/// enabling runtime selection of interpolation methods.
/// @tparam FT Floating point coordinate type
/// @tparam PT Point template type
template <class FT, template <class> class PT>
struct InterpolationHelper {
  typedef FT (*func_type)(const cv::Mat&, FT x, FT y);           ///< Function taking separate x, y
  typedef FT (*func_type_ptr)(const cv::Mat&, const FT*);        ///< Function taking array pointer
  typedef FT (*func_type_point)(const cv::Mat&, const PT<FT>&);  ///< Function taking point
};

/// @}  // end of interpolation group

}  // namespace lsfm
