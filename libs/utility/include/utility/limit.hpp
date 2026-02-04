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
// C by Benjamin Wassermann
//M*/

/// @file limit.hpp
/// @brief Numeric limit constants for floating-point types.

#pragma once

#include <limits>

namespace lsfm {

/// @brief Numeric constants for floating-point computations.
/// @tparam FT Floating-point type (float or double).
template <class FT>
struct LIMITS {
  /// @brief Small threshold for comparisons (1e-7 for double).
  static inline FT tau() { return static_cast<FT>(1.0E-7); }

  /// @brief Minimum positive value.
  static inline FT min() { return std::numeric_limits<FT>::min(); }

  /// @brief Machine epsilon.
  static inline FT eps() { return std::numeric_limits<FT>::epsilon(); }
};

/// @brief Specialization for float with appropriate tau value.
template <>
struct LIMITS<float> {
  /// @brief Small threshold (uses epsilon for float since eps < 1e-5).
  static inline float tau() { return std::numeric_limits<float>::epsilon(); }

  /// @brief Minimum positive value.
  static inline float min() { return std::numeric_limits<float>::min(); }

  /// @brief Machine epsilon.
  static inline float eps() { return std::numeric_limits<float>::epsilon(); }
};

}  // namespace lsfm
