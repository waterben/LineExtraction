//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
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
