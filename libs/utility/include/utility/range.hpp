//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file range.hpp
/// @brief Simple range template for lower/upper bounds.

#pragma once

#include <cmath>
#include <type_traits>

namespace lsfm {

/// @brief Simple range class storing lower and upper bounds.
/// @tparam T Numeric type for bounds.
template <class T>
struct Range {
  /// @brief Construct range with given bounds.
  /// @param l Lower bound.
  /// @param u Upper bound.
  Range(T l = 0, T u = 0) : lower(l), upper(u) {}

  /// @brief Swap lower and upper bounds.
  void swap() { std::swap(lower, upper); }

  /// @brief Get absolute size of range.
  /// @return Absolute difference between upper and lower.
  T size() const { return (upper >= lower) ? (upper - lower) : (lower - upper); }

  T lower;  ///< Lower bound.
  T upper;  ///< Upper bound.
};
}  // namespace lsfm
