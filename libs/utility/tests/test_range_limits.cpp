//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_range_limits.cpp
/// @brief Unit tests for range limit utilities.

#include <utility/limit.hpp>
#include <utility/range.hpp>

#include <gtest/gtest.h>

using lsfm::LIMITS;
using lsfm::Range;

TEST(RangeTest, SizeAndSwap) {
  Range<int> r(2, 10);
  EXPECT_EQ(r.size(), static_cast<decltype(r.size())>(8));
  r.swap();
  EXPECT_EQ(r.lower, 10);
  EXPECT_EQ(r.upper, 2);
  EXPECT_EQ(r.size(), static_cast<decltype(r.size())>(8));
}

TEST(LimitsTest, TauAndEps) {
  EXPECT_FLOAT_EQ(LIMITS<float>::tau(), std::numeric_limits<float>::epsilon());
  EXPECT_DOUBLE_EQ(LIMITS<double>::eps(), std::numeric_limits<double>::epsilon());
  EXPECT_DOUBLE_EQ(LIMITS<double>::min(), std::numeric_limits<double>::min());
}
