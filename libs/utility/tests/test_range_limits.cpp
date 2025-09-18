#include <gtest/gtest.h>
#include <utility/range.hpp>
#include <utility/limit.hpp>

using lsfm::Range;
using lsfm::LIMITS;

TEST(RangeTest, SizeAndSwap)
{
  Range<int> r(2, 10);
  EXPECT_EQ(r.size(), 8);
  r.swap();
  EXPECT_EQ(r.lower, 10);
  EXPECT_EQ(r.upper, 2);
  EXPECT_EQ(r.size(), 8);
}

TEST(LimitsTest, TauAndEps)
{
  EXPECT_FLOAT_EQ(LIMITS<float>::tau(), std::numeric_limits<float>::epsilon());
  EXPECT_DOUBLE_EQ(LIMITS<double>::eps(), std::numeric_limits<double>::epsilon());
  EXPECT_DOUBLE_EQ(LIMITS<double>::min(), std::numeric_limits<double>::min());
}

