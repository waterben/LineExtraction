#include <gtest/gtest.h>
#include <utility/format.hpp>
#include <string>

TEST(FormatTest, IntegerPadding)
{
  EXPECT_EQ(utility::format("%04d", 7), "0007");
  EXPECT_EQ(utility::format("%2d", 3), " 3");
}

TEST(FormatTest, FloatingPrecision)
{
  EXPECT_EQ(utility::format("%.3f", 1.23456), std::string("1.235"));
  EXPECT_EQ(utility::format("%.2f", 3.1), std::string("3.10"));
}

TEST(FormatTest, StringAndPercent)
{
  EXPECT_EQ(utility::format("Hello %s", std::string("World")), std::string("Hello World"));
  EXPECT_EQ(utility::format("50%% done"), std::string("50% done"));
}

TEST(FormatTest, Mixed)
{
  std::string s = utility::format("%s-%04d-%.2f", "id", 12, 3.14159);
  EXPECT_EQ(s, std::string("id-0012-3.14"));
}

