//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_options.cpp
/// @brief Unit tests for options utilities.

#include <utility/options.hpp>

#include <gtest/gtest.h>

using utility::Options;

TEST(OptionsTest, ParseLongAndShort) {
  std::string input;
  bool verbose = false;
  Options opts;
  opts.add_string("input", 'i', "Input file", input, true);
  opts.add_switch("verbose", 'v', "Verbose", verbose);

  const char* argv[] = {"app", "--verbose", "-i", "file.txt"};
  auto rest = opts.parse(4, const_cast<char**>(argv));
  EXPECT_TRUE(verbose);
  EXPECT_EQ(input, std::string("file.txt"));
  EXPECT_TRUE(rest.empty());
}

TEST(OptionsTest, EqualsSyntaxAndDefaults) {
  std::string out = "default.txt";
  bool flag = false;
  Options opts;
  opts.add_string("output", 'o', "Output file", out, false, out);
  opts.add_switch("flag", 'f', "Flag", flag);

  const char* argv[] = {"app", "--output=result.bin", "pos1", "-f", "pos2"};
  auto rest = opts.parse(5, const_cast<char**>(argv));
  ASSERT_EQ(rest.size(), 2u);
  EXPECT_EQ(rest[0], std::string("pos1"));
  EXPECT_EQ(rest[1], std::string("pos2"));
  EXPECT_TRUE(flag);
  EXPECT_EQ(out, std::string("result.bin"));
}

TEST(OptionsTest, MissingRequiredThrows) {
  std::string in;
  Options opts;
  opts.add_string("input", 'i', "Input file", in, true);
  const char* argv[] = {"app"};
  EXPECT_THROW(opts.parse(1, const_cast<char**>(argv)), utility::options_error);
}
