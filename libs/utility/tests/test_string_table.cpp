#include <utility/string_table.hpp>

#include <gtest/gtest.h>

#include <fstream>

using lsfm::StringTable;

TEST(StringTableTest, AccessAndRowCol) {
  StringTable t(2, 3);
  t(0, 0) = "a";
  t(0, 1) = "b";
  t(0, 2) = "c";
  t(1, 0) = "d";
  t(1, 1) = "e";
  t(1, 2) = "f";

  auto r0 = t.row(0);
  ASSERT_EQ(r0.size(), 3u);
  EXPECT_EQ(r0[0], "a");
  EXPECT_EQ(r0[2], "c");

  auto c1 = t.col(1);
  ASSERT_EQ(c1.size(), 2u);
  EXPECT_EQ(c1[0], "b");
  EXPECT_EQ(c1[1], "e");
}

TEST(StringTableTest, Transpose) {
  StringTable t(2, 3);
  int v = 0;
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 3; j++) t(i, j) = std::to_string(++v);
  auto tt = t.transpose();
  EXPECT_EQ(tt.rows(), 3u);
  EXPECT_EQ(tt.cols(), 2u);
  EXPECT_EQ(tt(0, 0), t(0, 0));
  EXPECT_EQ(tt(2, 1), t(1, 2));
}

TEST(StringTableTest, SaveCSV) {
  StringTable t(2, 2);
  t(0, 0) = "foo";
  t(0, 1) = "bar";
  t(1, 0) = "baz";
  t(1, 1) = "qux";
  std::string filename = "test_string_table.csv";
  t.saveCSV(filename);

  std::ifstream ifs(filename);
  ASSERT_TRUE(ifs.is_open());
  std::string line;
  std::getline(ifs, line);
  EXPECT_EQ(line, "foo;bar;");
  std::getline(ifs, line);
  EXPECT_EQ(line, "baz;qux;");
  ifs.close();
  std::remove(filename.c_str());
}

TEST(StringTableTest, OperatorAccess) {
  StringTable t(3, 2);
  t(0, 0) = "x";
  t(2, 1) = "y";
  EXPECT_EQ(t(0, 0), "x");
  EXPECT_EQ(t(2, 1), "y");
}

TEST(StringTableTest, EmptyTable) {
  StringTable t(0, 0);
  EXPECT_EQ(t.rows(), 0u);
  EXPECT_EQ(t.cols(), 0u);
  EXPECT_TRUE(t.row(0).empty());
  EXPECT_TRUE(t.col(0).empty());
}
