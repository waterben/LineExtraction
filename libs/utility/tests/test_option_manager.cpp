#include <utility/option_manager.hpp>

#include <gtest/gtest.h>

using lsfm::OptionManager;

namespace {
struct MyOM : public OptionManager {
  MyOM() : calls() {}
  explicit MyOM(const OptionManager::OptionVector& v) : OptionManager(v), calls() {}
  std::vector<std::pair<std::string, double>> calls;
  void setOptionImpl(const std::string& name, double value) override { calls.emplace_back(name, value); }
};
}  // namespace

TEST(OptionManagerTest, GetSetOptions) {
  OptionManager::OptionVector v = {{"alpha", 1.0, "float", "alpha value"}, {"beta", 0.0, "bool", "beta value"}};
  MyOM om(v);

  auto e = om.getOption("alpha");
  EXPECT_EQ(e.name, std::string("alpha"));
  EXPECT_DOUBLE_EQ(e.value, 1.0);
  EXPECT_EQ(om.getOption("missing").name, std::string());

  om.setOption("alpha", 2.0);
  ASSERT_EQ(om.calls.size(), 1u);
  EXPECT_EQ(om.calls[0].first, std::string("alpha"));
  EXPECT_DOUBLE_EQ(om.calls[0].second, 2.0);

  om.setOptions(v);  // triggers setOptionImpl twice
  ASSERT_EQ(om.calls.size(), 3u);
}

TEST(OptionManagerTest, OptionEntryGetSet) {
  OptionManager::OptionEntry e("gamma", 5.0, "float", "desc");
  EXPECT_EQ(e.get<int>(), 5);
  e.set<int>(7);
  EXPECT_DOUBLE_EQ(e.value, 7.0);
}
