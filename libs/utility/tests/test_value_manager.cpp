#include <utility/value_manager.hpp>

#include <gtest/gtest.h>

using lsfm::Value;
using lsfm::ValueManager;

namespace {
struct MyVM : public ValueManager {
  int alpha{0};
  bool beta{false};

  MyVM() {
    add(
        "alpha",
        [this](const Value& v) {
          if (&v != &Value::NAV()) alpha = v.get<int>();
          return Value(alpha);
        },
        "alpha parameter");
    add(
        "beta",
        [this](const Value& v) {
          if (&v != &Value::NAV()) beta = v.get<bool>();
          return Value(beta);
        },
        "beta flag");
  }
};
}  // namespace

TEST(ValueManagerTest, ListAndGet) {
  MyVM vm;
  auto vals = vm.values();
  ASSERT_EQ(vals.size(), 2u);
  EXPECT_EQ(vm.value("alpha").get<int>(), 0);
  EXPECT_EQ(vm.value("beta").get<bool>(), false);
}

TEST(ValueManagerTest, SetByNameAndIndex) {
  MyVM vm;
  vm.value("alpha", Value(5));
  EXPECT_EQ(vm.value("alpha").get<int>(), 5);

  // set by index (index 1 points to second entry)
  vm.value(static_cast<size_t>(1), Value(true));
  EXPECT_TRUE(vm.value("beta").get<bool>());
}

TEST(ValueManagerTest, SetByInitializerList) {
  MyVM vm;
  vm.value({{"alpha", 3}, {"beta", true}});
  EXPECT_EQ(vm.value("alpha").get<int>(), 3);
  EXPECT_TRUE(vm.value("beta").get<bool>());
}
