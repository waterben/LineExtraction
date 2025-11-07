#include <eval/data_provider.hpp>
#include <eval/task.hpp>

#include <gtest/gtest.h>

namespace {

struct TestData : public lsfm::GenericInputData {
  TestData(const std::string& n, int v) : GenericInputData{n}, value(v) {}
  int value;
};

struct TestDataProvider : public lsfm::DataProvider<TestData> {
  TestDataProvider(const std::string& provider_name) : DataProvider(provider_name), data_(), pos_(0) {
    // Add some test data
    data_.emplace_back("item1", 10);
    data_.emplace_back("item2", 20);
    data_.emplace_back("item3", 30);
  }

  bool get(TestData& data) override {
    if (pos_ >= data_.size()) {
      return false;
    }
    data = data_[pos_++];
    return true;
  }

  void rewind() override { pos_ = 0; }

  void clear() override {
    data_.clear();
    pos_ = 0;
  }

 private:
  std::vector<TestData> data_;
  std::size_t pos_;
};

}  // namespace

TEST(DataProviderTest, ConstructorSetsName) {
  TestDataProvider provider("test_provider");
  EXPECT_EQ(provider.name, "test_provider");
}

TEST(DataProviderTest, GetData) {
  TestDataProvider provider("test_provider");
  TestData data("", 0);

  // Get first item
  EXPECT_TRUE(provider.get(data));
  EXPECT_EQ(data.name, "item1");
  EXPECT_EQ(data.value, 10);

  // Get second item
  EXPECT_TRUE(provider.get(data));
  EXPECT_EQ(data.name, "item2");
  EXPECT_EQ(data.value, 20);

  // Get third item
  EXPECT_TRUE(provider.get(data));
  EXPECT_EQ(data.name, "item3");
  EXPECT_EQ(data.value, 30);

  // No more data
  EXPECT_FALSE(provider.get(data));
}

TEST(DataProviderTest, Rewind) {
  TestDataProvider provider("test_provider");
  TestData data("", 0);

  // Consume all data
  provider.get(data);
  provider.get(data);
  provider.get(data);
  EXPECT_FALSE(provider.get(data));

  // Rewind and get first item again
  provider.rewind();
  EXPECT_TRUE(provider.get(data));
  EXPECT_EQ(data.name, "item1");
  EXPECT_EQ(data.value, 10);
}

TEST(DataProviderTest, Clear) {
  TestDataProvider provider("test_provider");
  TestData data("", 0);

  // Initially has data
  EXPECT_TRUE(provider.get(data));

  // Clear should remove all data
  provider.clear();
  EXPECT_FALSE(provider.get(data));
}

TEST(DataProviderTest, SharedPointer) {
  auto provider = std::make_shared<TestDataProvider>("shared_provider");
  EXPECT_EQ(provider->name, "shared_provider");

  lsfm::DataProvider<TestData>::PtrList providers;
  providers.push_back(provider);

  EXPECT_EQ(providers.size(), static_cast<size_t>(1));
  EXPECT_EQ(providers[0]->name, "shared_provider");
}
