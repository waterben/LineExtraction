#include <imgproc/filter.hpp>
#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

using namespace lsfm;
using namespace cv;

// Mock filter implementation for testing
class MockFilter : public FilterI<float> {
 public:
  MockFilter() : processed_(false), last_img_size_(), result_data_() {}

  IntensityRange intensityRange() const override { return IntensityRange(0.0f, 255.0f); }

  void process(const cv::Mat& img) override {
    processed_ = true;
    last_img_size_ = img.size();
    // Simple filter: just copy the image
    result_data_ = img.clone();
  }

  FilterResults results() const override {
    FilterResults results;
    if (processed_) {
      results["output"] = FilterData(result_data_, intensityRange());
    }
    return results;
  }

  std::string name() const override { return "MockFilter"; }

  bool wasProcessed() const { return processed_; }
  Size getLastImageSize() const { return last_img_size_; }

 private:
  bool processed_;
  Size last_img_size_;
  Mat result_data_;
};

class FilterTest : public ::testing::Test {
 protected:
  FilterTest() : test_img(), filter() {}

  void SetUp() override {
    // Create test image
    test_img = Mat::zeros(5, 5, CV_32F);
    for (int y = 0; y < 5; ++y) {
      for (int x = 0; x < 5; ++x) {
        test_img.at<float>(y, x) = static_cast<float>(y * 5 + x);
      }
    }

    filter = std::make_unique<MockFilter>();
  }

  Mat test_img;
  std::unique_ptr<MockFilter> filter;
};

TEST_F(FilterTest, FilterDataConstruction) {
  // Test default construction
  FilterData default_data;
  EXPECT_TRUE(default_data.data.empty());
  EXPECT_EQ(default_data.range.lower, 0.0);
  EXPECT_EQ(default_data.range.upper, 0.0);

  // Test construction with Mat and range
  Mat test_mat = Mat::ones(3, 3, CV_32F);
  FilterData data_with_mat(test_mat, 0.5, 1.5);
  EXPECT_FALSE(data_with_mat.data.empty());
  EXPECT_EQ(data_with_mat.data.rows, 3);
  EXPECT_EQ(data_with_mat.data.cols, 3);
  EXPECT_DOUBLE_EQ(data_with_mat.range.lower, 0.5);
  EXPECT_DOUBLE_EQ(data_with_mat.range.upper, 1.5);
}

TEST_F(FilterTest, FilterDataWithRange) {
  Mat test_mat = Mat::ones(2, 2, CV_32F);
  lsfm::Range<float> float_range(1.0f, 10.0f);
  FilterData data_with_range(test_mat, float_range);

  EXPECT_FALSE(data_with_range.data.empty());
  EXPECT_DOUBLE_EQ(data_with_range.range.lower, 1.0);
  EXPECT_DOUBLE_EQ(data_with_range.range.upper, 10.0);
}

TEST_F(FilterTest, FilterInterface) {
  // Test basic filter interface
  EXPECT_EQ(filter->name(), "MockFilter");

  // Test intensity range
  auto range = filter->intensityRange();
  EXPECT_FLOAT_EQ(range.lower, 0.0f);
  EXPECT_FLOAT_EQ(range.upper, 255.0f);

  // Test initial state
  EXPECT_FALSE(filter->wasProcessed());

  // Test processing
  filter->process(test_img);
  EXPECT_TRUE(filter->wasProcessed());
  EXPECT_EQ(filter->getLastImageSize(), test_img.size());
}

TEST_F(FilterTest, FilterResults) {
  // Initially no results
  auto results = filter->results();
  EXPECT_TRUE(results.empty());

  // After processing, should have results
  filter->process(test_img);
  results = filter->results();
  EXPECT_FALSE(results.empty());
  EXPECT_EQ(results.size(), static_cast<size_t>(1));

  // Check result content
  auto it = results.find("output");
  EXPECT_NE(it, results.end());

  const FilterData& output_data = it->second;
  EXPECT_FALSE(output_data.data.empty());
  EXPECT_EQ(output_data.data.size(), test_img.size());
}

TEST_F(FilterTest, FilterResultsMap) {
  // Test FilterResults as std::map
  FilterResults results;

  Mat mat1 = Mat::ones(2, 2, CV_32F);
  Mat mat2 = Mat::zeros(3, 3, CV_32F);

  results["result1"] = FilterData(mat1, 0.0, 1.0);
  results["result2"] = FilterData(mat2, -1.0, 1.0);

  EXPECT_EQ(results.size(), static_cast<size_t>(2));
  EXPECT_NE(results.find("result1"), results.end());
  EXPECT_NE(results.find("result2"), results.end());
  EXPECT_EQ(results.find("nonexistent"), results.end());

  // Test result content
  EXPECT_EQ(results["result1"].data.rows, 2);
  EXPECT_EQ(results["result2"].data.rows, 3);
  EXPECT_DOUBLE_EQ(results["result1"].range.upper, 1.0);
  EXPECT_DOUBLE_EQ(results["result2"].range.lower, -1.0);
}

TEST_F(FilterTest, IntensityRangeTypes) {
  // Test different intensity range types
  using UCharFilter = FilterI<unsigned char>;
  using IntFilter = FilterI<int>;
  using DoubleFilter = FilterI<double>;

  // Check type definitions compile correctly
  UCharFilter::IntensityRange uchar_range(0, 255);
  IntFilter::IntensityRange int_range(-1000, 1000);
  DoubleFilter::IntensityRange double_range(-1.0, 1.0);

  EXPECT_EQ(uchar_range.lower, 0);
  EXPECT_EQ(uchar_range.upper, 255);
  EXPECT_EQ(int_range.size(), static_cast<decltype(int_range.size())>(2000));
  EXPECT_DOUBLE_EQ(double_range.size(), 2.0);
}

TEST_F(FilterTest, EmptyImageProcessing) {
  Mat empty_img;
  filter->process(empty_img);

  EXPECT_TRUE(filter->wasProcessed());
  EXPECT_EQ(filter->getLastImageSize().width, 0);
  EXPECT_EQ(filter->getLastImageSize().height, 0);
}
