/// @file threshold.cpp
/// @brief Threshold performance tests for various thresholding methods
#include "performance_test.hpp"
#include <edge/nms.hpp>
#include <edge/threshold.hpp>
#include <eval/results.hpp>
#include <imgproc/derivative_gradient.hpp>

#include <filesystem>


using namespace lsfm;
namespace fs = std::filesystem;


// =============================================================================
// Threshold Performance Tasks
// =============================================================================

/// @brief Generic threshold performance task wrapper
template <class FT>
class Entry : public CVPerformanceTaskBase {
  cv::Ptr<FilterI<uchar>> filter_;
  cv::Ptr<Threshold<FT>> threshold_;
  FilterResults filterRes_;
  cv::Mat thresholdRes_;
  std::string resName_;

 public:
  Entry() : CVPerformanceTaskBase("") {}

  Entry(cv::Ptr<FilterI<uchar>> filter, cv::Ptr<Threshold<FT>> threshold, const std::string& n, int flags = 0)
      : CVPerformanceTaskBase(n, flags), filter_(std::move(filter)), threshold_(std::move(threshold)) {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    resName_ = "./results/visual/Threshold/" + currentData().name;
    resName_ = resName_.substr(0, resName_.size() - 4) + "_" + this->name;
    filter_->process(src);
    filterRes_ = filter_->results();
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    cv::Mat mag = filterRes_["mag"].data;
    thresholdRes_ = threshold_->process(mag);
  }

 public:
  void saveResults() override {
    NonMaximaSuppression<short, FT, FT> nms;
    cv::Mat high = thresholdRes_.clone();
    cv::GaussianBlur(high, high, cv::Size(0, 0), 15);
    cv::Mat low = high * 0.5;
    // set fixed lower threshold
    low.setTo(filterRes_["mag"].range.upper * 0.004, low < filterRes_["mag"].range.upper * 0.004);
    nms.process(filterRes_["gx"].data, filterRes_["gy"].data, filterRes_["mag"].data, low, high);
    std::cout << "    Save visual results " << resName_ << std::endl;
    saveEdge(nms.hysteresis(), resName_ + "_mag");
    saveNormalized(high, resName_ + "_th");
  }
};


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create threshold performance test with all tasks
CVPerformanceTestPtr createThresholdPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "Threshold");

  fs::create_directory("./results/visual/Threshold");

  using FT = float;

  test->tasks.push_back(std::make_shared<Entry<FT>>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                                                    new GlobalThreshold<FT, ThresholdOtsu<FT, 256>>(1141), "Otsu_G"));

  test->tasks.push_back(std::make_shared<Entry<FT>>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                                                    new LocalThresholdTiles<FT, ThresholdOtsu<FT, 256>>(3, 3, 1141),
                                                    "Otsu_LTiles4"));

  test->tasks.push_back(std::make_shared<Entry<FT>>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                                                    new LocalThresholdTiles<FT, ThresholdOtsu<FT, 256>>(10, 10, 1141),
                                                    "Otsu_LTiles10"));

  test->tasks.push_back(std::make_shared<Entry<FT>>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                                                    new LocalThreshold<FT, ThresholdOtsu<FT, 256>>(30, 30, true, 1141),
                                                    "Otsu_LWindow"));

  test->tasks.push_back(std::make_shared<Entry<FT>>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                                                    new DynamicThreshold<FT, ThresholdOtsu<FT, 256>>(5, 1141),
                                                    "Otsu_D"));

  return test;
}

bool addThreshold() {
  addPerformanceTestCreator(createThresholdPerformanceTest);
  std::cout << "Added threshold performance test" << std::endl;
  return true;
}

// bool thresholdAdded = addThreshold();
