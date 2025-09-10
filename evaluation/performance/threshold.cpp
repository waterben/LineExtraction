#include "performance_test.hpp"
#include <filesystem>
#include <edge/nms.hpp>
#include <edge/threshold.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <utility/results.hpp>


using namespace lsfm;
namespace fs = std::filesystem;

template <class FT>
struct Entry : public PerformanceTaskDefault {
  Entry() {}

  Entry(const cv::Ptr<FilterI<uchar>>& a, const cv::Ptr<Threshold<FT>>& t, const std::string& n, int f = 0)
      : PerformanceTaskDefault(n, f), filter(a), threshold(t) {}


  cv::Ptr<FilterI<uchar>> filter;
  cv::Ptr<Threshold<FT>> threshold;

  FilterResults filterRes;
  cv::Mat thresholdRes;
  std::string resName;


  virtual void run(const std::string& src_name, const cv::Mat& src, int loops, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    resName = "./results/visual/Threshold/" + src_name.substr(0, src_name.size() - 4) + "_" + this->name;
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    filter->process(src);
    filterRes = filter->results();
    cv::Mat mag = filterRes["mag"].data;
    uint64 start;
    for (int i = 0; i != loops; ++i) {
      start = cv::getTickCount();
      thresholdRes = threshold->process(mag);
      pm.measures.push_back(cv::getTickCount() - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((cv::getTickCount() - start) * 1000) / (loops * cv::getTickFrequency()) << "ms"
                << std::endl;
  }

  void saveResults(bool verbose) {
    NonMaximaSuppression<short, FT, FT> nms;
    cv::Mat high = thresholdRes.clone();
    cv::GaussianBlur(high, high, cv::Size(0, 0), 15);
    cv::Mat low = high * 0.5;
    // set fixed lower threshold
    low.setTo(filterRes["mag"].range.upper * 0.004, low < filterRes["mag"].range.upper * 0.004);
    nms.process(filterRes["gx"].data, filterRes["gy"].data, filterRes["mag"].data, low, high);
    if (verbose) std::cout << "    Save visual results " << resName << std::endl;
    saveEdge(nms.hysteresis(), resName + "_mag");
    saveNormalized(high, resName + "_th");
  }
};


PerformanceTestPtr createThresholdPerformanceTest(const lsfm::DataProviderList& provider) {
  auto test = std::make_shared<PerformanceTest>();
  test->name = "Threshold";
  try {
    test->data = provider;

    // add other
  } catch (std::exception& e) {
    std::cout << test->name << " parse error: " << e.what() << std::endl;
    return PerformanceTestPtr();
  }

  fs::create_directory("./results/visual/Threshold");

  typedef float FT;

  test->tasks.push_back(
      PerformanceTaskPtr(new Entry<FT>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                                       new GlobalThreshold<FT, ThresholdOtsu<FT, 256>>(1141), "Otsu_G")));
  test->tasks.push_back(PerformanceTaskPtr(
      new Entry<FT>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                    new LocalThresholdTiles<FT, ThresholdOtsu<FT, 256>>(3, 3, 1141), "Otsu_LTiles4")));
  test->tasks.push_back(PerformanceTaskPtr(
      new Entry<FT>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                    new LocalThresholdTiles<FT, ThresholdOtsu<FT, 256>>(10, 10, 1141), "Otsu_LTiles10")));
  test->tasks.push_back(PerformanceTaskPtr(
      new Entry<FT>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                    new LocalThreshold<FT, ThresholdOtsu<FT, 256>>(30, 30, true, 1141), "Otsu_LWindow")));
  test->tasks.push_back(
      PerformanceTaskPtr(new Entry<FT>(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>,
                                       new DynamicThreshold<FT, ThresholdOtsu<FT, 256>>(5, 1141), "Otsu_D")));
  return test;
}

bool addThreshold() {
  addPerformanceTestCreator(createThresholdPerformanceTest);
  std::cout << "Added threshold performance test" << std::endl;
  return true;
}

// bool thresholdAdded = addThreshold();
