/// @file line_fit.cpp
/// @brief Line fitting performance tests

#include "performance_test.hpp"
#include <edge/edge_linking.hpp>
#include <edge/fit.hpp>
#include <edge/nms.hpp>
#include <edge/spe.hpp>
#include <edge/split.hpp>
#include <imgproc/derivative_gradient.hpp>

#include <memory>

using namespace lsfm;


using Grad = DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>;
using Nms = NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>>;
using Edge = EsdLinking<int>;
using PointVector = std::vector<cv::Point>;

constexpr float TH_LOW = 0.004F;
constexpr float TH_HIGH = 0.012F;


// =============================================================================
// Line Fitting Performance Tasks
// =============================================================================

/// @brief Generic line fitting performance task
template <class FIT>
class Entry : public CVPerformanceTaskBase {
  FIT fit_;
  Grad grad_{};
  Nms nms_{};
  std::unique_ptr<Edge> edge_{};
  PointVector points_{};

 public:
  Entry(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
      : CVPerformanceTaskBase(n), fit_(list), grad_(), nms_(), edge_(), points_() {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    nms_ = Nms(TH_LOW, TH_HIGH);
    grad_.process(src);
    nms_.process(grad_);
    edge_ = std::make_unique<Edge>(10, 3, 3, static_cast<float>(grad_.magnitudeThreshold(TH_LOW)));
    edge_->detect(grad_, nms_);
    PixelEstimator<float, cv::Point>::convert(edge_->points(), points_, grad_.magnitude(), nms_.directionMap());
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    std::vector<LineSegment<float, Vec2>> lsegs;
    fit_.apply(edge_->segments(), points_, lsegs);
  }
};


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create line fitting performance test with all tasks
CVPerformanceTestPtr createFitPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "Line fit");

  test->input_tasks.push_back(std::make_shared<Entry<FitLine<RegressionFit<float, cv::Point>>>>("RegressionFit"));
  test->input_tasks.push_back(std::make_shared<Entry<FitLine<EigenFit<float, cv::Point>>>>("EigenFit"));
  test->input_tasks.push_back(std::make_shared<Entry<FitLine<EigenCVFit<float, cv::Point>>>>("EigenCVFit"));
  test->input_tasks.push_back(std::make_shared<Entry<MEstimatorFitLine<float, cv::Point>>>("MEstimatorFit"));

  return test;
}

bool addFit() {
  addPerformanceTestCreator(createFitPerformanceTest);
  std::cout << "Added line fit performance test" << std::endl;
  return true;
}

// bool fitAdded = addFit();
