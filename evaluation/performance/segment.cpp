/// @file segment.cpp
/// @brief Edge segment detection performance tests
#include "performance_test.hpp"
#include <edge/edge_drawing.hpp>
#include <edge/edge_linking.hpp>
#include <edge/edge_pattern.hpp>
#include <edge/edge_simple.hpp>
#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>


using namespace lsfm;


using Grad = DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>;
using Nms = NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>>;

constexpr float TH_LOW = 0.004F;
constexpr float TH_HIGH = 0.012F;


// =============================================================================
// Edge Segment Detection Performance Tasks
// =============================================================================

/// @brief Generic edge segment detection performance task
class Entry : public CVPerformanceTaskBase {
  cv::Ptr<EsdBase<int>> edge_;
  Grad grad_{};
  Nms nms_{};

 public:
  Entry(cv::Ptr<EsdBase<int>> edge, const std::string& n)
      : CVPerformanceTaskBase(n), edge_(std::move(edge)), grad_(), nms_() {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    nms_ = Nms(TH_LOW, TH_HIGH);
    grad_.process(src);
    nms_.process(grad_);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override { edge_->detect(grad_, nms_); }
};


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create segment detection performance test with all tasks
CVPerformanceTestPtr createSegmentPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "Segment");

  Grad grad;
  float mag_th = static_cast<float>(grad.magnitudeThreshold(TH_LOW));

  test->input_tasks.push_back(std::make_shared<Entry>(new EsdSimple<int>, "ESD Simple"));
  test->input_tasks.push_back(std::make_shared<Entry>(new EsdDrawing<int>(10, 3, mag_th), "ESD Drawing"));
  test->input_tasks.push_back(std::make_shared<Entry>(new EsdLinking<int>(10, 3, 3, mag_th), "ESD Linking"));
  test->input_tasks.push_back(std::make_shared<Entry>(new EsdPattern<int>(10, 3, 3, mag_th), "ESD Pattern"));
  test->input_tasks.push_back(
      std::make_shared<Entry>(new EsdLinking<int, 8, true>(10, 3, 3, mag_th), "ESD Linking Corner"));
  test->input_tasks.push_back(
      std::make_shared<Entry>(new EsdPattern<int, 8, true>(10, 3, 3, mag_th), "ESD Pattern Corner"));

  return test;
}

bool addSegment() {
  addPerformanceTestCreator(createSegmentPerformanceTest);
  std::cout << "Added segment performance test" << std::endl;
  return true;
}

// bool segmentAdded = addSegment();
