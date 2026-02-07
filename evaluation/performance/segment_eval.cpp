/// @file segment_eval.cpp
/// @brief Edge segment evaluation (NFA) performance tests

#include "performance_test.hpp"
#include <edge/edge_linking.hpp>
#include <edge/nfa.hpp>
#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>

#include <memory>

using namespace lsfm;


using Grad = DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>;
using Nms = NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>>;
using Edge = EsdLinking<int>;

constexpr float TH_LOW = 0.004F;
constexpr float TH_HIGH = 0.012F;


// =============================================================================
// Segment Evaluation Performance Tasks
// =============================================================================

/// @brief Generic segment evaluation (NFA) performance task
template <class EVAL>
class Entry : public CVPerformanceTaskBase {
  EVAL eval_;
  Grad grad_{};
  Nms nms_{};
  std::unique_ptr<Edge> edge_{};

 public:
  Entry(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
      : CVPerformanceTaskBase(n), eval_(list), grad_(), nms_(), edge_() {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    nms_ = Nms(TH_LOW, TH_HIGH);
    grad_.process(src);
    nms_.process(grad_);
    edge_ = std::make_unique<Edge>(10, 3, 3, static_cast<float>(grad_.magnitudeThreshold(TH_LOW)));
    edge_->detect(grad_, nms_);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    EdgeSegmentVector out;
    std::vector<float> nfa_values;
    eval_.update(grad_);
    eval_.eval(*edge_, out, nfa_values);
  }
};


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create segment evaluation performance test with all tasks
CVPerformanceTestPtr createSegEvalPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "SegEval");

  test->input_tasks.push_back(
      std::make_shared<Entry<NfaContrast<int, float, index_type, std::map<int, float>>>>("NFA Contrast"));
  test->input_tasks.push_back(std::make_shared<Entry<NfaBinom<short, float, index_type>>>("NFA Binom"));
  test->input_tasks.push_back(std::make_shared<Entry<NfaBinom2<short, float, index_type>>>("NFA Binom2"));

  return test;
}

bool addSegEval() {
  addPerformanceTestCreator(createSegEvalPerformanceTest);
  std::cout << "Added segment evalation performance test" << std::endl;
  return true;
}

// bool segEvalAdded = addSegEval();
