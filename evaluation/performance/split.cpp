/// @file split.cpp
/// @brief Edge segment splitting performance tests
#include "performance_test.hpp"
#include <edge/edge_pattern.hpp>
#include <edge/nms.hpp>
#include <edge/spe.hpp>
#include <edge/split.hpp>
#include <imgproc/derivative_gradient.hpp>

#include <memory>

using namespace lsfm;


using Grad = DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>;
using Nms = NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>>;
using Edge = EsdPattern<int>;

constexpr float TH_LOW = 0.004F;
constexpr float TH_HIGH = 0.012F;


// =============================================================================
// Split Performance Tasks
// =============================================================================

/// @brief Generic segment split performance task
template <class SPLIT>
class Entry : public CVPerformanceTaskBase {
  SPLIT split_;
  Grad grad_{};
  Nms nms_{};
  std::unique_ptr<Edge> edge_{};
  std::vector<Vec2i> points_{};

 public:
  Entry(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
      : CVPerformanceTaskBase(n), split_(list), grad_(), nms_(), edge_(), points_() {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    nms_ = Nms(TH_LOW, TH_HIGH);
    grad_.process(src);
    nms_.process(grad_);
    edge_ = std::make_unique<Edge>(10, 3, 3, static_cast<float>(grad_.magnitudeThreshold(TH_LOW)));
    edge_->detect(grad_, nms_);
    // Make sure segments are built from pattern segments
    edge_->segments();
    PixelEstimator<float>::convert(edge_->points(), points_, grad_.magnitude(), nms_.directionMap());
    split_.setup(grad_, nms_);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    EdgeSegmentVector out;
    split_.apply(*edge_, points_, out);
  }
};

/// @brief Pattern-based segment split performance task
template <class SPLIT>
class EntryPattern : public CVPerformanceTaskBase {
  SPLIT split_;
  Grad grad_{};
  Nms nms_{};
  std::unique_ptr<Edge> edge_{};
  std::vector<Vec2i> points_{};

 public:
  EntryPattern(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
      : CVPerformanceTaskBase(n), split_(list), grad_(), nms_(), edge_(), points_() {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    nms_ = Nms(TH_LOW, TH_HIGH);
    grad_.process(src);
    nms_.process(grad_);
    edge_ = std::make_unique<Edge>(10, 3, 3, static_cast<float>(grad_.magnitudeThreshold(TH_LOW)));
    edge_->detect(grad_, nms_);
    PixelEstimator<float>::convert(edge_->points(), points_, grad_.magnitude(), nms_.directionMap());
    split_.setup(grad_, nms_);
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& /*src*/) override {
    EdgeSegmentVector out;
    split_.applyP(*edge_, points_, out);
  }
};


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create split performance test with all tasks
CVPerformanceTestPtr createSplitPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "Split");

  test->tasks.push_back(std::make_shared<Entry<RamerSplit<float, Vec2i, false>>>("Ramer"));
  test->tasks.push_back(std::make_shared<Entry<RamerSplit<float, Vec2i, true>>>("Ramer +m"));
  test->tasks.push_back(std::make_shared<Entry<ExtRamerSplit<NoMerge<SimpleSplitCheck<float>>>>>("ExtRamer"));
  test->tasks.push_back(std::make_shared<Entry<ExtRamerSplit<SimpleMerge<SimpleSplitCheck<float>>>>>("ExtRamer +m"));
  test->tasks.push_back(std::make_shared<Entry<ExtRamerSplit<NoMerge<ExtSplitCheck<float, int>>>>>("ExtRamer +es"));
  test->tasks.push_back(
      std::make_shared<Entry<ExtRamerSplit<SimpleMerge<ExtSplitCheck<float, int>>>>>("ExtRamer +m+es"));
  test->tasks.push_back(std::make_shared<Entry<LeastSquareSplit<float>>>("LSqr"));
  test->tasks.push_back(std::make_shared<Entry<LeastSquareSplit<float, Vec2i, false>>>("LSqr +p"));
  test->tasks.push_back(std::make_shared<Entry<AdaptiveLeastSquareSplit<float>>>("ALSqr"));

  test->tasks.push_back(std::make_shared<EntryPattern<RamerSplit<float, Vec2i, false>>>("PRamer"));
  test->tasks.push_back(std::make_shared<EntryPattern<RamerSplit<float, Vec2i, true>>>("PRamer +m"));
  test->tasks.push_back(std::make_shared<EntryPattern<ExtRamerSplit<NoMerge<SimpleSplitCheck<float>>>>>("PExtRamer"));
  test->tasks.push_back(
      std::make_shared<EntryPattern<ExtRamerSplit<SimpleMerge<SimpleSplitCheck<float>>>>>("PExtRamer +m"));
  test->tasks.push_back(
      std::make_shared<EntryPattern<ExtRamerSplit<NoMerge<ExtSplitCheck<float, int>>>>>("PExtRamer +es"));
  test->tasks.push_back(
      std::make_shared<EntryPattern<ExtRamerSplit<SimpleMerge<ExtSplitCheck<float, int>>>>>("PExtRamer +m+es"));
  test->tasks.push_back(std::make_shared<EntryPattern<LeastSquareSplit<float>>>("PLSqr"));
  test->tasks.push_back(std::make_shared<EntryPattern<LeastSquareSplit<float, Vec2i, false>>>("PLSqr +p"));
  test->tasks.push_back(std::make_shared<EntryPattern<AdaptiveLeastSquareSplit<float>>>("PALSqr"));

  return test;
}

bool addSplit() {
  addPerformanceTestCreator(createSplitPerformanceTest);
  std::cout << "Added split performance test" << std::endl;
  return true;
}

bool splitAdded = addSplit();
