/// @file lsd.cpp
/// @brief Line Segment Detector performance tests
#include "performance_test.hpp"
#include <lsd/lsd_burns.hpp>
#include <lsd/lsd_cc.hpp>
#include <lsd/lsd_cp.hpp>
#include <lsd/lsd_edlz.hpp>
#include <lsd/lsd_el.hpp>
#include <lsd/lsd_ep.hpp>
#include <lsd/lsd_fbw.hpp>
#include <lsd/lsd_fgioi.hpp>
#include <lsd/lsd_hcv.hpp>

#ifdef _MSC_VER
#  include <lsd/lsd_edta.hpp>
#  pragma comment(lib, "../../../../lib/EDLinesLib.lib")
#  if (_MSC_VER >= 1900)
#    pragma comment(lib, "legacy_stdio_definitions.lib")
#  endif
#endif


using namespace lsfm;


// =============================================================================
// LSD Performance Tasks
// =============================================================================

/// @brief Generic LSD performance task wrapper
template <class FT>
class Entry : public CVPerformanceTaskBase {
  std::shared_ptr<LsdBase<FT>> lsd_{};

 public:
  Entry() : CVPerformanceTaskBase(""), lsd_() {}

  Entry(std::shared_ptr<LsdBase<FT>> lsd, const std::string& n) : CVPerformanceTaskBase(n), lsd_(std::move(lsd)) {}

  void value(const std::string& param_name, const lsfm::Value& param_value) override {
    lsd_->value(param_name, param_value);
  }

 protected:
  void prepareImpl(const cv::Mat& src) override { lsd_->detect(src); }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& src) override { lsd_->detect(src); }
};


// =============================================================================
// Test Registration
// =============================================================================

/// @brief Create LSD performance test with all tasks
CVPerformanceTestPtr createLSDPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "LSD");

  test->tasks.push_back(
      std::make_shared<Entry<float>>(std::make_shared<LsdEL<float>>(0.004, 0.012, 15, 2, 15, 0, 0), "Lsd EL"));

  test->tasks.push_back(std::make_shared<Entry<float>>(
      std::make_shared<
          LsdEL<float, Vec2, Vec2<int>,
                EdgeSourceGRAD<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>,
                               NonMaximaSuppression<short, int, float>>,
                EsdLinking<int, 8, true>>>(0.004, 0.012, 15, 2, 15, 0, 0),
      "Lsd EL Corner"));

  test->tasks.push_back(std::make_shared<Entry<float>>(
      std::make_shared<
          LsdEL<float, Vec2, Vec2<int>,
                EdgeSourceGRAD<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>,
                               NonMaximaSuppression<short, int, float>>,
                EsdLinking<int, 8>, NfaBinom<short, float, index_type>, PixelEstimator<float, Vec2<int>>,
                LeastSquareSplit<float, Vec2<int>>>>(0.004, 0.012, 15, 2, 15, 0, 0),
      "Lsd EL LSSplit"));

  test->tasks.push_back(std::make_shared<Entry<float>>(
      std::make_shared<LsdEP<float, Vec2, false>>(0.004, 0.012, 15, 2, 15, 2, 3, 3, 5, 0), "Lsd EP"));

  test->tasks.push_back(std::make_shared<Entry<float>>(
      std::make_shared<LsdEP<float, Vec2, true>>(0.004, 0.012, 15, 2, 15, 2, 3, 3, 5, 0), "Lsd EP Corner"));

  test->tasks.push_back(std::make_shared<Entry<float>>(
      std::make_shared<
          LsdEP<float, Vec2, false, Vec2<int>,
                EdgeSourceGRAD<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>,
                               NonMaximaSuppression<short, int, float>>,
                PixelEstimator<float, Vec2<int>>, LeastSquareSplit<float, Vec2<int>>>>(0.004, 0.012, 15, 2, 15, 2, 3, 3,
                                                                                       5, 0),
      "Lsd EP LSSplit"));

  test->tasks.push_back(
      std::make_shared<Entry<float>>(std::make_shared<LsdCC<float>>(0.004, 0.012, 15, 3, 2, 0), "Lsd CC"));

  test->tasks.push_back(std::make_shared<Entry<float>>(
      std::make_shared<LsdCC<float>>(0.004, 0.012, 15, 3, 2, CC_CORNER_RULE), "Lsd CC Corner"));

  test->tasks.push_back(
      std::make_shared<Entry<float>>(std::make_shared<LsdCP<float>>(0.004, 0.012, 15, 0, 2, 2), "Lsd CP"));

  test->tasks.push_back(std::make_shared<Entry<float>>(
      std::make_shared<LsdCP<float>>(0.004, 0.012, 15, 0, 2, 2, CP_CORNER_RULE), "Lsd CP Corner"));

  test->tasks.push_back(std::make_shared<Entry<float>>(
      std::make_shared<LsdHoughP<float>>(0.004, 0.012, 1.5, CV_PI / 180, 50, 15, 3), "Lsd HoughP"));

  auto lsd_hough =
      std::make_shared<Entry<float>>(std::make_shared<LsdHough<float>>(0.004, 0.012, 15, 3, 0), "Lsd Hough");
  lsd_hough->value("hough_vote_th", 50);
  test->tasks.push_back(lsd_hough);

  test->tasks.push_back(
      std::make_shared<Entry<float>>(std::make_shared<LsdFGioi<float>>(2, 22.5, 0, 0.7, 1024), "Lsd FGioi"));

  test->tasks.push_back(
      std::make_shared<Entry<float>>(std::make_shared<LsdFBW<float>>(0.004, 0.012, 15, 22.5, 0), "Lsd FBW"));

  test->tasks.push_back(
      std::make_shared<Entry<float>>(std::make_shared<LsdFBW<float>>(0.004, 0.012, 15, 22.5, FBW_NMS), "Lsd FBW NMS"));

  test->tasks.push_back(
      std::make_shared<Entry<float>>(std::make_shared<LsdBurns<float>>(0.004, 0.012, 15, 12, 0), "Lsd Burns"));

  test->tasks.push_back(std::make_shared<Entry<float>>(
      std::make_shared<LsdBurns<float>>(0.004, 0.012, 15, 12, BURNS_NMS), "Lsd Burns NMS"));

  test->tasks.push_back(
      std::make_shared<Entry<float>>(std::make_shared<LsdEDLZ<float>>(10, 2, 1, 15, 2, false), "Lsd EDLZ"));

  test->tasks.push_back(
      std::make_shared<Entry<float>>(std::make_shared<LsdEDLZ<float>>(10, 2, 1, 15, 2, true), "Lsd EDLZ NFA"));

#ifdef _MSC_VER
  test->tasks.push_back(std::make_shared<Entry<float>>(std::make_shared<LsdEDTA<float>>(), "Lsd EDTA"));
#endif

  return test;
}

bool addLSD() {
  addPerformanceTestCreator(createLSDPerformanceTest);
  std::cout << "Added LSD performance test" << std::endl;
  return true;
}

// bool LSDAdded = addLSD();
