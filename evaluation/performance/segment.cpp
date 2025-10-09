#include "performance_test.hpp"
#include <edge/edge_drawing.hpp>
#include <edge/edge_linking.hpp>
#include <edge/edge_pattern.hpp>
#include <edge/edge_simple.hpp>
#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>


using namespace lsfm;

typedef DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> Grad;
typedef NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> Nms;

constexpr float th_low = 0.004f, th_high = 0.012f;
struct SegPerformaceData : public TaskData {
  SegPerformaceData(const std::string& n, const cv::Mat& s) : TaskData(n, s), grad(), nms(th_low, th_high) {
    if (src.channels() == 3) cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    grad.process(src);
    nms.process(grad);
  }

  virtual ~SegPerformaceData() = default;

  Grad grad;
  Nms nms;
};

struct SegPerformanceTest : public PerformanceTest {
  SegPerformanceTest(const std::string& testName = std::string()) : PerformanceTest(testName) {}
  virtual ~SegPerformanceTest() {}

 protected:
  // prepare task data
  std::unique_ptr<TaskData> prepareTaskData(const std::string& src_name, cv::Mat& src) override {
    prepareSource(src);
    return std::make_unique<SegPerformaceData>(src_name, src);
  }
};

struct Entry : public PerformanceTaskBase {
  Entry(const cv::Ptr<EsdBase<int>>& e, const std::string& n) : PerformanceTaskBase(n), edge(e) {}
  virtual ~Entry() {}
  virtual void run(const TaskData& data, int runs, bool verbose) {
    const SegPerformaceData& pdata = dynamic_cast<const SegPerformaceData&>(data);
    this->measure.push_back(PerformanceMeasure(pdata.name, this->name, pdata.src.cols, pdata.src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      edge->detect(pdata.grad, pdata.nms);
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  cv::Ptr<EsdBase<int>> edge;
};

PerformanceTestPtr createSegmentPerformanceTest(const lsfm::DataProviderList& provider) {
  auto test = std::make_shared<SegPerformanceTest>();
  test->name = "Segment";
  try {
    // add default
    test->data = provider;

    // add other
  } catch (std::exception& e) {
    std::cout << test->name << " parse error: " << e.what() << std::endl;
    return PerformanceTestPtr();
  }

  Grad grad;

  test->tasks.push_back(PerformanceTaskPtr(new Entry(new EsdSimple<int>, "ESD Simple")));
  test->tasks.push_back(PerformanceTaskPtr(
      new Entry(new EsdDrawing<int>(10, 3, static_cast<float>(grad.magnitudeThreshold(th_low))), "ESD Drawing")));
  test->tasks.push_back(PerformanceTaskPtr(
      new Entry(new EsdLinking<int>(10, 3, 3, static_cast<float>(grad.magnitudeThreshold(th_low))), "ESD Linking")));
  test->tasks.push_back(PerformanceTaskPtr(
      new Entry(new EsdPattern<int>(10, 3, 3, static_cast<float>(grad.magnitudeThreshold(th_low))), "ESD Pattern")));
  test->tasks.push_back(PerformanceTaskPtr(
      new Entry(new EsdLinking<int, 8, true>(10, 3, 3, static_cast<float>(grad.magnitudeThreshold(th_low))),
                "ESD Linking Corner")));
  test->tasks.push_back(PerformanceTaskPtr(
      new Entry(new EsdPattern<int, 8, true>(10, 3, 3, static_cast<float>(grad.magnitudeThreshold(th_low))),
                "ESD Pattern Corner")));
  return test;
}

bool addSegment() {
  addPerformanceTestCreator(createSegmentPerformanceTest);
  std::cout << "Added segment performance test" << std::endl;
  return true;
}

// bool segmentAdded = addSegment();
