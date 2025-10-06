#include "performance_test.hpp"
#include <edge/edge_linking.hpp>
#include <edge/fit.hpp>
#include <edge/nms.hpp>
#include <edge/spe.hpp>
#include <edge/split.hpp>
#include <imgproc/derivative_gradient.hpp>


using namespace lsfm;

typedef DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> Grad;
typedef NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> Nms;
typedef EsdLinking<int> Edge;
typedef std::vector<cv::Point> PointVector;

constexpr float th_low = 0.004f, th_high = 0.012f;

struct FitPerformaceData : public TaskData {
  FitPerformaceData(const std::string& n, const cv::Mat& s)
      : TaskData(n, s), nms(th_low, th_high), edge(10, 3, 3, grad.magnitudeThreshold(th_low)) {
    if (src.channels() == 3) cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    grad.process(src);
    nms.process(grad);
    edge.detect(grad, nms);
    PixelEstimator<float, cv::Point>::convert(edge.points(), points, grad.magnitude(), nms.directionMap());
  }

  Grad grad;
  Nms nms;
  Edge edge;
  PointVector points;
};

struct FitPerformanceTest : public PerformanceTest {
  FitPerformanceTest(const std::string& testName = std::string()) : PerformanceTest(testName) {}
  virtual ~FitPerformanceTest() {}

 protected:
  // prepare task data
  std::unique_ptr<TaskData> prepareTaskData(const std::string& src_name, cv::Mat& src) override {
    prepareSource(src);
    return std::make_unique<FitPerformaceData>(src_name, src);
  }
};

template <class FIT>
struct Entry : public PerformanceTaskBase {
  Entry(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
      : PerformanceTaskBase(n), fit(list) {}
  virtual ~Entry() {}

  virtual void run(const TaskData& data, int runs, bool verbose) {
    const FitPerformaceData& pdata = dynamic_cast<const PerformaceData&>(data);
    this->measure.push_back(PerformanceMeasure(pdata.name, this->name, pdata.src.cols, pdata.src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    std::vector<LineSegment<float, Vec2>> lsegs;
    uint64 start = 0;
    for (int i = 0; i != runs; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      fit.apply(pdata.edge.segments(), pdata.points, lsegs);
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (runs * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  FIT fit;
};


PerformanceTestPtr createFitPerformanceTest(const lsfm::DataProviderList& provider) {
  auto test = std::make_shared<FitPerformanceTest>();
  test->name = "Line fit";
  try {
    // add default
    test->data = provider;

    // add other
  } catch (std::exception& e) {
    std::cout << test->name << " parse error: " << e.what() << std::endl;
    return PerformanceTestPtr();
  }
  test->tasks.push_back(
      PerformanceTaskPtr(std::make_shared<Entry<FitLine<RegressionFit<float, cv::Point>>>>("RegressionFit")));
  test->tasks.push_back(PerformanceTaskPtr(std::make_shared<Entry<FitLine<EigenFit<float, cv::Point>>>>("EigenFit")));
  test->tasks.push_back(
      PerformanceTaskPtr(std::make_shared<Entry<FitLine<EigenCVFit<float, cv::Point>>>>("EigenCVFit")));
  test->tasks.push_back(
      PerformanceTaskPtr(std::make_shared<Entry<MEstimatorFitLine<float, cv::Point>>>("MEstimatorFit")));
  return test;
}

bool addFit() {
  addPerformanceTestCreator(createFitPerformanceTest);
  std::cout << "Added line fit performance test" << std::endl;
  return true;
}

// bool fitAdded = addFit();
