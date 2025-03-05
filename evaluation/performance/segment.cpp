#include "performance_test.hpp"

#include <imgproc/derivative_gradient.hpp>
#include <edge/nms.hpp>

#include <edge/edge_simple.hpp>
#include <edge/edge_drawing.hpp>
#include <edge/edge_linking.hpp>
#include <edge/edge_pattern.hpp>


using namespace lsfm;

typedef DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> Grad;
typedef NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> Nms;

constexpr  float th_low = 0.004f, th_high = 0.012f;
struct SegPerformaceData : public TaskData {
    SegPerformaceData(const std::string& n, const cv::Mat& s) : TaskData(n, s), nms(th_low, th_high) {
      if (src.channels() == 3) cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
      grad.process(src);
      nms.process(grad);
    }

    Grad grad;
    Nms nms;
};

struct SegPerformanceTest : public PerformanceTest {
    SegPerformanceTest(const std::string& testName = std::string()) : PerformanceTest(testName) {}
    virtual ~SegPerformanceTest() {}

protected:
    // prepare task data
    virtual TaskData* prepareTaskData(const std::string& src_name, cv::Mat& src) {
        prepareSource(src);
        return new SegPerformaceData
(src_name, src);
    }

};

struct Entry : public PerformanceTaskBase {

    Entry(const cv::Ptr<EsdBase<int>>& e, const std::string& n)
        : PerformanceTaskBase(n), edge(e) {}
    virtual ~Entry() {}
    virtual void run(TaskData* data, int runs, bool verbose) {
        SegPerformaceData* pdata = dynamic_cast<SegPerformaceData*>(data);
        this->measure.push_back(PerformanceMeasure(pdata->name, this->name, pdata->src.cols, pdata->src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        uint64 start;
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            edge->detect(pdata->grad, pdata->nms);
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }

    cv::Ptr<EsdBase<int>> edge;
};

void createSegmentPerformanceTest(PerformanceTestPtr& test, const DataProviderMap& provider)
{
    test.reset(new SegPerformanceTest);
    test->name = "Segment";
    try {
        // add default
        addDefault(provider, test->data);

        //add other
    }
    catch (std::exception& e) {
        std::cout << test->name << " parse error: " << e.what() << std::endl;
        return;
    }

    Grad grad;

    test->tasks.push_back(PerformanceTaskPtr(new Entry(new EsdSimple<int>, "ESD Simple")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry(new EsdDrawing<int>(10,3, grad.magnitudeThreshold(th_low)), "ESD Drawing")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry(new EsdLinking<int>(10, 3, 3, grad.magnitudeThreshold(th_low)), "ESD Linking")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry(new EsdPattern<int>(10, 3, 3, grad.magnitudeThreshold(th_low)), "ESD Pattern")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry(new EsdLinking<int,8,true>(10, 3, 3, grad.magnitudeThreshold(th_low)), "ESD Linking Corner")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry(new EsdPattern<int,8,true>(10, 3, 3, grad.magnitudeThreshold(th_low)), "ESD Pattern Corner")));
    

}

bool addSegment() {
    addPerformanceTestCreator(createSegmentPerformanceTest);
    std::cout << "Added segment performance test" << std::endl;
    return true;
}

//bool segmentAdded = addSegment();
