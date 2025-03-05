#include "performance_test.hpp"

#include <imgproc/derivative_gradient.hpp>
#include <edge/nms.hpp>

#include <edge/edge_linking.hpp>
#include <edge/split.hpp>
#include <edge/spe.hpp>
#include <edge/fit.hpp>

using namespace lsfm;

typedef DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> Grad;
typedef NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> Nms;
typedef EsdLinking<int> Edge;
typedef std::vector<cv::Point> PointVector;

constexpr float th_low = 0.004f, th_high = 0.012f;

struct FitPerformaceData : public TaskData {
    FitPerformaceData(const std::string& n, const cv::Mat& s) : TaskData(n, s), nms(th_low, th_high), edge(10, 3, 3, grad.magnitudeThreshold(th_low)) {
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
    virtual TaskData* prepareTaskData(const std::string& src_name, cv::Mat& src) {
        prepareSource(src);
        return new FitPerformaceData(src_name, src);
    }

};

template<class FIT>
struct Entry : public PerformanceTaskBase {

    Entry(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
        : PerformanceTaskBase(n), fit(list) {}
    virtual ~Entry() {}

    virtual void run(TaskData* data, int runs, bool verbose) {
        FitPerformaceData* pdata = dynamic_cast<FitPerformaceData*>(data);
        this->measure.push_back(PerformanceMeasure(pdata->name, this->name, pdata->src.cols, pdata->src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        std::vector < LineSegment<float, Vec2>> lsegs;
        uint64 start;
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            fit.apply(pdata->edge.segments(), pdata->points,lsegs);
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }

    FIT fit;
};


void createFitPerformanceTest(PerformanceTestPtr& test, const DataProviderMap& provider)
{
    test.reset(new FitPerformanceTest);
    test->name = "Line fit";
    try {
        // add default
        addDefault(provider, test->data);

        //add other
    }
    catch (std::exception& e) {
        std::cout << test->name << " parse error: " << e.what() << std::endl;
        return;
    }
    test->tasks.push_back(PerformanceTaskPtr(new Entry<FitLine<RegressionFit<float,cv::Point>>>("RegressionFit")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<FitLine<EigenFit<float, cv::Point>>>("EigenFit")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<FitLine<EigenCVFit<float, cv::Point>>>("EigenCVFit")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<MEstimatorFitLine<float, cv::Point>>("MEstimatorFit")));
}

bool addFit() {
    addPerformanceTestCreator(createFitPerformanceTest);
    std::cout << "Added line fit performance test" << std::endl;
    return true;
}

//bool fitAdded = addFit();
