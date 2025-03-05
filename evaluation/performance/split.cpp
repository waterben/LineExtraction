#include "performance_test.hpp"

#include <imgproc/derivative_gradient.hpp>
#include <edge/nms.hpp>

#include <edge/edge_pattern.hpp>
#include <edge/spe.hpp>
#include <edge/split.hpp>

using namespace lsfm;

typedef DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> Grad;
typedef NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> Nms;
typedef EsdPattern<int> Edge;

constexpr float th_low = 0.004f, th_high = 0.012f;

struct SplitPerformaceData : public TaskData {
    SplitPerformaceData(const std::string& n, const cv::Mat& s) : TaskData(n, s), nms(th_low, th_high), edge(10, 3, 3, grad.magnitudeThreshold(th_low)) {
      if (src.channels() == 3) cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
      grad.process(src);
      nms.process(grad);
      edge.detect(grad, nms);
    }

    Grad grad;
    Nms nms;
    Edge edge;
};

struct SplitPerformanceTest : public PerformanceTest {
    SplitPerformanceTest(const std::string& testName = std::string()) : PerformanceTest(testName) {}
    virtual ~SplitPerformanceTest() {}

protected:
    // prepare task data
    virtual TaskData* prepareTaskData(const std::string& src_name, cv::Mat& src) {
        prepareSource(src);
        return new SplitPerformaceData(src_name, src);
    }

};

template<class SPLIT>
struct Entry : public PerformanceTaskBase {

    Entry(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
        : PerformanceTaskBase(n), split(list) {}
    virtual ~Entry() {}

    virtual void run(TaskData* data, int runs, bool verbose) {
        SplitPerformaceData* pdata = dynamic_cast<SplitPerformaceData*>(data);
        this->measure.push_back(PerformanceMeasure(pdata->name, this->name, pdata->src.cols, pdata->src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        EdgeSegmentVector out;
        // make sure segments are build form pattern segments
        pdata->edge.segments();
        std::vector<Vec2i> points;
        PixelEstimator<float>::convert(pdata->edge.points(), points, pdata->grad.magnitude(), pdata->nms.directionMap());
        uint64 start;
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            split.setup(pdata->grad, pdata->nms);
            split.apply(pdata->edge, points, out);
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }

    SPLIT split;
};

template<class SPLIT>
struct EntryPattern : public PerformanceTaskBase {

    EntryPattern(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
        : PerformanceTaskBase(n), split(list) {}
    virtual ~EntryPattern() {}

    virtual void run(TaskData* data, int runs, bool verbose) {
        SplitPerformaceData* pdata = dynamic_cast<SplitPerformaceData*>(data);
        this->measure.push_back(PerformanceMeasure(pdata->name, this->name, pdata->src.cols, pdata->src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        EdgeSegmentVector out;
        std::vector<Vec2i> points;
        PixelEstimator<float>::convert(pdata->edge.points(), points, pdata->grad.magnitude(), pdata->nms.directionMap());
        uint64 start;
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            split.setup(pdata->grad, pdata->nms);
            split.applyP(pdata->edge, points, out);
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }

    SPLIT split;
};



void createSplitPerformanceTest(PerformanceTestPtr& test, const DataProviderMap& provider)
{
    test.reset(new SplitPerformanceTest);
    test->name = "Split";
    try {
        // add default
        addDefault(provider, test->data);

        //add other
    }
    catch (std::exception& e) {
        std::cout << test->name << " parse error: " << e.what() << std::endl;
        return;
    }

    test->tasks.push_back(PerformanceTaskPtr(new Entry<RamerSplit<float,Vec2i,false> >("Ramer")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<RamerSplit<float, Vec2i, true> >("Ramer +m")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<ExtRamerSplit<NoMerge<SimpleSplitCheck<float>>> >("ExtRamer")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<ExtRamerSplit<SimpleMerge<SimpleSplitCheck<float>>> >("ExtRamer +m")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<ExtRamerSplit<NoMerge<ExtSplitCheck<float,int>>> >("ExtRamer +es")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<ExtRamerSplit<SimpleMerge<ExtSplitCheck<float, int>>> >("ExtRamer +m+es")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<LeastSquareSplit<float> >("LSqr")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<LeastSquareSplit<float, Vec2i, false> >("LSqr +p")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<AdaptiveLeastSquareSplit<float> >("ALSqr")));

    test->tasks.push_back(PerformanceTaskPtr(new EntryPattern<RamerSplit<float, Vec2i, false> >("PRamer")));
    test->tasks.push_back(PerformanceTaskPtr(new EntryPattern<RamerSplit<float, Vec2i, true> >("PRamer +m")));
    test->tasks.push_back(PerformanceTaskPtr(new EntryPattern<ExtRamerSplit<NoMerge<SimpleSplitCheck<float>>> >("PExtRamer")));
    test->tasks.push_back(PerformanceTaskPtr(new EntryPattern<ExtRamerSplit<SimpleMerge<SimpleSplitCheck<float>>> >("PExtRamer +m")));
    test->tasks.push_back(PerformanceTaskPtr(new EntryPattern<ExtRamerSplit<NoMerge<ExtSplitCheck<float, int>>> >("PExtRamer +es")));
    test->tasks.push_back(PerformanceTaskPtr(new EntryPattern<ExtRamerSplit<SimpleMerge<ExtSplitCheck<float, int>>> >("PExtRamer +m+es")));
    test->tasks.push_back(PerformanceTaskPtr(new EntryPattern<LeastSquareSplit<float> >("PLSqr")));
    test->tasks.push_back(PerformanceTaskPtr(new EntryPattern<LeastSquareSplit<float, Vec2i, false> >("PLSqr +p")));
    test->tasks.push_back(PerformanceTaskPtr(new EntryPattern<AdaptiveLeastSquareSplit<float> >("PALSqr")));
   
}

bool addSplit() {
    addPerformanceTestCreator(createSplitPerformanceTest);
    std::cout << "Added split performance test" << std::endl;
    return true;
}

//bool splitAdded = addSplit();
