#include "performance_test.hpp"

#include <imgproc/derivative_gradient.hpp>
#include <edge/nms.hpp>

#include <edge/edge_linking.hpp>
#include <edge/nfa.hpp>

using namespace lsfm;

typedef DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> Grad;
typedef NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> Nms;
typedef EsdLinking<int> Edge;

constexpr float th_low = 0.004f, th_high = 0.012f;

struct SegEvalPerformaceData : public TaskData {
    SegEvalPerformaceData(const std::string& n, const cv::Mat& s) : TaskData(n, s), nms(th_low, th_high), edge(10, 3, 3, grad.magnitudeThreshold(th_low)) {
        if (src.channels() == 3)
            cv::cvtColor(src, src, CV_BGR2GRAY);
        grad.process(src);
        nms.process(grad);
        edge.detect(grad, nms);
    }

    Grad grad;
    Nms nms;
    Edge edge;
};

struct SegEvalPerformanceTest : public PerformanceTest {
    SegEvalPerformanceTest(const std::string& testName = std::string()) : PerformanceTest(testName) {}
    virtual ~SegEvalPerformanceTest() {}

protected:
    // prepare task data
    virtual TaskData* prepareTaskData(const std::string& src_name, cv::Mat& src) {
        prepareSource(src);
        return new SegEvalPerformaceData(src_name, src);
    }

};

template<class EVAL>
struct Entry : public PerformanceTaskBase {

    Entry(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
        : PerformanceTaskBase(n), eval(list) {}
    virtual ~Entry() {}

    virtual void run(TaskData* data, int runs, bool verbose) {
        SegEvalPerformaceData* pdata = dynamic_cast<SegEvalPerformaceData*>(data);
        this->measure.push_back(PerformanceMeasure(pdata->name, this->name, pdata->src.cols, pdata->src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        EdgeSegmentVector out;
        std::vector<float> n;
        uint64 start;
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            eval.update(pdata->grad);
            eval.eval(pdata->edge, out, n);
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }

    EVAL eval;
};

void createSegEvalPerformanceTest(PerformanceTestPtr& test, const DataProviderMap& provider)
{
    test.reset(new SegEvalPerformanceTest);
    test->name = "SegEval";
    try {
        // add default
        addDefault(provider, test->data);

        //add other
    }
    catch (std::exception& e) {
        std::cout << test->name << " parse error: " << e.what() << std::endl;
        return;
    }
    test->tasks.push_back(PerformanceTaskPtr(new Entry<NfaContrast<int, float, index_type, std::map<int, float>>>("NFA Contrast")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<NfaBinom<short, float, index_type>>("NFA Binom")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<NfaBinom2<short, float, index_type>>("NFA Binom2")));
    // one more variant using mag, patterns etc.ß1
}

bool addSegEval() {
    addPerformanceTestCreator(createSegEvalPerformanceTest);
    std::cout << "Added segment evalation performance test" << std::endl;
    return true;
}

//bool segEvalAdded = addSegEval();
