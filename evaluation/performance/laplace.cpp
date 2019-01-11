#include "performance_test.hpp"

#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/laplace.hpp>


using namespace lsfm;

template<class LT>
struct Entry : public PerformanceTaskDefault {
    Entry() {}

    Entry(const cv::Ptr<LaplaceI<uchar,LT>>& l, const std::string& n, int f = 0)
        : PerformanceTaskDefault(n,f), laplace(l) {}

    cv::Ptr<LaplaceI<uchar, LT>> laplace;
    
    void run(const std::string& src_name, cv::Mat src, int runs, bool verbose) {
        this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        laplace->process(src);
        laplace->laplace();
        cv::Mat tmp;
        uint64 start;
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            laplace->process(src);
            tmp = laplace->laplace();
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (runs * cv::getTickFrequency()) << "ms" << std::endl;
    }
};


void createLaplacePerformanceTest(PerformanceTestPtr& test, const DataProviderMap& provider)
{
    test.reset(new PerformanceTest);
    test->name = "Laplace";
    try {
        // add default
        addDefault(provider, test->data);

        //add other
    }
    catch (std::exception& e) {
        std::cout << test->name << " parse error: " << e.what() << std::endl;
        return;
    }

    test->tasks.push_back(PerformanceTaskPtr(new Entry<short>(new LaplaceSimple<uchar, short>, "Laplace (3x3)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short>(new LaplaceCV<uchar, short>(5), "Laplace Sobel (5x5)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short>(new LaplaceCV<uchar, short>(7), "Laplace Sobel (7x7)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short>(new LaplaceCV<uchar, short>(9), "Laplace Sobel (9x9)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short>(new LaplaceCV<uchar, short>(11), "Laplace Sobel (11x11)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short>(new LaplaceCV<uchar, short>(13), "Laplace Sobel (13x13)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short>(new LaplaceCV<uchar, short>(15), "Laplace Sobel (15x15)")));

    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new LoG<uchar, float>(3, 1.24008), "Laplace LoG (3x3)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new LoG<uchar, float>(5, 1.00803), "Laplace LoG (5x5)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new LoG<uchar, float>(7, 0.873228), "Laplace LoG (7x7)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new LoG<uchar, float>(9, 0.781859), "Laplace LoG (9x9)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new LoG<uchar, float>(11, 0.67), "Laplace LoG (11x11)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new LoG<uchar, float>(13, 0.6), "Laplace LoG (13x13)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new LoG<uchar, float>(15, 0.5), "Laplace LoG (15x15)")));

    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new PCLSq<uchar, float>(1.f,2.f,3,1.2f), "Laplace SQF (3x3)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new PCLSq<uchar, float>(1.f,2.f,5,1.2f), "Laplace SQF (5x5)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new PCLSq<uchar, float>(1.f,2.f,7,1.2f), "Laplace SQF (7x7)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new PCLSq<uchar, float>(1.f,2.f,9,1.2f), "Laplace SQF (9x9)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new PCLSq<uchar, float>(1.f, 2.f, 11, 1.2f), "Laplace SQF (11x11)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new PCLSq<uchar, float>(1.f, 2.f, 13, 1.2f), "Laplace SQF (13x13)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new PCLSq<uchar, float>(1.f, 2.f, 15, 1.2f), "Laplace SQF (15x15)")));

    test->tasks.push_back(PerformanceTaskPtr(new Entry<float>(new PCLSqf<uchar, float>(1, 2, 1.2), "Laplace SQFF")));


}

bool addLaplace() {
    addPerformanceTestCreator(createLaplacePerformanceTest);
    std::cout << "Added laplace performance test" << std::endl;
    return true;
}

//bool laplaceAdded = addLaplace();

