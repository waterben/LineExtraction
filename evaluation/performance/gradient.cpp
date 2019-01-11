#include "performance_test.hpp"

#include <imgproc/gradient_adapter.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/susan.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/pc_lgf.hpp>
#include <imgproc/pc_matlab.hpp>
#include <imgproc/laplace.hpp>

using namespace lsfm;

template<class GT, class MT, class FT>
struct Entry : public PerformanceTaskDefault {
    Entry() {}

    Entry(const cv::Ptr<GradientI<uchar,GT,MT,FT>>& g, const std::string& n, int f = 0)
        : PerformanceTaskDefault(n, f), gradient(g) {}

    cv::Ptr<GradientI<uchar, GT, MT, FT>> gradient;
    
    void run(const std::string& src_name, cv::Mat src, int loops, bool verbose) {
        this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
        PerformanceMeasure& pm = this->measure.back();
        if (verbose)
            std::cout << "    Running " << this->name << " ... ";
        gradient->process(src);
        gradient->magnitude();
        cv::Mat tmp;
        uint64 start;
        for (int i = 0; i != loops; ++i) {
            start = cv::getTickCount();
            gradient->process(src);
            tmp = gradient->magnitude();
            pm.measures.push_back(cv::getTickCount() - start);
        }
        if (verbose)
            std::cout << std::setprecision(3) << static_cast<double>((cv::getTickCount() - start) * 1000) / (loops * cv::getTickFrequency()) << "ms" << std::endl;
    }

    void saveResults() {

    }
};

void createGradientPerformanceTest(PerformanceTestPtr& test, const DataProviderMap& provider)
{
    test.reset(new PerformanceTest);
    test->name = "Gradient";
    try {
        addDefault(provider, test->data);

        //add other
    }
    catch (std::exception& e) {
        std::cout << test->name << " parse error: " << e.what() << std::endl;
        return;
    }
    
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, RobertsDerivative, QuadraticMagnitude>, "Roberts (2x2)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, PrewittDerivative, QuadraticMagnitude>, "Prewitt (3x3)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, ScharrDerivative, QuadraticMagnitude>, "Scharr (3x3)")));
    
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>, "Sobel (3x3)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",5) }), "Sobel (5x5)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",7) }), "Sobel (7x7)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",9) }), "Sobel (9x9)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",11) }), "Sobel (11x11)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",13) }), "Sobel (13x13)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",15) }), "Sobel (15x15)")));

    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new SusanGradient<short, int>, "Susan (37)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new SusanGradient<short, int>(20, true), "Susan (3x3)")));

    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new RCMGradient<uchar, 1, short, int>(3, 1), "RMG (3x3)", TASK_NO_3)));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new RCMGradient<uchar, 1, short, int>(5, 3), "RMG (5x5)", TASK_NO_5)));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<short, int, float>(new RCMGradient<uchar, 3, short, int>(3, 1), "RCMG (3x3)", TASK_RGB)));

    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",3), NV("grad_range",1.5) }), "Gauss (3x3)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",5), NV("grad_range",2.3) }), "Gauss (5x5)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",7), NV("grad_range",3.0) }), "Gauss (7x7)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",9), NV("grad_range",3.5) }), "Gauss (9x9)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",11), NV("grad_range",4) }), "Gauss (11x11)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",13), NV("grad_range",4.5) }), "Gauss (13x13)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",15), NV("grad_range",5) }), "Gauss (15x15)")));
    
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size",3), NV("grad_kernel_spacing",1.24008) }), "QF_StG (3x3)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.008) }), "QF_StG (5x5)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 0.873226) }), "QF_StG (7x7)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 0.781854) }), "QF_StG (9x9)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 11), NV("grad_kernel_spacing", 0.7) }), "QF_StG (11x11)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 13), NV("grad_kernel_spacing", 0.65) }), "QF_StG (13x13)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 15), NV("grad_kernel_spacing", 0.6) }), "QF_StG (15x15)")));
    
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 3), NV("grad_kernel_spacing", 1.2) }), "SQF PO (3x3)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.2) }), "SQF PO (5x5)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 1.2) }), "SQF PO (7x7)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 1.2) }), "SQF PO (9x9)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 11), NV("grad_kernel_spacing", 1.2) }), "SQF PO (11x11)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 13), NV("grad_kernel_spacing", 1.2) }), "SQF PO (13x13)")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 15), NV("grad_kernel_spacing", 1.2) }), "SQF PO (15x15)")));
    
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureLGF<uchar, float, PolarCV>>({ NV("grad_waveLength", 3), NV("grad_sigmaOnf", 0.55) }), "SQFF LG")));
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientEnergy<QuadratureSF<uchar, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2) }), "SQFF PO")));
    
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientPC<PCLgf<uchar, float, PolarCV>>, "PC LGF")));
    
    test->tasks.push_back(PerformanceTaskPtr(new Entry<float, float, float>(new GradientPC<PCSqf<uchar, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2) }), "PC POF")));

    test->tasks.push_back(PerformanceTaskPtr(new Entry<double, double, double>(new GradientPC<PCLgf<uchar, double, PolarCV>>, "PC LGD")));

    test->tasks.push_back(PerformanceTaskPtr(new Entry<double, double, double>(new GradientPC<PCSqf<uchar, double, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2) }), "PC POD")));

    test->tasks.push_back(PerformanceTaskPtr(new Entry<double, double, double>(new GradientPC<PCMatlab<uchar>>, "PC ML")));

}

bool addGradient() {
    addPerformanceTestCreator(createGradientPerformanceTest);
    std::cout << "Added gradient performance test" << std::endl;
    return true;
}

//bool gradientAdded = addGradient();

