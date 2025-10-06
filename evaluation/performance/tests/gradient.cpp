#include "performance_test.hpp"
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/gradient_adapter.hpp>
#include <imgproc/laplace.hpp>
#include <imgproc/pc_lgf.hpp>
#include <imgproc/pc_matlab.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/susan.hpp>


using namespace lsfm;

template <class GT, class MT, class FT>
struct Entry : public PerformanceTaskDefault {
  Entry() {}

  Entry(std::shared_ptr<GradientI<uchar, GT, MT, FT>> g, const std::string& n, int f = 0)
      : PerformanceTaskDefault(n, f), gradient(std::move(g)) {}

  std::shared_ptr<GradientI<uchar, GT, MT, FT>> gradient;

  virtual void run(const std::string& src_name, const cv::Mat& src, int loops, bool verbose) {
    this->measure.push_back(PerformanceMeasure(src_name, this->name, src.cols, src.rows));
    PerformanceMeasure& pm = this->measure.back();
    if (verbose) std::cout << "    Running " << this->name << " ... ";
    gradient->process(src);
    gradient->magnitude();
    cv::Mat tmp;
    uint64 start = 0;
    for (int i = 0; i != loops; ++i) {
      start = static_cast<uint64>(cv::getTickCount());
      gradient->process(src);
      tmp = gradient->magnitude();
      pm.measures.push_back(static_cast<uint64>(cv::getTickCount()) - start);
    }
    if (verbose)
      std::cout << std::setprecision(3)
                << static_cast<double>((static_cast<uint64>(cv::getTickCount()) - start) * 1000) /
                       (loops * static_cast<double>(cv::getTickFrequency()))
                << "ms" << std::endl;
  }

  void saveResults() {}
};

PerformanceTestPtr createGradientPerformanceTest(const lsfm::DataProviderList& provider) {
  auto test = std::make_shared<PerformanceTest>();
  test->name = "Gradient";
  try {
    test->data = provider;

    // add other
  } catch (std::exception& e) {
    std::cout << test->name << " parse error: " << e.what() << std::endl;
    return PerformanceTestPtr();
  }

  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, RobertsDerivative, QuadraticMagnitude>>(),
      "Roberts (2x2)"));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, PrewittDerivative, QuadraticMagnitude>>(),
      "Prewitt (3x3)"));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, ScharrDerivative, QuadraticMagnitude>>(),
      "Scharr (3x3)"));

  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(),
      "Sobel (3x3)"));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 5)}),
      "Sobel (5x5)"));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 7)}),
      "Sobel (7x7)"));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 9)}),
      "Sobel (9x9)"));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 11)}),
      "Sobel (11x11)"));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 13)}),
      "Sobel (13x13)"));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 15)}),
      "Sobel (15x15)"));

  test->tasks.push_back(
      std::make_shared<Entry<short, int, float>>(std::make_shared<SusanGradient<short, int>>(), "Susan (37) "));
  test->tasks.push_back(
      std::make_shared<Entry<short, int, float>>(std::make_shared<SusanGradient<short, int>>(20, true), "Susan (3x3)"));

  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<RCMGradient<uchar, 1, short, int>>(3, 1), "RMG (3x3)", TASK_NO_3));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<RCMGradient<uchar, 1, short, int>>(5, 3), "RMG (5x5)", TASK_NO_5));
  test->tasks.push_back(std::make_shared<Entry<short, int, float>>(
      std::make_shared<RCMGradient<uchar, 3, short, int>>(3, 1), "RCMG (3x3)", TASK_RGB));

  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 3), NV("grad_range", 1.5)}),
      "Gauss (3x3)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 5), NV("grad_range", 2.3)}),
      "Gauss (5x5)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 7), NV("grad_range", 3.0)}),
      "Gauss (7x7)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 9), NV("grad_range", 3.5)}),
      "Gauss (9x9)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 11), NV("grad_range", 4)}),
      "Gauss (11x11)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 13), NV("grad_range", 4.5)}),
      "Gauss (13x13)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 15), NV("grad_range", 5)}),
      "Gauss (15x15)"));

  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 3), NV("grad_kernel_spacing", 1.24008)}),
      "QF_StG (3x3)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.008)}),
      "QF_StG (5x5)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 0.873226)}),
      "QF_StG (7x7)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 0.781854)}),
      "QF_StG (9x9)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 11), NV("grad_kernel_spacing", 0.7)}),
      "QF_StG (11x11)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 13), NV("grad_kernel_spacing", 0.65)}),
      "QF_StG (13x13)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 15), NV("grad_kernel_spacing", 0.6)}),
      "QF_StG (15x15)"));

  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 3), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (3x3)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (5x5)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (7x7)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (9x9)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 11), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (11x11)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 13), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (13x13)"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 15), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (15x15)"));

  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureLGF<uchar, float, PolarCV>>>(
          IL{NV("grad_waveLength", 3), NV("grad_sigmaOnf", 0.55)}),
      "SQFF LG"));
  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureSF<uchar, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2)}),
      "SQFF PO"));

  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientPC<PCLgf<uchar, float, PolarCV>>>(), "PC LGF"));

  test->tasks.push_back(std::make_shared<Entry<float, float, float>>(
      std::make_shared<GradientPC<PCSqf<uchar, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2)}),
      "PC POF"));

  test->tasks.push_back(std::make_shared<Entry<double, double, double>>(
      std::make_shared<GradientPC<PCLgf<uchar, double, PolarCV>>>(), "PC LGD"));

  test->tasks.push_back(std::make_shared<Entry<double, double, double>>(
      std::make_shared<GradientPC<PCSqf<uchar, double, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2)}),
      "PC POD"));

  test->tasks.push_back(
      std::make_shared<Entry<double, double, double>>(std::make_shared<GradientPC<PCMatlab<uchar>>>(), "PC ML"));
  return test;
}

bool addGradient() {
  addPerformanceTestCreator(createGradientPerformanceTest);
  std::cout << "Added gradient performance test" << std::endl;
  return true;
}

bool gradientAdded = addGradient();
