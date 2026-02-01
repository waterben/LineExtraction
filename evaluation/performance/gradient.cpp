/// @file gradient.cpp
/// @brief Gradient operator performance tests
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


/// @brief Generic gradient performance task
///
/// @tparam GT Gradient type
/// @tparam MT Magnitude type
/// @tparam FT Float type
template <class GT, class MT, class FT>
class GradientEntry : public CVPerformanceTaskBase {
 public:
  GradientEntry() : CVPerformanceTaskBase(""), gradient_() {}

  GradientEntry(std::shared_ptr<GradientI<uchar, GT, MT, FT>> g, const std::string& n, int flags = 0)
      : CVPerformanceTaskBase(n, flags), gradient_(std::move(g)) {}

 protected:
  void prepareImpl(const cv::Mat& src) override {
    // Warmup run
    gradient_->process(src);
    gradient_->magnitude();
  }

  void runImpl(const std::string& /*src_name*/, const cv::Mat& src) override {
    gradient_->process(src);
    cv::Mat tmp = gradient_->magnitude();
    static_cast<void>(tmp);
  }

 private:
  std::shared_ptr<GradientI<uchar, GT, MT, FT>> gradient_;
};


/// @brief Create gradient performance test with all gradient operators
CVPerformanceTestPtr createGradientPerformanceTest(const DataProviderList& provider) {
  auto test = std::make_shared<CVPerformanceTest>(provider, "Gradient");

  // Roberts, Prewitt, Scharr
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, RobertsDerivative, QuadraticMagnitude>>(),
      "Roberts (2x2)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, PrewittDerivative, QuadraticMagnitude>>(),
      "Prewitt (3x3)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, ScharrDerivative, QuadraticMagnitude>>(),
      "Scharr (3x3)"));

  // Sobel variants
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(),
      "Sobel (3x3)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 5)}),
      "Sobel (5x5)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 7)}),
      "Sobel (7x7)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 9)}),
      "Sobel (9x9)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 11)}),
      "Sobel (11x11)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 13)}),
      "Sobel (13x13)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>>(
          IL{NV("grad_kernel_size", 15)}),
      "Sobel (15x15)"));

  // SUSAN
  test->input_tasks.push_back(
      std::make_shared<GradientEntry<short, int, float>>(std::make_shared<SusanGradient<short, int>>(), "Susan (37) "));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<SusanGradient<short, int>>(20, true), "Susan (3x3)"));

  // RCM Gradient
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<RCMGradient<uchar, 1, short, int>>(3, 1), "RMG (3x3)", TASK_NO_3));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<RCMGradient<uchar, 1, short, int>>(5, 3), "RMG (5x5)", TASK_NO_5));
  test->input_tasks.push_back(std::make_shared<GradientEntry<short, int, float>>(
      std::make_shared<RCMGradient<uchar, 3, short, int>>(3, 1), "RCMG (3x3)", TASK_RGB));

  // Gaussian
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 3), NV("grad_range", 1.5)}),
      "Gauss (3x3)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 5), NV("grad_range", 2.3)}),
      "Gauss (5x5)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 7), NV("grad_range", 3.0)}),
      "Gauss (7x7)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 9), NV("grad_range", 3.5)}),
      "Gauss (9x9)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 11), NV("grad_range", 4)}),
      "Gauss (11x11)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 13), NV("grad_range", 4.5)}),
      "Gauss (13x13)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>>(
          IL{NV("grad_kernel_size", 15), NV("grad_range", 5)}),
      "Gauss (15x15)"));

  // Quadrature filters (G2)
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 3), NV("grad_kernel_spacing", 1.24008)}),
      "QF_StG (3x3)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.008)}),
      "QF_StG (5x5)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 0.873226)}),
      "QF_StG (7x7)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 0.781854)}),
      "QF_StG (9x9)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 11), NV("grad_kernel_spacing", 0.7)}),
      "QF_StG (11x11)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 13), NV("grad_kernel_spacing", 0.65)}),
      "QF_StG (13x13)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureG2<uchar, float, PolarCV>>>(
          IL{NV("grad_kernel_size", 15), NV("grad_kernel_spacing", 0.6)}),
      "QF_StG (15x15)"));

  // Quadrature filters (S/Poisson)
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 3), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (3x3)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (5x5)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (7x7)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (9x9)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 11), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (11x11)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 13), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (13x13)"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 15), NV("grad_kernel_spacing", 1.2)}),
      "SQF PO (15x15)"));

  // Quadrature filters (FFT-based)
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureLGF<uchar, float, PolarCV>>>(
          IL{NV("grad_waveLength", 3), NV("grad_sigmaOnf", 0.55)}),
      "SQFF LG"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientEnergy<QuadratureSF<uchar, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2)}),
      "SQFF PO"));

  // Phase Congruency
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientPC<PCLgf<uchar, float, PolarCV>>>(), "PC LGF"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<float, float, float>>(
      std::make_shared<GradientPC<PCSqf<uchar, float, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2)}),
      "PC POF"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<double, double, double>>(
      std::make_shared<GradientPC<PCLgf<uchar, double, PolarCV>>>(), "PC LGD"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<double, double, double>>(
      std::make_shared<GradientPC<PCSqf<uchar, double, PolarCV>>>(
          IL{NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2)}),
      "PC POD"));
  test->input_tasks.push_back(std::make_shared<GradientEntry<double, double, double>>(
      std::make_shared<GradientPC<PCMatlab<uchar>>>(), "PC ML"));

  return test;
}

bool addGradient() {
  addPerformanceTestCreator(createGradientPerformanceTest);
  std::cout << "Added gradient performance test" << std::endl;
  return true;
}

// bool gradientAdded = addGradient();
