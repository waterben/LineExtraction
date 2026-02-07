/// @file nms_test.cpp
/// @brief NMS algorithm benchmarking with multiple gradient operators.
///
/// Benchmarks Non-Maximum Suppression performance with various gradient
/// computation methods including:
/// - Sobel/Scharr derivatives
/// - RCMG (Robust Color Morphological Gradient)
/// - SUSAN edge detector
/// - Quadrature filters (Gabor, Log-Gabor)
/// - Phase Congruency
///
/// Reports timing and edge pixel throughput for each configuration.
///
/// @usage ./nms_test [image_path]
/// @param image_path Optional path to input image (default: windmill.jpg)

#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#define USE_PERIODIC_FFT
#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/gradient_adapter.hpp>
#include <imgproc/image_operator.hpp>
#include <imgproc/pc_lgf.hpp>
#include <imgproc/pc_matlab.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/susan.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <utility/test_images.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;

template <class GRAD>
void testGradient(GRAD& grad, const Mat& src, const std::string& name) {
  int runs = 20;
  int64 rt = 0, tmp;
  for (int i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    grad.process(src);
    grad.magnitude();
    rt += cv::getTickCount() - tmp;
  }

  std::cout << "gradient - " << name << ": " << (rt * 1000.0 / cv::getTickFrequency()) / runs << std::endl;
}

template <class NMS, class GRAD>
void testNMS(NMS& nms,
             GRAD& grad,
             const std::string& name,
             bool force_dir = false,
             double th_low = 0.008,
             double th_high = 0.016) {
  int runs = 20;
  int64 rt = 0, tmp;
  nms.threshold(th_low, th_high);
  for (int i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    nms.process(grad, force_dir);
    rt += cv::getTickCount() - tmp;
  }

  std::cout << "nms - " << name << ": "
            << (static_cast<double>(rt) * 1000.0 / static_cast<double>(cv::getTickFrequency())) / runs << std::endl;
}

template <class GRAD>
void showGradient(const std::string& name, GRAD& grad, int use_range = 0) {
  cv::Mat mag;
  grad.magnitude().convertTo(mag, CV_32F);
  if (use_range) {
    mag /= grad.magnitudeRange().upper;
    if (use_range > 1) mag *= use_range;
  } else {
    double vmin, vmax;
    cv::minMaxIdx(mag, &vmin, &vmax);
    mag /= vmax;
    // std::cout << "gradient max - " << name << ": " << vmax << std::endl;
  }
  imshow("gradient " + name, mag);
}

template <class NMS>
void showNMS(const std::string& name, NMS& nms, bool use_dir = false) {
  // cv::Mat emap = nms.directionMap();
  cv::Mat emap = nms.hysteresis();
  cv::Mat emapImg;
  emapImg.create(emap.rows, emap.cols, CV_8UC3);
  emapImg.setTo(cv::Vec3b(0, 0, 0));

  if (use_dir) {
    emapImg.setTo(cv::Vec3b(220, 150, 255), emap == 7);  // magenta2
    emapImg.setTo(cv::Vec3b(255, 0, 150), emap == 6);    // lila
    emapImg.setTo(cv::Vec3b(255, 0, 0), emap == 5);      // blue
    emapImg.setTo(cv::Vec3b(255, 255, 0), emap == 4);    // cyan
    emapImg.setTo(cv::Vec3b(0, 255, 0), emap == 3);      // green
    emapImg.setTo(cv::Vec3b(0, 255, 255), emap == 2);    // yellow
    emapImg.setTo(cv::Vec3b(0, 150, 255), emap == 1);    // orange
    emapImg.setTo(cv::Vec3b(0, 0, 255), emap == 0);      // red
  } else {
    emapImg.setTo(cv::Vec3b(255, 255, 255), emap >= 0);
  }

  imshow("nms " + name, emapImg);
}


int main(int argc, char** argv) {
  lsfm::TestImages::init(argv[0]);
  std::string filename = argc >= 2 ? argv[1] : lsfm::TestImages::windmill();

  cv::Mat src = cv::imread(filename, IMREAD_GRAYSCALE);
  if (src.empty()) {
    cout << "Can not open " << filename << endl;
    return -1;
  }

  if (src.channels() > 1) cvtColor(src, src, cv::COLOR_BGR2GRAY);

  GaussianNoiseOperator noise(10);
  // noise.apply(src);
  GaussianBlur(src, src, cv::Size(3, 3), 0.6);

  // imshow("img",src);

  DerivativeGradient<uchar, short, int, float, RobertsDerivative, QuadraticMagnitude> roberts;
  DerivativeGradient<uchar, short, int, float, PrewittDerivative, QuadraticMagnitude> prewitt;
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  DerivativeGradient<uchar, short, int, float, ScharrDerivative, QuadraticMagnitude> scharr;
  DerivativeGradient<uchar, float, float, float, GaussianDerivative> guassian({NV("grad_kernel_size", 5)});
  lsfm::GradientEnergy<QuadratureG2<uchar, double>> quad({NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.008)});
  lsfm::GradientEnergy<QuadratureS<uchar, double, double>> quadS(
      {NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.2)});
  lsfm::GradientEnergy<QuadratureSF<uchar, double>> quadSF(
      {NV("grad_scale", 1), NV("grad_muls", 3), NV("grad_kernel_spacing", 2)});
  lsfm::GradientEnergy<QuadratureLGF<uchar, double>> quadLGF({NV("grad_waveLength", 5), NV("grad_sigmaOnf", 0.55)});
  RCMGradient<uchar, 1, short, int, float> rcmg(3, 2);
  SusanGradient<short, int> susan(10, false);
  lsfm::GradientPC<PCLgf<uchar, double>> phase({NV("grad_numScales", 4), NV("grad_minWaveLength", 3),
                                                NV("grad_mult", 2.1), NV("grad_sigmaOnf", 0.55), NV("grad_k", 3),
                                                NV("grad_cutOff", 0.5), NV("grad_g", 10), NV("grad_deviationGain", 1.5),
                                                NV("grad_noiseMethod", -2)});
  lsfm::GradientPC<PCMatlab<uchar>> phaseml({NV("grad_numScales", 4), NV("grad_minWaveLength", 3), NV("grad_mult", 2.1),
                                             NV("grad_sigmaOnf", 0.55), NV("grad_k", 3), NV("grad_cutOff", 0.5),
                                             NV("grad_g", 10), NV("grad_deviationGain", 1.5),
                                             NV("grad_noiseMethod", -2)});

  NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> si_nms;
  NonMaximaSuppression<short, int, float, FastNMS4<short, int, float>> si_nms4;
  // NonMaximaSuppression<short,int,float,FastNMS4<short,int,float>> si_nms;
  // NonMaximaSuppression<short,int,float,PreciseNMS<short,int,true,float,EMap8,LinearInterpolator, PolarCV>> si_nms;
  // NonMaximaSuppression<short,int,float,PreciseNMS<short,int,true,float,EMap4,LinearInterpolator, PolarCV>> si_nms;
  NonMaximaSuppression<float, float, float, FastNMS8<float, float, float>> f_nms;
  NonMaximaSuppression<double, double, double, FastNMS8<double, double, double>> d_nms;
  // NonMaximaSuppression<float,float,float,FastNMS4<float,float,float>> f_nms;
  // NonMaximaSuppression<float,float,float,PreciseNMS<float,float,false,float,EMap8,LinearInterpolator, PolarCV>>
  // f_nms; NonMaximaSuppression<float, float, float, PreciseNMS<float, float, false, float, EMap4, LinearInterpolator,
  // PolarCV>> f_nms;
  // NonMaximaSuppression<double,double> d_nms;
  // NonMaximaSuppression<double,double,double,PreciseNMS<double,double,double,EMap8<double>>> d_nms;


  roberts.process(src);
  prewitt.process(src);
  sobel.process(src);
  scharr.process(src);
  guassian.process(src);
  quad.process(src);
  quadS.process(src);
  quadSF.process(src);
  quadLGF.process(src);
  susan.process(src);
  rcmg.process(src);
  phase.process(src);
  phaseml.process(src);


  /*testGradient(roberts,src,"roberts");
  testGradient(prewitt,src,"prewitt");
  testGradient(sobel,src,"sobel");
  testGradient(scharr,src,"scharr");
  testGradient(guassian,src,"guassianF");
  testGradient(quad,src,"quad");
  testGradient(quadS, src, "quadS");
  testGradient(quadSF, src, "quadSF");
  testGradient(quadLGF, src, "quadLGF");
  testGradient(susan,src,"susan");
  testGradient(rcmg,src,"rcmg");
  testGradient(phase, src, "phase");
  testGradient(phaseml, src, "phaseml");*/

  /*showGradient("roberts",roberts);
  showGradient("prewitt",prewitt);
  showGradient("sobel",sobel);
  showGradient("scharr",scharr);
  showGradient("guassian",guassian);
  showGradient("quad",quad);
  showGradient("quadS", quadS);
  showGradient("quadSF", quadSF);
  showGradient("quadLGF", quadLGF);
  showGradient("susan",susan);
  showGradient("rcmg",rcmg);
  showGradient("phase", phase);
  showGradient("phaseml", phaseml);*/

  testNMS(si_nms, roberts, "roberts");
  showNMS("roberts", si_nms);
  testNMS(si_nms, prewitt, "prewitt");
  showNMS("prewitt", si_nms);
  testNMS(si_nms, sobel, "sobel");
  showNMS("sobel", si_nms);
  testNMS(si_nms, scharr, "scharr");
  showNMS("scharr", si_nms);
  testNMS(f_nms, guassian, "guassian");
  showNMS("guassian", f_nms);
  testNMS(d_nms, quad, "quad");
  showNMS("quad", d_nms);
  testNMS(d_nms, quadS, "quadS");
  showNMS("quadS", d_nms);
  testNMS(d_nms, quadSF, "quadSF");
  showNMS("quadSF", d_nms);
  testNMS(d_nms, quadLGF, "quadLGF");
  showNMS("quadLGF", d_nms);
  testNMS(si_nms4, susan, "susan");
  showNMS("susan", si_nms4);
  testNMS(si_nms4, rcmg, "rcmg");
  showNMS("rcmg", si_nms4);
  testNMS(d_nms, phase, "phase");  //, true, 0.03, 0.07);
  showNMS("phase", d_nms);
  testNMS(d_nms, phaseml, "phaseml");  // , true, 0.03, 0.07);
  showNMS("phaseml", d_nms);

  waitKey();

  return 0;
}
