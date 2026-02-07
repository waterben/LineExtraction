/// @file grad_test.cpp
/// @brief Gradient operators (Sobel, RCMG, SUSAN) comparison.
///
/// Benchmarks and compares various gradient computation methods:
/// - Sobel/Scharr derivatives
/// - RCMG (Robust Color Morphological Gradient)
/// - SUSAN edge/corner detector
///
/// Includes automatic threshold estimation using Otsu's method.
///
/// @usage ./grad_test [image_path]
/// @param image_path Optional path to input image (default: windmill.jpg)

#include <edge/nms.hpp>
#include <edge/threshold_estimator.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/susan.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <utility/test_images.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>


using namespace std;
using namespace lsfm;
using namespace cv;

template <class OP>
void testOP(OP& op, const Mat& src, const std::string& name) {
  int runs = 2;
  int64 rt = 0, tmp;
  for (int i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    op.process(src);
    rt += cv::getTickCount() - tmp;
  }

  std::cout << name << ": " << (static_cast<double>(rt) * 1000.0 / cv::getTickFrequency()) / runs << std::endl;
}

template <class OP>
void showOp(const std::string& name, OP& op, int use_range = 0) {
  cv::Mat mag;
  op.magnitude().copyTo(mag);
  if (use_range) {
    mag /= op.magnitudeRange().upper;
    if (use_range > 1) mag *= use_range;
  } else {
    double vmin, vmax;
    cv::minMaxIdx(mag, &vmin, &vmax);
    mag /= vmax;
  }
  imshow(name, mag);
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
  GaussianBlur(src, src, cv::Size(3, 3), 0.8);

  ThresholdOtsu<uchar, 256, float> otsu;

  imshow("img", src);

  double tmp = static_cast<double>(cv::getTickCount());
  int th = otsu.process(src);
  std::cout << "otsu: " << th << ", "
            << (static_cast<double>(cv::getTickCount() - static_cast<int64>(tmp)) * 1000.0 / cv::getTickFrequency())
            << std::endl;
  cv::Mat src_th;
  src.copyTo(src_th);
  src_th.setTo(0, src < th);
  src_th.setTo(255, src >= th);

  imshow("img th", src_th);
  DerivativeGradient<uchar, short, float, float, RobertsDerivative> roberts;
  DerivativeGradient<uchar, short, float, float, PrewittDerivative> prewitt;
  DerivativeGradient<uchar, short, float, float, SobelDerivative> sobel;
  DerivativeGradient<uchar, short, float, float, ScharrDerivative> scharr;
  DerivativeGradient<uchar, short, float, float, GaussianDerivative> guassianI;
  DerivativeGradient<uchar, float, float, float, GaussianDerivative> guassianF;
  RCMGradient<uchar, 1, float, float, float> rcmg(3, 2);
  SusanGradient<short, float> susan(6, true);

  guassianI.value("kernel_size", 3);
  guassianI.value("range", 1);
  guassianI.value("scale", 10);
  // std::cout << guassianI.magnitudeRange().upper << std::endl;

  /*guassianF.value("kernel_size",5);
  guassianF.value("range",3);
  guassianF.value("scale",1);*/
  // std::cout << guassianF.magnitudeRange().upper << std::endl;

  roberts.process(src);
  prewitt.process(src);
  sobel.process(src);
  scharr.process(src);
  guassianI.process(src);
  // quad.process(src);
  // double minval, maxval;
  // cv::minMaxIdx(quad.magnitude(),&minval,&maxval);
  // std::cout << "quad mag upper: " << quad.magnitudeRange().upper << ", quad mag max: " << maxval << std::endl;
  // guassianF.process(src);
  susan.process(src);
  rcmg.process(src);
  // phase.process(src.colRange(0,512));

  testOP(roberts, src, "roberts");
  testOP(prewitt, src, "prewitt");
  testOP(sobel, src, "sobel");
  testOP(scharr, src, "scharr");
  testOP(guassianI, src, "guassianI");
  // testOP(quad,src,"quad");
  testOP(guassianF, src, "guassianF");
  testOP(susan, src, "susan");
  testOP(rcmg, src, "rcmg");
  // testOP(phase,src.colRange(0,512),"phase");

  showOp("roberts", roberts);
  showOp("prewitt", prewitt);
  showOp("sobel", sobel);
  showOp("scharr", scharr);
  showOp("guassianI", guassianI);
  // showOp("quad",quad);
  showOp("guassianF", guassianF);
  showOp("susan", susan);
  showOp("rcmg", rcmg);
  // showOp("phase",phase);

  ThresholdOtsu<float, 256, float> otsu2(sobel.magnitudeRange().upper);
  tmp = static_cast<double>(cv::getTickCount());
  cv::Mat mag;
  sobel.magnitude().copyTo(mag);
  th = static_cast<int>(otsu2.process(mag));
  std::cout << "otsu2: " << th << ", "
            << (static_cast<double>(cv::getTickCount() - static_cast<int64>(tmp)) * 1000.0 / cv::getTickFrequency())
            << std::endl;
  mag.setTo(0, mag < th);

  imshow("sobel_th", mag / sobel.magnitudeRange().upper * 4);

  // mag = sobel.magnitude();

  int histSize = 256;
  float range[] = {0, sobel.magnitudeRange().upper};
  const float* histRange = {range};

  cv::Mat hist;
  cv::calcHist(&mag, 1, nullptr, Mat(), hist, 1, &histSize, &histRange, true, false);

  int hist_w = 512;
  int hist_h = 400;
  int bin_w = cvRound(static_cast<double>(hist_w) / histSize);

  Mat histImage(hist_h, hist_w, CV_8U, Scalar(0, 0, 0));

  /// Normalize the result to [ 0, histImage.rows ]
  cv::normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());


  /// Draw for each channel
  for (int i = 1; i < histSize; i++) {
    cv::line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist.at<float>(i - 1))),
             cv::Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i))), cv::Scalar(255, 0, 0), 2, 8, 0);
  }

  imshow("calcHist Demo", histImage);


  waitKey();

  return 0;
}
