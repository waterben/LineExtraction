#include <edge/zc.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <imgproc/laplace.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>


using namespace std;
using namespace lsfm;
using namespace cv;


template <class ZC>
void showZC(const std::string& name, ZC& zc) {
  // cv::Mat emap = nms.directionMap();
  cv::Mat emap = zc.hysteresis();
  cv::Mat emapImg;
  emapImg.create(emap.rows, emap.cols, CV_8UC3);

  emapImg.setTo(cv::Vec3b(0, 0, 0));
  emapImg.setTo(cv::Vec3b(220, 150, 255), emap == 7);  // magenta2
  emapImg.setTo(cv::Vec3b(255, 0, 150), emap == 6);    // lila
  emapImg.setTo(cv::Vec3b(255, 0, 0), emap == 5);      // blue
  emapImg.setTo(cv::Vec3b(255, 255, 0), emap == 4);    // cyan
  emapImg.setTo(cv::Vec3b(0, 255, 0), emap == 3);      // green
  emapImg.setTo(cv::Vec3b(0, 255, 255), emap == 2);    // yellow
  emapImg.setTo(cv::Vec3b(0, 150, 255), emap == 1);    // orange
  emapImg.setTo(cv::Vec3b(0, 0, 255), emap == 0);      // red

  imshow("zc " + name, emapImg);
}

int main(int argc, char** argv) {
  const char* filename = argc >= 2 ? argv[1] : "../../images/circle2.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/ssmall.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/hall2_low.JPG";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.JPG";

  cv::Mat src = cv::imread(filename, IMREAD_GRAYSCALE);
  if (src.empty()) {
    cout << "Can not open " << filename << endl;
    return -1;
  }

  if (src.channels() > 1) cvtColor(src, src, cv::COLOR_BGR2GRAY);

  GaussianBlur(src, src, cv::Size(3, 3), 0.6);

  typedef float FT;
  DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude> sobel;
  DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude, FastDirection> sobeld;
  DerivativeGradient<uchar, short, FT, FT, SobelDerivative> sobelf;
  DerivativeGradient<uchar, short, FT, FT, SobelDerivative, Magnitude, FastDirection> sobelfd;
  LaplaceSimple<uchar, short> laplace;
  LaplaceSimple<uchar, FT> laplacef;

  ZeroCrossing<short, short, FT, FastZC<short, short, FT>> fast8_zci;
  ZeroCrossing<short, FT, FT, FastZC<short, FT, FT>> fast8_zcf;
  ZeroCrossing<short, short, FT, FastZC<short, short, FT, NCC_BASIC, EZCMap4>> fast4_zci;
  ZeroCrossing<short, FT, FT, FastZC<short, FT, FT, NCC_BASIC, EZCMap4>> fast4_zcf;

  ZeroCrossing<short, short, FT, PreciseZC<short, short, FT, NCC_BASIC, EZCMap8, LinearInterpolator, Polar>>
      precise8i_zc;
  ZeroCrossing<short, FT, FT, PreciseZC<short, FT, FT, NCC_BASIC, EZCMap8, LinearInterpolator, Polar>> precise8l_zc;
  ZeroCrossing<short, FT, FT, PreciseZC<short, FT, FT, NCC_BASIC, EZCMap8, LinearInterpolator, PolarCV>> precise8lcv_zc;
  ZeroCrossing<short, FT, FT, PreciseZC<short, FT, FT, NCC_BASIC, EZCMap8, CubicInterpolator, Polar>> precise8c_zc;
  ZeroCrossing<short, FT, FT, PreciseZC<short, FT, FT, NCC_BASIC, EZCMap4, LinearInterpolator, Polar>> precise4l_zc;


  laplace.process(src);
  fast8_zci.process(laplace);
  showZC("fast8_zci default", fast8_zci);

  sobel.process(src);
  fast8_zci.processG(laplace, sobel);
  showZC("fast8_zci grad", fast8_zci);

  fast8_zci.processD(laplace, sobel);
  showZC("fast8_zci dir", fast8_zci);

  laplacef.process(src);
  fast8_zcf.process(laplacef);
  showZC("fast8_zcf default", fast8_zcf);

  sobelf.process(src);
  fast8_zcf.processG(laplacef, sobelf);
  showZC("fast8_zcf grad", fast8_zcf);

  fast8_zcf.processD(laplacef, sobelf);
  showZC("fast8_zcf dir", fast8_zcf);

  waitKey();

  laplace.process(src);
  fast4_zci.process(laplace);
  showZC("fast4_zci default", fast4_zci);

  sobel.process(src);
  fast4_zci.processG(laplace, sobel);
  showZC("fast4_zci grad", fast4_zci);

  fast4_zci.processD(laplace, sobel);
  showZC("fast4_zci dir", fast4_zci);

  laplacef.process(src);
  fast4_zcf.process(laplacef);
  showZC("fast4_zcf default", fast4_zcf);

  sobelf.process(src);
  fast4_zcf.processG(laplacef, sobelf);
  showZC("fast4_zcf grad", fast4_zcf);

  fast4_zcf.processD(laplacef, sobelf);
  showZC("fast4_zcf dir", fast4_zcf);

  waitKey();

  laplace.process(src);
  precise8i_zc.process(laplace);
  showZC("precise8i_zc default", precise8i_zc);

  sobel.process(src);
  precise8i_zc.processG(laplace, sobel);
  showZC("precise8i_zc gard", precise8i_zc);

  precise8i_zc.processD(laplace, sobel);
  showZC("precise8i_zc dir", precise8i_zc);

  waitKey();

  laplacef.process(src);
  precise8l_zc.process(laplacef);
  showZC("precise8l_zc default", precise8l_zc);

  sobelf.process(src);
  precise8l_zc.processG(laplacef, sobelf);
  showZC("precise8l_zc gard", precise8l_zc);

  precise8l_zc.processD(laplacef, sobelf);
  showZC("precise8l_zc dir", precise8l_zc);

  waitKey();

  laplacef.process(src);
  precise8lcv_zc.process(laplacef);
  showZC("precise8lcv_zc default", precise8lcv_zc);

  sobelf.process(src);
  precise8lcv_zc.processG(laplacef, sobelf);
  showZC("precise8lcv_zc gard", precise8lcv_zc);

  precise8lcv_zc.processD(laplacef, sobelf);
  showZC("precise8lcv_zc dir", precise8lcv_zc);

  waitKey();

  laplacef.process(src);
  precise8c_zc.process(laplacef);
  showZC("precise8c_zc default", precise8c_zc);

  sobelf.process(src);
  precise8c_zc.processG(laplacef, sobelf);
  showZC("precise8c_zc gard", precise8c_zc);

  precise8c_zc.processD(laplacef, sobelf);
  showZC("precise8c_zc dir", precise8c_zc);

  waitKey();

  laplacef.process(src);
  precise4l_zc.process(laplacef);
  showZC("precise4l_zc default", precise4l_zc);

  sobelf.process(src);
  precise4l_zc.processG(laplacef, sobelf);
  showZC("precise4l_zc gard", precise4l_zc);

  precise4l_zc.processD(laplacef, sobelf);
  showZC("precise4l_zc dir", precise4l_zc);

  waitKey();

  return 0;
}
