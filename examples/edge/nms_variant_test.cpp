#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <edge/nms.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;


template<class NMS>
void showNMS(const std::string &name,NMS &nms) {

    //cv::Mat emap = nms.directionMap();
    cv::Mat emap = nms.hysteresis();
    cv::Mat emapImg;
    emapImg.create(emap.rows, emap.cols, CV_8UC3);

    emapImg.setTo(cv::Vec3b(0, 0, 0));
    emapImg.setTo(cv::Vec3b(220, 150, 255), emap == 7); // magenta2
    emapImg.setTo(cv::Vec3b(255, 0, 150), emap == 6); // lila
    emapImg.setTo(cv::Vec3b(255, 0, 0), emap == 5); // blue
    emapImg.setTo(cv::Vec3b(255, 255, 0), emap == 4); // cyan
    emapImg.setTo(cv::Vec3b(0, 255, 0), emap == 3); // green
    emapImg.setTo(cv::Vec3b(0, 255, 255), emap == 2); // yellow
    emapImg.setTo(cv::Vec3b(0, 150, 255), emap == 1); // orange
    emapImg.setTo(cv::Vec3b(0, 0, 255), emap == 0); // red
    
    imshow("nms " + name,emapImg);
}

int main(int argc, char** argv)
{
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

  NonMaximaSuppression<short, int, FT, FastNMS8<short, int, FT>> fast8_nmsi;
  NonMaximaSuppression<short, FT, FT, FastNMS8<short, FT, FT>> fast8_nmsf;
  NonMaximaSuppression<short, int, FT, FastNMS4<short, int, FT>> fast4_nmsi;
  NonMaximaSuppression<short, FT, FT, FastNMS4<short, FT, FT>> fast4_nmsf;

  NonMaximaSuppression<short, int, FT, PreciseNMS<short, int, true, FT, EMap8, LinearInterpolator, Polar>>
      precise8i_nms;
  NonMaximaSuppression<short, FT, FT, PreciseNMS<short, FT, false, FT, EMap8, LinearInterpolator, Polar>> precise8l_nms;
  NonMaximaSuppression<short, FT, FT, PreciseNMS<short, FT, false, FT, EMap8, LinearInterpolator, PolarCV>>
      precise8lcv_nms;
  NonMaximaSuppression<short, FT, FT, PreciseNMS<short, FT, false, FT, EMap8, CubicInterpolator, Polar>> precise8c_nms;
  NonMaximaSuppression<short, FT, FT, PreciseNMS<short, FT, false, FT, EMap4, LinearInterpolator, Polar>> precise4l_nms;


  sobel.process(src);
  fast8_nmsi.process(sobel);
  showNMS("fast8_nmsi default", fast8_nmsi);

  fast8_nmsi.processD(sobel);
  showNMS("fast8_nmsi dir half", fast8_nmsi);

  sobeld.process(src);
  fast8_nmsi.processD(sobeld);
  showNMS("fast8_nmsi dir full", fast8_nmsi);

  sobelf.process(src);
  fast8_nmsf.process(sobelf);
  showNMS("fast8_nmsf default", fast8_nmsf);

  fast8_nmsf.processD(sobelf);
  showNMS("fast8_nmsf dir half", fast8_nmsf);

  sobelfd.process(src);
  fast8_nmsf.processD(sobelfd);
  showNMS("fast8_nmsf dir full", fast8_nmsf);

  waitKey();

  sobel.process(src);
  fast4_nmsi.process(sobel);
  showNMS("fast4_nmsi default", fast4_nmsi);

  fast4_nmsi.processD(sobel);
  showNMS("fast4_nmsi dir half", fast4_nmsi);

  sobeld.process(src);
  fast4_nmsi.processD(sobeld);
  showNMS("fast4_nmsi dir full", fast4_nmsi);

  sobelf.process(src);
  fast4_nmsf.process(sobelf);
  showNMS("fast4_nmsf default", fast4_nmsf);

  fast4_nmsf.processD(sobelf);
  showNMS("fast4_nmsf dir half", fast4_nmsf);

  sobelfd.process(src);
  fast4_nmsf.processD(sobelfd);
  showNMS("fast4_nmsf dir full", fast4_nmsf);

  waitKey();

  sobel.process(src);
  precise8i_nms.process(sobel);
  showNMS("precise8i_nms default", precise8i_nms);

  precise8i_nms.processD(sobel);
  showNMS("precise8i_nms dir half", precise8i_nms);

  sobeld.process(src);
  precise8i_nms.processD(sobeld);
  showNMS("precise8i_nms dir full", precise8i_nms);

  waitKey();

  sobelf.process(src);
  precise8l_nms.process(sobelf);
  showNMS("precise8l_nms default", precise8l_nms);

  precise8l_nms.processD(sobelf);
  showNMS("precise8l_nms dir half", precise8l_nms);

  sobelfd.process(src);
  precise8l_nms.processD(sobelfd);
  showNMS("precise8l_nms dir full", precise8l_nms);

  waitKey();

  sobelf.process(src);
  precise8lcv_nms.process(sobelf);
  showNMS("precise8lcv_nms default", precise8lcv_nms);

  precise8lcv_nms.processD(sobelf);
  showNMS("precise8lcv_nms dir half", precise8lcv_nms);

  sobelfd.process(src);
  precise8lcv_nms.processD(sobelfd);
  showNMS("precise8lcv_nms dir full", precise8lcv_nms);

  waitKey();

  sobelf.process(src);
  precise8c_nms.process(sobelf);
  showNMS("precise8c_nms default", precise8c_nms);

  precise8c_nms.processD(sobelf);
  showNMS("precise8c_nms dir half", precise8c_nms);

  sobelfd.process(src);
  precise8c_nms.processD(sobelfd);
  showNMS("precise8c_nms dir full", precise8c_nms);

  waitKey();

  sobelf.process(src);
  precise4l_nms.process(sobelf);
  showNMS("precise4l_nms default", precise4l_nms);

  precise4l_nms.processD(sobelf);
  showNMS("precise4l_nms dir half", precise4l_nms);

  sobelfd.process(src);
  precise4l_nms.processD(sobelfd);
  showNMS("precise4l_nms dir full", precise4l_nms);

  waitKey();

  return 0;
}
