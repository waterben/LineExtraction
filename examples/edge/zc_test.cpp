#include <edge/zc.hpp>
#include <imgproc/image_operator.hpp>
#include <imgproc/laplace.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>


using namespace std;
using namespace lsfm;
using namespace cv;

typedef double FT;

template <class LAPLACE>
void testLaplace(LAPLACE& grad, const Mat& src, const std::string& name) {
  int runs = 1;
  int64 rt = 0, tmp;
  for (int i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    grad.process(src);
    grad.laplace();
    rt += cv::getTickCount() - tmp;
  }

  std::cout << "laplace - " << name << ": " << (rt * 1000.0 / cv::getTickFrequency()) / runs << std::endl;
}

template <class ZC, class LAPLACE>
void testZC(ZC& zc, LAPLACE& laplace, const std::string& name, double th_low = 0.004, double th_high = 0.008) {
  int runs = 1;
  int64 rt = 0, tmp;
  zc.threshold(th_low, th_high);
  for (int i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    zc.process(laplace);
    rt += cv::getTickCount() - tmp;
  }

  std::cout << "zc - " << name << ": "
            << (static_cast<double>(rt) * 1000.0 / static_cast<double>(cv::getTickFrequency())) / runs << std::endl;
}

template <class LAPLACE>
void showLaplace(const std::string& name, LAPLACE& laplace, int use_range = 0) {
  cv::Mat l;
  laplace.laplace().convertTo(l, CV_32F);
  if (use_range) {
    l = (l - laplace.laplaceRange().lower) / laplace.laplaceRange().size();
  } else {
    double vmin, vmax;
    cv::minMaxIdx(l, &vmin, &vmax);
    l = (l - vmin) / (vmax - vmin);
    // std::cout << "gradient max - " << name << ": " << vmax << std::endl;
  }
  imshow("laplace " + name, l);
}

void showN(const std::string& name, const cv::Mat& img, double mul = 1) {
  cv::Mat out;
  img.convertTo(out, CV_32F);

  double vmin, vmax;
  cv::minMaxIdx(out, &vmin, &vmax);
  std::cout << vmin << ", " << vmax << std::endl;
  out = (out - vmin) * (mul / (vmax - vmin));

  imshow("normalized " + name, out);
}

cv::Mat createNMS(const cv::Mat& emap) {
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
  return emapImg;
}

template <class ZC>
void showZC(const std::string& name, ZC& zc, bool use_dir = true) {
  // cv::Mat emap = zc.directionMap();
  cv::Mat emap = zc.hysteresis();
  cv::Mat emapImg;
  emapImg.create(emap.rows, emap.cols, CV_8UC3);

  if (use_dir) {
    emapImg = createNMS(emap);
  } else {
    emapImg.setTo(cv::Vec3b(255, 255, 255), emap >= 0);
  }

  imshow("zc " + name, emapImg);
}


double phaseError(const cv::Mat& gt, const cv::Mat& dir, bool hr = false) {
  cv::Mat res = cv::abs(gt - dir);
  cv::subtract(2 * CV_PI, res, res, res > CV_PI);
  if (hr) cv::subtract(CV_PI, res, res, res > CV_PI / 2);
  res.setTo(0, dir < 0.0001);
  imshow("phase error", res / CV_PI);
  return cv::sum(res)[0];
}

int main(int argc, char** argv) {
  // const char* filename = argc >= 2 ? argv[1] : "../../images/circle2.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/hall2_low.JPG";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.JPG";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/bike.png";
  const char* filename = argc >= 2 ? argv[1] : "../../images/geom.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/Mono/step_line.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/Mono/synthetic3d.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/Mono/blox.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/lopez.png";

  cv::Mat src = cv::imread(filename, IMREAD_GRAYSCALE);
  if (src.empty()) {
    cout << "Can not open " << filename << endl;
    return -1;
  }

  if (src.channels() > 1) cvtColor(src, src, cv::COLOR_BGR2GRAY);

  // GaussianBlur(src, src, cv::Size(3, 3), 0.6);

  imshow("img", src);

  LaplaceSimple<uchar, short> laplace;
  LaplaceCV<uchar, short> laplaceCV;
  LoG<uchar, FT> log;
  QuadratureG2<uchar, FT> quad;
  QuadratureLGF<uchar, FT> quadLGF;
  PCLSq<uchar, FT, FT> quadS(1, 2, 5, 1.2);
  PCLSqf<uchar, FT> quadSF(1, 2, 2);

  ZeroCrossing<short, short, FT, FastZC<short, short, FT>> s_zc;
  ZeroCrossing<FT, FT, FT, FastZC<FT, FT, FT>> f_zc;

  laplace.process(src);
  laplaceCV.process(src);
  log.process(src);
  quad.process(src);
  quadLGF.process(src);
  quadS.process(src);
  quadSF.process(src);

  testZC(s_zc, laplace, "default");
  showZC("default", s_zc);
  testZC(s_zc, laplaceCV, "CV");
  showZC("CV", s_zc);
  testZC(f_zc, log, "log");
  showZC("log", f_zc);

  f_zc.processQ(quad);
  showZC("quad", f_zc);

  f_zc.processQ(quadLGF);
  showZC("quadLGF", f_zc);

  f_zc.processQ(quadS);
  showZC("quadS", f_zc);

  f_zc.processQ(quadSF);
  showZC("quadSF", f_zc);

  cv::Mat lx, ly, tmp;
  quadS.pcLaplace(lx, ly);
  f_zc.process(quadS, lx);
  tmp = f_zc.hysteresis() > -1;
  showZC("quadSzcx", f_zc);
  f_zc.process(quadS, ly);
  showZC("quadSyzc", f_zc);

  imshow("quadSzc", (tmp | (f_zc.hysteresis() > -1)) > 0);

  quadSF.pcLaplace(lx, ly);
  f_zc.process(quadSF, lx);
  tmp = f_zc.hysteresis() > -1;
  showZC("quadSFzcx", f_zc);
  f_zc.process(quadSF, ly);
  showZC("quadSFzcy", f_zc);

  imshow("quadSFzc", (tmp | (f_zc.hysteresis() > -1)) > 0);

  /*showMat("phase QSF", quadSF.phaseF());
  showMat("phase QS", quadS.phaseF());
  std::cout << "pe: " << phaseError(quadSF.phaseF(), quadS.phaseF()) / quadS.phase().size().area() << std::endl;*/

  quadSF.pcLaplace(lx, ly);
  f_zc.process(quadSF.direction(), lx, quadSF.laplaceThreshold(f_zc.thresholdLow()),
               quadSF.laplaceThreshold(f_zc.thresholdHigh()), quadSF.directionRange().lower,
               quadSF.directionRange().upper);
  tmp = f_zc.hysteresis();
  f_zc.process(quadSF.direction(), ly, quadSF.laplaceThreshold(f_zc.thresholdLow()),
               quadSF.laplaceThreshold(f_zc.thresholdHigh()), quadSF.directionRange().lower,
               quadSF.directionRange().upper);
  // createNMS(tmp > -1 | (f_zc.hysteresis() > -1 & tmp < -1))
  imshow("quadSFcol", createNMS(tmp));

  waitKey();

  return 0;
}
