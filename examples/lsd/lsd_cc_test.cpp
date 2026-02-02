#include <eval/test_images.hpp>
#include <geometry/draw.hpp>
#include <imgproc/image_operator.hpp>
#include <lsd/lsd_burns.hpp>
#include <lsd/lsd_cc.hpp>
#include <lsd/lsd_cp.hpp>
#include <lsd/lsd_el.hpp>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;
using namespace lsfm;

static void help() {
  cout << "\nThis program demonstrates cc lsd.\n"
          "Usage:\n"
          "./test_lsd_data <image_name>, Default is ../images/office1_low.jpg\n"
       << endl;
}

int main(int argc, char** argv) {
  lsfm::TestImages::init(argv[0]);
  std::string filename = argc >= 2 ? argv[1] : lsfm::TestImages::windmill();

  cv::Mat src = cv::imread(filename, 0);
  if (src.empty()) {
    help();
    cout << "Can not open " << filename << endl;
    return -1;
  }

  if (src.channels() != 1) cvtColor(src, src, cv::COLOR_RGB2GRAY);
  cv::Mat srcS;
  resize(src, srcS, cv::Size(0, 0), 0.5, 0.5);

  // PipelineOperator pipe;
  // pipe.push(ResizeOperator::create(320, 320 * src.rows / src.cols));
  // pipe.push(GaussianNoiseOperator::create(5));
  // pipe.push(GaussianBlurOperator::create(0.6));
  // pipe.apply(src);

  // Reduce noise with a kernel 3x3
  // blur(src, src, Size(3, 3));
  GaussianBlur(src, src, cv::Size(3, 3), 0.8);

  /*VideoCapture capture("../../video/v1.mp4");
  for (int i = 0; i != 819; ++i)
      capture.grab();

  capture.retrieve(src, IMREAD_GRAYSCALE);*/

  typedef float FT;

  // LsdCC<FT> lsd(0.004, 0.012, 5, 0, 2, CC_FIND_NEAR_COMPLEX);
  // LsdBurns<FT> lsd(0.004, 0.012, 5, 12, BURNS_NMS);
  LsdEL<FT> lsd;


  double duration_ms = 0;
  int run = 10;
  for (int i = 0; i != run; ++i) {
    double start = double(cv::getTickCount());
    lsd.detect(src);
    duration_ms += (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency();
  }


  const LineSegment2Vector<FT>& lines = lsd.lineSegments();
  vector<cv::Mat> imageData = lsd.imageData();

  std::cout << lines.size() << " line segments found. For " << duration_ms / run << " ms." << std::endl;

  DataDescriptor idd = lsd.imageDataDescriptor();
  cv::RNG rng(static_cast<uint64>(time(nullptr)));

  cv::Mat emap = lsd.imageData()[3];
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

  imshow("edge map", emapImg);

  /*cv::Mat lmap = lsd.imageData()[4];
  double lmin = 0, lmax = 0;
  minMaxLoc(lmap,&lmin,&lmax);
  // create color map for segments


  cv::Mat lmapImg;
  //lmapImg.create(lmap.rows, lmap.cols, CV_8UC3);
  //lmapImg.setTo(Vec3b(0, 0, 0));
  cvtColor(src, lmapImg, cv::COLOR_GRAY2BGR);*/

  /*for_each(lsd.segments().begin(), lsd.segments().end(), [&](const LsdCC<FT>::LineData &seg){
      cv::Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
      for_each(seg.begin(), seg.end(), [&](const LsdCC<FT>::point_type& p){
          set<cv::Vec3b>(lmapImg, p, color);
      });
  });*/

  /*for (int i = 0; i != (int)lmax; ++i) {
      lmapImg.setTo(Scalar(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225)), lmap == i +
  1);
  }
  lmapImg.setTo(Scalar(255,255,255), lmap == -2);*/

  // imshow("segment map", lmapImg);

  /*cv::Mat pImg;
  pImg.create(src.rows, src.cols, CV_8UC3);
  pImg.setTo(Vec3b(0, 0, 0));

  const CPLineSegmentDetector<MyType>::PatternVector &patterns = lsd.patterns();
  const CPLineSegmentDetector<MyType>::PointVector &points = lsd.linePoints();
  for_each(patterns.begin(), patterns.end(), [&](const CPLineSegmentDetector<MyType>::Pattern &p){
      Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
      for_each(points.begin() + p.beg, points.begin() + p.endpos(), [&](const CPLineSegmentDetector<MyType>::Point &p){
          pImg.at<Vec3b>(p) = color;
      });
      //if (p.flags & 2)
      //    circle(pImg, points[p.end() - 1], 1, Scalar(color[0], color[1], color[2]));
  });

  imshow("Detected patterns", pImg);*/

  cv::Mat img;
  cvtColor(src, img, cv::COLOR_GRAY2BGR);
  for_each(lines.begin(), lines.end(), [&](const LineSegment<FT>& l) {
    cv::Vec3b color(static_cast<unsigned char>(20 + rng.uniform(0, 225)),
                    static_cast<unsigned char>(20 + rng.uniform(0, 225)),
                    static_cast<unsigned char>(20 + rng.uniform(0, 225)));
    cv::Scalar scolor(color[0], color[1], color[2]);
    line(img, l, scolor);
    // Vec2<FT> p1 = l.normalLineDist(0, l.centerPoint()), p2 = l.normalLineDist(10, l.centerPoint());
    // line(img, p1, p2, cv::Scalar(0, 0, 255));
  });


  imshow("Detected lines", img);

  cv::waitKey();

  return 0;
}
