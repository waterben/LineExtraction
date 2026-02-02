#include <geometry/draw.hpp>
#include <lfd/FeatureDescriptorLBD.hpp>
#include <lfd/LRDescriptor.hpp>
#include <lfd/PairwiseLineMatcher.hpp>
#include <lfd/StereoLineFilter.hpp>
#include <lsd/lsd_cc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <utility/test_images.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>


using namespace std;
using namespace lsfm;
using namespace cv;


int main(int argc, char** argv) {
  TestImages::init(argv[0]);

  // Get stereo pair - defaults to Adirondack scene from MDB dataset
  auto [filename1, filename2] = TestImages::stereoPair("Adirondack", "H");

  if (argc > 2) {
    filename1 = argv[1];
    filename2 = argv[2];
  }

  cv::Mat srcL = imread(filename1, IMREAD_GRAYSCALE);
  cv::Mat srcR = imread(filename2, IMREAD_GRAYSCALE);

  if (srcL.empty() || srcR.empty()) {
    cout << "Can not open files" << endl;
    return -1;
  }

  resize(srcL, srcL, Size(0, 0), 0.6, 0.6);
  resize(srcR, srcR, Size(0, 0), 0.6, 0.6);

  // GaussianBlur(srcL, srcL, Size(3, 3), 0.3);
  // GaussianBlur(srcR, srcR, Size(3, 3), 0.3);

  typedef double MyFloat;

  LsdCC<MyFloat> lsdL(0.004, 0.008, 20, 0, 3);
  LsdCC<MyFloat> lsdR(0.004, 0.008, 20, 0, 3);

  double start = double(getTickCount());
  lsdL.detect(srcL);
  lsdR.detect(srcR);
  double end = double(getTickCount());
  std::cout << "lines: " << lsdL.lineSegments().size() + lsdR.lineSegments().size()
            << ", time for detecting lines: " << (end - start) * 1000 / getTickFrequency() << "ms" << std::endl;

  MatMap dataL, dataR;
  dataL["gx"] = lsdL.imageData()[0];
  dataL["gy"] = lsdL.imageData()[1];
  dataL["img"] = srcL;

  dataR["gx"] = lsdR.imageData()[0];
  dataR["gy"] = lsdR.imageData()[1];
  dataR["img"] = srcR;

  // GchGradImgInterpolate helper type - kept for reference/future use
  // typedef GchGradImgInterpolate<MyFloat, 1, 2, NoAlign<MyFloat>, FastRoundNearestInterpolator<MyFloat, short>,
  //                               FastRoundNearestInterpolator<MyFloat, uchar>>
  //     MyGchHelper;

  // typedef FdcGenericLR<MyFloat,LsdCC<MyFloat>::LineSegment,MyGchHelper> MyFdc;
  typedef FdcLBD<MyFloat, LsdCC<MyFloat>::LineSegment, short, FastRoundNearestInterpolator<MyFloat, short>> MyFdc;
  std::vector<typename MyFdc::descriptor_type> dscL, dscR;
  MyFdc::FdcPtr fdcL = MyFdc::createFdc(dataL);
  MyFdc::FdcPtr fdcR = MyFdc::createFdc(dataR);

  StereoLineFilter<MyFloat, LsdCC<MyFloat>::LineSegmentVector> slf(srcL.rows, srcL.cols / 2);
  PairwiseLineMatcher<MyFloat, typename MyFdc::descriptor_type> pmatcher;
  std::vector<size_t> maskL, maskR;

  std::vector<DescriptorMatch<MyFloat>> bfmatches, bfmatchesRes;

  start = double(getTickCount());
  slf.create(lsdL.lineSegments(), lsdR.lineSegments(), bfmatches, maskL, maskR);
  end = double(getTickCount());
  std::cout << "candidates: " << bfmatches.size()
            << ", time for stereo filter create: " << (end - start) * 1000 / getTickFrequency() << "ms" << std::endl;

  start = double(getTickCount());
  fdcL->createList(lsdL.lineSegments(), maskL, dscL);
  fdcR->createList(lsdR.lineSegments(), maskR, dscR);
  end = double(getTickCount());
  std::cout << "time for creating descriptors: " << (end - start) * 1000 / getTickFrequency() << std::endl;

  std::cout << dscL[1].data << std::endl;
  std::cout << dscL[3].data << std::endl;

  start = double(getTickCount());
  pmatcher.match1D(lsdL.lineSegments(), lsdR.lineSegments(), dscL, dscR, bfmatches, bfmatchesRes);
  end = double(getTickCount());
  std::cout << "matches: " << bfmatchesRes.size()
            << ", time for matching: " << (end - start) * 1000 / getTickFrequency() << std::endl;

  imshow("Detected matches, SLF, PairwiseLineMatcher",
         drawMatches<MyFloat, DescriptorMatch<MyFloat>>(srcL, lsdL.lineSegments(), srcR, lsdR.lineSegments(),
                                                        bfmatchesRes));

  cv::Mat modelImage = lsfm::drawLines<MyFloat>(srcL, lsdL.lineSegments());
  cv::imshow("srcL", modelImage);
  modelImage = lsfm::drawLines<MyFloat>(srcR, lsdR.lineSegments());
  cv::imshow("srcR", modelImage);


  /*cv::Mat segL, segR;
  cvtColor(srcL, segL, cv::COLOR_GRAY2BGR);
  cvtColor(srcR, segR, cv::COLOR_GRAY2BGR);


  RNG& rng = theRNG();
  for_each(lsdL.lineSegments().begin(), lsdL.lineSegments().end(), [&](const LsdBase<MyFloat>::LineSegment &l){
      Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
      Scalar scolor(color[0], color[1], color[2]);
      drawLines(srcL, l, scolor);
      Point2f p1 = l.normalLineDist(0, l.centerPoint()), p2 = l.normalLineDist(10, l.centerPoint());
      //drawLines(srcL, p1, p2, Scalar(0, 0, 255));

  });


  imshow("Detected lines L", srcL);

  for_each(lsdR.lineSegments().begin(), lsdR.lineSegments().end(), [&](const LsdBase<MyFloat>::LineSegment &l){
      Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
      Scalar scolor(color[0], color[1], color[2]);
      drawLines(srcR, l, scolor);
      Point2f p1 = l.normalLineDist(0, l.centerPoint()), p2 = l.normalLineDist(10, l.centerPoint());
      //drawLines(srcR, p1, p2, Scalar(0, 0, 255));

  });


  imshow("Detected lines R", srcR);


  for_each(lsdL.segments().begin(), lsdL.segments().end(), [&](const LsdCC<MyFloat>::LineData &seg){
      Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
      for_each(seg.begin(), seg.end(), [&](const Point& p){
          segL.at<Vec3b>(p) = color;
      });
  });

  imshow("Detected Segements L", segL);

  for_each(lsdR.segments().begin(), lsdR.segments().end(), [&](const LsdCC<MyFloat>::LineData &seg){
      Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
      for_each(seg.begin(), seg.end(), [&](const Point& p){
          segR.at<Vec3b>(p) = color;
      });
  });

  imshow("Detected Segements R", segR);*/


  waitKey();

  return 0;
}
