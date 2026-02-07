/// @file dsc_test.cpp
/// @brief Descriptor creation and distance computation demonstration.
///
/// Demonstrates Line-Region (LR) descriptor creation and matching:
/// - LRDescriptor for combined line and region features
/// - Stereo line filtering to reduce candidate matches
/// - Descriptor distance computation and matching
///
/// @usage ./dsc_test [left_image] [right_image]
/// @param left_image Optional path to left stereo image
/// @param right_image Optional path to right stereo image

#include <geometry/draw.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <lfd/LRDescriptor.hpp>
#include <lfd/StereoLineFilter.hpp>
#include <lfd/StereoLineMatcher.hpp>
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

  cv::Mat srcL = cv::imread(filename1, IMREAD_GRAYSCALE);
  cv::Mat srcR = cv::imread(filename2, IMREAD_GRAYSCALE);

  if (srcL.empty() || srcR.empty()) {
    cout << "Can not open files" << endl;
    return -1;
  }

  resize(srcL, srcL, cv::Size(0, 0), 0.65, 0.65);
  resize(srcR, srcR, cv::Size(0, 0), 0.65, 0.65);

  // GaussianBlur(srcL, srcL, cv::Size(3, 3), 0.3);
  // GaussianBlur(srcR, srcR, cv::Size(3, 3), 0.3);

  typedef double MyFloat;

  LsdCC<MyFloat> lsdL(0.004, 0.008, 20, 0, 3);
  LsdCC<MyFloat> lsdR(0.004, 0.008, 20, 0, 3);

  double start = double(cv::getTickCount());
  lsdL.detect(srcL);
  lsdR.detect(srcR);
  double end = double(cv::getTickCount());
  std::cout << "lines: " << lsdL.lineSegments().size() + lsdR.lineSegments().size()
            << ", time for detecting lines: " << (end - start) * 1000 / cv::getTickFrequency() << "ms" << std::endl;

  MatMap dataL, dataR;
  dataL["gx"] = lsdL.imageData()[0];
  dataL["gy"] = lsdL.imageData()[1];
  dataL["img"] = srcL;

  dataR["gx"] = lsdR.imageData()[0];
  dataR["gy"] = lsdR.imageData()[1];
  dataR["img"] = srcR;

  // typedef GchGradImgMean<MyFloat,1,2,NoAlign<MyFloat>,
  //         FastMean<MyFloat,short,FastRoundNearestInterpolator<MyFloat, short> >,
  //         FastMean<MyFloat,uchar,FastRoundNearestInterpolator<MyFloat, uchar> > > MyGchHelper;

  // typedef GchGradMean<MyFloat,1,2,NoAlign<MyFloat>,
  //         FastMean<MyFloat,short,FastRoundNearestInterpolator<MyFloat, short> > > MyGchHelper;

  // typedef GchImgMean<MyFloat,1,2,FastMean<MyFloat,uchar,FastRoundNearestInterpolator<MyFloat, uchar> > > MyGchHelper;

  /*typedef GchGradImgInterpolate<MyFloat,1,2,NoAlign<MyFloat>,
          LinearInterpolator<MyFloat, short>,
          LinearInterpolator<MyFloat, uchar> > MyGchHelper;*/

  typedef GchGradImgInterpolate<MyFloat, 1, 2, NoAlign<MyFloat>, FastRoundNearestInterpolator<MyFloat, short>,
                                FastRoundNearestInterpolator<MyFloat, uchar>>
      MyGchHelper;

  // typedef FdcGenericLR<MyFloat,LsdCC<MyFloat>::LineSegment,GMyGchHelper> MyFdc;
  typedef FdcGenericLR<MyFloat, LsdCC<MyFloat>::LineSegment, MyGchHelper> MyFdc;
  std::vector<typename MyFdc::descriptor_type> dscL, dscR;
  MyFdc::FdcPtr fdcL = MyFdc::createFdc(dataL);
  MyFdc::FdcPtr fdcR = MyFdc::createFdc(dataR);
  StereoLineFilter<MyFloat, LsdCC<MyFloat>::LineSegmentVector> slf(srcL.rows, srcL.cols / 2);
  StereoLineMatcher<MyFloat, LsdCC<MyFloat>::LineSegmentVector, MyGchHelper> slm(fdcL, fdcR, srcL.rows, srcL.cols / 2,
                                                                                 5, 0.6);
  std::vector<size_t> maskL, maskR, maskL2, maskR2;


  std::vector<DescriptorMatch<MyFloat>> bfmatches, bfmatches2, bfmatches3, bfmatchesT, bfmatchesT2;
  FmBruteForce<MyFloat, typename MyFdc::descriptor_type> bfmatcher;

  start = double(cv::getTickCount());
  slf.train(lsdL.lineSegments(), lsdR.lineSegments());
  // slf.create(lsdL.lineSegments().size(), lsdR.lineSegments().size(), bfmatchesT);
  slf.create(lsdL.lineSegments().size(), lsdR.lineSegments().size(), bfmatchesT, maskL, maskR);
  end = double(cv::getTickCount());
  std::cout << "time for stereo filter: " << (end - start) * 1000 / cv::getTickFrequency()
            << "ms, candidates: " << bfmatchesT.size() << std::endl;

  start = double(cv::getTickCount());
  slf.create(lsdL.lineSegments(), lsdR.lineSegments(), bfmatchesT2, maskL2, maskR2);
  end = double(cv::getTickCount());
  std::cout << "time for stereo filter create: " << (end - start) * 1000 / cv::getTickFrequency()
            << "ms, candidates: " << bfmatchesT2.size() << std::endl;

  start = double(cv::getTickCount());
  // fdcL->createList(lsdL.lineSegments(), dscL);
  // fdcR->createList(lsdR.lineSegments(), dscR);
  fdcL->createList(lsdL.lineSegments(), maskL2, dscL);
  fdcR->createList(lsdR.lineSegments(), maskR2, dscR);
  end = double(cv::getTickCount());
  std::cout << "time for creating descriptors: " << (end - start) * 1000 / cv::getTickFrequency() << std::endl;


  start = double(cv::getTickCount());
  // bfmatcher.match(dscL, dscR, slf, bfmatches);
  // bfmatcher.match(dscL, dscR, bfmatches);

  // bfmatcher.train(dscL, dscR, slf);
  bfmatcher.trainMatches(dscL, dscR, bfmatchesT2);
  bfmatcher.best(bfmatches);
  end = double(cv::getTickCount());
  std::cout << "matches: " << bfmatches.size()
            << ", time for matching: " << (end - start) * 1000 / cv::getTickFrequency() << std::endl;

  start = double(cv::getTickCount());
  slm.match(lsdL.lineSegments(), lsdR.lineSegments(), bfmatches3);
  end = double(cv::getTickCount());
  std::cout << "slm matches: " << bfmatches3.size()
            << ", time for matching: " << (end - start) * 1000 / cv::getTickFrequency() << std::endl;


  // imshow("Detected Lines Img1", drawLines<MyFloat>(srcL, lsdL.lineSegments()));
  // imshow("Detected Lines Img2", drawLines<MyFloat>(srcR, lsdR.lineSegments()));

  MyFloat mean = 0;
  for_each(bfmatches3.begin(), bfmatches3.end(), [&](const DescriptorMatch<MyFloat>& data) { mean += data.distance; });

  mean /= static_cast<MyFloat>(bfmatches3.size());
  std::cout << "mean distance: " << mean << std::endl;
  for_each(bfmatches3.begin(), bfmatches3.end(), [&](const DescriptorMatch<MyFloat>& data) {
    // mean += data.distance;
    // if (data.distance < mean)
    // if (data.distance < 500)
    bfmatches2.push_back(data);
  });

  std::cout << "final matches: " << bfmatches2.size() << std::endl;
  imshow("Detected matches", drawMatches<MyFloat, DescriptorMatch<MyFloat>>(srcL, lsdL.lineSegments(), srcR,
                                                                            lsdR.lineSegments(), bfmatches2));

  /*cv::Mat segL, segR;
  cvtColor(srcL, segL, cv::COLOR_GRAY2BGR);
  cvtColor(srcR, segR, cv::COLOR_GRAY2BGR);


  RNG& rng = theRNG();
  for_each(lsdL.lineSegments().begin(), lsdL.lineSegments().end(), [&](const LsdBase<MyFloat>::LineSegment &l){
      Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
      cv::Scalar scolor(color[0], color[1], color[2]);
      drawLines(srcL, l, scolor);
      Point2f p1 = l.normalLineDist(0, l.centerPoint()), p2 = l.normalLineDist(10, l.centerPoint());
      //drawLines(srcL, p1, p2, cv::Scalar(0, 0, 255));

  });


  imshow("Detected lines L", srcL);

  for_each(lsdR.lineSegments().begin(), lsdR.lineSegments().end(), [&](const LsdBase<MyFloat>::LineSegment &l){
      Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
      cv::Scalar scolor(color[0], color[1], color[2]);
      drawLines(srcR, l, scolor);
      Point2f p1 = l.normalLineDist(0, l.centerPoint()), p2 = l.normalLineDist(10, l.centerPoint());
      //drawLines(srcR, p1, p2, cv::Scalar(0, 0, 255));

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


  cv::waitKey();

  return 0;
}
