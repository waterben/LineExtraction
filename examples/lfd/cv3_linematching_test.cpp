
#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION > 2

#  include "opencv2/core/utility.hpp"
#  include <geometry/draw.hpp>
#  include <lfd/FeatureDescriptorLBD.hpp>
#  include <lfd/LRDescriptor.hpp>
#  include <lfd/PairwiseLineMatcher.hpp>
#  include <lfd/StereoLineFilter.hpp>
#  include <lsd/lsd_cc.hpp>
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/line_descriptor.hpp>
#  include <opencv2/line_descriptor/descriptor.hpp>
#  include <opencv2/opencv.hpp>
#  include <utility/test_images.hpp>

#  include <chrono>
#  include <ctime>
#  include <fstream>
#  include <iostream>
#  include <stdio.h>
#  include <string>
// #include "opencv2/core/private.hpp"
#  include <lfd/lineMatching.hpp>  // drawKeyLineMatches
#  include <opencv2/features2d.hpp>
#  include <opencv2/highgui.hpp>
#  include <opencv2/imgproc.hpp>

#  define MATCHES_DIST_THRESHOLD 25

using namespace std;
using namespace lsfm;
using namespace cv;
using namespace cv::line_descriptor;


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

  // typedef double MyFloat;  // Unused - was used in commented-out LsdCC code below
  /*
      LsdCC<MyFloat> lsdL(0.004, 0.008, 20, 0, 3);
      LsdCC<MyFloat> lsdR(0.004, 0.008, 20, 0, 3);

      double start = double(getTickCount());
      lsdL.detect(srcL);
      lsdR.detect(srcR);
      double end = double(getTickCount());
      std::cout << "lines: " << lsdL.lineSegments().size() + lsdR.lineSegments().size() << ", time for detecting lines:
     " << (end - start) * 1000 / getTickFrequency() << "ms" << std::endl;

      MatMap dataL, dataR;
      dataL["gx"] = lsdL.imageData()[0];
      dataL["gy"] = lsdL.imageData()[1];
      dataL["img"] = srcL;

      dataR["gx"] = lsdR.imageData()[0];
      dataR["gy"] = lsdR.imageData()[1];
      dataR["img"] = srcR;
  */

  /* create an LSD detector */
  Ptr<LSDDetector> lsd = LSDDetector::createLSDDetector();

  /* create binary masks */
  // cv::Mat mask1 = Mat::ones( srcL.size(), CV_8UC1 );
  // cv::Mat mask2 = Mat::ones( srcR.size(), CV_8UC1 );

  /* create a pointer to a BinaryDescriptor object with default parameters */
  Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();

  /* detect lines */
  std::vector<KeyLine> klsd1, klsd2;
  Mat lsd_descr1, lsd_descr2;

  std::chrono::steady_clock::time_point detect1 = std::chrono::steady_clock::now();
  lsd->detect(srcL, klsd1, 2, 2);
  std::chrono::steady_clock::time_point detect2 = std::chrono::steady_clock::now();
  std::cout << "Processing Time lineDetection on first image in s: "
            << std::chrono::duration_cast<std::chrono::duration<double>>(detect2 - detect1).count() << std::endl;
  lsd->detect(srcR, klsd2, 2, 2);

  /* compute descriptors for lines from first octave */
  bd->compute(srcL, klsd1, lsd_descr1);
  bd->compute(srcR, klsd2, lsd_descr2);

  /* select lines and descriptors from first octave */
  std::vector<KeyLine> octave0_1, octave0_2;
  Mat leftDEscr, rightDescr;
  for (int i = 0; i < (int)klsd1.size(); i++) {
    if (klsd1[i].octave == 0) {
      octave0_1.push_back(klsd1[i]);
      leftDEscr.push_back(lsd_descr1.row(i));
    }
  }

  for (int j = 0; j < (int)klsd2.size(); j++) {
    if (klsd2[j].octave == 0) {
      octave0_2.push_back(klsd2[j]);
      rightDescr.push_back(lsd_descr2.row(j));
    }
  }

  /* Create Mask */
  StereoLineFilter<float, lsfm::LsdCC<float>::LineSegmentVector> slf(srcL.cols);

  std::vector<lsfm::LineSegment<float>> leftLines, rightLines;

  leftLines = lsfm::keyLine2LineSegment<float>(octave0_1);
  rightLines = lsfm::keyLine2LineSegment<float>(octave0_2);

  /* create a BinaryDescriptorMatcher object */
  Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

  /* compute matches */
  std::vector<DMatch> lsd_matches;

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  bdm->match(leftDEscr, rightDescr, lsd_matches);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::cout << "Processing Time matching in s: "
            << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() << std::endl;
  std::cout << "matches: " << lsd_matches.size() << std::endl;

  /* select best matches */
  [[maybe_unused]] std::vector<DMatch> removed_maches;
  [[maybe_unused]] std::vector<DMatch> filtered_matches2 =
      postfilterStereoMatches(lsd_matches, slf, leftLines, rightLines, removed_maches);
  /*
     std::vector<DMatch> good_matches;
     good_matches.clear();
     for ( int i = 0; i < (int) lsd_matches.size(); i++ )
     {
         if( lsd_matches[i].distance < MATCHES_DIST_THRESHOLD )
             good_matches.push_back( lsd_matches[i] );
     }
 */

  cv::Mat lImage = srcL.clone();
  lImage = lsfm::drawLines<FT>(srcL, leftLines);  //, visibleLineModel3dIDs, cv::Scalar(0, 0, 255), 2);
  cv::imshow("camLeft", lImage);

  cv::Mat rImage = srcR.clone();
  rImage = lsfm::drawLines<FT>(srcR, rightLines);  //, visibleLineModel3dIDs, cv::Scalar(0, 0, 255), 2);
  cv::imshow("camRight", rImage);
  /*
      imshow("Detected matches, cv3 lbd matching", drawKeyLineMatches<MyFloat,DescriptorMatch<MyFloat>>(srcL, octave0_1,
     srcR, octave0_2, lsd_matches)); imshow("Detected matches filtered2, cv3 lbd matching",
     drawKeyLineMatches<MyFloat,DescriptorMatch<MyFloat>>(srcL, octave0_1, srcR, octave0_2, filtered_matches2));
      imshow("Detected matches removed_maches, cv3 lbd matching",
     drawKeyLineMatches<MyFloat,DescriptorMatch<MyFloat>>(srcL, octave0_1, srcR, octave0_2, removed_maches));
  */

  /* draw lines extracted from octave 0 */
  /*    cv::Mat output = srcL.clone();
      if( output.channels() == 1 )
            cvtColor( output, output, COLOR_GRAY2BGR );
      for ( size_t i = 0; i < octave0_1.size(); i++ )
      {
            KeyLine kl = octave0_1[i];
            {
              /* get a random color */
  /*            int R = ( rand() % (int) ( 255 + 1 ) );
              int G = ( rand() % (int) ( 255 + 1 ) );
              int B = ( rand() % (int) ( 255 + 1 ) );

              /* get extremes of line */
  /*            Point pt1 = Point( kl.startPointX, kl.startPointY );
              Point pt2 = Point( kl.endPointX, kl.endPointY );

              /* draw line */
  /*            line( output, pt1, pt2, Scalar( B, G, R ), 5 );
            }

      }

      /* show lines on image */
  /*    imshow( "Lines left", output );
   */
  /* plot matches */
  /*    cv::Mat lsd_outImg;
      //resize( srcL, srcL, Size( srcL.cols / 2, srcL.rows / 2 ) );
      //resize( srcR, srcR, Size( srcR.cols / 2, srcR.rows / 2 ) );
      std::vector<char> lsd_mask( lsd_matches.size(), 1 );
      drawLineMatches( srcL, octave0_1, srcR, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ), Scalar::all( -1 ),
     lsd_mask, DrawLinesMatchesFlags::DEFAULT );

      imshow( "LSD matches", lsd_outImg );
  */
  waitKey();

  return 0;
}

#endif
