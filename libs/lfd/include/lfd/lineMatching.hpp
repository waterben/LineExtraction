#pragma once


#include <opencv2/highgui/highgui.hpp>

#include <iostream>
// #include <lsd/lsd_el.hpp>
#include <geometry/base.hpp>
#include <geometry/draw.hpp>
#include <lsd/lsd_cc.hpp>
// #include "../slam/slamDataModel.hpp"
// #include "../slam/pose_estimator.hpp"

#include <opencv2/line_descriptor/descriptor.hpp>

#define MATCHES_DIST_THRESHOLD_LM 75  // 25

typedef double FT;

using namespace cv;
using namespace cv::line_descriptor;

namespace lsfm {

template <class FT>
std::vector<KeyLine> lineSegment2KeyLine(const std::vector<LineSegment<FT>>& lines0, cv::Mat cam0grey, int octave = 0) {
  std::vector<KeyLine> klV0;
  for (int i = 0; i < lines0.size(); ++i) {
    KeyLine kl = lineSegment2KeyLine(lines0[i], cam0grey, i, octave);
    klV0.push_back(kl);
  }
  return klV0;
}

template <class FT>
KeyLine lineSegment2KeyLine(const LineSegment<FT>& ls, cv::Mat img, int classId = 0, int octave = 0) {
  int imgMaxSize = std::max(img.size().width, img.size().height);
  return lineSegment2KeyLine(ls, imgMaxSize, classId, octave);
}

/** Converts a LineSegment2 into a KeyLine, be careful the sign of the angle is sometimes wrong */
template <class FT>
KeyLine lineSegment2KeyLine(const LineSegment<FT>& ls, const int& imgMaxSize, int classId = 0, int octave = 0) {
  KeyLine kl;

  kl.startPointX = static_cast<float>(ls.startPoint().x());
  kl.startPointY = static_cast<float>(ls.startPoint().y());
  kl.endPointX = static_cast<float>(ls.endPoint().x());
  kl.endPointY = static_cast<float>(ls.endPoint().y());
  kl.sPointInOctaveX = static_cast<float>(ls.startPoint().x());
  kl.sPointInOctaveY = static_cast<float>(ls.startPoint().y());
  kl.ePointInOctaveX = static_cast<float>(ls.endPoint().x());
  kl.ePointInOctaveY = static_cast<float>(ls.endPoint().y());

  kl.angle = static_cast<float>(ls.angle());
  kl.octave = octave;
  kl.class_id = classId;
  kl.lineLength = static_cast<float>(ls.length());
  kl.pt = cv::Point2f(static_cast<float>(ls.centerPoint().x()), static_cast<float>(ls.centerPoint().y()));
  kl.response = static_cast<float>(ls.length() / imgMaxSize);
  kl.size = static_cast<float>(std::abs(ls.startPoint().x() - ls.endPoint().x()) *
                               std::abs(ls.startPoint().y() - ls.endPoint().y()));

  kl.numOfPixels = std::ceil(ls.length());

  return kl;
}

template <class FType>
LineSegment<FType> keyLine2LineSegment(const KeyLine& kl) {
  return LineSegment<FType>(kl.getStartPoint(), kl.getEndPoint(), kl.octave);
}


template <class FType>
std::vector<LineSegment<FType>> keyLine2LineSegment(const std::vector<KeyLine>& klV) {
  std::vector<LineSegment<FType>> lsV;
  for (uint i = 0; i < klV.size(); ++i) {
    lsV.push_back(keyLine2LineSegment<FType>(klV[i]));
  }
  return lsV;
}

//! Only keep Matches that are in the matches and in the filter vector
template <class FeatureTypeA, class FeatureTypeB>
std::vector<FeatureTypeA> postfilterStereoMatches(std::vector<FeatureTypeA> matches,
                                                  std::vector<FeatureTypeB> filter,
                                                  const int size1,
                                                  const int size2) {
  std::vector<std::vector<int>> map;
  std::vector<FeatureTypeA> result;
  map.assign(size1, std::vector<int>(size2, 0));

  for (int i = 0; i < filter.size(); ++i) {
    map.at(filter.at(i).queryIdx).at(filter.at(i).matchIdx) = 1;
  }
  for (int i = 0; i < matches.size(); ++i) {
    if (map.at(matches.at(i).queryIdx).at(matches.at(i).trainIdx) == 1) {
      result.push_back(matches.at(i));
    }
  }
  return result;
}

//! Filter mached lines by geometry checks
template <class FeatureTypeA, class GeoFilter, class GV>
std::vector<FeatureTypeA> postfilterStereoMatches(std::vector<FeatureTypeA> matches, GeoFilter filter, GV l1, GV l2) {
  std::vector<FeatureTypeA> result;

  for (int i = 0; i < matches.size(); ++i) {
    if (!filter.filter(l1[matches[i].queryIdx], l2[matches[i].trainIdx])) {
      result.push_back(matches[i]);
    }
  }
  return result;
}


//! Delete me (because I show the filtered results, which is only for debugging.)
template <class FeatureTypeA, class GeoFilter, class GV>
std::vector<FeatureTypeA> postfilterStereoMatches(
    std::vector<FeatureTypeA> matches, GeoFilter filter, GV l1, GV l2, std::vector<FeatureTypeA>& filtered) {
  std::vector<FeatureTypeA> result;

  for (uint i = 0; i < matches.size(); ++i) {
    // std::cout << "id: " << i;
    if (!filter.filter(l1[matches[i].queryIdx], l2[matches[i].trainIdx])) {
      result.push_back(matches[i]);
    } else {
      filtered.push_back(matches[i]);
    }
    // std::cout << std::endl;
  }
  return result;
}

template <class FT, class DM, class GV>
cv::Mat drawKeyLineMatches(const cv::Mat& img1,
                           const GV& l1,
                           const cv::Mat& img2,
                           const GV& l2,
                           const std::vector<DMatch>& matches,
                           const std::vector<int> lineNumber = std::vector<int>(),
                           bool connect = false) {
  cv::Size size(img1.cols + img2.cols, MAX(img1.rows, img2.rows));
  cv::Mat outImg, outImgL, outImgR;

  outImg.create(size, CV_MAKETYPE(img1.depth(), 3));
  outImg = cv::Scalar::all(0);
  outImgL = outImg(cv::Rect(0, 0, img1.cols, img1.rows));
  outImgR = outImg(cv::Rect(img1.cols, 0, img2.cols, img2.rows));

  if (img1.type() == CV_8U)
    cvtColor(img1, outImgL, CV_GRAY2BGR);
  else
    img1.copyTo(outImgL);

  if (img2.type() == CV_8U)
    cv::cvtColor(img2, outImgR, CV_GRAY2BGR);
  else
    img2.copyTo(outImgR);


  cv::RNG& rng = cv::theRNG();

  bool useNumbers = (lineNumber.size() > 0);
  int idx = 0;
  for_each(matches.begin(), matches.end(), [&](const DMatch& m) {
    cv::Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));

    char buffer[50];
    sprintf(buffer, "%i", useNumbers ? lineNumber.at(m.queryIdx) : idx);

    LineSegment<FT, lsfm::Vec2> line1(
        lsfm::Vec4<FT>(lsfm::getX(l1[m.queryIdx].getStartPoint()), lsfm::getY(l1[m.queryIdx].getStartPoint()),
                       lsfm::getX(l1[m.queryIdx].getEndPoint()), lsfm::getY(l1[m.queryIdx].getEndPoint())));
    LineSegment<FT, lsfm::Vec2> line2(
        lsfm::Vec4<FT>(lsfm::getX(l2[m.trainIdx].getStartPoint()), lsfm::getY(l2[m.trainIdx].getStartPoint()),
                       lsfm::getX(l2[m.trainIdx].getEndPoint()), lsfm::getY(l2[m.trainIdx].getEndPoint())));

    lsfm::drawMatch(outImg, outImgL, outImgR, line1, line2, cv::Scalar(color[0], color[1], color[2]), buffer, connect);
    idx++;
  });

  return outImg;
}


//    void lineMatchingCV(cv::Mat cam0img, cv::Mat cam1img){


//        /* create an LSD detector */
//        Ptr<LSDDetector> lsd = LSDDetector::createLSDDetector();

//        /* create binary masks */
//        cv::Mat mask1 = Mat::ones( cam0img.size(), CV_8UC1 );
//        cv::Mat mask2 = Mat::ones( cam1img.size(), CV_8UC1 );

//        /* create a pointer to a BinaryDescriptor object with default parameters */
//        Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor(  );

//        /* detect lines */
//        std::vector<KeyLine> klsd1, klsd2;
//        Mat lsd_descr1, lsd_descr2;
//        lsd->detect( cam0img, klsd1, 2, 2, mask1 );
//        lsd->detect( cam1img, klsd2, 2, 2, mask2 );

//        /* compute descriptors for lines from first octave */
//        bd->compute( cam0img, klsd1, lsd_descr1 );
//        bd->compute( cam1img, klsd2, lsd_descr2 );

//        /* select lines and descriptors from first octave */
//        std::vector<KeyLine> octave0_1, octave0_2;
//        Mat leftDEscr, rightDescr;
//        for ( int i = 0; i < static_cast<int> klsd1.size(); i++ )
//        {
//            if( klsd1[i].octave == 0 )
//            {
//                octave0_1.push_back( klsd1[i] );
//                leftDEscr.push_back( lsd_descr1.row( i ) );
//            }
//        }

//        for ( int j = 0; j < static_cast<int> klsd2.size(); j++ )
//        {
//            if( klsd2[j].octave == 0 )
//            {
//                octave0_2.push_back( klsd2[j] );
//                rightDescr.push_back( lsd_descr2.row( j ) );
//            }
//        }

//        /* create a BinaryDescriptorMatcher object */
//        Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

//        /* compute matches */
//        std::vector<DMatch> lsd_matches;
//        bdm->match( leftDEscr, rightDescr, lsd_matches );

//        /* select best matches */
//        std::vector<DMatch> good_matches;
//        good_matches.clear();
//        for ( int i = 0; i < static_cast<int> lsd_matches.size(); i++ )
//        {
//            if( lsd_matches[i].distance < MATCHES_DIST_THRESHOLD_LM )
//                good_matches.push_back( lsd_matches[i] );
//        }


//        /* draw lines extracted from octave 0 */
//        cv::Mat output = cam0img.clone();
//        if( output.channels() == 1 )
//              cvtColor( output, output, COLOR_GRAY2BGR );
//        for ( size_t i = 0; i < octave0_1.size(); i++ )
//        {
//              KeyLine kl = octave0_1[i];
//              {
//                /* get a random color */
//                int R = ( rand() % static_cast<int> ( 255 + 1 ) );
//                int G = ( rand() % static_cast<int> ( 255 + 1 ) );
//                int B = ( rand() % static_cast<int> ( 255 + 1 ) );

//                /* get extremes of line */
//                Point pt1 = Point( kl.startPointX, kl.startPointY );
//                Point pt2 = Point( kl.endPointX, kl.endPointY );

//                /* draw line */
//                line( output, pt1, pt2, Scalar( B, G, R ), 2 );
//              }

//        }


//        imshow("Detected matches", drawKeyLineMatches<FT,DescriptorMatch<FT>>(cam0img, octave0_1, cam1img, octave0_2,
//        good_matches));


//        /* plot matches */
// /*        cv::Mat lsd_outImg;
// //        resize( cam0img, cam0img, Size( cam0img.cols / 2, cam0img.rows / 2 ) );
// //        resize( cam1img, cam1img, Size( cam1img.cols / 2, cam1img.rows / 2 ) );
//        std::vector<char> lsd_mask( lsd_matches.size(), 1 );
//        drawLineMatches( cam0img, octave0_1, cam1img, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ),
//        Scalar::all( -1 ), lsd_mask,
//                         DrawLinesMatchesFlags::DEFAULT );

//        imshow( "LSD matches", lsd_outImg );
//*/
//        /* show lines on image */
//        imshow( "Lines", output );

//    }

inline std::vector<DMatch> lineMatchingLsdAndCV(cv::Mat lsd_descr0,
                                                cv::Mat lsd_descr1,
                                                cv::Mat mask = cv::Mat::ones(0, 0, CV_8UC1)) {
  if (mask.rows = 0 && mask.cols == 0) mask = cv::Mat::ones(lsd_descr0.rows, lsd_descr1.rows, CV_8UC1);

  /* create a BinaryDescriptorMatcher object */
  Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

  /* compute matches */
  std::vector<DMatch> lsd_matches;
  bdm->match(lsd_descr0, lsd_descr1, lsd_matches, mask);

  /* select best matches */
  std::vector<DMatch> good_matches;
  good_matches.clear();
  for (int i = 0; i < static_cast<int>(lsd_matches.size()); i++) {
    if (lsd_matches[i].distance < MATCHES_DIST_THRESHOLD_LM) good_matches.push_back(lsd_matches[i]);
  }
  //        imshow("Detected matches", drawKeyLineMatches<FT,DescriptorMatch<FT>>(cam0grey, klV0, cam1grey, klV1,
  //        good_matches));
  return good_matches;
}

template <class LSV>
inline std::vector<DMatch> lineMatchingLsdAndCV(
    cv::Mat cam0grey, cv::Mat cam1grey, LSV lines0, LSV lines1, cv::Mat mask = cv::Mat::ones(0, 0, CV_8UC1)) {
  if (mask.rows = 0 && mask.cols == 0) mask = cv::Mat::ones(lines0.size(), lines1.size(), CV_8UC1);

  std::vector<KeyLine> klV0, klV1;
  klV0 = lineSegment2KeyLine(lines0, cam0grey);
  klV1 = lineSegment2KeyLine(lines1, cam1grey);

  /* create a pointer to a BinaryDescriptor object with default parameters */
  Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();

  /* compute descriptors for lines from first octave */
  Mat lsd_descr0, lsd_descr1;
  bd->compute(cam0grey, klV0, lsd_descr0);
  bd->compute(cam1grey, klV1, lsd_descr1);

  return lineMatchingLsdAndCV(lsd_descr0, lsd_descr1);
}

template <class MATCH>
inline std::vector<MATCH> leftRightCheck(std::vector<MATCH> mVec1, std::vector<MATCH> mVec2) {
  std::vector<MATCH> lrMatches;

  for (int i = 0; i < mVec1.size(); ++i) {
    if (std::find_if(mVec2.begin(), mVec2.end(), [&](const DMatch& arg) {
          return arg.trainIdx == mVec1[i].queryIdx && arg.queryIdx == mVec1[i].trainIdx;
        }) != mVec2.end()) {
      lrMatches.push_back(mVec1[i]);
    }
  }
  return lrMatches;
}

inline cv::Mat createLBD(cv::Mat cam0grey,
                         std::vector<KeyLine> klV,
                         Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor()) {
  cv::Mat lsd_descr;
  bd->compute(cam0grey, klV, lsd_descr);
  return lsd_descr;
}
/// Overloading
template <class FT>
inline cv::Mat createLBD(cv::Mat cam0grey,
                         std::vector<LineSegment<FT>> lineVec,
                         Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor()) {
  std::vector<KeyLine> klV = lineSegment2KeyLine(lineVec, cam0grey);
  return createLBD(cam0grey, klV, bd);
}

inline std::vector<DMatch> matchingLRcheck(cv::Mat l_descr0, cv::Mat l_descr1) {
  std::vector<DMatch> matches01 = lineMatchingLsdAndCV(l_descr0, l_descr1);
  std::vector<DMatch> matches10 = lineMatchingLsdAndCV(l_descr1, l_descr0);
  return leftRightCheck(matches01, matches10);
}

template <class LSV>
inline std::vector<DMatch> matchingLRcheck(cv::Mat cam0grey, cv::Mat cam1grey, LSV lines0, LSV lines1) {
  cv::Mat l_descr0, l_descr1;

  /* create a pointer to a BinaryDescriptor object with default parameters */
  Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();

  l_descr0 = createLBD(cam0grey, lines0, bd);
  l_descr1 = createLBD(cam1grey, lines1, bd);
  return matchingLRcheck(l_descr0, l_descr1);
}

}  // namespace lsfm
