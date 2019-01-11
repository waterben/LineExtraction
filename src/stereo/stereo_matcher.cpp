//#include <iostream>
//#include <fstream>
//#include <string>

//#include <lsd/primal_lsd.hpp>
//#include <lsd/cv_lsd.hpp>
#include <lsd/lsd_cc.hpp>
//#include <lsd/fast_lsd.hpp>
//#include <opencv2/core/core.hpp>
#include <line_tracker/line_analyzer.hpp>
#include <line_tracker/stereo_line_analyzer.hpp>

#include <stereo/stereo_matcher.hpp>

//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/calib3d/calib3d.hpp"

#define BINSIZE 12

using namespace std;
using namespace lsfm;

/**
*   Function to find lines and return the stereo matches of a stereo image pair.
*   Takes two gray value images as input.
*/
std::vector<StereoCorrelation> getMatchingLines(cv::Mat im_grayLeft, cv::Mat im_grayRight){

    cv::Mat imLeftMod; cv::Mat imRightMod;
    return getMatchingLines(im_grayLeft, im_grayRight, imLeftMod, imRightMod);
}


/**
*   Function to find lines and return the stereo matches of a stereo image pair.
*   Takes two gray value images as input.
*/
std::vector<StereoCorrelation> getMatchingLines(cv::Mat im_grayLeft, cv::Mat im_grayRight, cv::Mat & imLeftMod, cv::Mat & imRightMod){

    Ptr<LsdBase<double>> lsd_cvLeft(new LsdCC<double>(0, 0.012, 0.004, 20, 0, 1.5));  // 0, 0.012, 0.004, 20, 0, 1.5
    Ptr<LsdBase<double>> lsd_cvRight(new LsdCC<double>(0, 0.012, 0.004, 20, 0, 1.5));

    LineAnalyzer laLeft(BINSIZE);
    LineAnalyzer laRight(BINSIZE);

    StereoLineAnalyzer sla(laLeft, laRight, BINSIZE * 2);

    vector<lsfm::Vec4d> detectedLinesLeftd, detectedLinesRightd;

    double start;

    int cycleNum = 1;
    start = double(getTickCount());

    start = double(getTickCount());

    laLeft.setCurrentFrameNr(cycleNum);
    laRight.setCurrentFrameNr(cycleNum);

    //  cvtColor(frame,im_gray,CV_RGB2GRAY);

    // 0: gx, 1: gy, 2: modgrad, 3: angles
    vector<cv::Mat> imageDataLeft, imageDataRight;

    // get lines
    lsd_cvLeft->detect(im_grayLeft, detectedLinesLeftd, imageDataLeft);
    lsd_cvRight->detect(im_grayRight, detectedLinesRightd, imageDataRight);
    LineSegment2Vector<double> leftLinesVec = lsd_cvLeft->lineSegments();
    LineSegment2Vector<double> rightLinesVec = lsd_cvRight->lineSegments();

    imageDataLeft[2].convertTo(imageDataLeft[2], CV_32FC1);
    imageDataRight[2].convertTo(imageDataRight[2], CV_32FC1);

    // wurzel aus gradienten bild (bei CC_LDE_REGRESSION), könnte noch wegoptimiert werden
    sqrt(imageDataLeft[2]/4, imageDataLeft[2]);
    sqrt(imageDataRight[2]/4, imageDataRight[2]);

    // analyze... get more data out of the lines
    laLeft.analyzeLines(leftLinesVec, im_grayLeft, imageDataLeft[2], (int)sqrt(im_grayLeft.size().height * im_grayLeft.size().height + im_grayLeft.size().width * im_grayLeft.size().width));
    laRight.analyzeLines(rightLinesVec, im_grayRight, imageDataRight[2], (int)sqrt(im_grayRight.size().height * im_grayRight.size().height + im_grayRight.size().width * im_grayRight.size().width));

  //faster way? - make gradient image available outside
 //   imageDataLeft[2].copyTo(imLeftMod);
 //   imageDataRight[2].copyTo(imRightMod);
    imLeftMod = imageDataLeft[2];
    imRightMod = imageDataRight[2];

    cv::Mat im_grayLeftPainted, im_grayRightPainted;
    im_grayLeft.copyTo(im_grayLeftPainted);
    im_grayRight.copyTo(im_grayRightPainted);

    cv::Mat unDistImGray(cv::Mat::zeros(Size(im_grayLeftPainted.size().width*2,im_grayLeftPainted.size().height), CV_8UC3 ));
    cv::Mat unDistImGrayLeft(unDistImGray, Rect(0, 0, im_grayLeftPainted.size().width, im_grayLeftPainted.size().height));
    cv::Mat unDistImGrayRight(unDistImGray, Rect(im_grayLeftPainted.size().width, 0, im_grayRightPainted.size().width, im_grayRightPainted.size().height));
    im_grayLeftPainted.copyTo(unDistImGrayLeft);
    im_grayRightPainted.copyTo(unDistImGrayRight);

    std::vector<AnalyzedLine*> leftAL = laLeft.getAnalyzedLines(), rightAL = laRight.getAnalyzedLines();
    sla.matchLines(im_grayLeft.size(), leftAL, rightAL, false);

    return sla.getCurrentStereoCorrelations();

}


