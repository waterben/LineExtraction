#include <iostream>
#include <fstream>
#include <string>

#include <line_tracker/stereo_line_analyzer.hpp>
#include <stereo/stereo_matcher.hpp>
#include <stereo/stereo_line_optimizer.hpp>

#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace lsfm;


int main(int argc, char** argv)
{

    cv::Mat im_Left, im_Right;

    const char* img1_filename = 0;
    const char* img2_filename = 0;

    if (argc == 2){
        return 0;
    } else if(argc > 2){
                img1_filename = argv[1];
                img2_filename = argv[2];
    } else {
        return 0;
    }

    im_Left  = imread(img1_filename, IMREAD_COLOR);  //im_gray(Range(0, im_gray.size().height), Range(0, im_gray.size().width/2));//no data copying here
    im_Right = imread(img2_filename, IMREAD_COLOR);  //im_gray(Range(0, im_gray.size().height), Range(im_gray.size().width/2, im_gray.size().width));//no data copying here

    // Match Lines
    cv::Mat imLeftMod; cv::Mat imRightMod;
    std::vector<StereoCorrelation> currentStereoCorrelations = getMatchingLines(im_Left, im_Right, imLeftMod, imRightMod);

    // Display Matches
    cv::Mat im_grayLeftPainted, im_grayRightPainted;
    im_Left.copyTo(im_grayLeftPainted);
    im_Right.copyTo(im_grayRightPainted);

    cv::Mat unDistImGray(cv::Mat::zeros(Size(im_Left.size().width*2,im_Left.size().height), CV_8UC3 ));
    cv::Mat unDistImGrayLeft(unDistImGray, Rect(0, 0, im_grayLeftPainted.size().width, im_grayLeftPainted.size().height));
    cv::Mat unDistImGrayRight(unDistImGray, Rect(im_grayLeftPainted.size().width, 0, im_grayRightPainted.size().width, im_grayRightPainted.size().height));
    im_grayLeftPainted.copyTo(unDistImGrayLeft);
    im_grayRightPainted.copyTo(unDistImGrayRight);

    // show big image
    //drawStereoLines(unDistImGray, false, 0, currentStereoCorrelations);
    //imshow("GreyImage", unDistImGray);

    // correct Matches
    for(int i = 0; i < currentStereoCorrelations.size(); i++){
        AnalyzedLine* leftLine = currentStereoCorrelations[i].leftLine;
        AnalyzedLine* rightLine = currentStereoCorrelations[i].rightLine;

        double shiftVal = 0, shearVal = 0;
        double tmpVal;
        optimizeStereopairLinearCV<float,double>(imLeftMod, imRightMod, leftLine->line, rightLine->line, shiftVal, shearVal, tmpVal);
        std::cout << "line: " << i << " shiftVal: " << shiftVal << " shearVal: " << shearVal << std::endl;

        /* // Triangulation example
        lsfm::Point3d startPoint3d, endPoint3d;
        double commonStartY, commonEndY;
        triangulateLine(this->P1, this->P2, leftLine.line, rightLine.line, startPoint3d, endPoint3d, commonStartY, commonEndY);
        std::cout << "plot3D([" << startPoint3d.x << "," << endPoint3d.x << "],[" << startPoint3d.y << "," << endPoint3d.y << "],[" << startPoint3d.z << "," << endPoint3d.z << "])" << std::endl;
        */

    }

    std::cout << "End." << std::endl;

    return 0;
}

