#include <iostream>
#include <fstream>
#include <string>

#include <lsd/lsd_cc.hpp>
#include <opencv2/core/core.hpp>
#include <line_tracker/line_analyzer.hpp>
#include <line_tracker/stereo_line_analyzer.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv2/calib3d/calib3d.hpp"

#define BINSIZE 12
#define MEASURE_TIME

using namespace std;
using namespace lsfm;
// using namespace lsdwrap;

int main(int argc, char** argv)
{

    cv::Mat im_grayLeft, im_grayRight,im_gray;

    VideoCapture capture;
    const char* img1_filename = 0;
    const char* img2_filename = 0;

    if (argc == 2){
        std::string in = argv[1];
        capture = VideoCapture(in);
    } else if(argc > 2){
                img1_filename = argv[1];
                img2_filename = argv[2];
    } else {
    }

    Ptr<LsdBase<double>> lsd_cvLeft = new LsdCC<double>(0, 0.012, 0.004, 20, 0, 1.5);
    Ptr<LsdBase<double>> lsd_cvRight = new LsdCC<double>(0, 0.012, 0.004, 20, 0, 1.5);

    vector<lsfm::Vec4i> detectedLinesLeft, detectedLinesRight;
    vector<lsfm::Vec4d> detectedLinesLeftd, detectedLinesRightd;
    LineAnalyzer laLeft(BINSIZE);
    LineAnalyzer laRight(BINSIZE);
    StereoLineAnalyzer sla(laLeft, laRight, BINSIZE * 2);

    double start, duration_ms;

    int last_key_press = 0;
    int cycleNum = 1;
    double totalDuration = 0;
    start = double(getTickCount());
    double overallAverageStableCorrespondences = 0.0;
    bool drawLines = true;

//    while (last_key_press != 'q' && cycleNum < 10000)
//    {
        start = double(getTickCount());

        laLeft.setCurrentFrameNr(cycleNum);
        laRight.setCurrentFrameNr(cycleNum);


        double captureTime1 = double(getTickCount());

/*
#ifdef MEASURE_TIME
        double captureTime2 = (double(getTickCount()) - captureTime1) * 1000 / getTickFrequency();
#endif
*/
        double convertTime1 = double(getTickCount());
      //  cvtColor(frame,im_gray,CV_RGB2GRAY);
/*
#ifdef MEASURE_TIME
        double convertTime2 = (double(getTickCount()) - convertTime1) * 1000 / getTickFrequency();
#endif
*/
/*
//        resize(im_gray, im_gray, Size(160,120));
//        resize(im_gray, im_gray, Size(320,240));
*/
        im_grayLeft  = imread(img1_filename, IMREAD_COLOR);  //im_gray(Range(0, im_gray.size().height), Range(0, im_gray.size().width/2));//no data copying here
        im_grayRight = imread(img2_filename, IMREAD_COLOR);  //im_gray(Range(0, im_gray.size().height), Range(im_gray.size().width/2, im_gray.size().width));//no data copying here

        double scaleFactor = 1.0;
        double resizeTime1 = double(getTickCount());
//        resize(im_gray, im_gray, Size(im_gray.size().width * scaleFactor, im_gray.size().height * scaleFactor));
        resize(im_grayLeft, im_grayLeft, Size(im_grayLeft.size().width * scaleFactor, im_grayLeft.size().height * scaleFactor));
        resize(im_grayRight, im_grayRight, Size(im_grayRight.size().width * scaleFactor, im_grayRight.size().height * scaleFactor));
/*
#ifdef MEASURE_TIME
        double resizeTime2 = (double(getTickCount()) - resizeTime1) * 1000 / getTickFrequency();
#endif
*/
        // 0: gx, 1: gy, 2: modgrad, 3: angles
        vector<cv::Mat> imageDataLeft, imageDataRight;

        double detectTime1 = double(getTickCount());
        lsd_cvLeft->detect(im_grayLeft, detectedLinesLeftd, imageDataLeft);

        LineSegment2Vector<double> leftLinesVec = lsd_cvLeft->lineSegments();

        lsd_cvRight->detect(im_grayRight, detectedLinesRightd, imageDataRight);

        LineSegment2Vector<double> rightLinesVec = lsd_cvRight->lineSegments();

        LineSegment<double> testLine;
        if(leftLinesVec.size() > 1)
            testLine = leftLinesVec.at(0);

        std::cout << "testVec1 Size: " << leftLinesVec.size() << "  testVec2 Size: " << leftLinesVec.size() << std::endl;
        std::cout << " x: " << testLine.x(10) << "  Y: " << testLine.endPoint() << std::endl;


        imageDataLeft[2].convertTo(imageDataLeft[2], CV_32FC1);
        imageDataRight[2].convertTo(imageDataRight[2], CV_32FC1);

        // wurzel aus gradienten bild (bei CC_LDE_REGRESSION), könnte noch wegoptimiert werden
        sqrt(imageDataLeft[2]/4, imageDataLeft[2]);
        sqrt(imageDataRight[2]/4, imageDataRight[2]);


        //convert
        detectedLinesLeft.clear();
        for(int i = 0; i < detectedLinesLeftd.size(); i++){
            lsfm::Vec4i vec;
            vec[0] = (int)detectedLinesLeftd[i][0];
            vec[1] = (int)detectedLinesLeftd[i][1];
            vec[2] = (int)detectedLinesLeftd[i][2];
            vec[3] = (int)detectedLinesLeftd[i][3];
            detectedLinesLeft.push_back(vec);
        }
        detectedLinesRight.clear();
        for(int i = 0; i < detectedLinesRightd.size(); i++){
            lsfm::Vec4i vec;
            vec[0] = (int)detectedLinesRightd[i][0];
            vec[1] = (int)detectedLinesRightd[i][1];
            vec[2] = (int)detectedLinesRightd[i][2];
            vec[3] = (int)detectedLinesRightd[i][3];
            detectedLinesRight.push_back(vec);
        }

/*
#ifdef MEASURE_TIME
        double detectTime2 = (double(getTickCount()) - detectTime1) * 1000 / getTickFrequency();
#endif
*/
        cv::Mat lineImage(cv::Mat::zeros(im_grayLeft.size(), CV_8UC3 ));
        cv::Mat lineImageMixedColors(cv::Mat::zeros(im_grayLeft.size(), CV_8UC3 ));
        cv::Mat lineImageCorrespondences(cv::Mat::zeros(im_grayLeft.size(), CV_8UC3 ));

        laLeft.drawSegmentsMixedColors(lineImage, detectedLinesLeft);

        double analyzeTime1 = double(getTickCount());

        // analyze... get more data out of the lines
        laLeft.analyzeLines(leftLinesVec, im_grayLeft, imageDataLeft[2], (int)sqrt(im_grayLeft.size().height * im_grayLeft.size().height + im_grayLeft.size().width * im_grayLeft.size().width));
        laRight.analyzeLines(rightLinesVec, im_grayRight, imageDataRight[2], (int)sqrt(im_grayRight.size().height * im_grayRight.size().height + im_grayRight.size().width * im_grayRight.size().width));
/*
#ifdef MEASURE_TIME
        double analyzeTime2 = (double(getTickCount()) - analyzeTime1) * 1000 / getTickFrequency();
#endif
*/
        double drawingTime1 = double(getTickCount());

        cv::Mat im_grayLeftPainted, im_grayRightPainted;
        im_grayLeft.copyTo(im_grayLeftPainted);
        im_grayRight.copyTo(im_grayRightPainted);

        laLeft.drawAnalyzedLinesMixedColors(lineImageMixedColors, true);
        if(drawLines){
            laLeft.drawAnalyzedLinesMixedColors(im_grayLeftPainted, true);
            laRight.drawAnalyzedLinesMixedColors(im_grayRightPainted, true);
        }
#ifdef MEASURE_TIME
        double drawingTime2 = (double(getTickCount()) - drawingTime1) * 1000 / getTickFrequency();
#endif

        double trackingTime1 = double(getTickCount());
        laLeft.findCorrespondingLines();
        laRight.findCorrespondingLines();

#ifdef MEASURE_TIME
        double trackingTime2 = (double(getTickCount()) - trackingTime1) * 1000 / getTickFrequency();
#endif

        //std::cout << "corr Anzahl: " << la->getNextCorrespondenceNr() << std::endl;
        drawingTime1 = double(getTickCount());
        cv::Mat unDistImGray(cv::Mat::zeros(Size(im_grayLeftPainted.size().width*2,im_grayLeftPainted.size().height), CV_8UC3 ));
        cv::Mat unDistImGrayLeft(unDistImGray, Rect(0, 0, im_grayLeftPainted.size().width, im_grayLeftPainted.size().height));
        cv::Mat unDistImGrayRight(unDistImGray, Rect(im_grayLeftPainted.size().width, 0, im_grayRightPainted.size().width, im_grayRightPainted.size().height));
        im_grayLeftPainted.copyTo(unDistImGrayLeft);
        im_grayRightPainted.copyTo(unDistImGrayRight);
#ifdef MEASURE_TIME
        drawingTime2 += (double(getTickCount()) - drawingTime1) * 1000 / getTickFrequency();
#endif

        double stereoTime1 = double(getTickCount());
        std::vector<AnalyzedLine*> leftAL = laLeft.getAnalyzedLines(), rightAL = laRight.getAnalyzedLines();
        sla.matchLines(im_grayLeft.size(), leftAL, rightAL, false);
       sla.correctMatchedLines(imageDataLeft[2], imageDataRight[2], unDistImGray, im_grayLeft, im_grayRight);

#ifdef MEASURE_TIME
        double stereoTime2 = (double(getTickCount()) - stereoTime1) * 1000 / getTickFrequency();
#endif

   //     unDistImGray = cv::Mat::zeros(unDistImGray.size(), CV_8UC3 );
        drawingTime1 = double(getTickCount());
        drawStereoLines(unDistImGray, false, 10, sla.getCurrentStereoCorrelations());

        //la->drawCorrespondences(correspondingLines,lineImageCorrespondences);
        laLeft.drawCorrespondingLines(lineImageCorrespondences);

/*
        double scale = 0.75;
        resize(unDistImGray, unDistImGray, Size(unDistImGray.size().width * scale, unDistImGray.size().height * scale));
        resize(lineImageCorrespondences, lineImageCorrespondences, Size(lineImageCorrespondences.size().width * scale, lineImageCorrespondences.size().height * scale));
        resize(im_grayLeftPainted, im_grayLeftPainted, Size(im_grayLeftPainted.size().width * scale, im_grayLeftPainted.size().height * scale));
        resize(im_grayRightPainted, im_grayRightPainted, Size(im_grayRightPainted.size().width * scale, im_grayRightPainted.size().height * scale));
*/
        imshow("GreyImage", unDistImGray);
//        imshow("MixedColors", lineImageMixedColors);
        imshow("Tracking left", lineImageCorrespondences);
        imshow("left", im_grayLeftPainted);
        imshow("right", im_grayRightPainted);

//        imshow("modgrad", imageDataLeft[3]!= -1024.0f);

        if(cycleNum == 1){
            moveWindow("GreyImage", 50, im_gray.size().height + 70);
            //moveWindow("GreyImage", im_gray.size().width + 150, im_gray.size().height + 80);
            moveWindow("left", 100, im_gray.size().height + 80);
            moveWindow("right", im_grayRight.size().width + 150, im_gray.size().height + 80);
            moveWindow("Correspondences", 100, 35);
        }

//  lsfm::waitKey(0);

        last_key_press = cv::waitKey(1)%256;  // wegen bug %256..
        if (last_key_press == 'p'){         // pause function
            last_key_press = 0;
            while (last_key_press != 'p')
                last_key_press = cv::waitKey(50)%256; // wegen bug %256..
        } else if(last_key_press == 'l'){
            drawLines = !drawLines;
        }
/*
#ifdef MEASURE_TIME
            drawingTime2 += (double(getTickCount()) - drawingTime1) * 1000 / getTickFrequency();
#endif
*/
/*
        double percentageStableCorrespondences = (((double)laLeft.getStableCorrespondencesNum()) / ((double)detectedLinesLeft.size())) * 100.0;
        std::cout << "StableCorrespondences: " << laLeft.getStableCorrespondencesNum() << ", percentage of detected lines: " << percentageStableCorrespondences << "%" << std::endl;
        overallAverageStableCorrespondences = (cycleNum * overallAverageStableCorrespondences + percentageStableCorrespondences) / (cycleNum + 1);
        //std::cout << "overallAverageStableCorrespondences: " << overallAverageStableCorrespondences << "%" << std::endl;
*/
/*
#ifdef MEASURE_TIME
        std::cout << "captureTime: " << captureTime2 << std::endl;
        std::cout << "convertTime: " << convertTime2 << std::endl;
        std::cout << "resizeTime: " << resizeTime2 << std::endl;
        std::cout << "lsd detectTime: " << detectTime2 << std::endl;
        std::cout << "analyzeTime: " << analyzeTime2 << std::endl;
        std::cout << "trackingTime: " << trackingTime2 << std::endl;
        std::cout << "stereoTime: " << stereoTime2 << std::endl;
        std::cout << "drawingTime: " << drawingTime2 << std::endl;

        std::cout << "sum: " << captureTime2 + convertTime2 + resizeTime2 + detectTime2 + analyzeTime2 + trackingTime2 + stereoTime2 + drawingTime2 << std::endl;

        duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
        totalDuration += duration_ms;
        std::cout << "OpenCV lsd - red\n\t" << detectedLinesLeft.size() <<" line segments found. For " << duration_ms << " ms, avg per frame: " << totalDuration/(double)cycleNum << " ms." << std::endl;

#endif
*/
        cycleNum++;
//    }

    capture.release();
    std::cout << "End." << std::endl;

    return 0;
}
