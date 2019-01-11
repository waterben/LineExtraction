#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry/draw.hpp>
#include <lsd/lsd_el.hpp>
#include <lfd/GenericDescriptor.hpp>
#include <lfd/FeatureMatcher.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;

int main(int argc, char** argv)
{
    // input data (stereo set, but stereo is not required)
#ifdef WIN32
    std::string filename1 = "../../images/elas/Adirondack/im0.png";
    std::string filename2 = "../../images/elas/Adirondack/im1.png";
#else
    std::string filename1 = "../images/elas/im0.png";
    std::string filename2 = "../images/elas/im1.png";
#endif
  
    if (argc > 2) {
        filename1 = argv[1];
        filename2 = argv[2];
    }

    cv::Mat srcL = cv::imread(filename1, IMREAD_GRAYSCALE);
    cv::Mat srcR = cv::imread(filename2, IMREAD_GRAYSCALE);
    
    if (srcL.empty() || srcR.empty())
    {
        cout << "Can not open files" << endl;
        return -1;
    }

    // downscale image for faster processing
    resize(srcL,srcL,cv::Size(0,0),0.65,0.65);
    resize(srcR,srcR,cv::Size(0,0),0.65,0.65);

    // add some blur to reduce artifacts from noise and aliasing
    GaussianBlur(srcL, srcL, cv::Size(3, 3), 0.6);
    GaussianBlur(srcR, srcR, cv::Size(3, 3), 0.6);

    typedef double MyFloat;

    // init line segment detectors
    LsdEL<MyFloat> lsdL(0.01, 0.03, 50, 3, 50);
    LsdEL<MyFloat> lsdR(0.01, 0.03, 50, 3, 50);

    // detect lines
    double start = double(cv::getTickCount());
    lsdL.detect(srcL);
    lsdR.detect(srcR);
    double end = double(cv::getTickCount());
    std::cout << "lines: " << lsdL.lineSegments().size() + lsdR.lineSegments().size() << ", time for detecting lines: " << (end - start) * 1000 / cv::getTickFrequency() << "ms" << std::endl;

    // create data source maps for descriptor creators
    MatMap dataL, dataR;
    dataL["gx"] = lsdL.imageData()[0];
    dataL["gy"] = lsdL.imageData()[1];
    dataL["img"] = srcL;

    dataR["gx"] = lsdR.imageData()[0];
    dataR["gy"] = lsdR.imageData()[1];
    dataR["img"] = srcR;

    // define helper for descriptor creators (method for creation)
    typedef GchGradImgInterpolate<MyFloat,10,2, NoAlign<MyFloat>,
            LinearInterpolator<MyFloat, short>,
            LinearInterpolator<MyFloat, uchar> > MyGchHelper;

    // define descriptor creator
    typedef FdcGeneric<MyFloat,LsdEL<MyFloat>::LineSegment,MyGchHelper> MyFdc;
    
    // define descriptor lists
    std::vector<typename MyFdc::descriptor_type> dscL, dscR;
    
    // define match list
    std::vector<DescriptorMatch<MyFloat>> bfmatches, bfmatches_th;
    
    // define matcher with max radius (distance threshold -> good matches have a small distance)
    FmBruteForce<MyFloat, typename MyFdc::descriptor_type> bfmatcher(22);

    // create descriptor creators
    MyFdc::FdcPtr fdcL = MyFdc::createFdc(dataL);
    MyFdc::FdcPtr fdcR = MyFdc::createFdc(dataR);

    // create descriptor from detected lines
    start = double(cv::getTickCount());
    fdcL->createList(lsdL.lineSegments(), dscL);
    fdcR->createList(lsdR.lineSegments(), dscR);
    end = double(cv::getTickCount());
    std::cout << "time for creating descriptors: " << (end - start) * 1000 / cv::getTickFrequency() << std::endl;

    // train matcher and get best candidate matches
    start = double(cv::getTickCount());
    bfmatcher.train(dscL, dscR);
    bfmatcher.best(bfmatches);
    end = double(cv::getTickCount());
    std::cout <<  "matches: " << bfmatches.size() << ", time for matching: " << (end - start) * 1000 / cv::getTickFrequency() << std::endl;

    // compute mean distance
    MyFloat mean = 0;
    for_each(bfmatches.begin(), bfmatches.end(), [&](const DescriptorMatch<MyFloat>& data){
        mean += data.distance;
    });
    mean /= bfmatches.size();

    // output results
    std::cout << "mean distance: " << mean << std::endl;
    std::cout <<  "final matches: " << bfmatches.size() << std::endl;
    imshow("Detected matches", drawMatches<MyFloat,DescriptorMatch<MyFloat>>(srcL, lsdL.lineSegments(), srcR, lsdR.lineSegments(), bfmatches));

    cv::waitKey();

    return 0;
}
