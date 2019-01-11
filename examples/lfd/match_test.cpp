#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry/draw.hpp>
#include <lsd/lsd_cc.hpp>
#include <lfd/FeatureDescriptorLBD.hpp>
#include <lfd/FeatureMatcher.hpp>
#include <lfd/GlobalRotationFilter.hpp>
#include <lfd/PairwiseLineMatcher.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>

#include <opencv2/line_descriptor/descriptor.hpp>
#include <opengl_sim/lineMatching.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;

typedef float MyFloat;


int main(int argc, char** argv)
{
#ifdef WIN32
    std::string filename1 = "../../images/elas/Adirondack/im0.png";
    std::string filename2 = "../../images/elas/Adirondack/im1.png";
#else
    std::string filename1 = "../../images/elas/im0.png";
    std::string filename2 = "../../images/elas/im1.png";
#endif

    if (argc > 2) {
        filename1 = argv[1];
        filename2 = argv[2];
    }

    cv::Mat src1 = imread(filename1, IMREAD_GRAYSCALE);
    cv::Mat src2 = imread(filename2, IMREAD_GRAYSCALE);
    
    if (src1.empty() || src2.empty())
    {
        cout << "Can not open files" << endl;
        return -1;
    }

    resize(src1,src1,Size(0,0),0.6,0.6);
    resize(src2,src2,Size(0,0),0.6,0.6);
    GaussianBlur(src1, src1, Size(3, 3), 0.6);
    GaussianBlur(src2, src2, Size(3, 3), 0.6);

    LsdCC<MyFloat> lsd1(0.008f, 0.012f, 50, 0, 2);
    LsdCC<MyFloat> lsd2(0.008f, 0.012f, 50, 0, 2);

    lsd1.detect(src1);
    lsd2.detect(src2);

//    imshow("dx", lsd1.imageData()[0]);
//    imshow("dy", lsd1.imageData()[1]);

//    waitKey();

    cv::Mat dsc1, dsc2;
     
    FdcLBD<MyFloat, LsdCC<MyFloat>::LineSegment, short, FastRoundNearestInterpolator<MyFloat, short>> fdc1(lsd1.imageData()[0], lsd1.imageData()[1], 7, 5);
    FdcLBD<MyFloat, LsdCC<MyFloat>::LineSegment, short, FastRoundNearestInterpolator<MyFloat, short>> fdc2(lsd2.imageData()[0], lsd2.imageData()[1], 7, 5);

    std::vector<FdLBD<MyFloat>> lbd1, lbd2;
    //Vector<FdLBD<MyFloat>> lbd1, lbd2;
    //std::list<FdLBD<MyFloat>> lbd1, lbd2;

    double start = double(getTickCount());
    fdc1.createMat(lsd1.lineSegments(), dsc1);
    fdc2.createMat(lsd2.lineSegments(), dsc2);
    double end = double(getTickCount());
    std::cout << "time for mat dsc: " << (end - start) * 1000 / getTickFrequency() << std::endl;

    start = double(getTickCount());
    fdc1.createList(lsd1.lineSegments(), lbd1);
    fdc2.createList(lsd2.lineSegments(), lbd2);
    end = double(getTickCount());
    std::cout << "time for lbd dsc: " << (end - start) * 1000 / getTickFrequency() << std::endl;

    GlobalRotationFilter<MyFloat, LsdCC<MyFloat>::LineSegmentVector> rf;
    start = double(getTickCount());
    rf.train(lsd1.lineSegments(), lsd2.lineSegments());
    end = double(getTickCount());
    std::cout << "time for rf train: " << (end - start) * 1000 / getTickFrequency() << std::endl;
    
    //std::vector<int> lm, rm;
    std::vector<DescriptorMatch<MyFloat>> ml;
    
    start = double(getTickCount());
    rf.create(lsd1.lineSegments().size(), lsd2.lineSegments().size(), ml/*, lm, rm*/);
    end = double(getTickCount());
    std::cout << "time for rf create: " << (end - start) * 1000 / getTickFrequency() << std::endl;

    //std::cout << norm(dsc1.row(0)) << std::endl;
    //std::cout << dsc1.row(0) << std::endl;

    //dsc1.convertTo(dsc1, CV_32F);
    //dsc2.convertTo(dsc2, CV_32F); 


    std::vector<DescriptorMatch<MyFloat>> bfmatches, bfmatchesR;
    FmBruteForce<MyFloat, FdLBD<MyFloat>> bfmatcher;
    
    start = double(getTickCount());
    bfmatcher.train(lbd1, lbd2, rf);
    end = double(getTickCount());
    std::cout << "time for lbd bf train: " << (end - start) * 1000 / getTickFrequency() << std::endl;

    start = double(getTickCount());
    bfmatcher.match(lbd1, lbd2, rf, bfmatches);
    end = double(getTickCount());
    std::cout << "time for lbd bf matcher: " << (end - start) * 1000 / getTickFrequency() << std::endl;

    start = double(getTickCount());
    bfmatcher.radius(bfmatchesR, 0.001f);
    end = double(getTickCount());
    std::cout << "time for lbd bf radius: " << (end - start) * 1000 / getTickFrequency() << ", candidates: " << bfmatchesR.size() << std::endl;

    std::vector<DescriptorMatch<MyFloat>> pmatches;
    PairwiseLineMatcher<MyFloat, FdLBD<MyFloat>> pmatcher;

    //start = double(getTickCount());
    //pmatcher.match2D(lsd1.lineSegments(), lsd2.lineSegments(), lbd1, lbd2, ml, pmatches);
    //end = double(getTickCount());
    //std::cout << "time for lbd pairwise matcher: " << (end - start) * 1000 / getTickFrequency() << std::endl;
    
    start = double(getTickCount());
    pmatcher.match1D(lsd1.lineSegments(), lsd2.lineSegments(), lbd1, lbd2, bfmatchesR, pmatches);
    end = double(getTickCount());
    std::cout << "time for lbd pairwise matcher with bfmatcher: " << (end - start) * 1000 / getTickFrequency() << std::endl;
    std::cout << "candidates: " << bfmatchesR.size() << std::endl;


    std::vector< DMatch > mbfmatches;
    BFMatcher mbfmatcher;

    start = double(getTickCount());       
    mbfmatcher.match(dsc1, dsc2, mbfmatches);
    end = double(getTickCount());
    std::cout << "time for mat bf matcher: " << (end - start) * 1000 / getTickFrequency() << std::endl;


    std::vector< DMatch > flannMatches;
    FlannBasedMatcher flmatcher;

    start = double(getTickCount());
    flmatcher.match(dsc1, dsc2, flannMatches);
    end = double(getTickCount());
    std::cout << "time for mat flann matcher: " << (end - start) * 1000 / getTickFrequency() << std::endl;

    
    MyFloat mmax_dist = 0; MyFloat mmin_dist = 100000;
    
    //-- Quick calculation of max and min distances between lines
    for (int i = 0; i < dsc1.rows; i++)
    {
        MyFloat dist = mbfmatches[i].distance;
        if (dist < mmin_dist) mmin_dist = dist;
        if (dist > mmax_dist) mmax_dist = dist;
    }

    std::cout << "-- Max dist mbf: " << mmax_dist << std::endl;
    std::cout << "-- Min dist mbf: " << mmin_dist << std::endl;

    mmax_dist = 0; mmin_dist = 100000;
    //-- Quick calculation of max and min distances between lines
    for (int i = 0; i < dsc1.rows; i++)
    {
        MyFloat dist = flannMatches[i].distance;
        if (dist < mmin_dist) mmin_dist = dist;
        if (dist > mmax_dist) mmax_dist = dist;
    }

    std::cout << "-- Max dist flann: " << mmax_dist << std::endl;
    std::cout << "-- Min dist flann: " << mmin_dist << std::endl;

    MyFloat max_dist = 0; MyFloat min_dist = 100000;
    //-- Quick calculation of max and min distances between lines
    for (int i = 0; i < bfmatches.size(); i++)
    {
        MyFloat dist = bfmatches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    std::cout << "-- Max dist bf: " << max_dist << std::endl;
    std::cout << "-- Min dist bf: " << min_dist << std::endl;

    MyFloat pmax_dist = 0; MyFloat pmin_dist = 100000;
    //-- Quick calculation of max and min distances between lines
    for (int i = 0; i < pmatches.size(); i++)
    {
        MyFloat dist = pmatches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    std::cout << "-- Max dist p: " << max_dist << std::endl;
    std::cout << "-- Min dist p: " << min_dist << std::endl;


    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    /*std::vector< DMatch > good_matches;
    

    for (int i = 0; i < dsc1.rows; i++)
    {
        if (mbfmatches[i].distance <= std::max(2 * mmin_dist, 0.1f))
        {
            good_matches.push_back(mbfmatches[i]);
        }
    }

    std::cout << "-- Num all  Matches: " << mbfmatches.size() << std::endl;
    std::cout << "-- Num good Matches: " << good_matches.size() << std::endl;*/

    /*std::vector< DescriptorMatch<MyFloat> > good_matches;
    for (int i = 0; i < dsc1.rows; i++)
    {
        if (bfmatches[i].distance <= 0.15f)
        {
            good_matches.push_back(bfmatches[i]);
        }
    }

    std::cout << "-- Num all  Matches: " << bfmatches.size() << std::endl;
    std::cout << "-- Num good Matches: " << good_matches.size() << std::endl;*/

    cv::Mat im1 = drawLines<MyFloat>(src1, lsd1.lineSegments());
    cv::Mat im2 = drawLines<MyFloat>(src2, lsd2.lineSegments());
//    cv::resize(im1, im1, cv::Size(im1.size().width / 2, im1.size().height / 2));
//    cv::resize(im2, im2, cv::Size(im2.size().width / 2, im2.size().height / 2));
    imshow("Detected Lines Img1", im1);
    imshow("Detected Lines Img2", im2);

    cv::Mat im3 = drawMatches<MyFloat>(src1, lsd1.lineSegments(), src2, lsd2.lineSegments(), pmatches);
//    cv::resize(im3, im3, cv::Size(im3.size().width / 2, im3.size().height / 2));
    imshow("Detected matches pmatches - wenig aber gut", im3);


    std::vector<cv::line_descriptor::KeyLine> klV1, klV2;
//    Mat leftDEscr, rightDescr;
    for ( int i = 0; i < lsd1.lineSegments().size(); i++ ){

        //if( klsd1[i].octave == 0 ){
//            octave0_1.push_back( klsd1[i] );
            klV1.push_back( lsfm::lineSegment2KeyLine(lsd1.lineSegments()[i], src1 ) );
        //}
    }

    for ( int j = 0; j < lsd2.lineSegments().size(); j++ ){

        //if( klsd2[j].octave == 0 ) {
//            octave0_2.push_back( klsd2[j] );
            klV2.push_back( lineSegment2KeyLine(lsd2.lineSegments()[j], src2 ) );
        //}
    }
    imshow("Detected matches flann - schlecht", drawKeyLineMatches<MyFloat,DescriptorMatch<MyFloat>>(src1, klV1, src2, klV2, flannMatches));

    imshow("Detected matches mbfmatches - schlecht", drawKeyLineMatches<MyFloat,DescriptorMatch<MyFloat>>(src1, klV1, src2, klV2, mbfmatches));


    waitKey();

    return 0;
}
