#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry/draw.hpp>
#include <lsd/lsd_cc.hpp>
#include <lfd/LRDescriptor.hpp>
#include <lfd/FeatureDescriptorLBD.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <list>
#include <lfd/StereoLineFilter.hpp>
#include <lfd/PairwiseLineMatcher.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;

int main(int argc, char** argv)
{

#ifdef WIN32
    std::string filename1 = "../../MiddEval3/testH/Crusade/im0.png";
    std::string filename2 = "../../MiddEval3/testH/Crusade/im1.png";
#else
    std::string filename1 = "../images/elas/im0.png";
    std::string filename2 = "../images/elas/im1.png";
#endif
  
    if (argc > 2) {
        filename1 = argv[1];
        filename2 = argv[2];
    }

    cv::Mat src1 = imread(filename1, IMREAD_GRAYSCALE);
    cv::Mat src2 = imread(filename2, IMREAD_GRAYSCALE);
    
    if (src1.empty() || src2.empty())
    {
        cout << "Can not open files " << endl;
        return -1;
    }

    //GaussianBlur(src1, src1, Size(3, 3), 0.6);
    //GaussianBlur(src2, src2, Size(3, 3), 0.6);

    cv::resize(src1,src1,Size(0,0),0.65,0.65);
    cv::resize(src2,src2,Size(0,0),0.65,0.65);

    typedef double MyType;

    //LsdCC<MyType> lsd1(0/*CC_FIND_NEAR_COMPLEX | CC_CORNER_RULE*/, 0.012, 0.008, 30, 0, 2);
    //LsdCC<MyType> lsd2(0/*CC_FIND_NEAR_COMPLEX | CC_CORNER_RULE*/, 0.012, 0.008, 30, 0, 2);
    LsdCC<MyType> lsd1(0.004, 0.008, 20, 0, 3);
    LsdCC<MyType> lsd2(0.004, 0.008, 20, 0, 3);

    lsd1.detect(src1);
    lsd2.detect(src2);

    MatMap data1, data2;
    data1["gx"] = lsd1.imageData()[0];
    data1["gy"] = lsd1.imageData()[1];
    data1["img"] = src1;

    data2["gx"] = lsd2.imageData()[0];
    data2["gy"] = lsd2.imageData()[1];
    data2["img"] = src2;

    typedef GchGradImgInterpolate<MyType,1,2,NoAlign<MyType>,
            FastRoundNearestInterpolator<MyType, short>,
            FastRoundNearestInterpolator<MyType, uchar> > MyGchHelper;

    typedef FdcLBD<MyType, LsdCC<MyType>::LineSegment, short, FastRoundNearestInterpolator<MyType, short>> MyFdc;
    std::vector<typename MyFdc::descriptor_type> dsc1, dsc2;
    MyFdc::FdcPtr fdc1 = MyFdc::createFdc(data1);
    MyFdc::FdcPtr fdc2 = MyFdc::createFdc(data2);

    std::vector<DescriptorMatch<MyType>> matches, bfmatchesRes;
    std::vector<std::vector<FeatureMatch<MyType>>> matches2D;
    //MotionLineFilter<MyType, LsdCC<MyType>::LineSegmentVector> mlf(100);

    double start = double(getTickCount());
    //MotionLineFilter<MyType, LsdCC<MyType>::LineSegmentVector>::Motion testMotion(3,5);
//    mlf.train(lsd1.lineSegments(), lsd2.lineSegments(), testMotion);
    //slf.create1D(lsd1.lineSegments().size(), lsd2.lineSegments().size(),matches);
    //mlf.create(lsd1.lineSegments(), lsd2.lineSegments(), matches);

    // For Testing only
//    StereoLineFilter<MyType, LsdCC<MyType>::LineSegmentVector> slf(src1.rows, src1.cols / 2);

    MotionLineFilter<MyType, LsdCC<MyType>::LineSegmentVector> mlf(500, 40, src1.size().width, src1.size().height);
    std::pair<MyType, MyType> movement(0,0);
    std::vector<size_t> mask1, mask2;

    start = double(getTickCount());
    mlf.create(lsd1.lineSegments(), lsd2.lineSegments(), movement, matches, mask1, mask2);
    //slf.create(lsd1.lineSegments(), lsd2.lineSegments(), matches, mask1, mask2);

    double end = double(getTickCount());
    std::cout << "stereo line filter time: " << (end - start) * 1000 / getTickFrequency() << "ms" << std::endl;
    std::cout <<  "candidates: " << matches.size() << ", time for MotionLineFilter create: " << (end - start) * 1000 / getTickFrequency() << "ms" << std::endl;
    std::cout << "lines left: " << lsd1.lineSegments().size() << ", lines right: " << lsd2.lineSegments().size() << std::endl;
    //size_t num = matches.size();
    size_t num = 0;
    for_each(matches2D.begin(),matches2D.end(),[&num](const std::vector<FeatureMatch<MyType>>& m) {
       num += m.size();
    });
    //std::cout << "reduced candidates from " << lsd1.lineSegments().size() * lsd2.lineSegments().size() << " to " << num << std::endl;

    start = double(getTickCount());
    fdc1->createList(lsd1.lineSegments(), mask1, dsc1);
    fdc2->createList(lsd2.lineSegments(), mask2, dsc2);
    end = double(getTickCount());
    std::cout << "time for creating descriptors: " << (end - start) * 1000 / getTickFrequency() << std::endl;

//    std::cout << dsc1[1].data << std::endl;
//    std::cout << dsc1[3].data << std::endl;

    PairwiseLineMatcher<MyType, typename MyFdc::descriptor_type> pmatcher;
    start = double(getTickCount());
    pmatcher.match1D(lsd1.lineSegments(), lsd2.lineSegments(), dsc1, dsc2, matches, bfmatchesRes);
    end = double(getTickCount());
    std::cout <<  "matches: " << bfmatchesRes.size() << ", time for matching: " << (end - start) * 1000 / getTickFrequency() << std::endl;

    imshow("Detected matches, Pairwise & Motion Line Filter - nicht so gut", drawMatches<MyType,DescriptorMatch<MyType>>(src1, lsd1.lineSegments(), src2, lsd2.lineSegments(), bfmatchesRes));

    cv::Mat modelImage = lsfm::drawLines<MyType>(src1, lsd1.lineSegments());
    cv::imshow("src1", modelImage);
    modelImage = lsfm::drawLines<MyType>(src2, lsd2.lineSegments());
    cv::imshow("src2", modelImage);


    waitKey();

    /*for (size_t i = 0; i != matches.size(); ++i) {
        imshow("Detected Lines Img1", drawLine<float>(src1, lsd1.lineSegments()[matches[i].queryIdx]));
        imshow("Detected Lines Img2", drawLine<float>(src2, lsd2.lineSegments()[matches[i].matchIdx]));
        waitKey();
    }

    waitKey();*/

    return 0;
}
