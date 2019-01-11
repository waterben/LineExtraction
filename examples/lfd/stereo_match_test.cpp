#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry/draw.hpp>
#include <lsd/lsd_cc.hpp>
#include <lfd/StereoLineFilter.hpp>
#include <list>


using namespace std;
using namespace lsfm;
using namespace cv;


int main(int argc, char** argv)
{
#ifdef WIN32
    std::string filename1 = "../../MiddEval3/testH/Crusade/im0.png";
    std::string filename2 = "../../MiddEval3/testH/Crusade/im1.png";
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

    GaussianBlur(src1, src1, Size(3, 3), 0.6);
    GaussianBlur(src2, src2, Size(3, 3), 0.6);

    typedef float MyType;

    LsdCC<MyType> lsd1(0.008, 0.012, 30, 0, 2);
    LsdCC<MyType> lsd2(0.008, 0.012, 30, 0, 2);

    lsd1.detect(src1);
    lsd2.detect(src2);

    std::vector<FeatureMatch<MyType>> matches, matches2;
    StereoLineFilter<MyType, LsdCC<MyType>::LineSegmentVector> slf(src1.cols);

    double start = double(getTickCount());
    slf.train(lsd1.lineSegments(), lsd2.lineSegments());
    //slf.create1D(lsd1.lineSegments().size(), lsd2.lineSegments().size(),matches);
    slf.create(lsd1.lineSegments().size(), lsd2.lineSegments().size(),matches2);
    double end = double(getTickCount());
    std::cout << "stereo line filter time: " << (end - start) * 1000 / getTickFrequency() << "ms" << std::endl;
    std::cout << "lines left: " << lsd1.lineSegments().size() << ", lines right: " << lsd2.lineSegments().size() << std::endl;
    //size_t num = matches.size();
    size_t num = matches2.size();

    std::cout << "reduced candidates from " << lsd1.lineSegments().size() * lsd2.lineSegments().size() << " to " << num << std::endl;

    for (size_t i = 0; i != matches2.size(); ++i) {
        imshow("Detected Lines Img1", drawLine(src1, lsd1.lineSegments()[matches2[i].queryIdx]));
        imshow("Detected Lines Img2", drawLine(src2, lsd2.lineSegments()[matches2[i].matchIdx]));
        waitKey();
    }

    waitKey();

    return 0;
}
