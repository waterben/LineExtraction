#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <imgproc/steerable.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;


int main(int argc, char** argv)
{
    double r = 0;
    do {
        Mat_<double> sg1 = SteerGaussianD1<double>(r,501,5);
        //std::cout << gaussianD1<double>(21,4) << std::endl;
        Mat_<double> sg2 = SteerGaussianD2<double>(r,501,5);
        //std::cout << gaussianD2<double>(21,4) << std::endl;

        double vmin, vmax;
        cv::minMaxIdx(sg1,&vmin,&vmax);
        std::cout << "min: " << vmin << ", max: " << vmax << std::endl;
        imshow("gaussian d1", sg1 / (vmax - vmin) + 0.5);

        cv::minMaxIdx(sg2,&vmin,&vmax);
        imshow("gaussian d2", sg2 / (vmax - vmin) + 0.5);
        std::cout << "min: " << vmin << ", max: " << vmax << std::endl;
        r += CV_PI / 180;
    } while (cv::waitKey() % 256 != 'q');


    return 0;
}
