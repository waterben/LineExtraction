#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/pyramid.hpp>
#include <utility/matlab_helpers.hpp>
#include <edge/nms.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;

template<class P>
void showPyramid(const std::string &name,P &p, bool BW = false) {
    cv::Mat out = draw(p);
    if (p.type() == CV_32F) {
        out = normalizeMat(out);
    }
    if (BW && p.type() == CV_8S) {
        out.setTo(1,out > -1);
        out.setTo(0,out < 0);
        out.convertTo(out,CV_8U);
        out.setTo(255,out > 0);
    }
    imshow(name,out);
}


int main(int argc, char** argv)
{
    //const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.JPG";
    //const char* filename = argc >= 2 ? argv[1] : "../../images/b1.JPG";
    //const char* filename = argc >= 2 ? argv[1] : "../../images/outsideC.jpg";
    const char* filename = argc >= 2 ? argv[1] : "../../images/hall2_low.JPG";

    cv::Mat src = cv::imread(filename, IMREAD_GRAYSCALE);
    if (src.empty())
    {
        cout << "Can not open " << filename << endl;
        return -1;
    }

    if (src.channels() > 1)
        cvtColor(src, src, CV_BGR2GRAY);

    GaussianBlur(src, src, cv::Size(3, 3),0.6);


    DerivativeGradient<uchar,short,float,float,SobelDerivative> sobel;
    NonMaximaSuppression<short,float,float,FastNMS8<short,float,float>> nms;

    Pyramid<uchar> imgP(src,-3);
    Pyramid<float> imgGrad(src.size(),-3);
    Pyramid<char> imgNMS(src.size(),-3);
    int64 start = cv::getTickCount();
    double t;
    for(int i = 0; i != imgP.size(); ++i) {
        sobel.process(imgP[i]);
        imgGrad[i] = sobel.magnitude();
        nms.process(sobel);
        imgNMS[i] = nms.hysteresis();
        if (i == 0)
            t = (cv::getTickCount() - start) * 1000.0 / cv::getTickFrequency();
    }
    double time = (cv::getTickCount() - start) * 1000.0 / cv::getTickFrequency();
    std::cout << "time image: " <<  t << "ms, time pyramid: " << time << "ms" << std::endl;
    showPyramid("img", imgP);
    showPyramid("grad", imgGrad);
    showPyramid("nms", imgNMS, true);


    waitKey();

    return 0;
}
