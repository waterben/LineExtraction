#include <edge/nms.hpp>
#include <edge/otsu.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/gradient_adapter.hpp>
#include <imgproc/image_operator.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/susan.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>



using namespace std;
using namespace lsfm;
using namespace cv;

template<class GRAD>
void testGradient(GRAD &grad, const Mat &src, const std::string &name) {
    int runs = 2;
    int64 rt = 0, tmp;
    for (int i = 0; i != runs; ++i) {
        tmp = cv::getTickCount();
        grad.process(src);
        rt += cv::getTickCount() - tmp;
    }

    std::cout << "gradient - " << name <<": " << (rt * 1000.0 / cv::getTickFrequency()) / runs << std::endl;
}

template<class NMS, class GRAD>
void testNMS(NMS &nms, GRAD &grad, const std::string &name, bool force_dir = false, double th_low = 0.004, double th_high = 0.012) {
    int runs = 20;
    int64 rt = 0, tmp;
    for (int i = 0; i != runs; ++i) {
        tmp = cv::getTickCount();
        nms.process(grad,th_low,th_high,force_dir);
        rt += cv::getTickCount() - tmp;
    }

    std::cout << "nms - " << name <<": " << (rt * 1000.0 / cv::getTickFrequency()) / runs << std::endl;
}

template<class GRAD>
void showGradient(const std::string &name,GRAD &grad,int use_range = 0) {
    cv::Mat mag;
    grad.magnitude().convertTo(mag,CV_32F);
    if (use_range) {
        mag /= grad.magnitudeRange().upper;
        if (use_range > 1)
            mag *= use_range;
    } else {
        double vmin,vmax;
        cv::minMaxIdx(mag,&vmin,&vmax);
        mag /= vmax;
        //std::cout << "gradient max - " << name << ": " << vmax << std::endl;
        
        cv::Mat tmp = mag * 255;
        tmp.convertTo(tmp, CV_8U);
        imwrite(name + "_energy.png", tmp);
    }
    imshow("gradient " + name,mag);
}

template<class NMS>
void showNMS(const std::string &name,NMS &nms,bool use_dir = true) {

    //cv::Mat emap = nms.directionMap();
	cv::Mat emap = nms.hysteresis();
    cv::Mat emapImg;
    emapImg.create(emap.rows, emap.cols, CV_8UC3);

    if (use_dir) {
        emapImg.setTo(cv::Vec3b(0, 0, 0));
        emapImg.setTo(cv::Vec3b(220, 150, 255), emap == 7); // magenta2
        emapImg.setTo(cv::Vec3b(255, 0, 150), emap == 6); // lila
        emapImg.setTo(cv::Vec3b(255, 0, 0), emap == 5); // blue
        emapImg.setTo(cv::Vec3b(255, 255, 0), emap == 4); // cyan
        emapImg.setTo(cv::Vec3b(0, 255, 0), emap == 3); // green
        emapImg.setTo(cv::Vec3b(0, 255, 255), emap == 2); // yellow
        emapImg.setTo(cv::Vec3b(0, 150, 255), emap == 1); // orange
        emapImg.setTo(cv::Vec3b(0, 0, 255), emap == 0); // red
    } else {
        emapImg.setTo(cv::Vec3b(255, 255,255),emap >= 0);
    }

    imwrite(name + "_nms.png", emapImg);
    imshow("nms " + name,emapImg);
}


int main(int argc, char** argv)
{
  const char* filename = argc >= 2 ? argv[1] : "../../images/step_line.png";

  cv::Mat src = cv::imread(filename, IMREAD_GRAYSCALE);
  if (src.empty()) {
    cout << "Can not open " << filename << endl;
    return -1;
  }

    if (src.channels() > 1) cvtColor(src, src, cv::COLOR_BGR2GRAY);

    GaussianNoiseOperator noise(10);
    //noise.apply(src);
    GaussianBlur(src, src, cv::Size(3, 3),0.6);

    imshow("img",src);

    DerivativeGradient<uchar, float, float, float, GaussianDerivative,Magnitude> guassian;
    GradientEnergy<QuadratureG2<uchar,float, Polar>> quad({NV("grad_kernel_size",5),NV("grad_kernel_spacing",0.95)});
    
	NonMaximaSuppression<float, float, float, FastNMS8<float, float, float>> f_nms;
	

    guassian.process(src);
    quad.process(src);
    showGradient("guassian", guassian);
    showGradient("quad",quad);

    f_nms.process(guassian, 0.01, 0.1);
    showNMS("guassian",f_nms,false);
    f_nms.process(quad, 0.01, 0.1);
    showNMS("quad", f_nms, false);
   
    waitKey();

    return 0;
}
