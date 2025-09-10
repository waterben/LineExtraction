#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <edge/nms.hpp>

#include <filesystem>
#include <algorithm>  



using namespace std;
using namespace lsfm;
using namespace cv;
namespace fs = std::filesystem;

struct MyData {
    MyData(int g = 0, int c = 0, int d = 0) : gray(g), color(c), diff(d) {}
    int gray, color, diff;
};

template <class GRAD, class NMS>
double test(const std::vector<fs::path> &files, GRAD &grad, NMS &nms, double blur, float th_low, float th_high, int dilation, bool show = false) {
    std::cout << "run test: " << blur << ", " << th_low << ", " << th_high << ", " << dilation;
    
    std::vector<MyData> data;
    data.reserve(files.size());

    std::for_each(files.begin(), files.end(), [&](const fs::path& file) {
        cv::Mat src = imread(file.generic_string(), IMREAD_COLOR);
        if (src.empty())
        {
            cout << "Can not open " << file.generic_string() << endl;
        }

        GaussianBlur(src, src, cv::Size(7, 7), blur);

        cv::Mat srcG;
        cvtColor(src, srcG, cv::COLOR_RGB2GRAY);
        cv::Mat srcRGB[3];
        split(src, srcRGB);

        grad.process(srcG);
        nms.process(grad, th_low, th_high);

        cv::Mat tmp(src.rows, src.cols, CV_8U);
        tmp.setTo(0);
        
        tmp.setTo(1, nms.hysteresis() > -1);
        int cG = static_cast<int>(sum(tmp)[0]);

        cv::Mat tmp2(src.rows, src.cols, CV_8U);
        tmp2.setTo(0);
        grad.process(srcRGB[0]);
        nms.process(grad, th_low, th_high);
        tmp2.setTo(1, nms.hysteresis() > -1);

        grad.process(srcRGB[1]);
        nms.process(grad, th_low, th_high);
        tmp2.setTo(1, nms.hysteresis() > -1);

        grad.process(srcRGB[2]);
        nms.process(grad, th_low, th_high);
        tmp2.setTo(1, nms.hysteresis() > -1);

        int cRGB = static_cast<size_t>(sum(tmp2)[0]);


        cv::Mat kernel = cv::Mat_<uchar>::ones(dilation, dilation);
        cv::Mat d(src.rows, src.cols, CV_8U);
        dilate(tmp, d, kernel);

        cv::Mat tmp3(src.rows, src.cols, CV_8U);
        tmp3.setTo(0);
        tmp3.setTo(1, (tmp2 - d) > 0);
        int cDiff = static_cast<size_t>(sum(tmp3)[0]);

        //std::cout << cG << ", " << cRGB << ", " << cDiff << std::endl;
        data.push_back(MyData(cG, cRGB, cDiff));

        if (show) {
            imshow("image", src > 0);
            imshow("nms gray", tmp > 0);
            imshow("nms RGB", tmp2 > 0);
            imshow("dilate", d > 0);
            imshow("diff", (tmp2 - d) > 0);

            waitKey(0);
        }
    });

    double sumG = 0, sumRGB = 0, sumDiff = 0;

    std::for_each(data.begin(), data.end(), [&sumG, &sumRGB, &sumDiff](const MyData &d) {
        sumG += d.gray;
        sumRGB += d.color;
        sumDiff += d.diff;
    });

    sumRGB = ((sumG + sumDiff) / sumG - 1) * 100;
    std::cout << " - " << sumRGB << std::endl;
    return sumRGB;
}


int main(int argc, char** argv)
{
  const char* cfolder = argc >= 2 ? argv[1] : "../../images/BSDS500";

  fs::path folder(cfolder);

  if (!fs::is_directory(folder)) {
    cout << "Can not open folder" << cfolder << endl;
    return -1;
  }

    std::vector<fs::path> files;
    fs::directory_iterator end_iter;
    for_each(fs::directory_iterator(folder), fs::directory_iterator(), [&files](const fs::path& file) {
        if (fs::is_regular_file(file))
        {
            std::string ext = file.extension().generic_string();
            boost::algorithm::to_lower(ext);
            if (ext == ".jpg" || ext == ".png") {
                files.push_back(file);
                //std::cout << file.filename().string() << std::endl;
            }
        }
    });


    std::cout << "Found image files: " << files.size() << std::endl;

    DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
    NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms;
    
    double mean = 0;

    std::ofstream f;
    f.open("data.csv");
    f << "Blur (Sigma);Lower Threshold;Upper Threshold;Dilation Size;Information Gain" << std::endl;
    int count = 0;
    double res = test(files, sobel, nms, 1, 0.03, 0.06, 3, true);

    res = test(files, sobel, nms, 0.6, 0.004, 0.012, 3);
    mean += res; ++count;
    f << "0.6;0.004;0.012;3;" << res << std::endl;
    //res = test(files, sobel, nms, 0.8, 0.004, 0.012, 3);
    //mean += res; ++count;
    //f << "0.8;0.004;0.012;3;" << res << std::endl;
    res = test(files, sobel, nms, 1.0, 0.004, 0.012, 3);
    mean += res; ++count;
    f << "1.0;0.004;0.012;3;" << res << std::endl;
    //res = test(files, sobel, nms, 1.2, 0.004, 0.012, 3);
    //mean += res; ++count;
    //f << "1.2;0.004;0.012;3;" << res << std::endl;

    res = test(files, sobel, nms, 0.6, 0.01, 0.03, 3);
    mean += res; ++count;
    f << "0.6;0.01;0.03;3;" << res << std::endl;
    //res = test(files, sobel, nms, 0.8, 0.01, 0.03, 3);
    //mean += res; ++count;
    //f << "0.8;0.01;0.03;3;" << res << std::endl;
    res = test(files, sobel, nms, 1.0, 0.01, 0.03, 3);
    mean += res; ++count;
    f << "1.0;0.01;0.03;3;" << res << std::endl;
    //res = test(files, sobel, nms, 1.2, 0.01, 0.03, 3);
    //mean += res; ++count;
    //f << "1.2;0.01;0.03;3;" << res << std::endl;

    res = test(files, sobel, nms, 0.6, 0.03, 0.06, 3);
    mean += res; ++count;
    f << "0.6;0.03;0.06;3;" << res << std::endl;
    //res = test(files, sobel, nms, 0.8, 0.03, 0.06, 3);
    //mean += res; ++count;
    //f << "0.8;0.03;0.06;3;" << res << std::endl;
    res = test(files, sobel, nms, 1.0, 0.03, 0.06, 3);
    mean += res; ++count;
    f << "1.0;0.03;0.06;3;" << res << std::endl;
    //res = test(files, sobel, nms, 1.2, 0.03, 0.06, 3);
    //mean += res; ++count;
    //f << "1.2;0.03;0.06;3;" << res << std::endl;

   /* res = test(files, sobel, nms, 0.6, 0.05, 0.1, 3);
    mean += res; ++count;
    f << "0.6;0.05;0.1;3;" << res << std::endl;
    res = test(files, sobel, nms, 0.8, 0.05, 0.1, 3);
    mean += res; ++count;
    f << "0.8;0.05;0.1;3;" << res << std::endl;
    res = test(files, sobel, nms, 1.0, 0.05, 0.1, 3);
    mean += res; ++count;
    f << "1.0;0.05;0.1;3;" << res << std::endl;
    res = test(files, sobel, nms, 1.2, 0.05, 0.1, 3);
    mean += res; ++count;
    f << "1.2;0.05;0.1;3;" << res << std::endl;*/

    res = test(files, sobel, nms, 0.6, 0.004, 0.012, 5);
    mean += res; ++count;
    f << "0.6;0.004;0.012;5;" << res << std::endl;
    //res = test(files, sobel, nms, 0.8, 0.004, 0.012, 5);
    //mean += res; ++count;
    //f << "0.8;0.004;0.012;5;" << res << std::endl;
    res = test(files, sobel, nms, 1.0, 0.004, 0.012, 5);
    mean += res; ++count;
    f << "1.0;0.004;0.012;5;" << res << std::endl;
    //res = test(files, sobel, nms, 1.2, 0.004, 0.012, 5);
    //mean += res; ++count;
    //f << "1.2;0.004;0.012;5;" << res << std::endl;

    res = test(files, sobel, nms, 0.6, 0.01, 0.03, 5);
    mean += res; ++count;
    f << "0.6;0.01;0.03;5;" << res << std::endl;
    //res = test(files, sobel, nms, 0.8, 0.01, 0.03, 5);
    //mean += res; ++count;
    //f << "0.8;0.01;0.03;5;" << res << std::endl;
    res = test(files, sobel, nms, 1.0, 0.01, 0.03, 5);
    mean += res; ++count;
    f << "1.0;0.01;0.03;5;" << res << std::endl;
    //res = test(files, sobel, nms, 1.2, 0.01, 0.03, 5);
    //mean += res; ++count;
    //f << "1.2;0.01;0.03;5;" << res << std::endl;

    res = test(files, sobel, nms, 0.6, 0.03, 0.06, 5);
    mean += res; ++count;
    f << "0.6;0.03;0.06;5;" << res << std::endl;
    //res = test(files, sobel, nms, 0.8, 0.03, 0.06, 5);
    //mean += res; ++count;
    //f << "0.8;0.03;0.06;5;" << res << std::endl;
    res = test(files, sobel, nms, 1.0, 0.03, 0.06, 5);
    mean += res; ++count;
    f << "1.0;0.03;0.06;5;" << res << std::endl;
    //res = test(files, sobel, nms, 1.2, 0.03, 0.06, 5);
    //mean += res; ++count;
    //f << "1.2;0.03;0.06;5;" << res << std::endl;

    /*res = test(files, sobel, nms, 0.6, 0.05, 0.1, 5);
    mean += res; ++count;
    f << "0.6;0.05;0.1;5;" << res << std::endl;
    res = test(files, sobel, nms, 0.8, 0.05, 0.1, 5);
    mean += res; ++count;
    f << "0.8;0.05;0.1;5;" << res << std::endl;
    res = test(files, sobel, nms, 1.0, 0.05, 0.1, 5);
    mean += res; ++count;
    f << "1.0;0.05;0.1;5;" << res << std::endl;
    res = test(files, sobel, nms, 1.2, 0.05, 0.1, 5);
    mean += res; ++count;
    f << "1.2;0.05;0.1;5;" << res << std::endl;*/
    
    std::cout << "mean info gain: " << mean / count << std::endl;

    char c;
    std::cin >> c;

    return 0;
}
