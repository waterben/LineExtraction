#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <imgproc/rcmg.hpp>
#include <edge/nms.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>  



using namespace std;
using namespace lsfm;
using namespace cv;
namespace fs = boost::filesystem;

struct MyData {
    MyData(int g = 0, int c = 0, int d = 0) : gray(g), color(c), diff(d) {}
    int gray, color, diff;
};

template <class GRAD1, class GRAD2, class NMS>
double test(const std::vector<fs::path> &files, GRAD1 &rcmg, GRAD2 &rmg, NMS &nms, double blur, float th_low, float th_high, int dilation, bool show = false) {
    std::cout << "run test: " << blur << ", " << th_low << ", " << th_high << ", " << dilation;

    std::vector<MyData> data;
    data.reserve(files.size());

    std::for_each(files.begin(), files.end(), [&](const fs::path& file) {
        cv::Mat src = imread(file.generic_string());
        if (src.empty())
        {
            cout << "Can not open " << file.generic_string() << endl;
        }

        GaussianBlur(src, src, cv::Size(7, 7), blur);

        cv::Mat srcG;
        cvtColor(src, srcG, cv::COLOR_RGB2GRAY);

        rmg.process(srcG);
        nms.process(rmg, th_low, th_high);

        cv::Mat tmp(src.rows, src.cols, CV_8U);
        tmp.setTo(0);

        tmp.setTo(1, nms.hysteresis() > -1);
        int cG = static_cast<int>(sum(tmp)[0]);

        cv::Mat tmp2(src.rows, src.cols, CV_8U);
        tmp2.setTo(0);
        rcmg.process(src);
        nms.process(rcmg, th_low, th_high);
        tmp2.setTo(1, nms.hysteresis() > -1);

        int cRGB = static_cast<size_t>(sum(tmp2)[0]);
        cv::Mat tmp3(src.rows, src.cols, CV_8U);
        cv::Mat d;
        if (dilation > 0) {
            cv::Mat kernel = cv::Mat_<uchar>::ones(dilation, dilation);
            d.create(src.rows, src.cols, CV_8U);
            dilate(tmp, d, kernel);

            tmp3.setTo(0);
            tmp3.setTo(1, (tmp2 - d) > 0);
        }
        else {
            tmp3.setTo(0);
            tmp3.setTo(1, (tmp2 - tmp) > 0);
        }
        int cDiff = static_cast<size_t>(sum(tmp3)[0]);

        data.push_back(MyData(cG, cRGB, cDiff));

        if (show) {
            std::cout << cG << ", " << cRGB << ", " << cDiff << ", " << (static_cast<double>(cG + cDiff) / cG - 1) * 100 << std::endl;
            imshow("image", src);
            imshow("nms gray", tmp > 0);
            imshow("nms RGB", tmp2 > 0);
            if (!d.empty()) imshow("dilate", d > 0);
            imshow("diff", tmp3 > 0);

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

    RCMGradient<uchar, 1, short, int> rmg(3, 1);
    RCMGradient<uchar, 3, short, int> rcmg(3, 1,cv::NORM_L2);
    NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms;
    std::cout << rmg.magnitudeRange().upper << ", " << rcmg.magnitudeRange().upper << ", " << rmg.magnitudeThreshold(0.1) << ", " << rcmg.magnitudeThreshold(0.1) << std::endl;

    double mean = 0;

    std::ofstream f;
    f.open("data.csv");
    f << "Blur (Sigma);Lower Threshold;Upper Threshold;Dilation Size;Information Gain" << std::endl;
    int count = 0;

    double res = test(files, rcmg, rmg, nms, 0.6, 0.004, 0.012, 0);
    mean += res; ++count;
    f << "0.6;0.004;0.012;0;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 1.0, 0.004, 0.012, 0);
    mean += res; ++count;
    f << "1.0;0.004;0.012;0;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 0.6, 0.01, 0.03, 0);
    mean += res; ++count;
    f << "0.6;0.01;0.03;0;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 1.0, 0.01, 0.03, 0);
    mean += res; ++count;
    f << "1.0;0.01;0.03;0;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 0.6, 0.03, 0.06, 0);
    mean += res; ++count;
    f << "0.6;0.03;0.06;0;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 1.0, 0.03, 0.06, 0);
    mean += res; ++count;
    f << "1.0;0.03;0.06;0;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 0.6, 0.004, 0.012, 3);
    mean += res; ++count;
    f << "0.6;0.004;0.012;3;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 1.0, 0.004, 0.012, 3);
    mean += res; ++count;
    f << "1.0;0.004;0.012;3;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 0.6, 0.01, 0.03, 3);
    mean += res; ++count;
    f << "0.6;0.01;0.03;3;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 1.0, 0.01, 0.03, 3);
    mean += res; ++count;
    f << "1.0;0.01;0.03;3;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 0.6, 0.03, 0.06, 3);
    mean += res; ++count;
    f << "0.6;0.03;0.06;3;" << res << std::endl;
    
    res = test(files, rcmg, rmg, nms, 1.0, 0.03, 0.06, 3);
    mean += res; ++count;
    f << "1.0;0.03;0.06;3;" << res << std::endl;

    std::cout << "mean info gain: " << mean / count << std::endl;

    char c;
    std::cin >> c;

    return 0;
}
