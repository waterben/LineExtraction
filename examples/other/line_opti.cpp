#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>

#include <geometry/draw.hpp>
#include <lsd/lsd_cc.hpp>
#include "cvplot.h"

using namespace std;
using namespace lsfm;
using namespace cv;

static void help()
{
    cout << "\nThis program demonstrates cc lsd.\n"
        "Usage:\n"
        "./test_lsd_data <image_name>, Default is ../../images/office1_low.jpg\n" << endl;
}

int main(int argc, char** argv)
{
    const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.jpg";
    //const char* filename = argc >= 2 ? argv[1] : "../../images/lines.jpg";
    //const char* filename = argc >= 2 ? argv[1] : "../../images/a1.jpg";

    cv::Mat src = imread(filename, 0);
    if (src.empty())
    {
        help();
        cout << "Can not open " << filename << endl;
        return -1;
    }

    if (src.channels() != 1)
        cvtColor(src, src, CV_RGB2GRAY);
    
    cv::Mat out;
    cvtColor(src, out, CV_GRAY2BGR);


    // Reduce noise with a kernel 3x3
    blur(src, src, Size(3, 3));
    //GaussianBlur(src, src, Size(3, 3),0.6);
    
    typedef double MyType;

    LsdCC<MyType> lsd(0/*CC_FIND_NEAR_COMPLEX | CC_CORNER_RULE*/, 0.032, 0.012, 25, 0, 2);

    double stime = double(getTickCount());
    lsd.detect(src);
    std::cout << "Time for line detection: " << (double(getTickCount()) - stime) * 1000 / getTickFrequency() << "ms" << std::endl;
        
    const LineSegment2Vector<double>& lines = lsd.lineSegments();
    vector<cv::Mat> imageData = lsd.imageData();

    // create color map for segments
    RNG rng(time(0));
    Scalar red(0, 0, 255);
    int border = 20;
    int subSteps = 10;
    LineSegment2Vector<double>::const_iterator beg = lines.begin(), iend = lines.end();

    for (int idx = 0; beg != iend; ++beg, ++idx) {
        std::cout << "Selected line: " << idx << "\r" << std::flush;
        //Point2f p1 = l.normal_s(0, l.center()), p2 = l.normal_s(10, l.center());
        //drawLines(src, p1, p2, Scalar(0, 0, 255));
        line(out, *beg, red);
        imshow("Select line", out);
        const LineSegment<double> &l = *beg;
        cv::Mat mag = imageData[2];

        int angle = static_cast<int>(ceil(l.anglef() + 45)) % 180;
        std::vector<double> profile(2 * border * subSteps + 1, 0);
        std::vector<double> X;
        X.resize(profile.size());
        Vec2<double> pos;

        auto interpolate = [](const cv::Mat &src, const Vec2<double> &pos) {
            int x1 = static_cast<int>(pos.x());
            int y1 = static_cast<int>(pos.y());

            // no interpolation
            if (x1 < 0 || y1 < 0 || x1 >= src.cols || y1 >= src.rows)
                return 0.0;

            double xd = pos.x() - x1;
            double yd = pos.y() - y1;

            // no interpolation
            if (xd == 0 && yd == 0)
                return static_cast<double>(src.at<int>(y1, x1));
            // interpolate with 2 pixels
            else if (xd == 0) {
                if (y1 + 1 >= src.rows) 
                    return 0.0;
                return src.at<int>(y1, x1) * (1 - yd) + src.at<int>(y1 + 1, x1) * yd;
            }
            // interpolate with 2 pixels
            else if (yd == 0)  {
                if (x1 + 1 >= src.cols) 
                    return 0.0;
                return src.at<int>(y1, x1) * (1 - xd) + src.at<int>(y1, x1 + 1) * xd;
            }
            // interpolate with 4 pixels
            else {
                if (y1 + 1 >= src.rows || x1 + 1 >= src.cols) 
                    return 0.0;
                double ix1 = src.at<int>(y1, x1) * (1 - yd) + src.at<int>(y1 + 1, x1) * yd;
                double ix2 = src.at<int>(y1, x1 + 1) * (1 - yd) + src.at<int>(y1 + 1, x1 + 1) * yd;
                return ix1 * (1 - xd) + ix2 * xd;
            }

        };

        double start = ceil(angle > 90 ? l.startPoint().y() : l.startPoint().x());
        double end = floor(angle > 90 ? l.endPoint().y() : l.endPoint().x());
        if (start > end)
            swap(start, end);

        for (; start < end; ++start) {
            pos.x() = angle > 90 ? l.x(start) : start;
            pos.y() = angle > 90 ? start : l.y(start);
            for (int i = 0; i != profile.size(); ++i) {
                double x = -border + static_cast<double>(i) / subSteps;
                X[i] = x;
                profile[i] += interpolate(mag, pos + l.normalLineDist(x));
            }
        }

        CvPlot::clear("Profile");
        CvPlot::plot("Profile", &profile[0], static_cast<int>(profile.size()));

        int key = waitKey();
        switch (key) {
            case 'q':
            case 'Q':
            case  27:
                return 0;
            default:
            {
                
                Vec3b color(50 + rng.uniform(0, 150), 50 + rng.uniform(0, 150), 50 + rng.uniform(0, 150));
                Scalar scolor(color[0], color[1], color[2]);
                line(out, *beg, scolor);
            }
                continue;
        }
        break;
    };  
    
    waitKey();

    return 0;
}
