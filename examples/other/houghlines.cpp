#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

static void help()
{
    cout << "\nThis program demonstrates line finding with the Hough transform.\n"
            "Usage:\n"
            "./houghlines <image_name>, Default is chairs.pgm\n" << endl;
}

int main(int argc, char** argv)
{
    const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.jpg";

    cv::Mat src = imread(filename, 0);
    if (src.empty())
    {
        help();
        cout << "can not open " << filename << endl;
        return -1;
    }

    cv::Mat dst, cdst;
    Canny(src, dst, 15, 30, 3);
    cvtColor(dst, cdst, COLOR_GRAY2BGR);

#if 0
    double duration_ms = 0;
    vector<Vec2f> lines;
    for (int i = 0; i != 100; ++i) {
        lines.clear();
        double start = double(getTickCount());
        HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );
        duration_ms  += (double(getTickCount()) - start) * 1000 / getTickFrequency();
    }
    std::cout << "Hough Lines: " << lines.size() <<" segments found. For " << duration_ms/100 << " ms." << std::endl;
    

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
    }
#else
    double duration_ms = 0;
    vector<Vec4i> lines;
    for (int i = 0; i != 100; ++i) {
        lines.clear();
        double start = double(getTickCount());
        HoughLinesP(dst, lines, 1, CV_PI / 180, 55, 20, 10);
        duration_ms  += (double(getTickCount()) - start) * 1000 / getTickFrequency();
    }
    std::cout << "Hough Lines: " << lines.size() <<" segments found. For " << duration_ms/100 << " ms." << std::endl;

    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
    }
#endif
    
    imshow("source", src);
    imshow("detected lines", cdst);

    waitKey();

    return 0;
}

