#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/matlab_helpers.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;


template<class FT>
void showFFT(const std::string &name, const cv::Mat &fft) {
    cv::Mat tmp = fftshift<std::complex<FT>>(fft);
    showMat(name + "real", real<FT>(tmp));
    showMat(name + "imag", imag<FT>(tmp));
}


int main(int argc, char** argv)
{
    //const char* filename = argc >= 2 ? argv[1] : "../../images/circle.png";
    //const char* filename = argc >= 2 ? argv[1] : "../../images/ssmall.png";
    //const char* filename = argc >= 2 ? argv[1] : "../../images/hall2_low.JPG";
    const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.JPG";
    //const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low_quad.png";

    cv::Mat im = cv::imread(filename, IMREAD_GRAYSCALE);
    if (im.empty())
    {
        cout << "Can not open " << filename << endl;
        return -1;
    }

    if (im.channels() > 1)
        cvtColor(im, im, cv::COLOR_BGR2GRAY);

    typedef double FT;
    
    cv::Mat src;
    im.copyTo(src);
    // adapt image size for perfft -> filter will be corrupted!
    int r = cv::getOptimalDFTSize(src.rows), c = cv::getOptimalDFTSize(src.cols);
    if (src.cols < c || src.rows < r)
        cv::copyMakeBorder(src, src, 0, r - src.rows, 0, c - src.cols, cv::BORDER_REPLICATE);
    
    cv::Mat fft = fft2<FT>(src);
    showFFT<FT>("fft", fft);
    cv::Mat perfft = perfft2<FT>(src);
    showFFT<FT>("perfft", perfft);

    cv::Mat ifft = real<FT>(ifft2<FT>(fft));
    if (ifft.cols > im.cols || ifft.rows > im.rows) 
        ifft.adjustROI(0, im.rows - ifft.rows, 0, im.cols - ifft.cols);

    cv::Mat iper = real<FT>(ifft2<FT>(perfft));
    if (iper.cols > im.cols || iper.rows > im.rows)
        iper.adjustROI(0, im.rows - iper.rows, 0, im.cols - iper.cols);
    
    showMat("src", im);
    showMat("ifft", ifft);
    showMat("iperfft", iper);

    waitKey();

    return 0;
}
