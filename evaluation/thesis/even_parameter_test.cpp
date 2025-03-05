#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <imgproc/derivative_gradient.hpp>
#define DISABLE_DC_ZERO_FIX
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/laplace.hpp>
#include <dlib/optimization.h>

using namespace std;
using namespace lsfm;
using namespace cv;

void showGradient(const std::string &name,const cv::Mat &mag, double mul = 1) {
    double vmin,vmax;
    cv::minMaxIdx(mag,&vmin,&vmax);
    mag -= vmin;
    mag /= vmax - vmin;
    mag *= mul;
    imshow("gradient " + name,mag);
}

double d_lower = 0.1, d_upper = 10, start = 1;
cv::Mat white(18, 18, CV_64F, 1);

template <
    class GRAD,
    class search_strategy_type = dlib::bfgs_search_strategy,
    class stop_strategy_type = dlib::objective_delta_stop_strategy
>
double optimizeGradKernel(GRAD& grad,
    double derivative_prec = 1e-7, search_strategy_type search = dlib::bfgs_search_strategy(),
    stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {

    typedef dlib::matrix<double, 0, 1> column_vector;
        
    auto eval = [&](const column_vector& v) -> double {
        grad.kernelSpacing(v(0));
        return std::abs(cv::sum(grad.kernel())[0]);
    };

    grad.kernelSpacing(start);
    column_vector starting_point(1), lower(1), upper(1);
    starting_point = start;
    lower = d_lower;
    upper = d_upper;
    return dlib::find_min_box_constrained(search, stop,
        eval, dlib::derivative(eval, derivative_prec), starting_point, lower, upper);

}

template <
    class GRAD,
    class search_strategy_type = dlib::bfgs_search_strategy,
    class stop_strategy_type = dlib::objective_delta_stop_strategy
>
double optimizeGradKernel2( GRAD& grad,
    double derivative_prec = 1e-7, search_strategy_type search = dlib::bfgs_search_strategy(),
    stop_strategy_type stop = dlib::objective_delta_stop_strategy(1e-7)) {

    typedef dlib::matrix<double, 0, 1> column_vector;

    auto eval = [&](const column_vector& v) -> double {
        grad.kernelSpacing(v(0));
        grad.process(white);
        cv::Mat mag = grad.even();
        return std::abs(mag.at<double>(0, 0));
    };

    grad.kernelSpacing(start);
    column_vector starting_point(1), lower(1), upper(1);
    starting_point = start;
    lower = d_lower;
    upper = d_upper;
    return dlib::find_min_box_constrained(search, stop,
        eval, dlib::derivative(eval, derivative_prec), starting_point, lower, upper);

}


int main(int argc, char** argv)
{
  const char* filename = argc >= 2 ? argv[1] : "../../images/circle2.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/bike.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.JPG";

  cv::Mat src = cv::imread(filename, IMREAD_GRAYSCALE);
  if (src.empty()) {
    cout << "Can not open " << filename << endl;
    return -1;
    }

    GaussianBlur(src, src, cv::Size(3, 3),0.6);
    typedef double FT;

    QuadratureG2<uchar, FT> quad3(3, 1.240080);
	QuadratureG2<uchar, FT> quad5(5, 1.008000);
    QuadratureG2<uchar, FT> quad7(7, 0.873226);
    QuadratureG2<uchar, FT> quad9(9, 0.781854);

    quad3.process(src);
    showGradient("Quad3 -", Mat(abs(quad3.even())));

    quad5.process(src);
    showGradient("Quad5 -", Mat(abs(quad5.even())));

    quad7.process(src);
    showGradient("Quad7 -", Mat(abs(quad7.even())));

    quad9.process(src);
    showGradient("Quad9 -", Mat(abs(quad9.even())));

    cv::waitKey();

    QuadratureS<uchar, FT, FT> quadS3(1, 2, 3, 1);
    QuadratureS<uchar, FT, FT> quadS5(1, 2, 5, 1);
    QuadratureS<uchar, FT, FT> quadS7(1, 2, 7, 1);
    QuadratureS<uchar, FT, FT> quadS9(1, 2, 9, 1);
    
    LoG<uchar, FT> log3(3, 1);
    LoG<uchar, FT> log5(5, 1);
    LoG<uchar, FT> log7(7, 1);
    LoG<uchar, FT> log9(9, 1);
    LoG<uchar, FT> log11(11, 1);
    LoG<uchar, FT> log15(15, 1);
    LoG<uchar, FT> log25(25, 1);
    LoG<uchar, FT> log75(75, 1);
    LoG<uchar, FT> log125(125, 1);
    
    double e;
    e = optimizeGradKernel(log3);
    std::cout << "LoG3 - error: " << e << ", spacing: " << log3.kernelSpacing() << std::endl;
    //std::cout << log3.kernel() << std::endl;

    e = optimizeGradKernel(log5);
    std::cout << "LoG5 - error: " << e << ", spacing: " << log5.kernelSpacing() << std::endl;
    //std::cout << log5.kernel() << std::endl;
    
    e = optimizeGradKernel(log7);
    std::cout << "LoG7 - error: " << e << ", spacing: " << log7.kernelSpacing() << std::endl;
    //std::cout << log7.kernel() << std::endl;

    e = optimizeGradKernel(log9);
    std::cout << "LoG9 - error: " << e << ", spacing: " << log9.kernelSpacing() << std::endl;
    //std::cout << log9.kernel() << std::endl;

    e = optimizeGradKernel2(log3);
    std::cout << "LoG3 - error2: " << e << ", spacing: " << log3.kernelSpacing() << std::endl;
    //std::cout << log3.kernel() << std::endl;

    e = optimizeGradKernel2(log5);
    std::cout << "LoG5 - error2: " << e << ", spacing: " << log5.kernelSpacing() << std::endl;
    //std::cout << log5.kernel() << std::endl;

    e = optimizeGradKernel2(log7);
    std::cout << "LoG7 - error2: " << e << ", spacing: " << log7.kernelSpacing() << std::endl;
    //std::cout << log7.kernel() << std::endl;

    e = optimizeGradKernel2(log9);
    std::cout << "LoG9 - error2: " << e << ", spacing: " << log9.kernelSpacing() << std::endl;
    //std::cout << log9.kernel() << std::endl;

    e = optimizeGradKernel2(log11);
    std::cout << "LoG11 - error2: " << e << ", spacing: " << log11.kernelSpacing() << std::endl;
    //std::cout << log11.kernel() << std::endl;

    e = optimizeGradKernel2(log15);
    std::cout << "LoG15 - error2: " << e << ", spacing: " << log15.kernelSpacing() << std::endl;
    //std::cout << log15.kernel() << std::endl;

    e = optimizeGradKernel2(log25);
    std::cout << "LoG25 - error2: " << e << ", spacing: " << log25.kernelSpacing() << std::endl;
    //std::cout << log25.kernel() << std::endl;

    e = optimizeGradKernel2(log75);
    std::cout << "LoG75 - error2: " << e << ", spacing: " << log75.kernelSpacing() << std::endl;
    //std::cout << log75.kernel() << std::endl;

    e = optimizeGradKernel2(log125);
    std::cout << "LoG125 - error2: " << e << ", spacing: " << log125.kernelSpacing() << std::endl;
    //std::cout << log25.kernel() << std::endl;
    
    
    /*log3.process(src);
    showGradient("LoG3 -", log3.laplace());

    log5.process(src);
    showGradient("LoG5 -", log5.laplace());

    log7.process(src);
    showGradient("LoG7 -", log7.laplace());

    log9.process(src);
    showGradient("LoG9 -", log9.laplace());

    cv::waitKey();*/

    e = optimizeGradKernel2(quad3);
    std::cout << "Quad3 - error: " << e << ", spacing: " << quad3.kernelSpacing() << std::endl;

    e = optimizeGradKernel2(quad5);
    std::cout << "Quad5 - error: " << e << ", spacing: " << quad5.kernelSpacing() << std::endl;

    e = optimizeGradKernel2(quad7);
    std::cout << "Quad7 - error: " << e << ", spacing: " << quad7.kernelSpacing() << std::endl;

    e = optimizeGradKernel2(quad9);
    std::cout << "Quad9 - error: " << e << ", spacing: " << quad9.kernelSpacing() << std::endl;


    quad3.process(src);
    showGradient("Quad3 -", Mat(abs(quad3.even())));

    quad5.process(src);
    showGradient("Quad5 -", Mat(abs(quad5.even())));

    quad7.process(src);
    showGradient("Quad7 -", Mat(abs(quad7.even())));

    quad9.process(src);
    showGradient("Quad9 -", Mat(abs(quad9.even())));

    cv::waitKey();

    e = optimizeGradKernel(quadS3);
    std::cout << "QuadS3 - error: " << e << ", spacing: " << quadS3.kernelSpacing() << ", scale: " << quadS3.scale() << ", muls: " << quadS3.muls() << std::endl;
    //std::cout << quadS3.kernel() << std::endl;

    e = optimizeGradKernel(quadS5);
    std::cout << "QuadS5 - error: " << e << ", spacing: " << quadS5.kernelSpacing() << ", scale: " << quadS5.scale() << ", muls: " << quadS5.muls() << std::endl;
    //std::cout << quadS5.kernel() << std::endl;

    e = optimizeGradKernel(quadS7);
    std::cout << "QuadS7 - error: " << e << ", spacing: " << quadS7.kernelSpacing() << ", scale: " << quadS7.scale() << ", muls: " << quadS7.muls() << std::endl;
    //std::cout << quadS7.kernel() << std::endl;

    e = optimizeGradKernel(quadS9);
    std::cout << "QuadS9 - error: " << e << ", spacing: " << quadS9.kernelSpacing() << ", scale: " << quadS9.scale() << ", muls: " << quadS9.muls() << std::endl;
    //std::cout << quadS9.kernel() << std::endl;

    e = optimizeGradKernel2(quadS3);
    std::cout << "QuadS3 - error2: " << e << ", spacing: " << quadS3.kernelSpacing() << ", scale: " << quadS3.scale() << ", muls: " << quadS3.muls() << std::endl;
    //std::cout << quadS3.kernel() << std::endl;

    e = optimizeGradKernel2(quadS5);
    std::cout << "QuadS5 - error2: " << e << ", spacing: " << quadS5.kernelSpacing() << ", scale: " << quadS5.scale() << ", muls: " << quadS5.muls() << std::endl;
    //std::cout << quadS5.kernel() << std::endl;

    e = optimizeGradKernel2(quadS7);
    std::cout << "QuadS7 - error2: " << e << ", spacing: " << quadS7.kernelSpacing() << ", scale: " << quadS7.scale() << ", muls: " << quadS7.muls() << std::endl;
    //std::cout << quadS7.kernel() << std::endl;

    e = optimizeGradKernel2(quadS9);
    std::cout << "QuadS9 - error2: " << e << ", spacing: " << quadS9.kernelSpacing() << ", scale: " << quadS9.scale() << ", muls: " << quadS9.muls() << std::endl;
    //std::cout << quadS9.kernel() << std::endl;


    /*quadS3.process(src);
    showGradient("QuadS3 -", quadS3.even());

    quadS5.process(src);
    showGradient("QuadS5 -", quadS5.even());

    quadS7.process(src);
    showGradient("QuadS7 -", quadS7.even());

    quadS9.process(src);
    showGradient("QuadS9 -", quadS9.even());

    cv::waitKey();*/

    

    /*QuadratureS<uchar, FT, FT> quadS5(1.0, 2.0, 5, 1);
    QuadratureS<uchar, FT, FT> quadS7(1.0, 2.0, 7, 1);
    QuadratureS<uchar, FT, FT> quadS9(1.0, 2.0, 9, 0.1);

    quadS5.process(src);
    quadS7.process(src);
    quadS9.process(src);

    //showGradient("QuadS5 -", quadS5.laplace(),5);
    //showGradient("QuadS7 -", quadS7.laplace(),5);
    showGradient("QuadS9 -", quadS9.laplace(),5);

    //showGradient("QuadS5 m-", quadS5.magnitude());
    //showGradient("QuadS7 m-", quadS7.magnitude());
    showGradient("QuadS9 m-", quadS9.magnitude());

    //showGradient("QuadS5 lm-", quadS5.localMagnitude());
    //showGradient("QuadS7 lm-", quadS7.localMagnitude());
    showGradient("QuadS9 lm-", quadS9.localMagnitude());

    //showGradient("QuadS5 e-", quadS5.even());
    //showGradient("QuadS7 e-", quadS7.even());
    showGradient("QuadS9 e-", quadS9.even());
    cv::waitKey();*/

    return 0;
}
