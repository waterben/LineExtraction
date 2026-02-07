/// @file derivative_filters.cpp
/// @brief Sobel and Laplacian filter visualization for thesis figures.
///
/// Generates derivative filter output images for thesis Chapter 2:
/// - Sobel gradient magnitude visualization
/// - Laplacian response visualization
/// - Writes results to ./results/chapter_02/
///
/// @usage ./derivative_filters
/// @author Benjamin Wassermann

// C by Benjamin Wassermann

#include <imgproc/derivative_gradient.hpp>
#include <imgproc/laplace.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <filesystem>
#include <iostream>

using namespace lsfm;
using namespace std;
namespace fs = std::filesystem;

constexpr bool invert_images = false;

cv::Mat showOpMag(const cv::Mat& src) {
  cv::Mat mag = src.clone();
  mag.convertTo(mag, CV_32F);
  double vmin, vmax;
  cv::minMaxIdx(mag, &vmin, &vmax);
  mag /= vmax;
  mag *= 255;
  mag.convertTo(mag, CV_8U);

  if constexpr (invert_images) {
    mag = 255 - mag;
  }
  return mag;
}

cv::Mat showOpLapalce(const cv::Mat& src) {
  cv::Mat lap = cv::abs(src);
  return showOpMag(lap);
}

int main(int /*argc*/, char** /*argv*/) {
  DerivativeGradient<uchar, short, float, float, SobelDerivative> sobel;
  LaplaceSimple<uchar, short> laplace;

  std::string file("../../images/windmill.jpg");
  cout << "Generate sobel and laplace filter (3x3) output from: " << file << endl;
  cv::Mat image = cv::imread(file.c_str(), cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    cout << "Failed to open file!" << endl;
    return 1;
  }

  if (image.channels() == 3) cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

  sobel.process(image);
  laplace.process(image);

  fs::create_directory("./results");
  fs::create_directory("./results/chapter_02");
  cv::imwrite("./results/chapter_02/windmill_sobel.png", showOpMag(sobel.magnitude()));
  cv::imwrite("./results/chapter_02/windmill_laplace.png", showOpLapalce(laplace.laplace()));

  return 0;
}
