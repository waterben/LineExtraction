// C by Benjamin Wassermann

#include <filesystem>
#include <imgproc/image_operator.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace lsfm;
using namespace std;
namespace fs = std::filesystem;

constexpr bool invert_images = false;

cv::Mat showOp(const cv::Mat& src) {
  // cv::Mat mag = cv::abs(src);
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

int main(int argc, char** argv) {
  GaussianBlurOperator gaussian5(5);
  GaussianBlurOperator gaussian11(11);
  std::string file("../../images/windmill.jpg");

  cout << "Generate gaussian pyramid differences output from: " << file << endl;
  cv::Mat src = cv::imread(file.c_str(), cv::IMREAD_GRAYSCALE);
  if (src.empty()) {
    cout << "Failed to open file!" << endl;
    return 1;
  }
  if (src.channels() == 3) cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);

  cv::Mat res_gaussian = src.clone();
  gaussian5.apply(res_gaussian);
  cv::Mat res_gaussian2 = src.clone();
  gaussian11.apply(res_gaussian2);

  fs::create_directory("./results");
  fs::create_directory("./results/chapter_02");

  cv::imwrite("./results/chapter_02/windmill_gaussian_once.png", res_gaussian);
  cv::imwrite("./results/chapter_02/windmill_gaussian_twice.png", res_gaussian2);


  cv::Mat div1;  // = src - res_gaussian;
  cv::absdiff(src, res_gaussian, div1);
  cv::Mat div2;  // = res_gaussian - res_gaussian2;
  cv::absdiff(res_gaussian, res_gaussian2, div2);

  cv::imwrite("./results/chapter_02/windmill_dog_a.png", showOp(div1));
  cv::imwrite("./results/chapter_02/windmill_dog_b.png", showOp(div2));

  return 0;
}
