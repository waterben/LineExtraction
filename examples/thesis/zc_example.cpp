#include <filesystem>
#include <edge/threshold.hpp>
#include <edge/zc.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/laplace.hpp>
#include <utility/eval_app.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>


using namespace std;
using namespace lsfm;
using namespace cv;
namespace fs = std::filesystem;


cv::Mat visLaplace(const cv::Mat& laplace, bool green = false) {
  double minVal, maxVal;
  cv::minMaxLoc(laplace, &minVal, &maxVal);

  cv::Mat negative = cv::max(-laplace, 0);
  cv::Mat positive = cv::max(laplace, 0);

  if (minVal < 0) {
    negative.convertTo(negative, CV_8UC1, 255.0 / -minVal);
  } else {
    negative = cv::Mat::zeros(laplace.size(), CV_8UC1);
  }

  if (maxVal > 0) {
    positive.convertTo(positive, CV_8UC1, 255.0 / maxVal);
  } else {
    positive = cv::Mat::zeros(laplace.size(), CV_8UC1);
  }


  cv::Mat fill(laplace.size(), CV_8UC1, cv::Scalar(255));
  cv::Mat neg_result;
  if (green) {
    merge(std::vector<cv::Mat>({255 - negative, fill, 255 - negative}), neg_result);
  } else {
    merge(std::vector<cv::Mat>({fill, 255 - negative, 255 - negative}), neg_result);
  }

  cv::Mat pos_result;
  merge(std::vector<cv::Mat>({255 - positive, 255 - positive, fill}), pos_result);


  cv::Mat output(laplace.size(), CV_8UC3, cv::Scalar(255, 255, 255));
  pos_result.copyTo(output, laplace > 0);
  neg_result.copyTo(output, laplace < 0);

  return output;
}

class ZcApp : public EvalApp {
  cv::Mat img{};

 public:
  ZcApp(std::string name = "ZcExampleApp",
        std::string description = "Zero crossing example processing",
        std::string version = "1.0.0")
      : EvalApp(std::move(name), std::move(description), std::move(version)) {}

  using ConsoleAppInterface::run;

  void defineArgs() {
    ConsoleApp::defineArgs();
    // clang-format off
        options_.add_options()
        ("input,i", boost::program_options::value<std::string>(&input_)->default_value("../../images/windmill.jpg"), "Input file, defaults to ../../windmill.jpg")
        ("output,o", boost::program_options::value<std::string>(&output_)->default_value("./results/zc_examples"), "Output folder")
        ("prio,p", boost::program_options::bool_switch(&run_high_prio_), "Run as high prio process")
        ("no-results", boost::program_options::bool_switch(&no_results_), "Don't write results")
        ("write-visuals", boost::program_options::bool_switch(&write_visuals_), "Write visual results")
        ("show-visuals", boost::program_options::bool_switch(&show_visuals_), "Show visual results");
    // clang-format on
  }

  template <class ZC>
  void visZC(const std::string& name, ZC& zc) {
    // cv::Mat emap = nms.directionMap();
    cv::Mat emap = zc.hysteresis();
    cv::Mat emap_img(cv::Size(emap.rows, emap.cols), CV_8UC3, cv::Scalar(255, 255, 255));

    // emap_img.setTo(cv::Vec3b(0, 0, 0));
    emap_img.setTo(cv::Vec3b(200, 130, 230), emap == 7);  // magenta2
    emap_img.setTo(cv::Vec3b(200, 0, 150), emap == 6);    // lila
    emap_img.setTo(cv::Vec3b(230, 0, 0), emap == 5);      // blue
    emap_img.setTo(cv::Vec3b(220, 220, 0), emap == 4);    // cyan
    emap_img.setTo(cv::Vec3b(0, 230, 0), emap == 3);      // green
    emap_img.setTo(cv::Vec3b(0, 220, 220), emap == 2);    // yellow
    emap_img.setTo(cv::Vec3b(0, 150, 200), emap == 1);    // orange
    emap_img.setTo(cv::Vec3b(0, 0, 230), emap == 0);      // red

    if (show_visuals_) {
      imshow("zc_" + name, emap_img);
    }

    if (write_visuals_) {
      cv::imwrite(output_ + "/zc_" + name + ".png", emap_img);
    }
  }


  void initEval() override {
    if (write_visuals_ || !no_results_) {
      fs::create_directory(output_.c_str());
    }

    cout.precision(5);
    cout.setf(std::ios::fixed, std::ios::floatfield);

    img = cv::imread(input_, IMREAD_GRAYSCALE);
    if (img.empty()) {
      throw std::runtime_error("Can not open " + input_);
    }

    if (img.channels() > 1) cvtColor(img, img, cv::COLOR_BGR2GRAY);
    GaussianBlur(img, img, cv::Size(3, 3), 0.6);
  }


  void runEval() override {
    LaplaceSimple<uchar, short> laplace;
    laplace.process(img);
    auto lap = laplace.laplace();
    auto abs_lap = cv::abs(lap);

    short abs_lap_max = std::max(std::abs(laplace.laplaceRange().upper), std::abs(laplace.laplaceRange().lower));
    ThresholdOtsu<short, 256> otsu_lap(abs_lap_max);
    short otsu_th_lap = otsu_lap.process(abs_lap);
    float otsu_th_lap_n = static_cast<float>(otsu_th_lap) / abs_lap_max / 2;

    if (verbose_) {
      std::cout << "otsu_lap: max: " << abs_lap_max << ", otsu_lap_th: " << otsu_th_lap
                << ", otsu_lpa_th normalized: " << otsu_th_lap_n << std::endl;
    }

    ZeroCrossing<short, short> zc_fast8(otsu_th_lap_n / 3.f, otsu_th_lap_n);

    zc_fast8.process(laplace);
    auto zc_result_otsu_th = zc_fast8.hysteresisBinary();

    cv::Mat lap_red_green = visLaplace(lap, true);
    cv::Mat lap_red_blue = visLaplace(lap);

    if (show_visuals_) {
      imshow("zc otsu", zc_result_otsu_th);
      imshow("Laplace red green", lap_red_green);
      imshow("Laplace red blue", lap_red_blue);
    }

    if (write_visuals_) {
      cv::imwrite(output_ + "/zc_otsu.png", zc_result_otsu_th);
      cv::imwrite(output_ + "/laplace_red_green.png", lap_red_green);
      cv::imwrite(output_ + "/laplace_red_blue.png", lap_red_blue);
    }

    img = cv::imread("../images/circle2.png", IMREAD_GRAYSCALE);

    DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
    sobel.process(img);
    laplace.process(img);
    zc_fast8.processG(laplace, sobel);
    visZC("fast8_grad", zc_fast8);

    ZeroCrossing<short, short, float, FastZC<short, short, float, NCC_BASIC, EZCMap4>> zc_fast4(otsu_th_lap_n / 3.f,
                                                                                                otsu_th_lap_n);

    zc_fast4.processG(laplace, sobel);
    visZC("fast4_grad", zc_fast4);
    waitKey();
  }
};

int main(int argc, char** argv) {
  ZcApp app("thesis_zero_crossing_examples");
  return app.run(argc, argv);
}
