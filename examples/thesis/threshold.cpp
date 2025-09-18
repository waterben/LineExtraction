#include <filesystem>
#include <edge/nms.hpp>
#include <edge/threshold.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/susan.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#if defined(__has_include)
#  if __has_include(<opencv2/ximgproc.hpp>)
#    define HAVE_OPENCV_XIMGPROC 1
#    include <opencv2/ximgproc.hpp>
#  endif
#endif
#include <eval/eval_app.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>


using namespace std;
using namespace lsfm;
using namespace cv;
namespace fs = std::filesystem;

template <class OP>
double performanceOP(OP& op, const Mat& src, size_t runs = 10) {
  int64 rt{}, tmp{};
  for (size_t i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    op.process(src);
    rt += cv::getTickCount() - tmp;
  }

  return (static_cast<double>(rt) * 1000.0 / cv::getTickFrequency()) / static_cast<double>(runs);
}

template <class OP>
void showOp(const std::string& name, OP& op, int use_range = 0) {
  cv::Mat mag;
  op.magnitude().copyTo(mag);
  if (use_range) {
    mag /= op.magnitudeRange().upper;
    if (use_range > 1) mag *= use_range;
  } else {
    double vmin, vmax;
    cv::minMaxIdx(mag, &vmin, &vmax);
    mag /= vmax;
  }
  imshow(name, mag);
}

Mat thinImage(const Mat& src) {
  Mat thinnedImage;
  cv::threshold(src, thinnedImage, 0, 255, cv::THRESH_BINARY);
  thinnedImage.convertTo(thinnedImage, CV_8UC1);
#ifdef HAVE_OPENCV_XIMGPROC
  ximgproc::thinning(thinnedImage, thinnedImage, ximgproc::THINNING_ZHANGSUEN);
#endif
  return thinnedImage;
}

class ThresholdApp : public EvalApp {
  cv::Mat img{};
  std::ofstream ofs_;

 public:
  ThresholdApp(std::string name = "ThresholdApp",
               std::string description = "Threshold example processing",
               std::string version = "1.0.0")
      : EvalApp(std::move(name), std::move(description), std::move(version)) {}

  using ConsoleAppInterface::run;

  void defineArgs() {
    ConsoleApp::defineArgs();
    opts_.add_string("input", 'i', "Input file", input_, false, "../../images/windmill.jpg");
    opts_.add_string("output", 'o', "Output folder", output_, false, "./results/threshold_example");
    opts_.add_switch("prio", 'p', "Run as high prio process", run_high_prio_);
    opts_.add_switch("no-results", '\0', "Don't write results", no_results_);
    opts_.add_switch("write-visuals", '\0', "Write visual results", write_visuals_);
    opts_.add_switch("show-visuals", '\0', "Show visual results", show_visuals_);
  }

  template <typename OP>
  void process(const OP& op, const Mat& mat, const std::string& name, const std::string& description) {
    if (verbose_ || !no_results_) {
      auto op_performance = performanceOP(op, mat);
      if (verbose_) {
        std::cout << name << ": " << op_performance << "ms" << std::endl;
      }
      if (!no_results_) {
        std::string table_name = name;
        std::replace(table_name.begin(), table_name.end(), '_', ' ');
        ofs_ << table_name << ";" << op_performance << std::endl;
      }
    }

    cv::Mat mat_op = op.process(mat);

    if (show_visuals_) {
      imshow(description, mat_op);
    }

    if (write_visuals_) {
      cv::imwrite(output_ + "/" + name + ".png", mat_op);
    }
  }

  void initEval() override {
    if (write_visuals_ || !no_results_) {
      fs::create_directory(output_.c_str());
    }

    cout.precision(5);
    cout.setf(std::ios::fixed, std::ios::floatfield);

    if (!no_results_) {
      ofs_.open(output_ + "/threshold_performance.csv");
      ofs_.precision(5);
      ofs_.setf(std::ios::fixed, std::ios::floatfield);
      ofs_ << "Method;Runtime" << std::endl;
    }

    cv::Mat src = cv::imread(input_, IMREAD_GRAYSCALE);
    if (src.empty()) {
      throw std::runtime_error("Can not open " + input_);
    }

    if (src.channels() > 1) cvtColor(src, src, cv::COLOR_BGR2GRAY);

    GaussianBlur(src, img, cv::Size(3, 3), 0.8);
  }


  void runEval() override {
    // Apply Otsu th on image
    if (verbose_) {
      ThresholdOtsu<uchar, 256> otsu_img{};
      int th = otsu_img.process(img);
      std::cout << "otsu_img: max: 255, th: " << th << ", runtime: " << performanceOP(otsu_img, img) << "ms"
                << std::endl;
    }

    GlobalThreshold<uchar, ThresholdOtsu<uchar, 256>> otsu_global_th_img;
    process(otsu_global_th_img, img, "otsu_global_th_img", "Image Otsu th");

    // Generate edge responses
    DerivativeGradient<uchar, short, float, float, SobelDerivative> sobel;
    sobel.process(img);

    cv::Mat mag;
    sobel.magnitude().copyTo(mag);

    // Otsu global on magnitude
    ThresholdOtsu<float, 256> otsu_mag(sobel.magnitudeRange().upper);
    if (verbose_) {
      int th = otsu_mag.process(mag);
      std::cout << "otsu_mag: max: " << sobel.magnitudeRange().upper << ", th: " << th
                << ", runtime:" << performanceOP(otsu_mag, mag) << "ms" << std::endl;
    }

    GlobalThreshold<float, ThresholdOtsu<float, 256>, true> otsu_global_th_mag(sobel.magnitudeRange().upper);
    process(otsu_global_th_mag, mag, "otsu_global_th_mag", "Sobel mag Otsu th");

    GlobalThreshold<float, ThresholdOtsu<float, 256>> otsu_global_th_mag_st(sobel.magnitudeRange().upper);
    process(otsu_global_th_mag_st, mag, "otsu_global_th_mag_st", "Sobel mag Otsu th single thread");

    if (show_visuals_ || write_visuals_) {
      GlobalThreshold<float, ThresholdUser<float>> otsu_global_th_low(sobel.magnitudeRange().upper,
                                                                      sobel.magnitudeRange().upper * 0.03);
      GlobalThreshold<float, ThresholdUser<float>> otsu_global_th_good(sobel.magnitudeRange().upper,
                                                                       sobel.magnitudeRange().upper * 0.12);
      GlobalThreshold<float, ThresholdUser<float>> otsu_global_th_high(sobel.magnitudeRange().upper,
                                                                       sobel.magnitudeRange().upper * 0.35);

      cv::Mat mag_th_low = otsu_global_th_low.process(mag);
      cv::Mat mag_th_good = otsu_global_th_good.process(mag);
      cv::Mat mag_th_high = otsu_global_th_high.process(mag);
      cv::Mat mag_th_otsu = otsu_global_th_mag.process(mag);

      cv::Mat mag_th_good_thin = thinImage(mag_th_good);
      cv::Mat mag_th_otsu_thin = thinImage(mag_th_otsu);

      if (show_visuals_) {
        imshow("Image", img);
        imshow("Sobel magnitude", mag / sobel.magnitudeRange().upper);
        imshow("Sobel magnitude with low threshold", mag_th_low);
        imshow("Sobel magnitude with good threshold", mag_th_good);
        imshow("Sobel magnitude with high threshold", mag_th_high);
        imshow("Sobel magnitude with good threshold thinned", mag_th_good_thin);
        imshow("Sobel magnitude with Otsu threshold thinned", mag_th_otsu_thin);
      }

      if (write_visuals_) {
        cv::imwrite(output_ + "/image.png", img);
        cv::imwrite(output_ + "/sobel_mag.png", mag / sobel.magnitudeRange().upper * 255.0);
        cv::imwrite(output_ + "/low_global_th_mag.png", mag_th_low);
        cv::imwrite(output_ + "/good_global_th_mag.png", mag_th_good);
        cv::imwrite(output_ + "/high_global_th_mag.png", mag_th_high);
        cv::imwrite(output_ + "/good_thin_global_th_mag.png", mag_th_good_thin);
        cv::imwrite(output_ + "/otsu_thin_global_th_mag.png", mag_th_otsu_thin);
      }
    }


    LocalThresholdTiles<float, ThresholdOtsu<float, 256>> otsu_local_tiles_th_3x3_mag(3, 3,
                                                                                      sobel.magnitudeRange().upper);
    process(otsu_local_tiles_th_3x3_mag, mag, "otsu_local_tiles_th_3x3_mag", "Sobel mag local tiles (3x3) Otsu th");

    LocalThresholdTiles<float, ThresholdOtsu<float, 256>> otsu_local_tiles_th_10x10_mag(10, 10,
                                                                                        sobel.magnitudeRange().upper);
    process(otsu_local_tiles_th_10x10_mag, mag, "otsu_local_tiles_th_10x10_mag",
            "Sobel mag local tiles (10x10) Otsu th");

    LocalThresholdTiles<float, ThresholdOtsu<float, 256>> otsu_local_tiles_th_20x20_mag(20, 20,
                                                                                        sobel.magnitudeRange().upper);
    process(otsu_local_tiles_th_20x20_mag, mag, "otsu_local_tiles_th_20x20_mag",
            "Sobel mag local tiles (20x20) Otsu th");

    LocalThresholdTiles<float, ThresholdOtsu<float, 256>, false> otsu_local_tiles_th_20x20_mag_st(
        20, 20, sobel.magnitudeRange().upper);
    process(otsu_local_tiles_th_20x20_mag_st, mag, "otsu_local_tiles_th_20x20_mag_st",
            "Sobel mag local tiles (20x20) Otsu th single thread");

    LocalThresholdTiles<float, ThresholdOtsu<float, 256>> otsu_local_tiles_th_3x3_2_mag(
        3, 3, sobel.magnitudeRange().upper, 2.f);
    process(otsu_local_tiles_th_3x3_2_mag, mag, "otsu_local_tiles_th_3x3_2_mag",
            "Sobel mag local tiles (3x3, scale=2) Otsu th");

    LocalThresholdTiles<float, ThresholdOtsu<float, 256>> otsu_local_tiles_th_10x10_2_mag(
        10, 10, sobel.magnitudeRange().upper, 2.f);
    process(otsu_local_tiles_th_10x10_2_mag, mag, "otsu_local_tiles_th_10x10_2_mag",
            "Sobel mag local tiles (10x10, scale=2) Otsu th");

    LocalThresholdTiles<float, ThresholdOtsu<float, 256>> otsu_local_tiles_th_20x20_2_mag(
        20, 20, sobel.magnitudeRange().upper, 2.f);
    process(otsu_local_tiles_th_20x20_2_mag, mag, "otsu_local_tiles_th_20x20_2_mag",
            "Sobel mag local tiles (20x20, scale=2) Otsu th");

    LocalThresholdTiles<float, ThresholdOtsu<float, 256>, false> otsu_local_tiles_th_20x20_2_st_mag(
        20, 20, sobel.magnitudeRange().upper, 2.f);
    process(otsu_local_tiles_th_20x20_2_st_mag, mag, "otsu_local_tiles_th_20x20_2_st_mag",
            "Sobel mag local tiles (20x20, scale=2) Otsu th single thread");

    DynamicThreshold<float, ThresholdOtsu<float, 256>> otsu_dynamic_th_r10_mag(10, sobel.magnitudeRange().upper);
    process(otsu_dynamic_th_r10_mag, mag, "otsu_dynamic_th_r10_mag", "Sobel mag dynamic (r10) Otsu th");

    DynamicThreshold<float, ThresholdOtsu<float, 256>> otsu_dynamic_th_r20_mag(20, sobel.magnitudeRange().upper);
    process(otsu_dynamic_th_r20_mag, mag, "otsu_dynamic_th_r20_mag", "Sobel mag dynamic (r20) Otsu th");

    DynamicThreshold<float, ThresholdOtsu<float, 256>> otsu_dynamic_th_r30_mag(30, sobel.magnitudeRange().upper);
    process(otsu_dynamic_th_r30_mag, mag, "otsu_dynamic_th_r30_mag", "Sobel mag dynamic (r30) Otsu th");

    DynamicThreshold<float, ThresholdOtsu<float, 256>> otsu_dynamic_th_r40_mag(40, sobel.magnitudeRange().upper);
    process(otsu_dynamic_th_r40_mag, mag, "otsu_dynamic_th_r40_mag", "Sobel mag dynamic (r40) Otsu th");

    DynamicThreshold<float, ThresholdOtsu<float, 256>> otsu_dynamic_th_r50_mag(50, sobel.magnitudeRange().upper);
    process(otsu_dynamic_th_r50_mag, mag, "otsu_dynamic_th_r50_mag", "Sobel mag dynamic (r50) Otsu th");

    DynamicThreshold<float, ThresholdOtsu<float, 256>, false> otsu_dynamic_th_r50_mag_st(50,
                                                                                         sobel.magnitudeRange().upper);
    process(otsu_dynamic_th_r50_mag_st, mag, "otsu_dynamic_th_r50_mag_st",
            "Sobel mag dynamic (r50) Otsu th single thread");

    // DynamicThreshold<uchar, ThresholdOtsu<uchar, 256>> otsu_dynamic_th_r10_img(10, 255);
    // process(otsu_dynamic_th_r10_img, img, "otsu_dynamic_th_r10_img", "Image dynamic (r10) Otsu th");

    // DynamicThreshold<uchar, ThresholdOtsu<uchar, 256>> otsu_dynamic_th_r20_img(20, 255);
    // process(otsu_dynamic_th_r20_img, img, "otsu_dynamic_th_r20_img", "Image dynamic (r20) Otsu th");

    // DynamicThreshold<uchar, ThresholdOtsu<uchar, 256>> otsu_dynamic_th_r30_img(30, 255);
    // process(otsu_dynamic_th_r30_img, img, "otsu_dynamic_th_r30_img", "Image dynamic (r30) Otsu th");

    // DynamicThreshold<uchar, ThresholdOtsu<uchar, 256>> otsu_dynamic_th_r40_img(40, 255);
    // process(otsu_dynamic_th_r40_img, img, "otsu_dynamic_th_r40_img", "Image dynamic (r40) Otsu th");

    // DynamicThreshold<uchar, ThresholdOtsu<uchar, 256>> otsu_dynamic_th_r50_img(50, 255);
    // process(otsu_dynamic_th_r50_img, img, "otsu_dynamic_th_r50_img", "Image dynamic (r50) Otsu th");


    waitKey();
  }
};

int main(int argc, char** argv) {
  ThresholdApp app("thesis_imgproc_threshold_examples");
  return app.run(argc, argv);
}
