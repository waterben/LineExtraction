#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/format.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#define WRITE_IMAGE_FILES
using namespace lsfm;
using namespace std;
namespace fs = std::filesystem;

#ifdef WRITE_IMAGE_FILES
constexpr int runs = 1;
#else
constexpr int runs = 10;
#endif

struct Entry {
  Entry() {}

  Entry(const ImageOperatorPtr& a, const std::string& b) : filter(a), name(b) {}


  ImageOperatorPtr filter;
  std::string name;

  inline cv::Mat process(const cv::Mat src) {
    cv::Mat ret;
    filter->apply(src, ret);
    return ret;
  }
};

template <class NMS>
cv::Mat createNMS(NMS& nms) {
  // cv::Mat emap = nms.directionMap();
  cv::Mat emap = nms.hysteresis();
  cv::Mat emapImg;
  emapImg.create(emap.rows, emap.cols, CV_8UC3);

  emapImg.setTo(cv::Vec3b(0, 0, 0));
  emapImg.setTo(cv::Vec3b(220, 150, 255), emap == 7);  // magenta2
  emapImg.setTo(cv::Vec3b(255, 0, 150), emap == 6);    // lila
  emapImg.setTo(cv::Vec3b(255, 0, 0), emap == 5);      // blue
  emapImg.setTo(cv::Vec3b(255, 255, 0), emap == 4);    // cyan
  emapImg.setTo(cv::Vec3b(0, 255, 0), emap == 3);      // green
  emapImg.setTo(cv::Vec3b(0, 255, 255), emap == 2);    // yellow
  emapImg.setTo(cv::Vec3b(0, 150, 255), emap == 1);    // orange
  emapImg.setTo(cv::Vec3b(0, 0, 255), emap == 0);      // red
  return emapImg;
}

void parseFolder(const fs::path& folder, std::vector<fs::path>& files) {
  fs::directory_iterator end_iter;
  for_each(fs::directory_iterator(folder), fs::directory_iterator(), [&files](const fs::path& file) {
    if (fs::is_regular_file(file)) {
      std::string ext = file.extension().generic_string();
      std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
      if (ext == ".jpg" || ext == ".png") {
        files.push_back(file);
      }
    }
    if (fs::is_directory(file)) parseFolder(file, files);
  });
}

double processError(Entry& e, const fs::path& path, int n, double& time) {
  std::cout << "processing noise" << n << " (" << e.name << ")...";
  std::vector<fs::path> files;
  parseFolder(path, files);
  GaussianNoiseOperator op_noise(n);
#ifdef WRITE_IMAGE_FILES
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms;
#endif

  double ret = 0;
  time = 0;
  int count = 0;
  std::for_each(files.begin(), files.end(), [&](const fs::path& file) {
    cv::Mat src = cv::imread(file.generic_string()), src_n;
    if (src.empty()) {
      cout << "Can not open " << file.generic_string() << endl;
      return;
    }

    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    src_n = src.clone();
    op_noise(src_n);

    cv::Mat res;
    int64 start;
    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();
      res = e.process(src_n);
      time += static_cast<double>(cv::getTickCount() - start);
    }

#ifdef WRITE_IMAGE_FILES
    std::string name = e.name;
    std::replace(name.begin(), name.end(), ' ', '_');
    cv::imwrite(utility::format("./denoise/%s_noise%i_%s.png", file.stem().generic_string(), n, name), res);
    sobel.process(res);
    nms.process(sobel);
    cv::imwrite(utility::format("./denoise/%s_noise%i_nms_%s.png", file.stem().generic_string(), n, name),
                createNMS(nms));
#endif
    res.convertTo(res, CV_32S);
    src.convertTo(src, CV_32S);

    ret += sum(abs(src - res))[0] / res.size().area() / 255;
    ++count;
  });
  ret /= count;
  time *= 1000 / (count * runs * cv::getTickFrequency());
  std::cout << "time: " << time << "ms, error: " << ret << std::endl;
  return ret;
}

int main() {
  char c;
  std::cin >> c;

  fs::path path = "../../images/MDB/MiddEval3-Q";

  std::vector<Entry> filter;

  filter.push_back(Entry(NoOp::create(), "noop"));

  filter.push_back(Entry(BlurOperator::create(3), "box 3"));
  filter.push_back(Entry(BlurOperator::create(5), "box 5"));
  filter.push_back(Entry(BlurOperator::create(7), "box 7"));
  filter.push_back(Entry(BlurOperator::create(9), "box 9"));

  filter.push_back(Entry(GaussianBlurOperator::create(3), "gauss 3"));
  filter.push_back(Entry(GaussianBlurOperator::create(5), "gauss 5"));
  filter.push_back(Entry(GaussianBlurOperator::create(7), "gauss 7"));
  filter.push_back(Entry(GaussianBlurOperator::create(9), "gauss 9"));

  filter.push_back(Entry(MedianBlurOperator::create(3), "median 3"));
  filter.push_back(Entry(MedianBlurOperator::create(5), "median 5"));
  filter.push_back(Entry(MedianBlurOperator::create(7), "median 7"));
  filter.push_back(Entry(MedianBlurOperator::create(9), "median 9"));

  filter.push_back(Entry(BilateralOperator::create(3), "bilat 3"));
  filter.push_back(Entry(BilateralOperator::create(5), "bilat 5"));
  filter.push_back(Entry(BilateralOperator::create(7), "bilat 7"));
  filter.push_back(Entry(BilateralOperator::create(9), "bilat 9"));

  filter.push_back(Entry(FastNlMeansOperator::create(5), "nlmean 5"));
  filter.push_back(Entry(FastNlMeansOperator::create(10), "nlmean 10"));
  filter.push_back(Entry(FastNlMeansOperator::create(15), "nlmean 15"));
  filter.push_back(Entry(FastNlMeansOperator::create(20), "nlmean 20"));

  size_t rows = filter.size() + 1;
  size_t cols = 11;
  std::vector<std::vector<std::string>> table;
  table.resize(rows);
  for_each(table.begin(), table.end(), [&](std::vector<std::string>& col) { col.resize(cols); });

  table[0][0] = "Method";
  table[0][1] = "n10 error";
  table[0][2] = "n20 error";
  table[0][3] = "n30 error";
  table[0][4] = "n40 error";
  table[0][5] = "n50 error";
  table[0][6] = "n10 time";
  table[0][7] = "n20 time";
  table[0][8] = "n30 time";
  table[0][9] = "n40 time";
  table[0][10] = "n50 time";

  size_t row = 1;
  for_each(filter.begin(), filter.end(), [&](Entry& e) { table[row++][0] = e.name; });

  for (size_t col = 1; col != 6; ++col) {
    size_t row = 1;
    for_each(filter.begin(), filter.end(), [&](Entry& e) {
      double time;
      table[row][col] = utility::format("%.3f", processError(e, path, static_cast<int>(10 * col), time));
      table[row++][col + 5] = utility::format("%.3f", time);
    });
  }

  std::ofstream ofs;
  ofs.open("image_denoise.csv");

  for_each(table.begin(), table.end(), [&](const std::vector<std::string>& col) {
    for_each(col.begin(), col.end(), [&](const std::string& cell) {
      std::cout << cell << "\t";
      ofs << cell << ";";
    });
    std::cout << std::endl;
    ofs << std::endl;
  });

  ofs.close();

  return 0;
}
