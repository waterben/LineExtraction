#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <iostream>


#define USE_PERIODIC_FFT
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/laplace.hpp>
#include <imgproc/pc_lgf.hpp>
#include <imgproc/pc_matlab.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/susan.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>


using namespace lsfm;
using namespace std;
namespace fs = std::filesystem;

constexpr int runs = 1;

constexpr int ENTRY_SQR = 1;
constexpr int ENTRY_RGB = 2;
constexpr int ENTRY_NO_3 = 4;
constexpr int ENTRY_NO_5 = 8;

typedef double FT;

constexpr FT mag_th = static_cast<FT>(0.05);

struct Entry {
  Entry() : filter(), name(), flags(0) {}

  Entry(const cv::Ptr<FilterI<uchar>>& a, const std::string& b, int f = 0) : filter(a), name(b), flags(f) {}


  cv::Ptr<FilterI<uchar>> filter;
  std::string name;
  int flags;

  Value value(const std::string& param_name) { return dynamic_cast<ValueManager*>(&(*filter))->value(param_name); }

  void value(const std::string& param_name, const Value& v) {
    dynamic_cast<ValueManager*>(&(*filter))->value(param_name, v);
  }


  inline bool rgb() const { return flags & ENTRY_RGB; }

  inline bool sqr() const { return flags & ENTRY_SQR; }

  inline int border() const {
    if (flags & ENTRY_NO_3) return 2;
    if (flags & ENTRY_NO_5) return 3;
    return 0;
  }

  inline lsfm::FilterResults process(const cv::Mat src) {
    filter->process(src);
    return filter->results();
  }
};

inline cv::Vec3b dirColor(FT val) {
  static FT r = static_cast<FT>(CV_PI / 4);
  static cv::Vec3b colors[] = {cv::Vec3b(255, 0, 150), cv::Vec3b(255, 0, 255), cv::Vec3b(0, 0, 255),
                               cv::Vec3b(0, 150, 255), cv::Vec3b(0, 255, 255), cv::Vec3b(0, 255, 0),
                               cv::Vec3b(255, 255, 0), cv::Vec3b(255, 0, 0),   cv::Vec3b(255, 0, 150)};
  FT div = val / r;
  int divi = static_cast<int>(div);
  cv::Vec<FT, 3> a = colors[divi + 4], b = colors[divi + 4 + (val < 0 ? -1 : 1)];
  /*cv::Vec<FT, 3> c = b - a;
  FT mul = abs(div) - abs(divi);
  cv::Vec<FT, 3> d = c * mul;
  cv::Vec3b res = a + d;
  return res;*/
  return a + (b - a) * (abs(div) - abs(divi));
}

void saveDir(cv::Mat& data, const std::string& name, const cv::Mat& mask = cv::Mat()) {
  cv::Mat tmp(data.size(), CV_8UC3);
  cv::Vec3b* pdst = tmp.ptr<cv::Vec3b>();
  if (data.isContinuous() && data.isContinuous()) {
    const FT* pd = data.ptr<FT>();
    int size = data.rows * data.cols;
    for (int j = 0; j != size; ++j) {
      pdst[j] = dirColor(pd[j]);
    }
  } else {
    for (int i = 0; i != data.rows; ++i, pdst += data.cols) {
      const FT* pd = data.ptr<FT>(i);
      for (int j = 0; j != data.cols; ++j) {
        pdst[j] = dirColor(pd[j]);
      }
    }
  }
  if (!mask.empty()) tmp.setTo(cv::Scalar(0, 0, 0), mask < mag_th);
  cv::imwrite("./grad/" + name + ".png", tmp);
}

void savePhase(cv::Mat& data, const std::string& name, const cv::Mat& energy) {
  double vmin, vmax;
  cv::minMaxIdx(energy, &vmin, &vmax);
  cv::Mat en = energy / vmax;

  cv::Mat tmp(data.size(), CV_8UC3);
  cv::Vec3b* pdst = tmp.ptr<cv::Vec3b>();
  if (data.isContinuous() && en.isContinuous()) {
    const FT* pd = data.ptr<FT>();
    const FT* pe = en.ptr<FT>();
    int size = data.rows * data.cols;
    for (int j = 0; j != size; ++j) {
      pdst[j] = dirColor(pd[j]) * pe[j];
    }
  } else {
    for (int i = 0; i != data.rows; ++i, pdst += data.cols) {
      const FT* pd = data.ptr<FT>(i);
      const FT* pe = en.ptr<FT>(i);
      for (int j = 0; j != data.cols; ++j) {
        pdst[j] = dirColor(pd[j]) * pe[j];
      }
    }
  }
  cv::imwrite("./grad/" + name + ".png", tmp);
}

void saveNormalized(cv::Mat& data, const std::string& name) {
  double vmin, vmax;
  cv::minMaxIdx(data, &vmin, &vmax);
  if (vmin < 0) {
    vmax = std::max(vmax, -vmin);
    cv::Mat tmp = cv::abs(data);
    tmp *= 255.0 / vmax;
    tmp.convertTo(tmp, CV_8U);
    cv::imwrite("./grad/" + name + "_abs.png", tmp);
    data += vmax;
    data *= 127.5 / vmax;
  } else
    data *= 255.0 / vmax;
  data.convertTo(data, CV_8U);
  // std::cout << "write: " << "./grad/" + name + ".png" << std::endl;
  cv::imwrite("./grad/" + name + ".png", data);
}

void saveResults(const lsfm::FilterResults& results, const std::string& name, bool sqr = false, int border = 0) {
  for_each(results.begin(), results.end(), [&](const FilterResult& r) {
    cv::Mat tmp;
    if (r.second.data.type() != cv::DataType<FT>::type)
      r.second.data.convertTo(tmp, cv::DataType<FT>::type);
    else
      r.second.data.copyTo(tmp);

    if (r.first == "phase") {
      lsfm::FilterResults::const_iterator f = results.find("energy");
      if (f != results.end())
        savePhase(tmp, name + "_" + r.first, f->second.data);
      else
        saveDir(tmp, name + "_" + r.first);
      return;
    }
    if (r.first == "dir") {
      lsfm::FilterResults::const_iterator f = results.find("odd");
      if (f == results.end()) f = results.find("mag");
      if (f != results.end())
        saveDir(tmp, name + "_" + r.first, f->second.data);
      else
        saveDir(tmp, name + "_" + r.first);
      return;
    }
    if (r.first == "mag" && sqr) cv::sqrt(tmp, tmp);

    if (border > 0) {
      tmp.colRange(0, border).setTo(0);
      tmp.colRange(tmp.cols - border, tmp.cols).setTo(0);
      tmp.rowRange(0, border).setTo(0);
      tmp.rowRange(tmp.rows - border, tmp.rows).setTo(0);
    }

    saveNormalized(tmp, name + "_" + r.first);
  });
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

void processPath(std::vector<Entry>& entries, const std::pair<fs::path, std::string>& path) {
  std::cout << "processing " << path.first << std::endl;
  std::vector<fs::path> files;
  parseFolder(path.first, files);

  std::for_each(files.begin(), files.end(), [&](const fs::path& file) {
    cv::Mat rgb = cv::imread(file.generic_string());
    if (rgb.empty()) {
      cout << "Can not open " << file.generic_string() << endl;
      return;
    }
    std::cout << file << std::endl;
    cv::Mat src;
    cv::cvtColor(rgb, src, cv::COLOR_BGR2GRAY);

    for_each(entries.begin(), entries.end(), [&](Entry& e) {
      saveResults(e.process(e.rgb() ? rgb : src), path.second + "/" + file.stem().generic_string() + "_" + e.name,
                  e.sqr(), e.border());
    });
  });
}

int main(int /*argc*/, char** /*argv*/) {
  std::vector<std::pair<fs::path, std::string>> sets;
  // sets.push_back(std::pair<fs::path, std::string>("../../images/noise", "noise"));
  sets.push_back(std::pair<fs::path, std::string>("../../images/Selection", "Selection"));
  // sets.push_back(std::pair<fs::path, std::string>("../../images/BSDS500", "BSDS500"));
  // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-Q", "MDB-Q"));
  // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-H", "MDB-H"));
  // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-F", "MDB-F"));

  std::vector<Entry> filter;
  filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, RobertsDerivative>, "Roberts_(2x2)"));
  filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, PrewittDerivative>, "Prewitt_(3x3)"));
  filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, ScharrDerivative>, "Scharr_(3x3)"));

  filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>, "Sobel_(3x3)"));
  filter.push_back(
      Entry(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>({NV("grad_kernel_size", 5)}), "Sobel_(5x5)"));
  filter.push_back(
      Entry(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>({NV("grad_kernel_size", 7)}), "Sobel_(7x7)"));
  filter.push_back(
      Entry(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>({NV("grad_kernel_size", 9)}), "Sobel_(9x9)"));

  filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>(
                             {NV("grad_kernel_size", 3), NV("grad_range", 1.5)}),
                         "Gauss_(3x3)"));
  filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>(
                             {NV("grad_kernel_size", 5), NV("grad_range", 2.3)}),
                         "Gauss_(5x5)"));
  filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>(
                             {NV("grad_kernel_size", 7), NV("grad_range", 3.0)}),
                         "Gauss_(7x7)"));
  filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>(
                             {NV("grad_kernel_size", 9), NV("grad_range", 3.5)}),
                         "Gauss_(9x9)"));

  filter.push_back(Entry(new SusanGradient<short, int>(20), "Susan_(37)"));
  filter.push_back(Entry(new SusanGradient<short, int>(20, true), "Susan_(3x3)"));
  filter.push_back(Entry(new RCMGradient<uchar, 1, short, int>(3, 1), "RMG_(3x3)", ENTRY_NO_3));
  filter.push_back(Entry(new RCMGradient<uchar, 1, short, int>(5, 3), "RMG_(5x5)", ENTRY_NO_5));
  filter.push_back(
      Entry(new RCMGradient<uchar, 3, short, int>(3, 1), "RCMG_(3x3)", ENTRY_NO_3 | ENTRY_SQR | ENTRY_RGB));

  filter.push_back(Entry(new LaplaceSimple<uchar, short>, "Laplace_(3x3)"));
  filter.push_back(Entry(new LaplaceCV<uchar, short>(5), "Laplace_Sobel_(5x5)"));
  filter.push_back(Entry(new LaplaceCV<uchar, short>(7), "Laplace_Sobel_(7x7)"));
  filter.push_back(Entry(new LaplaceCV<uchar, short>(9), "Laplace_Sobel_(9x9)"));

  filter.push_back(Entry(new LoG<uchar, FT>(3, 1.240080), "LoG_(3x3)"));
  filter.push_back(Entry(new LoG<uchar, FT>(5, 1.008030), "LoG_(5x5)"));
  filter.push_back(Entry(new LoG<uchar, FT>(7, 0.873228), "LoG_(7x7)"));
  filter.push_back(Entry(new LoG<uchar, FT>(9, 0.781859), "LoG_(9x9)"));

  filter.push_back(Entry(new QuadratureG2<uchar, FT>(3, 1.24008), "QF_StG_(3x3)"));
  filter.push_back(Entry(new QuadratureG2<uchar, FT>(5, 1.008), "QF_StG_(5x5)"));
  filter.push_back(Entry(new QuadratureG2<uchar, FT>(7, 0.873226), "QF_StG_(7x7)"));
  filter.push_back(Entry(new QuadratureG2<uchar, FT>(9, 0.781854), "QF_StG_(9x9)"));

  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 3, 1.2), "SQF_Po_(3x3)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 5, 1.2), "SQF_Po_(5x5)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 7, 1.2), "SQF_Po_(7x7)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 9, 1.2), "SQF_Po_(9x9)"));

  filter.push_back(
      Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 3, 1.2)), "SQF_Po_(3x3)"));
  filter.push_back(
      Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 5, 1.2)), "SQF_Po_(5x5)"));
  filter.push_back(
      Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 7, 1.2)), "SQF_Po_(7x7)"));
  filter.push_back(
      Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 9, 1.2)), "SQF_Po_(9x9)"));


  filter.push_back(Entry(new QuadratureSF<uchar, FT>(1, 2, 1.2), "SQFF_Po_12"));
  filter.push_back(
      Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSqf<uchar, FT>(1, 2, 1.2)), "SQFF_Po_12"));
  filter.push_back(
      Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSqf<uchar, FT>(1, 3, 2.5)), "SQFF_Po"));
  filter.push_back(Entry(new QuadratureLGF<uchar, FT>(5, 0.55), "SQFF_Lg"));

  filter.push_back(Entry(new PCLgf<uchar, FT>(4, 3, 2.1, 0.55), "PCF_Lg"));
  filter.push_back(Entry(new PCMatlab<uchar>(4, 3, 2.1, 0.55), "PCF_Ml"));
  filter.push_back(Entry(new PCSqf<uchar, FT>(1, 3, 2.5), "PCF_Po"));

  fs::create_directory("./grad");
  for_each(sets.begin(), sets.end(), [&](const std::pair<fs::path, std::string>& data) {
    fs::create_directory("./grad/" + data.second);
    processPath(filter, data);
  });

  return 0;
}
