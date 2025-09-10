#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define USE_PERIODIC_FFT
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/susan.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/pc_lgf.hpp>
#include <imgproc/pc_matlab.hpp>
#include <imgproc/laplace.hpp>
#include <imgproc/image_operator.hpp>

#include <filesystem>
#include <algorithm>  
#include <boost/format.hpp>

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
    Entry() {}

    Entry(const cv::Ptr<FilterI<uchar>>& a, const std::string& b, const std::string& w, int f = 0)
        : filter(a), name(b), what(w), flags(f) {}
   
    
    cv::Ptr<FilterI<uchar>> filter;
    std::string name, what;
    int flags;

    Value value(const std::string &name) {
        return dynamic_cast<ValueManager*>(&(*filter))->value(name);
    }

    void value(const std::string &name, const Value &v) {
        dynamic_cast<ValueManager*>(&(*filter))->value(name, v);
    }


    inline bool rgb() const {
        return flags & ENTRY_RGB;
    }

    inline bool sqr() const {
        return flags & ENTRY_SQR;
    }

    inline int border() const {
        if (flags & ENTRY_NO_3)
            return 2;
        if (flags & ENTRY_NO_5)
            return 3;
        return 0;
    }

    inline cv::Mat process(const cv::Mat src) {
        filter->process(src);
        return filter->results()[what].data.clone();
    }
};

void parseFolder(const fs::path &folder, std::vector<fs::path> &files) {
    fs::directory_iterator end_iter;
    for_each(fs::directory_iterator(folder), fs::directory_iterator(), [&files](const fs::path& file) {
        if (fs::is_regular_file(file))
        {
            std::string ext = file.extension().generic_string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return std::tolower(c); });
            if (ext == ".jpg" || ext == ".png") {
                files.push_back(file);
            }
        }
        if (fs::is_directory(file))
            parseFolder(file, files);
    });
}

double processError(Entry &e, const fs::path& path, int n) {
    std::cout << "processing noise" << n << " (" << e.name <<")...";
    std::vector<fs::path> files;
    parseFolder(path, files);
    GaussianNoiseOperator op_noise(n);

    double ret = 0;
    int count = 0;
    std::for_each(files.begin(), files.end(), [&](const fs::path& file) {
        cv::Mat rgb = cv::imread(file.generic_string());
        if (rgb.empty())
        {
            cout << "Can not open " << file.generic_string() << endl;
            return;
        }
        
        cv::Mat src_n, src = rgb, res, res_n;
        if (!e.rgb()) cv::cvtColor(rgb, src, cv::COLOR_BGR2GRAY);
        src_n = src.clone();
        op_noise(src_n);

        res = e.process(src);
        res_n = e.process(src_n);

        if (res.type() != cv::DataType<FT>::type) {
            res.convertTo(res, cv::DataType<FT>::type);
            res_n.convertTo(res_n, cv::DataType<FT>::type);
        }

        double vmin, vmax;
        cv::minMaxIdx(res, &vmin, &vmax);
        res /= vmax;
        res_n /= vmax;

        ret += sum(abs(res - res_n))[0] / res.size().area();
        ++count;
    });
    ret /= count;
    std::cout << ret << std::endl;
    return ret;
}

int main(int argc, char** argv) {
  fs::path path = "../../images/MDB/MiddEval3-Q";

  std::vector<Entry> filter;
  /*filter.push_back(Entry(new DerivativeGradient<uchar,short, FT, FT, RobertsDerivative<uchar,short>>,"Roberts (2x2)",
  "mag")); filter.push_back(Entry(new DerivativeGradient<uchar, short,FT, FT, PrewittDerivative<uchar, short>>, "Prewitt
  (3x3)", "mag")); filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, ScharrDerivative<uchar, short>>,
  "Scharr (3x3)", "mag"));

  filter.push_back(Entry(new DerivativeGradient<uchar, short,FT, FT, SobelDerivative<uchar, short>>, "Sobel (3x3)",
  "mag")); filter.push_back(Entry(new DerivativeGradient<uchar, short,FT, FT, SobelDerivative<uchar, short>>({
  NV("grad_kernel_size",5) }), "Sobel (5x5)", "mag")); filter.push_back(Entry(new DerivativeGradient<uchar, short,FT,
  FT, SobelDerivative<uchar, short>>({ NV("grad_kernel_size",7) }), "Sobel (7x7)", "mag")); filter.push_back(Entry(new
  DerivativeGradient<uchar, short,FT, FT, SobelDerivative<uchar, short>>({ NV("grad_kernel_size",9) }), "Sobel (9x9)",
  "mag"));*/

  /*filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative<uchar, FT>, Magnitude<FT, FT>>({
  NV("grad_kernel_size",3), NV("grad_range",1.5) }), "Gauss (3x3)", "mag")); filter.push_back(Entry(new
  DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative<uchar, FT>, Magnitude<FT, FT>>({ NV("grad_kernel_size",5),
  NV("grad_range",2.3) }), "Gauss (5x5)", "mag")); filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT,
  GaussianDerivative<uchar, FT>, Magnitude<FT, FT>>({ NV("grad_kernel_size",7), NV("grad_range",3.0) }), "Gauss (7x7)",
  "mag")); filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative<uchar, FT>, Magnitude<FT,
  FT>>({ NV("grad_kernel_size",9), NV("grad_range",3.5) }), "Gauss (9x9)", "mag"));*/

  /*filter.push_back(Entry(new SusanGradient<short, int>(20), "Susan (37)", "mag"));
  filter.push_back(Entry(new SusanGradient<short, int>(20,true), "Susan (3x3)", "mag"));
  filter.push_back(Entry(new RCMGradient<uchar, 1, short, int>(3,1), "RMG (3x3)", "mag", ENTRY_NO_3));
  filter.push_back(Entry(new RCMGradient<uchar, 1, short, int>(5, 3), "RMG (5x5)", "mag", ENTRY_NO_5));
  filter.push_back(Entry(new RCMGradient<uchar, 3, short, int>(3, 1), "RCMG (3x3)", "mag", ENTRY_NO_3 | ENTRY_SQR |
  ENTRY_RGB));*/

  /*filter.push_back(Entry(new LaplaceSimple<uchar, short>, "Laplace (3x3)", "laplace"));
  filter.push_back(Entry(new LaplaceCV<uchar, short>(5), "Laplace (5x5)", "laplace"));
  filter.push_back(Entry(new LaplaceCV<uchar, short>(7), "Laplace (7x7)", "laplace"));
  filter.push_back(Entry(new LaplaceCV<uchar, short>(9), "Laplace (9x9)", "laplace"));*/

  /*filter.push_back(Entry(new LoG<uchar, FT>(3, 1.240080), "LoG (3x3)", "laplace"));
  filter.push_back(Entry(new LoG<uchar, FT>(5, 1.008030), "LoG (5x5)", "laplace"));
  filter.push_back(Entry(new LoG<uchar, FT>(7, 0.873228), "LoG (7x7)", "laplace"));
  filter.push_back(Entry(new LoG<uchar, FT>(9, 0.781859), "LoG (9x9)", "laplace"));*/

  /*filter.push_back(Entry(new QuadratureG2<uchar, FT>(3, 1.24008), "QF StG (3x3)", "energy"));
  filter.push_back(Entry(new QuadratureG2<uchar, FT>(5, 1.008),   "QF StG (5x5)", "energy"));
  filter.push_back(Entry(new QuadratureG2<uchar, FT>(7, 0.873226), "QF StG (7x7)", "energy"));
  filter.push_back(Entry(new QuadratureG2<uchar, FT>(9, 0.781854), "QF StG (9x9)", "energy"));

  filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 3, 1.2)), "SQF
  Po (3x3)", "energy")); filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT,
  FT>(1, 2, 5, 1.2)), "SQF Po (5x5)", "energy")); filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT,
  FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 7, 1.2)), "SQF Po (7x7)", "energy"));
  filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 9, 1.2)), "SQF
  Po (9x9)", "energy"));


  filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSqf<uchar, FT>(1, 3, 2.5)), "SQFF Po",
  "energy")); filter.push_back(Entry(new QuadratureLGF<uchar, FT>(5, 0.55), "SQFF Lg", "energy"));

  filter.push_back(Entry(new PCLgf<uchar, FT>(4, 3,2.1,0.55), "PCF Lg", "pc"));
  filter.push_back(Entry(new PCMatlab<uchar>(4, 3, 2.1, 0.55), "PCF Ml", "pc"));
  filter.push_back(Entry(new PCSqf<uchar, FT>(1,3,2.5), "PCF Po", "pc"));*/

  filter.push_back(
      Entry(dynamic_cast<LaplaceI<uchar, FT>*>(new PCLSqf<uchar, FT>(1.0, 2.0, 1.2)), "Laplace SQFF", "pclmag"));

  int rows = filter.size() + 1;
  int cols = 6;
  std::vector<std::vector<std::string>> table;
  table.resize(rows);
  for_each(table.begin(), table.end(), [&](std::vector<std::string>& col) { col.resize(cols); });

  table[0][0] = "Method";
  table[0][1] = "noise 10";
  table[0][2] = "noise 20";
  table[0][3] = "noise 30";
  table[0][4] = "noise 40";
  table[0][5] = "noise 50";

  int row = 1;
  for_each(filter.begin(), filter.end(), [&](Entry& e) { table[row++][0] = e.name; });

  for (int col = 1; col != 6; ++col) {
    int row = 1;
    for_each(filter.begin(), filter.end(), [&](Entry& e) {
      table[row++][col] = boost::str(boost::format("%.3f") % (processError(e, path, 10 * col)));
    });
  }

    std::ofstream ofs;
    ofs.open("noise_error.csv");

    for_each(table.begin(), table.end(), [&](const std::vector<std::string> &col) {
        for_each(col.begin(), col.end(), [&](const std::string &cell) {
            std::cout << cell << "\t";
            ofs << cell << ";";
        });
        std::cout << std::endl;
        ofs << std::endl;
    });

    ofs.close();

    return 0;
}
