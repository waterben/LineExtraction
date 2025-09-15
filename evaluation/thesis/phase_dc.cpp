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
#include <utility/matlab_helpers.hpp>

#include <filesystem>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <iomanip>

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

    Entry(const cv::Ptr<QuadratureI<uchar, double, double, double, double>>& fi, const cv::Ptr<QuadratureI<uchar, double, double, double, double>>& g, const cv::Ptr<QuadratureI<uchar, double, double, double, double>>& f, const std::string& b)
        : filter(fi), gt(g), ft(f), name(b) {}
   
    
    cv::Ptr<QuadratureI<uchar, double, double, double, double>> filter, gt, ft;
    std::string name;

    inline double phaseError(const cv::Mat src) {
        filter->process(src);
        ft->process(src);
        cv::Mat p = filter->phase().clone();
        p.setTo(0, ft->energy() < ft->energyThreshold(0.05));
        //showMat("phase org", abs(p));
        gt->process(src);
        cv::Mat pgt = gt->phase().clone();
        pgt.setTo(0, ft->energy() < ft->energyThreshold(0.05));
        //showMat("phase diff", abs(p - pgt));
        //cvWaitKey();
        return sum(abs(p - pgt))[0] / p.size().area();
    }

};

void parseFolder(const fs::path &folder, std::vector<fs::path> &files) {
    fs::directory_iterator end_iter;
    for_each(fs::directory_iterator(folder), fs::directory_iterator(), [&files](const fs::path& file) {
        if (fs::is_regular_file(file))
        {
            std::string ext = file.extension().generic_string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch){
                return static_cast<char>(std::tolower(ch));
            });
            if (ext == ".jpg" || ext == ".png") {
                files.push_back(file);
            }
        }
        if (fs::is_directory(file))
            parseFolder(file, files);
    });
}

double processError(Entry &e, const fs::path& path) {
    std::cout << "processing " << e.name << "...";
    std::vector<fs::path> files;
    parseFolder(path, files);

    double ret = 0;
    int count = 0;
    std::for_each(files.begin(), files.end(), [&](const fs::path& file) {
        
        cv::Mat src = cv::imread(file.generic_string());
        cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
        if (src.empty())
        {
            cout << "Can not open " << file.generic_string() << endl;
            return;
        }
        
        ret += e.phaseError(src);
        ++count;
        
    });
    ret /= count;
    std::cout << ret << std::endl;
    return ret;
}

int main(int argc, char** argv) {
  fs::path path = "../../images/MDB/MiddEval3-Q";

  cv::Ptr<QuadratureI<uchar, double, double, double, double>> gt(new QuadratureSF<uchar, FT>(1, 2, 1.2));
  cv::Ptr<QuadratureI<uchar, double, double, double, double>> ft(new QuadratureS<uchar, FT, FT>(1, 2, 3, 1.2));

  std::vector<Entry> filter;

  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 3, 1.2), gt, ft, "SQF Po (3x3)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 5, 1.2), gt, ft, "SQF Po (5x5)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 7, 1.2), gt, ft, "SQF Po (7x7)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 9, 1.2), gt, ft, "SQF Po (9x9)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 11, 1.2), gt, ft, "SQF Po (11x11)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 13, 1.2), gt, ft, "SQF Po (13x13)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 15, 1.2), gt, ft, "SQF Po (15x15)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 17, 1.2), gt, ft, "SQF Po (17x17)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 19, 1.2), gt, ft, "SQF Po (19x19)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 31, 1.2), gt, ft, "SQF Po (31x31)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 63, 1.2), gt, ft, "SQF Po (63x63)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 127, 1.2), gt, ft, "SQF Po (127x127)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 255, 1.2), gt, ft, "SQF Po (255x255)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 511, 1.2), gt, ft, "SQF Po (511x511)"));
  filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 749, 1.2), gt, ft, "SQF Po (749x749)"));


  int rows = filter.size() + 1;
  int cols = 2;
  std::vector<std::vector<std::string>> table;
  table.resize(rows);
  for_each(table.begin(), table.end(), [&](std::vector<std::string>& col) { col.resize(cols); });

  table[0][0] = "Method";
  table[0][1] = "Error";

  int row = 1;
  for_each(filter.begin(), filter.end(), [&](Entry& e) { table[row++][0] = e.name; });

  row = 1;
  for_each(filter.begin(), filter.end(), [&](Entry& e) {
      std::ostringstream oss;
      oss.setf(std::ios::fixed);
      oss << std::setprecision(3) << processError(e, path);
      table[row++][1] = oss.str();
  });

  std::ofstream ofs;
  ofs.open("phase_error.csv");

  for_each(
      table.begin(), table.end(), [&](const std::vector<std::string>& col) {
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
