#include <edge/edge_linking.hpp>
#include <edge/fit.hpp>
#include <edge/nms.hpp>
#include <edge/spe.hpp>
#include <edge/split.hpp>
#include <geometry/draw.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>


using namespace lsfm;
using namespace std;
namespace fs = std::filesystem;

constexpr int runs = 10;

float th_low = 0.004f, th_high = 0.012f;
typedef DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> Grad;
typedef NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> NMS;
typedef EsdLinking<int> Edge;
typedef std::vector<cv::Point> PointVector;
Grad sobel;
NMS nms(th_low, th_high);
Edge edge(10, 3, 3, static_cast<float>(sobel.magnitudeThreshold(th_low)));

struct EntryBase {
  EntryBase(const std::string& n) : name(n), time(0), images(0) {}

  std::string name;
  int64 time;
  int images;

  virtual void process(const EdgeSegmentVector& segs, const PointVector& points) = 0;
};

template <class FIT>
struct Entry : public EntryBase {
  Entry() {}

  Entry(const std::string& n, ValueManager::InitializerList list = ValueManager::InitializerList())
      : EntryBase(n), fit(list) {}


  FIT fit;

  void process(const EdgeSegmentVector& segs, const PointVector& points) {
    std::vector<LineSegment<float, Vec2>> lsegs;
    int64 start;
    for (int i = 0; i != runs; ++i) {
      start = cv::getTickCount();
      fit.apply(segs, points, lsegs);
      time += cv::getTickCount() - start;
      ++images;
    }
  }
};


typedef std::shared_ptr<EntryBase> EntryPtr;

typedef std::vector<EntryPtr> EntryVector;

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


void processPath(EntryVector& entries, const std::pair<fs::path, std::string>& path) {
  std::cout << "processing " << path.first << std::endl;
  std::vector<fs::path> files;
  parseFolder(path.first, files);

  for_each(entries.begin(), entries.end(), [&](EntryPtr& e) {
    e->time = 0;
    e->images = 0;
  });

  std::for_each(files.begin(), files.end(), [&](const fs::path& file) {
    cv::Mat src = cv::imread(file.generic_string(), cv::IMREAD_GRAYSCALE);
    if (src.empty()) {
      cout << "Can not open " << file.generic_string() << endl;
    }
    std::cout << file << std::endl;

    sobel.process(src);
    nms.process(sobel);
    edge.detect(sobel, nms);
    PointVector points;
    PixelEstimator<float, cv::Point>::convert(edge.points(), points, sobel.magnitude(), nms.directionMap());

    for_each(entries.begin(), entries.end(), [&](EntryPtr& e) { e->process(edge.segments(), points); });
  });
}

int main() {
  char c;
  std::cin >> c;

  std::vector<std::pair<fs::path, std::string>> sets;
  // sets.push_back(std::pair<fs::path, std::string>("../../images/Selection", "Selection"));
  // sets.push_back(std::pair<fs::path, std::string>("../../images/BSDS500", "BSDS500"));
  sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-Q", "MDB-Q"));
  // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-H", "MDB-H"));
  // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-F", "MDB-F"));

  EntryVector fit;
  fit.push_back(EntryPtr(new Entry<FitLine<RegressionFit<float, cv::Point>>>("RegressionFit")));
  fit.push_back(EntryPtr(new Entry<FitLine<EigenFit<float, cv::Point>>>("EigenFit")));
  fit.push_back(EntryPtr(new Entry<FitLine<EigenCVFit<float, cv::Point>>>("EigenCVFit")));
  fit.push_back(EntryPtr(new Entry<MEstimatorFitLine<float, cv::Point>>("MEstimatorFit")));


  int rows = static_cast<int>(fit.size()) + 1;
  int cols = static_cast<int>(sets.size()) + 1;
  std::vector<std::vector<std::string>> table;
  table.resize(static_cast<size_t>(rows));
  for_each(table.begin(), table.end(), [&](std::vector<std::string>& row) { row.resize(static_cast<size_t>(cols)); });

  table[0][0] = "Method";

  int row = 1;
  for_each(fit.begin(), fit.end(), [&](const EntryPtr& e) { table[static_cast<size_t>(row++)][0] = e->name; });

  int col = 1;
  for_each(sets.begin(), sets.end(), [&](const std::pair<fs::path, std::string>& data) {
    processPath(fit, data);

    table[0][static_cast<size_t>(col)] = data.second;
    row = 1;
    for_each(fit.begin(), fit.end(), [&](const EntryPtr& e) {
      std::ostringstream oss;
      oss.setf(std::ios::fixed);
      oss << std::setprecision(3) << (static_cast<double>(e->time * 1000) / (e->images * cv::getTickFrequency()));
      table[static_cast<size_t>(row++)][static_cast<size_t>(col)] = oss.str() + "ms";
    });
    ++col;
  });

  std::ofstream ofs;
  ofs.open("line_fit_profiling.csv");

  for_each(table.begin(), table.end(), [&](const std::vector<std::string>& row) {
    for_each(row.begin(), row.end(), [&](const std::string& cell) {
      std::cout << cell << "\t";
      ofs << cell << ";";
    });
    std::cout << std::endl;
    ofs << std::endl;
  });

  ofs.close();

  return 0;
}
