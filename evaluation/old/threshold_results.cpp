#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <imgproc/derivative_gradient.hpp>
#include <imgproc/threshold.hpp>
#include <edge/nms.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>  
#include <boost/format.hpp>

using namespace lsfm;
using namespace std;
namespace fs = boost::filesystem;

constexpr int runs = 1;

typedef double FT;

struct Entry {
    Entry() {}

    Entry(const cv::Ptr<FilterI<uchar>>& a, const cv::Ptr<Threshold<FT>> &t, const std::string& b, int f = 0)
        : filter(a), threshold(t), name(b), flags(f), images(0), time(0) {}
   
    
    cv::Ptr<FilterI<uchar>> filter;
    cv::Ptr<Threshold<FT>> threshold;
    std::string name;
    int flags, images;
    int64 time;


    inline FilterResults processFilter(const cv::Mat src) {
        filter->process(src);
        return filter->results();
    }

    inline cv::Mat processThreshold(const cv::Mat src) {
        int64 start = cv::getTickCount();
        cv::Mat ret = threshold->process(src);
        time += cv::getTickCount() - start;
        ++images;
        return ret;
    }
};

cv::Mat createNMS(const cv::Mat &emap) {

    cv::Mat emapImg;
    emapImg.create(emap.rows, emap.cols, CV_8UC3);

    emapImg.setTo(cv::Vec3b(0, 0, 0));
    emapImg.setTo(cv::Vec3b(220, 150, 255), emap == 7); // magenta2
    emapImg.setTo(cv::Vec3b(255, 0, 150), emap == 6); // lila
    emapImg.setTo(cv::Vec3b(255, 0, 0), emap == 5); // blue
    emapImg.setTo(cv::Vec3b(255, 255, 0), emap == 4); // cyan
    emapImg.setTo(cv::Vec3b(0, 255, 0), emap == 3); // green
    emapImg.setTo(cv::Vec3b(0, 255, 255), emap == 2); // yellow
    emapImg.setTo(cv::Vec3b(0, 150, 255), emap == 1); // orange
    emapImg.setTo(cv::Vec3b(0, 0, 255), emap == 0); // red
    return emapImg;
}

inline cv::Mat normalize(const cv::Mat &in) {
    cv::Mat cpy;
    if (in.type() != CV_32F || in.type() != CV_64F)
        in.convertTo(cpy, CV_32F);
    else
        in.copyTo(cpy);

    double vmin, vmax;
    cv::minMaxIdx(cpy, &vmin, &vmax);
    cpy *= 255.0 / vmax;
    cpy.convertTo(cpy, CV_8U);
    return cpy;
}

void saveEdge(const cv::Mat &data, const std::string &name) {
    cv::imwrite("./threshold/" + name + ".png", createNMS(data));
}

void saveTh(const cv::Mat &data, const std::string &name) {
    cv::imwrite("./threshold/" + name + ".png", normalize(data));
}

void saveResults(FilterResults &f, const cv::Mat& t, const std::string &name) {
    NonMaximaSuppression<short, FT, FT> nms;
    cv::Mat high = t.clone();
    cv::GaussianBlur(high, high, cv::Size(0, 0),15);
    cv::Mat low = high * 0.5;
    // set fixed lower threshold
    low.setTo(f["mag"].range.upper * 0.004, low < f["mag"].range.upper * 0.004);
    nms.process(f["gx"].data, f["gy"].data, f["mag"].data, low, high);
    saveEdge(nms.hysteresis(), name + "_mag");  
    saveTh(high, name + "_th");
}

void parseFolder(const fs::path &folder, std::vector<fs::path> &files) {
    fs::directory_iterator end_iter;
    for_each(fs::directory_iterator(folder), fs::directory_iterator(), [&files](const fs::path& file) {
        if (fs::is_regular_file(file))
        {
            std::string ext = file.extension().generic_string();
            boost::algorithm::to_lower(ext);
            if (ext == ".jpg" || ext == ".png") {
                files.push_back(file);
            }
        }
        if (fs::is_directory(file))
            parseFolder(file, files);
    });
}

void processPath(std::vector<Entry> &entries, const std::pair<fs::path, std::string> &path) {
    std::cout << "processing " << path.first << std::endl;
    std::vector<fs::path> files;
    parseFolder(path.first, files);

    std::for_each(files.begin(), files.end(), [&](const fs::path& file) {
        cv::Mat src = cv::imread(file.generic_string());
        if (src.empty())
        {
            cout << "Can not open " << file.generic_string() << endl;
            return;
        }
        std::cout << file << std::endl;
        cv::cvtColor(src, src, CV_BGR2GRAY);
        cv::GaussianBlur(src, src, cv::Size(3, 3), 0.6);

        for_each(entries.begin(), entries.end(), [&](Entry &e) {
            FilterResults fs = e.processFilter(src);
            saveResults(fs, e.processThreshold(fs["mag"].data), path.second + "/" + file.stem().generic_string() + "_" + e.name);
        });
    });
}

int main(int argc, char** argv)
{  
    std::vector<std::pair<fs::path, std::string>> sets;
    //sets.push_back(std::pair<fs::path, std::string>("../../images/noise", "noise"));
    sets.push_back(std::pair<fs::path, std::string>("../../images/Selection", "Selection"));
    //sets.push_back(std::pair<fs::path, std::string>("../../images/BSDS500", "BSDS500"));
    //sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-Q", "MDB-Q"));
    //sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-H", "MDB-H"));
    //sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-F", "MDB-F"));

    std::vector<Entry> filter;
    filter.push_back(Entry(new DerivativeGradient<uchar, short,FT, FT, SobelDerivative>, new GlobalThreshold<FT,ThresholdOtsu<FT,256>>(1141), "Otsu_G"));
    filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>, new LocalThresholdTiles<FT, ThresholdOtsu<FT, 256>>(3, 3, 1141), "Otsu_LTiles4"));
    filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>, new LocalThresholdTiles<FT, ThresholdOtsu<FT, 256>>(10, 10, 1141), "Otsu_LTiles10"));
    filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>, new LocalThreshold<FT, ThresholdOtsu<FT, 256>>(30,30,true,1141), "Otsu_LWindow"));
    filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, SobelDerivative>, new DynamicThreshold<FT, ThresholdOtsu<FT, 256>>(5, 1141), "Otsu_D"));

    fs::create_directory("./threshold");
    for_each(sets.begin(), sets.end(), [&](const std::pair<fs::path, std::string> &data) {
        fs::create_directory("./threshold/" + data.second);
        processPath(filter, data);
    });

    for_each(filter.begin(), filter.end(), [&](Entry &e) {
        std::cout << e.name << ": " << e.time * 1000 / (e.images * cv::getTickFrequency()) << "ms" << std::endl;
    });

    return 0;
}

