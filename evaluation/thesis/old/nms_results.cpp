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
#include <edge/zc.hpp>
#include <edge/nms.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>  
#include <boost/format.hpp>

using namespace lsfm;
using namespace std;
namespace fs = boost::filesystem;

constexpr int runs = 1;

constexpr int ENTRY_SQR = 1;
constexpr int ENTRY_RGB = 2; 
constexpr int ENTRY_NO_3 = 4;
constexpr int ENTRY_NO_5 = 8;

typedef double FT;

struct Entry {
    Entry() {}

    Entry(const cv::Ptr<FilterI<uchar>>& a, const std::string& b, double l, double h, int f = 0)
        : filter(a), name(b), low(l), high(h), flags(f) {}

    Entry(const cv::Ptr<FilterI<uchar>>& a, const std::string& b, int f = 0)
        : filter(a), name(b), low(0.01), high(0.03), flags(f) {}
   
    
    cv::Ptr<FilterI<uchar>> filter;
    std::string name;
    int flags;
    double low, high;

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

    inline lsfm::FilterResults process(const cv::Mat src) {
        filter->process(src);
        return filter->results();
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

void saveEdge(const cv::Mat &data, const std::string &name) {
    cv::imwrite("./nms/" + name + ".png", createNMS(data));
}

void saveResults(const lsfm::FilterResults &results, const std::string &name, double th_low, double th_high, bool sqr = false, int border = 0) {
    ZeroCrossing<FT, FT, FT, FastZC<FT, FT, FT>> zc;
    NonMaximaSuppression<FT, FT, FT, FastNMS8<FT, FT, FT>> nms;
    lsfm::FilterResults::const_iterator f;
    cv::Mat tmp, tmpx, tmpy;
    double low, high;
    std::cout << name << std::endl;
    f = results.find("laplace");
    if (f != results.end() && results.find("even") == results.end()) {
        std::cout << "laplace range upper: " << f->second.range.size() << std::endl;
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmp, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmp);
        zc.process(tmp, f->second.range.size() * th_low, f->second.range.size() * th_high);
        saveEdge(zc.hysteresis(), name + "_laplace");
    }

    f = results.find("mag");
    if (f != results.end()) {
        std::cout << "mag range upper: " << f->second.range.upper << std::endl;
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmp, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmp);
        low = f->second.range.upper * th_low;
        high = f->second.range.upper * th_high;
        f = results.find("gx");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpx, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpx);
        f = results.find("gy");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpy, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpy);
        nms.process(tmpx,tmpy,tmp,low,high);
        saveEdge(nms.hysteresis(), name + "_mag");
    }

    f = results.find("even");
    if (f != results.end()) {
        std::cout << "even range upper: " << f->second.range.upper << std::endl;
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmp, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmp);
        low = f->second.range.size() * th_low;
        high = f->second.range.size() * th_high;
        f = results.find("oddx");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpx, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpx);
        f = results.find("oddy");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpy, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpy);
        zc.process(tmpx, tmpy, tmp, low, high);
        saveEdge(zc.hysteresis(), name + "_even");
    }

    f = results.find("odd");
    if (f != results.end()) {
        std::cout << "odd range upper: " << f->second.range.upper << std::endl;
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmp, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmp);
        low = f->second.range.upper * th_low;
        high = f->second.range.upper * th_high;
        f = results.find("oddx");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpx, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpx);
        f = results.find("oddy");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpy, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpy);
        nms.process(tmpx, tmpy, tmp, low, high);
        saveEdge(nms.hysteresis(), name + "_odd");
    }

    f = results.find("energy");
    if (f != results.end()) {
        std::cout << "energy range upper: " << f->second.range.upper << std::endl;
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmp, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmp);
        low = f->second.range.upper * th_low;
        high = f->second.range.upper * th_high;
        f = results.find("oddx");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpx, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpx);
        f = results.find("oddy");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpy, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpy);
        nms.process(tmpx, tmpy, tmp, low, high);
        saveEdge(nms.hysteresis(), name + "_energy");
    }

    f = results.find("pc");
    if (f != results.end()) {
        std::cout << "pc range upper: " << f->second.range.upper << std::endl;
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmp, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmp);
        low = f->second.range.upper * th_low;
        high = f->second.range.upper * th_high;
        f = results.find("oddx");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpx, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpx);
        f = results.find("oddy");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmpy, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmpy);
        nms.process(tmpx, tmpy, tmp, low, high);
        saveEdge(nms.hysteresis(), name + "_pc");
    }

    f = results.find("pclx");
    if (f != results.end()) {
        std::cout << "pcl range upper: " << f->second.range.upper << std::endl;
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmp, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmp);
        zc.process(tmp, f->second.range.size() * th_low * th_low, f->second.range.size() * th_high * th_high);
        saveEdge(zc.hysteresis(), name + "_pclx");
        f = results.find("pcly");
        if (f->second.data.type() != cv::DataType<FT>::type)
            f->second.data.convertTo(tmp, cv::DataType<FT>::type);
        else
            f->second.data.copyTo(tmp);
        zc.process(tmp, f->second.range.size() * th_low * th_low, f->second.range.size() * th_high * th_high);
        saveEdge(zc.hysteresis(), name + "_pcly");
    }
    
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
        cv::Mat rgb = cv::imread(file.generic_string());
        if (rgb.empty())
        {
            cout << "Can not open " << file.generic_string() << endl;
            return;
        }
        cv::GaussianBlur(rgb, rgb, cv::Size(3, 3), 0.6);
        std::cout << file << std::endl;
        cv::Mat src;
        cv::cvtColor(rgb, src, cv::COLOR_BGR2GRAY);

        for_each(entries.begin(), entries.end(), [&](Entry &e) {
            saveResults(e.process(e.rgb() ? rgb : src), path.second + "/" + file.stem().generic_string() + "_" + e.name,e.low, e.high, e.sqr(),e.border());
        });
    });
}

int main(int argc, char** argv)
{  
    std::vector<std::pair<fs::path, std::string>> sets;
    // sets.push_back(std::pair<fs::path, std::string>("../../images/noise", "noise"));
    sets.push_back(std::pair<fs::path, std::string>("../../images/Selection", "Selection"));
    // sets.push_back(std::pair<fs::path, std::string>("../../images/BSDS500", "BSDS500"));
    // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-Q", "MDB-Q"));
    // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-H", "MDB-H"));
    // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-F", "MDB-F"));

    std::vector<Entry> filter;
    filter.push_back(Entry(new DerivativeGradient<uchar,short, FT, FT, RobertsDerivative>,"Roberts_(2x2)"));
    filter.push_back(Entry(new DerivativeGradient<uchar, short,FT, FT, PrewittDerivative>, "Prewitt_(3x3)"));
    filter.push_back(Entry(new DerivativeGradient<uchar, short, FT, FT, ScharrDerivative>, "Scharr_(3x3)"));

    filter.push_back(Entry(new DerivativeGradient<uchar, short,FT, FT, SobelDerivative>, "Sobel_(3x3)"));
    filter.push_back(Entry(new DerivativeGradient<uchar, short,FT, FT, SobelDerivative>({ NV("grad_kernel_size",5) }), "Sobel_(5x5)"));
    filter.push_back(Entry(new DerivativeGradient<uchar, FT,FT, FT, SobelDerivative>({ NV("grad_kernel_size",7) }), "Sobel_(7x7)"));
    filter.push_back(Entry(new DerivativeGradient<uchar, FT,FT, FT, SobelDerivative>({ NV("grad_kernel_size",9) }), "Sobel_(9x9)"));

    filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",3), NV("grad_range",1.5) }), "Gauss_(3x3)"));
    filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",5), NV("grad_range",2.3) }), "Gauss_(5x5)"));
    filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",7), NV("grad_range",3.0) }), "Gauss_(7x7)"));
    filter.push_back(Entry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",9), NV("grad_range",3.5) }), "Gauss_(9x9)"));
    
    filter.push_back(Entry(new SusanGradient<short, FT,FT>(20), "Susan_(37)"));
    filter.push_back(Entry(new SusanGradient<short, FT,FT>(20,true), "Susan_(3x3)"));
    filter.push_back(Entry(new RCMGradient<uchar, 1, short, FT,FT>(3, 1, cv::NORM_L2), "RMG_(3x3)", ENTRY_NO_3));
    filter.push_back(Entry(new RCMGradient<uchar, 1, short, FT,FT>(5, 3, cv::NORM_L2), "RMG_(5x5)", ENTRY_NO_5));
    filter.push_back(Entry(new RCMGradient<uchar, 3, short, FT,FT>(3, 1, cv::NORM_L2), "RCMG_(3x3)", ENTRY_NO_3 | ENTRY_RGB));

    filter.push_back(Entry(new LaplaceSimple<uchar, short>, "Laplace_(3x3)"));
    filter.push_back(Entry(new LaplaceCV<uchar, short>(5), "Laplace_Sobel_(5x5)"));
    filter.push_back(Entry(new LaplaceCV<uchar, short>(7), "Laplace_Sobel_(7x7)"));
    filter.push_back(Entry(new LaplaceCV<uchar, FT>(9), "Laplace_Sobel_(9x9)"));

    filter.push_back(Entry(new LoG<uchar, FT>(3, 1.240080), "LoG_(3x3)"));
    filter.push_back(Entry(new LoG<uchar, FT>(5, 1.008030), "LoG_(5x5)"));
    filter.push_back(Entry(new LoG<uchar, FT>(7, 0.873228), "LoG_(7x7)"));
    filter.push_back(Entry(new LoG<uchar, FT>(9, 0.781859), "LoG_(9x9)"));

    filter.push_back(Entry(new QuadratureG2<uchar, FT>(3, 1.24008), "QF_StG_(3x3)"));
    filter.push_back(Entry(new QuadratureG2<uchar, FT>(5, 1.008),   "QF_StG_(5x5)"));
    filter.push_back(Entry(new QuadratureG2<uchar, FT>(7, 0.873226), "QF_StG_(7x7)"));
    filter.push_back(Entry(new QuadratureG2<uchar, FT>(9, 0.781854), "QF_StG_(9x9)"));

    /*filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 3, 1.2), "SQF_Po_(3x3)"));
    filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 5, 1.2), "SQF_Po_(5x5)"));
    filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 7, 1.2), "SQF_Po_(7x7)"));
    filter.push_back(Entry(new QuadratureS<uchar, FT, FT>(1, 2, 9, 1.2), "SQF_Po_(9x9)"));*/

    filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 3, 1.2)), "SQF_Po_(3x3)"));
    filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 5, 1.2)), "SQF_Po_(5x5)"));
    filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 7, 1.2)), "SQF_Po_(7x7)"));
    filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 9, 1.2)), "SQF_Po_(9x9)"));
    
    
    //filter.push_back(Entry(new QuadratureSF<uchar, FT>(1, 2, 1.2), "SQFF_Po"));
    filter.push_back(Entry(dynamic_cast<Quadrature<uchar,FT,FT,FT, FT>*>(new PCLSqf<uchar, FT>(1, 2, 1.2)), "SQFF_Po_12"));
    filter.push_back(Entry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSqf<uchar, FT>(1, 3, 2.5)), "SQFF_Po"));
    filter.push_back(Entry(new QuadratureLGF<uchar, FT>(5, 0.55), "SQFF_Lg"));

    filter.push_back(Entry(new PCLgf<uchar, FT>(4, 3,2.1,0.55), "PCF_Lg"));
    filter.push_back(Entry(new PCMatlab<uchar>(4, 3, 2.1, 0.55), "PCF_Ml"));
    filter.push_back(Entry(new PCSqf<uchar, FT>(1,3,2.5), "PCF_Po"));
    
    fs::create_directory("./nms");
    for_each(sets.begin(), sets.end(), [&](const std::pair<fs::path, std::string> &data) {
        fs::create_directory("./nms/" + data.second);
        processPath(filter, data);
    });

    return 0;
}

