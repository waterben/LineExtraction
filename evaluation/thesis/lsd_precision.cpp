#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <imgproc/gradient_adapter.hpp>
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

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>  
#include <boost/format.hpp>


using namespace lsfm;
using namespace std;
namespace fs = boost::filesystem;

constexpr int ENTRY_RGB = 2;

constexpr int runs = 10;

template<class GT, class MT, class FT>
struct Entry {
    Entry() {}

    Entry(const cv::Ptr<GradientI<uchar,GT,MT,FT>>& g, const std::string& n, int f = 0)
        : gradient(g), name(n), time(0), images(0), flags(f) {}

    cv::Ptr<GradientI<uchar, GT, MT, FT>> gradient;
    std::string name;
    int64 time;
    int images, flags;

    inline bool rgb() const {
        return flags & ENTRY_RGB;
    }

    inline void process(const cv::Mat src) {
        gradient->process(src);
        gradient->magnitude();
        cv::Mat tmp;
        int64 start;
        for (int i = 0; i != runs; ++i) {
            start = cv::getTickCount();
            gradient->process(src);
            tmp = gradient->magnitude();
            time += cv::getTickCount() - start;
            ++images;
        }
    }
};

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

template<class GT, class MT, class DT>
void processPath(std::vector<Entry<GT,MT,DT>> &entries, const std::pair<fs::path, std::string> &path) {
    std::cout << "processing " << path.first << std::endl;
    std::vector<fs::path> files;
    parseFolder(path.first, files);

    for_each(entries.begin(), entries.end(), [&](Entry<GT, MT, DT> &e) {
        e.time = 0;
        e.images = 0;
    });

    std::for_each(files.begin(), files.end(), [&](const fs::path& file) {
        cv::Mat rgb = cv::imread(file.generic_string());
        if (rgb.empty())
        {
            cout << "Can not open " << file.generic_string() << endl;
        }
        std::cout << file << std::endl;
        cv::Mat src;
        cv::cvtColor(rgb, src, cv::COLOR_BGR2GRAY);

        for_each(entries.begin(), entries.end(), [&](Entry<GT, MT, DT> &e) {
            e.process(e.rgb() ? rgb : src);
        });
    });
}

int main(int argc, char** argv)
{
    char c;
    std::cin >> c;

    std::vector<std::pair<fs::path, std::string>> sets;
    // sets.push_back(std::pair<fs::path, std::string>("../../images/Selection", "Selection"));
    // sets.push_back(std::pair<fs::path, std::string>("../../images/BSDS500", "BSDS500"));
    sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-Q", "MDB-Q"));
    // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-H", "MDB-H"));
    // sets.push_back(std::pair<fs::path, std::string>("../../images/MDB/MiddEval3-F", "MDB-F"));

    std::vector<Entry<short, int, float>> gradI;
    /*gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, RobertsDerivative<uchar, short>, QuadraticMagnitude>, "Roberts (2x2)"));
    gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, PrewittDerivative<uchar, short>, QuadraticMagnitude>, "Prewitt (3x3)"));
    gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, ScharrDerivative<uchar, short>, QuadraticMagnitude>, "Scharr (3x3)"));
    
    gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>, "Sobel (3x3)"));
    gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",5) }), "Sobel (5x5)"));
    gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",7) }), "Sobel (7x7)"));
    gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",9) }), "Sobel (9x9)"));*/
    gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",11) }), "Sobel (11x11)"));
    gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",13) }), "Sobel (13x13)"));
    gradI.push_back(Entry<short, int, float>(new DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude>({ NV("grad_kernel_size",15) }), "Sobel (15x15)"));

    /*gradI.push_back(Entry<short, int, float>(new SusanGradient<short, int>, "Susan (37)"));
    gradI.push_back(Entry<short, int, float>(new SusanGradient<short, int>(20, true), "Susan (3x3)"));

    gradI.push_back(Entry<short, int, float>(new RCMGradient<uchar, 1, short, int>(3, 1), "RMG (3x3)"));
    gradI.push_back(Entry<short, int, float>(new RCMGradient<uchar, 1, short, int>(5, 3), "RMG (5x5)"));
    gradI.push_back(Entry<short, int, float>(new RCMGradient<uchar, 3, short, int>(3, 1), "RCMG (3x3)", ENTRY_RGB));*/

    std::vector<Entry<float, float, float>> gradF;
    /*gradF.push_back(Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",3), NV("grad_range",1.5) }), "Gauss (3x3)"));
    gradF.push_back(Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",5), NV("grad_range",2.3) }), "Gauss (5x5)"));
    gradF.push_back(Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",7), NV("grad_range",3.0) }), "Gauss (7x7)"));
    gradF.push_back(Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",9), NV("grad_range",3.5) }), "Gauss (9x9)"));*/
    gradF.push_back(Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",11), NV("grad_range",4) }), "Gauss (11x11)"));
    gradF.push_back(Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",13), NV("grad_range",4.5) }), "Gauss (13x13)"));
    gradF.push_back(Entry<float, float, float>(new DerivativeGradient<uchar, float, float, float, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",15), NV("grad_range",5) }), "Gauss (15x15)"));
    
    /*gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size",3), NV("grad_kernel_spacing",1.24008) }), "QF_StG (3x3)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.008) }), "QF_StG (5x5)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 0.873226) }), "QF_StG (7x7)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 0.781854) }), "QF_StG (9x9)"));*/
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 11), NV("grad_kernel_spacing", 0.7) }), "QF_StG (11x11)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 13), NV("grad_kernel_spacing", 0.65) }), "QF_StG (13x13)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureG2<uchar, float, PolarCV>>({ NV("grad_kernel_size", 15), NV("grad_kernel_spacing", 0.6) }), "QF_StG (15x15)"));
    
    /*gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 3), NV("grad_kernel_spacing", 1.2) }), "SQF PO (3x3)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.2) }), "SQF PO (5x5)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 1.2) }), "SQF PO (7x7)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 1.2) }), "SQF PO (9x9)"));*/
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 11), NV("grad_kernel_spacing", 1.2) }), "SQF PO (11x11)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 13), NV("grad_kernel_spacing", 1.2) }), "SQF PO (13x13)"));
    gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureS<uchar, float, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 15), NV("grad_kernel_spacing", 1.2) }), "SQF PO (15x15)"));
    
    //gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureLGF<uchar, float, PolarCV>>({ NV("grad_waveLength", 3), NV("grad_sigmaOnf", 0.55) }), "SQFF LG"));
    //gradF.push_back(Entry<float, float, float>(new GradientEnergy<QuadratureSF<uchar, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2) }), "SQFF PO"));
    
    //gradF.push_back(Entry<float, float, float>(new GradientPC<PCLgf<uchar, float, PolarCV>>, "PC LG"));
    
    //gradF.push_back(Entry<float, float, float>(new GradientPC<PCSqf<uchar, float, PolarCV>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2) }), "PC PO"));

    std::vector<Entry<double, double, double>> gradD;
    //gradD.push_back(Entry<double, double, double>(new GradientPC<PCMatlab<uchar>>, "PC ML"));

    int cols = gradI.size() + gradF.size() + gradD.size() + 1;
    int rows = sets.size() + 1;
    std::vector<std::vector<std::string>> table;
    table.resize(cols);
    for_each(table.begin(), table.end(), [&](std::vector<std::string> &row) {
        row.resize(rows);
    });

    table[0][0] = "Method";

    int row = 1;
    for_each(gradI.begin(), gradI.end(), [&](const Entry<short, int, float> &e) {
        table[row++][0] = e.name;
    });
    for_each(gradF.begin(), gradF.end(), [&](const Entry<float, float, float> &e) {
        table[row++][0] = e.name;
    });
    for_each(gradD.begin(), gradD.end(), [&](const Entry<double, double, double> &e) {
        table[row++][0] = e.name;
    });

    int col = 1;
    for_each(sets.begin(), sets.end(), [&](const std::pair<fs::path, std::string> &data) {
        
        processPath(gradI, data);
        processPath(gradF, data);
        processPath(gradD, data);

        table[0][col] = data.second;
        row = 1;
        for_each(gradI.begin(), gradI.end(), [&](const Entry<short, int, float> &e) {
            table[row++][col] = boost::str(boost::format("%.3f") % (static_cast<double>(e.time * 1000) / (e.images * cv::getTickFrequency()))) + "ms";
        });

        for_each(gradF.begin(), gradF.end(), [&](const Entry<float, float, float> &e) {
            table[row++][col] = boost::str(boost::format("%.3f") % (static_cast<double>(e.time * 1000) / (e.images * cv::getTickFrequency()))) + "ms";
        });

        for_each(gradD.begin(), gradD.end(), [&](const Entry<double, double, double> &e) {
            table[row++][col] = boost::str(boost::format("%.3f") % (static_cast<double>(e.time * 1000) / (e.images * cv::getTickFrequency()))) + "ms";
        });
        ++col;
    });

    std::ofstream ofs;
    ofs.open("gradient_profiling.csv");

    for_each(table.begin(), table.end(), [&](const std::vector<std::string> &row) {
        for_each(row.begin(), row.end(), [&](const std::string &cell) {
            std::cout << cell << "\t";
            ofs << cell << ";";
        });
        std::cout << std::endl;
        ofs << std::endl;
    });

    ofs.close();
    
    return 0;
}

