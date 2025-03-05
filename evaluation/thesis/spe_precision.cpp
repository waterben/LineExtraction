#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry/draw.hpp>
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
#include <imgproc/image_operator.hpp>
#include <utility/matlab_helpers.hpp>

#include <edge/nms.hpp>
#include <edge/zc.hpp>
#include <edge/spe.hpp>
#include <map>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>  
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>


#define USE_GAUSSIAN


using namespace lsfm;
using namespace std;
namespace fs = boost::filesystem;

template<class FT, template<class> class PT = cv::Point_>
struct GroundTruth {
    GroundTruth() {
        cv::Mat tmp(32000, 32000, CV_8U);
        tmp.setTo(0);

        Polygon<FT, PT> poly;
        poly.push_back(PT<FT>(5089, 2023));
        poly.push_back(PT<FT>(29947, 2023));
        poly.push_back(PT<FT>(20971, 16007));
        poly.push_back(PT<FT>(29947, 29959));
        poly.push_back(PT<FT>(5089, 29959));
        poly.push_back(PT<FT>(2017, 16007));

        double scale = 0.01;
        poly.fill(tmp, 180);

        //cv::resize(tmp, img, cv::Size(), scale, scale, cv::INTER_AREA); //-> large error, even with sub precision 
#ifdef USE_GAUSSIAN
        int kernel = 201, sigma = 50;
        std::ostringstream file;
        file << "./spe/gt_guassian_" << kernel << "_" << sigma << ".png";
        if (!boost::filesystem::exists(file.str())) {
            
            std::cout << "create ground truth..." << std::flush;
            cv::GaussianBlur(tmp, tmp, cv::Size(kernel, kernel), sigma); // long runtime
            cv::resize(tmp, img, cv::Size(), scale, scale, cv::INTER_NEAREST);
            cv::imwrite(file.str(), img);
            std::cout << "done" << std::endl;
        }
        else {
            img = cv::imread(file.str(), cv::ImreadModes::IMREAD_GRAYSCALE);
            if (!img.data)                              // Check for invalid input
            {
                cout << "Could not open or find the image" << std::endl;
                exit(-1);
            }
            std::cout << "read ground truth...done" << std::endl;
        }
#else
        if (!boost::filesystem::exists("./spe/gt_box.png")) {
            std::cout << "create ground truth..." << std::flush;
            cv::blur(tmp, tmp, cv::Size(100, 100));
            cv::resize(tmp, img, cv::Size(), scale, scale, cv::INTER_NEAREST);
            cv::imwrite("./spe/gt_box.png", img);
            std::cout << "done" << std::endl;
        }
        else {
            img = cv::imread("./spe/gt_box.png", cv::ImreadModes::IMREAD_GRAYSCALE);
            if (!img.data)                              // Check for invalid input
            {
                cout << "Could not open or find the image" << std::endl;
                exit(-1);
            }
            std::cout << "read ground truth...done" << std::endl;
        }
        
#endif
        //cv::imshow("gt", img);
        //cv::waitKey();
        poly.scale(scale);
        segments = poly.edges();
    }

    LineSegment2Vector<FT,PT> segments;
    cv::Mat img;

    inline bool inRange(size_t seg, const PT<FT> &p, FT tol) const {
        return segments[seg].inRangeTol(p, tol);
    }

    inline FT error(const PT<FT> &p) const {
        FT e = std::numeric_limits<FT>::max();
        for_each(segments.begin(), segments.end(), [&](const LineSegment2<FT, PT> &seg) {
            if (!seg.inRangeTol(p, 1))
                return;
            e = std::min(e, std::abs(seg.distance(p)));
        });
        return e;
    }

    inline size_t line(const PT<FT> &p, FT &err = FT()) const {
        size_t ret = 0;
        err = std::numeric_limits<FT>::max();
        for (size_t i = 0; i != segments.size(); ++i) {
            if (!segments[i].inRangeTol(p, 1))
                continue;
            FT dist = std::abs(segments[i].distance(p));
            if (dist < err) {
                err = dist;
                ret = i;
            }
        }
        return ret;
    }

    void draw(cv::Mat &out, const PT<FT> &p, size_t l, FT err) const {
        static cv::Vec3b color[] = { cv::Vec3b(255, 0, 0), cv::Vec3b(0, 255, 0), cv::Vec3b(255, 255, 0), cv::Vec3b(255, 0, 255), cv::Vec3b(0, 255, 255), cv::Vec3b(255, 255, 255), cv::Vec3b(0, 0, 255) };
        if (err > 1 || l > 5)
            setPixel(out, cv::Point(getX(p), getY(p)), color[6]);
        else
            setPixel(out, cv::Point(getX(p), getY(p)), color[l]);
    }
};

template<class FT, template<class> class PT = cv::Point_>
class Entry {
protected:
    Entry(const cv::Ptr<FilterI<uchar>> &fil, const std::string& n, int f)
        : name(n), filter(fil), flags(f) {}

public:

    struct Data {
        Data() : points(0), outlier(0), error(0), stdDev(0) {}
        size_t points;
        int outlier;
        FT error;
        FT stdDev;
        cv::Mat img;
        std::string name;
    };

    static GroundTruth<FT, PT> gt;
    std::string name;
    std::vector<Data> allData;
    cv::Ptr<FilterI<uchar>> filter;
    int flags;

    virtual ~Entry() {}
    virtual void process(const cv::Mat &src) = 0;

    void process() {
        process(gt.img);
    }

    static void process(const std::vector<PT<FT>> &pts, Data& data) {
        data.points = pts.size();
        data.outlier = 0;
        data.error = 0;
        data.img.create(gt.img.size(), CV_8UC3);
        data.img.setTo(0);
        FT errSqr = 0;
        for_each(pts.begin(), pts.end(), [&](const PT<FT> &p) {
            FT e;
            size_t l = gt.line(p, e);
            if (e > 1)
                ++data.outlier;
            else {
                // only measure points for error that are far enough from corners
                if (gt.inRange(l, p, -10)) {
                    data.error += e;
                    errSqr += e*e;
                }
            }

            if (getX(p) > -1 && getX(p) < data.img.cols && getY(p) > -1 && getY(p) < data.img.rows)
                gt.draw(data.img, p, l, e);
        });
        if ((pts.size() - data.outlier)) {
            data.error /= (pts.size() - data.outlier);
            errSqr /= (pts.size() - data.outlier);
        }
        else
            data.error = 0;
        data.stdDev = sqrt(errSqr - data.error*data.error);
    }

    template<class ZC>
    void processZC(const std::string & name, const std::string & type, const cv::Mat &l, double th_low, double th_high, ZC &zc) {
        PixelEstimator<FT, cv::Point_<FT>> pe;
        Data data;
        data.name = name + "_spe_nearest_" + type;
        zc.process(l, th_low, th_high);
        IndexVector idxs = zc.hysteresis_edgels();
        std::vector<PT<FT>> pts;
        pe.convert(idxs, pts, l, zc.directionMap());
        process(pts, data);
        allData.push_back(data);

        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, LinearInterpolator>> spe_linear;
        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, CubicInterpolator>> spe_cubic;

        processZC(name + "_spe_linear_" + type, l, th_low, th_high, zc, spe_linear);
        processZC(name + "_spe_cubic_" + type, l, th_low, th_high, zc, spe_cubic);
    }

    template<class ZC, class SPE>
    void processZC(const std::string & name, const cv::Mat &l, double th_low, double th_high, ZC &zc, SPE &spe) {
        Data data;
        data.name = name;
        zc.process(l, th_low, th_high);
        IndexVector idxs = zc.hysteresis_edgels();
        std::vector<PT<FT>> pts;
        spe.convert(idxs, pts, l, zc.directionMap());
        process(pts, data);
        allData.push_back(data);
    }

    template<class ZC>
    void processZC(const std::string & name, const std::string & type, const cv::Mat &l, const cv::Mat &dir, double th_low, double th_high, ZC &zc) {
        PixelEstimator<FT, cv::Point_<FT>> pe;
        Data data;
        data.name = name + "_spe_nearest_" + type;
        zc.process(l, th_low, th_high);
        IndexVector idxs = zc.hysteresis_edgels();
        std::vector<PT<FT>> pts;
        pe.convert(idxs, pts, l, zc.directionMap());
        process(pts, data);
        allData.push_back(data);

        data.name = name + "_dir_spe_nearest_" + type;
        zc.process(dir, l, th_low, th_high, -CV_PI, CV_PI);
        idxs = zc.hysteresis_edgels();
        pe.convert(idxs, pts, l, zc.directionMap());
        process(pts, data);
        allData.push_back(data);

        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, LinearInterpolator>> spe_linear;
        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, CubicInterpolator>> spe_cubic;

        processZC(name, "spe_linear", type, l, dir, th_low, th_high, zc, spe_linear);
        processZC(name, "spe_cubic", type, l, dir, th_low, th_high, zc, spe_cubic);
    }

    template<class ZC, class SPE>
    void processZC(const std::string & name, const std::string & spe_type, const std::string & type, const cv::Mat &l, const cv::Mat &dir, double th_low, double th_high, ZC &zc, SPE &spe) {
        Data data;
        data.name = name + "_" + spe_type + "_" + type;
        zc.process(l, th_low, th_high);
        IndexVector idxs = zc.hysteresis_edgels();
        std::vector<PT<FT>> pts;
        spe.convert(idxs, pts, l, zc.directionMap());
        process(pts, data);
        allData.push_back(data);

        data.name = name + "_" + spe_type + "_dir_" + type;
        pts.clear();
        spe.convertDir(idxs, pts, l, dir);
        process(pts, data);
        allData.push_back(data);
        
        data.name = name + "_dir_" + spe_type + "_dir_" + type;
        zc.process(dir, l, th_low, th_high, -CV_PI, CV_PI);
        idxs = zc.hysteresis_edgels();
        pts.clear();
        spe.convertDir(idxs, pts, l, dir);
        process(pts, data);
        allData.push_back(data);

        data.name = name + "_dir_" + spe_type + "_" + type;
        pts.clear();
        spe.convert(idxs, pts, l, zc.directionMap());
        process(pts, data);
        allData.push_back(data);
    }

    static inline void filterDirX(const IndexVector &in, const cv::Mat &dir, IndexVector &out) {
        for_each(in.begin(), in.end(), [&](index_type idx) {
            FT d = abs(dir.ptr<FT>()[idx]);
            if (d <= CV_PI / 4 || d > CV_PI / 4 * 3)
                out.push_back(idx);
        });
    }
    static inline void filterDirY(const IndexVector &in, const cv::Mat &dir, IndexVector &out) {
        for_each(in.begin(), in.end(), [&](index_type idx) {
            FT d = abs(dir.ptr<FT>()[idx]);
            if (d > CV_PI / 4 && d <= CV_PI / 4 * 3)
                out.push_back(idx);
        });
    }

    static inline IndexVector filterDirX(const IndexVector &in, const cv::Mat &dir) {
        IndexVector ret;
        filterDirX(in, dir, ret);
        return ret;
    }

    static inline IndexVector filterDirY(const IndexVector &in, const cv::Mat &dir) {
        IndexVector ret;
        filterDirY(in, dir, ret);
        return ret;
    }

    template<class ZC>
    void processZC(const std::string & name, const std::string & type, const cv::Mat &lx, const cv::Mat &ly, const cv::Mat &dir, double th_low, double th_high, ZC &zc) {
        PixelEstimator<FT, cv::Point_<FT>> pe;
        Data data;
        data.name = name + "_spe_nearest_" + type;
        zc.process(lx, th_low, th_high);
        IndexVector idxsx = filterDirX(zc.hysteresis_edgels(), dir);
        zc.process(ly, th_low, th_high);
        IndexVector idxsy = filterDirY(zc.hysteresis_edgels(), dir);

        std::vector<PT<FT>> pts, tmp;
        pe.convert(idxsx, pts, lx, dir);
        pe.convert(idxsy, tmp, ly, dir);
        pts.insert(pts.end(), tmp.begin(), tmp.end());

        process(pts, data);
        allData.push_back(data);

        data.name = name + "_dir_spe_nearest_" + type;
        zc.process(dir, lx, th_low, th_high, -CV_PI, CV_PI);
        idxsx = filterDirX(zc.hysteresis_edgels(), dir);
        zc.process(dir, ly, th_low, th_high, -CV_PI, CV_PI);
        idxsy = filterDirY(zc.hysteresis_edgels(), dir);

        pe.convert(idxsx, pts, lx, dir);
        pe.convert(idxsy, tmp, ly, dir);
        pts.insert(pts.end(), tmp.begin(), tmp.end());

        process(pts, data);
        allData.push_back(data);

        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, LinearInterpolator>> spe_linear;
        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, CubicInterpolator>> spe_cubic;

        processZC(name, "spe_linear_dir_" + type, lx, ly, dir, th_low, th_high, zc, spe_linear);
        processZC(name, "spe_cubic_dir_" + type, lx, ly, dir, th_low, th_high, zc, spe_cubic);
    }

    template<class ZC, class SPE>
    void processZC(const std::string & name, const std::string & type, const cv::Mat &lx, const cv::Mat &ly, const cv::Mat &dir, double th_low, double th_high, ZC &zc, SPE &spe) {
        Data data;
        data.name = name + "_" + type;
        zc.process(lx, th_low, th_high);
        IndexVector idxsx = filterDirX(zc.hysteresis_edgels(), dir);
        zc.process(ly, th_low, th_high);
        IndexVector idxsy = filterDirY(zc.hysteresis_edgels(), dir);
        
        std::vector<PT<FT>> pts, tmp;
        spe.convertDir(idxsx, pts, lx, dir);
        spe.convertDir(idxsy, tmp, ly, dir);
        pts.insert(pts.end(), tmp.begin(), tmp.end());

        process(pts, data);
        allData.push_back(data);

        data.name = name + "_dir_" + type;
        zc.process(dir, lx, th_low, th_high, -CV_PI, CV_PI);
        idxsx = filterDirX(zc.hysteresis_edgels(), dir);
        zc.process(dir, ly, th_low, th_high, -CV_PI, CV_PI);
        idxsy = filterDirY(zc.hysteresis_edgels(), dir);
        
        spe.convertDir(idxsx, pts, lx, dir);
        spe.convertDir(idxsy, tmp, ly, dir);
        pts.insert(pts.end(), tmp.begin(), tmp.end());
        
        process(pts, data);
        allData.push_back(data);
    }

    template<class NMS>
    void processNMS(const std::string & name, const std::string & type, const cv::Mat &gx, const cv::Mat &gy, const cv::Mat &mag, const cv::Mat &dir, double th_low, double th_high, NMS &nms, bool force_dir = false) {
        PixelEstimator<FT, cv::Point_<FT>> pe;
        Data data;
        data.name = name + "_spe_nearest_" + type;
        if (force_dir)
            nms.process(dir, mag, th_low, th_high,-CV_PI,CV_PI);
        else
            nms.process(gx, gy, mag, th_low, th_high);
        IndexVector idxs = nms.hysteresis_edgels();
        std::vector<PT<FT>> pts;
        pe.convert(idxs, pts, mag, nms.directionMap());
        process(pts, data);
        allData.push_back(data);

        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, LinearInterpolator>> spe_linear_est_lin;
        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, CubicInterpolator>> spe_cubic_est_lin;

        processNMS(name + "_spe_linear", "est_lin_" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_linear_est_lin, force_dir);
        processNMS(name + "_spe_cubic", "est_lin_" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_cubic_est_lin, force_dir);

        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, LinearInterpolator>> spe_linear_est_quad;
        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, CubicInterpolator>> spe_cubic_est_quad;

        processNMS(name + "_spe_linear", "est_quad_" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_linear_est_quad, force_dir);
        processNMS(name + "_spe_cubic", "est_quad_" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_cubic_est_quad, force_dir);

        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, LinearInterpolator>> spe_linear_est_cog;
        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, CubicInterpolator>> spe_cubic_est_cog;

        processNMS(name + "_spe_linear", "est_cog_" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_linear_est_cog, force_dir);
        processNMS(name + "_spe_cubic", "est_cog_" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_cubic_est_cog, force_dir);

        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, LinearInterpolator>> spe_linear_est_sobel;
        PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, CubicInterpolator>> spe_cubic_est_sobel;

        processNMS(name + "_spe_linear", "est_sobel_" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_linear_est_sobel, force_dir);
        processNMS(name + "_spe_cubic", "est_sobel_" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_cubic_est_sobel, force_dir);
    }

    template<class NMS, class SPE>
    void processNMS(const std::string & name, const std::string & type, const cv::Mat &gx, const cv::Mat &gy, const cv::Mat &mag, const cv::Mat &dir,  double th_low, double th_high, NMS &nms, SPE &spe, bool force_dir = false) {
        Data data;
        data.name = name + "_" + type;
        if (force_dir)
            nms.process(dir, mag, th_low, th_high, -CV_PI, CV_PI);
        else
            nms.process(gx, gy, mag, th_low, th_high);
        IndexVector idxs = nms.hysteresis_edgels();
        data.points = idxs.size();
        std::vector<PT<FT>> pts;
        spe.convert(idxs, pts, mag, nms.directionMap());
        process(pts, data);
        allData.push_back(data);

        data.name = name + "_dir_" + type;
        pts.clear();
        spe.convertDir(idxs, pts, mag, dir);
        process(pts, data);
        allData.push_back(data);
    }

    template<class NMS, class ZC>
    void process(const cv::Mat &src, NMS &nms, ZC &zc, const std::string n, bool force_dir = false) {
        filter->process(src);
        FilterResults results = filter->results();
        double th_low = nms.thresholdLow(), th_high = nms.thresholdHigh(), low, high;
        cv::Mat tmp, tmpx, tmpy, dir;
        //std::cout << "th:" << th_low << ", " << th_high << std::endl;
        FilterResults::const_iterator f = results.find("laplace");
        if (f != results.end() && results.find("even") == results.end()) {
            low = f->second.range.upper * th_low;
            high = f->second.range.upper * th_high;
            //std::cout << "laplace:" << low << ", " << high << std::endl;
            processZC("zc_" + n, "laplace", f->second.data, low, high, zc);
        }

        f = results.find("mag");
        if (f != results.end()) {
            low = f->second.range.upper * th_low;
            high = f->second.range.upper * th_high;
            tmp = f->second.data;
            f = results.find("gx");
            tmpx = f->second.data;
            f = results.find("gy");
            tmpy = f->second.data;
            f = results.find("dir");
            dir = f->second.data;
            //std::cout << "mag:" << low << ", " << high << std::endl;
            processNMS("nms_" + n, "mag", tmpx, tmpy, tmp, dir, low, high, nms, force_dir);
        }

        f = results.find("even");
        if (f != results.end()) {
            low = f->second.range.size() * th_low;
            high = f->second.range.size() * th_high;
            tmp = f->second.data;
            f = results.find("dir");
            dir = f->second.data;
            //std::cout << "even:" << low << ", " << high << std::endl;
            processZC("zc_" + n, "even", tmp, dir, low, high, zc);
        }

        f = results.find("odd");
        if (f != results.end()) {
            low = f->second.range.upper * th_low;
            high = f->second.range.upper * th_high;
            tmp = f->second.data;
            f = results.find("oddx");
            tmpx = f->second.data;
            f = results.find("oddy");
            tmpy = f->second.data;
            f = results.find("dir");
            dir = f->second.data;
            //std::cout << "odd:" << low << ", " << high << std::endl;
            processNMS("nms_" + n, "odd", tmpx, tmpy, tmp, dir, low, high, nms, force_dir);
        }

        f = results.find("energy");
        if (f != results.end()) {
            low = f->second.range.upper * th_low;
            high = f->second.range.upper * th_high;
            tmp = f->second.data;
            f = results.find("oddx");
            tmpx = f->second.data;
            f = results.find("oddy");
            tmpy = f->second.data;
            f = results.find("dir");
            dir = f->second.data;
            //std::cout << "energy:" << low << ", " << high << std::endl;
            processNMS("nms_" + n, "energy", tmpx, tmpy, tmp, dir, low, high, nms, force_dir);
        }

        f = results.find("pc");
        if (f != results.end()) {
            low = f->second.range.upper * th_low;
            high = f->second.range.upper * th_high;
            tmp = f->second.data;
            f = results.find("oddx");
            tmpx = f->second.data;
            f = results.find("oddy");
            tmpy = f->second.data;
            f = results.find("dir");
            dir = f->second.data;
            //std::cout << "pc:" << low << ", " << high << std::endl;
            processNMS("nms_" + n, "pc", tmpx, tmpy, tmp, dir, low, high, nms, force_dir);
        }

        f = results.find("pclx");
        if (f != results.end()) {
            low = f->second.range.size() * th_low * th_low;
            high = f->second.range.size() * th_high * th_high;
            tmpx = f->second.data;
            f = results.find("pcly");
            tmpy = f->second.data;
            f = results.find("dir");
            dir = f->second.data;
            //std::cout << "pcl:" << low << ", " << high << std::endl;
            processZC("zc_" + n, "pcl", tmpx, tmpy, dir, low, high, zc);
        }
    }
};

template<class FT, template<class> class PT>
GroundTruth<FT, PT> Entry<FT, PT>::gt;

template<class FT, template<class> class PT = cv::Point_>
using EntryPtr = cv::Ptr<Entry<FT,PT>>;

template<class FT, template<class> class PT = cv::Point_>
class EntryT : public Entry<FT, PT> {
    double th_low_, th_high_;
public:
    EntryT(const cv::Ptr<FilterI<uchar>> &fil, const std::string& n, int f = 0, double th_low = 0.01, double th_high = 0.03)
    : Entry<FT, PT>(fil,n,f), th_low_(th_low), th_high_(th_high) {}

    using Entry<FT, PT>::process;

    void process(const cv::Mat &src) {
        NonMaximaSuppression<FT, FT, FT> fast_nms(th_low_, th_high_,3);
        NonMaximaSuppression<FT, FT, FT, PreciseNMS<FT, FT, false, FT, EMap8, LinearInterpolator, Polar>> precise_nms_linear(th_low_, th_high_, 3);
        NonMaximaSuppression<FT, FT, FT, PreciseNMS<FT, FT, false, FT, EMap8, CubicInterpolator, Polar>> precise_nms_cubic(th_low_, th_high_, 3);

        ZeroCrossing<FT, FT, FT> fast_zc(th_low_, th_high_, 3);
        ZeroCrossing<FT, FT, FT, PreciseZC<FT, FT, FT, NCC_BASIC, EZCMap8, LinearInterpolator, Polar>> precise_zc_linear(th_low_, th_high_, 3);
        ZeroCrossing<FT, FT, FT, PreciseZC<FT, FT, FT, NCC_BASIC, EZCMap8, CubicInterpolator, Polar>> precise_zc_cubic(th_low_, th_high_, 3);

        this->process(src, fast_nms, fast_zc, "fast");
        this->process(src, precise_nms_linear, precise_zc_linear, "precise_linear", true);
        this->process(src, precise_nms_cubic, precise_zc_cubic, "precise_cubic", true);
    }
};

int main(int argc, char** argv)
{  
    typedef double FT;
    typedef EntryT<FT, cv::Point_> MyEntry;
    typedef EntryPtr<FT, cv::Point_> MyEntryPtr;
    std::vector<MyEntryPtr> entries;
    
    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, RobertsDerivative>, "Roberts_(2x2)")));
    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, PrewittDerivative>, "Prewitt_(3x3)")));
    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, ScharrDerivative>, "Scharr_(3x3)")));

    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, SobelDerivative>, "Sobel_(3x3)")));
    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, SobelDerivative>({ NV("grad_kernel_size",5) }), "Sobel_(5x5)")));
    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, SobelDerivative>({ NV("grad_kernel_size",7) }), "Sobel_(7x7)")));
    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, SobelDerivative>({ NV("grad_kernel_size",9) }), "Sobel_(9x9)")));

    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",3), NV("grad_range",1.5) }), "Gauss_(3x3)")));
    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",5), NV("grad_range",2.3) }), "Gauss_(5x5)")));
    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",7), NV("grad_range",3.0) }), "Gauss_(7x7)")));
    entries.push_back(MyEntryPtr(new MyEntry(new DerivativeGradient<uchar, FT, FT, FT, GaussianDerivative, Magnitude>({ NV("grad_kernel_size",9), NV("grad_range",3.5) }), "Gauss_(9x9)")));

    entries.push_back(MyEntryPtr(new MyEntry(new SusanGradient<FT, FT, FT>(20), "Susan_(37)")));
    entries.push_back(MyEntryPtr(new MyEntry(new SusanGradient<FT, FT, FT>(20, true), "Susan_(3x3)")));
    entries.push_back(MyEntryPtr(new MyEntry(new RCMGradient<uchar, 1, FT, FT, FT>(3, 1, cv::NORM_L2), "RMG_(3x3)")));
    entries.push_back(MyEntryPtr(new MyEntry(new RCMGradient<uchar, 1, FT, FT, FT>(5, 3, cv::NORM_L2), "RMG_(5x5)")));

    entries.push_back(MyEntryPtr(new MyEntry(new LaplaceSimple<uchar, FT>, "Laplace_(3x3)")));
    entries.push_back(MyEntryPtr(new MyEntry(new LaplaceCV<uchar, FT>(5), "Laplace_Sobel_(5x5)")));
    entries.push_back(MyEntryPtr(new MyEntry(new LaplaceCV<uchar, FT>(7), "Laplace_Sobel_(7x7)")));
    entries.push_back(MyEntryPtr(new MyEntry(new LaplaceCV<uchar, FT>(9), "Laplace_Sobel_(9x9)")));

    entries.push_back(MyEntryPtr(new MyEntry(new LoG<uchar, FT>(3, 1.240080), "LoG_(3x3)")));
    entries.push_back(MyEntryPtr(new MyEntry(new LoG<uchar, FT>(5, 1.008030), "LoG_(5x5)")));
    entries.push_back(MyEntryPtr(new MyEntry(new LoG<uchar, FT>(7, 0.873228), "LoG_(7x7)")));
    entries.push_back(MyEntryPtr(new MyEntry(new LoG<uchar, FT>(9, 0.781859), "LoG_(9x9)")));

    entries.push_back(MyEntryPtr(new MyEntry(new QuadratureG2<uchar, FT>(3, 1.24008), "QF_StG_(3x3)")));
    entries.push_back(MyEntryPtr(new MyEntry(new QuadratureG2<uchar, FT>(5, 1.008), "QF_StG_(5x5)")));
    entries.push_back(MyEntryPtr(new MyEntry(new QuadratureG2<uchar, FT>(7, 0.873226), "QF_StG_(7x7)")));
    entries.push_back(MyEntryPtr(new MyEntry(new QuadratureG2<uchar, FT>(9, 0.781854), "QF_StG_(9x9)")));

    entries.push_back(MyEntryPtr(new MyEntry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 3, 1.2)), "SQF_Po_(3x3)")));
    entries.push_back(MyEntryPtr(new MyEntry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 5, 1.2)), "SQF_Po_(5x5)")));
    entries.push_back(MyEntryPtr(new MyEntry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 7, 1.2)), "SQF_Po_(7x7)")));
    entries.push_back(MyEntryPtr(new MyEntry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSq<uchar, FT, FT>(1, 2, 9, 1.2)), "SQF_Po_(9x9)")));


    entries.push_back(MyEntryPtr(new MyEntry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSqf<uchar, FT>(1, 2, 1.2)), "SQFF_Po_12")));
    entries.push_back(MyEntryPtr(new MyEntry(dynamic_cast<Quadrature<uchar, FT, FT, FT, FT>*>(new PCLSqf<uchar, FT>(1, 3, 2.5)), "SQFF_Po")));
    entries.push_back(MyEntryPtr(new MyEntry(new QuadratureLGF<uchar, FT>(5, 0.55), "SQFF_Lg")));

    entries.push_back(MyEntryPtr(new MyEntry(new PCLgf<uchar, FT>(4, 3, 2.1, 0.55), "PCF_Lg")));
    entries.push_back(MyEntryPtr(new MyEntry(new PCMatlab<uchar>(4, 3, 2.1, 0.55), "PCF_Ml")));
    entries.push_back(MyEntryPtr(new MyEntry(new PCSqf<uchar, FT>(1, 3, 2.5), "PCF_Po")));

    
    fs::create_directory("./spe");
    std::ofstream ofs;
    ofs.open("spe_precision.csv");
    std::ofstream ofs_best;
    ofs_best.open("spe_precision_best.csv");

    cout.precision(5);
    cout.setf(std::ios::fixed, std::ios::floatfield);
    ofs.precision(5);
    ofs.setf(std::ios::fixed, std::ios::floatfield);
    cout << "Method\tPoints\tOutlier\tError\tStdDev" << std::endl;
    ofs << "Method;Points;Outlier;Error;StdDev" << std::endl;
    ofs_best << "Method;Points;Outlier;Error;StdDev" << std::endl;
    Entry<FT, cv::Point_>::Data best;
    struct MeanError {
        MeanError() : err(0), stdDev(0), count(0), points(0), outlier(0) {}
        FT err, stdDev;
        int count, points, outlier;
    };
    std::map<std::string, MeanError> meanMap;
    for_each(entries.begin(), entries.end(), [&](const MyEntryPtr &e) {
        e->process();
        best.error = std::numeric_limits<FT>::max();
        for_each(e->allData.begin(), e->allData.end(), [&](const Entry<FT, cv::Point_>::Data &data) {
            ofs << data.name << "_" << e->name << ";" << data.points << ";" << data.outlier << ";" << data.error << ";" << data.stdDev << std::endl;
            cv::imwrite("./spe/" + data.name + "_" + e->name + ".png", data.img);
            if (data.error < best.error)
                best = data;
            MeanError& me = meanMap[data.name];
            ++me.count;
            me.err += data.error;
            me.stdDev += data.stdDev;
            me.points += data.points;
            me.outlier += data.outlier;
        });
        cout << best.name << "_" << e->name << "\t" << best.points << "\t" << best.outlier << "\t" << best.error << "\t" << best.stdDev << std::endl;
        ofs_best << best.name << "_" << e->name << ";" << best.points << ";" << best.outlier << ";" << best.error << ";" << best.stdDev << std::endl;
    });

    ofs.close();
    ofs_best.close();

    ofs.clear();
    ofs.open("spe_precision_mean_all.csv");
    ofs << "Method;Points;Outlier;Error;StdDev;Count" << std::endl;
    std::map<std::string, MeanError> meanMap2;
    for_each(meanMap.begin(), meanMap.end(), [&](const std::pair<std::string, MeanError> &me) {
        MeanError& me2 = meanMap2[me.first.substr(0,me.first.find_last_of('_'))];
        me2.count += me.second.count;
        me2.err += me.second.err;
        me2.stdDev += me.second.stdDev;
        me2.points += me.second.points;
        me2.outlier += me.second.outlier;
        ofs << me.first << ";" << me.second.points / me.second.count << ";" << me.second.outlier / me.second.count
            << ";" << me.second.err / me.second.count << ";" << me.second.stdDev / me.second.count << ";" << me.second.count << std::endl;
    });
    ofs.close();

    ofs.clear();
    ofs.open("spe_precision_mean.csv");
    ofs << "Method;Points;Outlier;Error;StdDev;Count" << std::endl;
    for_each(meanMap2.begin(), meanMap2.end(), [&](const std::pair<std::string, MeanError> &me) {
        ofs << me.first << ";" << me.second.points / me.second.count << ";" << me.second.outlier / me.second.count
            << ";" << me.second.err / me.second.count << ";" << me.second.stdDev / me.second.count << ";" << me.second.count << std::endl;
    });
    ofs.close();
    //char c;
    //std::cin >> c;
    return 0;
}

