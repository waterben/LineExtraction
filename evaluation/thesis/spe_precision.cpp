#include <edge/edge_linking.hpp>
#include <edge/fit.hpp>
#include <edge/nms.hpp>
#include <edge/spe.hpp>
#include <edge/threshold.hpp>
#include <edge/zc.hpp>
#include <eval/eval_app.hpp>
#include <geometry/draw.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <utility/matlab_helpers.hpp>
#include <utility/response_convert.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>


#define USE_GAUSSIAN


using namespace lsfm;
using namespace std;
namespace fs = std::filesystem;

constexpr int global_runs = 100;

template <class OP>
double performanceNMS(OP& op, const cv::Mat& src, double th_low, double th_high, size_t runs = global_runs) {
  int64 rt{}, tmp{};
  for (size_t i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    op.process(src, th_low, th_high);
    rt += cv::getTickCount() - tmp;
  }

  return (static_cast<double>(rt) * 1000.0 / cv::getTickFrequency()) / static_cast<double>(runs);
}

template <class OP>
double performanceNMS(OP& op,
                      const cv::Mat& dir,
                      const cv::Mat& src,
                      double th_low,
                      double th_high,
                      double r_low,
                      double r_high,
                      size_t runs = 10) {
  int64 rt{}, tmp{};
  for (size_t i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    op.process(dir, src, th_low, th_high, r_low, r_high);
    rt += cv::getTickCount() - tmp;
  }

  return (static_cast<double>(rt) * 1000.0 / cv::getTickFrequency()) / static_cast<double>(runs);
}

template <class OP>
double performanceNMS(OP& op,
                      const cv::Mat& gx,
                      const cv::Mat& gy,
                      const cv::Mat& mag,
                      double th_low,
                      double th_high,
                      size_t runs = global_runs) {
  int64 rt{}, tmp{};
  for (size_t i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    op.process(gx, gy, mag, th_low, th_high);
    rt += cv::getTickCount() - tmp;
  }

  return (static_cast<double>(rt) * 1000.0 / cv::getTickFrequency()) / static_cast<double>(runs);
}


template <class OP, typename FT, template <class> class PT>
double performanceSPE(OP& op,
                      const IndexVector& idxs,
                      std::vector<PT<FT>>& pts,
                      const cv::Mat& src,
                      const cv::Mat& dir,
                      size_t runs = 10) {
  int64 rt{}, tmp{};
  for (size_t i = 0; i != runs; ++i) {
    std::vector<PT<FT>> lpts;
    tmp = cv::getTickCount();
    op.convert(idxs, lpts, src, dir);
    rt += cv::getTickCount() - tmp;
    pts.swap(lpts);
  }

  return (static_cast<double>(rt) * 1000.0 / cv::getTickFrequency()) / static_cast<double>(runs);
}

template <class OP, typename FT, template <class> class PT>
double performanceSPE_DIR(OP& op,
                          const IndexVector& idxs,
                          std::vector<PT<FT>>& pts,
                          const cv::Mat& src,
                          const cv::Mat& dir,
                          size_t runs = 10) {
  int64 rt{}, tmp{};
  for (size_t i = 0; i != runs; ++i) {
    std::vector<PT<FT>> lpts;
    tmp = cv::getTickCount();
    op.convertDir(idxs, lpts, src, dir);
    rt += cv::getTickCount() - tmp;
    pts.swap(lpts);
  }

  return (static_cast<double>(rt) * 1000.0 / cv::getTickFrequency()) / static_cast<double>(runs);
}


cv::Mat thinImage(const cv::Mat& src) {
  cv::Mat thinnedImage;
  cv::threshold(src, thinnedImage, 0, 255, cv::THRESH_BINARY);
  thinnedImage.convertTo(thinnedImage, CV_8UC1);
  cv::ximgproc::thinning(thinnedImage, thinnedImage, cv::ximgproc::THINNING_ZHANGSUEN);
  return thinnedImage;
}


template <class FT, template <class> class PT = cv::Point_>
struct GroundTruth {
  static constexpr double scale = 0.01;
  static constexpr int kernel = 201, sigma = 50;

  GroundTruth(const std::string& input = "./spe", bool show_gt = false, bool verbose = false) {
    Polygon<FT, PT> poly;
    poly.push_back(PT<FT>(5089, 2023));
    poly.push_back(PT<FT>(29947, 2023));
    poly.push_back(PT<FT>(20971, 16007));
    poly.push_back(PT<FT>(29947, 29959));
    poly.push_back(PT<FT>(5089, 29959));
    poly.push_back(PT<FT>(2017, 16007));

    std::ostringstream file;
    file << input << "/gt_guassian_" << kernel << "_" << sigma << ".png";
    if (!std::filesystem::exists(file.str())) {
      if (verbose) {
        std::cout << "create ground truth..." << std::flush;
      }

      cv::Mat tmp(32000, 32000, CV_8U);
      tmp.setTo(0);

      poly.fill(tmp, 180);

      cv::GaussianBlur(tmp, tmp, cv::Size(kernel, kernel), sigma);  // long runtime
      // cv::blur(tmp, tmp, cv::Size(101, 101));
      cv::resize(tmp, img, cv::Size(), scale, scale, cv::INTER_NEAREST);

      cv::imwrite(file.str(), img);
      std::cout << "done" << std::endl;
    } else {
      if (verbose) {
        std::cout << "read ground truth..." << std::flush;
      }

      img = cv::imread(file.str(), cv::ImreadModes::IMREAD_GRAYSCALE);
      if (!img.data) {  // Check for invalid input
        if (verbose) {
          cout << "failed" << std::endl;
        }
        cerr << "Could not open or find the image" << std::endl;
        exit(-1);
      }
    }
    if (verbose) {
      std::cout << "done" << std::endl;
    }

    if (show_gt) {
      cv::imshow("gt", img);
      cv::waitKey();
    }
    poly.scale(scale);
    segments = poly.edges();
  }

  LineSegment2Vector<FT, PT> segments;
  cv::Mat img;

  bool inRange(size_t seg, const PT<FT>& p, FT tol) const { return segments[seg].inRangeTol(p, tol); }

  FT error(const PT<FT>& p) const {
    FT e = std::numeric_limits<FT>::max();
    for_each(segments.begin(), segments.end(), [&](const LineSegment2<FT, PT>& seg) {
      if (!seg.inRangeTol(p, 1)) return;
      e = std::min(e, std::abs(seg.distance(p)));
    });
    return e;
  }

  // Extract nearest line for point and calc distance as error
  size_t line(const PT<FT>& p, FT& err = FT()) const {
    size_t ret = 0;
    err = std::numeric_limits<FT>::max();
    for (size_t i = 0; i != segments.size(); ++i) {
      if (!segments[i].inRangeTol(p, 1)) continue;
      FT dist = std::abs(segments[i].distance(p));
      if (dist < err) {
        err = dist;
        ret = i;
      }
    }
    return ret;
  }

  void draw(cv::Mat& out, const PT<FT>& p, size_t l, FT err) const {
    static cv::Vec3b color[] = {cv::Vec3b(255, 0, 0),   cv::Vec3b(0, 255, 0),   cv::Vec3b(255, 255, 0),
                                cv::Vec3b(255, 0, 255), cv::Vec3b(0, 255, 255), cv::Vec3b(255, 255, 255),
                                cv::Vec3b(0, 0, 255)};
    if (err > 1 || l > 5)
      setPixel(out, cv::Point(getX(p), getY(p)), color[6]);
    else
      setPixel(out, cv::Point(getX(p), getY(p)), color[l]);
  }

  cv::Mat drawGT() const {
    static cv::Vec3b color[] = {cv::Vec3b(255, 0, 0),   cv::Vec3b(0, 255, 0),   cv::Vec3b(255, 255, 0),
                                cv::Vec3b(255, 0, 255), cv::Vec3b(0, 255, 255), cv::Vec3b(255, 255, 255),
                                cv::Vec3b(0, 0, 255)};
    cv::Mat ret;
    ret.create(img.size(), CV_8UC3);
    ret.setTo(0);
    for (std::size_t i = 0; i != segments.size(); ++i) {
      segments[i].draw(ret, color[i]);
    }
    return ret;
  }
};

template <class FT, template <class> class PT = cv::Point_>
struct Data {
  Data() = default;
  Data(const Data&) = default;
  Data(Data&&) = default;
  Data& operator=(const Data&) = default;
  Data& operator=(Data&&) = default;

  Data(std::string n, std::vector<PT<FT>>&& p, const GroundTruth<FT, PT>& gt, double rt_nms = 0, double rt_spe = 0)
      : points(std::move(p)), runtime_nms(rt_nms), runtime_spe(rt_spe), name(std::move(n)) {
    process(gt);
  }

  std::vector<PT<FT>> points{};
  std::vector<std::size_t> outlier{};
  std::vector<std::vector<std::size_t>> segment_points{};

  struct Error {
    double sum{};
    double sum_pow_2{};
    std::size_t count{};

    double value() const {
      if (count == 0) {
        return 0;
      }
      return sum / static_cast<double>(count);
    }

    double stdDev() const {
      if (count == 0) {
        return 0;
      }
      const auto val = value();
      return std::sqrt(std::abs(sum_pow_2 / static_cast<double>(count) - val * val));
    }

    void clear() {
      sum = 0;
      sum_pow_2 = 0;
      count = 0;
    }

    void add(double e) {
      sum += e;
      sum_pow_2 += e * e;
      ++count;
    }
  };

  Error error;
  std::vector<Error> error_lines{};

  void clearError() {
    error.clear();
    for (auto& lerr : error_lines) {
      lerr.clear();
    }
  }

  FT runtime_nms{};
  FT runtime_spe{};
  FT runtime() const { return runtime_nms + runtime_spe; }
  cv::Mat img{};
  std::string name{};

  void process(const GroundTruth<FT, PT>& gt) {
    img.create(gt.img.size(), CV_8UC3);
    img.setTo(0);
    outlier.clear();
    segment_points.clear();
    segment_points.resize(gt.segments.size());
    error_lines.clear();
    error_lines.resize(gt.segments.size());
    error.clear();

    for (std::size_t i = 0; i != points.size(); ++i) {
      const auto& p = points[i];
      FT e;
      size_t l = gt.line(p, e);
      if (e > 1) {
        outlier.push_back(i);
      } else {
        segment_points[l].push_back(i);
        // only measure points for error that are far enough from corners
        if (gt.inRange(l, p, -10)) {
          error.add(e);
          error_lines[l].add(e);
        }
      }
      if (getX(p) > -1 && getX(p) < img.cols && getY(p) > -1 && getY(p) < img.rows) {
        gt.draw(img, p, l, e);
      }
    }
  }

  LineSegment2Vector<FT, PT> lineSegsFromSegPoints() const {
    LineSegment2Vector<FT, PT> ret;
    using MyPoint = PT<FT>;
    using MyEigenFit = EigenFit<FT, MyPoint>;
    FitLine<MyEigenFit> fit;
    ret.reserve(segment_points.size());
    for (const auto& seg_points : segment_points) {
      std::vector<PT<FT>> point_list;
      point_list.reserve(segment_points.size());
      PT<FT> pt_min(std::numeric_limits<FT>::max(), std::numeric_limits<FT>::max());
      PT<FT> pt_max(std::numeric_limits<FT>::min(), std::numeric_limits<FT>::min());
      for (const auto& point_index : seg_points) {
        auto pt = points[point_index];
        pt_max.x = std::max(pt_max.x, pt.x);
        pt_max.y = std::max(pt_max.y, pt.y);
        pt_min.x = std::min(pt_min.x, pt.x);
        pt_min.y = std::min(pt_min.y, pt.y);
        point_list.push_back(pt);
      }
      Line<FT, PT> l;
      fit.apply(point_list.begin(), point_list.end(), l);

      ret.push_back(LineSegment<FT, PT>(l, pt_min, pt_max));
    }
    return ret;
  }
};

template <class FT, template <class> class PT = cv::Point_>
class Entry {
 protected:
  Entry(const GroundTruth<FT, PT>& gt_in,
        const cv::Ptr<FilterI<uchar>>& fil,
        const std::string& n,
        int f,
        cv::Mat odir = cv::Mat())
      : gt(gt_in), name(n), filter(fil), flags(f) {}

 public:
  using MyData = Data<FT, PT>;
  const GroundTruth<FT, PT>& gt;
  std::string name{};
  std::vector<MyData> allData{};
  cv::Ptr<FilterI<uchar>> filter{};
  cv::Mat optional_dir{};
  int flags{};


  virtual ~Entry() {}
  virtual void process(const cv::Mat& src) = 0;

  void process() { process(gt.img); }

  template <class ZC>
  void processZC(const std::string& name, std::string type, const cv::Mat& l, double th_low, double th_high, ZC& zc) {
    if (type.size() && type[0] != ' ') {
      type.insert(0, 1, ' ');
    }
    PixelEstimator<FT, cv::Point_<FT>> pe;
    auto rt_nms = performanceNMS(zc, l, th_low, th_high);
    IndexVector idxs = zc.hysteresis_edgels();
    std::vector<PT<FT>> pts;
    auto rt_spe = performanceSPE(pe, idxs, pts, l, zc.directionMap());
    allData.emplace_back(MyData{name + " spe near" + type, std::move(pts), gt, rt_nms, rt_spe});

    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, LinearInterpolator>>
        spe_linear;
    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, CubicInterpolator>>
        spe_cubic;

    processZC(name + " spe lin" + type, l, th_low, th_high, zc, spe_linear);
    processZC(name + " spe cub" + type, l, th_low, th_high, zc, spe_cubic);
  }

  template <class ZC, class SPE>
  void processZC(const std::string& name, const cv::Mat& l, double th_low, double th_high, ZC& zc, SPE& spe) {
    auto rt_nms = performanceNMS(zc, l, th_low, th_high);
    IndexVector idxs = zc.hysteresis_edgels();
    std::vector<PT<FT>> pts;
    auto rt_spe = performanceSPE(spe, idxs, pts, l, zc.directionMap());
    allData.emplace_back(MyData{name, std::move(pts), gt, rt_nms, rt_spe});
  }

  template <class ZC>
  void processZC(const std::string& name,
                 std::string type,
                 const cv::Mat& l,
                 const cv::Mat& dir,
                 double th_low,
                 double th_high,
                 ZC& zc) {
    if (type.size() && type[0] != ' ') {
      type.insert(0, 1, ' ');
    }
    PixelEstimator<FT, cv::Point_<FT>> pe;
    auto rt_nms = performanceNMS(zc, l, th_low, th_high);
    IndexVector idxs = zc.hysteresis_edgels();
    std::vector<PT<FT>> pts;
    auto rt_spe = performanceSPE(pe, idxs, pts, l, zc.directionMap());
    allData.emplace_back(MyData{name + " spe near" + type, std::move(pts), gt, rt_nms, rt_spe});

    if (name != "zc fast") {
      rt_nms = performanceNMS(zc, dir, l, th_low, th_high, -CV_PI, CV_PI);
      idxs = zc.hysteresis_edgels();
      pts.clear();
      rt_spe = performanceSPE(pe, idxs, pts, l, zc.directionMap());
      allData.emplace_back(MyData{name + " dir spe near" + type, std::move(pts), gt, rt_nms, rt_spe});
    }

    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, LinearInterpolator>>
        spe_linear;
    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, CubicInterpolator>>
        spe_cubic;

    processZC(name, "spe lin", type, l, dir, th_low, th_high, zc, spe_linear);
    processZC(name, "spe cub", type, l, dir, th_low, th_high, zc, spe_cubic);
  }

  template <class ZC, class SPE>
  void processZC(const std::string& name,
                 std::string spe_type,
                 std::string type,
                 const cv::Mat& l,
                 const cv::Mat& dir,
                 double th_low,
                 double th_high,
                 ZC& zc,
                 SPE& spe) {
    if (type.size() && type[0] != ' ') {
      type.insert(0, 1, ' ');
    }
    if (spe_type.size() && spe_type[0] != ' ') {
      spe_type.insert(0, 1, ' ');
    }
    auto rt_nms = performanceNMS(zc, l, th_low, th_high);
    IndexVector idxs = zc.hysteresis_edgels();
    std::vector<PT<FT>> pts;
    auto rt_spe = performanceSPE(spe, idxs, pts, l, zc.directionMap());
    allData.emplace_back(MyData{name + spe_type + type, std::move(pts), gt, rt_nms, rt_spe});

    pts.clear();
    rt_spe = performanceSPE_DIR(spe, idxs, pts, l, dir);
    allData.emplace_back(MyData{name + spe_type + " dir" + type, std::move(pts), gt, rt_nms, rt_spe});

    if (name != "zc fast") {
      rt_nms = performanceNMS(zc, dir, l, th_low, th_high, -CV_PI, CV_PI);
      idxs = zc.hysteresis_edgels();
      pts.clear();
      rt_spe = performanceSPE_DIR(spe, idxs, pts, l, dir);
      allData.emplace_back(MyData{name + " dir" + spe_type + " dir" + type, std::move(pts), gt, rt_nms, rt_spe});

      pts.clear();
      rt_spe = performanceSPE(spe, idxs, pts, l, zc.directionMap());
      allData.emplace_back(MyData{name + " dir" + spe_type + type, std::move(pts), gt, rt_nms, rt_spe});
    }
  }

  static inline void filterDirX(const IndexVector& in, const cv::Mat& dir, IndexVector& out) {
    for_each(in.begin(), in.end(), [&](index_type idx) {
      FT d = abs(dir.ptr<FT>()[idx]);
      if (d <= CV_PI / 4 || d > CV_PI / 4 * 3) out.push_back(idx);
    });
  }
  static inline void filterDirY(const IndexVector& in, const cv::Mat& dir, IndexVector& out) {
    for_each(in.begin(), in.end(), [&](index_type idx) {
      FT d = abs(dir.ptr<FT>()[idx]);
      if (d > CV_PI / 4 && d <= CV_PI / 4 * 3) out.push_back(idx);
    });
  }

  static inline IndexVector filterDirX(const IndexVector& in, const cv::Mat& dir) {
    IndexVector ret;
    filterDirX(in, dir, ret);
    return ret;
  }

  static inline IndexVector filterDirY(const IndexVector& in, const cv::Mat& dir) {
    IndexVector ret;
    filterDirY(in, dir, ret);
    return ret;
  }

  template <class ZC>
  void processZC(const std::string& name,
                 std::string type,
                 const cv::Mat& lx,
                 const cv::Mat& ly,
                 const cv::Mat& dir,
                 double th_low,
                 double th_high,
                 ZC& zc) {
    if (type.size() && type[0] != ' ') {
      type.insert(0, 1, ' ');
    }
    PixelEstimator<FT, cv::Point_<FT>> pe;
    auto rt_nms = performanceNMS(zc, lx, th_low, th_high);
    IndexVector idxsx = filterDirX(zc.hysteresis_edgels(), dir);
    rt_nms += performanceNMS(zc, ly, th_low, th_high);
    IndexVector idxsy = filterDirY(zc.hysteresis_edgels(), dir);

    std::vector<PT<FT>> pts, tmp;
    auto rt_spe = performanceSPE(pe, idxsx, pts, lx, dir);
    rt_spe += performanceSPE(pe, idxsy, tmp, ly, dir);
    pts.insert(pts.end(), tmp.begin(), tmp.end());
    allData.emplace_back(MyData{name + " spe near" + type, std::move(pts), gt, rt_nms, rt_spe});

    rt_nms = performanceNMS(zc, dir, lx, th_low, th_high, -CV_PI, CV_PI);
    idxsx = filterDirX(zc.hysteresis_edgels(), dir);
    rt_nms += performanceNMS(zc, dir, ly, th_low, th_high, -CV_PI, CV_PI);
    idxsy = filterDirY(zc.hysteresis_edgels(), dir);

    rt_spe = performanceSPE(pe, idxsx, pts, lx, dir);
    rt_spe += performanceSPE(pe, idxsy, tmp, ly, dir);
    pts.insert(pts.end(), tmp.begin(), tmp.end());
    allData.emplace_back(MyData{name + " dir spe near" + type, std::move(pts), gt, rt_nms, rt_spe});

    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, LinearInterpolator>>
        spe_linear;
    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, CubicInterpolator>>
        spe_cubic;

    processZC(name, "spe lin dir" + type, lx, ly, dir, th_low, th_high, zc, spe_linear);
    processZC(name, "spe cub dir" + type, lx, ly, dir, th_low, th_high, zc, spe_cubic);
  }

  template <class ZC, class SPE>
  void processZC(const std::string& name,
                 std::string type,
                 const cv::Mat& lx,
                 const cv::Mat& ly,
                 const cv::Mat& dir,
                 double th_low,
                 double th_high,
                 ZC& zc,
                 SPE& spe) {
    if (type.size() && type[0] != ' ') {
      type.insert(0, 1, ' ');
    }
    auto rt_nms = performanceNMS(zc, lx, th_low, th_high);
    IndexVector idxsx = filterDirX(zc.hysteresis_edgels(), dir);
    rt_nms += performanceNMS(zc, ly, th_low, th_high);
    IndexVector idxsy = filterDirY(zc.hysteresis_edgels(), dir);

    std::vector<PT<FT>> pts, tmp;
    auto rt_spe = performanceSPE_DIR(spe, idxsx, pts, lx, dir);
    rt_spe += performanceSPE_DIR(spe, idxsy, tmp, ly, dir);
    pts.insert(pts.end(), tmp.begin(), tmp.end());
    allData.emplace_back(MyData{name + type, std::move(pts), gt, rt_nms, rt_spe});

    rt_nms = performanceNMS(zc, dir, lx, th_low, th_high, -CV_PI, CV_PI);
    idxsx = filterDirX(zc.hysteresis_edgels(), dir);
    rt_nms += performanceNMS(zc, dir, ly, th_low, th_high, -CV_PI, CV_PI);
    idxsy = filterDirY(zc.hysteresis_edgels(), dir);

    rt_spe = performanceSPE_DIR(spe, idxsx, pts, lx, dir);
    rt_spe += performanceSPE_DIR(spe, idxsy, tmp, ly, dir);
    pts.insert(pts.end(), tmp.begin(), tmp.end());

    allData.emplace_back(MyData{name + " dir" + type, std::move(pts), gt, rt_nms, rt_spe});
  }

  template <class NMS>
  void processNMS(const std::string& name,
                  std::string type,
                  const cv::Mat& gx,
                  const cv::Mat& gy,
                  const cv::Mat& mag,
                  const cv::Mat& dir,
                  double th_low,
                  double th_high,
                  NMS& nms,
                  bool force_dir = false) {
    if (type.size() && type[0] != ' ') {
      type.insert(0, 1, ' ');
    }
    PixelEstimator<FT, cv::Point_<FT>> pe;
    auto rt_nms = force_dir ? performanceNMS(nms, dir, mag, th_low, th_high, -CV_PI, CV_PI)
                            : performanceNMS(nms, gx, gy, mag, th_low, th_high);
    IndexVector idxs = nms.hysteresis_edgels();
    std::vector<PT<FT>> pts;
    auto rt_spe = performanceSPE(pe, idxs, pts, mag, nms.directionMap());
    allData.emplace_back(MyData{name + " spe near" + type, std::move(pts), gt, rt_nms, rt_spe});


    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, LinearInterpolator>>
        spe_linear_est_lin;
    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, LinearEstimate, CubicInterpolator>>
        spe_cubic_est_lin;

    processNMS(name + " spe lin", "est lin" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_linear_est_lin,
               force_dir);
    processNMS(name + " spe cub", "est lin" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_cubic_est_lin,
               force_dir);

    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, LinearInterpolator>>
        spe_linear_est_quad;
    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, CubicInterpolator>>
        spe_cubic_est_quad;

    processNMS(name + " spe lin", "est quad" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_linear_est_quad,
               force_dir);
    processNMS(name + " spe cub", "est quad" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_cubic_est_quad,
               force_dir);

    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, LinearInterpolator>>
        spe_linear_est_cog;
    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, CoGEstimate, CubicInterpolator>>
        spe_cubic_est_cog;

    processNMS(name + " spe lin", "est cog" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_linear_est_cog,
               force_dir);
    processNMS(name + " spe cub", "est cog" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_cubic_est_cog,
               force_dir);

    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, LinearInterpolator>>
        spe_linear_est_sobel;
    PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelEstimate, CubicInterpolator>>
        spe_cubic_est_sobel;

    processNMS(name + " spe lin", "est sob" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_linear_est_sobel,
               force_dir);
    processNMS(name + " spe cub", "est sob" + type, gx, gy, mag, dir, th_low, th_high, nms, spe_cubic_est_sobel,
               force_dir);
  }

  template <class NMS, class SPE>
  void processNMS(const std::string& name,
                  std::string type,
                  const cv::Mat& gx,
                  const cv::Mat& gy,
                  const cv::Mat& mag,
                  const cv::Mat& dir,
                  double th_low,
                  double th_high,
                  NMS& nms,
                  SPE& spe,
                  bool force_dir = false) {
    if (type.size() && type[0] != ' ') {
      type.insert(0, 1, ' ');
    }
    auto rt_nms = force_dir ? performanceNMS(nms, dir, mag, th_low, th_high, -CV_PI, CV_PI)
                            : performanceNMS(nms, gx, gy, mag, th_low, th_high);
    IndexVector idxs = nms.hysteresis_edgels();
    std::vector<PT<FT>> pts;
    auto rt_spe = performanceSPE(spe, idxs, pts, mag, nms.directionMap());
    allData.emplace_back(MyData{name + type, std::move(pts), gt, rt_nms, rt_spe});

    pts.clear();
    rt_spe = performanceSPE_DIR(spe, idxs, pts, mag, dir);
    allData.emplace_back(MyData{name + " dir" + type, std::move(pts), gt, rt_nms, rt_spe});
  }

  template <class NMS, class ZC>
  void process(const cv::Mat& src, NMS& nms, ZC& zc, const std::string n, bool force_dir = false) {
    filter->process(src);
    FilterResults results = filter->results();
    double th_low = nms.thresholdLow(), th_high = nms.thresholdHigh(), low, high;
    cv::Mat tmp, tmpx, tmpy, dir;
    // std::cout << "th:" << th_low << ", " << th_high << std::endl;
    FilterResults::const_iterator f = results.find("laplace");
    if (f != results.end() && results.find("even") == results.end()) {
      low = f->second.range.upper * th_low;
      high = f->second.range.upper * th_high;
      // std::cout << "laplace:" << low << ", " << high << std::endl;
      if (optional_dir.empty()) {
        processZC("zc " + n, "", f->second.data, low, high, zc);
      } else {
        processZC("zc " + n, "", f->second.data, optional_dir, low, high, zc);
      }
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
      // std::cout << "mag:" << low << ", " << high << std::endl;
      processNMS("nms " + n, "", tmpx, tmpy, tmp, dir, low, high, nms, force_dir);
    }

    f = results.find("even");
    if (f != results.end()) {
      low = f->second.range.size() * th_low;
      high = f->second.range.size() * th_high;
      tmp = f->second.data;
      f = results.find("dir");
      dir = f->second.data;
      // std::cout << "even:" << low << ", " << high << std::endl;
      processZC("zc " + n, "even", tmp, dir, low, high, zc);
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
      // std::cout << "odd:" << low << ", " << high << std::endl;
      processNMS("nms " + n, "odd", tmpx, tmpy, tmp, dir, low, high, nms, force_dir);
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
      // std::cout << "energy:" << low << ", " << high << std::endl;
      processNMS("nms " + n, "energy", tmpx, tmpy, tmp, dir, low, high, nms, force_dir);
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
      // std::cout << "pc:" << low << ", " << high << std::endl;
      processNMS("nms " + n, "pc", tmpx, tmpy, tmp, dir, low, high, nms, force_dir);
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
      // std::cout << "pcl:" << low << ", " << high << std::endl;
      processZC("zc " + n, "pcl", tmpx, tmpy, dir, low, high, zc);
    }
  }
};

template <class FT, template <class> class PT = cv::Point_>
using EntryPtr = cv::Ptr<Entry<FT, PT>>;

template <class FT, template <class> class PT = cv::Point_>
class EntryT : public Entry<FT, PT> {
  double th_low_, th_high_;

 public:
  EntryT(const GroundTruth<FT, PT>& gt,
         const cv::Ptr<FilterI<uchar>>& fil,
         const std::string& n,
         int f = 0,
         double th_low = 0.1,
         double th_high = 0.2)
      : Entry<FT, PT>(gt, fil, n, f), th_low_(th_low), th_high_(th_high) {}

  using Entry<FT, PT>::process;

  void process(const cv::Mat& src) override {
    NonMaximaSuppression<FT, FT, FT> fast_nms(th_low_, th_high_, 3);
    NonMaximaSuppression<FT, FT, FT, PreciseNMS<FT, FT, false, FT, EMap8, LinearInterpolator, Polar>>
        precise_nms_linear(th_low_, th_high_, 3);
    NonMaximaSuppression<FT, FT, FT, PreciseNMS<FT, FT, false, FT, EMap8, CubicInterpolator, Polar>> precise_nms_cubic(
        th_low_, th_high_, 3);

    ZeroCrossing<FT, FT, FT> fast_zc(th_low_, th_high_, 3);
    ZeroCrossing<FT, FT, FT, PreciseZC<FT, FT, FT, NCC_BASIC, EZCMap8, LinearInterpolator, Polar>> precise_zc_linear(
        th_low_, th_high_, 3);
    ZeroCrossing<FT, FT, FT, PreciseZC<FT, FT, FT, NCC_BASIC, EZCMap8, CubicInterpolator, Polar>> precise_zc_cubic(
        th_low_, th_high_, 3);

    this->process(src, fast_nms, fast_zc, "fast");
    this->process(src, precise_nms_linear, precise_zc_linear, "prec lin", true);
    this->process(src, precise_nms_cubic, precise_zc_cubic, "prec cub", true);
  }
};

template <class FT, template <class> class PT = cv::Point_>
class EntryTreshT : public Entry<FT, PT> {
  double th_{};

 public:
  EntryTreshT(const GroundTruth<FT, PT>& gt,
              const cv::Ptr<FilterI<uchar>>& fil,
              const std::string& n,
              int f = 0,
              double th = 0.3)
      : Entry<FT, PT>(gt, fil, n, f), th_(th) {}

  using Entry<FT, PT>::process;
  using MyData = typename Entry<FT, PT>::MyData;

  void process(const cv::Mat& src) override {
    this->filter->process(src);
    FilterResults results = this->filter->results();
    auto iter = results.find("mag");
    if (iter == results.end()) {
      return;
    }
    cv::Mat mag = iter->second.data;
    GlobalThreshold<FT, ThresholdUser<FT>> global_th_mag(iter->second.range.upper * th_);
    cv::Mat edge_pix;

    int runs = global_runs;
    int64 rt{}, tmp{};
    for (int i = 0; i != runs; ++i) {
      tmp = cv::getTickCount();
      cv::Mat mag_th = global_th_mag.process(mag);
      edge_pix = thinImage(mag_th);
      rt += cv::getTickCount() - tmp;
    }
    double rt_nms = (static_cast<double>(rt) * 1000.0 / cv::getTickFrequency()) / static_cast<double>(runs);

    std::vector<PT<FT>> edge_pts;
    rt = 0;
    tmp = 0;
    for (int i = 0; i != runs; ++i) {
      tmp = cv::getTickCount();
      edge_pts.clear();
      cv::findNonZero(edge_pix, edge_pts);
      rt += cv::getTickCount() - tmp;
    }
    double rt_spe = (static_cast<double>(rt) * 1000.0 / cv::getTickFrequency()) / static_cast<double>(runs);

    this->allData.emplace_back(MyData{"threshold", std::move(edge_pts), this->gt, rt_nms, rt_spe});
  }
};

class SpeApp : public EvalApp {
  using FT = double;
  using MyEntry = EntryT<FT, cv::Point_>;
  using MyEntryTresh = EntryTreshT<FT, cv::Point_>;
  using MyEntryPtr = EntryPtr<FT, cv::Point_>;
  std::vector<MyEntryPtr> entries_{};
  std::unique_ptr<GroundTruth<FT, cv::Point_>> gt;

 public:
  SpeApp(std::string name = "SpeApp",
         std::string description = "Subpixel precision evaluation tool",
         std::string version = "1.0.0")
      : EvalApp(std::move(name), std::move(description), std::move(version)) {}

  using ConsoleAppInterface::run;

  void defineArgs() {
    ConsoleApp::defineArgs();
    opts_.add_string("output", 'o', "Output folder", output_, false, "./results/spe");
    opts_.add_switch("prio", 'p', "Run as high prio process", run_high_prio_);
    opts_.add_switch("no-results", '\0', "Don't write results", no_results_);
    opts_.add_switch("write-visuals", '\0', "Write visual results", write_visuals_);
    opts_.add_switch("show-visuals", '\0', "Show visual results", show_visuals_);
  }

  void initEval() override {
    fs::create_directory(output_.c_str());

    gt = make_unique<GroundTruth<FT, cv::Point_>>(output_, show_visuals_, verbose_);

    entries_.push_back(
        MyEntryPtr(new MyEntryTresh(*gt, new DerivativeGradient<uchar, FT, FT, FT, SobelDerivative>, "TH")));

    entries_.push_back(MyEntryPtr(new MyEntry(*gt, new DerivativeGradient<uchar, FT, FT, FT, SobelDerivative>, "S3")));
    entries_.push_back(MyEntryPtr(new MyEntry(*gt, new LaplaceSimple<uchar, FT>(), "L3")));
    {
      DerivativeGradient<uchar, FT, FT, FT, SobelDerivative> tmp;
      tmp.process(gt->img);
      entries_.back()->optional_dir = tmp.direction();
    }

    entries_.push_back(MyEntryPtr(new MyEntry(
        *gt, new DerivativeGradient<uchar, FT, FT, FT, SobelDerivative>({NV("grad_kernel_size", 5)}), "S5")));
    entries_.push_back(MyEntryPtr(new MyEntry(*gt, new LaplaceCV<uchar, FT>(5), "L5")));
    {
      DerivativeGradient<uchar, FT, FT, FT, SobelDerivative> tmp({NV("grad_kernel_size", 5)});
      tmp.process(gt->img);
      entries_.back()->optional_dir = tmp.direction();
    }

    entries_.push_back(MyEntryPtr(new MyEntry(
        *gt, new DerivativeGradient<uchar, FT, FT, FT, SobelDerivative>({NV("grad_kernel_size", 7)}), "S7")));
    entries_.push_back(MyEntryPtr(new MyEntry(*gt, new LaplaceCV<uchar, FT>(7), "L7")));
    {
      DerivativeGradient<uchar, FT, FT, FT, SobelDerivative> tmp({NV("grad_kernel_size", 7)});
      tmp.process(gt->img);
      entries_.back()->optional_dir = tmp.direction();
    }

    entries_.push_back(MyEntryPtr(new MyEntry(
        *gt, new DerivativeGradient<uchar, FT, FT, FT, SobelDerivative>({NV("grad_kernel_size", 9)}), "S9")));
    entries_.push_back(MyEntryPtr(new MyEntry(*gt, new LaplaceCV<uchar, FT>(9), "L9")));
    {
      DerivativeGradient<uchar, FT, FT, FT, SobelDerivative> tmp({NV("grad_kernel_size", 9)});
      tmp.process(gt->img);
      entries_.back()->optional_dir = tmp.direction();
    }
  }

  template <typename DT>
  void printDefaultData(const DT& data, std::ostream& os, const std::string entry_name = "") {
    if (!entry_name.empty()) {
      os << entry_name << " ";
    }
    os << data.name << ";" << data.points.size() << ";" << data.outlier.size() << ";" << data.error.value() << ";"
       << data.error.stdDev() << ";" << data.runtime() << std::endl;
  }

  void printDefaultHeader(std::ostream& os) { os << "Method;Points;Outlier;Error;StdDev;Runtime(ms)" << std::endl; }

  template <typename ET>
  void printEntry(const ET& entry) {
    std::ofstream ofs, ofsd;
    std::string spe_precision_out1 = output_ + "/spe_precision_" + entry.name + ".csv";
    std::string spe_precision_out2 = output_ + "/spe_precision_" + entry.name + "_details.csv";
    ofs.open(spe_precision_out1.c_str());
    ofsd.open(spe_precision_out2.c_str());
    printDefaultHeader(ofs);
    ofsd << "Method";
    for (std::size_t i = 0; i != gt->segments.size(); ++i) {
      ofsd << ";Error " << i << ";StdDev " << i;
    }
    ofsd << std::endl;
    for (const auto& data : entry.allData) {
      printDefaultData(data, ofs);
      ofsd << data.name;
      for (const auto& err : data.error_lines) {
        ofsd << ";" << err.value() << ";" << err.stdDev();
      }
      ofsd << std::endl;
    }
  }

  void runEval() override {
    fs::path spe_precision_out(output_ + "/spe_precision_all.csv");

    std::ofstream ofs;
    if (!no_results_) {
      ofs.open(spe_precision_out.c_str());

      cout.precision(7);
      cout.setf(std::ios::fixed, std::ios::floatfield);
      ofs.precision(7);
      ofs.setf(std::ios::fixed, std::ios::floatfield);
    }
    if (verbose_) {
      cout << "Method\t\t\tPoints\tOutlier\tError\t\tStdDev\t\tRuntime(ms)" << std::endl;
    }
    if (!no_results_) {
      ofs << "Method;Points;Outlier;Error;StdDev;Runtime(ms)";
      for (std::size_t i = 0; i != gt->segments.size(); ++i) {
        ofs << ";Error " << i << ";StdDev " << i;
      }
      ofs << std::endl;
    }

    struct BestData {
      std::size_t entry_index{0};
      std::size_t data_index{std::string::npos};
      double err = std::numeric_limits<double>::max();
    };
    struct BestDataCat {
      BestData fast{};
      BestData precise_linear{};
      BestData precise_cubic{};
    };
    std::vector<BestDataCat> bestData;

    for (std::size_t i = 0; i != entries_.size(); ++i) {
      auto& entry = *entries_[i];
      entry.process();
      BestData best_fast{i};
      BestData best_precise_linear{i};
      BestData best_precise_cubic{i};
      for (std::size_t j = 0; j != entry.allData.size(); ++j) {
        const auto& data = entry.allData[j];
        if (data.name.find("fast") != std::string::npos && data.error.sum < best_fast.err) {
          best_fast.data_index = j;
          best_fast.err = data.error.sum;
        }
        if (data.name.find("prec lin") != std::string::npos && data.error.sum < best_precise_linear.err) {
          best_precise_linear.data_index = j;
          best_precise_linear.err = data.error.sum;
        }
        if (data.name.find("prec cub") != std::string::npos && data.error.sum < best_precise_cubic.err) {
          best_precise_cubic.data_index = j;
          best_precise_cubic.err = data.error.sum;
        }
        if (!no_results_) {
          ofs << entry.name << " " << data.name << ";" << data.points.size() << ";" << data.outlier.size() << ";"
              << data.error.value() << ";" << data.error.stdDev() << ";" << data.runtime();
          for (const auto& err : data.error_lines) {
            ofs << ";" << err.value() << ";" << err.stdDev();
          }
          ofs << std::endl;
        }

        if (verbose_) {
          cout << entry.name << " " << data.name << "\t" << data.points.size() << "\t" << data.outlier.size() << "\t"
               << data.error.value() << "\t" << data.error.stdDev() << std::endl;
        }

        if (write_visuals_) {
          std::string fn = entry.name + "_" + data.name;
          std::replace(fn.begin(), fn.end(), ' ', '_');
          cv::imwrite(output_ + "/" + fn + ".png", data.img);

          FilterResults results = entry.filter->results();
          auto iter = results.find("laplace");
          if (iter != results.end()) {
            cv::imwrite(output_ + "/" + fn + "_response.png", convertLaplace(iter->second.data));
          }
          iter = results.find("mag");
          if (iter != results.end()) {
            cv::imwrite(output_ + "/" + fn + "_response.png", convertMag(iter->second.data));
          }
        }
      }
      bestData.emplace_back(BestDataCat{best_fast, best_precise_linear, best_precise_cubic});
    }
    if (!no_results_) {
      ofs.close();
      ofs.clear();
      spe_precision_out = output_ + "/spe_precision_best.csv";
      ofs.open(spe_precision_out.c_str());
      printDefaultHeader(ofs);
      for (const auto& best : bestData) {
        if (best.fast.data_index != 0 && best.precise_linear.data_index != 0 && best.precise_cubic.data_index != 0) {
          printDefaultData(entries_[best.fast.entry_index]->allData[0], ofs, entries_[best.fast.entry_index]->name);
        }
        if (best.fast.data_index != std::string::npos) {
          printDefaultData(entries_[best.fast.entry_index]->allData[best.fast.data_index], ofs,
                           entries_[best.fast.entry_index]->name);
        }
        if (best.precise_linear.data_index != std::string::npos) {
          printDefaultData(entries_[best.precise_linear.entry_index]->allData[best.precise_linear.data_index], ofs,
                           entries_[best.precise_linear.entry_index]->name);
        }
        if (best.precise_cubic.data_index != std::string::npos) {
          printDefaultData(entries_[best.precise_cubic.entry_index]->allData[best.precise_cubic.data_index], ofs,
                           entries_[best.precise_cubic.entry_index]->name);
        }
      }
      ofs.close();

      printEntry(*entries_[1]);  // S3
      printEntry(*entries_[2]);  // L3
    }

    if (write_visuals_) {
      cv::imwrite(output_ + "/gt_lines.png", gt->drawGT());

      // static cv::Vec3b color[] = {cv::Vec3b(200, 50, 50),  cv::Vec3b(50, 200, 50),  cv::Vec3b(50, 50, 200),
      //                             cv::Vec3b(200, 200, 50), cv::Vec3b(200, 50, 200), cv::Vec3b(50, 200, 200),
      //                             cv::Vec3b(150, 100, 50), cv::Vec3b(100, 150, 200)};

      static cv::Vec3b color[] = {cv::Vec3b(50, 200, 50),  cv::Vec3b(50, 50, 200),  cv::Vec3b(200, 50, 50),
                                  cv::Vec3b(200, 200, 50), cv::Vec3b(100, 50, 00),  cv::Vec3b(50, 200, 200),
                                  cv::Vec3b(40, 90, 180),  cv::Vec3b(100, 150, 200)};

      std::vector<Polygon<FT, cv::Point_>> polys;
      polys.emplace_back(Polygon<FT, cv::Point_>{gt->segments});                                     // gt grün
      polys.emplace_back(Polygon<FT, cv::Point_>{entries_[0]->allData[0].lineSegsFromSegPoints()});  // th rot
      polys.emplace_back(Polygon<FT, cv::Point_>{entries_[1]->allData[0].lineSegsFromSegPoints()});  // nms blau
      polys.emplace_back(Polygon<FT, cv::Point_>{entries_[1]->allData[1].lineSegsFromSegPoints()});  // nms fast Cyan
      polys.emplace_back(Polygon<FT, cv::Point_>{entries_[7]->allData[2].lineSegsFromSegPoints()});  // nms türkies
      polys.emplace_back(Polygon<FT, cv::Point_>{entries_[2]->allData[0].lineSegsFromSegPoints()});  // zc gelb
      polys.emplace_back(
          Polygon<FT, cv::Point_>{entries_[2]->allData[1].lineSegsFromSegPoints()});  // zc fast dark orange
      polys.emplace_back(
          Polygon<FT, cv::Point_>{entries_[8]->allData[1].lineSegsFromSegPoints()});  // zc precise orange


      int scale = 500;
      int x = static_cast<int>(std::floor(polys[0].verticies()[3].x) - 1);
      int y = static_cast<int>(std::floor(polys[0].verticies()[3].y) - 1);

      cv::Mat tmp;
      gt->img(cv::Rect(x, y, 3, 3)).copyTo(tmp);
      cv::resize(tmp, tmp, cv::Size(3 * scale, 3 * scale), scale, scale, cv::INTER_NEAREST);
      cvtColor(tmp, tmp, CV_GRAY2RGB);

      for (std::size_t i = 0; i != polys.size(); ++i) {
        polys[i].scale(scale);
      }
      cv::Point_<FT> trans{static_cast<double>(-(x - 0.2) * scale), static_cast<double>(-y * scale)};

      std::cout << "x: " << x << ", " << -trans.x << "; y: " << y << ", " << -trans.y << std::endl;

      for (std::size_t i = 0; i != polys.size(); ++i) {
        polys[i].translate(trans);
        polys[i].draw(tmp, color[i]);
      }
      cv::imwrite(output_ + "/lines_high_res.png", tmp);
    }
  }
};

int main(int argc, char** argv) {
  SpeApp app("eval_spe_precision");
  return app.run(argc, argv);
}
