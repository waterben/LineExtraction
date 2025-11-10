#include <edge/nms.hpp>
#include <edge/spe.hpp>
#include <edge/zc.hpp>
#include <geometry/draw.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/gradient_adapter.hpp>
#include <imgproc/image_operator.hpp>
#include <imgproc/pc_lgf.hpp>
#include <imgproc/pc_matlab.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/susan.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/matlab_helpers.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>


#define WRITE_IMAGE_FILES
// #define SHOW_IMAGES

using namespace lsfm;
using namespace std;

template <class FT, template <class> class PT = cv::Point_>
struct GroundTruth {
  GroundTruth() : segments(), img() {
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

    // cv::resize(tmp, img, cv::Size(), scale, scale, cv::INTER_AREA); //-> large error, even with sub precision
    // cv::GaussianBlur(tmp, tmp, cv::Size(101, 101),20); // long runtime
    cv::blur(tmp, tmp, cv::Size(100, 100));
    cv::resize(tmp, img, cv::Size(), scale, scale, cv::INTER_NEAREST);
    poly.scale(scale);
    segments = poly.edges();
  }

  LineSegment2Vector<FT, PT> segments;
  cv::Mat img;

  FT error(const PT<FT>& p) const {
    FT e = std::numeric_limits<FT>::max();
    for_each(segments.begin(), segments.end(), [&](const LineSegment2<FT, PT>& seg) {
      if (!seg.inRangeTol(p, 1)) return;
      e = std::min(e, std::abs(seg.distance(p)));
    });
    return e;
  }

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
      lsfm::set(out, cv::Point(static_cast<int>(getX(p)), static_cast<int>(getY(p))), color[6]);
    else
      lsfm::set(out, cv::Point(static_cast<int>(getX(p)), static_cast<int>(getY(p))), color[l]);
  }
};

template <class FT, template <class> class PT>
void outputSet(const std::string& name,
               const GroundTruth<FT, PT>& gt,
               const std::vector<PT<FT>>& points,
               bool show = true) {
  cv::Mat out(gt.img.size(), CV_8UC3);
  out.setTo(0);

  int outlier = 0;
  FT err = 0, errSqr = 0;
  for_each(points.begin(), points.end(), [&](const PT<FT>& p) {
    FT e;
    size_t l = gt.line(p, e);
    if (e > 1)
      ++outlier;
    else {
      err += e;
      errSqr += e * e;
    }
    if (getX(p) > -1 && getX(p) < gt.img.cols && getY(p) > -1 && getY(p) < gt.img.rows) gt.draw(out, p, l, e);
  });
  err /= static_cast<FT>(points.size() - static_cast<size_t>(outlier));
  errSqr /= static_cast<FT>(points.size() - static_cast<size_t>(outlier));
  std::cout << name << " - points: " << points.size() << ", outlier: " << outlier << ", mean error: " << err
            << ", std deviation: " << sqrt(errSqr - err * err) << std::endl;
  if (show) cv::imshow(name.c_str(), out);
}

int main() {
  typedef double FT;
  DerivativeGradient<uchar, FT, FT, FT, ScharrDerivative> grad(0, 255);
  LaplaceCV<uchar, FT> laplace(5, 0, 255);
  // LoG<uchar, FT> laplace(5, 1,1,0, 255);
  NonMaximaSuppression<FT, FT, FT, FastNMS8<FT, FT, FT>> nms;
  NonMaximaSuppression<FT, FT, FT, PreciseNMS<FT, FT, false, FT, EMap8, CubicInterpolator, Polar>> pnms;
  ZeroCrossing<FT, FT, FT, FastZC<FT, FT, FT>> zc;
  ZeroCrossing<FT, FT, FT, PreciseZC<FT, FT, FT, NCC_BASIC, EZCMap8, CubicInterpolator, Polar>> pzc;
  PixelEstimator<FT, cv::Point_<FT>> pe;
  // CoGEstimate, LinearEstimate, QuadraticEstimate, SobelEstimate, LinearInterpolator, CubicInterpolator
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, QuadraticEstimate, CubicInterpolator>> spe;
  PixelEstimator<FT, cv::Point_<FT>, SubPixelEstimator<FT, FT, cv::Point_, SobelZCEstimate, CubicInterpolator>> spezc;
  GroundTruth<FT, cv::Point_> gt;

  grad.process(gt.img);
  nms.process(grad);
  IndexVector idxs = nms.hysteresis_edgels();
  std::vector<cv::Point_<FT>> points, pointsSp, pointsSpDir;
  pe.convert(idxs, points, grad.magnitude(), nms.directionMap());
  spe.convert(idxs, pointsSp, grad.magnitude(), nms.directionMap());
  spe.convertDir(idxs, pointsSpDir, grad.magnitude(), grad.direction());


  outputSet("fnms_pe", gt, points);
  outputSet("fnms_spe", gt, pointsSp);
  outputSet("fnms_spe_dir", gt, pointsSpDir);

  cv::waitKey();

  pnms.process(grad);
  idxs = pnms.hysteresis_edgels();
  pe.convert(idxs, points, grad.magnitude(), pnms.directionMap());
  spe.convert(idxs, pointsSp, grad.magnitude(), pnms.directionMap());
  spe.convertDir(idxs, pointsSpDir, grad.magnitude(), grad.direction());

  outputSet("pnms_pe", gt, points);
  outputSet("pnms_spe", gt, pointsSp);
  outputSet("pnms_spe_dir", gt, pointsSpDir);

  cv::waitKey();

  laplace.process(gt.img);
  zc.process(laplace);
  idxs = zc.hysteresis_edgels();
  pe.convert(idxs, points, laplace.laplace(), zc.directionMap());
  spezc.convert(idxs, pointsSp, laplace.laplace(), zc.directionMap());
  spezc.convertDir(idxs, pointsSpDir, laplace.laplace(), grad.direction());

  // cv::imshow("gt", gt.img);
  outputSet("fzc_pe", gt, points);
  outputSet("fzc_spe", gt, pointsSp);
  outputSet("fzc_spe_dir", gt, pointsSpDir);

  cv::waitKey();

  zc.processG(laplace, grad);
  idxs = zc.hysteresis_edgels();
  pe.convert(idxs, points, laplace.laplace(), zc.directionMap());
  spezc.convert(idxs, pointsSp, laplace.laplace(), zc.directionMap());
  spezc.convertDir(idxs, pointsSpDir, laplace.laplace(), grad.direction());

  // cv::imshow("gt", gt.img);
  outputSet("fzcg_pe", gt, points);
  outputSet("fzcg_spe", gt, pointsSp);
  outputSet("fzcg_spe_dir", gt, pointsSpDir);
  cv::waitKey();

  pzc.process(laplace);
  idxs = pzc.hysteresis_edgels();
  pe.convert(idxs, points, laplace.laplace(), pzc.directionMap());
  spezc.convert(idxs, pointsSp, laplace.laplace(), pzc.directionMap());
  spezc.convertDir(idxs, pointsSpDir, laplace.laplace(), grad.direction());

  // cv::imshow("gt", gt.img);
  outputSet("pzc_pe", gt, points);
  outputSet("pzc_spe", gt, pointsSp);
  outputSet("pzc_spe_dir", gt, pointsSpDir);

  cv::waitKey();

  pzc.processG(laplace, grad);
  idxs = pzc.hysteresis_edgels();
  pe.convert(idxs, points, laplace.laplace(), pzc.directionMap());
  spezc.convert(idxs, pointsSp, laplace.laplace(), pzc.directionMap());
  spezc.convertDir(idxs, pointsSpDir, laplace.laplace(), grad.direction());

  // cv::imshow("gt", gt.img);
  outputSet("pzcg_pe", gt, points);
  outputSet("pzcg_spe", gt, pointsSp);
  outputSet("pzcg_spe_dir", gt, pointsSpDir);

  cv::imshow("gt", gt.img);
  cv::waitKey();
  return 0;
}
