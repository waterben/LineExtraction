#include <filesystem>
#include <edge/nfa.hpp>
#include <edge/nms.hpp>
#include <edge/threshold.hpp>
#include <edge/draw.hpp>
#include <geometry/draw.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <eval/eval_app.hpp>


#define EDGE_THICK_CHECK
#define GRADIENT_MAX_CHECK
#define CORNER_CHECK

#include <edge/edge_drawing.hpp>
#include <edge/edge_linking.hpp>
#include <edge/edge_pattern.hpp>
#include <edge/edge_simple.hpp>
#include <edge/nfa.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>


using namespace std;
using namespace lsfm;
using namespace cv;
namespace fs = std::filesystem;


class EdgeSegApp : public EvalApp {
  cv::Mat img{};

 public:
  EdgeSegApp(std::string name = "EdgeSegApp",
             std::string description = "Edge segment example processing",
             std::string version = "1.0.0")
      : EvalApp(std::move(name), std::move(description), std::move(version)) {}

  using ConsoleAppInterface::run;

  cv::RNG rng{time(0)};

  void defineArgs() {
    ConsoleApp::defineArgs();
    opts_.add_string("input", 'i', "Input file", input_, false, "../../images/windmill.jpg");
    opts_.add_string("output", 'o', "Output folder", output_, false, "./results/edge_seg_examples");
    opts_.add_switch("prio", 'p', "Run as high prio process", run_high_prio_);
    opts_.add_switch("no-results", '\0', "Don't write results", no_results_);
    opts_.add_switch("write-visuals", '\0', "Write visual results", write_visuals_);
    opts_.add_switch("show-visuals", '\0', "Show visual results", show_visuals_);
  }

  template <typename PV>
  void visEdge(const std::string& name,
               const EdgeSegmentVector& segments,
               const PV& points,
               cv::Mat& out,
               bool circles = false) {
    for_each(segments.begin(), segments.end(), [&](const EdgeSegment& seg) {
      Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
      for (size_t i = seg.begin(); i != seg.end(); ++i) setPixel(out, points[i], color);

      if (circles) {
        if (seg.closed()) setCircle(out, points[seg.begin()], Vec3b(0, 255, 0), 2);
        // else if (seg.reverse())
        //   setCircle(out, points[seg.end() - 1], Vec3b(0, 0, 255), 2);
        // else
        //   setCircle(out, points[seg.begin()], Vec3b(255, 255, 255), 2);
      }
    });

    if (show_visuals_) {
      imshow(name, out);
    }

    if (write_visuals_) {
      cv::imwrite(output_ + "/" + name + ".png", out);
    }
  }

  template <class EDGE>
  void visEdgeMat(const std::string& name, EDGE& edge, const cv::Mat& src, bool circles = false, bool img_bg = true) {
    cv::Mat edge_img;
    if (img_bg) {
      edge_img = src.clone();
    } else {
      edge_img = Mat::zeros(src.size(), CV_8UC3);
    }
    if (edge_img.type() == CV_8UC1) {
      cvtColor(edge_img, edge_img, cv::COLOR_GRAY2BGR);
    }
    visEdge(name, edge.segments(), edge.points(), edge_img, circles);
  }

  template <class EDGE, class NFA>
  void visNfa(const std::string& name,
              EDGE& edge,
              const cv::Mat& src,
              const NFA& nfa,
              bool circles = false,
              bool img_bg = false,
              std::size_t top = 0) {
    typedef typename EDGE::point_type point_type;

    std::vector<float> n;
    EdgeSegmentVector out;
    nfa.eval(edge, out, n);

    cv::Mat edge_img;
    if (img_bg) {
      edge_img = src.clone();
    } else {
      edge_img = Mat::zeros(src.size(), CV_8UC3);
    }
    if (edge_img.type() == CV_8UC1) {
      cvtColor(edge_img, edge_img, cv::COLOR_GRAY2BGR);
    }
    cv::Mat top_edges = edge_img.clone();

    std::cout << "removed segments: " << edge.segments().size() - out.size() << std::endl;

    cv::Mat org_edges = edge_img.clone();
    Vec3b removed(255, 255, 255);
    for_each(edge.segments().begin(), edge.segments().end(), [&](const EdgeSegment& seg) {
      for (size_t i = seg.begin(); i != seg.end(); ++i) setPixel(org_edges, edge.points()[i], removed);
    });
    visEdge("org_" + name, out, edge.points(), org_edges, circles);
    visEdge(name, out, edge.points(), edge_img, circles);


    if (top > 0) {
      std::vector<std::pair<size_t, float>> idxVec;
      for (size_t i = 0; i != n.size(); ++i) idxVec.push_back(std::pair<size_t, float>(i, n[i]));

      std::sort(idxVec.begin(), idxVec.end(), [](const std::pair<size_t, float>& a, const std::pair<size_t, float>& b) {
        return a.second > b.second;
      });

      top = std::min(top, n.size());
      for (size_t i = 0; i != top; ++i) {
        Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
        drawSegment(top_edges, out[idxVec[i].first], edge.points(), color);
      }
      std::string top_name = "top" + std::to_string(top) + "_" + name;

      if (show_visuals_) {
        imshow(top_name, top_edges);
      }

      if (write_visuals_) {
        cv::imwrite(output_ + "/" + top_name + ".png", top_edges);
      }
    }
  }

  template <typename GradT>
  void processTh(const std::string& name, GradT& grad, float th_low, float th_high, int min_pix = 3, int max_gap = 3) {
    using GradType = typename GradT::grad_type;
    using MagType = typename GradT::mag_type;
    NonMaximaSuppression<GradType, MagType> nms(th_low, th_high);
    nms.process(grad);

    auto nms_hist = nms.hysteresisBinary();

    if (show_visuals_) {
      imshow("nms " + name, nms_hist);
    }

    if (write_visuals_) {
      cv::imwrite(output_ + "/nms_" + name + ".png", nms_hist);
    }

    EsdSimple<int> simple(min_pix);
    simple.detect(grad, nms);
    visEdgeMat("simple_" + name, simple, img);

    EsdDrawing<int> draw(min_pix, 3, grad.magnitudeThreshold(th_low));
    draw.detect(grad, nms);
    visEdgeMat("draw_" + name, draw, img);

    EsdLinking<int> link(min_pix, max_gap, 3, grad.magnitudeThreshold(th_low));
    link.detect(grad, nms);
    visEdgeMat("link_" + name, link, img);

    EsdLinking<int, 8, true> linkc(min_pix, max_gap, 3, grad.magnitudeThreshold(th_low));
    linkc.detect(grad, nms);
    visEdgeMat("linkc_" + name, linkc, img);

    EsdPattern<int> pattern(min_pix, max_gap, 3, grad.magnitudeThreshold(th_low));
    pattern.detect(grad, nms);
    visEdgeMat("pattern_" + name, pattern, img);

    EsdPattern<int, 8, true> patternc(min_pix, max_gap, 3, grad.magnitudeThreshold(th_low));
    patternc.detect(grad, nms);
    visEdgeMat("patternc_" + name, patternc, img);
  }

  template <typename GradT>
  void processNfa(const std::string& name,
                  const GradT& grad,
                  float eps,
                  float th_low,
                  float th_high,
                  int min_pix = 3,
                  bool draw_top = true) {
    using GradType = typename GradT::grad_type;
    using MagType = typename GradT::mag_type;
    using DirType = typename GradT::dir_type;
    NonMaximaSuppression<GradType, MagType, DirType> nms(th_low, th_high);
    nms.process(grad);

    EsdLinking<int, 8, true> linkc(min_pix, 3, 3, grad.magnitudeThreshold(th_low));
    linkc.detect(grad, nms);

    NfaContrast<int, float, index_type, std::map<int, float>> nfac(eps);
    NfaBinom<short, float, index_type> nfab(eps);
    NfaBinom2<short, float, index_type> nfab2(eps);

    nfac.update(grad.magnitude());
    nfab.update(grad.gx(), grad.gy());
    nfab2.update(grad.gx(), grad.gy());

    std::size_t top = 25;

    visNfa("nfac_" + name, linkc, img, nfac, false, true, draw_top ? top : 0);
    visNfa("nfab_" + name, linkc, img, nfab, false, true, draw_top ? top : 0);
    visNfa("nfab2_" + name, linkc, img, nfab2, false, true, draw_top ? top : 0);
  }

  void initEval() override {
    if (write_visuals_ || !no_results_) {
      fs::create_directory(output_.c_str());
    }

    cout.precision(5);
    cout.setf(std::ios::fixed, std::ios::floatfield);

    img = cv::imread(input_, IMREAD_GRAYSCALE);
    if (img.empty()) {
      throw std::runtime_error("Can not open " + input_);
    }

    if (img.channels() > 1) cvtColor(img, img, cv::COLOR_BGR2GRAY);
  }


  void runEval() override {
    // Generate edge responses
    DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
    sobel.process(img);

    DerivativeGradient<uchar, short, float> sobel_float;
    sobel_float.process(img);
    auto mag = sobel_float.magnitude();

    ThresholdOtsu<float, 256> otsu_mag(sobel_float.magnitudeRange().upper);
    float otsu_th = otsu_mag.process(mag);


    float otsu_th_high = otsu_th / sobel_float.magnitudeRange().upper;
    float otsu_th_low = otsu_th_high / 3;

    if (verbose_) {
      std::cout << "otsu_mag: max: " << sobel_float.magnitudeRange().upper << ", otsu_th: " << otsu_th
                << ", otsu_th normalized: " << otsu_th_high << std::endl;
    }

    if (show_visuals_) {
      imshow("Sobel Magnitude", mag / sobel_float.magnitudeRange().upper);
    }

    if (write_visuals_) {
      cv::imwrite(output_ + "/sobel_mag.png", mag / sobel_float.magnitudeRange().upper * 255.0);
    }

    processTh("otsu", sobel, otsu_th_low, otsu_th_high);
    processTh("low", sobel, 0.03f, 0.01f);
    processTh("high", sobel, 0.3f, 0.1f);
    processTh("low_mpix10", sobel, 0.03f, 0.01f, 10);
    processTh("otsu_mpix10", sobel, otsu_th_low, otsu_th_high, 10);

    processNfa("low_eps", sobel, -2, 0.03f, 0.01f);
    processNfa("normal_eps", sobel, 0, 0.03f, 0.01f);
    processNfa("high_eps", sobel, 2, 0.03f, 0.01f);
    processNfa("normal_otsu_eps", sobel, 0, otsu_th_low, otsu_th_high);

    waitKey();
  }
};

int main(int argc, char** argv) {
  EdgeSegApp app("thesis_edge_segment_examples");
  return app.run(argc, argv);
}
