/// @file lsd_edge.cpp
/// @brief Edge-based LSD with pattern detection.
///
/// Demonstrates edge-based line segment detection:
/// - LsdEL: Line detection from edge linking
/// - LsdEP: Line detection from edge patterns
/// - Comparison of edge-based approaches
/// - Visualization with colored line segments
///
/// @usage ./lsd_edge [image_path]
/// @param image_path Optional path to input image (default: windmill.jpg)

#include <geometry/draw.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility/test_images.hpp>

#include <iostream>
#define GRADIENT_MAX_CHECK
#define EDGE_THICK_CHECK
#include <edge/edge_drawing.hpp>
#include <edge/edge_pattern.hpp>
#include <edge/edge_simple.hpp>
#include <lsd/lsd_el.hpp>
#include <lsd/lsd_ep.hpp>


using namespace lsfm;
using namespace std;

template <class FT>
struct Entry {
  Entry(const cv::Ptr<LsdBase<FT>>& a = cv::Ptr<LsdBase<FT>>(), const std::string& b = std::string(), double c = 0)
      : lsd(a), name(b), time(c) {}

  cv::Ptr<LsdBase<FT>> lsd;
  std::string name;
  double time;
};

static void help() {
  cout << "\nThis program demonstrates lsd.\n"
          "Usage:\n"
          "./test_lsd <image_name>, Default is ../images/office1_low.jpg\n"
       << endl;
}

template <typename FT>
void showData(const cv::Mat& src, const Entry<FT>& e) {
  std::cout << e.name << ": " << e.time << " ms" << std::endl;

  cv::Mat img;
  cvtColor(src, img, cv::COLOR_GRAY2BGR);

  cv::RNG rng(static_cast<uint64_t>(cv::getTickCount()));

  for_each(e.lsd->lineSegments().begin(), e.lsd->lineSegments().end(), [&](const LineSegment<FT>& l) {
    cv::Vec3b color(static_cast<unsigned char>(20 + rng.uniform(0, 225)),
                    static_cast<unsigned char>(20 + rng.uniform(0, 225)),
                    static_cast<unsigned char>(20 + rng.uniform(0, 225)));
    cv::Scalar scolor(color[0], color[1], color[2]);
    line(img, l, scolor, 1, 8, 10.0, 3.0);
    // Vec2<FT> p1 = l.normalLineDist(0, l.centerPoint()), p2 = l.normalLineDist(10, l.centerPoint());
    // line(img, p1, p2, cv::Scalar(0, 0, 255));
  });
  cv::imshow(e.name, img);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
  lsfm::TestImages::init(argv[0]);
  std::string filename = argc >= 2 ? argv[1] : lsfm::TestImages::windmill();

  cv::Mat src = cv::imread(filename, 0);
  if (src.empty()) {
    help();
    cout << "Can not open " << filename << endl;
    return -1;
  }

  if (src.channels() != 1) cvtColor(src, src, cv::COLOR_RGB2GRAY);

  // blur(src, src, Size(3, 3));
  GaussianBlur(src, src, cv::Size(3, 3), 0.6);

  typedef float FT;
  typedef Vec2i PT;
  typedef RamerSplit<FT, PT> Split;
  // typedef ExtRamerSplit<SimpleMerge<ExtSplitCheck<FT, PT>>> Split;
  typedef FitLine<EigenFit<FT, PT>> Fit;
  typedef DerivativeGradient<uchar, short, int, FT, SobelDerivative, QuadraticMagnitude> Grad;
  typedef NonMaximaSuppression<short, int, FT> Nms;
  typedef EdgeSourceGRAD<Grad, Nms> EdgeSource;
  typedef NfaBinom<short, FT, index_type> Nfa;
  typedef PixelEstimator<FT, PT> Conv;

  std::vector<Entry<FT>> vlsd;

  vlsd.push_back(Entry<FT>(
      new LsdEL<FT, Vec2, PT, EdgeSource, EsdSimple<int>, Nfa, Conv, Split, Fit>(0.004f, 0.012f, 10, 3, 10), "lsd es"));

  vlsd.push_back(
      Entry<FT>(new LsdEL<FT, Vec2, PT, EdgeSource, EsdDrawing<int>, Nfa, Conv, Split, Fit>(0.004f, 0.012f, 10, 3, 10),
                "lsd ed"));

  vlsd.push_back(Entry<FT>(
      new LsdEL<FT, Vec2, PT, EdgeSource, EsdLinking<int, 8, false>, Nfa, Conv, Split, Fit>(0.004f, 0.012f, 10, 3, 10),
      "lsd el"));

  vlsd.push_back(Entry<FT>(
      new LsdEL<FT, Vec2, PT, EdgeSource, EsdLinking<int, 8, true>, Nfa, Conv, Split, Fit>(0.004f, 0.012f, 10, 3, 10),
      "lsd elc"));

  vlsd.push_back(Entry<FT>(
      new LsdEL<FT, Vec2, PT, EdgeSource, EsdPattern<int, 8, false>, Nfa, Conv, Split, Fit>(0.004f, 0.012f, 10, 3, 10),
      "lsd elp"));

  vlsd.push_back(Entry<FT>(
      new LsdEL<FT, Vec2, PT, EdgeSource, EsdPattern<int, 8, true>, Nfa, Conv, Split, Fit>(0.004f, 0.012f, 10, 3, 10),
      "lsd elpc"));

  vlsd.push_back(
      Entry<FT>(new LsdEP<FT, Vec2, false, PT, EdgeSource, Conv, Split, Fit>(0.004f, 0.012f, 10, 3, 10, 3), "lsd ep"));

  vlsd.push_back(
      Entry<FT>(new LsdEP<FT, Vec2, true, PT, EdgeSource, Conv, Split, Fit>(0.004f, 0.012f, 10, 3, 10, 3), "lsd epc"));


  int runs = 10;
  std::for_each(vlsd.begin(), vlsd.end(), [&runs, &src](Entry<FT>& entry) {
    for (int i = 0; i != runs; ++i) {
      double start = double(cv::getTickCount());
      entry.lsd->detect(src);
      entry.time += (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency() / runs;
    }
    showData(src, entry);
  });

  cv::waitKey();

  return 0;
}
