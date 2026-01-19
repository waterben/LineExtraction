#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>


#define EDGE_THICK_CHECK
#define GRADIENT_MAX_CHECK
#include <edge/edge_drawing.hpp>
#include <edge/edge_linking.hpp>
#include <edge/edge_simple.hpp>
#include <edge/nfa.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;

template <class NMS, class GRAD, class EDGE, class NFA>
void testNfa(GRAD& grad, NMS& nms, EDGE& edge, const NFA& nfa, const std::string& name) {
  std::vector<float> n;
  EdgeSegmentVector out;
  edge.detect(grad, nms);
  int runs = 20;
  int64 rt = 0, tmp;
  for (int i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    nfa.eval(edge, out, n);
    rt += cv::getTickCount() - tmp;
  }

  std::cout << "nfa - " << name << ": "
            << (static_cast<double>(rt) * 1000.0 / static_cast<double>(cv::getTickFrequency())) / runs << std::endl;
}

inline void setPixel(cv::Mat& dst, index_type idx, const Vec3b& color) { dst.ptr<cv::Vec3b>()[idx] = color; }

inline void setPixel(cv::Mat& dst, cv::Point idx, const Vec3b& color) { dst.at<cv::Vec3b>(idx) = color; }

inline void setPixel(cv::Mat& dst, lsfm::Vec3i idx, const Vec3b& color) { dst.at<cv::Vec3b>(idx.y(), idx.x()) = color; }

inline void setCircle(cv::Mat& dst, index_type idx, const Vec3b& color, int r) {
  circle(dst,
         cv::Point(static_cast<int>(idx % static_cast<index_type>(dst.cols)),
                   static_cast<int>(idx / static_cast<index_type>(dst.cols))),
         r, Scalar(color[0], color[1], color[2]));
}

inline void setCircle(cv::Mat& dst, cv::Point idx, const Vec3b& color, int r) {
  circle(dst, idx, r, Scalar(color[0], color[1], color[2]));
}

inline void setCircle(cv::Mat& dst, lsfm::Vec3i idx, const Vec3b& color, int r) {
  cv::circle(dst, cv::Point(idx.x(), idx.y()), r, Scalar(color[0], color[1], color[2]));
}

template <class PT>
void drawSegment(cv::Mat img, const EdgeSegment& seg, const std::vector<PT>& v, cv::Vec3b& color) {
  for (size_t i = seg.begin(); i != seg.end(); ++i) setPixel(img, v[i], color);
}


template <class EDGE, class NFA>
void showNfa(EDGE& edge, const cv::Mat& src, const NFA& nfa, const std::string& name, bool circles = true) {
  Mat edgeImg;
  cvtColor(src, edgeImg, cv::COLOR_GRAY2BGR);
  cv::RNG rng(static_cast<uint64_t>(time(nullptr)));

  std::vector<float> n;
  EdgeSegmentVector out;
  nfa.eval(edge, out, n);

  std::cout << "removed segments: " << edge.segments().size() - out.size() << std::endl;

  Vec3b removed(255, 255, 255);
  for_each(edge.segments().begin(), edge.segments().end(), [&](const EdgeSegment& seg) {
    for (size_t i = seg.begin(); i != seg.end(); ++i) setPixel(edgeImg, edge.points()[i], removed);
  });

  for_each(out.begin(), out.end(), [&](const EdgeSegment& seg) {
    Vec3b color(static_cast<uchar>(20 + rng.uniform(0, 225)), static_cast<uchar>(20 + rng.uniform(0, 225)),
                static_cast<uchar>(20 + rng.uniform(0, 225)));
    drawSegment(edgeImg, seg, edge.points(), color);

    if (circles) {
      if (seg.closed())
        setCircle(edgeImg, edge.points()[seg.begin()], Vec3b(255, 0, 0), 2);
      else if (seg.reverse())
        setCircle(edgeImg, edge.points()[seg.end() - 1], Vec3b(0, 0, 255), 2);
      else
        setCircle(edgeImg, edge.points()[seg.begin()], Vec3b(255, 255, 255), 2);
    }
  });

  imshow("nfa " + name, edgeImg);

  if (n.size() > 20) {
    std::vector<std::pair<size_t, float>> idxVec;
    for (size_t i = 0; i != n.size(); ++i) idxVec.push_back(std::pair<size_t, float>(i, n[i]));

    std::sort(idxVec.begin(), idxVec.end(),
              [](const std::pair<size_t, float>& a, const std::pair<size_t, float>& b) { return a.second > b.second; });

    cvtColor(src, edgeImg, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i != 20; ++i) {
      Vec3b color(static_cast<uchar>(20 + rng.uniform(0, 225)), static_cast<uchar>(20 + rng.uniform(0, 225)),
                  static_cast<uchar>(20 + rng.uniform(0, 225)));
      drawSegment(edgeImg, out[idxVec[i].first], edge.points(), color);
    }

    imshow("nfa top 20 " + name, edgeImg);
  }
}


int main(int argc, char** argv) {
  // const char* filename = argc >= 2 ? argv[1] : "../../images/circle.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/circle2.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/hall2_low.JPG";
  const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.JPG";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/lopez.png";

  cv::Mat src = cv::imread(filename, IMREAD_GRAYSCALE);
  if (src.empty()) {
    std::cout << "Can not open " << filename << endl;
    return -1;
  }

  if (src.channels() > 1) cvtColor(src, src, cv::COLOR_BGR2GRAY);

  GaussianNoiseOperator noise(5);
  // noise.apply(src);
  GaussianBlur(src, src, cv::Size(5, 5), 0.6);

  // imshow("img",src);
  lsfm::Vec2i a(1, 2);
  lsfm::Vec2f b(static_cast<float>(getX(a)), static_cast<float>(getY(a)));

  float th_low = 0.004f, th_high = 0.012f;
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms(th_low, th_high);


  NfaContrast<int, float, index_type, std::map<int, float>> nfac(8);
  NfaBinom<short, float, index_type> nfab(8, 1.0 / 8);
  NfaBinom2<short, float, index_type> nfab2(8, 1.0 / 8);

  EsdSimple<int> simple;
  EsdDrawing<int> draw(10, 3, static_cast<int>(static_cast<float>(sobel.magnitudeThreshold(th_low))));
  EsdLinking<int> link(10, 3, 3, static_cast<int>(static_cast<float>(sobel.magnitudeThreshold(th_low))));

  sobel.process(src);
  nfac.update(sobel.magnitude());
  nfab.update(sobel.gx(), sobel.gy());
  nfab2.update(sobel.gx(), sobel.gy());
  nms.process(sobel);

  testNfa(sobel, nms, simple, nfac, "simple c");
  showNfa(simple, src, nfac, "simple c");

  testNfa(sobel, nms, simple, nfab, "simple b");
  showNfa(simple, src, nfab, "simple b");

  testNfa(sobel, nms, draw, nfac, "draw c");
  showNfa(draw, src, nfac, "draw c");

  testNfa(sobel, nms, draw, nfab, "draw b");
  showNfa(draw, src, nfab, "draw b");

  testNfa(sobel, nms, link, nfac, "link c");
  showNfa(link, src, nfac, "link c");

  testNfa(sobel, nms, link, nfab, "link b");
  showNfa(link, src, nfab, "link b");

  testNfa(sobel, nms, link, nfab2, "link b2");
  showNfa(link, src, nfab2, "link b2");

  waitKey();

  return 0;
}
