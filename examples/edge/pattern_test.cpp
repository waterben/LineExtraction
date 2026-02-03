/// @file pattern_test.cpp
/// @brief Pattern-based edge segment detection demonstration.
///
/// Demonstrates the EsdPattern detector which groups edge pixels into
/// patterns (consistent edge directions) before linking into segments.
/// This approach is more robust to noise and provides better
/// segmentation at junctions and corners.
///
/// @usage ./pattern_test [image_path]
/// @param image_path Optional path to input image (default: windmill.jpg)

#include <edge/nms.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <utility/test_images.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>

#define EDGE_THICK_CHECK
#define GRADIENT_MAX_CHECK
#define CORNER_CHECK
#include <edge/edge_drawing.hpp>
#include <edge/edge_linking.hpp>
#include <edge/edge_pattern.hpp>
#include <edge/edge_simple.hpp>

using namespace std;
using namespace lsfm;
using namespace cv;

inline void setPixel(cv::Mat& dst, index_type idx, const Vec3b& color) { dst.ptr<cv::Vec3b>()[idx] = color; }

inline void setPixel(cv::Mat& dst, const cv::Point& idx, const Vec3b& color) { dst.at<cv::Vec3b>(idx) = color; }

inline void setPixel(cv::Mat& dst, const lsfm::Vec2i& idx, const Vec3b& color) {
  dst.at<cv::Vec3b>(idx.y(), idx.x()) = color;
}

inline void setCircle(cv::Mat& dst, index_type idx, const Vec3b& color, int r) {
  circle(dst,
         cv::Point(static_cast<int>(idx % static_cast<index_type>(dst.cols)),
                   static_cast<int>(idx / static_cast<index_type>(dst.cols))),
         r, Scalar(color[0], color[1], color[2]));
}

inline void setCircle(cv::Mat& dst, const cv::Point& idx, const Vec3b& color, int r) {
  circle(dst, idx, r, Scalar(color[0], color[1], color[2]));
}

inline void setCircle(cv::Mat& dst, const lsfm::Vec2i& idx, const Vec3b& color, int r) {
  cv::circle(dst, cv::Point(idx.x(), idx.y()), r, Scalar(color[0], color[1], color[2]));
}


template <class EDGE>
void showPattern(EDGE& edge, const cv::Mat& src, const std::string& name, bool circles = true) {
  Mat edgeImg;
  cvtColor(src, edgeImg, cv::COLOR_GRAY2BGR);
  cv::RNG rng(static_cast<uint64_t>(time(nullptr)));

  for_each(edge.patternSegments().begin(), edge.patternSegments().end(), [&](const EdgeSegment& seg) {
    for (size_t p = seg.begin(); p != seg.end(); ++p) {
      Vec3b color(static_cast<uchar>(20 + rng.uniform(0, 225)), static_cast<uchar>(20 + rng.uniform(0, 225)),
                  static_cast<uchar>(20 + rng.uniform(0, 225)));
      for (size_t i = edge.patterns()[p].begin(); i != edge.patterns()[p].end(); ++i)
        setPixel(edgeImg, edge.points()[i], color);
    }

    if (circles) {
      if (seg.closed())
        setCircle(edgeImg, edge.points()[edge.patterns()[seg.begin()].begin()], Vec3b(255, 0, 0), 2);
      else if (seg.reverse())
        setCircle(edgeImg, edge.points()[edge.patterns()[seg.end() - 1].end() - 1], Vec3b(0, 0, 255), 2);
      else
        setCircle(edgeImg, edge.points()[edge.patterns()[seg.begin()].begin()], Vec3b(255, 255, 255), 2);
    }
  });

  imshow("edge " + name, edgeImg);
}


int main(int argc, char** argv) {
  lsfm::TestImages::init(argv[0]);
  std::string filename = argc >= 2 ? argv[1] : lsfm::TestImages::windmill();

  cv::Mat src = cv::imread(filename, IMREAD_GRAYSCALE);
  if (src.empty()) {
    cout << "Can not open " << filename << endl;
    return -1;
  }

  if (src.channels() > 1) cvtColor(src, src, cv::COLOR_BGR2GRAY);

  GaussianNoiseOperator noise(10);
  // noise.apply(src);
  GaussianBlur(src, src, cv::Size(5, 5), 0.6);

  // imshow("img", src);

  float th_low = 0.004f, th_high = 0.012f;
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms(th_low, th_high);


  EsdPattern<int, 8, true> pattern(10, 3, 3, static_cast<int>(static_cast<float>(sobel.magnitudeThreshold(th_low))), 2);


  sobel.process(src);
  nms.process(sobel);
  pattern.detect(sobel, nms);
  showPattern(pattern, src, "pattern");

  waitKey();

  return 0;
}
