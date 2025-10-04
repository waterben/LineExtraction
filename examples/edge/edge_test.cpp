#include <edge/nms.hpp>
#include <geometry/draw.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/image_operator.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>


#define EDGE_THICK_CHECK
#define GRADIENT_MAX_CHECK
#define CORNER_CHECK
// #define DRAW_MODE
#include <edge/edge_drawing.hpp>
#include <edge/edge_linking.hpp>
#include <edge/edge_pattern.hpp>
#include <edge/edge_simple.hpp>


using namespace std;
using namespace lsfm;
using namespace cv;

template <class NMS, class GRAD, class EDGE>
void testEdge(GRAD& grad, NMS& nms, EDGE& edge, const std::string& name) {
  int runs = 200;
  int64 rt = 0, tmp;
  for (int i = 0; i != runs; ++i) {
    tmp = cv::getTickCount();
    edge.detect(grad, nms);
    rt += cv::getTickCount() - tmp;
  }

  std::cout << "nms - " << name << ": " << edge.points().size() / ((rt * 1000.0 / cv::getTickFrequency()) / runs)
            << " pixels per ms" << std::endl;
}


template <class EDGE>
void showEdge(EDGE& edge, const cv::Mat& src, const std::string& name, bool circles = true) {
  Mat edgeImg;
  cvtColor(src, edgeImg, CV_GRAY2BGR);
  cv::RNG rng(time(0));

  for_each(edge.segments().begin(), edge.segments().end(), [&](const EdgeSegment& seg) {
    Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
    for (size_t i = seg.begin(); i != seg.end(); ++i) setPixel(edgeImg, edge.points()[i], color);

    if (circles) {
      if (seg.closed())
        setCircle(edgeImg, edge.points()[seg.begin()], Vec3b(255, 0, 0), 2);
      else if (seg.reverse())
        setCircle(edgeImg, edge.points()[seg.end() - 1], Vec3b(0, 0, 255), 2);
      else
        setCircle(edgeImg, edge.points()[seg.begin()], Vec3b(255, 255, 255), 2);
    }
  });

  imshow("edge " + name, edgeImg);
}


int main(int argc, char** argv) {
  // const char* filename = argc >= 2 ? argv[1] : "../../images/circle.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/circle2.png";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/hall2_low.JPG";
  const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.JPG";
  // const char* filename = argc >= 2 ? argv[1] : "../../images/lopez.png";

  cv::Mat src = cv::imread(filename, IMREAD_GRAYSCALE);
  if (src.empty()) {
    cout << "Can not open " << filename << endl;
    return -1;
  }

  if (src.channels() > 1) cvtColor(src, src, cv::COLOR_BGR2GRAY);

  // GaussianNoiseOperator noise(10);
  //  noise.apply(src);
  GaussianBlur(src, src, cv::Size(5, 5), 0.6);

  imshow("img", src);

  float th_low = 0.004, th_high = 0.012;
  DerivativeGradient<uchar, short, int, float, SobelDerivative, QuadraticMagnitude> sobel;
  NonMaximaSuppression<short, int, float, FastNMS8<short, int, float>> nms(th_low, th_high);


  EsdSimple<int> simple;
  EsdDrawing<int> draw(10, 3, sobel.magnitudeThreshold(th_low));
  EsdLinking<int> link(10, 3, 3, sobel.magnitudeThreshold(th_low));
  EsdLinking<int, 8, true> linkc(10, 3, 3, sobel.magnitudeThreshold(th_low));
  EsdPattern<int> pattern(10, 3, 3, sobel.magnitudeThreshold(th_low));
  EsdPattern<int, 8, true> patternc(10, 3, 3, sobel.magnitudeThreshold(th_low));


  sobel.process(src);
  nms.process(sobel);

  testEdge(sobel, nms, simple, "simple");
  showEdge(simple, src, "simple");

  testEdge(sobel, nms, draw, "draw");
  showEdge(draw, src, "draw");

  testEdge(sobel, nms, link, "link");
  showEdge(link, src, "link");

  testEdge(sobel, nms, linkc, "linkc");
  showEdge(linkc, src, "linkc");

  testEdge(sobel, nms, pattern, "pattern");
  showEdge(pattern, src, "pattern");

  testEdge(sobel, nms, patternc, "patternc");
  showEdge(patternc, src, "patternc");

  waitKey();

  return 0;
}
