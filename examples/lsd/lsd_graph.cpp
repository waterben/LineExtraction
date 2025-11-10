#include <geometry/draw.hpp>
#include <lsd/lsd_el.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>


using namespace lsfm;
using namespace std;
using namespace cv;


static void help() {
  cout << "\nThis program demonstrates lsd.\n"
          "Usage:\n"
          "./test_lsd_nfa <image_name>, Default is ../images/office1_low.jpg\n"
       << endl;
}

int main(int argc, char** argv) {
  const char* filename = argc >= 2 ? argv[1] : "../../images/office1_low.JPG";

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


  LsdEL<FT> lsd(0.004f, 0.012f, 30, 3, 5, 4, 0 /*EL_USE_NFA*/);

  lsd.detect(src);
  LsdEL<FT>::LineSegmentVector lines = lsd.lineSegments();

  // init as same line (e.g. diagonal of mat)
  struct Dists {
    Dists() : id(0), nd(0) {}
    Dists(FT i, FT n = 0) : id(i), nd(n) {}

    FT id, nd;  // intersection distance and normal distance

    // same line (could be oriented 180� in other direction and/or has different range)
    inline bool equal() const { return id == std::numeric_limits<FT>::infinity() && nd == 0; }

    inline bool parallel() const { return id == std::numeric_limits<FT>::infinity(); }
  };


  std::vector<Dists> data;
  data.resize(lines.size() * lines.size());
  int64 start = cv::getTickCount();
  int runs = 100;
  for (int r = 0; r != runs; ++r) {
    for (size_t i = 0; i != lines.size() - 1; ++i) {
      Dists& ii = data[i * lines.size() + i];
      ii.id = std::numeric_limits<FT>::infinity();
      ;
      ii.nd = 0;
      for (size_t j = i + 1; j != lines.size(); ++j) {
        Dists& ij = data[i * lines.size() + j];
        Dists& ji = data[j * lines.size() + i];
        LsdEL<FT>::line_point p;
        if (lines[i].intersection(lines[j], p)) {
          ij.id = lines[i].project(p);
          ji.id = lines[j].project(p);
        } else {
          ij.id = std::numeric_limits<FT>::infinity();
          ji.id = std::numeric_limits<FT>::infinity();
        }
        ij.id = lines[j].distance(lines[i].centerPoint());
        ji.id = lines[i].distance(lines[j].centerPoint());
      }
    }
    Dists& ii = data.back();
    ii.id = std::numeric_limits<FT>::infinity();
    ;
    ii.nd = 0;
  }
  double time = static_cast<double>(cv::getTickCount() - start) * 1000.0 / cv::getTickFrequency() / 100;
  std::cout << "line: " << lines.size() << ", time: " << time << "ms" << std::endl;

  imshow("lines", drawLinesR(src, lines, false, 1, 8, 5, 3));
  waitKey();

  return 0;
}
