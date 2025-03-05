#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <geometry/draw.hpp>
#include <lsd/lsd_el.hpp>
#include <lsd/lsd_cc.hpp>
#include <edge/nfa.hpp>



using namespace lsfm;
using namespace std;
using namespace cv;

template<class FT, class MT>
void calcMagnitude(const EdgeSegmentVector &seg, const IndexVector& points, const cv::Mat &Mag, std::vector<FT> n){
    size_t s = seg.size();
    for (size_t i = 0; i < s; ++i) {
        FT sum = 0;
        FT sqSum = 0;

        for(size_t lpos = seg[i].begin(); lpos != seg[i].end(); ++lpos){
            sum += get<MT>(Mag, points[lpos]);
            sqSum += std::sqrt(get<MT>(Mag, points[lpos]));
        }
        n.push_back(sqSum / (seg[i].size()));
        std::cout << "line: " << i << " sqrt(sum)/n: " << std::sqrt(sum) / (seg[i].size()) << "  sum(sqrt(val))/n: " << sqSum / (seg[i].size()) << std::endl;
    }
}

template<class LSD, class NFA>
void testNfa(LSD &lsd, const NFA &nfa, const std::string &name) {
    std::vector<float> n;
    int runs = 20;
    int64 rt = 0, tmp;
    for (int i = 0; i != runs; ++i) {
        tmp = cv::getTickCount();
        nfa.evalLV(lsd.segments(), lsd.indexes(), lsd.lineSegments(), n);
        rt += cv::getTickCount() - tmp;
    }

    std::cout << "nfa - " << name << ": " << (rt * 1000.0 / cv::getTickFrequency()) / runs << std::endl;
}

template<class LSD, class NFA>
void showNfa(LSD &lsd, const cv::Mat &src, const NFA &nfa, const std::string &name, bool circles = true) {

    Mat edgeImg;
    cvtColor(src, edgeImg, CV_GRAY2BGR);
    cv::RNG &rng = cv::theRNG();

    std::vector<float> n;
    nfa.evalLV(lsd.segments(), lsd.indexes(), lsd.lineSegments(), n);


    std::vector<std::pair<size_t, float>> idxVec;
    for (size_t i = 0; i != n.size(); ++i)
        idxVec.push_back(std::pair<size_t, float>(i, n[i]));

    std::sort(idxVec.begin(), idxVec.end(), [](const std::pair<size_t, float> &a, const std::pair<size_t, float> &b) {
        return a.second > b.second;
    });

    char buffer[50];
    for (size_t i = 0; i != n.size(); ++i) {
        Scalar color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
        sprintf(buffer, "%zu",i);
        std::cout << i << ": " << idxVec[i].second << std::endl;
        line(edgeImg, lsd.lineSegments()[idxVec[i].first], color);
        text(edgeImg, lsd.lineSegments()[idxVec[i].first].centerPoint(), buffer, color,0,0.4);
    }

    imshow("nfa all " + name, edgeImg);

    cvtColor(src, edgeImg, CV_GRAY2BGR);
    for (size_t i = 0; i != n.size() / 2; ++i) {
        if (idxVec[i].second < 0)
            break;
        Scalar color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
        sprintf(buffer, "%zu", i);
        line(edgeImg, lsd.lineSegments()[idxVec[i].first], color);
        text(edgeImg, lsd.lineSegments()[idxVec[i].first].centerPoint(), buffer, color, 0, 0.4);
    }

    imshow("nfa filtered " + name, edgeImg);

    cvtColor(src, edgeImg, CV_GRAY2BGR);
    for (size_t i = 0; i != n.size()/2; ++i) {
        Scalar color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
        sprintf(buffer, "%zu", i);
        line(edgeImg, lsd.lineSegments()[idxVec[i].first], color);
        text(edgeImg, lsd.lineSegments()[idxVec[i].first].centerPoint(), buffer, color, 0, 0.4);
    }

    imshow("nfa top 50% " + name, edgeImg);

    cvtColor(src, edgeImg, CV_GRAY2BGR);
    for (size_t i = 0; i != n.size() && i != 20; ++i) {
        Scalar color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
        sprintf(buffer, "%zu", i);
        line(edgeImg, lsd.lineSegments()[idxVec[i].first], color);
        text(edgeImg, lsd.lineSegments()[idxVec[i].first].centerPoint(), buffer, color, 0, 0.4);
    }

    imshow("nfa top 20 " + name, edgeImg);
}

static void help()
{
  cout << "\nThis program demonstrates lsd.\n"
          "Usage:\n"
          "./test_lsd_nfa <image_name>, Default is ../images/office1_low.JPG\n"
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
  GaussianBlur(src, src, cv::Size(3, 3), 0.8);

  typedef float FT;


  //    LsdCC<FT> lsd(0.008, 0.012, 30, 0, 2);
  LsdEL<FT> lsd(0.004, 0.012, 10, 3, 10, 4, 0 /*EL_USE_NFA*/);


  NfaContrast<int, float, index_type, std::map<int, float>> nfac(8);
  NfaBinom<short, float, index_type> nfab(8, 1.0 / 8);
  NfaBinom2<short, float, index_type> nfab2(8, 1.0 / 8);

  lsd.detect(src);
  std::cout << "lines: " << lsd.lineSegments().size() << std::endl;

  double scale = 2;
  cv::Mat q = quiver<short>(src, lsd.imageData("gx"), lsd.imageData("gy"), 4, 4, scale, 2, 10, Scalar(0, 0, 255), 1,
                            cv::LINE_AA);
  for_each(
      lsd.lineSegments().begin(), lsd.lineSegments().end(), [&](LineSegment<FT> l) {
        l.scale(scale);
        Vec2<FT> pt1 = l.centerPoint();
        Vec2<FT> pt2 = lsfm::Vec2<FT>(10 * cos(l.gradientAnglef() * CV_PI/180), 10 * sin(l.gradientAnglef() * CV_PI/180)) + l.centerPoint();
        LineSegment<FT> l2(pt1, pt2);
        line(q, l2, Scalar(255, 0, 0), 1, cv::LINE_AA, 0.0, 4.0);
      });
  imshow("quiver", q);

  nfac.update(lsd.edgeSource());
  nfab.update(lsd.edgeSource());
  nfab2.update(lsd.edgeSource());

  testNfa(lsd, nfac, "nfa constrast");
  testNfa(lsd, nfab, "nfa binomial");
  testNfa(lsd, nfab2, "nfa binomial 2");

  showNfa(lsd, src, nfac, "link constrast");
  showNfa(lsd, src, nfab, "link binomial");
  showNfa(lsd, src, nfab2, "link binomial 2");


  // template<class FT, class PT, class MT>
  std::vector<FT> n;
  calcMagnitude<FT, int>(lsd.segments(), lsd.indexes(), lsd.edgeSource().magnitude(), n);

  cv::waitKey();

  return 0;
}
