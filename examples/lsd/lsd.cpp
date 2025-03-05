#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry/draw.hpp>
#include <lsd/lsd_cc.hpp>
#include <lsd/lsd_cp.hpp>
#define GRADIENT_MAX_CHECK
#define EDGE_THICK_CHECK
#include <lsd/lsd_el.hpp>
#include <lsd/lsd_ep.hpp>
#include <lsd/lsd_burns.hpp>


#include <lsd/lsd_fgioi.hpp>
#include <lsd/lsd_fbw.hpp>
#include <lsd/lsd_edlz.hpp>
#include <lsd/lsd_hcv.hpp>
#ifdef _MSC_VER
#include <lsd/lsd_edta.hpp>
#pragma comment(lib, "../../../lib/EDLinesLib.lib")  // DetectLinesByED function
#if (_MSC_VER >= 1900)
#pragma comment(lib, "legacy_stdio_definitions.lib") // add legacy_stdio_definitions.lib for VS2015 because of define changes!!!
#endif
#endif

//#include <lsd/lsd_hough.hpp>
//#include <lsd/lsd_ransac.hpp>
//#include <lsd/lsd_kht.hpp>
//#include <lsd/lsd_ph.hpp>
//#include <lsd/lsd_rh.hpp>


using namespace lsfm;
using namespace std;

template<class FT>
struct Entry {
    Entry(const cv::Ptr<LsdBase<FT>>& a = cv::Ptr<LsdBase<FT>>(), const std::string& b = std::string(), double c = 0)
        : lsd(a), name(b), time(c) {}
    
    cv::Ptr<LsdBase<FT>> lsd;
    std::string name;
    double time;
};

static void help()
{
  cout << "\nThis program demonstrates lsd.\n"
          "Usage:\n"
          "./test_lsd <image_name>, Default is ../images/office1_low.jpg\n"
       << endl;
}

template<typename FT>
void showData(const cv::Mat &src, const Entry<FT>& e) {
    std::cout << e.name << ": " << e.time << " ms" << std::endl;

    cv::Mat img;
    cvtColor(src, img, CV_GRAY2BGR);

    cv::RNG rng(cv::getTickCount());
    
    for_each(e.lsd->lineSegments().begin(), e.lsd->lineSegments().end(), [&](const LineSegment<FT> &l) {
        cv::Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));
        cv::Scalar scolor(color[0], color[1], color[2]);
        line(img, l, scolor,1,8,10.0,3.0);
        //Vec2<FT> p1 = l.normalLineDist(0, l.centerPoint()), p2 = l.normalLineDist(10, l.centerPoint());
        //line(img, p1, p2, cv::Scalar(0, 0, 255));

    });
    cv::imshow(e.name, img);
    cv::waitKey(1);
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

  std::vector<Entry<FT>> vlsd;

  vlsd.push_back(Entry<FT>(new LsdCC<FT>(0.004, 0.012, 10, 2, 2, CC_FIND_NEAR_COMPLEX), "lsd cc"));
  vlsd.push_back(Entry<FT>(new LsdCP<FT>(0.004, 0.012, 10, 0, 2, 2, CP_FIND_NEAR_COMPLEX), "lsd cp"));
  vlsd.push_back(Entry<FT>(new LsdEL<FT>(0.004, 0.012, 10, 3, 10, 4, 0 /*EL_USE_NFA*/), "lsd el"));
  vlsd.push_back(Entry<FT>(new LsdEP<FT>(0.004, 0.012, 10, 3, 10, 2, 3, 3, 5, 0), "lsd ep"));
  vlsd.push_back(Entry<FT>(new LsdBurns<FT>(0.004, 0.012, 10, 12, BURNS_NMS), "lsd burns"));
  vlsd.push_back(Entry<FT>(new LsdFBW<FT>(0.004, 0.012, 10, 22.5, FBW_NMS), "lsd fbw"));
  vlsd.push_back(Entry<FT>(new LsdFGioi<FT>(2, 22.5, 0, 0.7, 1024), "lsd fgioi"));
  vlsd.push_back(Entry<FT>(new LsdEDLZ<FT>(10, 2, 2, 10, 2, false), "lsd edlz"));
#ifdef _MSC_VER
    vlsd.push_back(Entry<FT>(new LsdEDTA<FT>, "lsd edta"));
#endif
    

    int runs = 10;
    std::for_each(vlsd.begin(), vlsd.end(), [&runs, &src](Entry<FT>& entry) {
        for (int i = 0; i != runs; ++i) {
            double start = double(cv::getTickCount());
            entry.lsd->detect(src);
            entry.time += (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency() / runs;
        }
        showData(src, entry);
    });

    cv::imshow("quiver", quiver<short>(src, vlsd[0].lsd->imageData("gx"), vlsd[0].lsd->imageData("gy"),2,2,2,2,10));
    cv::waitKey();
    
    return 0;
}
