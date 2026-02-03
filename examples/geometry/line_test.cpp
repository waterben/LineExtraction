/// @file line_test.cpp
/// @brief Line primitive operations and transformations.
///
/// Demonstrates 2D line primitive operations:
/// - Line construction from points and angles
/// - Coordinate system transformations (world <-> line)
/// - Distance calculations and projections
/// - Intersection computations
///
/// @usage ./line_test

#include <geometry/line.hpp>
#include <imgproc/rcmg.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>


using namespace std;
using namespace lsfm;


int main(int /*argc*/, char** /*argv*/) {
  /*const char* filename = "../../images/office1_low.jpg";

  //cv::Mat src = imread(filename, IMREAD_GRAYSCALE);
  cv::Mat src = imread(filename, IMREAD_COLOR);
  if (src.empty())
  {
      cout << "Can not open " << filename << endl;
      return -1;
  }


  Line2d l1(Point2d(0, 0), Point2d(1, 1)), l2(CV_PI / 4 + CV_PI / 2, 0);

  std::cout << "l1.angle: " << l1.angle() / CV_PI * 180 << ", l1.nangle: " << l1.normalAngle() / CV_PI * 180 << ",
  l1.dist: " << l1.originDist() << std::endl; std::cout << "l2.angle: " << l2.angle() / CV_PI * 180 << ", l2.nangle: "
  << l2.normalAngle() / CV_PI * 180 << ", l2.dist: " << l2.originDist() << std::endl;

  Point2d p(3, 2);
  Point2d o(2, 2);
  Point2d lp = l1.cart2line(p,o);
  std::cout << "p: " << p << ", lp: " << lp << ", p: " << l1.line2cart(lp,o) << std::endl;
  lp = p;
  p = l1.line2cart(lp,o);
  std::cout << "lp: " << lp << ", p: " << p << ", lp: " << l1.cart2line(p,o) << std::endl;

  struct b { cv::Mat a; };

  std::cout << sizeof(b);

  char tmp;
  std::cin >> tmp;*/

  /* cv::Mat img(500, 500, CV_8UC3);
   img.setTo(0);

   Line<double> l1(0, 1, 200), l2(1, 0, 300);
   //Line<double> l1 = LineSegment<double>(Point2d(0, 200), Point2d(100, 200));
   //Line<double> l2 = LineSegment<double>(Point2d(300, 0), Point2d(300, 100));

   Point2d origin(300, 200);
   Point2d lineP(75, 50);


   do {
       img.setTo(0);
       l1.draw(img,Scalar(0,255,0));
       l2.draw(img, Scalar(255, 50, 50));
       circle(img, origin, 2, Scalar(0, 255, 0));

       Point2d world1 = l1.line2world(lineP, origin);
       circle(img, world1, 2, Scalar(0, 0, 255));
       Point2d world2 = l1.line2world(Point2d(lineP.x,-lineP.y), origin);
       circle(img, world2, 2, Scalar(255, 0, 0));
       Point2d world3 = l1.line2world(-lineP, origin);
       circle(img, world3, 2, Scalar(255, 0, 0));
       Point2d world4 = l1.line2world(Point2d(-lineP.x, lineP.y), origin);
       circle(img, world4, 2, Scalar(255, 0, 0));

       lsfm::line(img, world1, world2, Scalar(255, 0, 0));
       lsfm::line(img, world2, world3, Scalar(255, 0, 0));
       lsfm::line(img, world3, world4, Scalar(255, 0, 0));
       lsfm::line(img, world4, world1, Scalar(255, 0, 0));


       circle(img, l1.world2line(world1, origin), 2, Scalar(255, 255, 0));
       imshow("line", img);
       l1.rotate(CV_PI / 180, origin);
       l2.rotate(CV_PI / 180, origin);
   } while (waitKey(5000) != 'q');*/

  /*Line2d gtline(Point2d(10, 10), Point2d(100, 10));
  LineSegment2d line(Point2d(20,-10), Point2d(20, 10));
  std::cout << "error: " << line.error(gtline) << " errorSED: " << line.errorSED(gtline) << " angle: " << line.anglef()
  << std::endl; line.rotateP(10.0 / 180 * CV_PI, Point2d(20, 0)); std::cout << "error: " << line.error(gtline) << "
  errorSED: " << line.errorSED(gtline) << " angle: " << line.anglef() << std::endl; line.rotateP(10.0 / 180 * CV_PI,
  Point2d(20, 0)); std::cout << "error: " << line.error(gtline) << " errorSED: " << line.errorSED(gtline) << " angle: "
  << line.anglef() << std::endl; line.rotateP(10.0 / 180 * CV_PI, Point2d(20, 0)); std::cout << "error: " <<
  line.error(gtline) << " errorSED: " << line.errorSED(gtline) << " angle: " << line.anglef() << std::endl;
  line.rotateP(10.0 / 180 * CV_PI, Point2d(20, 0));
  std::cout << "error: " << line.error(gtline) << " errorSED: " << line.errorSED(gtline) << " angle: " << line.anglef()
  << std::endl; line.rotateP(10.0 / 180 * CV_PI, Point2d(20, 0)); std::cout << "error: " << line.error(gtline) << "
  errorSED: " << line.errorSED(gtline) << " angle: " << line.anglef() << std::endl; line.rotateP(10.0 / 180 * CV_PI,
  Point2d(20, 0)); std::cout << "error: " << line.error(gtline) << " errorSED: " << line.errorSED(gtline) << " angle: "
  << line.anglef() << std::endl; line.rotateP(10.0 / 180 * CV_PI, Point2d(20, 0)); std::cout << "error: " <<
  line.error(gtline) << " errorSED: " << line.errorSED(gtline) << " angle: " << line.anglef() << std::endl;
  line.rotateP(10.0 / 180 * CV_PI, Point2d(20, 0));
  std::cout << "error: " << line.error(gtline) << " errorSED: " << line.errorSED(gtline) << " angle: " << line.anglef()
  << std::endl; line.rotateP(10.0 / 180 * CV_PI, Point2d(20, 0)); std::cout << "error: " << line.error(gtline) << "
  errorSED: " << line.errorSED(gtline) << " angle: " << line.anglef() << std::endl; char c; std::cin >> c;*/


  LineSegment2d line1(Vec2d(25, 25), Vec2d(52, 3));
  std::cout << line1.startPoint() << ", " << line1.endPoint() << std::endl;
  if (line1.trim2Box(50, 50, 0, 0))
    std::cout << line1.startPoint() << ", " << line1.endPoint() << std::endl;
  else
    std::cout << "false" << std::endl;

  return 0;
}
