/// @file poly_test.cpp
/// @brief Polygon operations and manipulations.
///
/// Demonstrates polygon primitive operations:
/// - Polygon construction and transformation
/// - Area and perimeter calculations
/// - Point-in-polygon tests
/// - Polygon rendering and filling
///
/// @usage ./poly_test

#include <geometry/polygon.hpp>
#include <imgproc/image_operator.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>


using namespace std;
using namespace lsfm;


int main() {
  cv::Mat out(600, 800, CV_8UC3);
  out.setTo(0);

  Polygon<float> p1, p2(Vec2f(400, 300)), p3;
  p1.push_back(Vec2f(10, 10));
  p1.push_back(Vec2f(100, 10));
  p1.push_back(Vec2f(100, 100));
  p1.push_back(Vec2f(10, 100));

  p2.push_back(Vec2f(-50, -50));
  p2.push_back(Vec2f(50, -50));
  p2.push_back(Vec2f(50, 50));
  p2.push_back(Vec2f(-50, 50));

  p3.push_back(Vec2f(0, 0));
  p3.push_back(Vec2f(100, 0));
  p3.push_back(Vec2f(100, 100));
  p3.push_back(Vec2f(0, 100));
  p3.translate(Vec2f(650, 450));

  p1.fill(out, cv::Scalar(0, 0, 255));
  p2.fillComplex(out, cv::Scalar(0, 255, 0));
  p3.draw(out, cv::Scalar(255, 0, 0), 5);

  cv::imshow("poly", out);

  cv::waitKey();

  cv::Mat out2(32000, 32000, CV_8U);
  out2.setTo(0);

  Polygon<double> poly;
  poly.push_back(Vec2d(5089, 2023));
  poly.push_back(Vec2d(29947, 2023));
  poly.push_back(Vec2d(20971, 16007));
  poly.push_back(Vec2d(29947, 29959));
  poly.push_back(Vec2d(5089, 29959));
  poly.push_back(Vec2d(2017, 16007));

  // poly.scale(32000.0/40000);
  poly.fill(out2, 180);

  cv::resize(out2, out, cv::Size(), 0.01, 0.01, cv::INTER_AREA);
  cv::imshow("poly hires", out);

  cv::waitKey();

  return 0;
}
