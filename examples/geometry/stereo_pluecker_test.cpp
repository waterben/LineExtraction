#include <geometry/draw.hpp>
#include <geometry/stereocv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <map>

using namespace std;
using namespace lsfm;

typedef double FT;

void drawEdge(cv::Mat& img, const LineSegment2<FT>& edge, size_t idx) {
  if (edge.empty()) return;
  char buffer[50];
  line(img, edge, cv::Scalar(0, 0, 255));
  line(img, LineSegment2<FT>(edge.normalLineDist(0, edge.centerPoint()), edge.normalLineDist(10, edge.centerPoint())),
       cv::Scalar(0, 0, 200));
  Vec2<FT> ep = edge.lineDistOrigin(edge.end() - 3);
  cv::circle(img, cv::Point(static_cast<int>(round(ep.x())), static_cast<int>(round(ep.y()))), 1,
             cv::Scalar(100, 100, 255));
  sprintf(buffer, "%lu", idx);
  ep = edge.centerPoint();
  cv::putText(img, buffer, cv::Point(static_cast<int>(round(ep.x())), static_cast<int>(round(ep.y()))),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 255));
}

cv::Mat drawGeometry(const vector<Vec2<FT>>& points, const vector<pair<size_t, size_t>>& edges, const cv::Mat& img) {
  cv::Mat ret;
  if (img.type() == CV_8U)
    cv::cvtColor(img, ret, CV_GRAY2BGR);
  else
    img.copyTo(ret);

  for (size_t i = 0; i != edges.size(); ++i) {
    drawEdge(ret, LineSegment<FT>(points[edges[i].first], points[edges[i].second]), i);
  }

  char buffer[50];
  for (size_t i = 0; i != points.size(); ++i) {
    sprintf(buffer, "%lu", i);
    cv::putText(ret, buffer, cv::Point(static_cast<int>(round(points[i].x())), static_cast<int>(round(points[i].y()))),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 50, 50));
  }
  return ret;
}


cv::Mat drawGeometry(const vector<Line2<FT>>& lines, const cv::Mat& img) {
  cv::Mat ret;
  if (img.type() == CV_8U)
    cv::cvtColor(img, ret, CV_GRAY2BGR);
  else
    img.copyTo(ret);

  for (size_t i = 0; i != lines.size(); ++i) {
    drawEdge(ret, lsfm::trim2Box<FT>(lines[i], img.cols, img.rows), i);
  }
  return ret;
}

cv::Mat drawGeometry(const vector<LineSegment2<FT>>& edges, const cv::Mat& img) {
  cv::Mat ret;
  if (img.type() == CV_8U)
    cv::cvtColor(img, ret, CV_GRAY2BGR);
  else
    img.copyTo(ret);

  for (size_t i = 0; i != edges.size(); ++i) {
    drawEdge(ret, edges[i], i);
  }
  return ret;
}

cv::Mat drawGeometry(const vector<Vec2<FT>>& pointsL,
                     const vector<Vec2<FT>>& pointsR,
                     const vector<pair<size_t, size_t>>& edges,
                     const cv::Mat& img) {
  cv::Mat L = drawGeometry(pointsL, edges, img), R = drawGeometry(pointsR, edges, img);

  cv::Mat ret(L.rows, 2 * L.cols, L.type());
  cv::Mat retL = ret(cv::Rect(0, 0, L.cols, L.rows));
  cv::Mat retR = ret(cv::Rect(L.cols, 0, R.cols, R.rows));
  L.copyTo(retL);
  R.copyTo(retR);
  return ret;
}


cv::Mat drawGeometry(const vector<Line2<FT>>& linesL, const vector<Line2<FT>>& linesR, const cv::Mat& img) {
  cv::Mat L = drawGeometry(linesL, img), R = drawGeometry(linesR, img);

  cv::Mat ret(L.rows, 2 * L.cols, L.type());
  cv::Mat retL = ret(cv::Rect(0, 0, L.cols, L.rows));
  cv::Mat retR = ret(cv::Rect(L.cols, 0, R.cols, R.rows));
  L.copyTo(retL);
  R.copyTo(retR);
  return ret;
}

cv::Mat drawGeometry(const vector<LineSegment2<FT>>& edgesL,
                     const vector<LineSegment2<FT>>& edgesR,
                     const cv::Mat& img) {
  cv::Mat L = drawGeometry(edgesL, img), R = drawGeometry(edgesR, img);

  cv::Mat ret(L.rows, 2 * L.cols, L.type());
  cv::Mat retL = ret(cv::Rect(0, 0, L.cols, L.rows));
  cv::Mat retR = ret(cv::Rect(L.cols, 0, R.cols, R.rows));
  L.copyTo(retL);
  R.copyTo(retR);
  return ret;
}

cv::Mat drawGeometry(const vector<Vec2<FT>>& points1L,
                     const vector<Vec2<FT>>& points1R,
                     const vector<Vec2<FT>>& points2L,
                     const vector<Vec2<FT>>& points2R,
                     const vector<pair<size_t, size_t>>& edges,
                     const cv::Mat& img) {
  cv::Mat U = drawGeometry(points1L, points1R, edges, img), L = drawGeometry(points2L, points2R, edges, img);

  cv::Mat ret(2 * U.rows, U.cols, U.type());
  cv::Mat retU = ret(cv::Rect(0, 0, U.cols, U.rows));
  cv::Mat retL = ret(cv::Rect(0, U.rows, U.cols, U.rows));
  U.copyTo(retU);
  L.copyTo(retL);
  return ret;
}


cv::Mat drawGeometry(const vector<Line2<FT>>& lines1L,
                     const vector<Line2<FT>>& lines1R,
                     const vector<Line2<FT>>& lines2L,
                     const vector<Line2<FT>>& lines2R,
                     const cv::Mat& img) {
  cv::Mat U = drawGeometry(lines1L, lines1R, img), L = drawGeometry(lines2L, lines2R, img);

  cv::Mat ret(2 * U.rows, U.cols, U.type());
  cv::Mat retU = ret(cv::Rect(0, 0, U.cols, U.rows));
  cv::Mat retL = ret(cv::Rect(0, U.rows, U.cols, U.rows));
  U.copyTo(retU);
  L.copyTo(retL);
  return ret;
}

cv::Mat drawGeometry(const vector<LineSegment2<FT>>& edges1L,
                     const vector<LineSegment2<FT>>& edges1R,
                     const vector<LineSegment2<FT>>& edges2L,
                     const vector<LineSegment2<FT>>& edges2R,
                     const cv::Mat& img) {
  cv::Mat U = drawGeometry(edges1L, edges1R, img), L = drawGeometry(edges2L, edges2R, img);

  cv::Mat ret(2 * U.rows, U.cols, U.type());
  cv::Mat retU = ret(cv::Rect(0, 0, U.cols, U.rows));
  cv::Mat retL = ret(cv::Rect(0, U.rows, U.cols, U.rows));
  U.copyTo(retU);
  L.copyTo(retL);
  return ret;
}

int main(int argc, char** argv) {
  // camera parameters
  Vec3<FT> originL(-1, 0, 10);
  Vec3<FT> originR(1, 0, 10);
  Vec3<FT> orientation(CV_PI, 0, 0);
  Vec2<FT> size(4608 / 10.0, 3456 / 10.0);
  Vec2<FT> focal(3250.3 / 10.0, 3334 / 10.0);
  Vec2<FT> offset(2287.2 / 10.0, 2272.8 / 10.0);


  Vec3<FT> trans(2, -1, -2);
  // data model cube
  vector<Vec3<FT>> vertices;
  vertices.push_back(Vec3<FT>(-1, 1, 1) + trans);
  vertices.push_back(Vec3<FT>(1, 1, 1) + trans);
  vertices.push_back(Vec3<FT>(1, -1, 1) + trans);
  vertices.push_back(Vec3<FT>(-1, -1, 1) + trans);
  vertices.push_back(Vec3<FT>(-1, 1, -1) + trans);
  vertices.push_back(Vec3<FT>(1, 1, -1) + trans);
  vertices.push_back(Vec3<FT>(1, -1, -1) + trans);
  vertices.push_back(Vec3<FT>(-1, -1, -1) + trans);

  vector<pair<size_t, size_t>> edges;
  edges.push_back(pair<size_t, size_t>(0, 1));
  edges.push_back(pair<size_t, size_t>(1, 2));
  edges.push_back(pair<size_t, size_t>(2, 3));
  edges.push_back(pair<size_t, size_t>(3, 0));

  edges.push_back(pair<size_t, size_t>(4, 5));
  edges.push_back(pair<size_t, size_t>(5, 6));
  edges.push_back(pair<size_t, size_t>(6, 7));
  edges.push_back(pair<size_t, size_t>(7, 4));

  edges.push_back(pair<size_t, size_t>(0, 4));
  edges.push_back(pair<size_t, size_t>(1, 5));
  edges.push_back(pair<size_t, size_t>(2, 6));
  edges.push_back(pair<size_t, size_t>(3, 7));


  vector<Line3<FT>> lines;
  vector<LineSegment3<FT>> lineSegments;

  // init line and line segment vectors
  for (size_t i = 0; i != edges.size(); ++i) {
    lines.push_back(LineSegment3<FT>(vertices[edges[i].first], vertices[edges[i].second]));
    lineSegments.push_back(LineSegment3<FT>(vertices[edges[i].first], vertices[edges[i].second]));
  }

  // init Cameras
  CameraPluecker<FT> camLP(focal, offset, size, originL, orientation);
  CameraPluecker<FT> camRP(focal, offset, size, originR, orientation);
  // CameraPluecker<FT> camLP(focal,offset,originL,orientation);
  // CameraPluecker<FT> camRP(focal,offset,originR,orientation);
  //    CameraPluecker<FT> camLP(fov,size,originL,orientation);
  //    CameraPluecker<FT> camRP(fov,size,originR,orientation);

  vector<LineSegment<FT>> resRLSP, resLLSP, resbRLSP, resbLLSP;

  vector<LineSegment3<FT>> lineSegmentsP;

  cv::Mat img(static_cast<int>(ceil(size.y())), static_cast<int>(ceil(size.x())), CV_8UC3);
  img.setTo(cv::Vec3<uchar>(0, 0, 0));

  StereoPlane<FT> stereoP(camLP, camRP);

  auto updateMode = [&](int mode, Vec3<FT> val) {
    switch (mode) {
      case 'o':
        swap(val.x(), val.y());
        val *= CV_PI / 180;
        camLP.rotate(val, Vec3<FT>(0, 0, 0));
        camRP.rotate(val, Vec3<FT>(0, 0, 0));


        break;
      case 't':
        val *= 0.1;
        camLP.translate(val);
        camRP.translate(val);
        break;
    }

    stereoP = StereoPlane<FT>(camLP, camRP);
  };

  int key = 0;
  int mode = 'o';
  while (key != 'q') {
    switch (key) {
      case 'o':
        mode = key;
        cout << "switched to orientation mode (rotate camera around camera origin)" << endl;
        break;
      case 't':
        mode = key;
        cout << "switched to translation mode" << endl;
        break;
      case 'c':
        cout << "clear data" << endl;
        camLP.origin(originL);
        camLP.orientation(orientation);

        camRP.origin(originR);
        camRP.orientation(orientation);

        stereoP = StereoPlane<FT>(camLP, camRP);

        break;
      case 81:  // left
      case 2424832:
        updateMode(mode, Vec3<FT>(1, 0, 0));
        break;
      case 82:  // up
      case 2490368:
        updateMode(mode, Vec3<FT>(0, 1, 0));
        break;
      case 83:  // right
      case 2555904:
        updateMode(mode, Vec3<FT>(-1, 0, 0));
        break;
      case 84:  // down
      case 2621440:
        updateMode(mode, Vec3<FT>(0, -1, 0));
        break;
      case 85:  // bup
      case 2162688:
        updateMode(mode, Vec3<FT>(0, 0, 1));
        break;
      case 86:  // bdown
      case 2228224:
        updateMode(mode, Vec3<FT>(0, 0, -1));
        break;
    }

    camLP.project(lineSegments, resLLSP);
    camRP.project(lineSegments, resRLSP);
    stereoP.triangulate(resLLSP, resRLSP, lineSegmentsP);
    camLP.project(lineSegmentsP, resbLLSP);
    camRP.project(lineSegmentsP, resbRLSP);
    //   imshow("Pluecker segments", drawGeometry(resLLSP, resRLSP, resbLLSP, resbRLSP,img));
    imshow("Pluecker segments", drawGeometry(resbLLSP, img));
    FT ssdL = 0, ssdR = 0, ssd3 = 0;
    for (size_t i = 0; i != resLLSP.size(); ++i) {
      ssdL += resbLLSP[i].error(resLLSP[i]);
      ssdR += resbRLSP[i].error(resRLSP[i]);

      ssd3 += lineSegmentsP[i].error(lineSegments[i]);
    }
    if (ssd3 > LIMITS<FT>::tau() || ssdL > LIMITS<FT>::tau() || ssdR > LIMITS<FT>::tau())
      cout << "Pluecker, segments - ssd3: " << ssd3 << ", ssd L: " << ssdL << ", ssd R: " << ssdR << endl;

#ifdef WIN32
    key = cv::waitKey();
#else
    key = cv::waitKey() % 256;
#endif
  }

  return 0;
}
