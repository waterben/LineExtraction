#include <geometry/camera.hpp>
#include <geometry/cameracv.hpp>
#include <geometry/draw.hpp>
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
  sprintf(buffer, "%i", idx);
  ep = edge.centerPoint();
  cv::putText(img, buffer, cv::Point(static_cast<int>(round(ep.x())), static_cast<int>(round(ep.y()))),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 255));
}

cv::Mat drawGeometry(const vector<Vec2<FT>>& points, const vector<pair<size_t, size_t>>& edges, const cv::Mat& img) {
  cv::Mat ret;
  if (img.type() == CV_8U)
    cvtColor(img, ret, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(ret);

  for (size_t i = 0; i != edges.size(); ++i) {
    drawEdge(ret, LineSegment<FT>(points[edges[i].first], points[edges[i].second]), i);
  }

  char buffer[50];
  for (size_t i = 0; i != points.size(); ++i) {
    sprintf(buffer, "%i", i);
    cv::putText(ret, buffer, cv::Point(static_cast<int>(round(points[i].x())), static_cast<int>(round(points[i].y()))),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 50, 50));
  }
  return ret;
}


cv::Mat drawGeometry(const vector<Line2<FT>>& lines, const cv::Mat& img) {
  cv::Mat ret;
  if (img.type() == CV_8U)
    cv::cvtColor(img, ret, cv::COLOR_GRAY2BGR);
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
    cv::cvtColor(img, ret, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(ret);

  for (size_t i = 0; i != edges.size(); ++i) {
    drawEdge(ret, edges[i], i);
  }
  return ret;
}

int main(int argc, char** argv) {
  // camera parameters
  Vec3<FT> origin(0, 0, 10);
  Vec3<FT> orientation(CV_PI, 0, 0);
  FT fov = 50.0 / 180.0 * CV_PI;
  Vec2<FT> size(500, 500);

  // data model cube
  vector<Vec3<FT>> vertices;
  vertices.push_back(Vec3<FT>(-1, 1, 1));
  vertices.push_back(Vec3<FT>(1, 1, 1));
  vertices.push_back(Vec3<FT>(1, -1, 1));
  vertices.push_back(Vec3<FT>(-1, -1, 1));
  vertices.push_back(Vec3<FT>(-1, 1, -1));
  vertices.push_back(Vec3<FT>(1, 1, -1));
  vertices.push_back(Vec3<FT>(1, -1, -1));
  vertices.push_back(Vec3<FT>(-1, -1, -1));

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
  CameraHom<FT> camH(fov, size, origin, orientation);
  CameraPluecker<FT> camP(fov, size, origin, orientation);
  CameraCV<FT> camCV(fov, size, origin, orientation);
  Camera2P<FT> cam2P(fov, size, origin, orientation);

  // ouptut camera inputs
  cout << "Camera: " << std::endl;
  cout << "camera matrix: " << std::endl << camH.camM() << std::endl;
  cout << "rotation matrix: " << std::endl << camH.rotM() << std::endl;
  cout << "translation: " << std::endl << camH.origin() << std::endl;
  cout << "projection matrix: " << std::endl << camH.projM() << std::endl;

  // test if projection matrix is decomposed correctly
  Camera<FT> tmp(camH.projM());
  cout << "Camera reinitialzed by projection matrix: " << std::endl;
  cout << "camera matrix: " << std::endl << camH.camM() << std::endl;
  cout << "rotation matrix: " << std::endl << camH.rotM() << std::endl;
  cout << "translation: " << std::endl << camH.origin() << std::endl;
  cout << "projection matrix: " << std::endl << camH.projM() << std::endl;

  cv::Mat_<cv::Vec2d> vtmp;
  std::cout << vtmp.type() << std::endl;
  cv::Mat_<Vec2d> vtmp2;
  std::cout << vtmp2.type() << std::endl;
  // cv::DataType<int>
  // cv::Vec
  //  project points using projection matrix
  vector<Vec2<FT>> res, resCV;
  camH.project(vertices, res);
  cout << "default (hom): " << camH.project(vertices[0]) << endl << cv::Mat_<Vec2<FT>>(8, 1, &res[0]) << endl << endl;
  camP.project(vertices, res);
  cout << "default (pluecker): " << camP.project(vertices[0]) << endl
       << cv::Mat_<Vec2<FT>>(8, 1, &res[0]) << endl
       << endl;
  cam2P.project(vertices, res);
  cout << "default (2P): " << cam2P.project(vertices[0]) << endl << cv::Mat_<Vec2<FT>>(8, 1, &res[0]) << endl << endl;
  camCV.project(vertices, resCV);
  cout << "open cv: " << camCV.project(vertices[0]) << endl << cv::Mat_<Vec2<FT>>(8, 1, &resCV[0]) << endl << endl;

  // mat test
  // cv::Mat mat(vertices);
  // cout << "default Mat: " << endl << camH.project(mat) << endl << endl;
  // cout << "default  CV: " << endl << camCV.project(mat) << endl << endl;

  vector<Line<FT>> resLP, resL2P, resLCV;
  vector<LineSegment<FT>> resLSP, resLS2P, resLSCV;

  cv::Mat img(static_cast<int>(ceil(size.y())), static_cast<int>(ceil(size.x())), CV_8UC3);
  img.setTo(cv::Vec3b(0, 0, 0));

  auto updateMode = [&](int mode, Vec3<FT> val) {
    switch (mode) {
      case 'r':
        swap(val.x(), val.y());
        val *= CV_PI / 180;
        camH.rotate(val);
        camP.rotate(val);
        cam2P.rotate(val);
        camCV.rotate(val);
        break;
      case 'o':
        swap(val.x(), val.y());
        val *= CV_PI / 180;
        camH.rotate(val, Vec3<FT>(0, 0, 0));
        camP.rotate(val, Vec3<FT>(0, 0, 0));
        cam2P.rotate(val, Vec3<FT>(0, 0, 0));
        camCV.rotate(val, Vec3<FT>(0, 0, 0));
        std::cout << "rotation: " << camH.orientation() * (180 / CV_PI) << std::endl;
        break;
      case 't':
        val *= 0.1;
        camH.translate(val);
        camP.translate(val);
        cam2P.translate(val);
        camCV.translate(val);
        break;
    }
  };

  int key = 0;
  int mode = 'r';
  while (key != 'q') {
    // cout << key << endl;
    switch (key) {
      case 'r':
        mode = key;
        cout << "switched to rotation mode (rotate camera around world origin)" << endl;
        break;
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
        camH.origin(origin);
        camH.orientation(orientation);
        camP.origin(origin);
        camP.orientation(orientation);
        cam2P.origin(origin);
        cam2P.orientation(orientation);
        camCV.origin(origin);
        camCV.orientation(orientation);

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

    camH.project(vertices, res);
    imshow("projected points Hom", drawGeometry(res, edges, img));

    camCV.project(vertices, resCV);
    imshow("projected points CV", drawGeometry(resCV, edges, img));

    camP.project(lines, resLP);
    imshow("projected lines pluecker", drawGeometry(resLP, img));

    /*cam2P.project(lines,resL2P);
    imshow("projected lines two points", drawGeometry(resL2P,img));

    camCV.project(lines,resLCV);
    imshow("projected lines two points CV", drawGeometry(resLCV,img));*/

    camP.project(lineSegments, resLSP);
    imshow("projected line segments pluecker", drawGeometry(resLSP, img));

    cam2P.project(lineSegments, resLS2P);
    imshow("projected line segments two points", drawGeometry(resLS2P, img));

    camCV.project(lineSegments, resLSCV);
    imshow("projected line segments two points CV", drawGeometry(resLSCV, img));

    /*Camera<FT> tmp(camH.projM());
    cout << "orientation: " << std::endl << camH.orientation() << std::endl;
    cout << "orientation: " << std::endl << tmp.orientation() << std::endl;
    cout << "origin: " << std::endl << camH.origin() << std::endl;
    cout << "origin: " << std::endl << tmp.origin() << std::endl;*/
#ifdef WIN32
    key = cv::waitKey();
#else
    key = cv::waitKey() % 256;
#endif
  }

  /*Camera<FT> cl(49.134 * CV_PI / 180,Vec2<FT>(1024,1024),Vec3<FT>(0,-0.7,10),Vec3<FT>(CV_PI,0,CV_PI/2));
  Camera<FT> cr(49.134 * CV_PI / 180,Vec2<FT>(1024,1024),Vec3<FT>(0,0.7,10),Vec3<FT>(CV_PI,0,CV_PI/2));
  std::cout << cl.projM() << endl;
  std::cout << cr.projM() << endl;*/

  return 0;
}
