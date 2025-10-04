/*M///////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// C by Benjamin Wassermann
//M*/

#pragma once

#include <geometry/line.hpp>
#include <geometry/polygon.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <utility/range.hpp>

namespace lsfm {

template <class PT>
inline void text(cv::Mat& img,
                 const PT& pos,
                 const std::string& txt,
                 const cv::Scalar& color = cv::Scalar(0, 0, 255),
                 int font = cv::FONT_HERSHEY_SIMPLEX,
                 double scale = 0.5,
                 int thickness = 1,
                 int lineType = 8,
                 bool bottomLeftOrigin = false) {
  cv::putText(
      img, txt.c_str(),
      cv::Point(static_cast<int>(std::round(getScalar(getX(pos)))), static_cast<int>(std::round(getScalar(getY(pos))))),
      font, scale, color, thickness, lineType, bottomLeftOrigin);
}

template <class PT>
inline void line(cv::Mat& img,
                 const PT& v1,
                 const PT& v2,
                 const cv::Scalar& color = cv::Scalar(0, 0, 255),
                 int thickness = 1,
                 int lineType = 8) {
  cv::line(
      img,
      cv::Point(static_cast<int>(std::round(getScalar(getX(v1)))), static_cast<int>(std::round(getScalar(getY(v1))))),
      cv::Point(static_cast<int>(std::round(getScalar(getX(v2)))), static_cast<int>(std::round(getScalar(getY(v2))))),
      color, thickness, lineType);
}

template <class FT>
inline void line(cv::Mat& img,
                 const Vec4<FT>& l,
                 const cv::Scalar& color = cv::Scalar(0, 0, 255),
                 int thickness = 1,
                 int lineType = 8) {
  cv::line(img, cv::Point(static_cast<int>(std::round(l[0])), static_cast<int>(std::round(l[1]))),
           cv::Point(static_cast<int>(std::round(l[2])), static_cast<int>(std::round(l[3]))), color, thickness,
           lineType);
}

inline void line(cv::Mat& img,
                 const Vec<int, 4>& l,
                 const cv::Scalar& color = cv::Scalar(0, 0, 255),
                 int thickness = 1,
                 int lineType = 8) {
  cv::line(img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, thickness, lineType);
}

template <class FT, template <class> class PT>
inline void line(cv::Mat& img,
                 const Line<FT, PT>& l,
                 const cv::Scalar& color = cv::Scalar(0, 0, 255),
                 int thickness = 1,
                 int lineType = 8,
                 double normalLength = 0,
                 double tipLength = 0) {
  l.draw(img, color, thickness, lineType, normalLength, tipLength);
}

template <class FT, template <class> class PT>
inline void line(cv::Mat& img,
                 const LineSegment<FT, PT>& l,
                 const cv::Scalar& color = cv::Scalar(0, 0, 255),
                 int thickness = 1,
                 int lineType = 8,
                 double normalLength = 0,
                 double tipLength = 0) {
  l.draw(img, color, thickness, lineType, normalLength, tipLength);
}

template <class FT, template <class> class PT>
inline void line(cv::Mat& img,
                 const LineSegment<FT, PT>& l,
                 const std::string& txt,
                 const cv::Scalar& color = cv::Scalar(0, 0, 255),
                 int thickness = 1,
                 int lineType = 8,
                 double normalLength = 0,
                 double tipLength = 0) {
  l.draw(img, color, thickness, lineType, normalLength, tipLength);
  text(img, l.centerPoint(), txt, color);
}

template <class LV>
inline void lines(cv::Mat& img,
                  const LV& lv,
                  const cv::Scalar& color = cv::Scalar(0, 0, 255),
                  int thickness = 1,
                  int lineType = 8,
                  double normalLength = 0,
                  double tipLength = 0) {
  for (size_t i = 0; i != lv.size(); ++i) lv[i].draw(img, color, thickness, lineType, normalLength, tipLength);
}

template <class LV>
inline void lines(cv::Mat& img,
                  const LV& lv,
                  bool ids,
                  const cv::Scalar& color = cv::Scalar(0, 0, 255),
                  int thickness = 1,
                  int lineType = 8,
                  double normalLength = 0,
                  double tipLength = 0) {
  char buffer[50];
  for (size_t i = 0; i != lv.size(); ++i) {
    lv[i].draw(img, color, thickness, lineType, normalLength, tipLength);
    if (ids) {
      sprintf(buffer, "%i", static_cast<int>(i));
      text(img, lv[i].centerPoint(), buffer, color);
    }
  }
}

template <class LV>
inline void lines(cv::Mat& img,
                  const LV& lv,
                  const std::vector<std::string>& tv,
                  const cv::Scalar& color = cv::Scalar(0, 0, 255),
                  int thickness = 1,
                  int lineType = 8,
                  double normalLength = 0,
                  double tipLength = 0) {
  for (size_t i = 0; i != lv.size(); ++i) {
    lv[i].draw(img, color, thickness, lineType, normalLength, tipLength);
    text(img, lv[i].centerPoint(), tv[i], color);
  }
}

template <class LV, class TV>
inline void linesT(cv::Mat& img,
                   const LV& lv,
                   const TV& tv,
                   const cv::Scalar& color = cv::Scalar(0, 0, 255),
                   int thickness = 1,
                   int lineType = 8,
                   double normalLength = 0,
                   double tipLength = 0) {
  for (size_t i = 0; i != lv.size(); ++i) {
    lv[i].draw(img, color, thickness, lineType, normalLength, tipLength);
    text(img, lv[i].centerPoint(), tv[i], color);
  }
}

inline cv::Scalar randomColor(double lowest = 20, double higest = 225) {
  double rng = higest - lowest;
  return cv::Scalar(lowest + cv::theRNG().uniform(0.0, rng), lowest + cv::theRNG().uniform(0.0, rng),
                    lowest + cv::theRNG().uniform(0.0, rng));
}

template <class LV>
inline void linesR(
    cv::Mat& img, const LV& lv, int thickness = 1, int lineType = 8, double normalLength = 0, double tipLength = 0) {
  for (size_t i = 0; i != lv.size(); ++i) {
    lv[i].draw(img, randomColor(), thickness, lineType, normalLength, tipLength);
  }
}

template <class LV>
inline void linesR(cv::Mat& img,
                   const LV& lv,
                   bool ids,
                   int thickness = 1,
                   int lineType = 8,
                   double normalLength = 0,
                   double tipLength = 0) {
  char buffer[50];
  for (size_t i = 0; i != lv.size(); ++i) {
    const cv::Scalar& color = randomColor();
    lv[i].draw(img, color, thickness, lineType, normalLength, tipLength);
    if (ids) {
      sprintf(buffer, "%i", static_cast<int>(i));
      text(img, lv[i].centerPoint(), buffer, color);
    }
  }
}


template <class LV>
inline void linesR(cv::Mat& img,
                   const LV& lv,
                   const std::vector<std::string>& tv,
                   int thickness = 1,
                   int lineType = 8,
                   double normalLength = 0,
                   double tipLength = 0) {
  for (size_t i = 0; i != lv.size(); ++i) {
    const cv::Scalar& color = randomColor();
    lv[i].draw(img, color, thickness, lineType, normalLength, tipLength);
    text(img, lv[i].centerPoint(), tv[i], color);
  }
}

template <class LV, class TV>
inline void linesRT(cv::Mat& img,
                    const LV& lv,
                    const TV& tv,
                    int thickness = 1,
                    int lineType = 8,
                    double normalLength = 0,
                    double tipLength = 0) {
  for (size_t i = 0; i != lv.size(); ++i) {
    const cv::Scalar& color = randomColor();
    lv[i].draw(img, color, thickness, lineType, normalLength, tipLength);
    text(img, lv[i].centerPoint(), tv[i], color);
  }
}


template <class FT, template <class> class PT>
inline cv::Mat drawLine(const cv::Mat& img,
                        const LineSegment<FT, PT>& l,
                        const cv::Scalar& color = cv::Scalar(0, 0, 255),
                        int thickness = 1,
                        int lineType = 8,
                        double normalLength = 0,
                        double tipLength = 0) {
  cv::Mat outImg(img.size(), CV_MAKETYPE(img.depth(), 3));

  if (img.type() == CV_8U)
    cvtColor(img, outImg, CV_GRAY2BGR);
  else
    img.copyTo(outImg);

  line(outImg, l, color, thickness, lineType, normalLength, tipLength);
  return outImg;
}

template <class FT>
inline cv::Mat drawLines(const cv::Mat& img,
                         const std::vector<Vec4<FT>>& lines,
                         const cv::Scalar& color = cv::Scalar(0, 0, 255),
                         int thickness = 1,
                         int lineType = 8,
                         double normalLength = 0,
                         double tipLength = 0) {
  cv::Mat outImg(img.size(), CV_MAKETYPE(img.depth(), 3));

  if (img.type() == CV_8U)
    cvtColor(img, outImg, CV_GRAY2BGR);
  else
    img.copyTo(outImg);


  // Draw segments
  for_each(lines.begin(), lines.end(),
           [&](const Vec4<FT>& v) { line(outImg, v, color, thickness, lineType, normalLength, tipLength); });
  return outImg;
}

template <class FT, class LV>
inline cv::Mat drawLines(const cv::Mat& img,
                         const LV& ls,
                         bool ids = true,
                         const cv::Scalar& color = cv::Scalar(0, 0, 255),
                         int thickness = 1,
                         int lineType = 8,
                         FT normalLength = 0,
                         FT tipLength = 0) {
  cv::Mat outImg(img.size(), CV_MAKETYPE(img.depth(), 3));

  if (img.type() == CV_8U)
    cvtColor(img, outImg, CV_GRAY2BGR);
  else
    img.copyTo(outImg);

  lines(outImg, ls, ids, color, thickness, lineType, normalLength, tipLength);
  return outImg;
}


template <class FT, class LV>
inline cv::Mat drawLines(const cv::Mat& img,
                         const LV& ls,
                         const std::vector<int>& tv,
                         const cv::Scalar& color = cv::Scalar(0, 0, 255),
                         int thickness = 1,
                         int lineType = 8,
                         FT normalLength = 0,
                         FT tipLength = 0) {
  std::vector<std::string> lineIds0Str;
  std::transform(tv.begin(), tv.end(), std::back_inserter(lineIds0Str),
                 [](const int& id) { return std::to_string(id); });
  return drawLines(img, ls, lineIds0Str, color, thickness, lineType, normalLength, tipLength);
}

template <class FT, class LV>
inline cv::Mat drawLines(const cv::Mat& img,
                         const LV& ls,
                         const std::vector<std::string>& tv,
                         const cv::Scalar& color = cv::Scalar(0, 0, 255),
                         int thickness = 1,
                         int lineType = 8,
                         FT normalLength = 0,
                         FT tipLength = 0) {
  cv::Mat outImg(img.size(), CV_MAKETYPE(img.depth(), 3));

  if (img.type() == CV_8U)
    cvtColor(img, outImg, CV_GRAY2BGR);
  else
    img.copyTo(outImg);

  lines(outImg, ls, tv, color, thickness, lineType, normalLength, tipLength);
  return outImg;
}

template <class FT, class LV, class TV>
inline cv::Mat drawLinesT(const cv::Mat& img,
                          const LV& ls,
                          const TV& tv,
                          const cv::Scalar& color = cv::Scalar(0, 0, 255),
                          int thickness = 1,
                          int lineType = 8,
                          FT normalLength = 0,
                          FT tipLength = 0) {
  cv::Mat outImg(img.size(), CV_MAKETYPE(img.depth(), 3));

  if (img.type() == CV_8U)
    cvtColor(img, outImg, CV_GRAY2BGR);
  else
    img.copyTo(outImg);

  linesT(outImg, ls, tv, color, thickness, lineType, normalLength, tipLength);
  return outImg;
}

template <class FT, class LV>
inline cv::Mat drawLinesR(const cv::Mat& img,
                          const LV& ls,
                          bool ids = true,
                          int thickness = 1,
                          int lineType = 8,
                          FT normalLength = 0,
                          FT tipLength = 0) {
  cv::Mat outImg(img.size(), CV_MAKETYPE(img.depth(), 3));

  if (img.type() == CV_8U)
    cvtColor(img, outImg, CV_GRAY2BGR);
  else
    img.copyTo(outImg);

  linesR(outImg, ls, ids, thickness, lineType, normalLength, tipLength);
  return outImg;
}

template <class FT, class LV>
inline cv::Mat drawLinesR(const cv::Mat& img,
                          const LV& ls,
                          const std::vector<std::string>& tv,
                          int thickness = 1,
                          int lineType = 8,
                          FT normalLength = 0,
                          FT tipLength = 0) {
  cv::Mat outImg(img.size(), CV_MAKETYPE(img.depth(), 3));

  if (img.type() == CV_8U)
    cvtColor(img, outImg, CV_GRAY2BGR);
  else
    img.copyTo(outImg);

  linesR(outImg, ls, tv, thickness, lineType, normalLength, tipLength);
  return outImg;
}

template <class FT, class LV, class TV>
inline cv::Mat drawLinesRT(const cv::Mat& img,
                           const LV& ls,
                           const TV& tv,
                           int thickness = 1,
                           int lineType = 8,
                           FT normalLength = 0,
                           FT tipLength = 0) {
  cv::Mat outImg(img.size(), CV_MAKETYPE(img.depth(), 3));

  if (img.type() == CV_8U)
    cvtColor(img, outImg, CV_GRAY2BGR);
  else
    img.copyTo(outImg);

  linesRT(outImg, ls, tv, thickness, lineType, normalLength, tipLength);
  return outImg;
}

template <class FT, template <class> class PT>
void drawMatch(cv::Mat& img,
               cv::Mat& limg,
               cv::Mat& rimg,
               const LineSegment<FT, PT>& ll,
               const LineSegment<FT, PT>& lr,
               const cv::Scalar& color = cv::Scalar(0, 0, 255),
               const std::string& id = std::string(),
               bool connect = false) {
  if (connect) {
    line(limg, ll, color);
    line(rimg, lr, color);
    PT<FT> tmp = lr.centerPoint();
    getX(tmp) += FT(limg.cols);
    LineSegment<FT, PT> c(ll.centerPoint(), tmp);
    line(img, c, id, color);
  } else {
    line(limg, ll, id, color);
    line(rimg, lr, id, color);
  }
}

/*
    void drawMatch(cv::Mat& img, cv::Mat& limg, cv::Mat& rimg, const cv::KeyPoint pl, const cv::KeyPoint pr,
                   const cv::Scalar& color = cv::Scalar(0, 0, 255), const std::string &id = std::string(), bool connect
= false) {

//        inline void line(cv::Mat& img, const Vec<int,4>& l, const cv::Scalar& color = cv::Scalar(0, 0, 255),
//            int thickness = 1, int lineType = 8)

        Vec<int,4> l(pl.pt.x, pl.pt.y, pr.pt.x + img.cols / 2, pr.pt.y);
        line(img, l, color);

/*        if (connect) {
            line(limg, ll, color);
            line(rimg, lr, color);
            PT<FT> tmp = lr.centerPoint();
            getX(tmp) += FT(limg.cols);
            LineSegment<FT,PT> c(ll.centerPoint(), tmp);
            line(img, c, id, color);
        }
        else {
            line(limg, ll, id, color);
            line(rimg, lr, id, color);
        }
*/
/*    }
 */

template <class FT, class DM, class GV>
cv::Mat drawMatches(const cv::Mat& img1,
                    const GV& l1,
                    const cv::Mat& img2,
                    const GV& l2,
                    const std::vector<DM>& matches,
                    const std::vector<int> lineNumber = std::vector<int>(),
                    bool connect = false) {
  cv::Size size(img1.cols + img2.cols, MAX(img1.rows, img2.rows));
  cv::Mat outImg, outImgL, outImgR;

  outImg.create(size, CV_MAKETYPE(img1.depth(), 3));
  outImg = cv::Scalar::all(0);
  outImgL = outImg(cv::Rect(0, 0, img1.cols, img1.rows));
  outImgR = outImg(cv::Rect(img1.cols, 0, img2.cols, img2.rows));

  if (img1.type() == CV_8U)
    cvtColor(img1, outImgL, CV_GRAY2BGR);
  else
    img1.copyTo(outImgL);

  if (img2.type() == CV_8U)
    cv::cvtColor(img2, outImgR, CV_GRAY2BGR);
  else
    img2.copyTo(outImgR);


  cv::RNG& rng = cv::theRNG();

  bool useNumbers = (lineNumber.size() > 0);
  int idx = 0;
  for_each(matches.begin(), matches.end(), [&](const DM& m) {
    cv::Vec3b color(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225));

    char buffer[50];
    sprintf(buffer, "%i", useNumbers ? lineNumber.at(m.queryIdx) : idx);
    drawMatch(outImg, outImgL, outImgR, l1[m.queryIdx], l2[m.matchIdx], cv::Scalar(color[0], color[1], color[2]),
              buffer, connect);
    idx++;
  });

  return outImg;
}

template <class FT, class DM, class LV>
cv::Mat drawMatches(const cv::Mat& img1,
                    const LV& l1,
                    const cv::Mat& img2,
                    const LV& l2,
                    const std::vector<std::vector<DM>>& matches,
                    bool connect = false) {
  std::vector<DM> mv;
  decltype(mv.size()) sz = 0;
  for_each(matches.begin(), matches.end(), [&sz](const std::vector<DM>& v) { sz += v.size(); });
  mv.reserve(matches.size() ? matches.size() * matches[0].size() : 0);
  for_each(matches.begin(), matches.end(),
           [&mv](const std::vector<DM>& v) { mv.insert(mv.end(), v.begin(), v.end()); });

  return drawMatches(img1, l1, img2, l2, mv, connect);
}


template <class FT, template <class> class PT>
void drawEdge(cv::Mat& img, const LineSegment<FT, PT>& edge, size_t idx) {
  if (edge.empty()) return;
  char buffer[50];
  sprintf(buffer, "%i", static_cast<int>(idx));
  line(img, edge, buffer, cv::Scalar(0, 0, 255), 1, 8, 10.0, 4.0);
}

template <class FT, template <class> class PT>
void drawEdge(cv::Mat& img, const Line<FT, PT>& edge, size_t idx) {
  if (edge.empty()) return;
  char buffer[50];
  sprintf(buffer, "%i", static_cast<int>(idx));
  line(img, trim2Box<FT, PT>(edge, FT(img.cols), FT(img.rows)), buffer, cv::Scalar(0, 0, 255), 1, 8, 10.0, 4.0);
}

template <class FT>
cv::Mat drawGeometry(const std::vector<Vec2<FT>>& points,
                     const std::vector<std::pair<size_t, size_t>>& edges,
                     const cv::Mat& img) {
  cv::Mat ret;
  if (img.type() == CV_8U)
    cvtColor(img, ret, CV_GRAY2BGR);
  else
    img.copyTo(ret);

  for (size_t i = 0; i != edges.size(); ++i) {
    drawEdge(ret, lsfm::LineSegment<FT>(points[edges[i].first], points[edges[i].second]), i);
  }

  char buffer[50];
  for (size_t i = 0; i != points.size(); ++i) {
    sprintf(buffer, "%i", static_cast<int>(i));
    cv::putText(ret, buffer,
                cv::Point(static_cast<int>(std::round(points[i].x())), static_cast<int>(std::round(points[i].y()))),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 50, 50));
  }
  return ret;
}

template <class LV>
cv::Mat drawGeometry(const LV& lines, const cv::Mat& img) {
  cv::Mat ret;
  if (img.type() == CV_8U)
    cv::cvtColor(img, ret, CV_GRAY2BGR);
  else
    img.copyTo(ret);

  for (size_t i = 0; i != lines.size(); ++i) {
    drawEdge(ret, lines[i], i);
  }
  return ret;
}


// A poor man's matlab quiver display, via upsampling and anti-aliased line drawing
template <class GT>
cv::Mat quiver(const cv::Mat& image,
               const cv::Mat_<cv::Vec<GT, 2>>& orientation,
               int yTic = 10,
               int xTic = 10,
               double scale = 1.0,
               double length = 0,
               double tol = 1e-6,
               const cv::Scalar& color = cv::Scalar(0, 0, 255),
               int thickness = 1,
               int lineType = 8,
               double normalLength = 0,
               double tipLength = 3.0) {
  cv::Mat canvas = image.clone();
  if (image.channels() == 1) cv::cvtColor(image, canvas, cv::COLOR_GRAY2BGR);

  cv::resize(canvas, canvas, cv::Size(static_cast<int>(canvas.cols * scale), static_cast<int>(canvas.rows * scale)));

  for (int y = 0; y < orientation.rows; y += yTic) {
    for (int x = 0; x < orientation.cols; x += xTic) {
      cv::Point_<float> p(static_cast<float>(x), static_cast<float>(y));
      cv::Vec<GT, 2> v = orientation(y, x);
      double d = cv::norm(v);
      if (d > tol) {
        if (length != 0) v *= length / d;
        p *= scale;
        v *= scale;
        line(canvas, LineSegment<float>(p, p + cv::Point_<float>(v[0], v[1])), color, 1, lineType, normalLength,
             tipLength);
      }
    }
  }

  return canvas;
}

template <class GT>
cv::Mat quiver(const cv::Mat& image,
               const cv::Mat& dx,
               const cv::Mat& dy,
               int yTic = 10,
               int xTic = 10,
               double scale = 1.0,
               double length = 0,
               double tol = 1e-6,
               const cv::Scalar& color = cv::Scalar(0, 0, 255),
               int thickness = 1,
               int lineType = 8,
               double normalLength = 0,
               double tipLength = 3.0) {
  cv::Mat_<cv::Vec<GT, 2>> v;
  cv::Mat tmp[2] = {dx, dy};
  cv::merge(tmp, 2, v);
  return quiver(image, v, yTic, xTic, scale, length, tol, color, thickness, lineType, normalLength, tipLength);
}

template <class DT, class MT>
cv::Mat quiverDir(const cv::Mat& image,
                  const cv::Mat& orientation,
                  const Range<DT>& r,
                  const cv::Mat& mag,
                  int yTic = 10,
                  int xTic = 10,
                  double scale = 1.0,
                  double length = 3,
                  double tol = 1,
                  const cv::Scalar& color = cv::Scalar(0, 0, 255),
                  int thickness = 1,
                  int lineType = 8,
                  double normalLength = 0,
                  double tipLength = 3.0) {
  cv::Mat canvas = image.clone();
  if (image.channels() == 1) cv::cvtColor(image, canvas, cv::COLOR_GRAY2BGR);

  cv::resize(canvas, canvas, cv::Size(canvas.cols * scale, canvas.rows * scale));

  for (int y = 0; y < orientation.rows; y += yTic) {
    for (int x = 0; x < orientation.cols; x += xTic) {
      cv::Point_<float> p(x, y);

      if (mag.at<MT>(y, x) > tol) {
        DT dir = orientation.at<DT>(y, x) / r.size() * 2 * CV_PI;
        cv::Point_<float> v(static_cast<float>(std::cos(dir)), static_cast<float>(std::sin(dir)));
        v *= length;
        p *= scale;
        v *= scale;
        line(canvas, LineSegment<float>(p, p + v), color, 1, lineType, normalLength, tipLength);
      }
    }
  }

  return canvas;
}

inline void setPixel(cv::Mat& dst, size_t idx, const cv::Vec3b& color) { dst.ptr<cv::Vec3b>()[idx] = color; }

inline void setPixel(cv::Mat& dst, const cv::Point& idx, const cv::Vec3b& color) { dst.at<cv::Vec3b>(idx) = color; }

inline void setPixel(cv::Mat& dst, const lsfm::Vec2i& idx, const cv::Vec3b& color) {
  dst.at<cv::Vec3b>(idx.y(), idx.x()) = color;
}

inline void setCircle(cv::Mat& dst, size_t idx, const cv::Vec3b& color, int r) {
  circle(dst, cv::Point(static_cast<int>(idx % dst.cols), static_cast<int>(idx / dst.cols)), r,
         cv::Scalar(color[0], color[1], color[2]));
}

inline void setCircle(cv::Mat& dst, const cv::Point& idx, const cv::Vec3b& color, int r) {
  circle(dst, idx, r, cv::Scalar(color[0], color[1], color[2]));
}

inline void setCircle(cv::Mat& dst, const lsfm::Vec2i& idx, const cv::Vec3b& color, int r) {
  cv::circle(dst, cv::Point(idx.x(), idx.y()), r, cv::Scalar(color[0], color[1], color[2]));
}

template <class FT, template <class> class PT>
inline void fill(const Polygon<FT, PT>& p, cv::Mat& img, const cv::Scalar& color, int lineType = 8) {
  p.fill(img, color, lineType);
}

template <class FT, template <class> class PT>
inline void fillComplex(const Polygon<FT, PT>& p, cv::Mat& img, const cv::Scalar& color, int lineType = 8) {
  p.fillComplex(img, color, lineType);
}

template <class FT, template <class> class PT>
inline void draw(const Polygon<FT, PT>& p, cv::Mat& img, const cv::Scalar& color, int thickness = 1, int lineType = 8) {
  p.draw(img, color, thickness, lineType);
}

template <class FT, template <class> class PT>
inline void fill(const PolygonVector<FT, PT>& pv, cv::Mat& img, const cv::Scalar& color, int lineType = 8) {
  for_each(pv.begin(), pv.end(), [&](const Polygon<FT, PT>& p) { p.fill(img, color, lineType); });
}

template <class FT, template <class> class PT>
inline void fillComplex(const PolygonVector<FT, PT>& pv, cv::Mat& img, const cv::Scalar& color, int lineType = 8) {
  for_each(pv.begin(), pv.end(), [&](const Polygon<FT, PT>& p) { p.fillComplex(img, color, lineType); });
}

template <class FT, template <class> class PT>
inline void draw(
    const PolygonVector<FT, PT>& pv, cv::Mat& img, const cv::Scalar& color, int thickness = 1, int lineType = 8) {
  for_each(pv.begin(), pv.end(), [&](const Polygon<FT, PT>& p) { p.draw(img, color, thickness, lineType); });
}
}  // namespace lsfm
