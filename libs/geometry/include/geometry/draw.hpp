//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann & Manuel Lange
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file draw.hpp
/// @brief Drawing utilities for geometric primitives.
///
/// This file provides comprehensive drawing functions for visualizing
/// geometric objects on OpenCV images. Includes:
/// - Basic primitives: text, lines, points
/// - Line collections: with IDs, random colors, labels
/// - Match visualization: stereo line matches
/// - Quiver plots: vector field visualization (like MATLAB)
/// - Polygon drawing and filling
/// - Geometry visualization with indices
/// Drawing functions use OpenCV's drawing API and support various
/// line types and thicknesses for publication-quality output.
/// @see line.hpp for line representations.
/// @see polygon.hpp for polygon representation.

#pragma once

#include <geometry/line.hpp>
#include <geometry/polygon.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <utility/range.hpp>

namespace lsfm {

/// @name Basic Drawing Primitives
/// @{

/// @brief Draw text at a point on an image.
/// @tparam PT Point type.
/// @param img Image to draw on.
/// @param pos Position for text baseline.
/// @param txt Text string to display.
/// @param color Text color (default red).
/// @param font OpenCV font face.
/// @param scale Font scale factor.
/// @param thickness Text thickness.
/// @param lineType OpenCV line type.
/// @param bottomLeftOrigin If true, origin is at bottom-left.
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

/// @brief Draw line segment between two points.
/// @tparam PT Point type.
/// @param img Image to draw on.
/// @param v1 Start point.
/// @param v2 End point.
/// @param color Line color (default red).
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
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

/// @brief Draw line from Vec4 (x1, y1, x2, y2).
/// @tparam FT Floating-point type.
/// @param img Image to draw on.
/// @param l Line as (x1, y1, x2, y2) vector.
/// @param color Line color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
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

/// @brief Draw line from integer Vec4.
/// @param img Image to draw on.
/// @param l Line as (x1, y1, x2, y2) integer vector.
/// @param color Line color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
inline void line(cv::Mat& img,
                 const Vec<int, 4>& l,
                 const cv::Scalar& color = cv::Scalar(0, 0, 255),
                 int thickness = 1,
                 int lineType = 8) {
  cv::line(img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, thickness, lineType);
}

/// @brief Draw Line object (infinite line trimmed to image).
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param img Image to draw on.
/// @param l Line object.
/// @param color Line color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Length of normal vector visualization.
/// @param tipLength Arrow tip length for normal.
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

/// @brief Draw LineSegment object.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param img Image to draw on.
/// @param l Line segment.
/// @param color Line color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Length of normal vector visualization.
/// @param tipLength Arrow tip length for normal.
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

/// @brief Draw LineSegment with text label at center.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param img Image to draw on.
/// @param l Line segment.
/// @param txt Label text.
/// @param color Line and text color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Length of normal visualization.
/// @param tipLength Arrow tip length.
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

/// @}

/// @name Line Collection Drawing
/// @{

/// @brief Draw multiple lines with uniform color.
/// @tparam LV Line vector type.
/// @param img Image to draw on.
/// @param lv Vector of lines.
/// @param color Line color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal vector length.
/// @param tipLength Arrow tip length.
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

/// @brief Draw lines with optional index labels.
/// @tparam LV Line vector type.
/// @param img Image to draw on.
/// @param lv Vector of lines.
/// @param ids If true, show index numbers at line centers.
/// @param color Line and text color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal vector length.
/// @param tipLength Arrow tip length.
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

/// @brief Draw lines with custom string labels.
/// @tparam LV Line vector type.
/// @param img Image to draw on.
/// @param lv Vector of lines.
/// @param tv Vector of label strings.
/// @param color Line and text color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal vector length.
/// @param tipLength Arrow tip length.
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

/// @brief Draw lines with typed labels.
/// @tparam LV Line vector type.
/// @tparam TV Label vector type.
/// @param img Image to draw on.
/// @param lv Vector of lines.
/// @param tv Vector of labels (streamable type).
/// @param color Line and text color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal vector length.
/// @param tipLength Arrow tip length.
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

/// @brief Generate random color for visualization.
/// @param lowest Minimum channel value.
/// @param higest Maximum channel value.
/// @return Random BGR color.
inline cv::Scalar randomColor(double lowest = 20, double higest = 225) {
  double rng = higest - lowest;
  return cv::Scalar(lowest + cv::theRNG().uniform(0.0, rng), lowest + cv::theRNG().uniform(0.0, rng),
                    lowest + cv::theRNG().uniform(0.0, rng));
}

/// @brief Draw lines with random colors.
/// @tparam LV Line vector type.
/// @param img Image to draw on.
/// @param lv Vector of lines.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal vector length.
/// @param tipLength Arrow tip length.
template <class LV>
inline void linesR(
    cv::Mat& img, const LV& lv, int thickness = 1, int lineType = 8, double normalLength = 0, double tipLength = 0) {
  for (size_t i = 0; i != lv.size(); ++i) {
    lv[i].draw(img, randomColor(), thickness, lineType, normalLength, tipLength);
  }
}

/// @brief Draw lines with random colors and optional IDs.
/// @tparam LV Line vector type.
/// @param img Image to draw on.
/// @param lv Vector of lines.
/// @param ids If true, show index numbers.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal vector length.
/// @param tipLength Arrow tip length.
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

/// @brief Draw lines with random colors and string labels.
/// @tparam LV Line vector type.
/// @param img Image to draw on.
/// @param lv Vector of lines.
/// @param tv Vector of label strings.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal vector length.
/// @param tipLength Arrow tip length.
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

/// @brief Draw lines with random colors and typed labels.
/// @tparam LV Line vector type.
/// @tparam TV Label vector type.
/// @param img Image to draw on.
/// @param lv Vector of lines.
/// @param tv Vector of labels.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal vector length.
/// @param tipLength Arrow tip length.
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

/// @}

/// @name Image-Creating Draw Functions
/// These functions create a new image with the drawings, returning
/// the result without modifying the input image.
/// @{

/// @brief Create new image with single line segment drawn.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param img Source image.
/// @param l Line segment to draw.
/// @param color Line color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return New image with line drawn.
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
    cvtColor(img, outImg, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(outImg);

  line(outImg, l, color, thickness, lineType, normalLength, tipLength);
  return outImg;
}

/// @brief Create new image with Vec4 lines drawn.
/// @tparam FT Floating-point type.
/// @param img Source image.
/// @param lines Vector of (x1,y1,x2,y2) lines.
/// @param color Line color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return New image with lines drawn.
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
    cvtColor(img, outImg, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(outImg);


  // Draw segments
  for_each(lines.begin(), lines.end(),
           [&](const Vec4<FT>& v) { line(outImg, v, color, thickness, lineType, normalLength, tipLength); });
  return outImg;
}

/// @brief Create new image with lines and optional IDs.
/// @tparam FT Floating-point type.
/// @tparam LV Line vector type.
/// @param img Source image.
/// @param ls Vector of lines.
/// @param ids If true, show index numbers.
/// @param color Line and text color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return New image with lines drawn.
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
    cvtColor(img, outImg, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(outImg);

  lines(outImg, ls, ids, color, thickness, lineType, normalLength, tipLength);
  return outImg;
}

/// @brief Create new image with lines and integer labels.
/// @tparam FT Floating-point type.
/// @tparam LV Line vector type.
/// @param img Source image.
/// @param ls Vector of lines.
/// @param tv Vector of integer labels.
/// @param color Line and text color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return New image with lines drawn.
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

/// @brief Create new image with lines and string labels.
/// @tparam FT Floating-point type.
/// @tparam LV Line vector type.
/// @param img Source image.
/// @param ls Vector of lines.
/// @param tv Vector of string labels.
/// @param color Line and text color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return New image with lines drawn.
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
    cvtColor(img, outImg, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(outImg);

  lines(outImg, ls, tv, color, thickness, lineType, normalLength, tipLength);
  return outImg;
}

/// @brief Create new image with lines and typed labels.
/// @tparam FT Floating-point type.
/// @tparam LV Line vector type.
/// @tparam TV Label vector type.
/// @param img Source image.
/// @param ls Vector of lines.
/// @param tv Vector of labels.
/// @param color Line and text color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return New image with lines drawn.
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
    cvtColor(img, outImg, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(outImg);

  linesT(outImg, ls, tv, color, thickness, lineType, normalLength, tipLength);
  return outImg;
}

/// @brief Create new image with lines drawn in random colors.
/// @tparam FT Floating-point type.
/// @tparam LV Line vector type.
/// @param img Source image.
/// @param ls Vector of lines.
/// @param ids If true, show index numbers.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return New image with lines drawn.
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
    cvtColor(img, outImg, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(outImg);

  linesR(outImg, ls, ids, thickness, lineType, normalLength, tipLength);
  return outImg;
}

/// @brief Create new image with lines and labels in random colors.
/// @tparam FT Floating-point type.
/// @tparam LV Line vector type.
/// @param img Source image.
/// @param ls Vector of lines.
/// @param tv Vector of string labels.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return New image with lines drawn.
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
    cvtColor(img, outImg, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(outImg);

  linesR(outImg, ls, tv, thickness, lineType, normalLength, tipLength);
  return outImg;
}

/// @brief Create new image with lines and typed labels in random colors.
/// @tparam FT Floating-point type.
/// @tparam LV Line vector type.
/// @tparam TV Label vector type.
/// @param img Source image.
/// @param ls Vector of lines.
/// @param tv Vector of labels.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return New image with lines drawn.
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
    cvtColor(img, outImg, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(outImg);

  linesRT(outImg, ls, tv, thickness, lineType, normalLength, tipLength);
  return outImg;
}

/// @}

/// @name Stereo Match Drawing
/// Functions for visualizing stereo line matches.
/// @{

/// @brief Draw a stereo line match connecting left and right images.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param img Combined output image.
/// @param limg Left image.
/// @param rimg Right image.
/// @param ll Left line segment.
/// @param lr Right line segment.
/// @param color Line color.
/// @param id Optional label string.
/// @param connect If true, draw connecting line between centers.
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

        // TODO: Implement connection logic
        // if (connect) {
        //     line(limg, ll, color);
        //     line(rimg, lr, color);
        //     PT<FT> tmp = lr.centerPoint();
        //     getX(tmp) += FT(limg.cols);
        //     LineSegment<FT,PT> c(ll.centerPoint(), tmp);
        //     line(img, c, id, color);
        }
        else {
            line(limg, ll, id, color);
            line(rimg, lr, id, color);
        }
*/
/*    }
 */

/// @brief Draw stereo line matches between two images.
/// @tparam FT Floating-point type.
/// @tparam DM Match descriptor type.
/// @tparam GV Geometry vector type.
/// @param img1 Left image.
/// @param l1 Left line segments.
/// @param img2 Right image.
/// @param l2 Right line segments.
/// @param matches Vector of descriptor matches.
/// @param lineNumber Optional line number labels.
/// @param connect If true, draw connecting lines.
/// @return Combined image with matches visualized.
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
    cvtColor(img1, outImgL, cv::COLOR_GRAY2BGR);
  else
    img1.copyTo(outImgL);

  if (img2.type() == CV_8U)
    cv::cvtColor(img2, outImgR, cv::COLOR_GRAY2BGR);
  else
    img2.copyTo(outImgR);


  cv::RNG& rng = cv::theRNG();

  bool useNumbers = (lineNumber.size() > 0);
  int idx = 0;
  for_each(matches.begin(), matches.end(), [&](const DM& m) {
    cv::Vec3b color(static_cast<uchar>(20 + rng.uniform(0, 225)), static_cast<uchar>(20 + rng.uniform(0, 225)),
                    static_cast<uchar>(20 + rng.uniform(0, 225)));

    char buffer[50];
    sprintf(buffer, "%i", useNumbers ? lineNumber.at(static_cast<size_t>(m.queryIdx)) : idx);
    drawMatch(outImg, outImgL, outImgR, l1[static_cast<size_t>(m.queryIdx)], l2[static_cast<size_t>(m.matchIdx)],
              cv::Scalar(color[0], color[1], color[2]), buffer, connect);
    idx++;
  });

  return outImg;
}

/// @brief Draw stereo line matches (multi-match version).
/// @tparam FT Floating-point type.
/// @tparam DM Match descriptor type.
/// @tparam LV Line vector type.
/// @param img1 Left image.
/// @param l1 Left line segments.
/// @param img2 Right image.
/// @param l2 Right line segments.
/// @param matches Vector of match vectors.
/// @param connect If true, draw connecting lines.
/// @return Combined image with matches visualized.
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

/// @}

/// @name Edge and Geometry Drawing
/// Functions for drawing edges and geometric primitives.
/// @{

/// @brief Draw an edge (line segment) with index label.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param img Image to draw on.
/// @param edge Line segment to draw.
/// @param idx Index to display as label.
template <class FT, template <class> class PT>
void drawEdge(cv::Mat& img, const LineSegment<FT, PT>& edge, size_t idx) {
  if (edge.empty()) return;
  char buffer[50];
  sprintf(buffer, "%i", static_cast<int>(idx));
  line(img, edge, buffer, cv::Scalar(0, 0, 255), 1, 8, 10.0, 4.0);
}

/// @brief Draw an edge (infinite line) with index label.
///
/// The line is trimmed to fit within the image bounds.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param img Image to draw on.
/// @param edge Infinite line to draw.
/// @param idx Index to display as label.
template <class FT, template <class> class PT>
void drawEdge(cv::Mat& img, const Line<FT, PT>& edge, size_t idx) {
  if (edge.empty()) return;
  char buffer[50];
  sprintf(buffer, "%i", static_cast<int>(idx));
  line(img, trim2Box<FT, PT>(edge, FT(img.cols), FT(img.rows)), buffer, cv::Scalar(0, 0, 255), 1, 8, 10.0, 4.0);
}

/// @brief Draw geometry with labeled points and edges.
/// @tparam FT Floating-point type.
/// @param points Vector of 2D points.
/// @param edges Vector of (start, end) index pairs.
/// @param img Source image.
/// @return New image with geometry drawn.
template <class FT>
cv::Mat drawGeometry(const std::vector<Vec2<FT>>& points,
                     const std::vector<std::pair<size_t, size_t>>& edges,
                     const cv::Mat& img) {
  cv::Mat ret;
  if (img.type() == CV_8U)
    cvtColor(img, ret, cv::COLOR_GRAY2BGR);
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

/// @brief Draw labeled line segments.
/// @tparam LV Line vector type.
/// @param lines Vector of line segments.
/// @param img Source image.
/// @return New image with lines drawn.
template <class LV>
cv::Mat drawGeometry(const LV& lines, const cv::Mat& img) {
  cv::Mat ret;
  if (img.type() == CV_8U)
    cv::cvtColor(img, ret, cv::COLOR_GRAY2BGR);
  else
    img.copyTo(ret);

  for (size_t i = 0; i != lines.size(); ++i) {
    drawEdge(ret, lines[i], i);
  }
  return ret;
}

/// @}

/// @name Vector Field Visualization
/// Functions for visualizing vector fields using quiver plots.
/// @{

/// @brief Draw quiver plot from 2D orientation field.
///
/// Visualizes a vector field by drawing arrows at regular intervals.
/// @tparam GT Gradient type.
/// @param image Background image.
/// @param orientation 2-channel orientation matrix.
/// @param yTic Vertical spacing between arrows.
/// @param xTic Horizontal spacing between arrows.
/// @param scale Display scale factor.
/// @param length Fixed arrow length (0 for magnitude-based).
/// @param tol Minimum magnitude threshold.
/// @param color Arrow color.
/// @param thickness Arrow thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return Image with quiver plot drawn.
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
        line(canvas, LineSegment<float>(p, p + cv::Point_<float>(v[0], v[1])), color, thickness, lineType, normalLength,
             tipLength);
      }
    }
  }

  return canvas;
}

/// @brief Draw quiver plot from separate dx/dy gradient matrices.
/// @tparam GT Gradient type.
/// @param image Background image.
/// @param dx X-component gradient.
/// @param dy Y-component gradient.
/// @param yTic Vertical spacing between arrows.
/// @param xTic Horizontal spacing between arrows.
/// @param scale Display scale factor.
/// @param length Fixed arrow length (0 for magnitude-based).
/// @param tol Minimum magnitude threshold.
/// @param color Arrow color.
/// @param thickness Arrow thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return Image with quiver plot drawn.
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

/// @brief Draw quiver plot from direction and magnitude matrices.
/// @tparam DT Direction type.
/// @tparam MT Magnitude type.
/// @param image Background image.
/// @param orientation Direction matrix (angles in range units).
/// @param r Range for direction values.
/// @param mag Magnitude matrix.
/// @param yTic Vertical spacing between arrows.
/// @param xTic Horizontal spacing between arrows.
/// @param scale Display scale factor.
/// @param length Fixed arrow length.
/// @param tol Minimum magnitude threshold.
/// @param color Arrow color.
/// @param thickness Arrow thickness.
/// @param lineType OpenCV line type.
/// @param normalLength Normal visualization length.
/// @param tipLength Arrow tip length.
/// @return Image with quiver plot drawn.
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
        line(canvas, LineSegment<float>(p, p + v), color, thickness, lineType, normalLength, tipLength);
      }
    }
  }

  return canvas;
}

/// @}

/// @name Pixel and Shape Utilities
/// Functions for setting individual pixels and drawing simple shapes.
/// @{

/// @brief Set pixel color by linear index.
/// @param dst Target image.
/// @param idx Linear pixel index.
/// @param color Pixel color.
inline void setPixel(cv::Mat& dst, size_t idx, const cv::Vec3b& color) { dst.ptr<cv::Vec3b>()[idx] = color; }

/// @brief Set pixel color by Point coordinates.
/// @param dst Target image.
/// @param idx Pixel coordinates.
/// @param color Pixel color.
inline void setPixel(cv::Mat& dst, const cv::Point& idx, const cv::Vec3b& color) { dst.at<cv::Vec3b>(idx) = color; }

/// @brief Set pixel color by Vec2i coordinates.
/// @param dst Target image.
/// @param idx Pixel coordinates.
/// @param color Pixel color.
inline void setPixel(cv::Mat& dst, const lsfm::Vec2i& idx, const cv::Vec3b& color) {
  dst.at<cv::Vec3b>(idx.y(), idx.x()) = color;
}

/// @brief Draw circle at linear index position.
/// @param dst Target image.
/// @param idx Linear pixel index.
/// @param color Circle color.
/// @param r Circle radius.
inline void setCircle(cv::Mat& dst, size_t idx, const cv::Vec3b& color, int r) {
  // Safe conversion: dst.cols is always positive for valid images
  const auto cols = static_cast<size_t>(dst.cols);
  circle(dst, cv::Point(static_cast<int>(idx % cols), static_cast<int>(idx / cols)), r,
         cv::Scalar(color[0], color[1], color[2]));
}

/// @brief Draw circle at Point position.
/// @param dst Target image.
/// @param idx Circle center.
/// @param color Circle color.
/// @param r Circle radius.
inline void setCircle(cv::Mat& dst, const cv::Point& idx, const cv::Vec3b& color, int r) {
  circle(dst, idx, r, cv::Scalar(color[0], color[1], color[2]));
}

/// @brief Draw circle at Vec2i position.
/// @param dst Target image.
/// @param idx Circle center.
/// @param color Circle color.
/// @param r Circle radius.
inline void setCircle(cv::Mat& dst, const lsfm::Vec2i& idx, const cv::Vec3b& color, int r) {
  cv::circle(dst, cv::Point(idx.x(), idx.y()), r, cv::Scalar(color[0], color[1], color[2]));
}

/// @}

/// @name Polygon Drawing
/// Functions for filling and drawing polygons.
/// @{

/// @brief Fill polygon interior.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param p Polygon to fill.
/// @param img Target image.
/// @param color Fill color.
/// @param lineType OpenCV line type.
template <class FT, template <class> class PT>
inline void fill(const Polygon<FT, PT>& p, cv::Mat& img, const cv::Scalar& color, int lineType = 8) {
  p.fill(img, color, lineType);
}

/// @brief Fill complex (self-intersecting) polygon interior.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param p Polygon to fill.
/// @param img Target image.
/// @param color Fill color.
/// @param lineType OpenCV line type.
template <class FT, template <class> class PT>
inline void fillComplex(const Polygon<FT, PT>& p, cv::Mat& img, const cv::Scalar& color, int lineType = 8) {
  p.fillComplex(img, color, lineType);
}

/// @brief Draw polygon outline.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param p Polygon to draw.
/// @param img Target image.
/// @param color Outline color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
template <class FT, template <class> class PT>
inline void draw(const Polygon<FT, PT>& p, cv::Mat& img, const cv::Scalar& color, int thickness = 1, int lineType = 8) {
  p.draw(img, color, thickness, lineType);
}

/// @brief Fill multiple polygons.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param pv Vector of polygons.
/// @param img Target image.
/// @param color Fill color.
/// @param lineType OpenCV line type.
template <class FT, template <class> class PT>
inline void fill(const PolygonVector<FT, PT>& pv, cv::Mat& img, const cv::Scalar& color, int lineType = 8) {
  for_each(pv.begin(), pv.end(), [&](const Polygon<FT, PT>& p) { p.fill(img, color, lineType); });
}

/// @brief Fill multiple complex polygons.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param pv Vector of polygons.
/// @param img Target image.
/// @param color Fill color.
/// @param lineType OpenCV line type.
template <class FT, template <class> class PT>
inline void fillComplex(const PolygonVector<FT, PT>& pv, cv::Mat& img, const cv::Scalar& color, int lineType = 8) {
  for_each(pv.begin(), pv.end(), [&](const Polygon<FT, PT>& p) { p.fillComplex(img, color, lineType); });
}

/// @brief Draw multiple polygon outlines.
/// @tparam FT Floating-point type.
/// @tparam PT Point type template.
/// @param pv Vector of polygons.
/// @param img Target image.
/// @param color Outline color.
/// @param thickness Line thickness.
/// @param lineType OpenCV line type.
template <class FT, template <class> class PT>
inline void draw(
    const PolygonVector<FT, PT>& pv, cv::Mat& img, const cv::Scalar& color, int thickness = 1, int lineType = 8) {
  for_each(pv.begin(), pv.end(), [&](const Polygon<FT, PT>& p) { p.draw(img, color, thickness, lineType); });
}

/// @}

}  // namespace lsfm
