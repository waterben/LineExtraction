/// @file render_gl.hpp
/// @brief OpenGL rendering utilities for 3D geometry.
/// Provides functions for rendering 3D objects, poses, and line segments
/// using OpenGL. Includes utilities for converting between OpenGL and
/// OpenCV coordinate systems and reading rendered images.

#pragma once

#include <geometry/cameracv.hpp>
#include <geometry/line3.hpp>

#include <functional>
#include <map>
#include <sstream>
// #include <boost/filesystem/fstream.hpp>

#include <GL/glut.h>
#include <geometry/draw.hpp>
#include <geometry/object3d.hpp>

#include <bitset>
#include <math.h>
// #include <slam/slamDataModel.hpp>

namespace lsfm {

/*
    cv::Mat render(const Camerad &cam, std::function<void()> &func);
    cv::Mat render(const Camerad &cam, const Object3DList<double> &objs);
    cv::Mat render(const Cameraf &cam, std::function<void()> &func);
    cv::Mat render(const Cameraf &cam, const Object3DList<float> &objs);
*/

/// @brief Test rendering function.
void testRenderingTR();

/// @brief Convert OpenGL pose to OpenCV coordinate system.
/// Transforms a pose from OpenGL conventions to OpenCV conventions
/// by rotating 180° around the X-axis.
/// @tparam FT Floating-point type.
/// @param glPose Pose in OpenGL coordinates.
/// @return Pose in OpenCV coordinates.
template <class FT>
lsfm::Pose<FT> gl2cvPose(lsfm::Pose<FT> glPose) {
  lsfm::Pose<FT> tmpPose(glPose);
  tmpPose.origin(lsfm::Vec3<FT>(-tmpPose.origin().x(), -tmpPose.origin().y(), -tmpPose.origin().z()));
  tmpPose.orientation(lsfm::Vec3<FT>(tmpPose.orientation().x(), tmpPose.orientation().y(), tmpPose.orientation().z()));
  lsfm::Matx33<FT> currentRot, rotToCv(1, 0, 0, 0, -1, 0, 0, 0, -1);  // rotate around x-axis by 180°
  currentRot = tmpPose.rotM();
  tmpPose.orientation(lsfm::Vec3<FT>(0, 0, 0));
  currentRot = currentRot * rotToCv;
  tmpPose.rotate(currentRot);
  return tmpPose;
}

/// @brief Render multiple poses as 3D spheres with axes.
/// @tparam FT Floating-point type.
/// @param keyFramePoses Vector of poses to render.
template <class FT>
void renderPoses3d(std::vector<lsfm::Pose<FT>> keyFramePoses) {
  for_each(keyFramePoses.begin(), keyFramePoses.end(), [&](const lsfm::Pose<FT>& obj) { renderPose3d(obj); });
}

/// @brief Render single pose as 3D sphere with coordinate axes.
/// Draws a solid sphere at the pose origin with X, Y, Z axis lines
/// showing the orientation.
/// @tparam FT Floating-point type.
/// @param obj Pose to render.
template <class FT>
void renderPose3d(const lsfm::Pose<FT>& obj) {
  glDisable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glLoadIdentity();  // Reset the model-view matrix

  glTranslatef(-obj.origin().x(), -obj.origin().y(), -obj.origin().z());
  glutSolidSphere(0.3, 20, 50);

  // Draw Y and Z Axis
  glBegin(GL_LINES);

  glVertex3f(0, 0, 0);
  lsfm::Vec3<FT> unitX(2, 0, 0);
  unitX = obj.rotM() * unitX;
  glVertex3f(unitX[0], unitX[1], unitX[2]);

  glVertex3f(0, 0, 0);
  lsfm::Vec3<FT> unitY(0, 2, 0);
  unitY = obj.rotM() * unitY;
  glVertex3f(unitY[0], unitY[1], unitY[2]);

  glVertex3f(0, 0, 0);
  lsfm::Vec3<FT> unitZ(0, 0, -2);
  unitZ = obj.rotM() * unitZ;
  glVertex3f(unitZ[0], unitZ[1], unitZ[2]);

  glEnd();

  glEnable(GL_LIGHTING);
}


/// @brief Render multiple 3D objects as filled meshes.
/// @tparam FT Floating-point type.
/// @param objs List of objects to render.
template <class FT>
void renderObjects3d(const Object3DList<FT> objs) {
  for_each(objs.begin(), objs.end(), [&](const Object3D<FT>& obj) { renderObject3d(obj); });
}

/// @brief Render single 3D object as filled mesh with lighting.
/// Draws triangles with normals and optional texture coordinates.
/// @tparam FT Floating-point type.
/// @param obj Object to render.
template <class FT>
void renderObject3d(const Object3D<FT>& obj) {
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glLoadIdentity();  // Reset the model-view matrix
  glBegin(GL_TRIANGLES);
  for_each(obj.triangles.begin(), obj.triangles.end(), [&](const Triangle& t) {
    glNormal3d(obj.normals[t.normal][0], obj.normals[t.normal][1], obj.normals[t.normal][2]);

    if ((t.ta == std::size_t(0) && t.tb == std::size_t(0) && t.tc == std::size_t(0)) ||
        obj.textureCoords[t.ta][0] == 0.0 && obj.textureCoords[t.ta][1] == 0.0 && obj.textureCoords[t.tb][0] == 0.0 &&
            obj.textureCoords[t.tb][1] == 0.0 && obj.textureCoords[t.tc][0] == 0.0 &&
            obj.textureCoords[t.tc][1] == 0.0) {
      glVertex3f(obj.points[t.a][0], obj.points[t.a][1], obj.points[t.a][2]);
      glVertex3f(obj.points[t.b][0], obj.points[t.b][1], obj.points[t.b][2]);
      glVertex3f(obj.points[t.c][0], obj.points[t.c][1], obj.points[t.c][2]);
    } else {
      glTexCoord2f(obj.textureCoords[t.ta][0], obj.textureCoords[t.ta][1]);
      glVertex3f(obj.points[t.a][0], obj.points[t.a][1], obj.points[t.a][2]);
      glTexCoord2f(obj.textureCoords[t.tb][0], obj.textureCoords[t.tb][1]);
      glVertex3f(obj.points[t.b][0], obj.points[t.b][1], obj.points[t.b][2]);
      glTexCoord2f(obj.textureCoords[t.tc][0], obj.textureCoords[t.tc][1]);
      glVertex3f(obj.points[t.c][0], obj.points[t.c][1], obj.points[t.c][2]);
    }
  });
  glEnd();  // End of drawing
}


/// @brief Render multiple objects as wireframes with unique line IDs.
/// @tparam FT Floating-point type.
/// @param objs List of objects to render.
template <class FT>
void renderWireIdObjects3d(const Object3DList<FT> objs) {
  std::vector<std::pair<lsfm::Vec2<FT>, lsfm::Vec2<FT>>> endPoints;
  for_each(objs.begin(), objs.end(), [&](const Object3D<FT>& obj) { renderWireIdObject3d(obj, endPoints); });
}

/// @brief Render multiple objects as wireframes, outputting edge endpoints.
/// @tparam FT Floating-point type.
/// @param objs List of objects to render.
/// @param endPoints Output vector of edge start/end point pairs.
template <class FT>
void renderWireIdObjects3d(const Object3DList<FT> objs,
                           std::vector<std::pair<lsfm::Vec3<FT>, lsfm::Vec3<FT>>>& endPoints) {
  for_each(objs.begin(), objs.end(), [&](const Object3D<FT>& obj) { renderWireIdObject3d(obj, endPoints); });
}

/// @brief Render single object as wireframe with unique line IDs.
/// Renders edges with unique colors for line identification. Removes
/// near-duplicate edges based on distance and angle thresholds.
/// @tparam FT Floating-point type.
/// @param obj Object to render.
/// @param endPoints Output vector of edge start/end point pairs.
template <class FT>
void renderWireIdObject3d(const Object3D<FT>& obj, std::vector<std::pair<lsfm::Vec3<FT>, lsfm::Vec3<FT>>>& endPoints) {
  glDisable(GL_LIGHTING);
  /*
          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
          glLoadIdentity();                 // Reset the model-view matrix
          glBegin(GL_TRIANGLES);
          glColor3f(0.0f, 0.0f, 0.0f);     // Black
          for_each(obj.triangles.begin(),obj.triangles.end(),[&](const Triangle &t){
              glNormal3d(obj.normals[t.normal][0], obj.normals[t.normal][1], obj.normals[t.normal][2]);
              glVertex3f( obj.points[t.a][0], obj.points[t.a][1], obj.points[t.a][2]);
              glVertex3f( obj.points[t.b][0], obj.points[t.b][1], obj.points[t.b][2]);
              glVertex3f( obj.points[t.c][0], obj.points[t.c][1], obj.points[t.c][2]);
          });
          glEnd();  // End of drawing
  */
  // convert edges to 3d Lines to use 3d Line functions
  std::vector<lsfm::LineSegment3<FT>> lineSegments3, ls3Tmp;
  lineSegments3.reserve(obj.triangles.size() * 3);
  for_each(obj.triangles.begin(), obj.triangles.end(), [&](const Triangle& t) {
    ls3Tmp.push_back(
        lsfm::LineSegment3<FT>(lsfm::Vec3<FT>(obj.points[t.a][0], obj.points[t.a][1], obj.points[t.a][2]),
                               lsfm::Vec3<FT>(obj.points[t.b][0], obj.points[t.b][1], obj.points[t.b][2])));
    ls3Tmp.push_back(
        lsfm::LineSegment3<FT>(lsfm::Vec3<FT>(obj.points[t.b][0], obj.points[t.b][1], obj.points[t.b][2]),
                               lsfm::Vec3<FT>(obj.points[t.c][0], obj.points[t.c][1], obj.points[t.c][2])));
    ls3Tmp.push_back(
        lsfm::LineSegment3<FT>(lsfm::Vec3<FT>(obj.points[t.c][0], obj.points[t.c][1], obj.points[t.c][2]),
                               lsfm::Vec3<FT>(obj.points[t.a][0], obj.points[t.a][1], obj.points[t.a][2])));
  });

  // remove (almost) identical lines
  for (int i = 0; i < ls3Tmp.size(); ++i) {
    lsfm::LineSegment3<FT> currentLineSeg3 = ls3Tmp[i];
    bool add = true;
    for (int j = 0; j < i; ++j) {
      FT dist = ls3Tmp[j].distance(currentLineSeg3.nearestPointOnLineSegment(ls3Tmp[j]));
      // deal with lines that are 180° rotated to each other
      FT angle = fmod(currentLineSeg3.angle(ls3Tmp[j]), M_PI);
      if (angle > M_PI_2) angle = std::abs(angle - M_PI);

      if (dist < 0.05 && angle < 0.01745 * 1) {  // less than one degree
        add = false;
        break;
      }
    }
    if (add) lineSegments3.push_back(currentLineSeg3);
  }

  //        std::cout << "num ls3: " << lineSegments3.size() << std::endl;
  //        std::cout << "num l3: " << obj.edges().size() << std::endl;


  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glColor3i(std::numeric_limits<GLint>::max(), std::numeric_limits<GLint>::max(), std::numeric_limits<GLint>::max());

  glLineWidth(3);  // float lineWidth2[2] = {0,0}; glGetFloatv(GL_LINE_WIDTH, lineWidth2);
  glBegin(GL_LINES);

  for_each(lineSegments3.begin(), lineSegments3.end(), [&](const lsfm::LineSegment3<FT>& seg) {
    //            glNormal3d(obj.normals[seg.normal][0], obj.normals[seg.normal][1], obj.normals[seg.normal][2]);

    int lineId = endPoints.size();
    char lowerByte = lineId;
    char higherByte = lineId >> 8;
    /*  std::cout << "lineId: " << std::bitset<16>(lineId) << std::endl;
        std::cout << "lowerByte: " << std::bitset<8>(lowerByte) << std::endl;
        std::cout << "higherByte: " << std::bitset<8>(higherByte) << std::endl;  */

    glColor3ub(lowerByte, higherByte, 255);
    glVertex3f(seg.startPoint()[0], seg.startPoint()[1], seg.startPoint()[2]);
    glVertex3f(seg.endPoint()[0], seg.endPoint()[1], seg.endPoint()[2]);
    endPoints.push_back(std::pair<lsfm::Vec3<FT>, lsfm::Vec3<FT>>(seg.startPoint(), seg.endPoint()));
  });
  glEnd();  // End of drawing

  glEnable(GL_LIGHTING);
}


/// @brief Render 3D objects projected to 2D using OpenCV.
/// Projects 3D edges to 2D using camera parameters and draws them
/// on an OpenCV image.
/// @tparam FT Floating-point type.
/// @param objs List of objects to render.
/// @param cam Camera for projection.
/// @param img Output image to draw on.
template <class FT>
void renderObjects3dCV(const Object3DList<FT> objs, const Camera<FT>& cam, cv::Mat& img) {
  typename lsfm::Object3D<FT>::EdgeList lineSegments3;
  for_each(objs.begin(), objs.end(), [&](const Object3D<FT>& obj) {
    typename lsfm::Object3D<FT>::EdgeList tmpSegs3 = obj.edgesAll();
    lineSegments3.insert(lineSegments3.end(), tmpSegs3.begin(), tmpSegs3.end());
  });

  lsfm::CameraCV<FT> camProj(cam);
  /*
          for_each(lineSegments3.begin(),lineSegments3.end(),[&](const LineSegment3<FT> &l3){
              camProj.project(l3).draw(img, cv::Scalar(255, 255, 255));
          });
  */

  // lsfm::Line2Vector<FT> lineSegments2;

  std::vector<lsfm::LineSegment<FT>> models2d;
  camProj.project(lineSegments3, models2d);

  img = lsfm::drawLines<FT>(img, models2d);
}


/// @brief Read current OpenGL framebuffer into OpenCV Mat.
/// Captures the current OpenGL rendering to an existing OpenCV image,
/// converting from RGB to BGR and flipping vertically.
/// @tparam MatType OpenCV Mat type (default cv::Mat).
/// @param width Image width in pixels.
/// @param height Image height in pixels.
/// @param img Output image (must be pre-allocated).
template <class MatType = cv::Mat>
void getGlImageAsOpenCvMat(int width, int height, MatType& img) {
  // Initialise a Mat to contain the image
  // cv::Mat temp = cv::Mat::zeros(height, width, CV_8UC3);
  MatType tempImage;

  // Read Image from buffer
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, img.data);

  // Process buffer so it matches correct format and orientation
  cv::cvtColor(img, tempImage, CV_BGR2RGB);
  cv::flip(tempImage, img, 0);
}

/// @brief Read OpenGL buffer to OpenCV Mat with format control.
/// Reads from the OpenGL framebuffer with specified format and data type,
/// creating a new OpenCV Mat. Supports various pixel formats and depths.
/// @tparam MT OpenCV data type (e.g., CV_8UC3, CV_32FC1).
/// @param width Image width in pixels.
/// @param height Image height in pixels.
/// @param format OpenGL pixel format (GL_RGB, GL_DEPTH_COMPONENT, etc.).
/// @return OpenCV Mat containing the framebuffer contents.
template <class MT>
cv::Mat getGlBufferAsOpenCvMat(int width, int height, GLenum format = GL_RGB) {
  // GL_STENCIL_INDEX, GL_DEPTH_COMPONENT, GL_DEPTH_STENCIL, GL_RED, GL_GREEN, GL_BLUE, GL_RGB, GL_BGR, GL_RGBA, and
  // GL_BGRA.

  // Initialise a Mat to contain the image
  cv::Mat temp;

  int depth = cv::DataType<MT>::depth;
  int channels = cv::DataType<MT>::channels;

  GLenum type = GL_UNSIGNED_BYTE;
  switch (depth) {
    case CV_8U:
      type = GL_UNSIGNED_BYTE;
      break;
    case CV_8S:
      type = GL_BYTE;
      break;
    case CV_16U:
      type = GL_UNSIGNED_SHORT;
      break;
    case CV_16S:
      type = GL_SHORT;
      break;
    case CV_32S:
      type = GL_INT;
      break;
    case CV_32F:
      type = GL_FLOAT;
      break;
    default:
      std::cerr << "Unexpected type: " << type << std::endl;
      return cv::Mat();
  }

  switch (format) {
    case GL_RGB:
    case GL_BGR:
      if (channels != 3 && channels != 1) {
        std::cerr << "Unexpected channels: " << channels << std::endl;
        return cv::Mat();
      }
      temp.create(height, width, CV_MAKETYPE(depth, 3));
      break;
    case GL_RGBA:
    case GL_BGRA:
      if (channels != 4 && channels != 1) {
        std::cerr << "Unexpected channels: " << channels << std::endl;
        return cv::Mat();
      }
      temp.create(height, width, CV_MAKETYPE(depth, 4));
      break;
    case GL_STENCIL_INDEX:
    case GL_DEPTH_COMPONENT:
    case GL_DEPTH_STENCIL:
    case GL_RED:
    case GL_GREEN:
    case GL_BLUE:
      if (channels != 1) {
        std::cerr << "Unexpected channels: " << channels << std::endl;
        return cv::Mat();
      }
      temp.create(height, width, CV_MAKETYPE(depth, 1));
      break;
    default:
      std::cerr << "Unexpected format: " << format << std::endl;
      return cv::Mat();
  }

  // Read Image from buffer
  glReadPixels(0, 0, width, height, format, type, temp.data);

  // Process buffer so it matches correct orientation
  cv::flip(temp, temp, 0);
  return temp;
}

/// @brief Render 3D line segments in OpenGL.
/// Draws white line segments using OpenGL immediate mode.
/// @tparam FT Floating-point type.
/// @tparam LineType Container type for line segments.
/// @param lineSegments3 Line segments to render.
template <class FT, class LineType>
void renderLines3d(LineType lineSegments3) {
  // std::vector<lsfm::LineSegment3<FT>> lineSegments3;

  glLineWidth(3);  // float lineWidth2[2] = {0,0}; glGetFloatv(GL_LINE_WIDTH, lineWidth2);
  glBegin(GL_LINES);

  for_each(lineSegments3.begin(), lineSegments3.end(), [&](const lsfm::LineSegment3<FT>& seg) {
    //            glNormal3d(obj.normals[seg.normal][0], obj.normals[seg.normal][1], obj.normals[seg.normal][2]);
    /*
                int lineId = endPoints.size();
                char lowerByte = lineId;
                char higherByte = lineId >> 8;
                std::cout << "lineId: " << std::bitset<16>(lineId) << std::endl;
                std::cout << "lowerByte: " << std::bitset<8>(lowerByte) << std::endl;
                std::cout << "higherByte: " << std::bitset<8>(higherByte) << std::endl;  */

    glColor3ub(255, 255, 255);
    glVertex3f(seg.startPoint()[0], seg.startPoint()[1], seg.startPoint()[2]);
    glVertex3f(seg.endPoint()[0], seg.endPoint()[1], seg.endPoint()[2]);
    //          endPoints.push_back(std::pair<lsfm::Vec3<FT>, lsfm::Vec3<FT>>(seg.startPoint(), seg.endPoint()));
  });
  glEnd();  // End of drawing
}


}  // namespace lsfm
