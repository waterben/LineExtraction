/// @file visualizeRviz.hpp
/// @brief ROS RViz visualization utilities.
///
/// Provides functions for visualizing 3D line segments and points in RViz
/// using ROS visualization markers.

#pragma once

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>


namespace lsfm {


/// @brief Visualize 3D line segments in RViz.
///
/// Publishes line segments as ROS visualization markers without text labels.
/// @tparam MyLine3Dsegment Line segment type with startPoint()/endPoint() methods.
/// @tparam FT Floating-point type.
/// @param line3Segments Line segments to visualize.
/// @param marker_pub ROS publisher for visualization markers.
template <class MyLine3Dsegment, class FT>
void visualizeLinesRviz(const std::vector<MyLine3Dsegment> line3Segments, const ros::Publisher marker_pub) {
  std::vector<std::string> markerText;
  const std::vector<lsfm::Pose<FT>> robotPoses;
  visualizeLinesRviz(line3Segments, markerText, marker_pub, robotPoses);
}

/// @brief Visualize 3D line segments in RViz with labels and poses.
///
/// Publishes line segments as red LINE_LIST markers with optional text labels
/// and robot poses as green ARROW markers.
/// @tparam MyLine3Dsegment Line segment type with startPoint()/endPoint() methods.
/// @tparam FT Floating-point type.
/// @param line3Segments Line segments to visualize.
/// @param markerText Text labels (one per line, or empty for no labels).
/// @param marker_pub ROS publisher for visualization markers.
/// @param robotPoses Robot poses to visualize as arrows.
template <class MyLine3Dsegment, class FT>
void visualizeLinesRviz(const std::vector<MyLine3Dsegment> line3Segments,
                        const std::vector<std::string> markerText,
                        const ros::Publisher marker_pub,
                        const std::vector<lsfm::Pose<FT>> robotPoses) {
  bool addText;
  if (markerText.size() == line3Segments.size())
    addText = true;
  else
    addText = false;

  // %Tag(MARKER_INIT)%
  visualization_msgs::MarkerArray marker_list;

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "world";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  // %EndTag(MARKER_INIT)%

  // %Tag(DELETE_ALL_MARKER)%
  visualization_msgs::Marker reset_marker_;
  reset_marker_.header.frame_id = "world";
  reset_marker_.header.stamp = ros::Time();
  reset_marker_.action = 3;  // In ROS-J: visualization_msgs::Marker::DELETEALL;
  reset_marker_.id = 0;
  // %EndTag(MARKER_INIT)%

  // First Marker deletes all markers
  marker_list.markers.push_back(reset_marker_);

  // Start adding markers -------------------------------------
  uint32_t markerId = 1;

  // %Tag(ID)%
  line_list.id = markerId++;
  // %EndTag(ID)%

  // %Tag(TYPE)%
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  // %EndTag(TYPE)%

  // %Tag(SCALE)%
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.1;
  // %EndTag(SCALE)%

  // %Tag(COLOR)%
  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 0.8;
  // %EndTag(COLOR)%

  // Create the vertices for the points and lines
  for (uint32_t i = 0; i < line3Segments.size(); ++i) {
    // Line Segment
    MyLine3Dsegment lineSegment = line3Segments[i];
    lsfm::Vec3<FT> sp = lineSegment.startPoint();
    lsfm::Vec3<FT> ep = lineSegment.endPoint();

    geometry_msgs::Point p;
    p.x = sp.x();
    p.y = sp.z();
    p.z = -sp.y();
    line_list.points.push_back(p);

    // The line list needs two points for each line
    p.x = ep.x();
    p.y = ep.z();
    p.z = -ep.y() - 0.1;
    line_list.points.push_back(p);

    if (addText) {
      markerId++;

      // Text marker
      visualization_msgs::Marker text;
      text.pose.position = p;
      text.header.frame_id = "world";
      text.header.stamp = ros::Time::now();
      text.id = markerId;
      text.ns = "points_and_lines";
      text.action = visualization_msgs::Marker::ADD;
      text.text = markerText[i];
      text.pose.orientation.w = 1.0;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.color.a = 1.0;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.scale.z = 0.4;

      marker_list.markers.push_back(text);
    }
  }

  for (uint32_t i = 0; i < robotPoses.size(); ++i) {
    markerId++;

    visualization_msgs::Marker pose;
    geometry_msgs::Point origin;
    origin.x = robotPoses[i].origin().x();
    origin.y = robotPoses[i].origin().z();
    origin.z = -robotPoses[i].origin().y();

    pose.pose.position = origin;
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();
    pose.id = markerId;
    pose.ns = "points_and_lines";
    pose.action = visualization_msgs::Marker::ADD;
    //        pose.text = markerText[i];
    pose.type = visualization_msgs::Marker::ARROW;
    cv::Matx33<FT> convMat(robotPoses[i].rotM().data());
    cv::Matx33<FT> rotZ(0, -1, 0, 1, 0, 0, 0, 0, 1);
    cv::Vec4<FT> q = quaternion(convMat * rotZ);
    pose.pose.orientation.x = q[0];
    pose.pose.orientation.y = q[1];
    pose.pose.orientation.z = q[2];
    pose.pose.orientation.w = q[3];
    pose.color.a = 1.0;
    pose.color.r = 0.0f;
    pose.color.g = 1.0f;
    pose.color.b = 0.0f;
    pose.scale.x = 1.0f;
    pose.scale.y = 0.2f;
    pose.scale.z = 0.2f;

    marker_list.markers.push_back(pose);
  }

  marker_list.markers.push_back(line_list);

  marker_pub.publish(marker_list);
}


/// @brief Visualize 3D points in RViz with labels and poses.
///
/// Publishes points as red SPHERE_LIST markers with optional text labels
/// and robot poses as green ARROW markers.
/// @tparam MyPoint3D Point type with point member having x(), y(), z() methods.
/// @tparam FT Floating-point type.
/// @param points3D Points to visualize.
/// @param markerText Text labels (one per point, or empty for no labels).
/// @param marker_pub ROS publisher for visualization markers.
/// @param robotPoses Robot poses to visualize as arrows.
template <class MyPoint3D, class FT>
void visualizePointsRviz(const std::vector<MyPoint3D> points3D,
                         const std::vector<std::string> markerText,
                         const ros::Publisher marker_pub,
                         const std::vector<lsfm::Pose<FT>> robotPoses) {
  bool addText;
  if (markerText.size() == points3D.size())
    addText = true;
  else
    addText = false;

  // %Tag(MARKER_INIT)%
  visualization_msgs::MarkerArray marker_list;

  visualization_msgs::Marker sphere_list;
  sphere_list.header.frame_id = "world";
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.ns = "points_and_lines";
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.pose.orientation.w = 1.0;

  // %EndTag(MARKER_INIT)%

  // %Tag(DELETE_ALL_MARKER)%
  visualization_msgs::Marker reset_marker_;
  reset_marker_.header.frame_id = "world";
  reset_marker_.header.stamp = ros::Time();
  reset_marker_.action = 3;  // In ROS-J: visualization_msgs::Marker::DELETEALL;
  reset_marker_.id = 0;
  // %EndTag(MARKER_INIT)%

  // First Marker deletes all markers
  marker_list.markers.push_back(reset_marker_);

  // Start adding markers -------------------------------------
  uint32_t markerId = 1;

  // %Tag(ID)%
  sphere_list.id = markerId++;
  // %EndTag(ID)%

  // %Tag(TYPE)%
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
  // %EndTag(TYPE)%

  // %Tag(SCALE)%
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  sphere_list.scale.x = 0.5;
  sphere_list.scale.y = 0.5;
  // %EndTag(SCALE)%

  // %Tag(COLOR)%
  // Line list is red
  sphere_list.color.r = 1.0;
  sphere_list.color.a = 0.8;
  // %EndTag(COLOR)%

  // Create the vertices for the points and lines
  for (uint32_t i = 0; i < points3D.size(); ++i) {
    // Line Segment
    MyPoint3D point = points3D[i];

    geometry_msgs::Point p;
    p.x = point.point.x();
    p.y = point.point.z();
    p.z = -point.point.y();
    sphere_list.points.push_back(p);


    if (addText) {
      markerId++;

      // Text marker
      visualization_msgs::Marker text;
      text.pose.position = p;
      text.header.frame_id = "world";
      text.header.stamp = ros::Time::now();
      text.id = markerId;
      text.ns = "points_and_lines";
      text.action = visualization_msgs::Marker::ADD;
      text.text = markerText[i];
      text.pose.orientation.w = 1.0;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.color.a = 1.0;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.scale.z = 0.4;

      marker_list.markers.push_back(text);
    }
  }

  for (uint32_t i = 0; i < robotPoses.size(); ++i) {
    markerId++;

    visualization_msgs::Marker pose;
    geometry_msgs::Point origin;
    origin.x = robotPoses[i].origin().x();
    origin.y = robotPoses[i].origin().z();
    origin.z = -robotPoses[i].origin().y();

    pose.pose.position = origin;
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();
    pose.id = markerId;
    pose.ns = "points_and_lines";
    pose.action = visualization_msgs::Marker::ADD;
    //        pose.text = markerText[i];
    pose.type = visualization_msgs::Marker::ARROW;
    cv::Matx33<FT> convMat(robotPoses[i].rotM().data());
    cv::Matx33<FT> rotZ(0, -1, 0, 1, 0, 0, 0, 0, 1);
    cv::Vec4<FT> q = quaternion(convMat * rotZ);
    pose.pose.orientation.x = q[0];
    pose.pose.orientation.y = q[1];
    pose.pose.orientation.z = q[2];
    pose.pose.orientation.w = q[3];
    pose.color.a = 1.0;
    pose.color.r = 0.0f;
    pose.color.g = 1.0f;
    pose.color.b = 0.0f;
    pose.scale.x = 1.0f;
    pose.scale.y = 0.2f;
    pose.scale.z = 0.2f;

    marker_list.markers.push_back(pose);
  }

  marker_list.markers.push_back(sphere_list);

  marker_pub.publish(marker_list);
}


}  // namespace lsfm
