#pragma once
#include <Core/thread.h>
#include <Perception/percept.h>
#include <RosCom/roscom.h>

#ifdef MLR_ROS_INDIGO
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  namespace ar = ar_track_alvar_msgs;
#endif
#if MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  namespace ar = ar_track_alvar;
#endif
#include <object_recognition_msgs/TableArray.h>

Cluster conv_ROSMarker2Cluster(const visualization_msgs::Marker& marker);
Plane conv_ROSTable2Plane(const object_recognition_msgs::Table& table);
Alvar conv_ROSAlvar2Alvar(const ar::AlvarMarker& marker);
OptitrackMarker conv_tf2OptitrackMarker(const geometry_msgs::TransformStamped& msg);
OptitrackBody conv_tf2OptitrackBody(const geometry_msgs::TransformStamped& msg);

/** listens to ros topics, such as Alvar and OptiTrack and TableTop, and pipes them into the percepts_input
 */
struct Collector : Thread{
  Access_typed<visualization_msgs::MarkerArray> tabletop_clusters;
  Access_typed<ar::AlvarMarkers> ar_pose_markers;
  Access_typed<std::vector<geometry_msgs::TransformStamped>> opti_markers;
  Access_typed<std::vector<geometry_msgs::TransformStamped>> opti_bodies;
  Access_typed<object_recognition_msgs::TableArray> tabletop_tableArray;

  ACCESSname(mlr::Transformation, tabletop_srcFrame)
  ACCESSname(mlr::Transformation, alvar_srcFrame)
  ACCESSname(mlr::Transformation, optitrack_srcFrame)

  ACCESSname(PerceptL, percepts_input)

  Collector(const bool simulate = false);

  virtual void open(){}
  virtual void step();
  virtual void close(){}

private:
  bool useRos = mlr::getParameter<bool>("useRos", false);
  int tabletop_clusters_revision = 0;
  int ar_pose_markers_revision = 0;
  int tabletop_tableArray_revision = 0;
  int opti_markers_revision = 0;
  int opti_bodies_revision = 0;

  bool simulate;

  bool has_tabletop_transform = false;
  bool has_alvar_transform = false;
};
