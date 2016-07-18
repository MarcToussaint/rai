#pragma once
#include <Core/module.h>
#include <RosCom/filterObject.h>
#include <RosCom/roscom.h>

#ifdef MLR_ROS_INDIGO
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  namespace ar = ar_track_alvar_msgs;
#endif
#if MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  namespace ar = ar_track_alvar;
#endif

Cluster conv_ROSMarker2Cluster(const visualization_msgs::Marker& marker);
Alvar conv_ROSAlvar2Alvar(const ar::AlvarMarker& marker);
OptitrackMarker conv_tf2OptitrackMarker(const geometry_msgs::TransformStamped& msg);
OptitrackBody conv_tf2OptitrackBody(const geometry_msgs::TransformStamped& msg);

struct Collector : Module{
  ACCESSname(visualization_msgs::MarkerArray, tabletop_clusters)
  ACCESSname(ar::AlvarMarkers, ar_pose_markers)
  ACCESSname(std::vector<geometry_msgs::TransformStamped>, opti_markers)
  ACCESSname(std::vector<geometry_msgs::TransformStamped>, opti_bodies)


  ACCESSname(FilterObjects, perceptual_inputs)

  Collector();

  virtual void open(){}
  virtual void step();
  virtual void close(){}

private:
  bool useRos = mlr::getParameter<bool>("useRos", false);
  int tabletop_clusters_revision = 0;
  int ar_pose_markers_revision = 0;
  int opti_markers_revision = 0;
  int opti_bodies_revision = 0;

  ors::Transformation tf; // Transformation from the camera to the body
  bool has_transform = false;
};
