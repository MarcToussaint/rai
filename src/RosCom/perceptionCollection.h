#pragma once

#include <Algo/filterObject.h>
#include <visualization_msgs/MarkerArray.h>
#include <Core/module.h>

#ifdef MLR_ROS_INDIGO
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  namespace ar = ar_track_alvar_msgs;
#endif
#if MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  namespace ar = ar_track_alvar;
#endif


FilterObject conv_Marker2FilterObject(const visualization_msgs::Marker& marker);
FilterObject conv_Alvar2FilterObject(const ar::AlvarMarker& marker);

struct Collector : Module{
  ACCESSname(visualization_msgs::MarkerArray, tabletop_clusters)
  ACCESSname(ar::AlvarMarkers, ar_pose_markers)
  ACCESSname(FilterObjects, perceptual_inputs)

//  ros::NodeHandle* nh;
//  ros::Publisher pub;

  Collector();

  virtual void open(){}
  virtual void step();
  virtual void close(){}

  int tabletop_revision = 0;
  int ar_pose_markers_revision = 0;

  uint count = 0;
};
