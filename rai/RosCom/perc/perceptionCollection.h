#pragma once

#include "../Core/thread.h"
#include "../Perception/percept.h"
#include "../RosCom/roscom.h"

#include <ar_track_alvar_msgs/AlvarMarkers.h>
namespace ar = ar_track_alvar_msgs;
#include <object_recognition_msgs/TableArray.h>

PercCluster conv_ROSMarker2Cluster(const visualization_msgs::Marker& marker);
PercPlane conv_ROSTable2Plane(const object_recognition_msgs::Table& table);
PercAlvar conv_ROSAlvar2Alvar(const ar::AlvarMarker& marker);
OptitrackMarker conv_tf2OptitrackMarker(const geometry_msgs::TransformStamped& msg);
OptitrackBody conv_tf2OptitrackBody(const geometry_msgs::TransformStamped& msg);

/** listens to ros topics, such as Alvar and OptiTrack and TableTop, and pipes them into the percepts_input
 */
struct Collector : Thread{
  Var<visualization_msgs::MarkerArray> tabletop_clusters;
  Var<ar::AlvarMarkers> ar_pose_markers;
  Var<std::vector<geometry_msgs::TransformStamped>> opti_markers;
  Var<std::vector<geometry_msgs::TransformStamped>> opti_bodies;
  Var<object_recognition_msgs::TableArray> tabletop_tableArray;

  VAR(rai::Transformation, tabletop_srcFrame)
  VAR(rai::Transformation, alvar_srcFrame)
  VAR(rai::Transformation, optitrack_srcFrame)

  VAR(PerceptL, percepts_input)

  Collector(const bool simulate = false);

  virtual void open(){}
  virtual void step();
  virtual void close(){}

private:
  bool useRos = rai::getParameter<bool>("useRos", false);
  int tabletop_clusters_revision = 0;
  int ar_pose_markers_revision = 0;
  int tabletop_tableArray_revision = 0;
  int opti_markers_revision = 0;
  int opti_bodies_revision = 0;

  bool simulate;

  bool has_tabletop_transform = false;
  bool has_alvar_transform = false;
};
