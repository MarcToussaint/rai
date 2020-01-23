#pragma once

#include "../Core/thread.h"
#include "../Core/array.h"
#include "../RosCom/roscom.h"

#include "../Perception/percept.h"

/// 'publishes' to both, ROS topics and modelWorld
struct PublishDatabase : Thread{
  Var<PerceptL> percepts_filtered;
  VAR(rai::Configuration, modelWorld)

  PublishDatabase();

  ros::NodeHandle* nh;
  ros::Publisher cluster_pub;
  ros::Publisher alvar_pub;
  ros::Publisher plane_pub;
  ros::Publisher optitrackmarker_pub;
  ros::Publisher optitrackbody_pub;
  ros::Publisher plane_marker_pub; // Publish plane info as visualization markers.

  virtual void open();
  virtual void step();
  virtual void close();

private:
  //TODO: these should be virtual methods of Percept
  rai::Array<uint> stored_clusters, stored_alvars, stored_planes, stored_optitrackmarkers, stored_optitrackbodies;
  int revision = -1;
};
