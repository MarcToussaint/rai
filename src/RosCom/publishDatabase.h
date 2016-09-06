#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <RosCom/roscom.h>

#include <RosCom/filterObject.h>

struct PublishDatabase : Module{
  Access_typed<FilterObjects> object_database;
  ACCESSname(ors::KinematicWorld, modelWorld)

  PublishDatabase();

  ros::NodeHandle* nh;
  ros::Publisher cluster_pub;
  ros::Publisher alvar_pub;
  ros::Publisher plane_pub;
  ros::Publisher plane_marker_pub; // Publish plane info as visualization markers.

  virtual void open();
  virtual void step();
  virtual void close();


  void syncCluster(const Cluster* cluster);
  void syncPlane(const Plane* plane);
  void syncAlvar(const Alvar* alvar);
  mlr::Array<uint> stored_clusters, stored_alvars, stored_planes;
  int revision = -1;
};
