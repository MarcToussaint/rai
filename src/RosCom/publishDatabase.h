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
  ros::Publisher optitrackmarker_pub;
  ros::Publisher optitrackbody_pub;


  virtual void open();
  virtual void step();
  virtual void close();

private:
  void syncCluster(const Cluster* cluster);
  void syncAlvar(const Alvar* alvar);
  void syncOptitrackMarker(const OptitrackMarker* optitrackmarker);
  void syncOptitrackBody(const OptitrackBody* optitrackbody);
  mlr::Array<uint> stored_clusters, stored_alvars, stored_optitrackmarkers, stored_optitrackbodies;
  int revision = -1;
};
