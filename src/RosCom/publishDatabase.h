#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <RosCom/roscom.h>

#include <RosCom/filterObject.h>

struct PublishDatabase : Module{
  ACCESSname(ors::KinematicWorld, modelWorld)
  ACCESSname(FilterObjects, object_database)

  PublishDatabase();

  ros::NodeHandle* nh;
  ros::Publisher cluster_pub;
  ros::Publisher alvar_pub;
  ros::Publisher plane_pub;

  virtual void open();
  virtual void step();
  virtual void close();

private:
  void syncCluster(const Cluster* cluster);
  void syncAlvar(const Alvar* alvar);
  mlr::Array<uint> stored_clusters, stored_alvars;
};
