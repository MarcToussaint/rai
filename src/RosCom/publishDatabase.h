#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <RosCom/roscom.h>

#include <RosCom/filterObject.h>

struct PublishDatabase : Module{
  ACCESSname(FilterObjects, object_database)

  PublishDatabase();

  ros::NodeHandle* nh;
  ros::Publisher tabletop_pub;
  ros::Publisher alvar_pub;

  virtual void open();
  virtual void step();
  virtual void close();

};
