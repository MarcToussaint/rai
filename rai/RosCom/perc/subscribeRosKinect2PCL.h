#pragma once
//#include "roscom.h"

#ifdef MLR_PCL

#include <sensor_msgs/PointCloud2.h>
#include <PCL/conv.h>
#include <Geo/geo.h>

struct SubscribeRosKinect2PCL{
  struct sSubscribeRosKinect2PCL *s;
  Var<Pcl> cloud;
  Var<mlr::Transformation> kinect_frame;


  SubscribeRosKinect2PCL(const char* cloud_name="pclRawInput", const char* topic_name = "/kinect_head/depth_registered/points");
  ~SubscribeRosKinect2PCL();

  void callback(const typename sensor_msgs::PointCloud2::ConstPtr& msg);
};

#endif
