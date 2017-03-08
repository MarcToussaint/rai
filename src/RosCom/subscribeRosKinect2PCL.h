#pragma once
#include "roscom.h"

#include <sensor_msgs/PointCloud2.h>
#include <PCL/conv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct SubscribeRosKinect2PCL{
  Access_typed<Pcl> cloud;
  Access_typed<mlr::Transformation> kinect_frame;
  SubscriberConv<sensor_msgs::PointCloud2, Pcl, &conv_pointcloud22pcl> subPoints;

  SubscribeRosKinect2PCL(const char* cloud_name="pclRawInput")
    : cloud(NULL, cloud_name),
      kinect_frame(NULL, "kinect_frame"),
      subPoints("/kinect_head/depth_registered/points", cloud){
  }
  ~SubscribeRosKinect2PCL(){
  }

};
