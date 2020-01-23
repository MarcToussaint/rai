#pragma once

#include "roscom.h"

struct SubscribeRosKinect{
  Var<byteA> kinect_rgb;
  Var<uint16A> kinect_depth;
  Var<rai::Transformation> kinect_frame;
//  Var<visualization_msgs::MarkerArray> tableTopObjects;
  SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA> subRgb;
  SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A> subDepth;
//  SubscriberConv<sensor_msgs::PointCloud2, uint16A, &conv_pointcloud2uint16A> subPoints;

  SubscribeRosKinect()
    : kinect_rgb(nullptr, "kinect_rgb"),
      kinect_depth(nullptr, "kinect_depth"),
      kinect_frame(nullptr, "kinect_frame"),
      subRgb("/kinect_head/rgb/image_rect_color", kinect_rgb),
      subDepth("/kinect_head/depth_registered/image_rect_raw", kinect_depth, &kinect_frame){
//      subDepth("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame){
  }
  ~SubscribeRosKinect(){
  }

};
