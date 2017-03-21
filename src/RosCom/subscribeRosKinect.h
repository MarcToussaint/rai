#pragma once
#include "roscom.h"

struct SubscribeRosKinect{
  Access_typed<byteA> kinect_rgb;
  Access_typed<uint16A> kinect_depth;
  Access_typed<mlr::Transformation> kinect_frame;
//  Access_typed<visualization_msgs::MarkerArray> tableTopObjects;
  SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA> subRgb;
  SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A> subDepth;
//  SubscriberConv<sensor_msgs::PointCloud2, uint16A, &conv_pointcloud2uint16A> subPoints;

  SubscribeRosKinect()
    : kinect_rgb(NULL, "kinect_rgb"),
      kinect_depth(NULL, "kinect_depth"),
      kinect_frame(NULL, "kinect_frame"),
      subRgb("/kinect_head/rgb/image_rect_color", kinect_rgb),
      subDepth("/kinect_head/depth_registered/image_rect_raw", kinect_depth, &kinect_frame){
//      subDepth("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame){
  }
  ~SubscribeRosKinect(){
  }

};
