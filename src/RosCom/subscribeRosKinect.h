#include "roscom.h"

struct SubscribeRosKinect{
  Access_typed<byteA> kinect_rgb;
  Access_typed<uint16A> kinect_depth;
  Access_typed<visualization_msgs::MarkerArray> tableTopObjects;
  SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA> subRgb;
  SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A> subDepth;

  SubscribeRosKinect()
    : subRgb("/kinect_head/rgb/image_color", kinect_rgb),
      subDepth("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame),
      kinect_rgb(this, "kinect_rgb"),
      kinect_depth(this, "kinect_depth") {
  }
  ~SubscribeRosKinect(){
  }

};
