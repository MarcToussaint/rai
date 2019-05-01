#include <RosCom/roscom.h>

#include "rosCamera.h"

struct sRosCamera {
  RosCom ROS;
  ptr<SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>> subRgb;
  ptr<SubscriberConv<sensor_msgs::Image, floatA, &conv_imageFloat32_floatA>> subDepth;

  sRosCamera(const char* rosNodeName, Var<byteA>& rgb, Var<floatA>& depth) : ROS(rosNodeName){
    if(rgb.name().N) ROS.subscribe(subRgb, rgb);
    if(depth.name().N) ROS.subscribe(subDepth, depth);
  }
};

RosCamera::RosCamera(Var<byteA>& _rgb, Var<floatA> _depth,
                     const char* rosNodeName,
                     const char* rgb_topic,
                     const char* depth_topic)
  : rgb(_rgb), depth(_depth) {
  if(rgb_topic) rgb.name() = rgb_topic;
  if(depth_topic) depth.name() = depth_topic;
  s = make_shared<sRosCamera>(rosNodeName, rgb, depth);
}

RosCamera::~RosCamera(){
}
