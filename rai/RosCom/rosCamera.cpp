#include <RosCom/roscom.h>

#include "rosCamera.h"

#ifdef RAI_ROS

struct sRosCamera {
  RosCom ROS;
  ptr<SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>> subRgb;
  ptr<SubscriberConv<sensor_msgs::Image, floatA, &conv_imageFloat32_floatA>> subDepthFloat;
  ptr<SubscriberConv<sensor_msgs::Image, floatA, &conv_imageu162floatA>> subDepthUint16;

  sRosCamera(const char* rosNodeName, Var<byteA>& rgb, Var<floatA>& depth, bool useUint) : ROS(rosNodeName){
    if(rgb.name().N) ROS.subscribe(subRgb, rgb);
    if(useUint){
      if(depth.name().N) ROS.subscribe(subDepthUint16, depth);
    }else{
      if(depth.name().N) ROS.subscribe(subDepthFloat, depth);
    }
  }
};

RosCamera::RosCamera(Var<byteA>& _rgb, Var<floatA> _depth,
                     const char* rosNodeName,
                     const char* rgb_topic,
                     const char* depth_topic,
                     bool useUint)
  : rgb(_rgb), depth(_depth) {
  if(rgb_topic) rgb.name() = rgb_topic;
  if(depth_topic) depth.name() = depth_topic;
  s = make_shared<sRosCamera>(rosNodeName, rgb, depth, useUint);
}

RosCamera::~RosCamera(){
}

#else //RAI_ROS

RosCamera::RosCamera(Var<byteA>& _rgb, Var<floatA> _depth,
		     const char* rosNodeName,
		     const char* rgb_topic,
		     const char* depth_topic){ NICO }
RosCamera::~RosCamera(){ NICO }

#endif
