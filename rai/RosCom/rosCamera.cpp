/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "rosCamera.h"
#include "../RosCom/roscom.h"

#ifdef RAI_ROS

struct sRosCamera {
  RosCom ROS;
  ptr<SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>> subRgb;
  ptr<SubscriberConv<sensor_msgs::Image, floatA, &conv_imageFloat32_floatA>> subDepthFloat;
  ptr<SubscriberConv<sensor_msgs::Image, floatA, &conv_imageu162floatA>> subDepthUint16;

  sRosCamera(const char* rosNodeName, Var<byteA>& rgb, Var<floatA>& depth, bool useUint) : ROS(rosNodeName) {
    if(rgb.name().N) ROS.subscribe(subRgb, rgb);
    if(useUint) {
      if(depth.name().N) ROS.subscribe(subDepthUint16, depth);
    } else {
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
  s = make_unique<sRosCamera>(rosNodeName, rgb, depth, useUint);
}

RosCamera::~RosCamera() {
}

#else //RAI_ROS

struct sRosCamera {};

RosCamera::RosCamera(Var<byteA>& _rgb, Var<floatA> _depth,
                     const char* rosNodeName,
                     const char* rgb_topic,
                     const char* depth_topic,
                     bool useUint) { NICO }
RosCamera::~RosCamera() { NICO }

#endif
