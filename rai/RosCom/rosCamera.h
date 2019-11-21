#pragma once

#include <Kin/kin.h>
#include <Core/thread.h>

struct RosCamera {
  Var<byteA> rgb;
  Var<floatA> depth;
  std::shared_ptr<struct sRosCamera> s;

  RosCamera(Var<byteA>& _rgb, Var<floatA> _depth,
            const char* rosNodeName = "rai_node",
            const char* rgb_topic = "/camera/rgb/image_rect_color",
            const char* depth_topic = "/camera/depth_registered/image_raw",
            bool useUint=false);
  ~RosCamera();
};
