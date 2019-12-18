#pragma once

#include <memory>
#include <Core/array.h>
#include <Core/thread.h>

struct RosCamera;

namespace ry {

    struct RyCamera {
      Var<byteA> rgb;
      Var<floatA> depth;
      std::shared_ptr<RosCamera> C;
      RyCamera(const char* rosNodeName,
               const char* rgb_topic,
               const char* depth_topic,
               bool useUint=false)
        : C(make_shared<RosCamera>(rgb, depth, rosNodeName, rgb_topic, depth_topic, useUint)) {}
    };

};

