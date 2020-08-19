/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Core/thread.h"
#include <memory>

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

