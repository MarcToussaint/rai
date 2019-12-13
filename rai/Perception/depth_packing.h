/*  ------------------------------------------------------------------
    Copyright (c) 2019 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>

struct KinectDepthPacking : Thread {
  struct sKinectDepthPacking* s;
  Var<uint16A> kinect_depth;
  Var<byteA> kinect_depthRgb;
  KinectDepthPacking() : Thread("KinectDepthPacking"), kinect_depth(this, true) {}
  void open();
  void step();
  void close();
};

namespace rai {
// pack 16bit depth image into 3 8-bit channels
void pack_kindepth2rgb(const uint16A& depth, byteA& buffer);
}
