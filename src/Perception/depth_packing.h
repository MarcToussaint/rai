#pragma once

#include <Core/module.h>

BEGIN_MODULE(KinectDepthPacking)
  ACCESSlisten(uint16A, kinect_depth);
  ACCESSnew(byteA, kinect_depthRgb);
END_MODULE()

namespace MLR {
  // pack 16bit depth image into 3 8-bit channels
  void pack_kindepth2rgb(const uint16A& depth, byteA& buffer);
}
