#pragma once

#include <Core/module.h>

struct KinectDepthPacking:Module{
  struct sKinectDepthPacking *s;
  ACCESSlisten(uint16A, kinect_depth);
  ACCESSnew(byteA, kinect_depthRgb);
  KinectDepthPacking():Module("KinectDepthPacking"){}
  void open();
  void step();
  void close();
};

namespace MLR {
  // pack 16bit depth image into 3 8-bit channels
  void pack_kindepth2rgb(const uint16A& depth, byteA& buffer);
}
