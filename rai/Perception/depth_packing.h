#pragma once

#include <Core/thread.h>

struct KinectDepthPacking : Thread {
  struct sKinectDepthPacking *s;
  ACCESSlisten(uint16A, kinect_depth);
  VAR(byteA, kinect_depthRgb);
  KinectDepthPacking() : Thread("KinectDepthPacking"){}
  void open();
  void step();
  void close();
};

namespace mlr {
  // pack 16bit depth image into 3 8-bit channels
  void pack_kindepth2rgb(const uint16A& depth, byteA& buffer);
}
