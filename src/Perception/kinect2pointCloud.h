#pragma once

#include <Core/module.h>

struct Kinect2PointCloud: Module {
  ACCESS(byteA, kinect_rgb)
  ACCESS(MT::Array<uint16_t>, kinect_depth)

  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)

  arr pts,cols;
  floatA depth;
  byteA rgb; //helpers

  Kinect2PointCloud():Module("Kinect2PointCloud"){}
  virtual ~Kinect2PointCloud(){}

  void open(){}
  void step();
  void close(){}
};
