#pragma once

#include <Core/module.h>
#include <Gui/opengl.h>
#include <Core/geo.cpp>

extern ors::Camera kinectCam;
void initKinectCam();

struct Kinect2PointCloud: Module {
  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)

  ACCESS(ors::Transformation, kinect_frame)
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)

  arr pts,cols;
  uint16A depth;
  byteA rgb; //helpers

  Kinect2PointCloud():Module("Kinect2PointCloud"){}
  virtual ~Kinect2PointCloud(){}

  void open(){}
  void step();
  void close(){}
};

namespace MLR{
  /// convert raw image data into depth and color arrays like in a pointcloud
  void images2pointcloud(arr& pts, arr& cols, const byteA& rgb, const uint16A& depth);

  /// convert raw depth data to a pointcloud (no color)
  void depthData2pointCloud(arr& pts, const uint16A& depth);
}
