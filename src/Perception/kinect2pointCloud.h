#pragma once

#include <Core/thread.h>
#include <Gui/opengl.h>
#include <Geo/geo.h>

struct Kinect2PointCloud : Thread{
  ACCESS(byteA, kinect_rgb)
  ACCESSlisten(uint16A, kinect_depth)

//  ACCESS(mlr::Transformation, kinect_frame)
  ACCESS(arr, kinect_points)
  ACCESS(arr, kinect_pointColors)

  arr pts,cols;
  uint16A depth;
  byteA rgb; //helpers

  Kinect2PointCloud() : Thread("Kinect2PointCloud"){}
  virtual ~Kinect2PointCloud(){}

  void open(){}
  void step();
  void close(){}
};

/// convert raw image data into depth and color arrays like in a pointcloud
void images2pointcloud(arr& pts, arr& cols, const byteA& rgb, const uint16A& depth);

/// convert raw depth data to a pointcloud (no color)
void depthData2pointCloud(arr& pts, const uint16A& depth);
