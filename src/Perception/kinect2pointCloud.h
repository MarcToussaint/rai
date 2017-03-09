#pragma once

#include <Core/thread.h>
#include <Gui/opengl.h>
#include <Geo/geo.h>

struct Kinect2PointCloud : Thread{
  //inputs
  ACCESS(byteA, kinect_rgb)
  ACCESSlisten(uint16A, kinect_depth)
  ACCESS(mlr::Transformation, kinect_frame)
  //outputs
  ACCESS(arr, kinect_points)

  arr pts,cols;
  uint16A depth;
  byteA rgb; //helpers
  mlr::Transformation frame;
  int depthShift_dx,depthShift_dy;
  arr frameShift;

  Kinect2PointCloud();
  virtual ~Kinect2PointCloud();

  void open(){}
  void step();
  void close(){}
};

/// convert raw depth data to a pointcloud (no color)
void depthData2pointCloud(arr& pts, const uint16A& depth, int depthShift_dx=0, int depthShift_dy=0);
