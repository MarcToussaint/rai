/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>
#include <Gui/opengl.h>
#include <Geo/geo.h>

struct Kinect2PointCloud : Thread{
  //inputs
  VAR(byteA, kinect_rgb)
  VARlisten(uint16A, kinect_depth)
  VAR(rai::Transformation, kinect_frame)
  VAR(arr, pr2_odom)
  //outputs
  VAR(arr, kinect_points)

  arr pts,cols;
  uint16A depth;
  byteA rgb; //helpers
  rai::Transformation frame;
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
