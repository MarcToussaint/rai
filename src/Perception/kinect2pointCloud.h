/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/thread.h"
#include "../Gui/opengl.h"
#include "../Geo/geo.h"

struct Kinect2PointCloud : Thread {
  //inputs
  Var<byteA> kinect_rgb;
  Var<uint16A> kinect_depth;
  Var<rai::Transformation> kinect_frame;
  Var<arr> pr2_odom;
  //outputs
  Var<arr> kinect_points;

  arr pts, cols;
  uint16A depth;
  byteA rgb; //helpers
  rai::Transformation frame;
  int depthShift_dx, depthShift_dy;
  arr frameShift;

  Kinect2PointCloud();
  virtual ~Kinect2PointCloud();

  void open() {}
  void step();
  void close() {}
};

/// convert raw depth data to a pointcloud (no color)
void depthData2pointCloud(arr& pts, const uint16A& depth, int depthShift_dx=0, int depthShift_dy=0);
