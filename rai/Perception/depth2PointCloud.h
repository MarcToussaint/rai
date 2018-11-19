/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>
#include <Geo/geo.h>

struct Depth2PointCloud : Thread {
  //inputs
  Var<arr> depth;
  Var<rai::Transformation> pose;
  //outputs
  Var<arr> pts;
  
  double focalLength;
  arr _pts,_depth;

  Depth2PointCloud(Var<arr>& _depth, double _forcalLength=1.);
  virtual ~Depth2PointCloud();
  
  void open() {}
  void step();
  void close() {}
};

void depthData2pointCloud(arr& pts, const arr& depth, double focalLength);
