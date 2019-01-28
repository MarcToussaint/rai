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
  Var<floatA> depth;
  Var<rai::Transformation> pose;
  //outputs
  Var<arr> points;
  
  float fx,fy,px,py;
  floatA _depth;
  arr _points;

  Depth2PointCloud(Var<floatA>& _depth, float _fx=NAN, float _fy=NAN, float _px=NAN, float _py=NAN);
  Depth2PointCloud(Var<floatA>& _depth, const arr& Fxypxy);
  virtual ~Depth2PointCloud();
  
  void open() {}
  void step();
  void close() {}
};

void depthData2pointCloud(arr& pts, const floatA& depth, float fx, float fy, float px, float py);
