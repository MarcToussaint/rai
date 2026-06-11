/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>
#include <Geo/geo.h>

struct PointView{ uint j, k; arr p; double d=-1.; };
typedef rai::Array<PointView> PointViewA;

struct MultiViewSolver{

  // j enumerates marker points
  // k enumerates cameras
  // i enumerates data points

  uint J, K;
  uint selectedJ;
  uintA selectedI; //subselections of marker points and data points
  intA selectedJidx;

  arr X; //J world points
  arr P; //K camera matrices
  PointViewA data;

  MultiViewSolver(uint J_numMarkers, uint K_numCameras)
      : J(J_numMarkers), K(K_numCameras){}

  void setCamera(uint k, const arr& P_k);
  void setCamera(uint k, const arr& fxycxy, const rai::Transformation& X);

  void addDataPoint(uint j, uint k, const arr& p, double d=-1.);
  void subSelectObservedPoints();

  arr& solveColinearityForPoints();
};
