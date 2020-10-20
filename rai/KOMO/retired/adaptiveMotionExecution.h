/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifndef ADAPTIVEMOTIONEXECUTION_H
#define ADAPTIVEMOTIONEXECUTION_H

#include "../Core/util.h"
#include "../Core/array.h"
#include "../Core/array.ipp"
#include "../Kin/kin.h"
#include "../Core/util.h"
#include <stdlib.h>
#include "mobject.h"
#include "../Algo/spline.h"
#include "../Gui/opengl.h"

struct AdaptiveMotionExecution {

  AdaptiveMotionExecution(rai::Configuration& _world, arr& _trajRef, double _dt, double _TRef, arr& _x0, arr& _q0, MObject& _goalMO, \
                          bool _useOrientation);
  void printState();
  void plotState();
  void warpTrajectory();
  void iterate(arr& state, double _dtReal=0.);
  void getNextState(arr& state, arr& dstate);

  void moveGoal(arr& _pos);

  void computeIK(arr& q, arr& qd);
  rai::Configuration* world;

  double dt;
  double TRef;
  double dsRef;

  bool useOrientation;

  MObject* goalMO;

  // Actual Trajectory
  arr traj;
  arr x0; // start pos
  arr q0;
  arr s;
  arr goal;
  arr lastGoal;
  arr state;

  arr desState;
  arr desVel;

  // Costs
  arr posCosts;
  arr vecCosts;
  arr colCosts;

  // Wrapped Trajectory
  rai::Path* trajWrap;

  // Reference Trajectory
  rai::Path* trajRef;
  arr dtrajRef;
  arr goalRef;
  arr sRef;

  String scene;
  arr joints_bk;
};

#endif // ADAPTIVEMOTIONEXECUTION_H
