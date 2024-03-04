/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "WaypointMPC.h"
#include "ShortPathMPC.h"
#include "TimingMPC.h"

//===========================================================================

namespace rai {
struct SecMPC_Options {
  RAI_PARAM("SecMPC/", int, verbose, 1)
  RAI_PARAM("SecMPC/", double, precision, .1)
  RAI_PARAM("SecMPC/", double, tauCutoff, .0)
};
}//namespace

//===========================================================================

struct SecMPC {
  WaypointMPC waypointMPC;
  TimingMPC timingMPC;
  ShortPathMPC shortMPC;

  int subSeqStart=0, subSeqStop=-1;
  bool setNextWaypointTangent;
  rai::String msg;

  double ctrlTimeDelta = 0.;
  double ctrlTime_atLastUpdate = -1.;
  arr q_ref_atLastUpdate, qDot_ref_atLastUpdate, q_refAdapted;
  bool phaseSwitch=false;
  int tauStalling=0;
  int wayInfeasible=0;

  rai::SecMPC_Options opt;

  SecMPC(KOMO& komo, int subSeqStart=0, int subSeqStop=-1, double timeCost=1e0, double ctrlCost=1e0, bool _setNextWaypointTangent=true, const StringA& explicitCollisions= {});

  void updateWaypoints(const rai::Configuration& C);
  void updateTiming(const rai::Configuration& C, const ObjectiveL& phi, const arr& q_real);
  void updateShortPath(const rai::Configuration& C);
  void cycle(const rai::Configuration& C, const arr& q_ref, const arr& qDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime);
  rai::CubicSplineCtor getSpline(double realtime, bool prependRef=false);
  rai::CubicSplineCtor getShortPath(double realtime);
  rai::CubicSplineCtor getShortPath_debug(double realtime);
  void report(const rai::Configuration& C);
};

