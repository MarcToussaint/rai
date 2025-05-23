/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"
#include "../Optim/options.h"
#include "../Algo/spline.h"

struct SolverReturn;

//A wrapper of TimingOpt optimize the timing (and vels) along given waypoints, and progressing/backtracking the phase
struct TimingMPC {
  //inputs
  arr waypoints;
  arr tangents;
  //outputs
  arr vels;
  arr tau;

  //optimization parameters
  arr warmstart_dual;
  double timeCost;
  double ctrlCost;
  rai::OptOptions opt;

  //tangent options
  bool useNextWaypointTangent=true;

  //phase management
  uint phase=0;
  uintA backtrackingTable;

  bool neverDone=false;

  TimingMPC(const arr& _waypoints, double _timeCost=1e0, double _ctrlCost=1e0);

  shared_ptr<SolverReturn> solve(const arr& x0, const arr& v0, int verbose=1);

  uint nPhases() const { return waypoints.d0; }
  bool done() const { return phase>=nPhases(); }
  arr getWaypoints() const { if(done()) return waypoints[-1].copy().reshape(1, -1); return waypoints({phase, -1+1}).copy(); }
  arr getTimes() const { if(done()) return {.1}; return integral(tau({phase, -1+1})); }
  arr getVels() const;

  bool set_progressedTime(double gap, double tauCutoff=0.);
  void set_updatedWaypoints(const arr& _waypoints, bool setNextWaypointTangent);

  void update_backtrack();
  void update_setPhase(uint phaseTo);

  void getCubicSpline(rai::CubicSpline& S, const arr& x0, const arr& v0) const;
};
