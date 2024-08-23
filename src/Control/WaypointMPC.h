/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../KOMO/komo.h"

//===========================================================================

//A wrapper of KOMO to optimize waypoints for a given sequence of constraints
struct WaypointMPC {
  KOMO& komo;

  arr qHome;
  uint steps=0;
  //result
  arr path;
  arr tau;
  bool feasible=false;
  rai::String msg;

  WaypointMPC(KOMO& _komo, const arr& qHome= {});

  void reinit(const rai::Configuration& C);
  std::shared_ptr<SolverReturn> solve(int verbose);
};
