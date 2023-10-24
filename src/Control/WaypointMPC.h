#pragma once

#include "../KOMO/komo.h"

//===========================================================================

//A wrapper of KOMO to optimize waypoints for a given sequence of constraints
struct WaypointMPC{
  KOMO& komo;

  arr qHome;
  uint steps=0;
  //result
  arr path;
  arr tau;
  bool feasible=false;
  rai::String msg;

  WaypointMPC(KOMO& _komo, const arr& qHome={});

  void reinit(const rai::Configuration& C);
  void solve(int verbose);
};
