/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../KOMO/komo.h"

struct ShortPathMPC {
  KOMO komo;
  arr qHome;
  uint iters=0;

  double defaultTau;
  int sliceOfConstraint;

  arr x0, v0;

  //results
  arr times;
  arr path;
  arr tau;
  arr vels;
  bool feasible=false;
  rai::String msg;

  ShortPathMPC(rai::Configuration& C, uint steps=10, double _defaultTau=.1);

  void reinit_taus(double timeToConstraint);
  void reinit(const arr& x, const arr& v);  //update robot state
  void reinit(const rai::Configuration& C); //update object movements

  std::shared_ptr<SolverReturn> solve(bool alsoVels, int verbose);
  arr getPath();
};
