#pragma once

#include "../Core/array.h"
#include "../Optim/options.h"
#include "../Algo/spline.h"

struct FlagHuntingControl{
  arr flags;
  arr tangents;
  arr vels;
  arr tau;
  //optimization parameters
  double alpha = 1e4;
  rai::OptOptions opt;
  uint phase=0;

  FlagHuntingControl(const arr& _flags);

  void solve(const arr& x0, const arr& v0, int verbose=1);

  void getCubicSpline(rai::CubicSpline& S, const arr& x0, const arr& v0) const;
};
