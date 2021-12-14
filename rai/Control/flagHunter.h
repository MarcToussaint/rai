#pragma once

#include "../Core/array.h"
#include "../Optim/options.h"
#include "../Algo/spline.h"

struct SolverReturn;

struct FlagHuntingControl{
  arr flags;
  arr tangents;
  arr vels;
  arr tau;
  //optimization parameters
  double alpha = 1e4;
  rai::OptOptions opt;
  uint phase=0;

  FlagHuntingControl(const arr& _flags, double _alpha=1e4);

  shared_ptr<SolverReturn> solve(const arr& x0, const arr& v0, int verbose=1);

  bool done() const{ return phase>=flags.d0; }
  arr getFlags() const{ return flags({phase, -1}).copy(); }
  arr getTimes() const{ return integral(tau({phase, -1})); }
  arr getVels() const{
      arr _vels = vels({phase, -1}).copy();
      if(tangents.N) _vels = _vels % tangents;
      _vels.append(zeros(flags.d1));
      _vels.reshape(flags.d0 - phase, flags.d1);
      return _vels;
  }

  void getCubicSpline(rai::CubicSpline& S, const arr& x0, const arr& v0) const;
};
