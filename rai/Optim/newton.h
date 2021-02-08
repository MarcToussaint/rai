/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"

int optNewton(arr& x, const ScalarFunction& f, OptOptions opt=NOOPT);

struct OptNewton {
  arr& x;
  ScalarFunction f;
  OptOptions options;

  enum StopCriterion { stopNone=0, stopDeltaConverge, stopTinyFSteps, stopTinyXSteps, stopCritEvals, stopStepFailed };

  OptNewton(arr& x, const ScalarFunction& f, OptOptions options=NOOPT, ostream* _logFile=0);
  ~OptNewton();
  OptNewton& setBounds(const arr& _bounds_lo, const arr& _bounds_up){
    bounds_lo = _bounds_lo;
    bounds_up = _bounds_up;
    if(x.N){
      CHECK_EQ(bounds_lo.N, x.N, "");
      CHECK_EQ(bounds_up.N, x.N, "");
      boundClip(x, bounds_lo, bounds_up);
      reinit(x);
    }
    return *this;
  }
  void reinit(const arr& _x);

  StopCriterion step();
  StopCriterion run(uint maxIt = 1000);

public:
  double fx;
  arr gx, Hx;
  double alpha, beta;
  uint its=0, evals=0, numTinyFSteps=0, numTinyXSteps=0;
  StopCriterion stopCriterion;
  arr bounds_lo, bounds_up;
  bool rootFinding=false;
  ostream* logFile=nullptr, *simpleLog=nullptr;
  double timeNewton=0., timeEval=0.;
};
