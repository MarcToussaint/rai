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
  OptOptions o;

  enum StopCriterion { stopNone=0, stopDeltaConverge, stopTinyFSteps, stopTinyXSteps, stopCritEvals, stopStepFailed };
  double fx;
  arr gx, Hx;
  double alpha, beta;
  uint its=0, evals=0, numTinyFSteps=0, numTinyXSteps=0;
  StopCriterion stopCriterion;
  arr bound_lo, bound_up;
  bool rootFinding=false;
  ostream* logFile=nullptr, *simpleLog=nullptr;
  double timeNewton=0., timeEval=0.;

  OptNewton(arr& x, const ScalarFunction& f, OptOptions o=NOOPT, ostream* _logFile=0);
  ~OptNewton();
  StopCriterion step();
  StopCriterion run(uint maxIt = 1000);
  void reinit(const arr& _x);
};
