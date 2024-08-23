/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "options.h"
#include "../Core/array.h"

int optNewton(arr& x, const ScalarFunction& f, rai::OptOptions opt=DEFAULT_OPTIONS);

struct OptNewton {
  arr& x;
  ScalarFunction f;
  rai::OptOptions options;

  enum StopCriterion { stopNone=0, stopDeltaConverge, stopTinyFSteps, stopTinyXSteps, stopCritEvals, stopStepFailed, stopLineSearchSteps };

  OptNewton(arr& x, const ScalarFunction& f, rai::OptOptions options=DEFAULT_OPTIONS, ostream* _logFile=0);
  ~OptNewton();
  OptNewton& setBounds(const arr& _bounds);
  void reinit(const arr& _x);

  StopCriterion step();
  StopCriterion run(uint maxIt = 1000);

 public:
  double fx;
  arr gx, Hx;
  double alpha, beta;
  int its=0, evals=0, numTinyFSteps=0, numTinyXSteps=0;
  StopCriterion stopCriterion;
  arr bounds;
  bool rootFinding=false;
  ostream* logFile=nullptr, *simpleLog=nullptr;
  double timeNewton=0., timeEval=0.;
};
