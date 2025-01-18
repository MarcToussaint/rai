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
  ScalarFunction f;
  arr& x;
  rai::OptOptions opt;

  enum StopCriterion { stopNone=0, stopDeltaConverge, stopTinyFSteps, stopTinyXSteps, stopCritEvals, stopStepFailed, stopLineSearchSteps };

  OptNewton(arr& _x, const ScalarFunction& _f, const rai::OptOptions& _opt);
  ~OptNewton();
  OptNewton& setBounds(const arr& _bounds);
  void reinit(const arr& _x);

  StopCriterion step();
  StopCriterion run(uint maxIt = 1000);

 public:
  arr bounds;
  double fx;
  arr gx, Hx;
  double alpha, beta;
  int inner_iters=0, evals=0, numTinyFSteps=0, numTinyXSteps=0;
  StopCriterion stopCriterion;
  bool rootFinding=false;
  double timeNewton=0., timeEval=0.;
};
