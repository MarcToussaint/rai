/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>
#include "optimization.h"

int optNewton(arr& x, const ScalarFunction& f, OptOptions opt=NOOPT);

struct OptNewton{
  arr& x;
  ScalarFunction f;
  OptOptions o;
  arr *additionalRegularizer;

  enum StopCriterion { stopNone=0, stopCrit1, stopCrit2, stopCritEvals, stopStepFailed };
  double fx;
  arr gx, Hx;
  double alpha, beta;
  uint it=0, evals=0, numTinySteps=0;
  StopCriterion stopCriterion;
  arr bound_lo, bound_hi;
  bool rootFinding=false;
  ofstream *fil=NULL;


  OptNewton(arr& x, const ScalarFunction& f, OptOptions o=NOOPT);
  ~OptNewton();
  StopCriterion step();
  StopCriterion run(uint maxIt = 1000);
  void reinit(const arr& _x);
};
