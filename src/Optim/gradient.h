/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>
#include "optimization.h"

//===========================================================================
//
// proper (monotone) plain gradient descent with line search
//

struct OptGrad{
  arr& x;
  ScalarFunction f;
  OptOptions o;

  enum StopCriterion { stopNone=0, stopCrit1, stopCrit2, stopCritLineSteps, stopCritEvals, stopStepFailed };
  double fx;
  arr gx;
  double alpha;
  uint it, evals, numTinySteps;
  StopCriterion stopCriterion;
  ofstream fil;

  OptGrad(arr& x, const ScalarFunction& f, OptOptions o=NOOPT);
  ~OptGrad();
  StopCriterion step();
  StopCriterion run(uint maxIt = 1000);
  void reinit(const arr& _x=NoArr);
};

inline int optGrad(arr& x, const ScalarFunction& f, OptOptions opt=NOOPT){
  return OptGrad(x, f, opt).run();
}


//===========================================================================
//
// Rprop
//

/** Rprop, a fast gradient-based minimization */
struct Rprop {
  struct sRprop *s;
  Rprop();
  ~Rprop();
  void init(double initialStepSize=1., double minStepSize=1e-6, double maxStepSize=50.);
  bool step(arr& x, const ScalarFunction& f);
  uint loop(arr& x, const ScalarFunction& f, double *fmin_return=NULL, double stoppingTolerance=1e-2, double initialStepSize=1., uint maxIterations=1000, uint verbose=0);
};

inline uint optRprop(arr& x, const ScalarFunction& f, OptOptions opt=NOOPT){
  return Rprop().loop(x, f, opt.fmin_return, opt.stopTolerance, opt.initStep, opt.stopEvals, opt.verbose);
}



