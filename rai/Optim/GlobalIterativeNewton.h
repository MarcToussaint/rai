/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"
#include "newton.h"
#include "gradient.h"

struct GlobalIterativeNewton {
  arr x;
  OptNewton newton;
  OptGrad grad;
  arr bounds_lo, bounds_hi;
  
  struct LocalMinimum { arr x; double fx; uint hits; };
  rai::Array<LocalMinimum> localMinima;
  LocalMinimum *best;
  
  GlobalIterativeNewton(const ScalarFunction& f, const arr& bounds_lo, const arr& bounds_hi, OptOptions o=NOOPT);
  ~GlobalIterativeNewton();
  
  void step();
  void run(uint maxIt=10);
  void report();
  
  void reOptimizeAllPoints();
};
