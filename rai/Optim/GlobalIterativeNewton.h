/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

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
  LocalMinimum* best;

  GlobalIterativeNewton(const ScalarFunction& f, const arr& bounds_lo, const arr& bounds_up, OptOptions o=NOOPT);
  ~GlobalIterativeNewton();

  void step();
  void run(uint maxIt=10);
  void report();

  void reOptimizeAllPoints();
};
