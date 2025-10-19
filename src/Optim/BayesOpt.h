/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "GlobalIterativeNewton.h"
#include "../Core/array.h"

struct KernelRidgeRegression;
struct DefaultKernelFunction;

namespace rai {

struct BayesOpt {
  ScalarFunction f;
  arr bounds;

  arr data_X;
  arr data_y;

  KernelRidgeRegression* f_now;
  KernelRidgeRegression* f_smaller;

  GlobalIterativeNewton alphaMinima_now;
  GlobalIterativeNewton alphaMinima_smaller;

  DefaultKernelFunction* kernel_now;
  DefaultKernelFunction* kernel_smaller;
  double lengthScale;

  //lengthScale is always relative to hi-lo
  BayesOpt(ScalarFunction f, const arr& bounds, shared_ptr<OptOptions> opt, double init_lengthScale=1., double prior_var=1.);
  ~BayesOpt();

  void step();
  void run(uint maxIt=10);
  void report(bool display, ScalarFunction f);

 private:
  void addDataPoint(const arr& x, double y); //and update the regressions
  void reOptimizeAlphaMinima();
  arr pickNextPoint();
  void reduceLengthScale();
};

} //namespace
