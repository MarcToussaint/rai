/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "GlobalIterativeNewton.h"
#include "../Core/array.h"

struct BayesOpt {
  ScalarFunction f;
  arr bounds;

  arr data_X;
  arr data_y;

  struct KernelRidgeRegression* f_now;
  struct KernelRidgeRegression* f_smaller;

  GlobalIterativeNewton alphaMinima_now;
  GlobalIterativeNewton alphaMinima_smaller;

  struct DefaultKernelFunction* kernel_now;
  struct DefaultKernelFunction* kernel_smaller;
  double lengthScale;

  //lengthScale is always relative to hi-lo
  BayesOpt(const ScalarFunction& f, const arr& bounds, rai::OptOptions& opt, double init_lengthScale=1., double prior_var=1.);
  ~BayesOpt();

  void step();
  void run(uint maxIt=10);
  void report(bool display=true, const ScalarFunction& f=ScalarFunction());

 private:
  void addDataPoint(const arr& x, double y); //and update the regressions
  void reOptimizeAlphaMinima();
  arr pickNextPoint();
  void reduceLengthScale();
};
