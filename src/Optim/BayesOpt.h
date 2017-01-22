#pragma once

#include "optimization.h"
#include "GlobalIterativeNewton.h"

struct BayesOpt{
  ScalarFunction f;
  arr bounds_lo, bounds_hi;

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
  BayesOpt(const ScalarFunction& f, const arr& bounds_lo, const arr& bounds_hi, double init_lengthScale=1., OptOptions o=NOOPT);
  ~BayesOpt();

  void step();
  void run(uint maxIt=10);
  void report(bool display=true);

private:
  void addDataPoint(const arr& x, double y); //and update the regressions
  void reOptimizeAlphaMinima();
  arr pickNextPoint();
  void reduceLengthScale();
};
