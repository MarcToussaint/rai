#pragma once

#include "NLP.h"
#include "../Core/util.h"

struct NelderMead {
  ScalarFunction f;
  arr x;
  double f_x=0.;

  //-- counters
  uint steps=0, tinySteps=0, maxSteps=300;

  NelderMead(ScalarFunction _f, const arr& x_init={});

  shared_ptr<SolverReturn> solve();
};
