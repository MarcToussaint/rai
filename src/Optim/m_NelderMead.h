#pragma once

#include "NLP.h"
#include "options.h"
#include "../Core/util.h"

namespace rai {

struct NelderMead {
  ScalarFunction f;
  shared_ptr<OptOptions> opt;
  arr x;
  double f_x=0.;

  //-- counters
  uint steps=0, tinySteps=0, maxSteps=300;

  NelderMead(ScalarFunction _f, const arr& x_init={}, shared_ptr<OptOptions> opt = make_shared<OptOptions>());

  shared_ptr<SolverReturn> solve();
};

} //namespace
