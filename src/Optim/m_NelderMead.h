#pragma once

#include "NLP.h"
#include "../Core/util.h"

struct NelderMead {
  shared_ptr<NLP> P;

  //-- buffers to avoid re-evaluating points
  arr x;               ///< point where P was last evaluated
  double f_x=0.;

  //-- counters
  uint steps=0, tinySteps=0, maxSteps=300;

  NelderMead(shared_ptr<NLP> P, const arr& x_init={});

  shared_ptr<SolverReturn> solve();
};
