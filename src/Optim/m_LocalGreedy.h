#pragma once

#include "NLP.h"
#include "../Core/util.h"

struct LocalGreedy {
  RAI_PARAM("LocalGreedy/", double, sigma, .1)
  RAI_PARAM("opt/", int, verbose, 1)

  ScalarFunction f;


  //-- buffers to avoid re-evaluating points
  arr x;               ///< point where P was last evaluated
  double f_x=0.;

  //-- counters
  uint steps=0, tinySteps=0, maxSteps=300;

  LocalGreedy(ScalarFunction _f, const arr& x_init={});

  shared_ptr<SolverReturn> solve();

  bool step();
};
