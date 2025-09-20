#pragma once

#include "NLP.h"
#include "options.h"

struct OptLBFGS{
  ScalarFunction& f;
  uint dimension;
  arr& x;
  rai::OptOptions opt;

  OptLBFGS(arr& x, ScalarFunction& f, rai::OptOptions o=DEFAULT_OPTIONS);

  std::shared_ptr<SolverReturn> solve();

//private:
  arr g;
};
