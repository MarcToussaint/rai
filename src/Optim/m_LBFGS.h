#pragma once

#include "NLP.h"
#include "options.h"

namespace rai {

struct LBFGS{
  ScalarFunction f;
  shared_ptr<rai::OptOptions> opt;
  arr x;

  LBFGS(ScalarFunction _f, const arr& x_init, std::shared_ptr<OptOptions> _opt);

  std::shared_ptr<SolverReturn> solve();

//private:
  arr g;
};

} //namespace
