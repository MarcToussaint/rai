#pragma once

#include "NLP.h"
#include "options.h"

namespace rai {

struct LBFGS{
  shared_ptr<NLP> P;
  shared_ptr<rai::OptOptions> opt;
  arr x;

  LBFGS(std::shared_ptr<NLP> _P, const arr& x_init, std::shared_ptr<OptOptions> _opt);

  std::shared_ptr<SolverReturn> solve();

//private:
  arr g;
};

} //namespace
