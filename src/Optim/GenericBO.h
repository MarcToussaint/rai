#pragma once

#include "NLP.h"
#include "options.h"
#include "../Core/util.h"
#include "m_EvoStrategies.h"

namespace rai {

struct GenericBO{
  shared_ptr<NLP> P;
  std::shared_ptr<OptOptions> opt;
  str method;

  arr best_x, best_phi;
  double best_f=1e20;

  uint evals=0, tinySteps=0, rejectedSteps=0;

  GenericBO(str method, shared_ptr<NLP> _P, const arr& x_init, std::shared_ptr<OptOptions> _opt) : P(_P), opt(_opt), method(method), best_x(x_init) {}

  virtual arr generateSamples() = 0;
  virtual void update(arr& X, arr& Phi, arr& F) = 0;

  shared_ptr<SolverReturn> solve();
  bool step();
protected:
  void evaluateSamples(arr& Phi, arr& F, const arr& X);
  void update_best(const arr& X, const arr& Phi, const arr& F);
  friend struct LS_CMA;
};

} //namespace
