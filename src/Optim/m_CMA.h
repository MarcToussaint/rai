#pragma once

#include "GenericBO.h"

namespace rai {

struct CMAES : GenericBO { //EvolutionStrategy {
  unique_ptr<struct CMA_self> self;
  RAI_PARAM("CMA/", int, lambda, 20)
  RAI_PARAM("CMA/", double, sigmaInit, .1)

  CMAES(shared_ptr<NLP> P, const arr& x_init, shared_ptr<OptOptions> opt = make_shared<OptOptions>());
  ~CMAES();

  virtual arr generateSamples();
  virtual void update(arr& X, arr& Phi, arr& F);

  arr getBestEver();
  arr getCurrentMean();
  double getSigma();

  void overwriteSamples(const arr& X);
  void overwriteMean(const arr& x);
};

} //namespace
