#pragma once

#include "NLP.h"
#include "options.h"
#include "../Core/util.h"
#include "m_EvoStrategies.h"

namespace rai {

struct LeastSquaresDerivativeFree{
  shared_ptr<NLP> P;
  std::shared_ptr<OptOptions> opt;

  //-- parameters
  str method="rank1";
  double alpha = .5;
  RAI_PARAM("LSZO/", double, alpha_min, .001)
  RAI_PARAM("LSZO/", double, damping, 1e-2)
  RAI_PARAM("LSZO/", double, noiseRatio, .2)
  RAI_PARAM("LSZO/", double, noiseAbs, .0)
  RAI_PARAM("LSZO/", double, dataRatio, 1.)
  RAI_PARAM("LSZO/", bool, pruneData, false)
  RAI_PARAM("LSZO/", bool, covariantNoise, false)
  RAI_PARAM("LSZO/", double, stepInc, 1.5)
  RAI_PARAM("LSZO/", double, stepDec, .5)
  RAI_PARAM("LSZO/", int, lambda, 1)
  RAI_PARAM("LSZO/", int, mu, 1)

  //-- state and data
  arr x;
  arr elite_X, elite_Phi;
  arr J, phi0, Hinv;
  arr phi_x;
  double phi2_x=-1.;
  arr data_X, data_Phi;

  uint evals=0, tinySteps=0, rejectedSteps=0;

  LeastSquaresDerivativeFree(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt=make_shared<OptOptions>());

  shared_ptr<SolverReturn> solve();

  bool step();

  void updateJ_rank1(arr& J, const arr& x, const arr& x_last, const arr& phi, const arr& phi_last);

  arr generateNewSamples(uint lambda);
  void update(arr& y, arr& Phi);
};


//===========================================================================

std::tuple<arr,arr> updateJ_linReg(const arr& Xraw, const arr& Y, const arr& x_now, double dataRatio);

//===========================================================================

struct LS_CMA{
  shared_ptr<NLP> P;
  std::shared_ptr<OptOptions> opt;

  CMAES cma;
  LeastSquaresDerivativeFree lsdf;
  arr x_best, phi_best;

  RAI_PARAM("LS_CMA/", int, ls_lambda, 1)

  uint evals=0, steps=0, tinySteps=0, rejectedSteps=0;

  LS_CMA(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt=make_shared<OptOptions>());

  virtual arr generateNewSamples();
  virtual void update(arr& X, arr& Phi);

  bool step();

  shared_ptr<SolverReturn> solve();

};


} //namespace
