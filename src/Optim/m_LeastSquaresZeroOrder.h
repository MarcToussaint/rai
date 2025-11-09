#pragma once

#include "NLP.h"
#include "options.h"
#include "../Core/util.h"
#include "m_EvoStrategies.h"

namespace rai {

struct GenericDF{
  shared_ptr<NLP> P;
  std::shared_ptr<OptOptions> opt;
  str method;

  RAI_PARAM("LSZO/", int, lambda, 1)

  arr best_x, best_phi;
  double best_f=1e20;

  uint evals=0, tinySteps=0, rejectedSteps=0;

  GenericDF(str method, shared_ptr<NLP> _P, std::shared_ptr<OptOptions> _opt) : P(_P), opt(_opt), method(method) {}

  virtual arr generateSamples(uint lambda) = 0;
  virtual void update(arr& X, arr& Phi) = 0;

  shared_ptr<SolverReturn> solve();
  arr evaluateSamples(const arr& X);
  bool step();
protected:
  void update_best(const arr& X, const arr& Phi);
  friend struct LS_CMA;
};

struct LeastSquaresDerivativeFree : GenericDF{

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

  //-- state and data
  arr x, J, phi0, Hinv;
  arr data_X, data_Phi;

  LeastSquaresDerivativeFree(shared_ptr<NLP> _P, const arr& x_init, std::shared_ptr<OptOptions> _opt=make_shared<OptOptions>());

  arr generateSamples(uint lambda);
  void update(arr& X, arr& Phi);

private:
  void updateJ_rank1(arr& J, const arr& x, const arr& x_last, const arr& phi, const arr& phi_last);
};


//===========================================================================

std::tuple<arr,arr> updateJ_linReg(const arr& Xraw, const arr& Y, const arr& x_now, double dataRatio);

//===========================================================================

struct LS_CMA : GenericDF{

  CMAES cma;
  LeastSquaresDerivativeFree lsdf;

  arr X_cma, X_lsdf;

  RAI_PARAM("LS_CMA/", int, ls_lambda, 1)

  LS_CMA(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt=make_shared<OptOptions>());

  virtual arr generateSamples(uint lambda);
  virtual void update(arr& X, arr& Phi);
};


} //namespace
