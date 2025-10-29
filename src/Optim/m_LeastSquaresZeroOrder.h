#pragma once

#include "NLP.h"
#include "options.h"
#include "../Core/util.h"

namespace rai {

struct LeastSquaredZeroOrder{
  shared_ptr<NLP> P;
  std::shared_ptr<OptOptions> opt;

  //-- parameters
  str method="rank1";
  double alpha = .5;
  RAI_PARAM("LSZO/", double, alpha_min, .001)
  RAI_PARAM("LSZO/", double, damping, 1e-2)
  RAI_PARAM("LSZO/", double, noiseRatio, .2)
  RAI_PARAM("LSZO/", double, noiseAbs, .0)
  RAI_PARAM("LSZO/", int, maxIters, 500)
  RAI_PARAM("LSZO/", double, dataRatio, 1.)
  RAI_PARAM("LSZO/", bool, pruneData, false)
  RAI_PARAM("LSZO/", bool, covariantNoise, false)
  RAI_PARAM("LSZO/", double, stepInc, 1.5)
  RAI_PARAM("LSZO/", double, stepDec, .5)
  RAI_PARAM("LSZO/", uint, lambda, 1)
  RAI_PARAM("LSZO/", uint, mu, 1)

  //-- state and data
  arr x;               ///< point where P was last evaluated
  arr elite_X, elite_Phi;
  arr J, phi0, Hinv;
  arr phi_x; //, J_x, H_x; ///< features at x
  double phi2_x=-1.;
  arr data_X, data_Phi;

  uint steps=0, tinySteps=0, rejectedSteps=0;

  LeastSquaredZeroOrder(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt=make_shared<OptOptions>());

  shared_ptr<SolverReturn> solve(){
    while(!step()){}
    shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
    ret->x = x;
    arr err = P->summarizeErrors(phi_x);
    ret->f = err(OT_f);
    ret->sos = err(OT_sos);
    ret->eq = err(OT_eq);
    ret->ineq = err(OT_ineq);
    ret->feasible=true;
    return ret;
  }

  bool step();

  void updateJ_rank1(arr& J, const arr& x, const arr& x_last, const arr& phi, const arr& phi_last);

  arr generateNewSamples();
  void update(arr& y, arr& Phi);
};


//===========================================================================

struct LSGaussEDA {
  shared_ptr<NLP> P;
  std::shared_ptr<OptOptions> opt;
  arr mean;
  arr cov;
  arr elite_X, elite_y;
  RAI_PARAM("GaussEDA/", double, sigmaInit, .1)
  RAI_PARAM("GaussEDA/", double, momentum, .5)
  RAI_PARAM("GaussEDA/", double, beta, .1)
  RAI_PARAM("ES/", double, sigmaDecay, .001)
  RAI_PARAM("ES/", uint, lambda, 20)
  RAI_PARAM("ES/", uint, mu, 5)

  uint steps=0, tinySteps=0, rejectedSteps=0;

  LSGaussEDA(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt=make_shared<OptOptions>());

  virtual arr generateNewSamples();
  virtual void update(arr& X, arr& Phi);
  bool step();


  shared_ptr<SolverReturn> solve(){
    while(!step()){}
    shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
    ret->x = elite_X[0];
    arr err = P->summarizeErrors(elite_y[0]);
    ret->f = err(OT_f);
    ret->sos = err(OT_sos);
    ret->eq = err(OT_eq);
    ret->ineq = err(OT_ineq);
    ret->feasible=true;
    return ret;
  }

};

//===========================================================================

std::tuple<arr,arr> updateJ_linReg(const arr& Xraw, const arr& Y, const arr& x_now, double dataRatio);

} //namespace
