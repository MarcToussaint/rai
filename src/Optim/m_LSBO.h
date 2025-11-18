#pragma once

#include "NLP.h"
#include "options.h"
#include "../Core/util.h"
#include "m_CMA.h"
#include "GenericBO.h"

namespace rai {

struct LeastSquaresBlackboxOpt : GenericBO {

  //-- parameters
  str method="rank1";
  double alpha, beta;
  RAI_PARAM("LSBO/", int, lambda, 1)
  RAI_PARAM("LSBO/", double, minDamping, 1e-2)
  RAI_PARAM("LSBO/", double, maxDamping, 1e+2)
  RAI_PARAM("LSBO/", double, noiseRatio, .1)
  RAI_PARAM("LSBO/", double, weightRatio, 2.)

  //-- state and data
  arr x, J, phi0, Hinv;
  arr data_X, data_Phi;
  double data_radius;

  LeastSquaresBlackboxOpt(shared_ptr<NLP> _P, const arr& x_init, std::shared_ptr<OptOptions> _opt=make_shared<OptOptions>());

  arr generateSamples();
  void update(arr& X, arr& Phi, arr& F);


private:
  void pruneData();
  void updateJ_rank1(arr& J, const arr& x, const arr& x_last, const arr& phi, const arr& phi_last);
};


//===========================================================================

std::tuple<arr,arr> updateJ_linReg(const arr& Xraw, const arr& Y, const arr& x_now, double weightRatio);

//===========================================================================

struct LS_CMA : GenericBO {

  CMAES cma;
  LeastSquaresBlackboxOpt lsdf;

  arr X_cma, X_lsdf;
  LS_CMA(shared_ptr<NLP> P, const arr& x_init, std::shared_ptr<OptOptions> _opt=make_shared<OptOptions>());

  virtual arr generateSamples();
  virtual void update(arr& X, arr& Phi, arr& F);
};


} //namespace
