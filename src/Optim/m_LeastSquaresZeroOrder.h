#pragma once

#include "NLP.h"
#include "../Core/util.h"

struct LeastSquaredZeroOrder{
  shared_ptr<NLP> P;

  //-- parameters of the inner problem (Lagrangian, unconstrained problem)
  double sigma=1e-3;
  double lambda=1e-3;
  uint steps=0;

  //-- buffers to avoid re-evaluating points
  arr x;               ///< point where P was last evaluated
  double phi2_x=-1.;
  arr phi_x, J_x, H_x; ///< features at x
  arr J;
  arr data_X, data_Phi, data_Phi2;
  str method="rank1";

  LeastSquaredZeroOrder(shared_ptr<NLP> P, const arr& x_init={});

  shared_ptr<SolverReturn> solve(){
    while(!step()){}
    shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
    ret->x = x;
    ret->sos = phi2_x;
    ret->feasible=true;
    return ret;
  }

  bool step();

  void updateJ_rank1(arr& J, const arr& x, const arr& x_last, const arr& phi, const arr& phi_last);

  void updateJ_linReg(arr& J, const arr& Xraw, const arr& Y, double lambdaReg=1e-3);
};
