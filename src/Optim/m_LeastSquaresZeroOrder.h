#pragma once

#include "NLP.h"
#include "../Core/util.h"

struct LeastSquaredZeroOrder{
  shared_ptr<NLP> P;
  bool hasLinTerms=false;

  //-- parameters
  str method="rank1";
  double alpha = .5;
  RAI_PARAM("LSZO/", double, alpha_min, .001)
  RAI_PARAM("LSZO/", double, damping, 1e-2)

  //-- state and data
  arr x;               ///< point where P was last evaluated
  arr J;
  arr phi_x; //, J_x, H_x; ///< features at x
  double phi2_x=-1.;
  arr data_X, data_Phi;

  uint steps=0, tinySteps=0, rejectedSteps=0;

  LeastSquaredZeroOrder(shared_ptr<NLP> P, const arr& x_init={});

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

  void updateJ_linReg(arr& J, const arr& Xraw, const arr& Y);
};
