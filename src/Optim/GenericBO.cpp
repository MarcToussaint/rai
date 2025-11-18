#include "GenericBO.h"

namespace rai {

void GenericBO::update_best(const arr& X, const arr& Phi, const arr& F){
  //-- update best
  uint best_i = argmin(F);
  double f_y = F(best_i);

  if(opt->verbose>1) cout <<' ' <<evals;

  //-- update x (reject?)
  if(f_y>=best_f){
    rejectedSteps++;
    if(opt->verbose>1) cout <<" f:" <<f_y <<" [reject]";
  }else{
    rejectedSteps=0;
    if(best_x.N && length(best_x-X[best_i])<1e-4) tinySteps++; else tinySteps=0;
    best_x = X[best_i];
    best_phi = Phi[best_i];
    best_f = F(best_i);
    if(opt->verbose>1) cout <<" f:" <<best_f <<" [accept]";
  }
}

void GenericBO::evaluateSamples(arr& Phi, arr& F, const arr& X){
  Phi.resize(X.d0, P->featureTypes.N);
  for(uint i=0;i<X.d0;i++){
    P->evaluate(Phi[i].noconst(), NoArr, X[i]);
    evals++;
  }

  F.resize(Phi.d0);
  for(uint i=0;i<F.N;i++) F(i) = ::sum(P->summarizeErrors(Phi[i]));

  update_best(X, Phi, F);
}

bool GenericBO::step(){

  if(opt->verbose>1) cout <<"--" <<method <<"--";

  arr X = generateSamples();

  arr Phi, F;
  evaluateSamples(Phi, F, X);

  update(X, Phi, F);

  if(opt->verbose>1) cout <<endl;

  if((int)evals>opt->stopEvals) return true;
  if(rejectedSteps>10*P->dimension) return true;
  if(tinySteps>10) return true;
  return false;
}

shared_ptr<SolverReturn> GenericBO::solve(){
  while(!step()){}
  if(opt->verbose>0) cout <<"--" <<method <<" done-- " <<evals <<" f: " <<best_f <<std::endl;
  shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
  ret->x = best_x;
#if 1
  arr err = P->summarizeErrors(best_phi);
  ret->f = err(OT_f);
  ret->sos = err(OT_sos);
  ret->eq = err(OT_eq);
  ret->ineq = err(OT_ineq);
#else
  ret->f = best_f;
#endif
  ret->feasible=true;
  return ret;
}

} //namespace
