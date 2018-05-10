/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "constrained.h"

//==============================================================================
//
// PhaseOneProblem
//

void PhaseOneProblem::phi(arr& meta_phi, arr& meta_J, arr& meta_H, ObjectiveTypeA& tt, const arr& x, arr& lambda){
  NIY;
  arr g, Jg;
//  f_orig(NoArr, NoArr, g, (&meta_Jg?Jg:NoArr), x.sub(0,-2)); //the underlying problem only receives a x.N-1 dimensional x

  // meta_g.resize(g.N+1);
  // meta_g(0) = x.last();                                       //cost
  // for(uint i=0;i<g.N;i++) meta_g(i) = g(i)-x.last();  //slack constraints
  // meta_g.last() = -x.last();                                  //last constraint

  // if(&meta_Jg){
  //   meta_Jg.resize(meta_g.N, x.N);  meta_Jg.setZero();
  //   meta_Jg(0,x.N-1) = 1.; //cost
  //   for(uint i=0;i<g.N;i++) for(uint j=0;j<x.N-1;j++) meta_Jg(i,j) = Jg(i,j);
  //   for(uint i=0;i<g.N;i++) meta_Jg(i,x.N-1) = -1.;
  //   meta_Jg(g.N, x.N-1) = -1.;
  // }
}

//==============================================================================
//
// Solvers
//

const char* MethodName[]={ "NoMethod", "SquaredPenalty", "AugmentedLagrangian", "LogBarrier", "AnyTimeAugmentedLagrangian", "SquaredPenaltyFixed"};

//==============================================================================

OptConstrained::OptConstrained(arr& x, arr &dual, ConstrainedProblem& P, OptOptions opt)
  : L(P, opt, dual), newton(x, L, opt), dual(dual), opt(opt){

  if(opt.verbose>0) cout <<"***** optConstrained: method=" <<MethodName[opt.constrainedMethod] <<endl;
}

bool OptConstrained::step(){
  if(fil) (*fil) <<"constr " <<its <<' ' <<newton.evals <<' ' <<L.get_costs() <<' ' <<L.get_sumOfGviolations() <<' ' <<L.get_sumOfHviolations() <<endl;
  newton.fil = fil;

  if(opt.verbose>0){
    cout <<"** optConstr. it=" <<its
         <<(earlyPhase?'e':'l')
         <<" mu=" <<L.mu <<" nu=" <<L.nu <<" muLB=" <<L.muLB;
    if(newton.x.N<5) cout <<" \tlambda=" <<L.lambda;
    cout <<endl;
  }

  arr x_old = newton.x;

  //check for no constraints
  bool newtonOnce=false;
  if(L.get_dimOfType(OT_ineq)==0 && L.get_dimOfType(OT_eq)==0){
    if(opt.verbose>0) cout <<"** optConstr. NO CONSTRAINTS -> run Newton once and stop" <<endl;
    newtonOnce=true;
  }

  //run newton on the Lagrangian problem
  if(newtonOnce || opt.constrainedMethod==squaredPenaltyFixed){
    newton.run();
  }else{
    double stopTol = newton.o.stopTolerance;
    newton.o.stopTolerance *= (earlyPhase?10.:2.);
    if(opt.constrainedMethod==anyTimeAula)  newton.run(20);
    else                                    newton.run();
    newton.o.stopTolerance = stopTol;
  }

  if(opt.verbose>0){
    cout <<"** optConstr. it=" <<its
         <<(earlyPhase?'e':'l')
         <<' ' <<newton.evals
         <<" f(x)=" <<L.get_costs()
         <<" \tg_compl=" <<L.get_sumOfGviolations()
         <<" \th_compl=" <<L.get_sumOfHviolations()
         <<" \t|x-x'|=" <<absMax(x_old-newton.x);
    if(newton.x.N<5) cout <<" \tx=" <<newton.x;
    cout <<endl;
  }

  //check for squaredPenaltyFixed method
  if(opt.constrainedMethod==squaredPenaltyFixed){
    if(opt.verbose>0) cout <<"** optConstr. squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }

  //check for newtonOnce
  if(newtonOnce){
    return true;
  }

  //check for squaredPenaltyFixed method
  if(opt.constrainedMethod==squaredPenaltyFixed){
    if(opt.verbose>0) cout <<"** optConstr. squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }

  //stopping criteron
  if(its>=2 && absMax(x_old-newton.x) < (earlyPhase?5.:1.)*opt.stopTolerance){
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
    if(earlyPhase) earlyPhase=false;
    else{
      if(opt.stopGTolerance<0.
         || L.get_sumOfGviolations() + L.get_sumOfHviolations() < opt.stopGTolerance)
        return true;
     }
  }
  if(newton.evals>=opt.stopEvals){
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX EVALS" <<endl;
    return true;
  }
  if(newton.it>=opt.stopIters){
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX ITERS" <<endl;
    return true;
  }
  if(its>=opt.stopOuters){
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX OUTERS" <<endl;
    return true;
  }

  //upate Lagrange parameters
  switch(opt.constrainedMethod){
//  case squaredPenalty: UCP.mu *= opt.aulaMuInc;  break;
  case squaredPenalty: L.aulaUpdate(false, -1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
  case augmentedLag:   L.aulaUpdate(false, 1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
  case anyTimeAula:    L.aulaUpdate(true,  1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
  case logBarrier:     L.muLB /= 2.;  break;
  case squaredPenaltyFixed: HALT("you should not be here"); break;
  case noMethod: HALT("need to set method before");  break;
  }

  if(&dual) dual=L.lambda;

  its++;

  return false;
}

uint OptConstrained::run(){
//  earlyPhase=true;
  while(!step());
  return newton.evals;
}

OptConstrained::~OptConstrained(){
}

