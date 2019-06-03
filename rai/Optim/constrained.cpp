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

void PhaseOneProblem::initialize(arr& x){
  arr phi;
  ObjectiveTypeA ot;
  f_orig.phi(phi, NoArr, NoArr, ot, x, NoArr);
  dim_x=x.N;
  dim_eq=dim_ineq=0;
  double gmax=0.;
  for(uint i=0;i<phi.N;i++){
    if(ot.elem(i)==OT_ineq){
      dim_ineq++;
      gmax = rai::MAX(gmax, phi.elem(i));
    }
    if(ot.elem(i)==OT_eq){
      dim_eq++;
    }
  }
  x.append(gmax);
}

void PhaseOneProblem::phi(arr& meta_phi, arr& meta_J, arr& meta_H, ObjectiveTypeA& meta_ot, const arr& meta_x, arr& lambda) {
  CHECK_EQ(meta_x.N, dim_x+1, "");
  arr x = meta_x({0,-2});
  double s = meta_x(-1);

  arr phi, J;
  ObjectiveTypeA ot;
  f_orig.phi(phi, J, NoArr, ot, x, NoArr);

  meta_phi.resize(1+dim_ineq+dim_eq);
  meta_ot.resize(1+dim_ineq+dim_eq);

  uint m=0;
  for(uint i=0;i<phi.N;i++) if(ot.elem(i)==OT_ineq){
    meta_phi(m) = phi(i) - s; //subtract slack!
    meta_ot(m) = OT_ineq;
    m++;
  }
  for(uint i=0;i<phi.N;i++) if(ot.elem(i)==OT_eq){
    meta_phi(m) = phi(i);
    meta_ot(m) = OT_eq;
    m++;
  }
  CHECK_EQ(m, dim_ineq+dim_eq, "");
  meta_phi(m) = s;

   if(!!meta_J){
     meta_J.resize(meta_phi.N, meta_x.N).setZero();
     m=0;
     for(uint i=0;i<phi.N;i++) if(ot.elem(i)==OT_ineq){
       meta_J[m] = J[i];
       m++;
     }
     for(uint i=0;i<phi.N;i++) if(ot.elem(i)==OT_eq){
       meta_J[m] = J[i];
       m++;
     }
     meta_J(-1,-1) = 1.;
     CHECK_EQ(m, dim_ineq+dim_eq, "");
   }
}

//==============================================================================
//
// Solvers
//

const char* MethodName[]= { "NoMethod", "SquaredPenalty", "AugmentedLagrangian", "LogBarrier", "AnyTimeAugmentedLagrangian", "SquaredPenaltyFixed"};

//==============================================================================

OptConstrained::OptConstrained(arr& _x, arr &_dual, ConstrainedProblem& P, int verbose, OptOptions _opt, std::ostream* _logFile)
  : L(P, _opt, _dual), newton(_x, L, _opt, _logFile), dual(_dual), opt(_opt), logFile(_logFile) {

  if(verbose>=0) opt.verbose=verbose;
  newton.o.verbose = rai::MAX(opt.verbose-1,0);
  
  if(opt.verbose>0) cout <<"***** optConstrained: method=" <<MethodName[opt.constrainedMethod] <<endl;

  if(logFile){
    (*logFile) <<"{ optConstraint: " <<its <<", mu: " <<L.mu <<", nu: " <<L.nu <<", L_x: " <<newton.fx <<", errors: ["<<L.get_costs() <<", " <<L.get_sumOfGviolations() <<", " <<L.get_sumOfHviolations() <<"], lambda: " <<L.lambda <<" }," <<endl;
  }
}

bool OptConstrained::step() {
  newton.logFile = logFile;
  L.logFile = logFile;
  
  if(opt.verbose>0) {
    cout <<"** optConstr. it=" <<its
         <<(earlyPhase?'e':'l')
         <<" mu=" <<L.mu <<" nu=" <<L.nu <<" muLB=" <<L.muLB;
    if(newton.x.N<5) cout <<" \tlambda=" <<L.lambda;
    cout <<endl;
  }
  
  arr x_old = newton.x;
  
  //check for no constraints
  bool newtonOnce=false;
  if(L.get_dimOfType(OT_ineq)==0 && L.get_dimOfType(OT_eq)==0) {
    if(opt.verbose>0) cout <<"** optConstr. NO CONSTRAINTS -> run Newton once and stop" <<endl;
    newtonOnce=true;
  }
  
  //run newton on the Lagrangian problem
  if(newtonOnce || opt.constrainedMethod==squaredPenaltyFixed) {
    newton.run();
  } else {
    double stopTol = newton.o.stopTolerance;
    if(earlyPhase) newton.o.stopTolerance *= 10.;
    if(opt.constrainedMethod==anyTimeAula)  newton.run(20);
    else                                    newton.run();
    newton.o.stopTolerance = stopTol;
  }
  
  if(opt.verbose>0) {
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
  if(opt.constrainedMethod==squaredPenaltyFixed) {
    if(opt.verbose>0) cout <<"** optConstr. squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }
  
  //check for newtonOnce
  if(newtonOnce) {
    return true;
  }
  
  //check for squaredPenaltyFixed method
  if(opt.constrainedMethod==squaredPenaltyFixed) {
    if(opt.verbose>0) cout <<"** optConstr. squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }
  
  //stopping criteron
  if(its>=2 && absMax(x_old-newton.x) < (earlyPhase?5.:1.)*opt.stopTolerance) {
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
    if(earlyPhase) earlyPhase=false;
    else {
      if(opt.stopGTolerance<0.
          || L.get_sumOfGviolations() + L.get_sumOfHviolations() < opt.stopGTolerance)
        return true;
    }
  }
  if(newton.evals>=opt.stopEvals) {
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX EVALS" <<endl;
    return true;
  }
  if(newton.its>=opt.stopIters) {
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX ITERS" <<endl;
    return true;
  }
  if(its>=opt.stopOuters) {
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX OUTERS" <<endl;
    return true;
  }
  
  double L_x_before = newton.fx;

  //upate Lagrange parameters
  switch(opt.constrainedMethod) {
//  case squaredPenalty: UCP.mu *= opt.aulaMuInc;  break;
    case squaredPenalty: L.aulaUpdate(false, -1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
    case augmentedLag:   L.aulaUpdate(false, 1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
    case anyTimeAula:    L.aulaUpdate(true,  1., opt.aulaMuInc, &newton.fx, newton.gx, newton.Hx);  break;
    case logBarrier:     L.muLB /= 2.;  break;
    case squaredPenaltyFixed: HALT("you should not be here"); break;
    case noMethod: HALT("need to set method before");  break;
  }
  
  if(!!dual) dual=L.lambda;

  its++;

  if(logFile){
    (*logFile) <<"{ optConstraint: " <<its <<", mu: " <<L.mu <<", nu: " <<L.nu <<", L_x_beforeUpdate: " <<L_x_before <<", L_x_afterUpdate: " <<newton.fx <<", errors: ["<<L.get_costs() <<", " <<L.get_sumOfGviolations() <<", " <<L.get_sumOfHviolations() <<"], lambda: " <<L.lambda <<" }," <<endl;
  }

  return false;
}

uint OptConstrained::run() {
//  earlyPhase=true;
  while(!step());
  newton.beta *= 1e-3;
  step();
  return newton.evals;
}

OptConstrained::~OptConstrained() {
}

