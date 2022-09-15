/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "constrained.h"

#include <math.h>

//==============================================================================
//
// PhaseOneProblem
//

void PhaseOneProblem::initialize(arr& x) {
  arr phi;
  P->evaluate(phi, NoArr, x);
  dim_eq=dim_ineq=0;
  double gmax=0.;
  for(uint i=0; i<phi.N; i++) {
    if(featureTypes.elem(i)==OT_ineq) {
      dim_ineq++;
      gmax = rai::MAX(gmax, phi.elem(i));
    }
    if(featureTypes.elem(i)==OT_eq) {
      dim_eq++;
    }
  }
  x.append(gmax);
}

void PhaseOneProblem::evaluate(arr& meta_phi, arr& meta_J, const arr& meta_x) {
  CHECK_EQ(meta_x.N, dimension+1, "");
  arr x = meta_x({0, -2});
  double s = meta_x(-1);

  arr phi, J;
  P->evaluate(phi, J, x);

  meta_phi = phi;
  meta_phi.append(-s);

  for(uint i=0; i<phi.N; i++) if(P->featureTypes.elem(i)==OT_ineq) {
      meta_phi(i) = phi(i) - s; //subtract slack!
    }

  if(!!meta_J) {
    meta_J = J;
    meta_J.append(zeros(meta_J.d1));
    meta_J(-1, -1) = -1.;
  }
}

//==============================================================================
//
// Solvers
//

const char* MethodName[]= { "NoMethod", "SquaredPenalty", "AugmentedLagrangian", "LogBarrier", "AnyTimeAugmentedLagrangian", "SquaredPenaltyFixed"};

//==============================================================================

OptConstrained::OptConstrained(arr& _x, arr& _dual, const shared_ptr<NLP>& P, rai::OptOptions _opt, std::ostream* _logFile)
  : L(P, _opt, _dual), newton(_x, L, _opt, _logFile), dual(_dual), opt(_opt), logFile(_logFile) {

  if(opt.boundedNewton){
    arr lo, up;
    P->getBounds(lo, up);
    if(lo.N || up.N) newton.setBounds(lo, up);
  }

  if(opt.constrainedMethod==rai::logBarrier){
    L.useLB=true;
  }

  newton.options.verbose = rai::MAX(opt.verbose-1, 0);

  if(opt.verbose>0) cout <<"***** optConstrained: method=" <<MethodName[opt.constrainedMethod] <<" bounds: " <<(opt.boundedNewton?"yes":"no") <<endl;

  if(logFile) {
    (*logFile) <<"{ optConstraint: " <<its <<", mu: " <<L.mu <<", nu: " <<L.nu <<", L_x: " <<newton.fx <<", errors: ["<<L.get_costs() <<", " <<L.get_sumOfGviolations() <<", " <<L.get_sumOfHviolations() <<"], lambda: " <<L.lambda <<" }," <<endl;
  }

  newton.logFile = logFile;
  L.logFile = logFile;

  //check for no constraints
  if(L.get_dimOfType(OT_ineq)==0 && L.get_dimOfType(OT_ineqB)==0 && L.get_dimOfType(OT_eq)==0) {
    if(opt.verbose>0) cout <<"** optConstr. NO CONSTRAINTS -> run just Newton once" <<endl;
    opt.constrainedMethod=rai::squaredPenaltyFixed;
  }

  //in first iteration, if not squaredPenaltyFixed, increase stop tolerance
  org_stopTol = opt.stopTolerance;
  org_stopGTol = opt.stopGTolerance;
  if(!its && opt.constrainedMethod!=rai::squaredPenaltyFixed){
    newton.options.stopTolerance = 3.*org_stopTol;
    newton.options.stopGTolerance = 3.*org_stopGTol;
  }

  x_beforeNewton = newton.x;

  if(opt.verbose>0) {
    cout <<"** optConstr. it=" <<its
         <<"start evals:" <<newton.evals
         <<" mu=" <<L.mu <<" nu=" <<L.nu <<" muLB=" <<L.muLB;
    if(newton.x.N<5) cout <<" \tlambda=" <<L.lambda;
    cout <<endl;
  }
}

uint OptConstrained::run() {
#if 0
  while(!step());
#else
  while(!ministep());
#endif
  return newton.evals;
}

bool OptConstrained::ministep(){
  //-- first a Newton step
  newton.step();
  if(L.lambda.N) CHECK_EQ(L.lambda.N, L.phi_x.N, "the evaluation (within newton) changed the phi-dimensionality");

  //-- check end of Newton loop
  bool endNewton=false;
  if(newton.its>1000) endNewton=true;
  if(newton.stopCriterion==OptNewton::stopStepFailed) endNewton=true;;
  if(newton.stopCriterion>=OptNewton::stopDeltaConverge) endNewton=true;;

  if(!endNewton) return false;

  //<< Newton loop ended

  if(opt.verbose>0) {
    //END of old Newton loop
    cout <<"** optConstr. it=" <<its
         <<"start evals:" <<newton.evals
         <<" f(x)=" <<L.get_costs()
         <<" \tg_compl=" <<L.get_sumOfGviolations()
         <<" \th_compl=" <<L.get_sumOfHviolations()
         <<" \t|x-x'|=" <<absMax(x_beforeNewton-newton.x);
    if(newton.x.N<5) cout <<" \tx=" <<newton.x;
    cout <<endl;
  }

  //-- STOPPING CRITERIA

  //check for squaredPenaltyFixed method
  if(opt.constrainedMethod==rai::squaredPenaltyFixed) {
    if(opt.verbose>0) cout <<"** optConstr. squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }

  //check for newtonOnce
  if(opt.constrainedMethod==rai::squaredPenaltyFixed) {
    return true;
  }

  //check for squaredPenaltyFixed method
  if(opt.constrainedMethod==rai::squaredPenaltyFixed) {
    if(opt.verbose>0) cout <<"** optConstr. squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }

  //main stopping criteron
  if(its>=1 && absMax(x_beforeNewton-newton.x) < (earlyPhase?5.:1.)*opt.stopTolerance) {
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion Delta<" <<opt.stopTolerance <<endl;
    if(earlyPhase) earlyPhase=false;
    else {
      if(opt.stopGTolerance<0.
          || L.get_sumOfGviolations() + L.get_sumOfHviolations() < opt.stopGTolerance)
        return true;
    }
  }

  //  if(its>=2 && newtonStop==OptNewton::stopTinySteps) {
  //    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion TinySteps<" <<opt.stopTolerance <<endl;
  //    if(earlyPhase) earlyPhase=false;
  //    else {
  //      if(opt.stopGTolerance<0.
  //          || L.get_sumOfGviolations() + L.get_sumOfHviolations() < opt.stopGTolerance)
  //        return true;
  //    }
  //  }

  if(opt.stopEvals>0 && newton.evals>=opt.stopEvals) {
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX EVALS" <<endl;
    return true;
  }
  if(opt.stopIters>0 && newton.its>=opt.stopIters) {
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX ITERS" <<endl;
    return true;
  }
  if(opt.stopOuters>0 && its>=opt.stopOuters) {
    if(opt.verbose>0) cout <<"** optConstr. StoppingCriterion MAX OUTERS" <<endl;
    return true;
  }

  //-- CONTINUE WITH NEXT NEWTON LOOP
  its++;

  double L_x_before = newton.fx;

  //upate Lagrange parameters
  L.autoUpdate(opt, &newton.fx, newton.gx, newton.Hx);

  if(!!dual) dual=L.lambda;

  if(logFile) {
    (*logFile) <<"{ optConstraint: " <<its <<", mu: " <<L.mu <<", nu: " <<L.nu <<", L_x_beforeUpdate: " <<L_x_before <<", L_x_afterUpdate: " <<newton.fx <<", errors: ["<<L.get_costs() <<", " <<L.get_sumOfGviolations() <<", " <<L.get_sumOfHviolations() <<"], lambda: " <<L.lambda <<" }," <<endl;
  }

  if(opt.verbose>0) {
    //START of new Newton loop
    cout <<"** optConstr. it=" <<its
         <<"end   evals:" <<newton.evals
         <<" mu=" <<L.mu <<" nu=" <<L.nu <<" muLB=" <<L.muLB;
    if(newton.x.N<5) cout <<" \tlambda=" <<L.lambda;
    cout <<endl;
  }

  x_beforeNewton = newton.x;

  if(L.lambda.N) CHECK_EQ(L.lambda.N, L.phi_x.N, "");

  if(earlyPhase){
    newton.options.stopTolerance = 10.*org_stopTol;
    newton.options.stopGTolerance = 10.*org_stopGTol;
  }else{
    newton.options.stopTolerance = org_stopTol;
    newton.options.stopGTolerance = org_stopGTol;
  }

  return false;
}


void evaluateNLP(const arr& x, NLP& P, std::ostream& os) {
  arr phi_x;
  P.evaluate(phi_x, NoArr, x);
  double Ef=0., Eh=0., Eg=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(P.featureTypes(i)==OT_f) Ef += phi_x(i);
    if(P.featureTypes(i)==OT_sos) Ef += rai::sqr(phi_x(i));
    if(P.featureTypes(i)==OT_ineq && phi_x(i)>0.) Eg += phi_x(i);
    if(P.featureTypes(i)==OT_eq) Eh += fabs(phi_x(i));
  }
  os <<"f=" <<Ef <<" sum([g>0]g)="<<Eg <<" sum(|h|)=" <<Eh <<std::endl;
}
