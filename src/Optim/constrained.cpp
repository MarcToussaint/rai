/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "constrained.h"

#include <iomanip>
#include <math.h>

namespace rai {

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
      gmax = MAX(gmax, phi.elem(i));
    }
    if(featureTypes.elem(i)==OT_eq) {
      dim_eq++;
    }
  }
  x.append(gmax);
}

void PhaseOneProblem::evaluate(arr& meta_phi, arr& meta_J, const arr& meta_x) {
  CHECK_EQ(meta_x.N, dimension+1, "");
  arr x = meta_x({0, -2+1});
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

ConstrainedSolver::ConstrainedSolver(arr& _x, arr& _dual, const shared_ptr<NLP>& P, shared_ptr<OptOptions> _opt)
  : L(P, _opt), newton(_x, L.f_scalar(), _opt), dual(_dual), opt(_opt) {

  if(!!_dual && _dual.N) L.lambda = _dual;

  if(opt->boundedNewton) {
    if(P->bounds.N) newton.setBounds(P->bounds);
  }

  if(opt->method==M_LogBarrier) {
    L.useLB=true;
  }

  newton.opt->verbose = MAX(opt->verbose-1, 0);

  if(opt->verbose>0){
    cout <<"====nlp===="
        <<" problem-dim: " <<P->dimension <<'/' <<P->featureTypes.N
         <<" method:" <<Enum<OptMethod>(opt->method)
      <<" bounded: " <<(opt->boundedNewton?"yes":"no") <<endl;
  }

  //check for no constraints
  if(P->get_numOfType(OT_ineq)==0 && P->get_numOfType(OT_ineqB)==0 && P->get_numOfType(OT_eq)==0) {
    if(opt->verbose>0) cout <<"==nlp== NO CONSTRAINTS -> run just Newton once" <<endl;
    opt->method=M_singleSquaredPenalty;
  }

  //in first iteration, if not squaredPenaltyFixed, increase stop tolerance
  org_stopTol = opt->stopTolerance;
  org_stopGTol = opt->stopGTolerance;
  if(!outer_iters && opt->method!=M_singleSquaredPenalty) {
    newton.opt->stopTolerance = 3.*org_stopTol;
    newton.opt->stopGTolerance = 3.*org_stopGTol;
  }

  x_beforeNewton = newton.x;

  if(opt->verbose>0) {
    cout <<"==nlp== it:" <<outer_iters
         <<" evals:" <<newton.evals
         <<" mu:" <<L.mu <<" nu:" <<L.mu <<" muLB:" <<L.muLB;
    if(newton.x.N<5) cout <<" \tlambda:" <<L.lambda;
    cout <<endl;
  }
}

std::shared_ptr<SolverReturn> ConstrainedSolver::run() {
  while(!ministep());
  std::shared_ptr<SolverReturn> ret = make_shared<SolverReturn>();
  arr err = L.P->summarizeErrors(L.phi_x);
  ret->ineq = err(OT_ineq);
  ret->eq = err(OT_eq);
  ret->sos = err(OT_sos);
  ret->f = err(OT_f);
  ret->feasible = (ret->ineq<.1) && (ret->eq<.1);
  return ret;
}

bool ConstrainedSolver::ministep() {
  //-- first a Newton step
  newton.step();
  if(L.lambda.N) CHECK_EQ(L.lambda.N, L.phi_x.N, "the evaluation (within newton) changed the phi-dimensionality");

  //-- if Newton loop not finished, return false to keep calling ministeps
  if(newton.stopCriterion==OptNewton::stopNone) return false;

  //-- Newton loop finished

  arr err = L.P->summarizeErrors(L.phi_x);
  double step = absMax(x_beforeNewton-newton.x);
  if(newton.stopCriterion>OptNewton::stopDeltaConverge) {
    numBadSteps++;
  } else {
    numBadSteps=0;
  }

  if(opt->verbose>0) {
    //END of old Newton loop
    cout <<"==nlp== it:" <<std::setw(4) <<outer_iters
         <<"  evals:" <<std::setw(4) <<newton.evals
         <<"  A(x):" <<std::setw(11) <<newton.fx
         <<"  f:" <<std::setw(11) <<err(OT_sos)+err(OT_f)
         <<"  g:" <<std::setw(11) <<err(OT_ineq)
         <<"  h:" <<std::setw(11) <<err(OT_eq)
         <<"  |x-x'|:" <<std::setw(11) <<step
         <<" \tstop:" <<Enum<OptNewton::StopCriterion>(newton.stopCriterion);
    if(numBadSteps) cout <<" (bad:" <<numBadSteps <<")";
    if(newton.x.N<5) cout <<" \tx:" <<newton.x;
    cout <<endl;
    if(opt->verbose>4) L.P->report(cout, opt->verbose, STRING("evals:" <<newton.evals));
  }

  //-- STOPPING CRITERIA

  //check for squaredPenaltyFixed method
  if(opt->method==M_singleSquaredPenalty) {
    if(opt->verbose>0) cout <<"==nlp== squaredPenaltyFixed stops after one outer iteration" <<endl;
    return true;
  }

  //main stopping criteron: convergence
  if(outer_iters>=1 && step < opt->stopTolerance) {
    if(opt->verbose>0) cout <<"==nlp== StoppingCriterion Delta<" <<opt->stopTolerance <<endl;
    if(opt->stopGTolerance<0. || err(OT_ineq)+err(OT_eq)<opt->stopGTolerance) {
      return true; //good: small step in last loop and err small
    } else {
      if(opt->verbose>0) cout <<"               -- but err too large " <<err(OT_ineq)+err(OT_eq) <<'>' <<opt->stopGTolerance <<endl;
      if(numBadSteps>6) {
        cout <<"               -- but numBadSteps > 6" <<endl;
        return true;
      }
    }
  }

  if(opt->stopEvals>0 && newton.evals>=opt->stopEvals) {
    if(opt->verbose>0) cout <<"==nlp== StoppingCriterion MAX EVALS" <<endl;
    return true;
  }
  if(opt->stopInners>0 && newton.inner_iters>=opt->stopInners) {
    if(opt->verbose>0) cout <<"==nlp== inner aborted" <<endl;
    newton.inner_iters=0;
//    return true;
  }
  if(opt->stopOuters>0 && outer_iters>=opt->stopOuters) {
    if(opt->verbose>0) cout <<"==nlp== StoppingCriterion MAX OUTERS" <<endl;
    return true;
  }

  //-- CONTINUE WITH NEXT ITERATION
  outer_iters++;

  //upate Lagrange parameters
  // double L_x_before = newton.fx;
  L.autoUpdate(&newton.fx, newton.gx, newton.Hx);
  if(!!dual) dual=L.lambda;

  if(opt->verbose>0) {
    //START of new Newton loop
    cout <<"==nlp== it:" <<std::setw(4) <<outer_iters
         <<"  evals:" <<std::setw(4) <<newton.evals
         <<"  A(x):" <<std::setw(11) <<newton.fx
         <<"  mu:" <<L.mu;
    if(L.useLB) cout <<" muLB:" <<std::setw(11) <<L.muLB;
    if(newton.x.N<5) cout <<" \tlambda:" <<L.lambda;
    cout <<endl;
  }

#if 0 //this would trigger a new evaluation; only to confirm that autoUpdate updates also fx correctly
  //add noise!?
  // rndGauss(newton.x, 1e-3, true);
  newton.reinit(newton.x);
  if(opt->verbose>0) {
    //START of new Newton loop
    cout <<"==nlp== it:" <<std::setw(4) <<its
         <<"  evals:" <<std::setw(4) <<newton.evals
         <<"  A(x):" <<std::setw(11) <<newton.fx
         <<"  mu:" <<L.mu;
    if(L.useLB) cout <<" muLB:" <<std::setw(11) <<L.muLB;
    if(newton.x.N<5) cout <<" \tlambda:" <<L.lambda;
    cout <<endl;
  }
#endif

  //prepare next Newton loop
  x_beforeNewton = newton.x;
  clip(newton.alpha, 1e-2, 1.);

  if(L.lambda.N) CHECK_EQ(L.lambda.N, L.phi_x.N, "");

  newton.opt->stopTolerance = org_stopTol;
  newton.opt->stopGTolerance = org_stopGTol;

  return false;
}

} //namespace
