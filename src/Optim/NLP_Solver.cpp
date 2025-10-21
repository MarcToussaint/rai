/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "NLP.h"
#include "NLP_Solver.h"
#include "utils.h"

#include "m_Gradient.h"
#include "m_Newton.h"
#include "m_LeastSquaresZeroOrder.h"
#include "m_SlackGaussNewton.h"
#include "m_LocalGreedy.h"
#include "m_NelderMead.h"
#include "m_EvoStrategies.h"
#include "m_LBFGS.h"
#include "i_NLopt.h"
#include "i_Ipopt.h"
#include "i_Ceres.h"
#include "constrained.h"

namespace rai {

NLP_Solver::NLP_Solver() { opt = make_shared<OptOptions>(); }

NLP_Solver& NLP_Solver::setProblem(const shared_ptr<NLP>& _P) {
  if(P) {
    CHECK_EQ(P->P.get(), _P.get(), "");
    P->clear();
    P->copySignature(*_P);
  } else {
    P = make_shared<NLP_Traced>(_P);
  }
  return *this;
}

std::shared_ptr<SolverReturn> NLP_Solver::solve(int resampleInitialization, int verbose) {
  ret = make_shared<SolverReturn>();
  double time = -cpuTime();

  if(resampleInitialization==1 || !x.N) {
    x = P->getInitializationSample();
    dual.clear();
  } else {
    CHECK(x.N, "x is of zero dimensionality - needs initialization");
  }

  if(verbose>-100) opt->verbose=verbose;

  std::shared_ptr<NLP> Phere = P;
  if(opt->finiteDifference>0.){
    Phere = make_shared<NLP_FiniteDifference>(P, opt->finiteDifference);
  }

  if(opt->method==M_Newton) {
    OptNewton newton(x, Phere->f_scalar(), opt);
    ret = newton.run();

  } else if(opt->method==M_slackGN) {
    SlackGaussNewton sgn(Phere, x);
    ret = sgn.solve();
    x = ret->x;

  } else if(opt->method==M_GradientDescent) {
    OptGrad grad(x, Phere->f_scalar(), opt);
    grad.run();
    ret->evals = grad.evals;
    ret->f = grad.f_x;
    ret->feasible = true;

  } else if(opt->method==M_Rprop) {
    Rprop grad;
    grad.loop(x, Phere->f_scalar(), opt->stopTolerance, opt->stepInit, opt->stopEvals, opt->verbose);
    ret->evals = grad.evals;
    ret->f = grad.fx;
    ret->feasible = true;

  } else if(opt->method==M_LSZO) {
    ret = LeastSquaredZeroOrder(Phere, x). solve();
    x = ret->x;

  } else if(opt->method==M_NelderMead) {
    ret = NelderMead(Phere->f_scalar(), x). solve();
    x = ret->x;

  } else if(opt->method==M_CMA) {
    ret = rai::CMAES(Phere->f_scalar(), x). solve();
    x = ret->x;

  } else if(opt->method==M_greedy) {
    ret = LocalGreedy(Phere->f_scalar(), x). solve();
    x = ret->x;

  } else if(opt->method==M_AugmentedLag) {
    optCon = make_shared<ConstrainedSolver>(x, dual, Phere, opt);
    ret = optCon->run();

  } else if(opt->method==M_SquaredPenalty) {
    optCon = make_shared<ConstrainedSolver>(x, dual, Phere, opt);
    ret = optCon->run();

  } else if(opt->method==M_LogBarrier) {
    optCon = make_shared<ConstrainedSolver>(x, dual, Phere, opt);
    ret = optCon->run();

  } else if(opt->method==M_slackGN_logBarrier) {
    SlackGaussNewton sgn(Phere, x);
    sgn.opt.interiorPadding = 1e-2;
    ret = sgn.solve();
    x = ret->x;
    ret->feasible = ret->ineq==0. && ret->eq<.1;
    if(ret->feasible){
      opt->set_method(M_LogBarrier);
      optCon = make_shared<ConstrainedSolver>(x, dual, Phere, opt);
      ret = optCon->run();
    }

  } else if(opt->method==M_NLopt) {
    NLoptInterface nlo(Phere);
    x = nlo.solve(x);

  } else if(opt->method==M_Ipopt) {
    IpoptInterface ipo(Phere);
    ret = ipo.solve(x);
    x = ret->x;

  } else if(opt->method==M_Ceres) {
    auto P1 = make_shared<Conv_NLP2TrivialFactoredNLP>(Phere);
    CeresInterface ceres(P1);
    x = ceres.solve();

  } else if(opt->method==M_LBFGS) {
    ret = LBFGS(Phere->f_scalar(), x, opt).solve();
    x = ret->x;

  } else HALT("solver wrapper not implemented yet for solver ID '" <<Enum<OptMethod>(opt->method) <<"'");

  //checkJacobianCP(*P, x, 1e-4);

  time += cpuTime();
  ret->x = x;
  ret->dual = dual;
  ret->evals = P->evals;
  ret->time = time;
  ret->done = true;
  return ret;
}

shared_ptr<SolverReturn> NLP_Solver::solveStepping(int resampleInitialization, int verbose) {
  if(resampleInitialization==1) x.clear();
  if(verbose>-100) opt->verbose=verbose;
  while(!step());
  return ret;
}

bool NLP_Solver::step() {
  CHECK(opt->method==M_AugmentedLag
        || opt->method==M_SquaredPenalty
        || opt->method==M_LogBarrier, "stepping only implemented for these");

  if(!optCon) { //first step -> initialize
    CHECK(!ret, "");
    ret = make_shared<SolverReturn>();

    if(!x.N) {
      x = P->getInitializationSample();
      dual.clear();
    } else {
      CHECK(x.N, "x is of zero dimensionality - needs initialization");
    }

    if(opt->method==M_AugmentedLag) {
      opt->set_method(M_AugmentedLag);
    } else if(opt->method==M_SquaredPenalty) {
      opt->set_method(M_SquaredPenalty);
    } else if(opt->method==M_LogBarrier) {
      opt->set_method(M_LogBarrier);
    }
    optCon = make_shared<ConstrainedSolver>(x, dual, P, opt);
  }

  ret->time -= cpuTime();
  ret->done = optCon->ministep();
  ret->time += cpuTime();

  ret->x = x;
  ret->dual = dual;
  ret->evals = P->evals;

  arr err = P->summarizeErrors(optCon->L.phi_x);
  ret->f = err(OT_f);
  ret->sos = err(OT_sos);
  ret->ineq = err(OT_ineq);
  ret->eq = err(OT_eq);
  ret->feasible = (ret->ineq<.1) && (ret->eq<.1);

  return ret->done;
}

Graph NLP_Solver::reportLagrangeGradients(const StringA& featureNames) {
  CHECK(optCon, "");
  return optCon->L.reportGradients(featureNames);
}

}; //namespace
