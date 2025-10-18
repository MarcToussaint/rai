/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "NLP_Solver.h"

#include "gradient.h"
#include "newton.h"
#include "m_LeastSquaresZeroOrder.h"
#include "m_LocalGreedy.h"
#include "m_NelderMead.h"
#include "m_EvoStrategies.h"
#include "m_LBFGS.h"
#include "opt-nlopt.h"
#include "opt-ipopt.h"
#include "opt-ceres.h"
#include "NLP.h"
#include "constrained.h"
#include "utils.h"
#include "SlackGaussNewton.h"

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

  if(opt->method==M_Newton) {
    Conv_NLP2ScalarProblem P1(P);
    OptNewton newton(x, P1, opt);
    ret = newton.run();

  } else if(opt->method==M_Newton_FD) {
    auto P1 = make_shared<NLP_FiniteDifference>(P);
    auto P2 = make_shared<Conv_NLP2ScalarProblem>(P1);
    OptNewton newton(x, *P2, opt);
    ret = newton.run();

  } else if(opt->method==M_slackGN) {
    SlackGaussNewton sgn(P, x);
    ret = sgn.solve();
    x = ret->x;

  } else if(opt->method==M_GradientDescent) {
    Conv_NLP2ScalarProblem P1(P);
    OptGrad grad(x, P1, opt);
    grad.run();
    ret->evals = grad.evals;
    ret->f = grad.fx;
    ret->feasible = true;

  } else if(opt->method==M_Rprop) {
    Conv_NLP2ScalarProblem P1(P);
    Rprop grad;
    grad.loop(x, P1, opt->stopTolerance, opt->stepInit, opt->stopEvals, opt->verbose);
    ret->evals = grad.evals;
    ret->f = grad.fx;
    ret->feasible = true;

  } else if(opt->method==M_LSZO) {
    ret = LeastSquaredZeroOrder(P, x). solve();

  } else if(opt->method==M_NelderMead) {
    ret = NelderMead(P, x). solve();

  } else if(opt->method==M_CMA) {
    ret = rai::CMAES(P, x). solve();

  } else if(opt->method==M_greedy) {
    ret = LocalGreedy(P, x). solve();

  } else if(opt->method==M_AugmentedLag) {
    opt->set_method(M_AugmentedLag);
    optCon = make_shared<ConstrainedSolver>(x, dual, P, opt);
    ret = optCon->run();

  } else if(opt->method==M_squaredPenalty) {
    opt->set_method(M_squaredPenalty);
    optCon = make_shared<ConstrainedSolver>(x, dual, P, opt);
    ret = optCon->run();

  } else if(opt->method==M_LogBarrier) {
    opt->set_method(M_LogBarrier);
    optCon = make_shared<ConstrainedSolver>(x, dual, P, opt);
    ret = optCon->run();

  } else if(opt->method==M_slackGN_logBarrier) {
    SlackGaussNewton sgn(P, x);
    sgn.opt.interiorPadding = 1e-2;
    ret = sgn.solve();
    x = ret->x;
    ret->feasible = ret->ineq==0. && ret->eq<.1;
    if(ret->feasible){
      opt->set_method(M_LogBarrier);
      optCon = make_shared<ConstrainedSolver>(x, dual, P, opt);
      ret = optCon->run();
    }
  } else if(opt->method==M_NLopt) {
    NLoptInterface nlo(P);
    x = nlo.solve(x);

  } else if(opt->method==M_Ipopt) {
    IpoptInterface ipo(P);
    ret = ipo.solve(x);
    x = ret->x;

  } else if(opt->method==M_Ceres) {
    auto P1 = make_shared<Conv_NLP2TrivialFactoredNLP>(P);
    CeresInterface ceres(P1);
    x = ceres.solve();

  } else if(opt->method==M_LBFGS) {
    ret = LBFGS(P, x, opt).solve();

  } else if(opt->method==M_LBFGS_FD) {
    // ret = LBFGS(P, x, opt).solve();
    auto P1 = make_shared<NLP_FiniteDifference>(P);
    ret = LBFGS(P1, x, opt).solve();

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
        || opt->method==M_squaredPenalty
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
    } else if(opt->method==M_squaredPenalty) {
      opt->set_method(M_squaredPenalty);
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
