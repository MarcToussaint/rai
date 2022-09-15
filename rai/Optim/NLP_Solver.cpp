#include "NLP_Solver.h"

#include "optimization.h"
#include "gradient.h"
#include "newton.h"
#include "opt-nlopt.h"
#include "opt-ipopt.h"
#include "opt-ceres.h"
#include "NLP.h"
#include "constrained.h"

template<> const char* rai::Enum<NLP_SolverID>::names []= {
  "gradientDescent", "rprop", "LBFGS", "newton",
  "augmentedLag", "squaredPenalty", "logBarrier", "singleSquaredPenalty",
  "NLopt", "Ipopt", "Ceres", nullptr
};

template<> const char* rai::Enum<NLopt_SolverOption>::names []= {
    "LD_SLSQP",
    "LD_MMA",
    "LN_COBYLA",
    "LD_AUGLAG",
    "LD_AUGLAG_EQ",
    "LN_NELDERMEAD",
    "LD_LBFGS",
    "LD_TNEWTON",
    "LD_TNEWTON_RESTART",
    "LD_TNEWTON_PRECOND",
    "LD_TNEWTON_PRECOND_RESTART", nullptr };

shared_ptr<SolverReturn> NLP_Solver::solve(int resampleInitialization){
  ret = make_shared<SolverReturn>();
  double time = -rai::cpuTime();

  if(resampleInitialization==1 || !x.N){
    x = P->getInitializationSample();
    dual.clear();
  }else{
    CHECK(x.N, "x is of zero dimensionality - needs initialization");
  }
  if(solverID==NLPS_newton){
    Conv_NLP_ScalarProblem P1(P);
    OptNewton newton(x, P1, opt);
    newton.run();
    ret->f = newton.fx;
  }
  else if(solverID==NLPS_gradientDescent){
    Conv_NLP_ScalarProblem P1(P);
    OptGrad(x, P1).run();
  }
  else if(solverID==NLPS_rprop){
    Conv_NLP_ScalarProblem P1(P);
    Rprop().loop(x, P1, opt.stopTolerance, opt.initStep, opt.stopEvals, opt.verbose);
  }
  else if(solverID==NLPS_augmentedLag){
    opt.set_constrainedMethod(rai::augmentedLag);
    optCon = make_shared<OptConstrained>(x, dual, P, opt);
    optCon->run();
  }
  else if(solverID==NLPS_squaredPenalty){
    opt.set_constrainedMethod(rai::squaredPenalty);
    optCon = make_shared<OptConstrained>(x, dual, P, opt);
    optCon->run();
  }
  else if(solverID==NLPS_logBarrier){
    opt.set_constrainedMethod(rai::logBarrier);
    optCon = make_shared<OptConstrained>(x, dual, P, opt);
    optCon->run();
  }
  else if(solverID==NLPS_NLopt){
    NLoptInterface nlo(P);
    x = nlo.solve(x);
  }
  else if(solverID==NLPS_Ipopt){
    IpoptInterface nlo(P);
    x = nlo.solve(x);
  }
  else if(solverID==NLPS_Ceres){
    auto P1 = make_shared<Conv_NLP_TrivialFactoreded>(P);
    CeresInterface nlo(P1);
    x = nlo.solve();
  }
  else HALT("solver wrapper not implemented yet for solver ID '" <<rai::Enum<NLP_SolverID>(solverID) <<"'");

  if(optCon){
    ret->ineq = optCon->L.get_sumOfGviolations();
    ret->eq = optCon->L.get_sumOfHviolations();
    ret->sos = optCon->L.get_cost_sos();
    ret->f = optCon->L.get_cost_f();
    ret->feasible = (ret->ineq<.5) && (ret->eq<.5);
    optCon.reset();
  }

  //checkJacobianCP(*P, x, 1e-4);

  time += rai::cpuTime();
  ret->x=x;
  ret->dual=dual;
  ret->evals=P->evals;
  ret->time = time;
  return ret;
}

shared_ptr<SolverReturn> NLP_Solver::solveStepping(int resampleInitialization){
  if(resampleInitialization==1) x.clear();
  while(!step());
  return ret;
}

bool NLP_Solver::step(){
  CHECK(solverID==NLPS_augmentedLag
        || solverID==NLPS_squaredPenalty
        || solverID==NLPS_logBarrier, "stepping only implemented for these");

  if(!optCon){ //first step -> initialize
    CHECK(!ret, "");
    ret = make_shared<SolverReturn>();

    if(!x.N){
      x = P->getInitializationSample();
      dual.clear();
    }else{
      CHECK(x.N, "x is of zero dimensionality - needs initialization");
    }

    if(solverID==NLPS_augmentedLag){
      opt.set_constrainedMethod(rai::augmentedLag);
    }else if(solverID==NLPS_squaredPenalty){
      opt.set_constrainedMethod(rai::squaredPenalty);
    }else if(solverID==NLPS_logBarrier){
      opt.set_constrainedMethod(rai::logBarrier);
    }
    optCon = make_shared<OptConstrained>(x, dual, P, opt);

  }

  ret->time -= rai::cpuTime();
  ret->done=optCon->ministep();
  ret->time += rai::cpuTime();

  ret->x=x;
  ret->dual=dual;
  ret->evals=P->evals;

  arr feats = optCon->L.get_totalFeatures();
  ret->f = feats(OT_f);
  ret->sos = feats(OT_sos);
  ret->ineq = feats(OT_ineq) + feats(OT_ineqB) + feats(OT_ineqP);
  ret->eq = feats(OT_eq);
  ret->feasible = (ret->ineq<.5) && (ret->eq<.5);

  return ret->done;
}
