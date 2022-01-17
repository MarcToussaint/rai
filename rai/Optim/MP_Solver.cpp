#include "MP_Solver.h"

#include "optimization.h"
#include "gradient.h"
#include "newton.h"
#include "opt-nlopt.h"
#include "opt-ipopt.h"
#include "opt-ceres.h"
#include "MathematicalProgram.h"
#include "constrained.h"

template<> const char* rai::Enum<MP_SolverID>::names []= {
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

shared_ptr<SolverReturn> MP_Solver::solve(int resampleInitialization){
  auto ret = make_shared<SolverReturn>();
  shared_ptr<OptConstrained> optCon;
  double time = -rai::cpuTime();

  if(resampleInitialization==1 || !x.N){
    x = P->getInitializationSample();
  }else{
    CHECK(x.N, "x is of zero dimensionality - needs initialization");
  }
  if(solverID==MPS_newton){
    Conv_MathematicalProgram_ScalarProblem P1(P);
    OptNewton newton(x, P1, opt);
    newton.run();
    ret->f = newton.fx;
  }
  else if(solverID==MPS_gradientDescent){
    Conv_MathematicalProgram_ScalarProblem P1(P);
    OptGrad(x, P1).run();
  }
  else if(solverID==MPS_rprop){
    Conv_MathematicalProgram_ScalarProblem P1(P);
    Rprop().loop(x, P1, opt.stopTolerance, opt.initStep, opt.stopEvals, opt.verbose);
  }
  else if(solverID==MPS_augmentedLag){
    opt.set_constrainedMethod(rai::augmentedLag);
    optCon = make_shared<OptConstrained>(x, dual, P, opt);
    optCon->run();
  }
  else if(solverID==MPS_squaredPenalty){
    opt.set_constrainedMethod(rai::squaredPenalty);
    optCon = make_shared<OptConstrained>(x, dual, P, opt);
    optCon->run();
  }
  else if(solverID==MPS_logBarrier){
    opt.set_constrainedMethod(rai::logBarrier);
    optCon = make_shared<OptConstrained>(x, dual, P, opt);
    optCon->run();
  }
  else if(solverID==MPS_NLopt){
    NLoptInterface nlo(P);
    x = nlo.solve(x);
  }
  else if(solverID==MPS_Ipopt){
    IpoptInterface nlo(P);
    x = nlo.solve(x);
  }
  else if(solverID==MPS_Ceres){
    auto P1 = make_shared<Conv_MathematicalProgram_TrivialFactoreded>(P);
    CeresInterface nlo(P1);
    x = nlo.solve();
  }
  else HALT("solver wrapper not implemented yet for solver ID '" <<rai::Enum<MP_SolverID>(solverID) <<"'");

  if(optCon){
      ret->ineq = optCon->L.get_sumOfGviolations();
      ret->eq = optCon->L.get_sumOfHviolations();
      ret->sos = optCon->L.get_cost_sos();
      ret->f = optCon->L.get_cost_f();
  }

  //checkJacobianCP(*P, x, 1e-4);

  time += rai::cpuTime();
  ret->x=x;
  ret->dual=dual;
  ret->evals=P->evals;
  ret->time = time;
  return ret;
}
