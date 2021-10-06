#include "MP_Solver.h"

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
  double time = -rai::cpuTime();

  if(resampleInitialization==1 || !x.N){
    x = P->getInitializationSample();
  }else{
    CHECK(x.N, "x is of zero dimensionality - needs initialization");
  }
  if(solverID==MPS_newton){
    Conv_MathematicalProgram_ScalarProblem P1(*P);
    OptNewton newton(x, P1, OptOptions()
                     .set_verbose(verbose));
    newton.run();
  }
  else if(solverID==MPS_gradientDescent){
    Conv_MathematicalProgram_ScalarProblem P1(*P);
    OptGrad(x, P1).run();
  }
  else if(solverID==MPS_rprop){
    Conv_MathematicalProgram_ScalarProblem P1(*P);
    OptOptions opts;
    Rprop().loop(x, P1, opts.fmin_return, opts.stopTolerance, opts.initStep, opts.stopEvals, opts.verbose);
  }
  else if(solverID==MPS_augmentedLag){
    OptConstrained opt(x, dual, *P, OptOptions()
                       .set_constrainedMethod(augmentedLag)
                       .set_verbose(verbose) );
    opt.run();
    ret->ineq = opt.L.get_sumOfGviolations();
    ret->eq = opt.L.get_sumOfHviolations();
    ret->sos = opt.L.get_cost_sos();
    ret->f = opt.L.get_cost_f();
  }
  else if(solverID==MPS_squaredPenalty){
    OptConstrained opt(x, dual, *P, OptOptions()
                       .set_constrainedMethod(squaredPenalty)
                       .set_verbose(verbose) );
    opt.run();
  }
  else if(solverID==MPS_logBarrier){
    OptConstrained opt(x, dual, *P, OptOptions()
                       .set_constrainedMethod(logBarrier)
                       .set_verbose(verbose) );
    opt.run();
  }
  else if(solverID==MPS_NLopt){
    NLoptInterface nlo(*P);
    x = nlo.solve(x);
  }
  else if(solverID==MPS_Ipopt){
    IpoptInterface nlo(*P);
    x = nlo.solve(x);
  }
  else if(solverID==MPS_Ceres){
    Conv_MathematicalProgram_TrivialFactoreded P1(*P);
    CeresInterface nlo(P1);
    x = nlo.solve();
  }
  else HALT("solver wrapper not implemented yet for solver ID '" <<rai::Enum<MP_SolverID>(solverID) <<"'");

  time += rai::cpuTime();
  ret->x=x;
  ret->evals=P->evals;
  ret->time = time;
  return ret;
}
