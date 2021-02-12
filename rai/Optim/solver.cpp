#include "solver.h"
#include "gradient.h"
#include "newton.h"
#include "opt-nlopt.h"
#include "opt-ipopt.h"
#include "opt-ceres.h"
#include "MathematicalProgram.h"
#include "constrained.h"

template<> const char* rai::Enum<NLP_SolverID>::names []= {
  "gradientDescent", "rprop", "LBFGS", "newton",
  "augmentedLag", "squaredPenalty", "logBarrier", "singleSquaredPenalty",
  "NLopt", "Ipopt", "Ceres", nullptr
};

template<> const char* rai::Enum<NLP_SolverOption>::names []= {
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

arr& NLP_Solver::solve(int resampleInitialization){
  if(resampleInitialization==1 || !x.N){
    x = P->getInitializationSample();
  }else{
    CHECK(x.N, "x is of zero dimensionality - needs initialization");
  }
  if(solverID==NLPS_newton){
    Conv_MathematicalProgram_ScalarProblem P1(*P);
    OptNewton newton(x, P1);
    newton.run();
  }
  else if(solverID==NLPS_gradientDescent){
    Conv_MathematicalProgram_ScalarProblem P1(*P);
    OptGrad(x, P1).run();
  }
  else if(solverID==NLPS_rprop){
    Conv_MathematicalProgram_ScalarProblem P1(*P);
    OptOptions opts;
    Rprop().loop(x, P1, opts.fmin_return, opts.stopTolerance, opts.initStep, opts.stopEvals, opts.verbose);
  }
  else if(solverID==NLPS_augmentedLag){
    OptConstrained opt(x, dual, *P, OptOptions()
                       .set_constrainedMethod(augmentedLag) );
    opt.run();
  }
  else if(solverID==NLPS_squaredPenalty){
    OptConstrained opt(x, dual, *P, OptOptions()
                       .set_constrainedMethod(squaredPenalty) );
    opt.run();
  }
  else if(solverID==NLPS_logBarrier){
    OptConstrained opt(x, dual, *P, OptOptions()
                       .set_constrainedMethod(logBarrier) );
    opt.run();
  }
  else if(solverID==NLPS_NLopt){
    NLoptInterface nlo(*P);
    x = nlo.solve(x);
  }
  else if(solverID==NLPS_Ipopt){
    IpoptInterface nlo(*P);
    x = nlo.solve(x);
  }
  else if(solverID==NLPS_Ceres){
    Conv_MathematicalProgram_TrivialFactoreded P1(*P);
    CeresInterface nlo(P1);
    x = nlo.solve();
  }
  else HALT("solver wrapper not implemented yet for solver ID '" <<rai::Enum<NLP_SolverID>(solverID) <<"'");

  return x;
}
