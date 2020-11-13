#include "solver.h"
#include "newton.h"

template<> const char* rai::Enum<NLP_SolverID>::names []= {
  "gradientDescent", "rprop", "LBFGS", "newton",
  "augmentedLag", "squaredPenalty", "logBarrier", "singleSquaredPenalty",
  "NLopt", "Ipopt", "Ceres", nullptr
};

arr NLP_Solver::solve(bool resampleInitialization){
  if(resampleInitialization){
    x = P->getInitializationSample();
  }else{
    CHECK(x.N, "x is of zero dimensionality - needs initialization");
  }
  if(solverID==NLPS_newton){
    Conv_MathematicalProgram_ScalarProblem P1(*P);
    OptNewton newton(x, P1);
    newton.run();
  } else HALT("solver wrapper not implemented yet for solver ID '" <<rai::Enum<NLP_SolverID>(solverID) <<"'");

  return x;
}
