#pragma once

#include "MathematicalProgram.h"
#include "../Core/graph.h"

enum NLP_SolverID { NLPS_none=-1,
                    NLPS_gradientDescent, NLPS_rprop, NLPS_LBFGS, NLPS_newton,
                    NLPS_augmentedLag, NLPS_squaredPenalty, NLPS_logBarrier, NLPS_singleSquaredPenalty,
                    NLPS_NLopt, NLPS_Ipopt, NLPS_Ceres
                  };

/** User Interface: Meta class to call several different solvers in a unified manner. */
struct NLP_Solver : NonCopyable {
  NLP_SolverID solverID=NLPS_none;
  arr x, dual;
  shared_ptr<MathematicalProgram_Logged> P;

  NLP_Solver& setSolver(NLP_SolverID _solverID){ solverID=_solverID; return *this; }
  NLP_Solver& setProblem(MathematicalProgram& _P){ CHECK(!P, "problem was already set!"); P = make_shared<MathematicalProgram_Logged>(_P); return *this; }
  rai::Graph getOptions(){ return rai::Graph(); }
  void setOptions(const rai::Graph& opt){ NIY }
  void setLogging(bool log_x, bool log_costs, bool log_phi, bool log_J){ NIY }

  arr solve(bool resampleInitialization=true);

  arr getLog_x(){ return P->xLog; }
  arr getLog_costs(){ return P->costLog; }
  arr getLog_phi(){ return P->phiLog; }
  arr getLog_J(){ return P->JLog; }
};
