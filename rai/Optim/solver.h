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
  shared_ptr<MathematicalProgram_Traced> P;

  NLP_Solver& setSolver(NLP_SolverID _solverID){ solverID=_solverID; return *this; }
  NLP_Solver& setProblem(MathematicalProgram& _P){ CHECK(!P, "problem was already set!"); P = make_shared<MathematicalProgram_Traced>(_P); return *this; }
  void setInitialization(const arr& _x){ x=_x; }
  rai::Graph getOptions(){ return rai::Graph(); }
  void setOptions(const rai::Graph& opt){ NIY }
  void setTracing(bool trace_x, bool trace_costs, bool trace_phi, bool trace_J){ P->setTracing(trace_x, trace_costs, trace_phi, trace_J); }

  arr solve(int resampleInitialization=-1); ///< -1: only when not yet set

  arr getTrace_x(){ return P->xTrace; }
  arr getTrace_costs(){ return P->costTrace; }
  arr getTrace_phi(){ return P->phiTrace; }
  arr getTrace_J(){ return P->JTrace; }
};
