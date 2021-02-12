#pragma once

#include "MathematicalProgram.h"
#include "../Core/graph.h"

enum NLP_SolverID { NLPS_none=-1,
                    NLPS_gradientDescent, NLPS_rprop, NLPS_LBFGS, NLPS_newton,
                    NLPS_augmentedLag, NLPS_squaredPenalty, NLPS_logBarrier, NLPS_singleSquaredPenalty,
                    NLPS_NLopt, NLPS_Ipopt, NLPS_Ceres
                  };

enum NLP_SolverOption { _NLopt_LD_SLSQP,
                        _NLopt_LD_MMA,
                        _NLopt_LN_COBYLA,
                        _NLopt_LD_AUGLAG,
                        _NLopt_LD_AUGLAG_EQ,
                        _NLopt_LN_NELDERMEAD,
                        _NLopt_LD_LBFGS,
                        _NLopt_LD_TNEWTON,
                        _NLopt_LD_TNEWTON_RESTART,
                        _NLopt_LD_TNEWTON_PRECOND,
                        _NLopt_LD_TNEWTON_PRECOND_RESTART,
                      };

/** User Interface: Meta class to call several different solvers in a unified manner. */
struct NLP_Solver : NonCopyable {
  NLP_SolverID solverID=NLPS_none;
  arr x, dual;
  shared_ptr<MathematicalProgram_Traced> P;

  NLP_Solver& setSolver(NLP_SolverID _solverID){ solverID=_solverID; return *this; }
  NLP_Solver& setProblem(MathematicalProgram& _P){ CHECK(!P, "problem was already set!"); P = make_shared<MathematicalProgram_Traced>(_P); return *this; }
  NLP_Solver& setInitialization(const arr& _x){ x=_x; return *this; }
  NLP_Solver& setOptions(const rai::Graph& opt){ NIY; return *this; }
  NLP_Solver& setTracing(bool trace_x, bool trace_costs, bool trace_phi, bool trace_J){ P->setTracing(trace_x, trace_costs, trace_phi, trace_J); return *this; }
  rai::Graph getOptions(){ return rai::Graph(); }

  arr& solve(int resampleInitialization=-1); ///< -1: only when not yet set

  arr getTrace_x(){ return P->xTrace; }
  arr getTrace_costs(){ return P->costTrace; }
  arr getTrace_phi(){ return P->phiTrace; }
  arr getTrace_J(){ return P->JTrace; }
  void gnuplot_costs(){
    FILE("z.opt.trace") <<getTrace_costs();
    gnuplot("plot 'z.opt.trace' us 0:1 t 'sos', '' us 0:2 t 'ineq', '' us 0:3 t 'eq'");
  }
};
