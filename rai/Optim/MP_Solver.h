#pragma once

#include "MathematicalProgram.h"
#include "../Core/graph.h"

enum MP_SolverID { MPS_none=-1,
                   MPS_gradientDescent, MPS_rprop, MPS_LBFGS, MPS_newton,
                   MPS_augmentedLag, MPS_squaredPenalty, MPS_logBarrier, MPS_singleSquaredPenalty,
                   MPS_NLopt, MPS_Ipopt, MPS_Ceres
                  };

enum NLopt_SolverOption { _NLopt_LD_SLSQP,
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

struct SolverReturn {
  arr x;
  uint evals=0;
  double time=0.;
  bool feasible=false;
  double sos=-1., f=-1., ineq=-1., eq=-1.;
  void write(ostream& os) const{
    os <<"SolverReturn: time: " <<time <<" feasible: " <<feasible;
    os <<" sos: " <<sos <<" f: " <<f <<" ineq: " <<ineq <<" eq: " <<eq;
  }
};
stdOutPipe(SolverReturn)

/** User Interface: Meta class to call several different solvers in a unified manner. */
struct MP_Solver : NonCopyable {
  MP_SolverID solverID=MPS_augmentedLag;
  arr x, dual;
  shared_ptr<MP_Traced> P;
  int verbose=0;

  MP_Solver& setSolver(MP_SolverID _solverID){ solverID=_solverID; return *this; }
  MP_Solver& setProblem(const shared_ptr<MathematicalProgram>& _P){ CHECK(!P, "problem was already set!"); P = make_shared<MP_Traced>(_P); return *this; }
  MP_Solver& setInitialization(const arr& _x){ x=_x; return *this; }
  MP_Solver& setOptions(const rai::Graph& opt){ NIY; return *this; }
  MP_Solver& setVerbose(int _verbose){ verbose=_verbose; return *this; }
  MP_Solver& setTracing(bool trace_x, bool trace_costs, bool trace_phi, bool trace_J){ P->setTracing(trace_x, trace_costs, trace_phi, trace_J); return *this; }
  rai::Graph getOptions(){ return rai::Graph(); }

  shared_ptr<SolverReturn> solve(int resampleInitialization=-1); ///< -1: only when not yet set

  arr getTrace_x(){ return P->xTrace; }
  arr getTrace_costs(){ return P->costTrace; }
  arr getTrace_phi(){ return P->phiTrace; }
  arr getTrace_J(){ return P->JTrace; }
  void gnuplot_costs(){
    FILE("z.opt.trace") <<getTrace_costs();
    gnuplot("plot 'z.opt.trace' us 0:1 t 'sos', '' us 0:2 t 'ineq', '' us 0:3 t 'eq'");
  }
};
