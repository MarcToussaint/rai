#pragma once

#include "NLP.h"
#include "options.h"
#include "../Core/graph.h"

struct OptConstrained;

enum NLP_SolverID { NLPS_none=-1,
                   NLPS_gradientDescent, NLPS_rprop, NLPS_LBFGS, NLPS_newton,
                   NLPS_augmentedLag, NLPS_squaredPenalty, NLPS_logBarrier, NLPS_singleSquaredPenalty,
                   NLPS_NLopt, NLPS_Ipopt, NLPS_Ceres
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
  arr x, dual;
  uint evals=0;
  double time=0.;
  bool feasible=false;
  double sos=-1., f=-1., ineq=-1., eq=-1.;
  bool done=false;
  void write(ostream& os) const{
    os <<"SolverReturn: { time: " <<time <<", evals: " <<evals;
    os <<", done: " <<done <<", feasible: " <<feasible;
    os <<", sos: " <<sos <<", f: " <<f <<", ineq: " <<ineq <<", eq: " <<eq <<" }";
  }
};
stdOutPipe(SolverReturn)

/** User Interface: Meta class to call several different solvers in a unified manner. */
struct NLP_Solver : NonCopyable {
  NLP_SolverID solverID=NLPS_augmentedLag;
  arr x, dual;
  rai::OptOptions opt;
  std::shared_ptr<SolverReturn> ret;
  std::shared_ptr<OptConstrained> optCon;
  std::shared_ptr<NLP_Traced> P;

  NLP_Solver& setSolver(NLP_SolverID _solverID){ solverID=_solverID; return *this; }
  NLP_Solver& setProblem(const shared_ptr<NLP>& _P){ if(P){ CHECK_EQ(P->P.get(), _P.get(), ""); P->clear(); P->copySignature(*_P); }else{ P = make_shared<NLP_Traced>(_P); }return *this; }
  NLP_Solver& setOptions(const rai::OptOptions& _opt){ opt = _opt; return *this; }
  NLP_Solver& setInitialization(const arr& _x){ x=_x; return *this; }
  NLP_Solver& setWarmstart(const arr& _x, const arr& _dual){ x=_x; dual=_dual; return *this; }
  NLP_Solver& setTracing(bool trace_x, bool trace_costs, bool trace_phi, bool trace_J){ P->setTracing(trace_x, trace_costs, trace_phi, trace_J); return *this; }

  shared_ptr<SolverReturn> solve(int resampleInitialization=-1); ///< -1: only when not yet set
  shared_ptr<SolverReturn> solveStepping(int resampleInitialization=-1); ///< -1: only when not yet set
  bool step();

  arr getTrace_x(){ return P->xTrace; }
  arr getTrace_costs(){ return P->costTrace; }
  arr getTrace_phi(){ return P->phiTrace; }
  arr getTrace_J(){ return P->JTrace; }
  void gnuplot_costs(){
    FILE("z.opt.trace") <<getTrace_costs();
    gnuplot("plot 'z.opt.trace' us 0:1 t 'sos', '' us 0:2 t 'ineq', '' us 0:3 t 'eq'");
  }
};
