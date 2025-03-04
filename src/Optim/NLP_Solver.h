/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"
#include "options.h"
#include "../Core/graph.h"

namespace rai {

struct ConstrainedSolver;

/** User Interface: Meta class to call several different solvers in a unified manner. */
struct NLP_Solver : NonCopyable {
  arr x, dual; //owner of decision variables, which are passed by reference to lower level solvers
  rai::OptOptions opt; //owner of options, which are passed by reference to lower level solvers
  std::shared_ptr<SolverReturn> ret;
  std::shared_ptr<ConstrainedSolver> optCon;
  std::shared_ptr<NLP_Traced> P;

  NLP_Solver();
  NLP_Solver(const shared_ptr<NLP>& _P, int verbose=-100) { setProblem(_P); if(verbose>-100) opt.verbose=verbose; }

  NLP_Solver& setSolver(OptMethod _method) { opt.method=_method; return *this; }
  NLP_Solver& setProblem(const shared_ptr<NLP>& _P);
  NLP_Solver& setOptions(const rai::OptOptions& _opt) { opt = _opt; return *this; }
  NLP_Solver& setInitialization(const arr& _x) { x=_x; return *this; }
  NLP_Solver& setWarmstart(const arr& _x, const arr& _dual) { x=_x; dual=_dual; return *this; }
  NLP_Solver& setTracing(bool trace_x, bool trace_costs, bool trace_phi, bool trace_J) { P->setTracing(trace_x, trace_costs, trace_phi, trace_J); return *this; }
  NLP_Solver& clear() { P.reset(); optCon.reset(); ret.reset(); x.clear(); dual.clear(); return *this; }

  std::shared_ptr<SolverReturn> solve(int resampleInitialization=-1, int verbose=-100); ///< -1: only when not already yet set
  std::shared_ptr<SolverReturn> solveStepping(int resampleInitialization=-1, int verbose=-100); ///< -1: only when not already yet set
  bool step();

  std::shared_ptr<NLP> getProblem(){ return P->P; }
  arr getTrace_x() { return P->xTrace; }
  arr getTrace_costs() { return P->costTrace; }
  arr getTrace_phi() { return P->phiTrace; }
  arr getTrace_J() { return P->JTrace; }
  arr getTrace_lambda();
  arr getTrace_evals();
  rai::Graph reportLagrangeGradients(const StringA& featureNames);
  void gnuplot_costs() {
    FILE("z.opt.trace") <<getTrace_costs();
    gnuplot("plot 'z.opt.trace' us 0:1 t 'f', '' us 0:2 t 'sos', '' us 0:3 t 'ineq', '' us 0:4 t 'eq'");
  }
};

} //namespace
