/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP_Solver.h"

struct NLP_GraphSolver : rai::NonCopyable {
  rai::NLP_Solver subSolver;
  arr x, dual;
  std::shared_ptr<NLP_Factored> P;
  rai::OptOptions opt;
  std::shared_ptr<SolverReturn> ret;

  NLP_GraphSolver& setProblem(const shared_ptr<NLP_Factored>& _P) { CHECK(!P, "problem was already set!"); P = _P; return *this; }

  shared_ptr<SolverReturn> solve(int resampleInitialization=-1); ///< -1: only when not yet set

  //return values
  uintA infeasibleSubset;
  uintA conflictSet;
  double f_low;

  int chooseNextVariableToAssign(const uintA& Y);

  uintA getVariablesForObjectives(uintA& O);

  void evaluate(const arr& x);

  bool run();

  std::shared_ptr<SolverReturn> solveFull();
  std::shared_ptr<SolverReturn> solveRandom();
  std::shared_ptr<SolverReturn> solveInOrder(uintA order= {});
  void test();
};
