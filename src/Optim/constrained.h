/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "lagrangian.h"
#include "newton.h"

namespace rai {

extern const char* MethodName[];

//==============================================================================
//
// Solvers
//

struct ConstrainedSolver {
  LagrangianProblem L;
  OptNewton newton;
  arr& dual;
  rai::OptOptions opt;
  int outer_iters=0, numBadSteps=0;

  ConstrainedSolver(arr& x, arr& dual, const shared_ptr<NLP>& P, const rai::OptOptions& opt=DEFAULT_OPTIONS);

  uint run();
  bool ministep();
//  void reinit();
 private:
  arr x_beforeNewton;
  double org_stopTol, org_stopGTol;
};

//==============================================================================
//
// PhaseOneProblem
//
// we define a constraint optimization problem that corresponds
// to the phase one problem of another constraint problem
//

struct PhaseOneProblem : NLP {
  shared_ptr<NLP> P;
  uint dim_ineq, dim_eq;

  PhaseOneProblem(const shared_ptr<NLP>& _P):P(_P) {
    dimension = P->dimension;
    featureTypes = P->featureTypes;
    featureTypes.append(OT_ineq);
  }
  void initialize(arr& x);
  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

} //namespace
