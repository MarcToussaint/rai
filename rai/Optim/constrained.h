/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "lagrangian.h"
#include "newton.h"

extern const char* MethodName[];

//==============================================================================
//
// Solvers
//

struct OptConstrained {
  LagrangianProblem L;
  OptNewton newton;
  arr& dual;
  rai::OptOptions opt;
  int its=0;
  bool earlyPhase=false;
  ostream* logFile=nullptr;

  OptConstrained(arr& x, arr& dual, const shared_ptr<NLP>& P, rai::OptOptions opt=NOOPT, ostream* _logFile=0);

  uint run();
  bool ministep();
//  void reinit();
private:
  arr x_beforeNewton;
  double org_stopTol, org_stopGTol;
};

//==============================================================================
//
// evaluating
//

void evaluateNLP(const arr& x, NLP& P, std::ostream& os);

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
    dimension = P->getDimension();
    featureTypes = P->featureTypes;
    featureTypes.append(OT_ineq);
  }
  void initialize(arr& x);
  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

