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
  OptOptions opt;
  uint its=0;
  bool earlyPhase=false;
  ostream* logFile=nullptr;

  OptConstrained(arr& x, arr& dual, MathematicalProgram& P, OptOptions opt=NOOPT, ostream* _logFile=0);
  ~OptConstrained();
  bool step();
  uint run();
//  void reinit();
};

//==============================================================================
//
// evaluating
//

inline void evaluateMathematicalProgram(const arr& x, MathematicalProgram& P, std::ostream& os) {
  arr phi_x;
  ObjectiveTypeA tt_x;
  P.getFeatureTypes(tt_x);
  P.evaluate(phi_x, NoArr, x);
  double Ef=0., Eh=0., Eg=0.;
  for(uint i=0; i<phi_x.N; i++) {
    if(tt_x(i)==OT_f) Ef += phi_x(i);
    if(tt_x(i)==OT_sos) Ef += rai::sqr(phi_x(i));
    if(tt_x(i)==OT_ineq && phi_x(i)>0.) Eg += phi_x(i);
    if(tt_x(i)==OT_eq) Eh += fabs(phi_x(i));
  }
  os <<"f=" <<Ef <<" sum([g>0]g)="<<Eg <<" sum(|h|)=" <<Eh <<endl;
}

//==============================================================================
//
// PhaseOneProblem
//
// we define a constraint optimization problem that corresponds
// to the phase one problem of another constraint problem
//

struct PhaseOneProblem : MathematicalProgram {
  MathematicalProgram& f_orig;
  ObjectiveTypeA ft;
  uint dim_x, dim_ineq, dim_eq;

  PhaseOneProblem(MathematicalProgram& f_orig):f_orig(f_orig) {}
  void initialize(arr& x);
  virtual void getFeatureTypes(ObjectiveTypeA& featureTypes);
  virtual void evaluate(arr& phi, arr& J, const arr& x);
};

