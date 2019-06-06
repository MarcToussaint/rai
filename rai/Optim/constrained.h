/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

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
  arr &dual;
  OptOptions opt;
  uint its=0;
  bool earlyPhase=false;
  ostream *logFile=NULL;
  
  OptConstrained(arr& x, arr &dual, ConstrainedProblem& P, int verbose=-1, OptOptions opt=NOOPT, ostream* _logFile=0);
  ~OptConstrained();
  bool step();
  uint run();
//  void reinit();
};

//TODO: remove:
inline uint optConstrained(arr& x, arr &dual, ConstrainedProblem& P, int verbose=-1, OptOptions opt=NOOPT) {
  return OptConstrained(x, dual, P, verbose, opt).run();
}

//==============================================================================
//
// evaluating
//

inline void evaluateConstrainedProblem(const arr& x, ConstrainedProblem& P, std::ostream& os) {
  arr phi_x;
  ObjectiveTypeA tt_x;
  P.phi(phi_x, NoArr, NoArr, tt_x, x, NoArr);
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

struct PhaseOneProblem : ConstrainedProblem {
  ConstrainedProblem &f_orig;
  uint dim_x, dim_ineq, dim_eq;
  
  PhaseOneProblem(ConstrainedProblem &f_orig):f_orig(f_orig) {}
  void initialize(arr& x);
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& meta_ot, const arr& x, arr& lambda);
};

