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
  ofstream *fil=NULL;
  
  OptConstrained(arr& x, arr &dual, ConstrainedProblem& P, OptOptions opt=NOOPT);
  ~OptConstrained();
  bool step();
  uint run();
//  void reinit();
};

//TODO: remove:
inline uint optConstrained(arr& x, arr &dual, ConstrainedProblem& P, OptOptions opt=NOOPT) {
  return OptConstrained(x, dual, P, opt).run();
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
  
  PhaseOneProblem(ConstrainedProblem &f_orig):f_orig(f_orig) {}
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda);
};

