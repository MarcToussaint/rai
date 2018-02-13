/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include "lagrangian.h"
#include "newton.h"

extern const char* MethodName[];

//==============================================================================
//
// Solvers
//

struct OptConstrained{
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
inline uint optConstrained(arr& x, arr &dual, ConstrainedProblem& P, OptOptions opt=NOOPT){
  return OptConstrained(x, dual, P, opt).run();
}


//==============================================================================
//
// evaluating
//

inline void evaluateConstrainedProblem(const arr& x, ConstrainedProblem& P, std::ostream& os){
  arr phi_x;
  ObjectiveTypeA tt_x;
  P.phi(phi_x, NoArr, NoArr, tt_x, x, NoArr);
  double Ef=0., Eh=0., Eg=0.;
  for(uint i=0;i<phi_x.N;i++){
    if(tt_x(i)==OT_f) Ef += phi_x(i);
    if(tt_x(i)==OT_sumOfSqr) Ef += mlr::sqr(phi_x(i));
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

struct PhaseOneProblem : ConstrainedProblem{
  ConstrainedProblem &f_orig;

  PhaseOneProblem(ConstrainedProblem &f_orig):f_orig(f_orig) {}
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda);
};

