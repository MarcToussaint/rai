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

#include "optimization.h"
#include "newton.h"

extern const char* MethodName[];

//==============================================================================
//
// LagrangianProblem
//
// we define an unconstraint optimization problem from a constrained one
// that can include lagrange terms, penalties, log barriers, and augmented lagrangian terms
//

struct LagrangianProblem : ScalarFunction{ //TODO: rename: UnconstrainedLagrangianProblem
  ConstrainedProblem& P;

  //-- parameters of the unconstrained (Lagrangian) scalar function
  double muLB;       ///< log barrier weight
  double mu;         ///< squared penalty weight for inequalities g
  double nu;         ///< squared penalty weight for equalities h
  arr lambda;        ///< lagrange multipliers for inequalities g and equalities h

  //-- buffers to avoid recomputing gradients
  arr x;          ///< point where P was last evaluated
  arr phi_x, J_x, H_x; ///< everything else at x
  ObjectiveTypeA tt_x; ///< everything else at x

  LagrangianProblem(ConstrainedProblem &P, OptOptions opt=NOOPT, arr& lambdaInit=NoArr);

  double lagrangian(arr& dL, arr& HL, const arr& x); ///< the unconstrained scalar function F

  double get_costs();            ///< info on the terms from last call
  double get_sumOfGviolations(); ///< info on the terms from last call
  double get_sumOfHviolations(); ///< info on the terms from last call
  uint get_dimOfType(const ObjectiveType& tt); ///< info on the terms from last call

  void aulaUpdate(bool anyTimeVariant, double lambdaStepsize=1., double muInc=1., double *L_x=NULL, arr &dL_x=NoArr, arr &HL_x=NoArr);
};

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
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x);
};


//==============================================================================
//
// Solvers
//

struct OptConstrained{
  LagrangianProblem UCP;
  OptNewton newton;
  arr &dual;
  OptOptions opt;
  uint its;
  bool earlyPhase;
  ofstream fil;

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
  P.phi(phi_x, NoArr, NoArr, tt_x, x);
  double Ef=0., Eh=0., Eg=0.;
  for(uint i=0;i<phi_x.N;i++){
    if(tt_x(i)==OT_f) Ef += phi_x(i);
    if(tt_x(i)==OT_sumOfSqr) Ef += mlr::sqr(phi_x(i));
    if(tt_x(i)==OT_ineq && phi_x(i)>0.) Eg += phi_x(i);
    if(tt_x(i)==OT_eq) Eh += fabs(phi_x(i));
  }
  os <<"f=" <<Ef <<" sum([g>0]g)="<<Eg <<" sum(|h|)=" <<Eh <<endl;
}


