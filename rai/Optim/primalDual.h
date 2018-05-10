/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"
#include "lagrangian.h"

struct PrimalDualProblem : ScalarFunction {
  LagrangianProblem L;

  //duality gap parameter (log barrier parameter) of the primal dual equation system
  double mu;

  arr x_lambda; //last evaluation

  PrimalDualProblem(const arr& x, ConstrainedProblem &P, OptOptions opt=NOOPT, arr& lambdaInit=NoArr);

  double primalDual(arr& r, arr& R, const arr& x); ///< CORE METHOD: the unconstrained scalar function F
};

//==============================================================================
//
// Solvers
//

struct OptPrimalDual{
  arr& x;
  PrimalDualProblem PD;
  OptNewton newton;
  OptOptions opt;
  uint its=0;
  ofstream *fil=NULL;

  OptPrimalDual(arr& x, arr &dual, ConstrainedProblem& P, OptOptions opt=NOOPT);
  ~OptPrimalDual();
  uint run();
};
