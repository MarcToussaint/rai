/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "options.h"
#include "newton.h"
#include "lagrangian.h"

struct PrimalDualProblem : ScalarFunction {
  rai::LagrangianProblem L;

  //duality gap parameter (log barrier parameter) of the primal dual equation system
  double mu;

  uint n_eq=0, n_ineq=0;
  arr x_lambda; //last evaluation
  double dualityMeasure=1.;
  bool primalFeasible=false;

  PrimalDualProblem(const arr& x, const shared_ptr<NLP>& P, const rai::OptOptions& opt);

  double primalDual(arr& r, arr& R, const arr& x); ///< CORE METHOD: the unconstrained scalar function F

  void updateMu();
};

//==============================================================================
//
// Solvers
//

struct OptPrimalDual {
  arr& x;
  PrimalDualProblem PD;
  OptNewton newton;
  const rai::OptOptions& opt;
  uint its=0;

  OptPrimalDual(arr& x, arr& dual, const shared_ptr<NLP>& P, const rai::OptOptions& opt);
  ~OptPrimalDual();
  uint run(uint maxIt=1000);
};
