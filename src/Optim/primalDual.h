/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "options.h"
#include "m_Newton.h"
#include "lagrangian.h"

namespace rai {

struct PrimalDualProblem : NLP_Scalar {
  rai::LagrangianProblem L;

  //duality gap parameter (log barrier parameter) of the primal dual equation system
  double mu;

  uint n_eq=0, n_ineq=0;
  arr x_lambda; //last evaluation
  double dualityMeasure=1.;
  bool primalFeasible=false;

  PrimalDualProblem(const arr& x, const shared_ptr<NLP>& P, std::shared_ptr<OptOptions> opt);

  double f(arr& r, arr& R, const arr& x); ///< CORE METHOD: the unconstrained scalar function F

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
  std::shared_ptr<OptOptions> opt;
  uint its=0;

  OptPrimalDual(arr& x, arr& dual, const shared_ptr<NLP>& P, std::shared_ptr<OptOptions> _opt);
  ~OptPrimalDual();
  uint run(uint maxIt=1000);
};

} //namespace
