/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Optim/optimization.h"
#include "../Kin/kin.h"

//===========================================================================

struct EffectivePoseProblem:ConstrainedProblem {
  rai::Configuration& effKinematics;
  const Graph& KB;
  const Graph& symbolicState_before;
  const Graph& symbolicState_after;
  arr x0;
  int verbose;
  EffectivePoseProblem(rai::Configuration& effKinematics_before,
                       const Graph& KB, const Graph& symbolicState_before, const Graph& symbolicState_after,
                       int verbose);
  void phi(arr& phi, arr& phiJ, arr& H, ObjectiveTypeA& tt, const arr& x);

  double optimize(arr& x);
};

