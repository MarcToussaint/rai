/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Optim/optimization.h"
#include "../KOMO/komo.h"

//===========================================================================

struct PathProblem {
  rai::Configuration world;
  const Graph& symbolicState;
  uint microSteps;
  int verbose;

  KOMO MP;

  PathProblem(const rai::Configuration& world_initial,
              const rai::Configuration& world_final,
              const Graph& symbolicState,
              uint microSteps,
              int verbose);

  double optimize(arr& x);
};
