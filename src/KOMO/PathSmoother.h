/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../PathAlgos/ConfigurationProblem.h"

struct ReceedingHorizonPathSmoother {
  ConfigurationProblem& P;
  uint horizon;
  double totalDuration;
  const arr initialPath;

  ReceedingHorizonPathSmoother(ConfigurationProblem& _P, const arr& initialPath, double _duration=20, uint _horizon=10)
    : P(_P),
      horizon(_horizon),
      totalDuration(_duration) {
    if(initialPath.d0 < horizon) {
      horizon = initialPath.d0;
    }
  };

  arr run(int verbose=0);
};
