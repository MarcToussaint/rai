/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Optim/optimization.h"

struct PR2EndPoseProblem : ConstrainedProblem {
  struct sPR2EndPoseProblem& s;

  PR2EndPoseProblem();

  arr getInitialization();
  void setState(const arr&);
  void report();
};
