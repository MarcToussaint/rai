/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Optim/optimization.h>

struct PR2EndPoseProblem : ConstrainedProblem {
  struct sPR2EndPoseProblem& s;
  
  PR2EndPoseProblem();
  
  arr getInitialization();
  void setState(const arr&);
  void report();
};
