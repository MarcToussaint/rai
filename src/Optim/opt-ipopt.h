/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"

struct IpoptInterface {
  shared_ptr<NLP> P;

  IpoptInterface(const shared_ptr<NLP>& P) : P(P) {}

  shared_ptr<SolverReturn> solve(const arr& x_init=NoArr);
};

