/*  ------------------------------------------------------------------
    Copyright (c) 2011-2024 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "NLP.h"

namespace ceres {
class Problem;
}

//===========================================================================

struct CeresInterface {
  shared_ptr<NLP_Factored> P;
//  NLP& P;

  CeresInterface(const shared_ptr<NLP_Factored>& P) : P(P) {}

  arr solve();
};

//===========================================================================

struct Conv_NLP_CeresProblem {
  shared_ptr<NLP_Factored> P;

  arr x_full, phi_full; //the full linear memory for all decision variables and all features

  shared_ptr<ceres::Problem> ceresProblem;

  Conv_NLP_CeresProblem(const shared_ptr<NLP_Factored>& _P);
};
