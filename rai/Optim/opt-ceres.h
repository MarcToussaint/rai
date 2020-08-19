#pragma once

#include "MathematicalProgram.h"

namespace ceres{
  class Problem;
}

//===========================================================================

struct CeresInterface {
  MathematicalProgram_Factored& P;
//  MathematicalProgram& P;

  CeresInterface(MathematicalProgram_Factored& P) : P(P){}

  arr solve();
};

//===========================================================================

struct Conv_MathematicalProgram_CeresProblem {
  MathematicalProgram_Factored& MP;

  arr x_full, phi_full; //the full linear memory for all decision variables and all features

  ptr<ceres::Problem> ceresProblem;

  Conv_MathematicalProgram_CeresProblem(MathematicalProgram_Factored& _MP);
};
