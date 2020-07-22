#pragma once

#include "MathematicalProgram.h"

namespace ceres{
  class Problem;
}

//===========================================================================

struct Conv_MatematicalProgram_CeresProblem {
  std::shared_ptr<MathematicalProgram_Structured> MP;

  uintA variableDimensions, featureDimensions, variableDimIntegral, featureDimIntegral;
  intAA featureVariables;
  ObjectiveTypeA featureTypes;

  arr x_full, phi_full;
  arr bounds_lo, bounds_up;
  arrA x;
  arrA phi;
  arrA J;

  ptr<ceres::Problem> ceresProblem;

  Conv_MatematicalProgram_CeresProblem(const ptr<MathematicalProgram_Structured>& _MP);
};
