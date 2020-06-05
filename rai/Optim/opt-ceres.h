#pragma once

#include "MathematicalProgram.h"

namespace ceres{
  class Problem;
}

//===========================================================================

struct Conv_MatematicalProgram_CeresProblem {
  std::shared_ptr<MathematicalProgram> MP;

  uintA variableDimensions, featureDimensions, variableDimIntegral;
  intAA featureVariables;
  ObjectiveTypeA featureTypes;

  arr x_base;
  arr bounds_lo, bounds_up;
  arrA x;
  arrA phi;
  arrA J;

  ptr<ceres::Problem> ceresProblem;

  Conv_MatematicalProgram_CeresProblem(const ptr<MathematicalProgram>& _MP);
};
