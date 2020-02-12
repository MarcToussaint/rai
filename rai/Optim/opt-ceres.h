#pragma once

#include "newOptim.h"

#undef LOG
#undef CHECK
#undef CHECK_EQ
#undef CHECK_LE
#undef CHECK_GE
#include <ceres/problem.h>
#include <ceres/cost_function.h>
#include <ceres/solver.h>
#include <ceres/loss_function.h>

class Conv_CostFunction : public ceres::CostFunction {
  std::shared_ptr<MathematicalProgram> MP;
  uint feature_id;
  uint featureDim;
  intA varIds;
  uintA varDims;
  uint varTotalDim;

public:
  Conv_CostFunction(const ptr<MathematicalProgram>& _MP,
                    uint _feature_id,
                    const uintA& variableDimensions,
                    const uintA& featureDimensions,
                    const intAA& featureVariables);

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const;
};


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

  ceres::Problem cs;

  Conv_MatematicalProgram_CeresProblem(const ptr<MathematicalProgram>& _MP);
};
