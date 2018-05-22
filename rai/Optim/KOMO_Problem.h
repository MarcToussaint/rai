/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>
#include "optimization.h"
#include "Graph_Problem.h"

struct KOMO_Problem {
  virtual uint get_k() = 0;
  virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes)=0;
  virtual void phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x, arr& lambda) = 0;
  
  bool checkStructure(const arr& x);                 ///< check if Jacobians and Hessians have right dimensions (=clique size)
  void report(const arr& phi=NoArr);
};

//-- converters
struct Conv_KOMO_ConstrainedProblem : ConstrainedProblem {
  KOMO_Problem& KOMO;
  uintA variableDimensions, varDimIntegral;
  uintA featureTimes;
  ObjectiveTypeA featureTypes;
  arrA J_KOMO, H_KOMO;
  
  Conv_KOMO_ConstrainedProblem(KOMO_Problem& P);
  
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& tt, const arr& x, arr& lambda);
};

struct KOMO_GraphProblem : GraphProblem {
  KOMO_Problem& KOMO;
  KOMO_GraphProblem(KOMO_Problem& P) : KOMO(P) {}
  virtual void getStructure(uintA& variableDimensions, uintAA& featureVariables, ObjectiveTypeA& featureTypes);
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x, arr& lambda);
};
