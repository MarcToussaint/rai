/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "optimization.h"
#include "Graph_Problem.h"

struct KOMO_Problem {
  virtual uint get_k() = 0;
  virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, ObjectiveTypeA& featureTypes)=0;
  virtual void phi(arr& phi, arrA& J, arrA& H, uintA& featureTimes, ObjectiveTypeA& tt, const arr& x) = 0;

  bool checkStructure(const arr& x);                 ///< check if Jacobians and Hessians have right dimensions (=clique size)
  void report(const arr& phi=NoArr);
};

//-- converters
struct Conv_KOMOProblem_MathematicalProgram : MathematicalProgram {
  KOMO_Problem& KOMO;
  uintA variableDimensions, varDimIntegral;
  uintA featureTimes;
  ObjectiveTypeA featureTypes;
  arrA J_KOMO, H_KOMO;

  Conv_KOMOProblem_MathematicalProgram(KOMO_Problem& P);

  virtual uint getDimension() { return varDimIntegral.elem(-1); }
  virtual void getFeatureTypes(ObjectiveTypeA& ft) { ft = featureTypes; }
  virtual void evaluate(arr& phi, arr& J, const arr& z);
};

struct KOMO_GraphProblem : GraphProblem {
  KOMO_Problem& KOMO;
  KOMO_GraphProblem(KOMO_Problem& P) : KOMO(P) {}
  virtual void getStructure(uintA& variableDimensions, uintAA& featureVariables, ObjectiveTypeA& featureTypes);
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x);
};
