#pragma once

#include <Core/array.h>
#include "optimization.h"
#include "Graph_Problem.h"

struct KOMO_Problem {
  virtual uint get_k() = 0;
  virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, TermTypeA& featureTypes)=0;
  virtual void phi(arr& phi, arrA& J, arrA& H, TermTypeA& tt, const arr& x) = 0;

  bool checkStructure(const arr& x);                 ///< check if Jacobians and Hessians have right dimensions (=clique size)
};


//-- converters
struct Conv_KOMO_ConstrainedProblem : ConstrainedProblem{
  KOMO_Problem& KOMO;
  uintA variableDimensions, varDimIntegral;
  uintA featureTimes;
  TermTypeA featureTypes;
  arrA J_KOMO, H_KOMO;

  Conv_KOMO_ConstrainedProblem(KOMO_Problem& P);


  void f(arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x);
};

struct KOMO_GraphProblem : GraphProblem{
  KOMO_Problem& KOMO;
  KOMO_GraphProblem(KOMO_Problem& P) : KOMO(P){}
  virtual void getStructure(uintA& variableDimensions, uintAA& featureVariables, TermTypeA& featureTypes);
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x);
};
