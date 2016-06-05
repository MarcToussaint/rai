#pragma once

#include <Core/array.h>
#include "optimization.h"

struct KOMO_Problem {
  virtual uint get_k() = 0;
  virtual void getStructure(uintA& variableDimensions, uintA& featureTimes, TermTypeA& featureTypes)=0;

  virtual void phi(arr& phi, arrA& J, arrA& H, TermTypeA& tt, const arr& x) = 0;

  bool checkStructure(const arr& x);                 ///< check if Jacobians and Hessians have right dimensions (=clique size)
};

